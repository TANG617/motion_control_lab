#include "admittance.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace app = motion_control_lab::
    hierarchical_kinematics_step;
namespace mcc = motion_control::core;
namespace mcs = motion_control::sim;

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

std::array<mcs::DragReference, 2> inactiveDrags() {
  std::array<mcs::DragReference, 2> result;
  result[0].id = "left";
  result[1].id = "right";
  return result;
}

mcc::CartesianTrajectorySample referenceSample() {
  mcc::CartesianTrajectorySample sample;
  sample.time_from_start = 0.25;
  sample.frames.resize(2U);
  for (std::size_t index = 0; index < sample.frames.size(); ++index) {
    auto &frame = sample.frames[index];
    frame.reference_frame_name = "base_link";
    frame.frame_name =
        index == 0U ? "left_arm_ee_link" : "right_arm_ee_link";
    frame.pose = mcc::Pose::Identity();
    frame.pose.translation() =
        Eigen::Vector3d{0.4, index == 0U ? 0.2 : -0.2, 0.8};
    frame.twist << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6;
    frame.acceleration << -0.7, 0.8, -0.9, 1.0, -1.1, 1.2;
  }
  return sample;
}

void testControlPointPvaRoundTrip() {
  auto source = referenceSample().frames.at(0);
  source.pose.linear() =
      Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitY()).toRotationMatrix();
  mcc::Pose ee_to_tcp = mcc::Pose::Identity();
  ee_to_tcp.translation() = Eigen::Vector3d{0.04, -0.03, 0.1};
  ee_to_tcp.linear() =
      Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();

  const auto tcp =
      app::shiftCartesianControlPoint(source, ee_to_tcp, "left_tcp");
  const Eigen::Vector3d r =
      tcp.pose.translation() - source.pose.translation();
  const Eigen::Vector3d expected_velocity =
      source.twist.head<3>() + source.twist.tail<3>().cross(r);
  const Eigen::Vector3d expected_acceleration =
      source.acceleration.head<3>() +
      source.acceleration.tail<3>().cross(r) +
      source.twist.tail<3>().cross(source.twist.tail<3>().cross(r));
  require(tcp.twist.head<3>().isApprox(expected_velocity, 1.0e-12),
          "EE-to-TCP velocity omitted a rigid-point term");
  require(tcp.acceleration.head<3>().isApprox(expected_acceleration, 1.0e-12),
          "EE-to-TCP acceleration omitted angular cross terms");
  require(tcp.twist.tail<3>().isApprox(source.twist.tail<3>(), 1.0e-12),
          "control-point shift changed angular velocity");
  require(tcp.acceleration.tail<3>().isApprox(source.acceleration.tail<3>(),
                                              1.0e-12),
          "control-point shift changed angular acceleration");

  const auto round_trip = app::shiftCartesianControlPoint(
      tcp, ee_to_tcp.inverse(), source.frame_name);
  require(round_trip.pose.matrix().isApprox(source.pose.matrix(), 1.0e-12),
          "control-point pose round-trip failed");
  require(round_trip.twist.isApprox(source.twist, 1.0e-12),
          "control-point twist round-trip failed");
  require(round_trip.acceleration.isApprox(source.acceleration, 1.0e-12),
          "control-point acceleration round-trip failed");
}

void testAdmittanceReferencePvaAndDrag() {
  app::DualArmAdmittance admittance;
  admittance.configure(app::AdmittanceOptions{});
  const auto nominal = referenceSample();
  mcc::Pose tcp_offset = mcc::Pose::Identity();
  tcp_offset.translation().z() = 0.1;
  auto drags = inactiveDrags();

  auto output = admittance.step(
      nominal, nominal.frames.at(0).pose, nominal.frames.at(1).pose,
      tcp_offset, tcp_offset, drags, 0.001);
  require(output.command_sample.frames.at(0).pose.matrix().isApprox(
              nominal.frames.at(0).pose.matrix(), 1.0e-12),
          "zero wrench changed nominal left pose");
  require(output.command_sample.frames.at(0).twist.isApprox(
              nominal.frames.at(0).twist, 1.0e-12),
          "zero wrench dropped planner twist");
  require(output.command_sample.frames.at(0).acceleration.isApprox(
              nominal.frames.at(0).acceleration, 1.0e-12),
          "zero wrench dropped planner acceleration");

  drags[0].active = true;
  drags[0].target_pose.position_m =
      {{output.left.actual_tcp_pose.translation().x() + 0.1,
        output.left.actual_tcp_pose.translation().y(),
        output.left.actual_tcp_pose.translation().z()}};
  for (int step = 0; step < 500; ++step) {
    output = admittance.step(
        nominal, nominal.frames.at(0).pose, nominal.frames.at(1).pose,
        tcp_offset, tcp_offset, drags, 0.001);
  }
  require(output.left.command_link.pose.translation().x() >
              nominal.frames.at(0).pose.translation().x(),
          "left command did not move along the applied force");
  require(output.left.filtered_wrench.force.norm() <= 20.0 + 1.0e-12,
          "filtered wrench exceeded the configured force limit");
  require(output.right.command_link.pose.matrix().isApprox(
              nominal.frames.at(1).pose.matrix(), 1.0e-12),
          "left drag leaked into right admittance state");

  drags[0].active = false;
  for (int step = 0; step < 3000; ++step) {
    output = admittance.step(
        nominal, nominal.frames.at(0).pose, nominal.frames.at(1).pose,
        tcp_offset, tcp_offset, drags, 0.001);
  }
  require(output.left.raw_wrench.force.norm() == 0.0,
          "released drag did not clear raw wrench");
  require((output.left.command_link.pose.translation() -
           nominal.frames.at(0).pose.translation())
              .norm() < 1.0e-3,
          "left admittance did not return within 1 mm in 3 seconds");
}

} // namespace

int main() {
  try {
    testControlPointPvaRoundTrip();
    testAdmittanceReferencePvaAndDrag();
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
