#include <motion_control_sim/mujoco_kinematic_simulation.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace mcs = motion_control::sim;

namespace {

const std::vector<std::string> kControlledJoints{
    "head_yaw_joint",    "head_pitch_joint", "torso_yaw_joint",
    "torso_pitch_joint", "knee_pitch_joint", "ankle_pitch_joint",
    "left_arm_joint1",   "left_arm_joint2",  "left_arm_joint3",
    "left_arm_joint4",   "left_arm_joint5",  "left_arm_joint6",
    "left_arm_joint7",   "right_arm_joint1", "right_arm_joint2",
    "right_arm_joint3",  "right_arm_joint4", "right_arm_joint5",
    "right_arm_joint6",  "right_arm_joint7"};

const mcs::SiteState &site(const mcs::KinematicSnapshot &snapshot,
                           const std::string &name) {
  const auto found =
      std::find_if(snapshot.sites.begin(), snapshot.sites.end(),
                   [&](const auto &value) { return value.name == name; });
  if (found == snapshot.sites.end()) {
    throw std::runtime_error("snapshot is missing site " + name);
  }
  return *found;
}

Eigen::Isometry3d sitePose(const mcs::SiteState &site_state) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d{site_state.pose.position_m[0],
                                         site_state.pose.position_m[1],
                                         site_state.pose.position_m[2]};
  result.linear() = Eigen::Quaterniond{site_state.pose.orientation_wxyz[0],
                                       site_state.pose.orientation_wxyz[1],
                                       site_state.pose.orientation_wxyz[2],
                                       site_state.pose.orientation_wxyz[3]}
                        .normalized()
                        .toRotationMatrix();
  return result;
}

void compare(const Eigen::Isometry3d &expected, const Eigen::Isometry3d &actual,
             const std::string &label, int sample_index) {
  const double translation_error =
      (expected.translation() - actual.translation()).norm();
  const Eigen::AngleAxisd rotation_error(expected.linear().transpose() *
                                         actual.linear());
  const double orientation_error = std::abs(rotation_error.angle());
  if (translation_error > 1.0e-6 || orientation_error > 1.0e-6) {
    throw std::runtime_error(
        label + " FK mismatch at sample " + std::to_string(sample_index) +
        ": translation=" + std::to_string(translation_error) +
        " orientation=" + std::to_string(orientation_error));
  }
}

} // namespace

int main(int argc, char **argv) {
  try {
    if (argc != 3) {
      throw std::runtime_error("expected <urdf> <mjcf>");
    }

    pinocchio::Model model;
    pinocchio::urdf::buildModel(argv[1], model);
    pinocchio::Data data(model);
    Eigen::VectorXd q = pinocchio::neutral(model);

    std::vector<pinocchio::JointIndex> joint_ids;
    joint_ids.reserve(kControlledJoints.size());
    for (const auto &name : kControlledJoints) {
      const auto id = model.getJointId(name);
      if (id == 0 || model.joints[id].nq() != 1) {
        throw std::runtime_error("Pinocchio is missing scalar joint " + name);
      }
      joint_ids.push_back(id);
    }
    const auto left_frame = model.getFrameId("left_arm_ee_link");
    const auto right_frame = model.getFrameId("right_arm_ee_link");
    if (left_frame >= model.nframes || right_frame >= model.nframes) {
      throw std::runtime_error("Pinocchio is missing an arm EE frame");
    }

    mcs::MujocoKinematicSimulation simulation;
    simulation.load(mcs::ModelDescription{argv[2],
                                          kControlledJoints,
                                          {"left_tcp_site", "right_tcp_site"},
                                          "floating_base"});
    if (simulation.jointNames().size() != 20U) {
      throw std::runtime_error(
          "MuJoCo did not map exactly 20 controlled joints");
    }

    std::mt19937 generator(0x524f424fU);
    for (int sample_index = 0; sample_index <= 100; ++sample_index) {
      mcs::JointState joints;
      joints.names = kControlledJoints;
      joints.positions.resize(kControlledJoints.size());
      joints.velocities.resize(kControlledJoints.size());
      joints.sample_time_seconds = 0.001 * sample_index;
      for (std::size_t index = 0; index < joint_ids.size(); ++index) {
        const auto joint_id = joint_ids[index];
        const Eigen::Index q_index = model.joints[joint_id].idx_q();
        double value = q[q_index];
        if (sample_index != 0) {
          const double lower = model.lowerPositionLimit[q_index];
          const double upper = model.upperPositionLimit[q_index];
          if (!std::isfinite(lower) || !std::isfinite(upper) ||
              lower >= upper) {
            throw std::runtime_error("invalid position limit for " +
                                     kControlledJoints[index]);
          }
          value =
              std::uniform_real_distribution<double>(lower, upper)(generator);
        }
        q[q_index] = value;
        joints.positions[index] = value;
        joints.velocities[index] =
            sample_index == 0
                ? 0.0
                : std::uniform_real_distribution<double>(-2.0, 2.0)(
                      generator);
      }

      pinocchio::forwardKinematics(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      simulation.setKinematicState(joints);
      simulation.forward();
      const auto snapshot = simulation.snapshot();
      if (snapshot.joints.names != kControlledJoints ||
          snapshot.joints.positions.size() != joints.positions.size() ||
          snapshot.joints.velocities.size() != joints.velocities.size() ||
          std::abs(snapshot.sample_time_seconds - joints.sample_time_seconds) >
              1.0e-15) {
        throw std::runtime_error(
            "MuJoCo committed joint-state ordering or sample time changed");
      }
      for (std::size_t index = 0; index < joints.positions.size(); ++index) {
        if (std::abs(snapshot.joints.positions[index] -
                     joints.positions[index]) > 1.0e-12 ||
            std::abs(snapshot.joints.velocities[index] -
                     joints.velocities[index]) > 1.0e-12) {
          throw std::runtime_error(
              "MuJoCo committed joint position/velocity units changed at " +
              kControlledJoints[index]);
        }
      }
      if (std::abs(snapshot.floating_base.pose.position_m[0]) > 1.0e-15 ||
          std::abs(snapshot.floating_base.pose.position_m[1]) > 1.0e-15 ||
          std::abs(snapshot.floating_base.pose.position_m[2]) > 1.0e-15 ||
          std::abs(snapshot.floating_base.pose.orientation_wxyz[0] - 1.0) >
              1.0e-15 ||
          std::abs(snapshot.floating_base.pose.orientation_wxyz[1]) >
              1.0e-15 ||
          std::abs(snapshot.floating_base.pose.orientation_wxyz[2]) >
              1.0e-15 ||
          std::abs(snapshot.floating_base.pose.orientation_wxyz[3]) >
              1.0e-15 ||
          std::any_of(snapshot.floating_base.twist.linear_mps.begin(),
                      snapshot.floating_base.twist.linear_mps.end(),
                      [](double value) { return std::abs(value) > 1.0e-15; }) ||
          std::any_of(snapshot.floating_base.twist.angular_radps.begin(),
                      snapshot.floating_base.twist.angular_radps.end(),
                      [](double value) { return std::abs(value) > 1.0e-15; })) {
        throw std::runtime_error("MuJoCo floating base is not fixed at identity");
      }

      Eigen::Isometry3d tcp_offset = Eigen::Isometry3d::Identity();
      tcp_offset.translation().z() = 0.1;
      Eigen::Isometry3d left_ee = Eigen::Isometry3d::Identity();
      left_ee.translation() = data.oMf[left_frame].translation();
      left_ee.linear() = data.oMf[left_frame].rotation();
      Eigen::Isometry3d right_ee = Eigen::Isometry3d::Identity();
      right_ee.translation() = data.oMf[right_frame].translation();
      right_ee.linear() = data.oMf[right_frame].rotation();
      const Eigen::Isometry3d expected_left = left_ee * tcp_offset;
      const Eigen::Isometry3d expected_right = right_ee * tcp_offset;
      compare(expected_left, sitePose(site(snapshot, "left_tcp_site")), "left",
              sample_index);
      compare(expected_right, sitePose(site(snapshot, "right_tcp_site")),
              "right", sample_index);
    }
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
