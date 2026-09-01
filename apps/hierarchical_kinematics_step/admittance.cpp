#include "admittance.hpp"

#include <motion_control_core/transform.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace motion_control_lab::hierarchical_kinematics_step {
namespace {

mcc::Pose toCorePose(const mcs::Pose3d &pose) {
  const Eigen::Quaterniond orientation(
      pose.orientation_wxyz[0], pose.orientation_wxyz[1],
      pose.orientation_wxyz[2], pose.orientation_wxyz[3]);
  if (!orientation.coeffs().allFinite() || orientation.norm() <= 1.0e-12) {
    throw std::runtime_error("drag reference contains an invalid orientation");
  }
  mcc::Pose result = mcc::Pose::Identity();
  result.translation() = Eigen::Vector3d{pose.position_m[0], pose.position_m[1],
                                         pose.position_m[2]};
  result.linear() = orientation.normalized().toRotationMatrix();
  if (!mcc::isRigidTransform(result)) {
    throw std::runtime_error("drag reference contains a non-finite pose");
  }
  return result;
}

Eigen::Vector3d clampNorm(const Eigen::Vector3d &value, double maximum) {
  const double norm = value.norm();
  if (norm <= maximum || norm <= 1.0e-15) {
    return value;
  }
  return value * (maximum / norm);
}

mcc::Wrench lowPass(const mcc::Wrench &previous, const mcc::Wrench &current,
                    double alpha) {
  mcc::Wrench result;
  result.force = alpha * current.force + (1.0 - alpha) * previous.force;
  result.torque = alpha * current.torque + (1.0 - alpha) * previous.torque;
  return result;
}

void requireOk(const mcc::Status &status, const char *side) {
  if (!status.ok()) {
    throw std::runtime_error(std::string{side} +
                             " Cartesian admittance failed: " + status.message);
  }
}

} // namespace

mcc::CartesianFrameSample shiftCartesianControlPoint(
    const mcc::CartesianFrameSample &source,
    const mcc::Pose &source_to_target,
    const mcc::FrameName &target_frame_name) {
  mcc::CartesianFrameSample target = source;
  target.frame_name = target_frame_name;
  target.pose = source.pose * source_to_target;
  const Eigen::Vector3d source_to_target_in_reference =
      target.pose.translation() - source.pose.translation();
  const Eigen::Vector3d angular_velocity = source.twist.tail<3>();
  const Eigen::Vector3d angular_acceleration = source.acceleration.tail<3>();
  target.twist.head<3>() =
      source.twist.head<3>() +
      angular_velocity.cross(source_to_target_in_reference);
  target.acceleration.head<3>() =
      source.acceleration.head<3>() +
      angular_acceleration.cross(source_to_target_in_reference) +
      angular_velocity.cross(
          angular_velocity.cross(source_to_target_in_reference));
  return target;
}

void DualArmAdmittance::configure(const AdmittanceOptions &options) {
  const std::array<double, 6> positive_values{
      {options.linear_environment_stiffness_n_per_m,
       options.linear_environment_damping_ns_per_m, options.maximum_force_n,
       options.angular_environment_stiffness_nm_per_rad,
       options.angular_environment_damping_nms_per_rad,
       options.maximum_torque_nm}};
  if (std::any_of(
          positive_values.begin(), positive_values.end(),
          [](double value) { return !std::isfinite(value) || value <= 0.0; }) ||
      !std::isfinite(options.wrench_filter_alpha) ||
      options.wrench_filter_alpha <= 0.0 || options.wrench_filter_alpha > 1.0) {
    throw std::invalid_argument("invalid admittance/environment configuration");
  }
  mcc::CartesianAdmittanceConfig config;
  config.enabled = {{true, true, true, options.angular_enabled,
                     options.angular_enabled, options.angular_enabled}};
  requireOk(left_.admittance.configure(config), "left");
  requireOk(right_.admittance.configure(config), "right");
  scalar_options_ = {{options.linear_environment_stiffness_n_per_m,
                      options.linear_environment_damping_ns_per_m,
                      options.maximum_force_n,
                      options.angular_environment_stiffness_nm_per_rad,
                      options.angular_environment_damping_nms_per_rad,
                      options.maximum_torque_nm, options.wrench_filter_alpha}};
  angular_enabled_ = options.angular_enabled;
  reset();
}

void DualArmAdmittance::reset() {
  left_.admittance.reset();
  right_.admittance.reset();
  left_.filtered_wrench = {};
  right_.filtered_wrench = {};
  left_.has_previous_actual_pose = false;
  right_.has_previous_actual_pose = false;
}

ArmAdmittanceOutput
DualArmAdmittance::stepArm(ArmState &state,
                           const mcc::CartesianFrameSample &nominal_link,
                           const mcc::Pose &actual_link_pose,
                           const mcc::Pose &tcp_offset,
                           const mcs::DragReference &drag, double dt_seconds) {
  if (!std::isfinite(dt_seconds) || dt_seconds <= 1.0e-6) {
    throw std::runtime_error("admittance step requires dt > 1e-6 seconds");
  }
  if (!mcc::isRigidTransform(nominal_link.pose) ||
      !mcc::isRigidTransform(actual_link_pose) ||
      !mcc::isRigidTransform(tcp_offset)) {
    throw std::runtime_error("admittance received a non-rigid pose");
  }

  ArmAdmittanceOutput output;
  output.nominal_link = nominal_link;
  output.nominal_tcp = shiftCartesianControlPoint(
      nominal_link, tcp_offset, nominal_link.frame_name + "_tcp");
  output.actual_tcp_pose = actual_link_pose * tcp_offset;
  output.drag_active = drag.active;

  if (state.has_previous_actual_pose) {
    output.actual_tcp_twist.head<3>() =
        (output.actual_tcp_pose.translation() -
         state.previous_actual_tcp_pose.translation()) /
        dt_seconds;
    output.actual_tcp_twist.tail<3>() =
        mcc::transform::rotationLog(
            output.actual_tcp_pose.linear() *
            state.previous_actual_tcp_pose.linear().transpose()) /
        dt_seconds;
  }
  state.previous_actual_tcp_pose = output.actual_tcp_pose;
  state.has_previous_actual_pose = true;

  if (drag.active) {
    const mcc::Pose drag_pose = toCorePose(drag.target_pose);
    output.raw_wrench.force =
        scalar_options_[0] *
            (drag_pose.translation() - output.actual_tcp_pose.translation()) -
        scalar_options_[1] * output.actual_tcp_twist.head<3>();
    output.raw_wrench.force =
        clampNorm(output.raw_wrench.force, scalar_options_[2]);
    if (angular_enabled_) {
      const Eigen::Vector3d orientation_error = mcc::transform::rotationLog(
          drag_pose.linear() * output.actual_tcp_pose.linear().transpose());
      output.raw_wrench.torque = scalar_options_[3] * orientation_error -
                                 scalar_options_[4] *
                                     output.actual_tcp_twist.tail<3>();
      output.raw_wrench.torque =
          clampNorm(output.raw_wrench.torque, scalar_options_[5]);
    }
  }

  state.filtered_wrench =
      lowPass(state.filtered_wrench, output.raw_wrench, scalar_options_[6]);
  output.filtered_wrench = state.filtered_wrench;

  mcc::CartesianAdmittanceInput input;
  input.reference_pose = output.nominal_tcp.pose;
  input.reference_twist = output.nominal_tcp.twist;
  input.reference_acceleration = output.nominal_tcp.acceleration;
  input.external_wrench = output.filtered_wrench;
  input.dt = dt_seconds;
  mcc::CartesianAdmittanceOutput admittance_output;
  requireOk(state.admittance.step(input, admittance_output, output.diagnostics),
            "arm");
  output.command_tcp = output.nominal_tcp;
  output.command_tcp.pose = admittance_output.command_pose;
  output.command_tcp.twist = admittance_output.command_twist;
  output.command_tcp.acceleration = admittance_output.command_acceleration;
  output.command_link = shiftCartesianControlPoint(
      output.command_tcp, tcp_offset.inverse(), nominal_link.frame_name);
  return output;
}

DualAdmittanceOutput DualArmAdmittance::step(
    const mcc::CartesianTrajectorySample &nominal_link_sample,
    const mcc::Pose &left_actual_link_pose,
    const mcc::Pose &right_actual_link_pose, const mcc::Pose &left_tcp_offset,
    const mcc::Pose &right_tcp_offset,
    const std::array<mcs::DragReference, 2> &drag_references,
    double dt_seconds) {
  DualAdmittanceOutput output;
  output.left = stepArm(left_, nominal_link_sample.frames.at(0),
                        left_actual_link_pose,
                        left_tcp_offset, drag_references[0], dt_seconds);
  output.right = stepArm(right_, nominal_link_sample.frames.at(1),
                         right_actual_link_pose, right_tcp_offset,
                         drag_references[1], dt_seconds);
  output.command_sample = nominal_link_sample;
  output.command_sample.frames = {output.left.command_link,
                                  output.right.command_link};
  return output;
}

} // namespace motion_control_lab::hierarchical_kinematics_step
