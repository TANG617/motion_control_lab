#include "planning.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion_control_lab::planned_hierarchical_step {
namespace {

void clampComponent(double &value, double limit, std::size_t segment_index,
                    RetargetClampComponent component, std::size_t axis,
                    RetargetClampDiagnostics &diagnostics) {
  if (!std::isfinite(value) || !std::isfinite(limit) || limit <= 0.0 ||
      (value >= -limit && value <= limit)) {
    return;
  }
  if (diagnostics.clamped_component_count >= diagnostics.events.size()) {
    throw std::logic_error(
        "planned retarget clamp diagnostic capacity exceeded");
  }
  const double applied = std::clamp(value, -limit, limit);
  diagnostics.events[diagnostics.clamped_component_count] =
      RetargetClampEvent{segment_index, component, axis, value, applied, limit};
  ++diagnostics.clamped_component_count;
  diagnostics.maximum_limit_ratio =
      std::max(diagnostics.maximum_limit_ratio, std::abs(value) / limit);
  value = applied;
}

template <typename SpatialVector>
void clampTriplet(SpatialVector &values, std::size_t offset,
                  const Eigen::Vector3d &limits, std::size_t segment_index,
                  RetargetClampComponent component,
                  RetargetClampDiagnostics &diagnostics) {
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    clampComponent(values[static_cast<Eigen::Index>(offset + axis)],
                   limits[static_cast<Eigen::Index>(axis)], segment_index,
                   component, axis, diagnostics);
  }
}

} // namespace

motion_control::core::CartesianRetargetRequest makeRetargetRequest(
    const motion_control::core::Pose &left_goal,
    const motion_control::core::Pose &right_goal,
    const motion_control::core::CartesianTrajectorySample &accepted,
    const R1RobotConfig &robot, const PlanningLimitOptions &limits,
    double rate_hz) {
  namespace mcc = motion_control::core;
  mcc::CartesianRetargetRequest request;
  request.reference_frame_name = robot.base_frame;
  request.sample_period = 1.0 / rate_hz;
  request.synchronization = mcc::TrajectorySynchronization::Time;
  request.limits.max_linear_velocity =
      Eigen::Vector3d::Constant(limits.max_linear_velocity_mps);
  request.limits.max_linear_acceleration =
      Eigen::Vector3d::Constant(limits.max_linear_acceleration_mps2);
  request.limits.max_linear_jerk =
      Eigen::Vector3d::Constant(limits.max_linear_jerk_mps3);
  request.limits.max_rotation_vector_velocity =
      Eigen::Vector3d::Constant(limits.max_angular_velocity_rps);
  request.limits.max_rotation_vector_acceleration =
      Eigen::Vector3d::Constant(limits.max_angular_acceleration_rps2);
  request.limits.max_rotation_vector_jerk =
      Eigen::Vector3d::Constant(limits.max_angular_jerk_rps3);
  request.segments = {{robot.left_end_effector_frame,
                       accepted.frames.at(0).pose, accepted.frames.at(0).twist,
                       accepted.frames.at(0).acceleration, left_goal},
                      {robot.right_end_effector_frame,
                       accepted.frames.at(1).pose, accepted.frames.at(1).twist,
                       accepted.frames.at(1).acceleration, right_goal}};
  return request;
}

RetargetClampDiagnostics clampRetargetCurrentState(
    motion_control::core::CartesianRetargetRequest &request) {
  if (request.segments.size() > 2U) {
    throw std::logic_error(
        "planned hierarchical Step retarget clamp expects at most two arms");
  }
  RetargetClampDiagnostics diagnostics;
  for (std::size_t index = 0U; index < request.segments.size(); ++index) {
    auto &segment = request.segments[index];
    clampTriplet(segment.current_twist, 0U, request.limits.max_linear_velocity,
                 index, RetargetClampComponent::LinearVelocity, diagnostics);
    clampTriplet(segment.current_twist, 3U,
                 request.limits.max_rotation_vector_velocity, index,
                 RetargetClampComponent::AngularVelocity, diagnostics);
    clampTriplet(segment.current_acceleration, 0U,
                 request.limits.max_linear_acceleration, index,
                 RetargetClampComponent::LinearAcceleration, diagnostics);
    clampTriplet(segment.current_acceleration, 3U,
                 request.limits.max_rotation_vector_acceleration, index,
                 RetargetClampComponent::AngularAcceleration, diagnostics);
  }
  return diagnostics;
}

const char *plannerStateName(motion_control::core::PlanningState state) {
  using motion_control::core::PlanningState;
  switch (state) {
  case PlanningState::Idle:
    return "idle";
  case PlanningState::Planning:
    return "planning";
  case PlanningState::Finished:
    return "finished";
  case PlanningState::Error:
    return "error";
  }
  return "unknown";
}

} // namespace motion_control_lab::planned_hierarchical_step
