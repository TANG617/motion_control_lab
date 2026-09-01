#include "planning.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace motion_control_lab::hierarchical_kinematics_step {
namespace {

motion_control::core::TrajectorySynchronization
toCoreSynchronization(PlanningSynchronization value) {
  switch (value) {
  case PlanningSynchronization::None:
    return motion_control::core::TrajectorySynchronization::None;
  case PlanningSynchronization::Time:
    return motion_control::core::TrajectorySynchronization::Time;
  case PlanningSynchronization::Phase:
    return motion_control::core::TrajectorySynchronization::Phase;
  }
  throw std::logic_error("unknown planning synchronization");
}

motion_control::core::JointTrajectoryAlgorithm
toCoreJointAlgorithm(JointPlanningAlgorithm value) {
  switch (value) {
  case JointPlanningAlgorithm::JerkLimited:
    return motion_control::core::JointTrajectoryAlgorithm::JerkLimited;
  }
  throw std::logic_error("unknown joint planning algorithm");
}

void requireSize(const std::vector<double> &values, std::size_t expected,
                 const char *name) {
  if (values.size() != expected) {
    throw std::runtime_error(std::string{name} + " size mismatch");
  }
  if (!std::all_of(values.begin(), values.end(),
                   [](double value) { return std::isfinite(value); })) {
    throw std::runtime_error(std::string{name} +
                             " contains a non-finite value");
  }
}

void requireLimits(const JointTargetLimits &limits, std::size_t count) {
  requireSize(limits.position_lower, count, "position lower limits");
  requireSize(limits.position_upper, count, "position upper limits");
  requireSize(limits.max_velocity, count, "velocity limits");
  requireSize(limits.max_acceleration, count, "acceleration limits");
  requireSize(limits.max_jerk, count, "jerk limits");
  for (std::size_t index = 0U; index < count; ++index) {
    if (limits.position_lower[index] > limits.position_upper[index] ||
        limits.max_velocity[index] <= 0.0 ||
        limits.max_acceleration[index] <= 0.0 ||
        limits.max_jerk[index] <= 0.0) {
      throw std::runtime_error("invalid configured joint limit");
    }
  }
}

} // namespace

JointTargetBuilder::JointTargetBuilder(const JointTargetOptions &options,
                                       double sample_period,
                                       std::size_t joint_count)
    : mode_(options.mode), sample_period_(sample_period),
      joint_count_(joint_count),
      velocity_deadband_(options.future_o1_velocity_deadband_rad_per_s),
      previous_target_velocity_(joint_count, 0.0) {
  if (!std::isfinite(sample_period_) || sample_period_ <= 0.0 ||
      joint_count_ == 0U || !std::isfinite(velocity_deadband_) ||
      velocity_deadband_ < 0.0) {
    throw std::runtime_error("invalid joint target builder configuration");
  }
}

JointTarget
JointTargetBuilder::preview(const std::vector<double> &raw_positions,
                            const std::vector<double> &raw_velocities) const {
  requireSize(raw_positions, joint_count_, "raw positions");
  requireSize(raw_velocities, joint_count_, "raw velocities");

  JointTarget target;
  target.positions = raw_positions;
  target.velocities.assign(joint_count_, 0.0);
  target.accelerations.assign(joint_count_, 0.0);
  if (mode_ == JointTargetMode::IkPv) {
    target.velocities = raw_velocities;
    return target;
  }

  target.future_o1_startup = accepted_sample_count_ < 2U;
  if (target.future_o1_startup) {
    return target;
  }

  for (std::size_t index = 0U; index < joint_count_; ++index) {
    target.positions[index] = 3.0 * raw_positions[index] -
                              3.0 * previous_position_[index] +
                              position_before_previous_[index];
    const double velocity =
        (2.0 * raw_positions[index] - 3.0 * previous_position_[index] +
         position_before_previous_[index]) /
        sample_period_;
    target.velocities[index] =
        std::abs(velocity - previous_target_velocity_[index]) <=
                velocity_deadband_
            ? previous_target_velocity_[index]
            : velocity;
  }
  return target;
}

void JointTargetBuilder::commit(const std::vector<double> &raw_positions,
                                const JointTarget &target) {
  requireSize(raw_positions, joint_count_, "committed raw positions");
  requireSize(target.velocities, joint_count_, "committed target velocities");
  if (accepted_sample_count_ > 0U) {
    position_before_previous_ = previous_position_;
  }
  previous_position_ = raw_positions;
  previous_target_velocity_ = target.velocities;
  ++accepted_sample_count_;
}

JointTarget
mapActiveIkToFull(const std::vector<double> &current_positions,
                  const std::vector<std::size_t> &active_joint_full_indices,
                  const std::vector<double> &active_positions,
                  const std::vector<double> &active_velocities) {
  requireSize(current_positions, current_positions.size(), "current positions");
  requireSize(active_positions, active_joint_full_indices.size(),
              "active positions");
  requireSize(active_velocities, active_joint_full_indices.size(),
              "active velocities");
  JointTarget result;
  result.positions = current_positions;
  result.velocities.assign(current_positions.size(), 0.0);
  result.accelerations.assign(current_positions.size(), 0.0);
  std::vector<bool> seen(current_positions.size(), false);
  for (std::size_t active_index = 0U;
       active_index < active_joint_full_indices.size(); ++active_index) {
    const auto full_index = active_joint_full_indices[active_index];
    if (full_index >= current_positions.size() || seen[full_index]) {
      throw std::runtime_error("invalid active-to-full joint mapping");
    }
    seen[full_index] = true;
    result.positions[full_index] = active_positions[active_index];
    result.velocities[full_index] = active_velocities[active_index];
  }
  return result;
}

JointTargetLimits
makeJointTargetLimits(const R1RobotConfig &robot,
                      const JointStreamProfileOptions &profile) {
  if (robot.joint_names.size() != profile.joint_names.size()) {
    throw std::runtime_error("R1 joint OTG profile requires exactly 20 joints");
  }
  JointTargetLimits result;
  result.position_lower.reserve(robot.joint_names.size());
  result.position_upper.reserve(robot.joint_names.size());
  result.max_velocity.reserve(robot.joint_names.size());
  result.max_acceleration.reserve(robot.joint_names.size());
  result.max_jerk.reserve(robot.joint_names.size());
  for (std::size_t index = 0U; index < robot.joint_names.size(); ++index) {
    if (robot.joint_names[index] != profile.joint_names[index]) {
      throw std::runtime_error(
          "runtime R1 joint order does not match stream profile at index " +
          std::to_string(index));
    }
    result.position_lower.push_back(profile.position_lower_rad[index]);
    result.position_upper.push_back(profile.position_upper_rad[index]);
    result.max_velocity.push_back(profile.max_velocity_rad_per_s[index]);
    result.max_acceleration.push_back(
        profile.max_acceleration_rad_per_s2[index]);
    result.max_jerk.push_back(profile.max_jerk_rad_per_s3[index]);
  }
  return result;
}

JointTarget projectConfiguredLimits(const JointTarget &raw,
                                    const JointTargetLimits &limits,
                                    ProjectionDiagnostics &diagnostics) {
  const std::size_t count = raw.positions.size();
  requireSize(raw.velocities, count, "target velocities");
  requireSize(raw.accelerations, count, "target accelerations");
  requireLimits(limits, count);
  diagnostics = ProjectionDiagnostics{};
  JointTarget projected = raw;
  std::vector<bool> modified(count, false);
  const auto record = [&](std::size_t index, ProjectionComponent component,
                          double original, double applied, double limit) {
    if (original == applied) {
      return;
    }
    diagnostics.events.push_back(
        ProjectionEvent{index, component, original, applied, limit});
    modified[index] = true;
  };

  for (std::size_t index = 0U; index < count; ++index) {
    if (raw.positions[index] < limits.position_lower[index] ||
        raw.positions[index] > limits.position_upper[index]) {
      throw std::runtime_error(
          "target position outside configured limit at joint " +
          std::to_string(index));
    }
    const double raw_acceleration = projected.accelerations[index];
    projected.accelerations[index] =
        std::clamp(raw_acceleration, -limits.max_acceleration[index],
                   limits.max_acceleration[index]);
    record(index, ProjectionComponent::AccelerationLimit, raw_acceleration,
           projected.accelerations[index], limits.max_acceleration[index]);

    const double raw_velocity = projected.velocities[index];
    projected.velocities[index] = std::clamp(
        raw_velocity, -limits.max_velocity[index], limits.max_velocity[index]);
    record(index, ProjectionComponent::VelocityLimit, raw_velocity,
           projected.velocities[index], limits.max_velocity[index]);

    const double acceleration = projected.accelerations[index];
    const double before_envelope = projected.velocities[index];
    if (acceleration > 0.0) {
      const double reverse_lower =
          -limits.max_velocity[index] +
          acceleration * acceleration / (2.0 * limits.max_jerk[index]);
      projected.velocities[index] =
          std::max(projected.velocities[index], reverse_lower);
      record(index, ProjectionComponent::JerkStoppingEnvelope, before_envelope,
             projected.velocities[index], reverse_lower);
    } else if (acceleration < 0.0) {
      const double reverse_upper =
          limits.max_velocity[index] -
          acceleration * acceleration / (2.0 * limits.max_jerk[index]);
      projected.velocities[index] =
          std::min(projected.velocities[index], reverse_upper);
      record(index, ProjectionComponent::JerkStoppingEnvelope, before_envelope,
             projected.velocities[index], reverse_upper);
    }
  }
  diagnostics.modified_joint_count = static_cast<std::size_t>(
      std::count(modified.begin(), modified.end(), true));
  return projected;
}

const char *projectionComponentName(ProjectionComponent component) {
  switch (component) {
  case ProjectionComponent::VelocityLimit:
    return "velocity-limit";
  case ProjectionComponent::AccelerationLimit:
    return "acceleration-limit";
  case ProjectionComponent::JerkStoppingEnvelope:
    return "jerk-stopping-envelope";
  }
  return "unknown";
}

bool ReplaySettlingCounter::update(bool input_consumed, bool cartesian_finished,
                                   double left_position_error_m,
                                   double left_orientation_error_rad,
                                   double right_position_error_m,
                                   double right_orientation_error_rad,
                                   double maximum_velocity_rad_per_s,
                                   double maximum_acceleration_rad_per_s2) {
  const bool settled = input_consumed && cartesian_finished &&
                       left_position_error_m <= options_.fk_position_m &&
                       left_orientation_error_rad <=
                           options_.fk_orientation_rad &&
                       right_position_error_m <= options_.fk_position_m &&
                       right_orientation_error_rad <=
                           options_.fk_orientation_rad &&
                       maximum_velocity_rad_per_s <=
                           options_.velocity_rad_per_s &&
                       maximum_acceleration_rad_per_s2 <=
                           options_.acceleration_rad_per_s2;
  consecutive_cycles_ = settled ? consecutive_cycles_ + 1U : 0U;
  return consecutive_cycles_ >= options_.required_cycles;
}

motion_control::core::JointPlannerConfig
makeJointPlannerConfig(const PlanningOptions &options) {
  motion_control::core::JointPlannerConfig config;
  config.algorithm = toCoreJointAlgorithm(options.joint_algorithm);
  config.synchronization =
      toCoreSynchronization(options.joint_synchronization);
  return config;
}

motion_control::core::CartesianRetargetRequest makeRetargetRequest(
    const motion_control::core::Pose &left_goal,
    const motion_control::core::Pose &right_goal,
    const motion_control::core::CartesianTrajectorySample &accepted,
    const R1RobotConfig &robot, const PlanningOptions &options,
    double rate_hz) {
  namespace mcc = motion_control::core;
  mcc::CartesianRetargetRequest request;
  request.reference_frame_name = robot.base_frame;
  request.sample_period = 1.0 / rate_hz;
  request.synchronization =
      toCoreSynchronization(options.cartesian_synchronization);
  request.limits.max_linear_velocity =
      Eigen::Vector3d::Constant(options.max_linear_velocity_mps);
  request.limits.max_linear_acceleration =
      Eigen::Vector3d::Constant(options.max_linear_acceleration_mps2);
  request.limits.max_linear_jerk =
      Eigen::Vector3d::Constant(options.max_linear_jerk_mps3);
  request.limits.max_rotation_vector_velocity =
      Eigen::Vector3d::Constant(options.max_angular_velocity_rps);
  request.limits.max_rotation_vector_acceleration =
      Eigen::Vector3d::Constant(options.max_angular_acceleration_rps2);
  request.limits.max_rotation_vector_jerk =
      Eigen::Vector3d::Constant(options.max_angular_jerk_rps3);
  request.segments = {{robot.left_end_effector_frame,
                       accepted.frames.at(0).pose, accepted.frames.at(0).twist,
                       accepted.frames.at(0).acceleration, left_goal},
                      {robot.right_end_effector_frame,
                       accepted.frames.at(1).pose, accepted.frames.at(1).twist,
                       accepted.frames.at(1).acceleration, right_goal}};
  return request;
}

namespace {

void clampRetargetComponent(double &value, double limit,
                            std::size_t segment_index,
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
void clampRetargetTriplet(SpatialVector &values, std::size_t offset,
                          const Eigen::Vector3d &limits,
                          std::size_t segment_index,
                          RetargetClampComponent component,
                          RetargetClampDiagnostics &diagnostics) {
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    clampRetargetComponent(values[static_cast<Eigen::Index>(offset + axis)],
                           limits[static_cast<Eigen::Index>(axis)],
                           segment_index, component, axis, diagnostics);
  }
}

} // namespace

RetargetClampDiagnostics clampRetargetCurrentState(
    motion_control::core::CartesianRetargetRequest &request) {
  if (request.segments.size() > 2U) {
    throw std::logic_error(
        "planned hierarchical Step retarget clamp expects at most two arms");
  }
  RetargetClampDiagnostics diagnostics;
  for (std::size_t index = 0U; index < request.segments.size(); ++index) {
    auto &segment = request.segments[index];
    clampRetargetTriplet(segment.current_twist, 0U,
                         request.limits.max_linear_velocity, index,
                         RetargetClampComponent::LinearVelocity, diagnostics);
    clampRetargetTriplet(segment.current_twist, 3U,
                         request.limits.max_rotation_vector_velocity, index,
                         RetargetClampComponent::AngularVelocity, diagnostics);
    clampRetargetTriplet(segment.current_acceleration, 0U,
                         request.limits.max_linear_acceleration, index,
                         RetargetClampComponent::LinearAcceleration,
                         diagnostics);
    clampRetargetTriplet(segment.current_acceleration, 3U,
                         request.limits.max_rotation_vector_acceleration, index,
                         RetargetClampComponent::AngularAcceleration,
                         diagnostics);
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

} // namespace motion_control_lab::hierarchical_kinematics_step
