#include "joint_target_processing.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace motion_control_lab::planned_grouped_step_otg {
namespace {

void requireSize(const std::vector<double> &values, std::size_t expected,
                 const char *name) {
  if (values.size() != expected) {
    throw std::runtime_error(std::string{name} + " size mismatch");
  }
  if (!std::all_of(values.begin(), values.end(),
                   [](double value) { return std::isfinite(value); })) {
    throw std::runtime_error(std::string{name} + " contains a non-finite value");
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

JointTargetBuilder::JointTargetBuilder(JointTargetMode mode,
                                       double sample_period,
                                       std::size_t joint_count,
                                       double velocity_deadband)
    : mode_(mode), sample_period_(sample_period), joint_count_(joint_count),
      velocity_deadband_(velocity_deadband),
      previous_target_velocity_(joint_count, 0.0) {
  if (!std::isfinite(sample_period_) || sample_period_ <= 0.0 ||
      joint_count_ == 0U || !std::isfinite(velocity_deadband_) ||
      velocity_deadband_ < 0.0) {
    throw std::runtime_error("invalid joint target builder configuration");
  }
}

JointTarget JointTargetBuilder::preview(
    const std::vector<double> &raw_positions,
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

JointTarget mapActiveIkToFull(
    const std::vector<double> &current_positions,
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
      throw std::runtime_error("target position outside configured limit at joint " +
                               std::to_string(index));
    }
    const double raw_acceleration = projected.accelerations[index];
    projected.accelerations[index] =
        std::clamp(raw_acceleration, -limits.max_acceleration[index],
                   limits.max_acceleration[index]);
    record(index, ProjectionComponent::AccelerationLimit, raw_acceleration,
           projected.accelerations[index], limits.max_acceleration[index]);

    const double raw_velocity = projected.velocities[index];
    projected.velocities[index] =
        std::clamp(raw_velocity, -limits.max_velocity[index],
                   limits.max_velocity[index]);
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
      record(index, ProjectionComponent::JerkStoppingEnvelope,
             before_envelope, projected.velocities[index], reverse_lower);
    } else if (acceleration < 0.0) {
      const double reverse_upper =
          limits.max_velocity[index] -
          acceleration * acceleration / (2.0 * limits.max_jerk[index]);
      projected.velocities[index] =
          std::min(projected.velocities[index], reverse_upper);
      record(index, ProjectionComponent::JerkStoppingEnvelope,
             before_envelope, projected.velocities[index], reverse_upper);
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

bool ReplaySettlingCounter::update(
    bool input_consumed, bool cartesian_finished, double left_position_error_m,
    double left_orientation_error_rad, double right_position_error_m,
    double right_orientation_error_rad, double maximum_velocity_rad_per_s,
    double maximum_acceleration_rad_per_s2) {
  const bool settled =
      input_consumed && cartesian_finished &&
      left_position_error_m <= 1.0e-4 &&
      left_orientation_error_rad <= 1.0e-4 &&
      right_position_error_m <= 1.0e-4 &&
      right_orientation_error_rad <= 1.0e-4 &&
      maximum_velocity_rad_per_s <= 1.0e-3 &&
      maximum_acceleration_rad_per_s2 <= 1.0e-2;
  consecutive_cycles_ = settled ? consecutive_cycles_ + 1U : 0U;
  return consecutive_cycles_ >= required_cycles_;
}

} // namespace motion_control_lab::planned_grouped_step_otg
