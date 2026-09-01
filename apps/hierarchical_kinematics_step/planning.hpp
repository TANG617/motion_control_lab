#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <motion_control_core/motion_control_core.hpp>

#include "options.hpp"

namespace motion_control_lab::hierarchical_kinematics_step {

enum class RetargetClampComponent {
  LinearVelocity,
  AngularVelocity,
  LinearAcceleration,
  AngularAcceleration,
};

struct RetargetClampEvent {
  std::size_t segment_index{0U};
  RetargetClampComponent component{RetargetClampComponent::LinearVelocity};
  std::size_t axis{0U};
  double original_value{0.0};
  double applied_value{0.0};
  double limit{0.0};
};

inline constexpr std::size_t kMaximumRetargetClampEvents{24U};

struct RetargetClampDiagnostics {
  std::array<RetargetClampEvent, kMaximumRetargetClampEvents> events{};
  std::size_t clamped_component_count{0U};
  double maximum_limit_ratio{0.0};

  bool clamped() const { return clamped_component_count != 0U; }
};

struct JointTarget {
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
  bool future_o1_startup{false};
};

struct JointTargetLimits {
  std::vector<double> position_lower;
  std::vector<double> position_upper;
  std::vector<double> max_velocity;
  std::vector<double> max_acceleration;
  std::vector<double> max_jerk;
};

JointTargetLimits
makeJointTargetLimits(const R1RobotConfig &robot,
                      const JointStreamProfileOptions &profile);

enum class ProjectionComponent {
  VelocityLimit,
  AccelerationLimit,
  JerkStoppingEnvelope,
};

struct ProjectionEvent {
  std::size_t joint_index{0U};
  ProjectionComponent component{ProjectionComponent::VelocityLimit};
  double original_value{0.0};
  double applied_value{0.0};
  double limit{0.0};
};

struct ProjectionDiagnostics {
  std::vector<ProjectionEvent> events;
  std::size_t modified_joint_count{0U};

  bool projected() const { return !events.empty(); }
};

class JointTargetBuilder {
public:
  JointTargetBuilder(const JointTargetOptions &options, double sample_period,
                     std::size_t joint_count);

  JointTarget preview(const std::vector<double> &raw_positions,
                      const std::vector<double> &raw_velocities) const;
  void commit(const std::vector<double> &raw_positions,
              const JointTarget &target);
  std::size_t acceptedSampleCount() const { return accepted_sample_count_; }

private:
  JointTargetMode mode_;
  double sample_period_;
  std::size_t joint_count_;
  double velocity_deadband_;
  std::vector<double> previous_position_;
  std::vector<double> position_before_previous_;
  std::vector<double> previous_target_velocity_;
  std::size_t accepted_sample_count_{0U};
};

JointTarget
mapActiveIkToFull(const std::vector<double> &current_positions,
                  const std::vector<std::size_t> &active_joint_full_indices,
                  const std::vector<double> &active_positions,
                  const std::vector<double> &active_velocities);

JointTarget projectConfiguredLimits(const JointTarget &raw,
                                    const JointTargetLimits &limits,
                                    ProjectionDiagnostics &diagnostics);

const char *projectionComponentName(ProjectionComponent component);

class ReplaySettlingCounter {
public:
  explicit ReplaySettlingCounter(const ReplaySettlingOptions &options)
      : options_(options) {}

  bool update(bool input_consumed, bool cartesian_finished,
              double left_position_error_m, double left_orientation_error_rad,
              double right_position_error_m, double right_orientation_error_rad,
              double maximum_velocity_rad_per_s,
              double maximum_acceleration_rad_per_s2);
  std::size_t consecutiveCycles() const { return consecutive_cycles_; }

private:
  ReplaySettlingOptions options_;
  std::size_t consecutive_cycles_{0U};
};

motion_control::core::JointPlannerConfig
makeJointPlannerConfig(const PlanningOptions &options);

motion_control::core::CartesianRetargetRequest makeRetargetRequest(
    const motion_control::core::Pose &left_goal,
    const motion_control::core::Pose &right_goal,
    const motion_control::core::CartesianTrajectorySample &accepted,
    const R1RobotConfig &robot, const PlanningOptions &options,
    double rate_hz);

RetargetClampDiagnostics clampRetargetCurrentState(
    motion_control::core::CartesianRetargetRequest &request);

const char *plannerStateName(motion_control::core::PlanningState state);

} // namespace motion_control_lab::hierarchical_kinematics_step
