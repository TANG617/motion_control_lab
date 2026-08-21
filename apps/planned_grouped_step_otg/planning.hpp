#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <motion_control_core/motion_control_core.hpp>

#include "components/robot/r1/r1_robot_config.hpp"

namespace motion_control_lab::planned_grouped_step_otg {

struct PlanningLimitOptions;

enum class JointTargetMode {
  FutureO1Pv,
  IkPv,
};

const char *jointTargetModeName(JointTargetMode mode);

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

inline constexpr double kFutureO1VelocityDeadbandRadPerS = 1.0e-10;
inline constexpr std::size_t kR1JointCount = 20U;
inline constexpr char kR1StreamProfileRevision[] =
    "42ed3ce3a19f5a7346874a31ec659c0298751137";
inline constexpr char kR1StreamProfileSha256[] =
    "895416681f8fd41138f3be280b0f7a330ca004f46951b12b3fea2a230e779d1b";

inline constexpr std::array<const char *, kR1JointCount> kR1StreamJointNames{
    "head_yaw_joint",    "head_pitch_joint", "torso_yaw_joint",
    "torso_pitch_joint", "knee_pitch_joint", "ankle_pitch_joint",
    "left_arm_joint1",   "left_arm_joint2",  "left_arm_joint3",
    "left_arm_joint4",   "left_arm_joint5",  "left_arm_joint6",
    "left_arm_joint7",   "right_arm_joint1", "right_arm_joint2",
    "right_arm_joint3",  "right_arm_joint4", "right_arm_joint5",
    "right_arm_joint6",  "right_arm_joint7"};

inline constexpr std::array<double, kR1JointCount> kR1StreamMaxVelocityRadPerS{
    3.0, 3.0, 3.0, 2.0,  2.0,  2.0,  5.05, 5.05, 5.71, 5.24,
    4.1, 4.1, 4.1, 5.05, 5.05, 5.71, 5.24, 4.1,  4.1,  4.1};
inline constexpr std::array<double, kR1JointCount>
    kR1StreamMaxAccelerationRadPerS2{
        6.0,  6.0,  6.0,  4.0,   4.0,   4.0,   10.10, 10.10, 12.42, 12.48,
        16.2, 16.2, 16.2, 10.10, 10.10, 12.42, 12.48, 16.2,  16.2,  16.2};
inline constexpr std::array<double, kR1JointCount> kR1StreamMaxJerkRadPerS3{
    3200.0, 3200.0, 3200.0, 3200.0, 3200.0, 3200.0, 3200.0,
    3200.0, 3200.0, 3200.0, 3200.0, 3200.0, 3200.0, 3200.0,
    3200.0, 3200.0, 3200.0, 3200.0, 3200.0, 3200.0};

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
makeJointTargetLimits(const motion_control::core::RobotModel &model,
                      const R1RobotConfig &robot);

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
  JointTargetBuilder(
      JointTargetMode mode, double sample_period, std::size_t joint_count,
      double velocity_deadband = kFutureO1VelocityDeadbandRadPerS);

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
  explicit ReplaySettlingCounter(std::size_t required_cycles = 20U)
      : required_cycles_(required_cycles) {}

  bool update(bool input_consumed, bool cartesian_finished,
              double left_position_error_m, double left_orientation_error_rad,
              double right_position_error_m, double right_orientation_error_rad,
              double maximum_velocity_rad_per_s,
              double maximum_acceleration_rad_per_s2);
  std::size_t consecutiveCycles() const { return consecutive_cycles_; }

private:
  std::size_t required_cycles_;
  std::size_t consecutive_cycles_{0U};
};

motion_control::core::CartesianRetargetRequest makeRetargetRequest(
    const motion_control::core::Pose &left_goal,
    const motion_control::core::Pose &right_goal,
    const motion_control::core::CartesianTrajectorySample &accepted,
    const R1RobotConfig &robot, const PlanningLimitOptions &limits,
    double rate_hz);

RetargetClampDiagnostics clampRetargetCurrentState(
    motion_control::core::CartesianRetargetRequest &request);

const char *plannerStateName(motion_control::core::PlanningState state);

} // namespace motion_control_lab::planned_grouped_step_otg
