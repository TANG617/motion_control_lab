#pragma once

#include <array>
#include <cstddef>

#include <motion_control_core/motion_control_core.hpp>

#include "components/robot/r1/r1_robot_config.hpp"
#include "options.hpp"

namespace motion_control_lab::planned_hierarchical_step {

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

motion_control::core::CartesianRetargetRequest makeRetargetRequest(
    const motion_control::core::Pose &left_goal,
    const motion_control::core::Pose &right_goal,
    const motion_control::core::CartesianTrajectorySample &accepted,
    const R1RobotConfig &robot, const PlanningLimitOptions &limits,
    double rate_hz);

RetargetClampDiagnostics clampRetargetCurrentState(
    motion_control::core::CartesianRetargetRequest &request);

const char *plannerStateName(motion_control::core::PlanningState state);

} // namespace motion_control_lab::planned_hierarchical_step
