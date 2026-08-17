#pragma once

#include <array>

namespace motion_control_lab::contracts::
    foxglove_planned_grouped_servo_step_v1 {

inline constexpr char kSchemaVersion[] =
    "mcl.foxglove_planned_grouped_servo_step.v1";

inline constexpr char kLeftPlanningRequestPose[] =
    "/mc/planning/request/left_pose";
inline constexpr char kRightPlanningRequestPose[] =
    "/mc/planning/request/right_pose";

inline constexpr char kLeftPlanningRequestEntity[] = "left_planning_request";
inline constexpr char kRightPlanningRequestEntity[] = "right_planning_request";

inline constexpr std::array<const char *, 2> kRequiredChannels{
    kLeftPlanningRequestPose, kRightPlanningRequestPose};

} // namespace
  // motion_control_lab::contracts::foxglove_planned_grouped_servo_step_v1
