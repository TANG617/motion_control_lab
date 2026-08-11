#pragma once

#include <array>

namespace motion_control_lab::contracts::foxglove_ik_v1
{

inline constexpr char kSchemaVersion[] = "mcl.foxglove_ik_visualization.v1";

inline constexpr char kJointStates[] = "/mc/ik/joint_states";
inline constexpr char kLeftTargetPose[] = "/mc/ik/target/left_pose";
inline constexpr char kRightTargetPose[] = "/mc/ik/target/right_pose";
inline constexpr char kLeftEndEffectorPose[] =
  "/mc/fk/pose/left_end_effector";
inline constexpr char kRightEndEffectorPose[] =
  "/mc/fk/pose/right_end_effector";

inline constexpr std::array<const char *, 5> kRequiredChannels{
  kJointStates,
  kLeftTargetPose,
  kRightTargetPose,
  kLeftEndEffectorPose,
  kRightEndEffectorPose};

}  // namespace motion_control_lab::contracts::foxglove_ik_v1
