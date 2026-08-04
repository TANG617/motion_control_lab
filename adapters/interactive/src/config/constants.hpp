#pragma once

#include <array>

namespace motion_control_lab
{

inline constexpr const char * kR1UrdfEnvironmentVariable = "MOTION_CONTROL_R1_URDF";
inline constexpr const char * kBaseFrame = "base_link";
inline constexpr const char * kLeftTargetFrame = "left_arm_ee_link";
inline constexpr const char * kRightTargetFrame = "right_arm_ee_link";
inline constexpr const char * kLeftTargetPoseTopic = "/mc/ik/target/left_pose";
inline constexpr const char * kRightTargetPoseTopic = "/mc/ik/target/right_pose";
inline constexpr const char * kJointStatesTopic = "/mc/ik/joint_states";
inline constexpr double kStepScale = 2.0;
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr std::array<const char *, 3> kRotationAxes{"x", "y", "z"};

}  // namespace motion_control_lab
