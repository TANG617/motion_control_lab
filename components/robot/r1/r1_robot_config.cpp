#include "components/robot/r1/r1_robot_config.hpp"

namespace motion_control_lab
{
namespace
{

Eigen::Isometry3d makeTcpOffset()
{
  auto offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d{0.0, 0.0, 0.1};
  return offset;
}

}  // namespace

const R1RobotConfig & r1RobotConfig()
{
  static const R1RobotConfig config{
    "base_link",
    "left_arm_ee_link",
    "right_arm_ee_link",
    "left_arm_link4",
    "right_arm_link4",
    makeTcpOffset(),
    makeTcpOffset(),
    {
      "head_yaw_joint", "head_pitch_joint", "torso_yaw_joint", "torso_pitch_joint",
      "knee_pitch_joint", "ankle_pitch_joint", "left_arm_joint1", "left_arm_joint2",
      "left_arm_joint3", "left_arm_joint4", "left_arm_joint5", "left_arm_joint6",
      "left_arm_joint7", "right_arm_joint1", "right_arm_joint2", "right_arm_joint3",
      "right_arm_joint4", "right_arm_joint5", "right_arm_joint6", "right_arm_joint7",
    },
    {
      0.0, 0.31, 0.0, 0.5, 0.5, -0.5, 0.9, -1.38, -1.57, -1.4,
      -0.45, 0.0, 0.0, -0.9, 1.38, 1.57, 1.4, 0.45, 0.0, 0.0,
    },
    {6, 7, 8, 9, 10, 11, 12},
    {13, 14, 15, 16, 17, 18, 19},
    {
      16.0, 16.0, 450.0, 450.0, 600.0, 600.0, 108.0, 108.0, 66.0, 66.0,
      19.0, 19.0, 19.0, 108.0, 108.0, 66.0, 66.0, 19.0, 19.0, 19.0,
    }};
  return config;
}

}  // namespace motion_control_lab
