#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <string>
#include <vector>

namespace motion_control_lab
{

inline constexpr const char * kMotionControlUrdfEnvironmentVariable = "MOTION_CONTROL_URDF";

struct R1RobotConfig
{
  std::string base_frame;
  std::string left_end_effector_frame;
  std::string right_end_effector_frame;
  std::string left_link4_frame;
  std::string right_link4_frame;
  Eigen::Isometry3d left_tcp_offset{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d right_tcp_offset{Eigen::Isometry3d::Identity()};
  std::vector<std::string> joint_names;
  std::vector<double> default_positions;
  std::vector<std::size_t> left_arm_joint_indices;
  std::vector<std::size_t> right_arm_joint_indices;
  /** @brief Curated symmetric effort magnitude in joint_names order; no rate limits implied. */
  std::vector<double> effort_limits;
};

const R1RobotConfig & r1RobotConfig();

}  // namespace motion_control_lab
