#pragma once

#include "runtime/interactive_types.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace motion_control_lab
{

struct R1RobotConfig
{
  std::string base_frame;
  std::string left_end_effector_frame;
  std::string right_end_effector_frame;
  JointNames joint_names;
  std::vector<double> default_positions;
  std::vector<std::size_t> left_arm_joint_indices;
  std::vector<std::size_t> right_arm_joint_indices;
};

struct ArmVisualizationChannels
{
  std::string joint_states;
  std::string left_target;
  std::string right_target;
  std::string left_forward_kinematics;
  std::string right_forward_kinematics;
};

const R1RobotConfig & r1RobotConfig();

ArmVisualizationChannels foxgloveIkVisualizationChannels();

const std::string & frameForSide(
  const R1RobotConfig & robot,
  ArmSide side);

InteractiveIkPresentation makeArmPresentation(
  const R1RobotConfig & robot,
  const ArmVisualizationChannels & channels);

}  // namespace motion_control_lab
