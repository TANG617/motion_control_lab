#pragma once

#include "r1_robot_config.hpp"
#include "runtime/interactive_types.hpp"

#include <string>

namespace motion_control_lab
{

struct ArmVisualizationChannels
{
  std::string joint_states;
  std::string left_target;
  std::string right_target;
  std::string left_forward_kinematics;
  std::string right_forward_kinematics;
};

ArmVisualizationChannels foxgloveIkVisualizationChannels();

const std::string & frameForSide(
  const R1RobotConfig & robot,
  ArmSide side);

InteractiveIkPresentation makeArmPresentation(
  const R1RobotConfig & robot,
  const ArmVisualizationChannels & channels);

}  // namespace motion_control_lab
