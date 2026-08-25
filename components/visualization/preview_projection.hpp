#pragma once

#include "components/robot/r1/r1_robot_config.hpp"
#include "contracts/presentation/ik_app_snapshot.hpp"

#include <motion_control_viz/render_batch.hpp>

#include <cstdint>
#include <string>

namespace motion_control_lab
{

struct ArmVisualizationChannels
{
  std::string joint_states;
  std::string left_input;
  std::string right_input;
  std::string left_goal;
  std::string right_goal;
  std::string left_ik;
  std::string right_ik;
};

ArmVisualizationChannels foxgloveIkVisualizationChannels();
const std::string & frameForSide(const R1RobotConfig & robot, ArmSide side);
InteractiveIkPresentation makeArmPresentation(
  const R1RobotConfig & robot,
  const ArmVisualizationChannels & channels);
motion_control::viz::RenderBatch makeIkRenderBatch(
  const IkDebugFrame & frame,
  const InteractiveIkPresentation & presentation,
  std::uint64_t timestamp_ns);

}  // namespace motion_control_lab
