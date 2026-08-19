#include "r1_interactive_config.hpp"

#include "contracts/visualization/foxglove_ik_v1.hpp"

namespace motion_control_lab
{

ArmVisualizationChannels foxgloveIkVisualizationChannels()
{
  namespace contract = contracts::foxglove_ik_v1;
  return {
    contract::kIkOutputJointStateTopic,
    contract::kLeftInputTargetTopic,
    contract::kRightInputTargetTopic,
    contract::kLeftFkOutputTopic,
    contract::kRightFkOutputTopic};
}

const std::string & frameForSide(
  const R1RobotConfig & robot,
  ArmSide side)
{
  return side == ArmSide::Left
    ? robot.left_end_effector_frame
    : robot.right_end_effector_frame;
}

InteractiveIkPresentation makeArmPresentation(
  const R1RobotConfig & robot,
  const ArmVisualizationChannels & channels)
{
  InteractiveIkPresentation presentation;
  presentation.base_frame_id = robot.base_frame;
  presentation.joint_state_channel = channels.joint_states;
  presentation.arms = {
    {ArmSide::Left, channels.left_target, channels.left_forward_kinematics,
      robot.left_arm_joint_indices},
    {ArmSide::Right, channels.right_target, channels.right_forward_kinematics,
      robot.right_arm_joint_indices}};
  return presentation;
}

}  // namespace motion_control_lab
