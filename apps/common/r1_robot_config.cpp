#include "r1_robot_config.hpp"

#include "contracts/visualization/foxglove_ik_v1.hpp"

namespace motion_control_lab
{

const R1RobotConfig & r1RobotConfig()
{
  static const R1RobotConfig config{
    "base_link",
    "left_arm_ee_link",
    "right_arm_ee_link",
    {
      "head_yaw_joint",
      "head_pitch_joint",
      "torso_yaw_joint",
      "torso_pitch_joint",
      "knee_pitch_joint",
      "ankle_pitch_joint",
      "left_arm_joint1",
      "left_arm_joint2",
      "left_arm_joint3",
      "left_arm_joint4",
      "left_arm_joint5",
      "left_arm_joint6",
      "left_arm_joint7",
      "right_arm_joint1",
      "right_arm_joint2",
      "right_arm_joint3",
      "right_arm_joint4",
      "right_arm_joint5",
      "right_arm_joint6",
      "right_arm_joint7"
    },
    {
      0.0, 0.31, 0.0, 0.5, 0.5, -0.5,
      0.9, -1.38, -1.57, -1.4, -0.45, 0.0, 0.0,
      -0.9, 1.38, 1.57, 1.4, 0.45, 0.0, 0.0
    },
    {6, 7, 8, 9, 10, 11, 12},
    {13, 14, 15, 16, 17, 18, 19}};
  return config;
}

ArmVisualizationChannels foxgloveIkVisualizationChannels()
{
  namespace contract = contracts::foxglove_ik_v1;
  return {
    contract::kJointStates,
    contract::kLeftTargetPose,
    contract::kRightTargetPose,
    contract::kLeftEndEffectorPose,
    contract::kRightEndEffectorPose};
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
