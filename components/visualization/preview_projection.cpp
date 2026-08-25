#include "components/visualization/preview_projection.hpp"

#include "contracts/visualization/mcl_state_v1.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>

namespace motion_control_lab
{

ArmVisualizationChannels foxgloveIkVisualizationChannels()
{
  namespace contract = contracts::mcl_state_v1;
  return {
    contract::kJointIkTopic,
    contract::kLeftCartesianInputTopic,
    contract::kRightCartesianInputTopic,
    contract::kLeftCartesianGoalTopic,
    contract::kRightCartesianGoalTopic,
    contract::kLeftCartesianIkTopic,
    contract::kRightCartesianIkTopic};
}

const std::string & frameForSide(const R1RobotConfig & robot, ArmSide side)
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
    {ArmSide::Left, channels.left_input, channels.left_goal, channels.left_ik,
      robot.left_arm_joint_indices},
    {ArmSide::Right, channels.right_input, channels.right_goal, channels.right_ik,
      robot.right_arm_joint_indices}};
  return presentation;
}

motion_control::viz::RenderBatch makeIkRenderBatch(
  const IkDebugFrame & frame,
  const InteractiveIkPresentation & presentation,
  std::uint64_t timestamp_ns)
{
  namespace mcv = motion_control::viz;
  mcv::RenderBatch batch;
  batch.timestamp_ns = timestamp_ns;
  batch.poses.reserve(presentation.arms.size() * 3U);
  for (const auto & arm : presentation.arms) {
    const auto input = std::find_if(
      frame.input_targets.begin(), frame.input_targets.end(),
      [&arm](const ArmTarget & value) { return value.side == arm.side; });
    if (input == frame.input_targets.end()) {
      throw std::runtime_error(
        std::string{"render batch is missing the "} + armSideName(arm.side) + " input");
    }
    const Eigen::Quaterniond input_orientation(input->target_pose.linear());
    mcv::PoseSample input_pose;
    input_pose.channel = arm.input_channel;
    input_pose.frame_id = presentation.base_frame_id;
    input_pose.pose.position_m = {
      input->target_pose.translation().x(), input->target_pose.translation().y(),
      input->target_pose.translation().z()};
    input_pose.pose.orientation_xyzw = {
      input_orientation.x(), input_orientation.y(), input_orientation.z(), input_orientation.w()};
    batch.poses.push_back(std::move(input_pose));

    const auto target = std::find_if(
      frame.targets.begin(), frame.targets.end(),
      [&arm](const ArmTarget & value) { return value.side == arm.side; });
    if (target == frame.targets.end()) {
      throw std::runtime_error(
        std::string{"render batch is missing the "} + armSideName(arm.side) + " target");
    }
    const Eigen::Quaterniond orientation(target->target_pose.linear());
    mcv::PoseSample pose;
    pose.channel = arm.goal_channel;
    pose.frame_id = presentation.base_frame_id;
    pose.pose.position_m = {
      target->target_pose.translation().x(), target->target_pose.translation().y(),
      target->target_pose.translation().z()};
    pose.pose.orientation_xyzw = {
      orientation.x(), orientation.y(), orientation.z(), orientation.w()};
    batch.poses.push_back(std::move(pose));

    const auto fk = std::find_if(
      frame.forward_kinematics.begin(), frame.forward_kinematics.end(),
      [&arm](const ArmForwardKinematics & value) { return value.side == arm.side; });
    if (fk == frame.forward_kinematics.end()) {
      throw std::runtime_error(
        std::string{"render batch is missing the "} + armSideName(arm.side) + " FK output");
    }
    const Eigen::Quaterniond fk_orientation(fk->pose.linear());
    mcv::PoseSample fk_pose;
    fk_pose.channel = arm.ik_channel;
    fk_pose.frame_id = presentation.base_frame_id;
    fk_pose.pose.position_m = {
      fk->pose.translation().x(), fk->pose.translation().y(), fk->pose.translation().z()};
    fk_pose.pose.orientation_xyzw = {
      fk_orientation.x(), fk_orientation.y(), fk_orientation.z(), fk_orientation.w()};
    batch.poses.push_back(std::move(fk_pose));
  }
  batch.joint_states.push_back(mcv::JointStateSample{
    presentation.joint_state_channel, frame.joint_names, frame.positions, frame.velocities});
  return batch;
}

}  // namespace motion_control_lab
