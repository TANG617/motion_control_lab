#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "contracts/visualization/foxglove_ik_v1.hpp"
#include "ik_app_utils.hpp"
#include "r1_interactive_config.hpp"
#include "r1_robot_config.hpp"
#include "runtime/interactive_scheduler.hpp"
#include "runtime/interactive_types.hpp"
#include "sinks/ik_render_batch.hpp"
#include "tests/visualization_contract_conformance.hpp"

int main()
{
  namespace mcc = motion_control::core;

  const auto & robot = motion_control_lab::r1RobotConfig();
  const auto presentation = motion_control_lab::makeArmPresentation(
    robot, motion_control_lab::foxgloveIkVisualizationChannels());
  const auto * right =
    motion_control_lab::findArmPresentation(presentation, motion_control_lab::ArmSide::Right);
  const auto initial_positions = motion_control_lab::toEigen(robot.default_positions);
  if (
    robot.base_frame != "base_link" || robot.joint_names.size() != 20 ||
    robot.default_positions.size() != robot.joint_names.size() ||
    motion_control_lab::frameForSide(robot, motion_control_lab::ArmSide::Left) !=
      "left_arm_ee_link" ||
    right == nullptr ||
    right->target_channel !=
      motion_control_lab::contracts::foxglove_ik_v1::kRightInputTargetTopic ||
    right->forward_kinematics_channel !=
      motion_control_lab::contracts::foxglove_ik_v1::kRightFkOutputTopic ||
    right->joint_indices != robot.right_arm_joint_indices ||
    initial_positions.size() != static_cast<Eigen::Index>(robot.joint_names.size())) {
    return EXIT_FAILURE;
  }

  motion_control_lab::IkDebugFrame debug_frame;
  debug_frame.run_id = "interactive-preview-placo";
  debug_frame.targets = {
    {motion_control_lab::ArmSide::Left, motion_control_lab::Pose::Identity()},
    {motion_control_lab::ArmSide::Right, motion_control_lab::Pose::Identity()}};
  debug_frame.targets[0].target_pose.translation().x() = 1.0;
  debug_frame.targets[1].target_pose.translation().x() = 2.0;
  debug_frame.forward_kinematics = {
    {motion_control_lab::ArmSide::Left, motion_control_lab::Pose::Identity()},
    {motion_control_lab::ArmSide::Right, motion_control_lab::Pose::Identity()}};
  debug_frame.forward_kinematics[0].pose.translation().x() = 3.0;
  debug_frame.forward_kinematics[1].pose.translation().x() = 4.0;
  debug_frame.joint_names = robot.joint_names;
  debug_frame.positions = robot.default_positions;
  debug_frame.velocities.assign(robot.joint_names.size(), 0.0);
  const auto visualization =
    motion_control_lab::makeIkRenderBatch(debug_frame, presentation, 3);
  if (
    visualization.timestamp_ns != 3 || visualization.joint_states.size() != 1U ||
    visualization.joint_states.front().channel !=
      motion_control_lab::contracts::foxglove_ik_v1::kIkOutputJointStateTopic ||
    visualization.poses.size() != 4 ||
    visualization.poses[0].channel !=
      motion_control_lab::contracts::foxglove_ik_v1::kLeftInputTargetTopic ||
    visualization.poses[1].channel !=
      motion_control_lab::contracts::foxglove_ik_v1::kLeftFkOutputTopic ||
    visualization.poses[2].channel !=
      motion_control_lab::contracts::foxglove_ik_v1::kRightInputTargetTopic ||
    visualization.poses[3].channel !=
      motion_control_lab::contracts::foxglove_ik_v1::kRightFkOutputTopic ||
    visualization.poses[0].pose.position_m[0] != 1.0 ||
    visualization.poses[1].pose.position_m[0] != 3.0 ||
    !motion_control_lab::tests::requiredChannelsPresent(
      visualization, motion_control_lab::contracts::foxglove_ik_v1::kChannels)) {
    return EXIT_FAILURE;
  }

  motion_control_lab::installInteractiveSignalHandlers();
  motion_control_lab::InteractiveScheduler scheduler({50.0, 1.0});
  const auto first_schedule = scheduler.next();
  if (
    !first_schedule || !first_schedule->update_due || !first_schedule->draw_due ||
    first_schedule->dt != 0.02) {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
