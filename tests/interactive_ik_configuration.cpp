#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "contracts/visualization/mcl_state_v1.hpp"
#include "components/app_helpers/app_helpers.hpp"
#include "components/robot/r1/r1_robot_config.hpp"
#include "components/scheduler/single_rate_scheduler.hpp"
#include "components/visualization/preview_projection.hpp"
#include "contracts/presentation/ik_app_snapshot.hpp"
#include "tests/visualization_contract_conformance.hpp"

int main()
{
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
    right->input_channel !=
      motion_control_lab::contracts::mcl_state_v1::kRightCartesianInputTopic ||
    right->goal_channel !=
      motion_control_lab::contracts::mcl_state_v1::kRightCartesianGoalTopic ||
    right->ik_channel !=
      motion_control_lab::contracts::mcl_state_v1::kRightCartesianIkTopic ||
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
  debug_frame.input_targets = debug_frame.targets;
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
      motion_control_lab::contracts::mcl_state_v1::kJointIkTopic ||
    visualization.poses.size() != 6 ||
    visualization.poses[0].channel !=
      motion_control_lab::contracts::mcl_state_v1::kLeftCartesianInputTopic ||
    visualization.poses[1].channel !=
      motion_control_lab::contracts::mcl_state_v1::kLeftCartesianGoalTopic ||
    visualization.poses[2].channel !=
      motion_control_lab::contracts::mcl_state_v1::kLeftCartesianIkTopic ||
    visualization.poses[3].channel !=
      motion_control_lab::contracts::mcl_state_v1::kRightCartesianInputTopic ||
    visualization.poses[4].channel !=
      motion_control_lab::contracts::mcl_state_v1::kRightCartesianGoalTopic ||
    visualization.poses[5].channel !=
      motion_control_lab::contracts::mcl_state_v1::kRightCartesianIkTopic ||
    visualization.poses[0].pose.position_m[0] != 1.0 ||
    visualization.poses[2].pose.position_m[0] != 3.0 ||
    !motion_control_lab::tests::requiredChannelsPresent(
      visualization, motion_control_lab::contracts::mcl_state_v1::kChannels)) {
    return EXIT_FAILURE;
  }

  motion_control_lab::installRuntimeSignalHandlers();
  motion_control_lab::SingleRateScheduler scheduler({50.0, 1.0});
  const auto first_schedule = scheduler.next();
  if (
    !first_schedule || !first_schedule->update_due || !first_schedule->draw_due ||
    first_schedule->dt != 0.02) {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
