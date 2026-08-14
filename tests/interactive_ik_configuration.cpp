#include "ik_app_utils.hpp"
#include "r1_interactive_config.hpp"
#include "r1_robot_config.hpp"

#include "config/interactive_ik_options.hpp"
#include "contracts/visualization/foxglove_ik_v1.hpp"
#include "runtime/interactive_scheduler.hpp"
#include "runtime/interactive_types.hpp"
#include "sinks/ik_visualization.hpp"

#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>

int main()
{
  namespace mcc = motion_control::core;

  char program[] = "mcl_single_arm_ik";
  char urdf_option[] = "--urdf";
  char urdf_value[] = "/tmp/robot.urdf";
  char side_option[] = "--side";
  char side_value[] = "right";
  char rate_option[] = "--rate";
  char rate_value[] = "50";
  char * argv[]{
    program,
    urdf_option,
    urdf_value,
    side_option,
    side_value,
    rate_option,
    rate_value};
  const auto options = motion_control_lab::parseInteractiveIkOptions(7, argv);
  if (options.urdf_path != urdf_value ||
      options.tui.side != "right" ||
      options.rate_hz != 50.0 ||
      motion_control_lab::parseArmSide(options.tui.side) !=
      motion_control_lab::ArmSide::Right) {
    return EXIT_FAILURE;
  }

  char grouped_program[] = "mcl_grouped_dual_arm_ik";
  char grouped_urdf_option[] = "--urdf";
  char grouped_urdf_value[] = "/tmp/robot.urdf";
  char * grouped_argv[]{grouped_program, grouped_urdf_option, grouped_urdf_value};
  const auto grouped = motion_control_lab::parseGroupedInteractiveIkOptions(3, grouped_argv);
  if (grouped.red_rate_hz != 1000.0 || grouped.yellow_rate_hz != 100.0 ||
      grouped.ui_rate_hz != 100.0 ||
      grouped.deadline_policy != motion_control_lab::DeadlinePolicy::Strict) {
    return EXIT_FAILURE;
  }

  char deadline_policy_option[] = "--deadline-policy";
  char deadline_policy_value[] = "monitor";
  char * monitor_grouped_argv[]{
    grouped_program,
    grouped_urdf_option,
    grouped_urdf_value,
    deadline_policy_option,
    deadline_policy_value};
  const auto monitor = motion_control_lab::parseGroupedInteractiveIkOptions(
    5, monitor_grouped_argv);
  if (monitor.deadline_policy != motion_control_lab::DeadlinePolicy::Monitor) {
    return EXIT_FAILURE;
  }

  char invalid_deadline_policy_value[] = "ignore";
  char * invalid_policy_argv[]{
    grouped_program,
    grouped_urdf_option,
    grouped_urdf_value,
    deadline_policy_option,
    invalid_deadline_policy_value};
  try {
    (void)motion_control_lab::parseGroupedInteractiveIkOptions(5, invalid_policy_argv);
    return EXIT_FAILURE;
  } catch (const std::runtime_error &) {
  }

  char red_option[] = "--red-rate";
  char yellow_option[] = "--yellow-rate";
  char valid_red_value[] = "800";
  char valid_yellow_value[] = "80";
  char * custom_grouped_argv[]{
    grouped_program,
    grouped_urdf_option,
    grouped_urdf_value,
    red_option,
    valid_red_value,
    yellow_option,
    valid_yellow_value};
  const auto custom = motion_control_lab::parseGroupedInteractiveIkOptions(
    7, custom_grouped_argv);
  if (custom.red_rate_hz != 800.0 || custom.yellow_rate_hz != 80.0) {
    return EXIT_FAILURE;
  }

  char green_option[] = "--green-rate";
  char green_value[] = "10";
  char * removed_green_argv[]{
    grouped_program,
    grouped_urdf_option,
    grouped_urdf_value,
    green_option,
    green_value};
  try {
    (void)motion_control_lab::parseGroupedInteractiveIkOptions(5, removed_green_argv);
    return EXIT_FAILURE;
  } catch (const std::runtime_error &) {
  }

  char red_value[] = "100";
  char yellow_value[] = "100";
  char * invalid_grouped_argv[]{
    grouped_program,
    grouped_urdf_option,
    grouped_urdf_value,
    red_option,
    red_value,
    yellow_option,
    yellow_value};
  try {
    (void)motion_control_lab::parseGroupedInteractiveIkOptions(7, invalid_grouped_argv);
    return EXIT_FAILURE;
  } catch (const std::runtime_error &) {
  }

  const auto & robot = motion_control_lab::r1RobotConfig();
  const auto presentation = motion_control_lab::makeArmPresentation(
    robot, motion_control_lab::foxgloveIkVisualizationChannels());
  const auto * right = motion_control_lab::findArmPresentation(
    presentation, motion_control_lab::ArmSide::Right);
  const auto initial_positions = motion_control_lab::toEigen(robot.default_positions);
  if (robot.base_frame != "base_link" || robot.joint_names.size() != 20 ||
      robot.default_positions.size() != robot.joint_names.size() ||
      motion_control_lab::frameForSide(robot, motion_control_lab::ArmSide::Left) !=
      "left_arm_ee_link" ||
      right == nullptr ||
      right->target_channel !=
      motion_control_lab::contracts::foxglove_ik_v1::kRightTargetPose ||
      right->forward_kinematics_channel !=
      motion_control_lab::contracts::foxglove_ik_v1::kRightEndEffectorPose ||
      right->joint_indices != robot.right_arm_joint_indices ||
      initial_positions.size() != static_cast<Eigen::Index>(robot.joint_names.size())) {
    return EXIT_FAILURE;
  }

  motion_control_lab::IkDebugFrame debug_frame;
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
  const auto visualization = motion_control_lab::makeIkVisualizationFrame(
    debug_frame, presentation, 1, 2, 3);
  if (!visualization.joints.has_value() ||
      visualization.joints->channel !=
      motion_control_lab::contracts::foxglove_ik_v1::kJointStates ||
      visualization.poses.size() != 4 ||
      visualization.poses[0].channel !=
      motion_control_lab::contracts::foxglove_ik_v1::kLeftTargetPose ||
      visualization.poses[1].channel !=
      motion_control_lab::contracts::foxglove_ik_v1::kLeftEndEffectorPose ||
      visualization.poses[2].channel !=
      motion_control_lab::contracts::foxglove_ik_v1::kRightTargetPose ||
      visualization.poses[3].channel !=
      motion_control_lab::contracts::foxglove_ik_v1::kRightEndEffectorPose ||
      visualization.poses[0].pose.position_m[0] != 1.0 ||
      visualization.poses[1].pose.position_m[0] != 3.0) {
    return EXIT_FAILURE;
  }

  motion_control_lab::installInteractiveSignalHandlers();
  motion_control_lab::InteractiveScheduler scheduler({50.0, 1.0});
  const auto first_schedule = scheduler.next();
  if (!first_schedule || !first_schedule->update_due ||
      !first_schedule->draw_due || first_schedule->dt != 0.02) {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
