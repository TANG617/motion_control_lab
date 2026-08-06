#include "ik_app_utils.hpp"
#include "r1_robot_config.hpp"

#include "config/interactive_ik_options.hpp"
#include "runtime/interactive_scheduler.hpp"
#include "runtime/interactive_types.hpp"

#include <cstdlib>
#include <stdexcept>
#include <string>

int main()
{
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
      grouped.green_rate_hz != 10.0 || grouped.ui_rate_hz != 20.0) {
    return EXIT_FAILURE;
  }

  char red_option[] = "--red-rate";
  char red_value[] = "100";
  char yellow_option[] = "--yellow-rate";
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
    robot,
    {"/joint_states", "/left_target", "/right_target"});
  const auto * right = motion_control_lab::findArmPresentation(
    presentation, motion_control_lab::ArmSide::Right);
  const auto initial_positions = motion_control_lab::toEigen(robot.default_positions);
  if (robot.base_frame != "base_link" || robot.joint_names.size() != 20 ||
      robot.default_positions.size() != robot.joint_names.size() ||
      motion_control_lab::frameForSide(robot, motion_control_lab::ArmSide::Left) !=
      "left_arm_ee_link" ||
      right == nullptr || right->target_channel != "/right_target" ||
      right->joint_indices != robot.right_arm_joint_indices ||
      initial_positions.size() != static_cast<Eigen::Index>(robot.joint_names.size())) {
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
