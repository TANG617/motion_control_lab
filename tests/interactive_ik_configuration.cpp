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
#include "sinks/ik_visualization.hpp"

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
    right->target_channel != motion_control_lab::contracts::foxglove_ik_v1::kRightTargetPose ||
    right->forward_kinematics_channel !=
      motion_control_lab::contracts::foxglove_ik_v1::kRightEndEffectorPose ||
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
  motion_control_lab::SolverDebug solver_debug;
  solver_debug.ik_solve_time_percentiles = {3, 4096, 3, 0.1, 0.2, 0.3};
  solver_debug.run_counters = motion_control_lab::SolverRunCounters{4, 3, 1};
  debug_frame.solvers = {solver_debug};
  const auto visualization =
    motion_control_lab::makeIkVisualizationFrame(debug_frame, presentation, 1, 2, 3);
  auto diagnosticValue = [&](const std::string & name) {
    for (const auto & diagnostic : visualization.diagnostics) {
      if (diagnostic.name == name) {
        return diagnostic.value;
      }
    }
    return -1.0;
  };
  if (
    !visualization.joints.has_value() || visualization.run_id != "interactive-preview-placo" ||
    visualization.joints->channel != motion_control_lab::contracts::foxglove_ik_v1::kJointStates ||
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
    visualization.poses[1].pose.position_m[0] != 3.0 ||
    diagnosticValue("ik.solve_time.p90") != 0.1 || diagnosticValue("ik.solve_time.p95") != 0.2 ||
    diagnosticValue("ik.solve_time.p99") != 0.3 || diagnosticValue("ik.solve_attempts") != 4.0 ||
    diagnosticValue("ik.accepted") != 3.0 || diagnosticValue("ik.rejected") != 1.0) {
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
