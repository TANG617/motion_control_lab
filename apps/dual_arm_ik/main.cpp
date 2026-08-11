#include "ik_app_utils.hpp"
#include "r1_robot_config.hpp"

#include "config/interactive_ik_options.hpp"
#include "runtime/interactive_scheduler.hpp"
#include "runtime/interactive_types.hpp"
#include "sinks/ik_visualization.hpp"
#include "sinks/visualization_sink_factory.hpp"
#include "teleop/tui_teleop_source.hpp"

#include <motion_control_core/motion_control_core.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

namespace mcc = motion_control::core;
namespace mcl = motion_control_lab;

constexpr const char * kProgramId = "mcl_dual_arm_ik";
constexpr const char * kTitle = "Motion Control Dual-arm IK";

int run(int argc, char ** argv)
{
  const auto options = mcl::parseInteractiveIkOptions(argc, argv);
  if (!std::filesystem::exists(options.urdf_path)) {
    throw std::runtime_error("URDF does not exist: " + options.urdf_path);
  }

  const auto & robot = mcl::r1RobotConfig();
  const auto & joint_names = robot.joint_names;
  auto positions = robot.default_positions;
  std::vector<double> velocities(positions.size(), 0.0);

  mcc::RobotModelDescription model_description;
  model_description.urdf_path = options.urdf_path;
  model_description.kinematics_reference_frame = robot.base_frame;
  model_description.joint_names = joint_names;

  std::shared_ptr<const mcc::RobotModel> model;
  mcl::requireOk(mcc::RobotModel::load(model_description, model), "Failed to load robot model");

  mcc::KinematicsSolverConfig solver_config;
  solver_config.mode = mcc::IkSolveMode::ServoStep;
  solver_config.servo_period = 1.0 / options.rate_hz;
  solver_config.joint_limit_policy = mcc::KinematicsJointLimitPolicy::ExplicitRequirements;
  solver_config.qp.backend = mcc::QpBackend::ProxQp;
  solver_config.qp.regularization = 1.0e-8;
  solver_config.maximum_iterations = 1;
  solver_config.soft_solve_time_budget_ms = 100.0;
  solver_config.position_tolerance_m = 1.0e-4;
  solver_config.orientation_tolerance_rad = 1.0e-4;
  solver_config.minimum_position_improvement_m = 1.0e-8;
  solver_config.minimum_orientation_improvement_rad = 1.0e-8;

  mcc::GroupedKinematicsSolverConfig grouped_config;
  grouped_config.profile = mcc::GroupedSolverProfile::RedOnly;
  grouped_config.red = solver_config;
  mcc::GroupedKinematicsSolverBuilder builder;
  mcl::requireOk(
    builder.configure(model, joint_names, grouped_config),
    "Failed to configure IK builder");

  mcc::PositionTaskConfig left_position_config;
  left_position_config.name = "left-position";
  left_position_config.enforcement = mcc::HardEnforcement{};
  mcc::GroupedPositionTaskHandle left_position_task;
  mcl::requireOk(
    builder.addPositionTask(
      mcc::SolverGroup::Red,
      robot.left_end_effector_frame,
      left_position_config,
      left_position_task),
    "Failed to register left position task");

  mcc::OrientationTaskConfig left_orientation_config;
  left_orientation_config.name = "left-orientation";
  left_orientation_config.enforcement = mcc::HardEnforcement{};
  mcc::GroupedOrientationTaskHandle left_orientation_task;
  mcl::requireOk(
    builder.addOrientationTask(
      mcc::SolverGroup::Red,
      robot.left_end_effector_frame,
      left_orientation_config,
      left_orientation_task),
    "Failed to register left orientation task");

  mcc::PositionTaskConfig right_position_config;
  right_position_config.name = "right-position";
  right_position_config.enforcement = mcc::HardEnforcement{};
  mcc::GroupedPositionTaskHandle right_position_task;
  mcl::requireOk(
    builder.addPositionTask(
      mcc::SolverGroup::Red,
      robot.right_end_effector_frame,
      right_position_config,
      right_position_task),
    "Failed to register right position task");

  mcc::OrientationTaskConfig right_orientation_config;
  right_orientation_config.name = "right-orientation";
  right_orientation_config.enforcement = mcc::HardEnforcement{};
  mcc::GroupedOrientationTaskHandle right_orientation_task;
  mcl::requireOk(
    builder.addOrientationTask(
      mcc::SolverGroup::Red,
      robot.right_end_effector_frame,
      right_orientation_config,
      right_orientation_task),
    "Failed to register right orientation task");

  mcc::JointPositionLimitConfig joint_limit_config;
  joint_limit_config.margin = 1e-3;
  joint_limit_config.enforcement = mcc::HardEnforcement{};
  mcc::GroupedJointPositionLimitHandle joint_limits;
  mcl::requireOk(
    builder.addJointPositionLimits(mcc::SolverGroup::Red, joint_limit_config, joint_limits),
    "Failed to register joint-position limits");

  mcc::JointVelocityLimitConfig velocity_limit_config;
  velocity_limit_config.enforcement = mcc::HardEnforcement{};
  mcc::GroupedJointVelocityLimitHandle velocity_limits;
  mcl::requireOk(
    builder.addJointVelocityLimits(
      mcc::SolverGroup::Red, velocity_limit_config, velocity_limits),
    "Failed to register joint-velocity limits");

  mcc::GroupedKinematicsSolver solver;
  mcl::requireOk(builder.finalize(solver), "Failed to finalize IK solver");
  mcl::requireOk(solver.beginRun(1), "Failed to begin grouped IK run");

  auto currentTargetPose = [&](mcl::ArmSide side) {
    mcc::ForwardKinematicsRequest request;
    request.state = mcl::makeRobotState(positions, velocities);
    request.frame_names = {mcl::frameForSide(robot, side)};
    request.reference_frame_name = robot.base_frame;
    mcc::ForwardKinematicsSolution solution;
    mcc::ForwardKinematicsDiagnostics diagnostics;
    mcl::requireOk(
      solver.computeForwardKinematics(
        mcc::SolverGroup::Red, request, solution, diagnostics),
      "FK failed");
    return mcl::requirePose(solution.poses, mcl::frameForSide(robot, side)).pose;
  };

  const auto presentation = mcl::makeArmPresentation(
    robot, mcl::foxgloveIkVisualizationChannels());
  const auto initial_left_fk = currentTargetPose(mcl::ArmSide::Left);
  const auto initial_right_fk = currentTargetPose(mcl::ArmSide::Right);
  mcl::TuiTeleopSource tui(
    options.tui,
    options.rate_hz,
    kTitle,
    presentation,
    {
      {mcl::ArmSide::Left, initial_left_fk},
      {mcl::ArmSide::Right, initial_right_fk}
    },
    true);
  auto visualization_sink = mcl::createVisualizationSink(options.visualization, kProgramId);

  mcl::installInteractiveSignalHandlers();
  mcl::InteractiveScheduler scheduler({options.rate_hz, options.duration_s});
  std::size_t publish_count = 0;
  std::uint64_t solve_sequence = 0;

  mcl::IkDebugFrame latest_frame;
  latest_frame.targets = tui.command().targets;
  latest_frame.forward_kinematics = {
    {mcl::ArmSide::Left, initial_left_fk},
    {mcl::ArmSide::Right, initial_right_fk}};
  latest_frame.joint_names = joint_names;
  latest_frame.positions = positions;
  latest_frame.velocities = velocities;
  latest_frame.selected_side = mcl::parseArmSide(options.tui.side);

  bool sink_open = false;
  try {
    visualization_sink->open({"interactive-preview", kProgramId});
    sink_open = true;

    while (const auto schedule = scheduler.next()) {
      tui.poll();
      if (const auto reset_side = tui.consumeResetRequest()) {
        try {
          tui.setTargetPose(
            *reset_side,
            currentTargetPose(*reset_side),
            std::string{"Reset "} + mcl::armSideName(*reset_side) +
            " target from current FK");
        } catch (const std::exception & error) {
          tui.setStatus("Reset failed: " + std::string{error.what()});
        }
      }

      const auto & command = tui.command();
      if (command.stop_requested) {
        break;
      }

      if (schedule->update_due && !command.paused) {
        const auto & left_target = mcl::requireTarget(command.targets, mcl::ArmSide::Left);
        const auto & right_target = mcl::requireTarget(command.targets, mcl::ArmSide::Right);

        mcc::GroupedInverseKinematicsRequest request;
        request.reference_frame_name = robot.base_frame;
        request.captured_state.state = mcl::makeRobotState(positions, velocities);
        request.captured_state.sequence = ++solve_sequence;
        request.captured_state.monotonic_time_nanoseconds =
          std::max<std::int64_t>(1, schedule->sample_time_ns);
        request.position_targets.push_back(
          {left_position_task, left_target.target_pose.translation(), true});
        request.orientation_targets.push_back(
          {left_orientation_task, left_target.target_pose.linear(), true});
        request.position_targets.push_back(
          {right_position_task, right_target.target_pose.translation(), true});
        request.orientation_targets.push_back(
          {right_orientation_task, right_target.target_pose.linear(), true});

        mcc::GroupedInverseKinematicsSolution solution;
        mcc::GroupedInverseKinematicsDiagnostics diagnostics;
        const auto status = solver.solveInverseKinematics(
          mcc::SolverGroup::Red, request, solution, diagnostics);
        if (!status.ok() || !diagnostics.attempt_accepted) {
          throw std::runtime_error(
                  "Red IK rejected attempt " + std::to_string(diagnostics.attempt_revision) +
                  ": " + (status.message.empty() ? "solver failure" : status.message));
        }
        if (solution.kinematics_solution.joint_positions.size() ==
            static_cast<Eigen::Index>(joint_names.size())) {
          positions = mcl::toStdVector(solution.kinematics_solution.joint_positions);
          if (solution.kinematics_solution.joint_velocities.size() ==
              static_cast<Eigen::Index>(joint_names.size())) {
            velocities = mcl::toStdVector(solution.kinematics_solution.joint_velocities);
          } else {
            velocities.assign(joint_names.size(), 0.0);
          }
        }

        latest_frame.targets = command.targets;
        latest_frame.forward_kinematics = {
          {mcl::ArmSide::Left, mcl::requirePose(
              solution.kinematics_solution.solved_poses,
              robot.left_end_effector_frame).pose},
          {mcl::ArmSide::Right, mcl::requirePose(
              solution.kinematics_solution.solved_poses,
              robot.right_end_effector_frame).pose}};
        latest_frame.joint_names = joint_names;
        latest_frame.positions = positions;
        latest_frame.velocities = velocities;
        latest_frame.ik_status = status.ok() ? "ok" : status.message;
        latest_frame.iterations = diagnostics.kinematics.iterations;
        latest_frame.converged = diagnostics.kinematics.converged;
        latest_frame.solve_time_ms = diagnostics.kinematics.solve_time_ms;
        latest_frame.target_errors.clear();

        mcl::ArmTargetError left_error;
        left_error.side = mcl::ArmSide::Left;
        bool has_left_error = false;
        mcl::ArmTargetError right_error;
        right_error.side = mcl::ArmSide::Right;
        bool has_right_error = false;
        for (const auto & error : diagnostics.kinematics.position_errors) {
          if (error.handle.value == left_position_task.value) {
            left_error.position_m = error.norm_m;
            has_left_error = true;
          } else if (error.handle.value == right_position_task.value) {
            right_error.position_m = error.norm_m;
            has_right_error = true;
          }
        }
        for (const auto & error : diagnostics.kinematics.orientation_errors) {
          if (error.handle.value == left_orientation_task.value) {
            left_error.orientation_rad = error.norm_rad;
            has_left_error = true;
          } else if (error.handle.value == right_orientation_task.value) {
            right_error.orientation_rad = error.norm_rad;
            has_right_error = true;
          }
        }
        if (has_left_error) {
          latest_frame.target_errors.push_back(left_error);
        }
        if (has_right_error) {
          latest_frame.target_errors.push_back(right_error);
        }
        latest_frame.status = command.status;
        latest_frame.paused = command.paused;
        latest_frame.selected_side = command.selected_side;

        try {
          visualization_sink->write(mcl::makeIkVisualizationFrame(
            latest_frame,
            presentation,
            publish_count,
            schedule->sample_time_ns,
            schedule->emit_time_ns));
        } catch (const std::exception & error) {
          tui.setStatus(error.what());
          latest_frame.status = error.what();
        }
        ++publish_count;
      }

      if (schedule->draw_due) {
        latest_frame.status = tui.command().status;
        latest_frame.paused = tui.command().paused;
        latest_frame.selected_side = tui.command().selected_side;
        tui.render(latest_frame, publish_count, visualization_sink->status());
      }
      scheduler.sleep();
    }

    visualization_sink->flush();
    visualization_sink->close();
    sink_open = false;
  } catch (...) {
    if (sink_open) {
      try {
        visualization_sink->close();
      } catch (...) {
      }
    }
    throw;
  }

  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    return run(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << kProgramId << ": " << error.what() << "\n";
    return EXIT_FAILURE;
  }
}
