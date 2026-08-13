#include "dual_arm_ik_app.hpp"
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

#include <cmath>
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

int runImpl(int argc, char ** argv, const mcl::DualArmIkAppConfig & app_config)
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

  const auto solver_setup = mcl::makeDualArmIkSolverSetup(
    app_config.solve_mode, options.rate_hz);

  mcc::KinematicsSolverBuilder builder;
  mcl::requireOk(
    builder.configure(model, joint_names, solver_setup.solver_config),
    "Failed to configure IK builder");

  mcc::PositionTaskConfig left_position_config;
  left_position_config.name = "left-position";
  left_position_config.enforcement = mcc::HardEnforcement{};
  mcc::PositionTaskHandle left_position_task;
  mcl::requireOk(
    builder.addPositionTask(
      robot.left_end_effector_frame,
      left_position_config,
      left_position_task),
    "Failed to register left position task");

  mcc::OrientationTaskConfig left_orientation_config;
  left_orientation_config.name = "left-orientation";
  left_orientation_config.enforcement = mcc::HardEnforcement{};
  mcc::OrientationTaskHandle left_orientation_task;
  mcl::requireOk(
    builder.addOrientationTask(
      robot.left_end_effector_frame,
      left_orientation_config,
      left_orientation_task),
    "Failed to register left orientation task");

  mcc::PositionTaskConfig right_position_config;
  right_position_config.name = "right-position";
  right_position_config.enforcement = mcc::HardEnforcement{};
  mcc::PositionTaskHandle right_position_task;
  mcl::requireOk(
    builder.addPositionTask(
      robot.right_end_effector_frame,
      right_position_config,
      right_position_task),
    "Failed to register right position task");

  mcc::OrientationTaskConfig right_orientation_config;
  right_orientation_config.name = "right-orientation";
  right_orientation_config.enforcement = mcc::HardEnforcement{};
  mcc::OrientationTaskHandle right_orientation_task;
  mcl::requireOk(
    builder.addOrientationTask(
      robot.right_end_effector_frame,
      right_orientation_config,
      right_orientation_task),
    "Failed to register right orientation task");

  mcc::JointPositionLimitConfig joint_limit_config;
  joint_limit_config.margin = 1e-3;
  joint_limit_config.enforcement = mcc::HardEnforcement{};
  mcc::JointPositionLimitHandle joint_limits;
  mcl::requireOk(
    builder.addJointPositionLimits(joint_limit_config, joint_limits),
    "Failed to register joint-position limits");

  if (solver_setup.register_joint_velocity_limits) {
    mcc::JointVelocityLimitConfig velocity_limit_config;
    velocity_limit_config.enforcement = mcc::HardEnforcement{};
    mcc::JointVelocityLimitHandle velocity_limits;
    mcl::requireOk(
      builder.addJointVelocityLimits(velocity_limit_config, velocity_limits),
      "Failed to register joint-velocity limits");
  }

  mcc::KinematicsSolver solver;
  mcl::requireOk(builder.finalize(solver), "Failed to finalize IK solver");

  auto currentTargetPose = [&](mcl::ArmSide side) {
    mcc::ForwardKinematicsRequest request;
    request.state = mcl::makeRobotState(positions, velocities);
    request.frame_names = {mcl::frameForSide(robot, side)};
    request.reference_frame_name = robot.base_frame;
    mcc::ForwardKinematicsSolution solution;
    mcc::ForwardKinematicsDiagnostics diagnostics;
    mcl::requireOk(
      solver.computeForwardKinematics(request, solution, diagnostics),
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
    app_config.title,
    presentation,
    {
      {mcl::ArmSide::Left, initial_left_fk},
      {mcl::ArmSide::Right, initial_right_fk}
    },
    true);
  auto visualization_sink = mcl::createVisualizationSink(
    options.visualization, app_config.program_id);

  mcl::installInteractiveSignalHandlers();
  mcl::InteractiveScheduler scheduler({options.rate_hz, options.duration_s});
  std::size_t publish_count = 0;

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
    visualization_sink->open({"interactive-preview", app_config.program_id});
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

        mcc::InverseKinematicsRequest request;
        request.reference_frame_name = robot.base_frame;
        request.state = mcl::makeRobotState(positions, velocities);
        request.position_targets.push_back(
          {left_position_task, left_target.target_pose.translation(), true});
        request.orientation_targets.push_back(
          {left_orientation_task, left_target.target_pose.linear(), true});
        request.position_targets.push_back(
          {right_position_task, right_target.target_pose.translation(), true});
        request.orientation_targets.push_back(
          {right_orientation_task, right_target.target_pose.linear(), true});

        mcc::InverseKinematicsSolution solution;
        mcc::InverseKinematicsDiagnostics diagnostics;
        const auto status = solver.solveInverseKinematics(request, solution, diagnostics);
        if (!status.ok() || !mcc::isAccepted(solution.disposition)) {
          throw std::runtime_error(
                  "IK rejected candidate: " +
                  (status.message.empty() ? "solver rejected candidate" : status.message));
        }
        if (solution.joint_positions.size() ==
            static_cast<Eigen::Index>(joint_names.size())) {
          positions = mcl::toStdVector(solution.joint_positions);
          if (solution.joint_velocities.size() ==
              static_cast<Eigen::Index>(joint_names.size())) {
            velocities = mcl::toStdVector(solution.joint_velocities);
          } else {
            // TargetSolve intentionally has no time semantics and returns no velocity vector.
            velocities.assign(joint_names.size(), 0.0);
          }
        }

        latest_frame.targets = command.targets;
        latest_frame.forward_kinematics = {
          {mcl::ArmSide::Left, mcl::requirePose(
              solution.solved_poses,
              robot.left_end_effector_frame).pose},
          {mcl::ArmSide::Right, mcl::requirePose(
              solution.solved_poses,
              robot.right_end_effector_frame).pose}};
        latest_frame.joint_names = joint_names;
        latest_frame.positions = positions;
        latest_frame.velocities = velocities;
        latest_frame.ik_status = status.ok() ? "ok" : status.message;
        latest_frame.iterations = diagnostics.iterations;
        latest_frame.converged = diagnostics.converged;
        latest_frame.solve_time_ms = diagnostics.solve_time_ms;
        latest_frame.target_errors.clear();

        mcl::ArmTargetError left_error;
        left_error.side = mcl::ArmSide::Left;
        bool has_left_error = false;
        mcl::ArmTargetError right_error;
        right_error.side = mcl::ArmSide::Right;
        bool has_right_error = false;
        for (const auto & error : diagnostics.position_errors) {
          if (error.handle.value == left_position_task.value) {
            left_error.position_m = error.norm_m;
            has_left_error = true;
          } else if (error.handle.value == right_position_task.value) {
            right_error.position_m = error.norm_m;
            has_right_error = true;
          }
        }
        for (const auto & error : diagnostics.orientation_errors) {
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

namespace motion_control_lab
{
DualArmIkSolverSetup makeDualArmIkSolverSetup(
  motion_control::core::IkSolveMode solve_mode,
  double rate_hz)
{
  namespace mcc = motion_control::core;
  if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
    throw std::runtime_error("rate must be a positive finite value");
  }
  if (solve_mode != mcc::IkSolveMode::ServoStep &&
      solve_mode != mcc::IkSolveMode::TargetSolve) {
    throw std::runtime_error("unsupported dual-arm IK solve mode");
  }

  DualArmIkSolverSetup setup;
  auto & solver_config = setup.solver_config;
  solver_config.mode = solve_mode;
  solver_config.servo_period =
    solve_mode == mcc::IkSolveMode::ServoStep ? 1.0 / rate_hz : 0.0;
  solver_config.joint_limit_policy = mcc::KinematicsJointLimitPolicy::ExplicitRequirements;
  solver_config.qp.backend = mcc::QpBackend::ProxQp;
  solver_config.qp.regularization = 1.0e-4;
  solver_config.maximum_iterations =
    solve_mode == mcc::IkSolveMode::ServoStep ? 1 : 80;
  solver_config.soft_solve_time_budget_ms = 100.0;
  solver_config.position_tolerance_m = 1.0e-4;
  solver_config.orientation_tolerance_rad = 1.0e-4;
  solver_config.minimum_position_improvement_m = 1.0e-8;
  solver_config.minimum_orientation_improvement_rad = 1.0e-8;
  setup.register_joint_velocity_limits = solve_mode == mcc::IkSolveMode::ServoStep;
  return setup;
}

int runDualArmIkApp(
  int argc,
  char ** argv,
  const DualArmIkAppConfig & app_config)
{
  try {
    return runImpl(argc, argv, app_config);
  } catch (const std::exception & error) {
    std::cerr << app_config.program_id << ": " << error.what() << "\n";
    return EXIT_FAILURE;
  }
}

}  // namespace motion_control_lab
