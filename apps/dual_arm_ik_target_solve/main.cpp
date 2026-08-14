#include "ik_app_utils.hpp"
#include "r1_interactive_config.hpp"
#include "r1_robot_config.hpp"

#include "config/interactive_ik_options.hpp"
#include "runtime/interactive_scheduler.hpp"
#include "runtime/interactive_types.hpp"
#include "sinks/ik_visualization.hpp"
#include "sinks/visualization_sink_factory.hpp"
#include "teleop/tui_teleop_source.hpp"

#include <motion_control_core/motion_control_core.hpp>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

namespace mcc = motion_control::core;
namespace mcl = motion_control_lab;

constexpr const char * kProgramId = "mcl_dual_arm_ik_target_solve";
constexpr const char * kTitle = "Dual-arm IK — TargetSolve";

void throwIfError(const mcc::Status & status)
{
  if (!status.ok()) {
    throw std::runtime_error(status.message);
  }
}

const mcc::Pose & poseForFrame(
  const std::vector<mcc::FramePose> & poses,
  const std::string & frame_name)
{
  return std::find_if(
    poses.begin(), poses.end(),
    [&](const mcc::FramePose & pose) { return pose.frame_name == frame_name; })->pose;
}

int run(int argc, char ** argv)
{
  const auto options = mcl::parseInteractiveIkOptions(argc, argv);
  const auto & robot = mcl::r1RobotConfig();
  const auto & joint_names = robot.joint_names;
  auto positions = robot.default_positions;
  std::vector<double> velocities(positions.size(), 0.0);

  mcc::RobotModelDescription model_description;
  model_description.urdf_path = options.urdf_path;
  model_description.kinematics_reference_frame = robot.base_frame;
  model_description.joint_names = joint_names;

  std::shared_ptr<const mcc::RobotModel> model;
  throwIfError(mcc::RobotModel::load(model_description, model));

  mcc::KinematicsSolverConfig solver_config;
  solver_config.mode = mcc::IkSolveMode::TargetSolve;
  solver_config.servo_period = 0.0;
  solver_config.joint_limit_policy = mcc::KinematicsJointLimitPolicy::ExplicitRequirements;
  solver_config.qp.backend = mcc::QpBackend::ProxQp;
  solver_config.qp.regularization = 1.0e-4;
  solver_config.maximum_iterations = 10000;
  solver_config.soft_solve_time_budget_ms = 100.0;
  solver_config.position_tolerance_m = 1.0e-4;
  solver_config.orientation_tolerance_rad = 1.0e-4;
  solver_config.minimum_position_improvement_m = 1.0e-8;
  solver_config.minimum_orientation_improvement_rad = 1.0e-8;
  solver_config.qp.proxqp.absolute_tolerance = 1.0e-8;

  mcc::KinematicsSolverBuilder builder;
  throwIfError(builder.configure(model, joint_names, solver_config));

  mcc::PositionTaskConfig left_position_config;
  left_position_config.name = "left-position";
  left_position_config.enforcement = mcc::HardEnforcement{};
  mcc::PositionTaskHandle left_position_task;
  throwIfError(builder.addPositionTask(
    robot.left_end_effector_frame, left_position_config, left_position_task));

  mcc::OrientationTaskConfig left_orientation_config;
  left_orientation_config.name = "left-orientation";
  left_orientation_config.enforcement = mcc::HardEnforcement{};
  mcc::OrientationTaskHandle left_orientation_task;
  throwIfError(builder.addOrientationTask(
    robot.left_end_effector_frame, left_orientation_config, left_orientation_task));

  mcc::PositionTaskConfig right_position_config;
  right_position_config.name = "right-position";
  right_position_config.enforcement = mcc::HardEnforcement{};
  mcc::PositionTaskHandle right_position_task;
  throwIfError(builder.addPositionTask(
    robot.right_end_effector_frame, right_position_config, right_position_task));

  mcc::OrientationTaskConfig right_orientation_config;
  right_orientation_config.name = "right-orientation";
  right_orientation_config.enforcement = mcc::HardEnforcement{};
  mcc::OrientationTaskHandle right_orientation_task;
  throwIfError(builder.addOrientationTask(
    robot.right_end_effector_frame, right_orientation_config, right_orientation_task));

  mcc::PostureTaskConfig posture_config;
  posture_config.name = "initial-posture";
  posture_config.enforcement =
    mcc::squaredL2Penalty(1.0e-5, static_cast<int>(joint_names.size()));
  posture_config.reference_positions = mcl::toEigen(positions);
  posture_config.role = mcc::PostureTaskRole::Regularization;
  mcc::PostureTaskHandle posture_task;
  throwIfError(builder.addPostureTask(posture_config, posture_task));

  mcc::JointPositionLimitConfig joint_limit_config;
  joint_limit_config.margin = 1.0e-3;
  joint_limit_config.enforcement = mcc::HardEnforcement{};
  mcc::JointPositionLimitHandle joint_limits;
  throwIfError(builder.addJointPositionLimits(joint_limit_config, joint_limits));

  mcc::KinematicsSolver solver;
  throwIfError(builder.finalize(solver));

  auto currentTargetPose = [&](mcl::ArmSide side) {
      mcc::ForwardKinematicsRequest request;
      request.state = mcl::makeRobotState(positions, velocities);
      request.frame_names = {mcl::frameForSide(robot, side)};
      request.reference_frame_name = robot.base_frame;
      mcc::ForwardKinematicsSolution solution;
      mcc::ForwardKinematicsDiagnostics diagnostics;
      throwIfError(solver.computeForwardKinematics(request, solution, diagnostics));
      return solution.poses.at(0).pose;
    };

  const auto presentation = mcl::makeArmPresentation(
    robot, mcl::foxgloveIkVisualizationChannels());
  const auto initial_left_fk = currentTargetPose(mcl::ArmSide::Left);
  const auto initial_right_fk = currentTargetPose(mcl::ArmSide::Right);
  mcl::TuiTeleopSource tui(
    options.tui, options.rate_hz, kTitle, presentation,
    {{mcl::ArmSide::Left, initial_left_fk}, {mcl::ArmSide::Right, initial_right_fk}}, true);
  auto visualization_sink = mcl::createVisualizationSink(options.visualization, kProgramId);

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

  visualization_sink->open({"interactive-preview", kProgramId});
  while (const auto schedule = scheduler.next()) {
    tui.poll();
    if (const auto reset_side = tui.consumeResetRequest()) {
      tui.setTargetPose(
        *reset_side,
        currentTargetPose(*reset_side),
        std::string{"Reset "} + mcl::armSideName(*reset_side) + " target from current FK");
    }

    const auto & command = tui.command();
    if (command.stop_requested) {
      break;
    }

    if (schedule->update_due && !command.paused) {
      const auto & left_target = command.targets.at(0);
      const auto & right_target = command.targets.at(1);
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
      latest_frame.targets = command.targets;
      latest_frame.iterations = diagnostics.iterations;
      latest_frame.solve_time_ms = diagnostics.solve_time_ms;
      if (latest_frame.solvers.empty()) {
        latest_frame.solvers.push_back(
          mcl::makeSolverDebug("IK", diagnostics, solution.disposition));
      } else {
        mcl::updateSolverDebug(latest_frame.solvers.front(), diagnostics, solution.disposition);
      }
      latest_frame.target_errors.clear();

      if (status.code == mcc::StatusCode::Infeasible) {
        latest_frame.ik_status = "rejected: infeasible";
        latest_frame.converged = false;
        latest_frame.status = status.message;
      } else {
        throwIfError(status);
        if (!mcc::isAccepted(solution.disposition)) {
          throw std::runtime_error("IK candidate rejected");
        }
        positions = mcl::toStdVector(solution.joint_positions);

        latest_frame.forward_kinematics = {
          {mcl::ArmSide::Left, poseForFrame(
              solution.solved_poses, robot.left_end_effector_frame)},
          {mcl::ArmSide::Right, poseForFrame(
              solution.solved_poses, robot.right_end_effector_frame)}};
        latest_frame.joint_names = joint_names;
        latest_frame.positions = positions;
        latest_frame.velocities = velocities;
        latest_frame.ik_status = "ok";
        latest_frame.converged = diagnostics.converged;

        mcl::ArmTargetError left_error;
        left_error.side = mcl::ArmSide::Left;
        mcl::ArmTargetError right_error;
        right_error.side = mcl::ArmSide::Right;
        for (const auto & error : diagnostics.position_errors) {
          if (error.handle.value == left_position_task.value) {
            left_error.position_m = error.norm_m;
          } else if (error.handle.value == right_position_task.value) {
            right_error.position_m = error.norm_m;
          }
        }
        for (const auto & error : diagnostics.orientation_errors) {
          if (error.handle.value == left_orientation_task.value) {
            left_error.orientation_rad = error.norm_rad;
          } else if (error.handle.value == right_orientation_task.value) {
            right_error.orientation_rad = error.norm_rad;
          }
        }
        latest_frame.target_errors = {left_error, right_error};
        latest_frame.status = "IK accepted";
      }
      latest_frame.paused = command.paused;
      latest_frame.selected_side = command.selected_side;
      visualization_sink->write(mcl::makeIkVisualizationFrame(
        latest_frame, presentation, publish_count, schedule->sample_time_ns,
        schedule->emit_time_ns));
      ++publish_count;
    }

    if (schedule->draw_due) {
      latest_frame.paused = tui.command().paused;
      latest_frame.selected_side = tui.command().selected_side;
      tui.render(latest_frame, publish_count, visualization_sink->status());
    }
    scheduler.sleep();
  }

  visualization_sink->flush();
  visualization_sink->close();
  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    return run(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << kProgramId << ": " << error.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << kProgramId << ": non-standard exception\n";
    return EXIT_FAILURE;
  }
}
