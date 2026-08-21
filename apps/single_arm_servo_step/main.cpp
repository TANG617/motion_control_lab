#include <Eigen/Core>
#include <array>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <motion_control_core/motion_control_core.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "app_options.hpp"
#include "components/app_helpers/app_helpers.hpp"
#include "components/robot/r1/r1_robot_config.hpp"
#include "components/scheduler/rolling_percentiles.hpp"
#include "components/scheduler/single_rate_scheduler.hpp"
#include "components/teleop/keyboard/keyboard_target_source.hpp"
#include "components/tui/tui_renderer.hpp"
#include "components/visualization/preview_projection.hpp"
#include "components/visualization/preview_transport.hpp"
#include "contracts/presentation/ik_app_snapshot.hpp"
#include "diagnostics_projection.hpp"
#include "components/tui/standard_ik_tui.hpp"

namespace
{

namespace mcc = motion_control::core;
namespace mcl = motion_control_lab;

mcc::RobotState makeRobotState(
  const std::vector<double> & positions, const std::vector<double> & velocities)
{
  mcc::RobotState state;
  state.joint_positions = mcl::toEigen(positions);
  state.joint_velocities = mcl::toEigen(velocities);
  return state;
}

constexpr const char * kProgramId = "mcl_single_arm_servo_step";
constexpr const char * kTitle = "Motion Control Single-arm IK";
constexpr std::array<unsigned int, 1> kMainCpuAffinity{8};

void throwIfError(const mcc::Status & status)
{
  if (!status.ok()) {
    throw std::runtime_error(status.message);
  }
}

int run(int argc, char ** argv)
{
  const auto options = mcl::single_arm_servo_step::parseAppOptions(argc, argv);
  const auto affinity_domain = mcl::CpuAffinityDomain::capture();
  const auto affinity_binding =
    affinity_domain.bindCurrentThread(kProgramId, "main", kMainCpuAffinity);
  const auto & robot = mcl::r1RobotConfig();
  const auto controlled_side = mcl::parseArmSide(options.tui.side);
  const std::string & controlled_frame = mcl::frameForSide(robot, controlled_side);
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
  solver_config.mode = mcc::IkSolveMode::ServoStep;
  solver_config.servo_period = 1.0 / options.rate_hz;
  solver_config.joint_limit_policy = mcc::KinematicsJointLimitPolicy::ExplicitRequirements;
  solver_config.qp.backend = mcc::QpBackend::ProxQp;
  solver_config.qp.regularization = options.regularization;
  solver_config.maximum_iterations = options.maximum_iterations;
  solver_config.soft_solve_time_budget_ms = 100.0;
  solver_config.position_tolerance_m = options.position_tolerance_m;
  solver_config.orientation_tolerance_rad = options.orientation_tolerance_rad;
  solver_config.minimum_position_improvement_m = 1.0e-8;
  solver_config.minimum_orientation_improvement_rad = 1.0e-8;

  mcc::KinematicsSolverBuilder builder;
  throwIfError(builder.configure(model, joint_names, solver_config));

  mcc::PositionTaskConfig position_task_config;
  position_task_config.name = std::string{mcl::armSideName(controlled_side)} + "-position";
  position_task_config.enforcement = mcc::HardEnforcement{};
  mcc::PositionTaskHandle position_task;
  throwIfError(builder.addPositionTask(controlled_frame, position_task_config, position_task));

  mcc::OrientationTaskConfig orientation_task_config;
  orientation_task_config.name = std::string{mcl::armSideName(controlled_side)} + "-orientation";
  orientation_task_config.enforcement = mcc::HardEnforcement{};
  mcc::OrientationTaskHandle orientation_task;
  throwIfError(
    builder.addOrientationTask(controlled_frame, orientation_task_config, orientation_task));

  mcc::JointPositionLimitConfig joint_limit_config;
  joint_limit_config.margin = options.joint_position_margin_rad;
  joint_limit_config.enforcement = mcc::HardEnforcement{};
  mcc::JointPositionLimitHandle joint_limits;
  throwIfError(builder.addJointPositionLimits(joint_limit_config, joint_limits));

  mcc::JointVelocityLimitConfig velocity_limit_config;
  velocity_limit_config.enforcement = mcc::HardEnforcement{};
  mcc::JointVelocityLimitHandle velocity_limits;
  throwIfError(builder.addJointVelocityLimits(velocity_limit_config, velocity_limits));

  mcc::KinematicsSolver solver;
  throwIfError(builder.finalize(solver));

  auto currentTargetPose = [&](mcl::ArmSide side) {
    mcc::ForwardKinematicsRequest request;
    request.state = makeRobotState(positions, velocities);
    request.frame_names = {mcl::frameForSide(robot, side)};
    request.reference_frame_name = robot.base_frame;
    mcc::ForwardKinematicsSolution solution;
    mcc::ForwardKinematicsDiagnostics diagnostics;
    throwIfError(solver.computeForwardKinematics(request, solution, diagnostics));
    return solution.poses.at(0).pose;
  };

  const auto presentation = mcl::makeArmPresentation(robot, mcl::foxgloveIkVisualizationChannels());
  const auto initial_left_fk = currentTargetPose(mcl::ArmSide::Left);
  const auto initial_right_fk = currentTargetPose(mcl::ArmSide::Right);
  mcl::TerminalFrontend terminal({true, options.tui_enabled});
  mcl::KeyboardTargetSource input(
    terminal, mcl::KeyboardSourceMode::Teleop, options.tui,
    {{mcl::ArmSide::Left, initial_left_fk}, {mcl::ArmSide::Right, initial_right_fk}}, false);
  mcl::TuiRenderer tui(options.tui_enabled);
  auto visualization_sink = mcl::createPreviewSink(options.visualization, kProgramId);

  mcl::installRuntimeSignalHandlers();
  mcl::SingleRateScheduler scheduler({options.rate_hz, options.duration_s});
  mcl::RollingPercentiles solve_time_percentiles;
  std::size_t publish_count = 0;

  mcl::IkDebugFrame latest_frame;
  latest_frame.targets = input.targets();
  latest_frame.forward_kinematics = {
    {mcl::ArmSide::Left, initial_left_fk}, {mcl::ArmSide::Right, initial_right_fk}};
  latest_frame.joint_names = joint_names;
  latest_frame.positions = positions;
  latest_frame.velocities = velocities;
  latest_frame.selected_side = controlled_side;
  latest_frame.cpu_affinities = {mcl::makeCpuAffinityDebug(affinity_binding)};

  visualization_sink->open();

  while (const auto schedule = scheduler.next()) {
    const auto input_update = input.poll(schedule->dt);
    for (const auto & event : input_update.navigation) {
      tui.handleNavigation(event);
    }
    if (const auto reset_side = input.consumeResetRequest()) {
      input.setTargetPose(
        *reset_side, currentTargetPose(*reset_side),
        std::string{"Reset "} + mcl::armSideName(*reset_side) + " target from current FK");
    }

    if (input.stopRequested()) {
      break;
    }

    if (schedule->update_due && !input.paused()) {
      const auto & target = input.targets().at(controlled_side == mcl::ArmSide::Left ? 0 : 1);
      mcc::InverseKinematicsRequest request;
      request.reference_frame_name = robot.base_frame;
      request.state = makeRobotState(positions, velocities);
      request.position_targets.push_back({position_task, target.target_pose.translation(), true});
      request.orientation_targets.push_back({orientation_task, target.target_pose.linear(), true});

      mcc::InverseKinematicsSolution solution;
      mcc::InverseKinematicsDiagnostics diagnostics;
      const auto status = solver.solveInverseKinematics(request, solution, diagnostics);
      solve_time_percentiles.record(diagnostics.solve_time_ms);
      throwIfError(status);
      if (!mcc::isAccepted(solution.disposition)) {
        throw std::runtime_error("IK candidate rejected");
      }
      positions = mcl::toStdVector(solution.joint_positions);
      velocities = mcl::toStdVector(solution.joint_velocities);

      latest_frame.targets = input.targets();
      latest_frame.forward_kinematics = {
        {mcl::ArmSide::Left, currentTargetPose(mcl::ArmSide::Left)},
        {mcl::ArmSide::Right, currentTargetPose(mcl::ArmSide::Right)}};
      latest_frame.joint_names = joint_names;
      latest_frame.positions = positions;
      latest_frame.velocities = velocities;
      latest_frame.ik_status = status.ok() ? "ok" : status.message;
      latest_frame.iterations = diagnostics.iterations;
      latest_frame.converged = diagnostics.converged;
      latest_frame.solve_time_ms = diagnostics.solve_time_ms;
      if (latest_frame.solvers.empty()) {
        latest_frame.solvers.push_back(
          mcl::makeSolverDebug("MCC", diagnostics, solution.disposition));
      } else {
        mcl::updateSolverDebug(latest_frame.solvers.front(), diagnostics, solution.disposition);
      }
      latest_frame.target_errors.clear();
      mcl::ArmTargetError target_error;
      target_error.side = controlled_side;
      bool has_error = false;
      for (const auto & error : diagnostics.position_errors) {
        if (error.handle.value == position_task.value) {
          target_error.position_m = error.norm_m;
          has_error = true;
          break;
        }
      }
      for (const auto & error : diagnostics.orientation_errors) {
        if (error.handle.value == orientation_task.value) {
          target_error.orientation_rad = error.norm_rad;
          has_error = true;
          break;
        }
      }
      if (has_error) {
        latest_frame.target_errors.push_back(target_error);
      }
      latest_frame.status = "IK accepted";
      latest_frame.paused = input.paused();
      latest_frame.selected_side = input.selectedSide();

      visualization_sink->write(mcl::makeIkRenderBatch(
        latest_frame, presentation, schedule->emit_time_ns));
      ++publish_count;
    }

    if (schedule->draw_due) {
      if (!latest_frame.solvers.empty()) {
        latest_frame.solvers.front().ik_solve_time_percentiles = solve_time_percentiles.snapshot();
      }
      latest_frame.paused = input.paused();
      tui.render(mcl::makeStandardIkTuiDocument(
        latest_frame, presentation, publish_count, visualization_sink->status(), kTitle,
        input.status()));
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
