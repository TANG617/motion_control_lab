#include "loop.hpp"

#include <array>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "components/app_helpers/app_helpers.hpp"
#include "components/scheduler/rolling_percentiles.hpp"
#include "components/scheduler/single_rate_scheduler.hpp"
#include "components/teleop/keyboard/keyboard_target_source.hpp"
#include "components/tui/standard_ik_tui.hpp"
#include "components/tui/tui_renderer.hpp"
#include "components/visualization/preview_projection.hpp"
#include "components/visualization/preview_transport.hpp"
#include "contracts/presentation/ik_app_snapshot.hpp"

namespace motion_control_lab::single_arm_servo_step {
namespace {

constexpr const char *kProgramId = "mcl_single_arm_servo_step";
constexpr const char *kTitle = "Motion Control Single-arm IK";
constexpr std::array<unsigned int, 1> kMainCpuAffinity{8};

mcc::RobotState makeRobotState(const std::vector<double> &positions,
                               const std::vector<double> &velocities) {
  mcc::RobotState state;
  state.joint_positions = toEigen(positions);
  state.joint_velocities = toEigen(velocities);
  return state;
}

const char *resultDispositionName(mcc::ResultDisposition value) {
  return mcc::isAccepted(value) ? "accepted" : "rejected";
}

const char *jointLimitPolicyName(mcc::KinematicsJointLimitPolicy value) {
  switch (value) {
  case mcc::KinematicsJointLimitPolicy::ModelPositionOnly:
    return "model-position";
  case mcc::KinematicsJointLimitPolicy::ModelPositionAndVelocity:
    return "model-position+velocity";
  case mcc::KinematicsJointLimitPolicy::ExplicitRequirements:
    return "explicit-requirements";
  case mcc::KinematicsJointLimitPolicy::Unconstrained:
    return "unconstrained";
  }
  return "unknown";
}

const char *terminationReasonName(mcc::IkTerminationReason value) {
  switch (value) {
  case mcc::IkTerminationReason::NotStarted:
    return "not-started";
  case mcc::IkTerminationReason::Converged:
    return "converged";
  case mcc::IkTerminationReason::NoEnabledConvergenceTasks:
    return "no-convergence-tasks";
  case mcc::IkTerminationReason::SingleIteration:
    return "single-iteration";
  case mcc::IkTerminationReason::Saturated:
    return "saturated";
  case mcc::IkTerminationReason::NoProgress:
    return "no-progress";
  case mcc::IkTerminationReason::IterationBudget:
    return "iteration-budget";
  case mcc::IkTerminationReason::SoftTimeBudget:
    return "soft-time-budget";
  case mcc::IkTerminationReason::HardConstraintViolation:
    return "hard-constraint-violation";
  case mcc::IkTerminationReason::InvalidNumericalSolution:
    return "invalid-numerical-solution";
  }
  return "unknown";
}

const char *qpBackendName(mcc::QpBackend value) {
  switch (value) {
  case mcc::QpBackend::Eiquadprog:
    return "eiquadprog";
  case mcc::QpBackend::ProxQp:
    return "proxqp";
  }
  return "unknown";
}

const char *qpStatusName(mcc::QpSolveStatus value) {
  switch (value) {
  case mcc::QpSolveStatus::Optimal:
    return "optimal";
  case mcc::QpSolveStatus::PrimalInfeasible:
    return "primal-infeasible";
  case mcc::QpSolveStatus::DualInfeasible:
    return "dual-infeasible";
  case mcc::QpSolveStatus::MaximumIterations:
    return "maximum-iterations";
  case mcc::QpSolveStatus::NumericalFailure:
    return "numerical-failure";
  case mcc::QpSolveStatus::NotRun:
    return "not-run";
  }
  return "unknown";
}

void updateSolverDebug(SolverDebug &output,
                       const mcc::InverseKinematicsDiagnostics &diagnostics,
                       mcc::ResultDisposition disposition) {
  output.disposition = resultDispositionName(disposition);
  output.joint_limit_policy =
      jointLimitPolicyName(diagnostics.joint_limit_policy);
  output.termination_reason =
      terminationReasonName(diagnostics.termination_reason);
  output.ik_iterations = diagnostics.iterations;
  output.converged = diagnostics.converged;
  output.ik_solve_time_ms = diagnostics.solve_time_ms;
  output.saturated_joints = diagnostics.saturated_joints;

  const auto &optimization = diagnostics.optimization;
  output.backend = qpBackendName(optimization.backend);
  output.qp_status = qpStatusName(optimization.solver_status);
  output.native_status = optimization.native_status;
  output.has_qp_diagnostics = true;
  output.objective_value = optimization.objective_value;
  output.primal_residual = optimization.primal_residual;
  output.dual_residual = optimization.dual_residual;
  output.maximum_hard_violation = optimization.maximum_hard_violation;
  output.qp_solve_time_ms = optimization.solve_time_ms;
  output.qp_iterations = optimization.iterations;
  output.active_set_size = optimization.active_set_size;
  output.warm_start_used = optimization.warm_start_used;

  output.task_scales.resize(optimization.task_scales.size());
  for (std::size_t index = 0; index < optimization.task_scales.size();
       ++index) {
    const auto &source = optimization.task_scales[index];
    auto &destination = output.task_scales[index];
    destination.name = source.name;
    destination.active = source.active;
    destination.scale = source.scale;
    destination.cost = source.cost;
    destination.degraded = source.degraded;
    destination.stuck = source.stuck;
  }
  output.requirements.resize(optimization.requirements.size());
  for (std::size_t index = 0; index < optimization.requirements.size();
       ++index) {
    const auto &source = optimization.requirements[index];
    auto &destination = output.requirements[index];
    destination.name = source.name;
    destination.unit = source.unit;
    destination.source = source.source;
    destination.enabled = source.enabled;
    destination.active = source.active;
    destination.maximum_violation = source.maximum_violation;
    destination.cost = source.cost;
  }
  output.grouped_attempt.reset();
}

SolverDebug
makeSolverDebug(std::string label,
                const mcc::InverseKinematicsDiagnostics &diagnostics,
                mcc::ResultDisposition disposition) {
  SolverDebug output;
  output.label = std::move(label);
  updateSolverDebug(output, diagnostics, disposition);
  return output;
}

} // namespace

int runLoop(const AppOptions &options, const R1RobotConfig &robot,
            ArmSide controlled_side, mcc::KinematicsSolver &solver,
            const SolverHandles &handles) {
  const auto affinity_domain = CpuAffinityDomain::capture();
  const auto affinity_binding =
      affinity_domain.bindCurrentThread(kProgramId, "main", kMainCpuAffinity);
  auto positions = robot.default_positions;
  std::vector<double> velocities(positions.size(), 0.0);

  auto currentTargetPose = [&](ArmSide side) {
    mcc::ForwardKinematicsRequest request;
    request.state = makeRobotState(positions, velocities);
    request.frame_names = {frameForSide(robot, side)};
    request.reference_frame_name = robot.base_frame;
    mcc::ForwardKinematicsSolution solution;
    mcc::ForwardKinematicsDiagnostics diagnostics;
    requireOk(solver.computeForwardKinematics(request, solution, diagnostics));
    return solution.poses.at(0).pose;
  };

  const auto presentation =
      makeArmPresentation(robot, foxgloveIkVisualizationChannels());
  const auto initial_left_fk = currentTargetPose(ArmSide::Left);
  const auto initial_right_fk = currentTargetPose(ArmSide::Right);
  TerminalFrontend terminal({true, options.tui_enabled});
  KeyboardTargetSource input(
      terminal, KeyboardSourceMode::Teleop, options.tui,
      {{ArmSide::Left, initial_left_fk}, {ArmSide::Right, initial_right_fk}},
      false);
  TuiRenderer tui(options.tui_enabled);
  auto visualization_sink =
      createPreviewSink(options.visualization, kProgramId);

  installRuntimeSignalHandlers();
  SingleRateScheduler scheduler({options.rate_hz, options.duration_s});
  RollingPercentiles solve_time_percentiles;
  std::size_t publish_count = 0;

  IkDebugFrame latest_frame;
  latest_frame.targets = input.targets();
  latest_frame.forward_kinematics = {{ArmSide::Left, initial_left_fk},
                                     {ArmSide::Right, initial_right_fk}};
  latest_frame.joint_names = robot.joint_names;
  latest_frame.positions = positions;
  latest_frame.velocities = velocities;
  latest_frame.selected_side = controlled_side;
  latest_frame.cpu_affinities = {makeCpuAffinityDebug(affinity_binding)};

  visualization_sink->open();
  while (const auto schedule = scheduler.next()) {
    const auto input_update = input.poll(schedule->dt);
    for (const auto &event : input_update.navigation) {
      tui.handleNavigation(event);
    }
    if (const auto reset_side = input.consumeResetRequest()) {
      input.setTargetPose(*reset_side, currentTargetPose(*reset_side),
                          std::string{"Reset "} + armSideName(*reset_side) +
                              " target from current FK");
    }
    if (input.stopRequested()) {
      break;
    }

    if (schedule->update_due && !input.paused()) {
      const auto &target =
          input.targets().at(controlled_side == ArmSide::Left ? 0 : 1);
      mcc::InverseKinematicsRequest request;
      request.reference_frame_name = robot.base_frame;
      request.state = makeRobotState(positions, velocities);
      request.position_targets.push_back(
          {handles.position, target.target_pose.translation(), true});
      request.orientation_targets.push_back(
          {handles.orientation, target.target_pose.linear(), true});

      mcc::InverseKinematicsSolution solution;
      mcc::InverseKinematicsDiagnostics diagnostics;
      const auto status =
          solver.solveInverseKinematics(request, solution, diagnostics);
      solve_time_percentiles.record(diagnostics.solve_time_ms);
      requireOk(status);
      if (!mcc::isAccepted(solution.disposition)) {
        throw std::runtime_error("IK candidate rejected");
      }
      positions = toStdVector(solution.joint_positions);
      velocities = toStdVector(solution.joint_velocities);

      latest_frame.targets = input.targets();
      latest_frame.forward_kinematics = {
          {ArmSide::Left, currentTargetPose(ArmSide::Left)},
          {ArmSide::Right, currentTargetPose(ArmSide::Right)}};
      latest_frame.positions = positions;
      latest_frame.velocities = velocities;
      latest_frame.ik_status = status.ok() ? "ok" : status.message;
      latest_frame.iterations = diagnostics.iterations;
      latest_frame.converged = diagnostics.converged;
      latest_frame.solve_time_ms = diagnostics.solve_time_ms;
      if (latest_frame.solvers.empty()) {
        latest_frame.solvers.push_back(
            makeSolverDebug("MCC", diagnostics, solution.disposition));
      } else {
        updateSolverDebug(latest_frame.solvers.front(), diagnostics,
                          solution.disposition);
      }
      latest_frame.target_errors.clear();
      ArmTargetError target_error;
      target_error.side = controlled_side;
      bool has_error = false;
      for (const auto &error : diagnostics.position_errors) {
        if (error.handle.value == handles.position.value) {
          target_error.position_m = error.norm_m;
          has_error = true;
          break;
        }
      }
      for (const auto &error : diagnostics.orientation_errors) {
        if (error.handle.value == handles.orientation.value) {
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
      visualization_sink->write(makeIkRenderBatch(latest_frame, presentation,
                                                  schedule->emit_time_ns));
      ++publish_count;
    }

    if (schedule->draw_due) {
      if (!latest_frame.solvers.empty()) {
        latest_frame.solvers.front().ik_solve_time_percentiles =
            solve_time_percentiles.snapshot();
      }
      latest_frame.paused = input.paused();
      latest_frame.selected_side = input.selectedSide();
      tui.render(makeStandardIkTuiDocument(
          latest_frame, presentation, publish_count,
          visualization_sink->status(), kTitle, input.status()));
    }
    scheduler.sleep();
  }

  visualization_sink->flush();
  visualization_sink->close();
  return EXIT_SUCCESS;
}

} // namespace motion_control_lab::single_arm_servo_step
