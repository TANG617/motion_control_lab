#include "diagnostics_projection.hpp"

#include <utility>

namespace motion_control_lab
{
namespace
{

namespace mcc = motion_control::core;

const char * resultDispositionName(mcc::ResultDisposition value)
{
  return mcc::isAccepted(value) ? "accepted" : "rejected";
}

const char * jointLimitPolicyName(mcc::KinematicsJointLimitPolicy value)
{
  switch (value) {
    case mcc::KinematicsJointLimitPolicy::ModelPositionOnly: return "model-position";
    case mcc::KinematicsJointLimitPolicy::ModelPositionAndVelocity: return "model-position+velocity";
    case mcc::KinematicsJointLimitPolicy::ExplicitRequirements: return "explicit-requirements";
    case mcc::KinematicsJointLimitPolicy::Unconstrained: return "unconstrained";
  }
  return "unknown";
}

const char * terminationReasonName(mcc::IkTerminationReason value)
{
  switch (value) {
    case mcc::IkTerminationReason::NotStarted: return "not-started";
    case mcc::IkTerminationReason::Converged: return "converged";
    case mcc::IkTerminationReason::NoEnabledConvergenceTasks: return "no-convergence-tasks";
    case mcc::IkTerminationReason::SingleIteration: return "single-iteration";
    case mcc::IkTerminationReason::Saturated: return "saturated";
    case mcc::IkTerminationReason::NoProgress: return "no-progress";
    case mcc::IkTerminationReason::IterationBudget: return "iteration-budget";
    case mcc::IkTerminationReason::SoftTimeBudget: return "soft-time-budget";
    case mcc::IkTerminationReason::HardConstraintViolation: return "hard-constraint-violation";
    case mcc::IkTerminationReason::InvalidNumericalSolution: return "invalid-numerical-solution";
  }
  return "unknown";
}

const char * qpBackendName(mcc::QpBackend value)
{
  switch (value) {
    case mcc::QpBackend::Eiquadprog: return "eiquadprog";
    case mcc::QpBackend::ProxQp: return "proxqp";
  }
  return "unknown";
}

const char * qpStatusName(mcc::QpSolveStatus value)
{
  switch (value) {
    case mcc::QpSolveStatus::Optimal: return "optimal";
    case mcc::QpSolveStatus::PrimalInfeasible: return "primal-infeasible";
    case mcc::QpSolveStatus::DualInfeasible: return "dual-infeasible";
    case mcc::QpSolveStatus::MaximumIterations: return "maximum-iterations";
    case mcc::QpSolveStatus::NumericalFailure: return "numerical-failure";
    case mcc::QpSolveStatus::NotRun: return "not-run";
  }
  return "unknown";
}

void updateLocalSolverDebug(
  SolverDebug & output,
  const mcc::InverseKinematicsDiagnostics & diagnostics,
  mcc::ResultDisposition disposition)
{
  output.disposition = resultDispositionName(disposition);
  output.joint_limit_policy = jointLimitPolicyName(diagnostics.joint_limit_policy);
  output.termination_reason = terminationReasonName(diagnostics.termination_reason);
  output.ik_iterations = diagnostics.iterations;
  output.converged = diagnostics.converged;
  output.ik_solve_time_ms = diagnostics.solve_time_ms;
  output.saturated_joints = diagnostics.saturated_joints;

  const auto & optimization = diagnostics.optimization;
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
  for (std::size_t index = 0; index < optimization.task_scales.size(); ++index) {
    const auto & source = optimization.task_scales[index];
    auto & destination = output.task_scales[index];
    destination.name = source.name;
    destination.active = source.active;
    destination.scale = source.scale;
    destination.cost = source.cost;
    destination.degraded = source.degraded;
    destination.stuck = source.stuck;
  }
  output.requirements.resize(optimization.requirements.size());
  for (std::size_t index = 0; index < optimization.requirements.size(); ++index) {
    const auto & source = optimization.requirements[index];
    auto & destination = output.requirements[index];
    destination.name = source.name;
    destination.unit = source.unit;
    destination.source = source.source;
    destination.enabled = source.enabled;
    destination.active = source.active;
    destination.maximum_violation = source.maximum_violation;
    destination.cost = source.cost;
  }
}

}  // namespace

SolverDebug makeSolverDebug(
  std::string label,
  const motion_control::core::InverseKinematicsDiagnostics & diagnostics,
  motion_control::core::ResultDisposition disposition)
{
  SolverDebug output;
  output.label = std::move(label);
  updateSolverDebug(output, diagnostics, disposition);
  return output;
}

void updateSolverDebug(
  SolverDebug & output,
  const motion_control::core::InverseKinematicsDiagnostics & diagnostics,
  motion_control::core::ResultDisposition disposition)
{
  updateLocalSolverDebug(output, diagnostics, disposition);
  output.grouped_attempt.reset();
}

}  // namespace motion_control_lab

