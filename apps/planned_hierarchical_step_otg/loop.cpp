#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <initializer_list>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <motion_control_core/motion_control_core.hpp>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "adapters/replay/replay_support.hpp"
#include "components/app_helpers/app_helpers.hpp"
#include "components/replay/replay_source.hpp"
#include "components/robot/r1/r1_robot_config.hpp"
#include "components/scheduler/grouped_worker.hpp"
#include "components/scheduler/latest_value_mailbox.hpp"
#include "components/scheduler/rolling_percentiles.hpp"
#include "components/scheduler/single_rate_scheduler.hpp"
#include "components/teleop/keyboard/keyboard_target_source.hpp"
#include "components/tui/planned_grouped_tui.hpp"
#include "components/visualization/preview_projection.hpp"
#include "components/visualization/preview_transport.hpp"
#include "contracts/presentation/ik_app_snapshot.hpp"
#include "contracts/visualization/mcl_execution_v1.hpp"
#include "contracts/visualization/mcl_planning_v1.hpp"
#include "contracts/visualization/mcl_state_v1.hpp"
#include "contracts/visualization/mcl_telemetry_v1.hpp"
#include "loop.hpp"
#include "motion_control_lab/run_artifacts.hpp"
#include "motion_control_lab/sha256.hpp"
#include "options.hpp"
#include "planning.hpp"
#include "solver.hpp"
#include "telemetry.hpp"

namespace motion_control_lab::planned_hierarchical_step_otg {
namespace {

namespace mcc = motion_control::core;
namespace mcl = motion_control_lab;
namespace replay = motion_control_lab::replay;

using mcl::toEigen;
using mcl::toStdVector;

constexpr const char *kProgramId = "mcl_planned_hierarchical_step_otg";
constexpr const char *kTitle = "Motion Control Planned Hierarchical Step OTG";
constexpr std::array<unsigned int, 1> kUiCpuAffinity{5};
constexpr std::array<unsigned int, 1> kRedCpuAffinity{6};
constexpr std::array<unsigned int, 1> kYellowCpuAffinity{7};

std::uint64_t workerTicksForReplayDuration(std::int64_t duration_ns,
                                           double rate_hz) {
  const long double ticks =
      static_cast<long double>(duration_ns) * rate_hz / 1.0e9L;
  return static_cast<std::uint64_t>(
      std::max<long double>(1.0L, std::ceil(ticks)));
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

const char *hierarchicalPassName(mcc::HierarchicalSolvePass value) {
  switch (value) {
  case mcc::HierarchicalSolvePass::Primary:
    return "Primary";
  case mcc::HierarchicalSolvePass::Secondary:
    return "Secondary";
  case mcc::HierarchicalSolvePass::Tertiary:
    return "Tertiary";
  case mcc::HierarchicalSolvePass::Terminal:
    return "Terminal";
  }
  return "unknown";
}

const char *hierarchicalConstraintKindName(
    mcc::HierarchicalConstraintKind value) {
  switch (value) {
  case mcc::HierarchicalConstraintKind::TaskEquation:
    return "task-equation";
  case mcc::HierarchicalConstraintKind::JointBound:
    return "joint-bound";
  case mcc::HierarchicalConstraintKind::TaskScaleBound:
    return "scale-bound";
  }
  return "unknown";
}

const char *hierarchicalBoundSourceName(mcc::HierarchicalBoundSource value) {
  switch (value) {
  case mcc::HierarchicalBoundSource::None:
    return "-";
  case mcc::HierarchicalBoundSource::JointPosition:
    return "joint-position";
  case mcc::HierarchicalBoundSource::JointPositionBraking:
    return "position-braking";
  case mcc::HierarchicalBoundSource::JointVelocity:
    return "joint-velocity";
  case mcc::HierarchicalBoundSource::JointAcceleration:
    return "joint-acceleration";
  case mcc::HierarchicalBoundSource::TaskScale:
    return "task-scale";
  }
  return "unknown";
}

const char *hierarchicalBoundSideName(mcc::HierarchicalBoundSide value) {
  switch (value) {
  case mcc::HierarchicalBoundSide::Lower:
    return "lower";
  case mcc::HierarchicalBoundSide::Upper:
    return "upper";
  }
  return "unknown";
}

const char *hierarchicalViolationUnit(
    const mcc::HierarchicalConstraintViolation &violation) {
  if (violation.kind == mcc::HierarchicalConstraintKind::TaskScaleBound) {
    return "1";
  }
  if (violation.kind == mcc::HierarchicalConstraintKind::JointBound ||
      violation.task_kind != mcc::HierarchicalTaskKind::Position) {
    return "rad/s";
  }
  return "m/s";
}

using QpPassTimePercentiles = std::array<mcl::RollingPercentiles, 4>;

void recordQpPassTimes(QpPassTimePercentiles &percentiles,
                       const SolverDiagnostics &diagnostics) {
  for (std::size_t index = 0; index < diagnostics.hierarchy.passes.size();
       ++index) {
    const auto &pass = diagnostics.hierarchy.passes[index];
    if (pass.attempted) {
      percentiles[index].record(pass.solve_time_ms);
    }
  }
}

void updateQpPassTimePercentiles(
    mcl::SolverDebug &output,
    const QpPassTimePercentiles &percentiles) {
  const std::size_t count =
      std::min(output.qp_passes.size(), percentiles.size());
  for (std::size_t index = 0; index < count; ++index) {
    output.qp_passes[index].solve_time_percentiles =
        percentiles[index].snapshot();
  }
}

const char *hierarchicalRejectionReasonName(SolverRejectionReason value) {
  switch (value) {
  case SolverRejectionReason::None:
    return "none";
  case SolverRejectionReason::InvalidTarget:
    return "invalid-target";
  case SolverRejectionReason::SolverRejected:
    return "solver-rejected";
  }
  return "unknown";
}

const char *couplingStateName(CouplingState value) {
  switch (value) {
  case CouplingState::Unavailable:
    return "unavailable";
  case CouplingState::WaitingForValue:
    return "waiting-for-value";
  case CouplingState::Active:
    return "active";
  case CouplingState::RejectedSource:
    return "rejected-source";
  }
  return "unknown";
}

void updateSolverDebug(mcl::SolverDebug &output,
                       const SolverDiagnostics &diagnostics,
                       mcc::ResultDisposition disposition) {
  if (diagnostics.hierarchical) {
    output.disposition = resultDispositionName(disposition);
    output.joint_limit_policy =
        jointLimitPolicyName(diagnostics.hierarchy.joint_limit_policy);
    output.termination_reason =
        diagnostics.hierarchy.same_tick_fallback_level.has_value()
            ? "same-tick-fallback"
            : "hierarchical-servo-step";
    output.ik_iterations = diagnostics.iterations;
    output.converged = false;
    output.ik_solve_time_ms = diagnostics.solve_time_ms;
    output.saturated_joints.clear();
    output.backend = "proxqp";
    const auto *last_pass = &diagnostics.hierarchy.passes.front();
    for (const auto &pass : diagnostics.hierarchy.passes) {
      if (pass.attempted) {
        last_pass = &pass;
      }
    }
    output.qp_status = qpStatusName(last_pass->backend_status);
    output.native_status = last_pass->native_status;
    output.has_qp_diagnostics = true;
    output.objective_value = last_pass->objective_value;
    output.primal_residual = last_pass->primal_residual;
    output.dual_residual = last_pass->dual_residual;
    output.maximum_hard_violation = diagnostics.maximum_hard_violation;
    output.qp_solve_time_ms = diagnostics.qp_solve_time_ms;
    output.qp_iterations = diagnostics.iterations;
    output.active_set_size = 0;
    output.warm_start_used = false;
    output.qp_passes.clear();
    output.qp_passes.reserve(diagnostics.hierarchy.passes.size());
    for (const auto &pass : diagnostics.hierarchy.passes) {
      output.warm_start_used =
          output.warm_start_used || pass.warm_start_used;
      output.qp_passes.push_back(
          {hierarchicalPassName(pass.pass), pass.attempted, pass.succeeded,
           qpStatusName(pass.backend_status), pass.native_status,
           pass.solve_time_ms, pass.iterations, pass.warm_start_used});
      auto &pass_output = output.qp_passes.back();
      pass_output.objective_value = pass.objective_value;
      pass_output.primal_residual = pass.primal_residual;
      pass_output.dual_residual = pass.dual_residual;
      pass_output.last_iterate_available = pass.last_iterate_available;
      pass_output.constraint_violations.reserve(
          pass.constraint_violations.size());
      for (const auto &violation : pass.constraint_violations) {
        pass_output.constraint_violations.push_back(
            {hierarchicalConstraintKindName(violation.kind),
             hierarchicalBoundSourceName(violation.bound_source),
             hierarchicalBoundSideName(violation.bound_side),
             violation.source_name, violation.component_name,
             hierarchicalViolationUnit(violation), violation.value,
             violation.lower, violation.upper, violation.violation});
      }
    }
    output.task_scales.resize(diagnostics.hierarchy.task_scales.size());
    for (std::size_t index = 0;
         index < diagnostics.hierarchy.task_scales.size(); ++index) {
      const auto &source = diagnostics.hierarchy.task_scales[index];
      output.task_scales[index] = {
          source.name, source.active,   source.weighted_progress_scale,
          0.0,         source.degraded, source.stuck};
    }
    output.requirements.clear();
  } else {
    const auto &local = diagnostics.kinematics;
    output.disposition = resultDispositionName(disposition);
    output.joint_limit_policy = jointLimitPolicyName(local.joint_limit_policy);
    output.termination_reason = terminationReasonName(local.termination_reason);
    output.ik_iterations = local.iterations;
    output.converged = local.converged;
    output.ik_solve_time_ms = local.solve_time_ms;
    output.saturated_joints = local.saturated_joints;

    const auto &optimization = local.optimization;
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
    output.qp_passes.clear();
    output.task_scales.resize(optimization.task_scales.size());
    for (std::size_t index = 0; index < optimization.task_scales.size();
         ++index) {
      const auto &source = optimization.task_scales[index];
      output.task_scales[index] = {source.name, source.active,   source.scale,
                                   source.cost, source.degraded, source.stuck};
    }
    output.requirements.resize(optimization.requirements.size());
    for (std::size_t index = 0; index < optimization.requirements.size();
         ++index) {
      const auto &source = optimization.requirements[index];
      output.requirements[index] = {source.name,   source.unit,
                                    source.source, source.enabled,
                                    source.active, source.maximum_violation,
                                    source.cost};
    }
  }
  if (!output.grouped_attempt.has_value()) {
    output.grouped_attempt.emplace();
  }
  auto &attempt = *output.grouped_attempt;
  attempt.rejection_reason =
      hierarchicalRejectionReasonName(diagnostics.rejection_reason);
  attempt.run_generation = diagnostics.run_generation;
  attempt.attempt_revision = diagnostics.attempt_revision;
  attempt.value_revision = diagnostics.value_revision;
  attempt.coupling_state = couplingStateName(diagnostics.coupling_state);
  attempt.consumed_source_value_revision =
      diagnostics.consumed_source_value_revision;
  attempt.captured_state_sequence = diagnostics.captured_state_sequence;
  attempt.captured_state_time_nanoseconds =
      diagnostics.captured_state_time_nanoseconds;
}

mcl::SolverDebug makeSolverDebug(std::string label,
                                 const SolverDiagnostics &diagnostics,
                                 mcc::ResultDisposition disposition) {
  mcl::SolverDebug output;
  output.label = std::move(label);
  updateSolverDebug(output, diagnostics, disposition);
  return output;
}
const mcc::FramePose &requirePose(const std::vector<mcc::FramePose> &poses,
                                  const std::string &frame_name) {
  return *std::find_if(poses.begin(), poses.end(),
                       [&](const mcc::FramePose &pose) {
                         return pose.frame_name == frame_name;
                       });
}

const mcl::ArmTarget &requireTarget(const std::vector<mcl::ArmTarget> &targets,
                                    mcl::ArmSide side) {
  return targets.at(side == mcl::ArmSide::Left ? 0 : 1);
}

struct TargetSnapshot {
  std::uint64_t revision{0};
  std::optional<std::size_t> replay_source_index;
  bool replay_joint_hold{false};
  mcc::Pose left{mcc::Pose::Identity()};
  mcc::Pose right{mcc::Pose::Identity()};
};

struct StateSnapshot {
  std::uint64_t sequence{0};
  std::int64_t monotonic_time_nanoseconds{0};
  Eigen::VectorXd positions;
  Eigen::VectorXd velocities;
  Eigen::VectorXd accelerations;
  Eigen::VectorXd jerks;
};

struct TaskScaleSnapshot {
  bool active{false};
  double scale{1.0};
  bool degraded{false};
  bool stuck{false};
};

struct RedOutputSnapshot {
  std::uint64_t event_timestamp_ns{0};
  std::uint64_t run_time_ns{0};
  std::uint64_t revision{0};
  TargetSnapshot accepted_target;
  TargetSnapshot source_goal;
  mcc::CartesianTrajectorySample accepted_planner_sample;
  mcc::PlanningState planner_state{mcc::PlanningState::Idle};
  mcc::PlanningDiagnostics cartesian_plan_diagnostics;
  StateSnapshot state;
  Eigen::VectorXd raw_ik_positions;
  Eigen::VectorXd raw_ik_velocities;
  mcl::planned_hierarchical_step_otg::JointTarget raw_joint_target;
  mcl::planned_hierarchical_step_otg::JointTarget projected_joint_target;
  mcl::planned_hierarchical_step_otg::ProjectionDiagnostics projection;
  std::uint64_t projection_event_count{0U};
  std::uint64_t projection_cycle_count{0U};
  bool future_o1_startup{false};
  mcc::PlanningDiagnostics joint_plan_diagnostics;
  mcc::PlanningDiagnostics joint_step_diagnostics;
  mcc::Pose raw_left_pose{mcc::Pose::Identity()};
  mcc::Pose raw_right_pose{mcc::Pose::Identity()};
  mcc::Pose left_pose{mcc::Pose::Identity()};
  mcc::Pose right_pose{mcc::Pose::Identity()};
  double solve_time_ms{0.0};
  int iterations{0};
  bool converged{false};
  double left_position_error_m{0.0};
  double left_orientation_error_rad{0.0};
  double right_position_error_m{0.0};
  double right_orientation_error_rad{0.0};
  TaskScaleSnapshot left_scale;
  TaskScaleSnapshot right_scale;
  mcl::SolverDebug solver_debug;
};

enum class RedAttemptState {
  Accepted,
  RecoverableRejected,
  FatalRejected,
};

struct RedAttemptSnapshot {
  std::uint64_t event_timestamp_ns{0};
  std::uint64_t run_time_ns{0};
  RedAttemptState state{RedAttemptState::Accepted};
  TargetSnapshot target;
  TargetSnapshot attempted_reference;
  mcl::SolverDebug solver_debug;
  mcl::planned_hierarchical_step_otg::RetargetClampDiagnostics retarget_clamp;
  std::uint64_t retarget_clamp_target_revision{0U};
  std::string detail;
};

const char *retargetClampComponentName(
    mcl::planned_hierarchical_step_otg::RetargetClampComponent component) {
  using Component = mcl::planned_hierarchical_step_otg::RetargetClampComponent;
  switch (component) {
  case Component::LinearVelocity:
    return "linear_velocity";
  case Component::AngularVelocity:
    return "angular_velocity";
  case Component::LinearAcceleration:
    return "linear_acceleration";
  case Component::AngularAcceleration:
    return "angular_acceleration";
  }
  return "unknown";
}

const char *compactJointName(std::size_t index) {
  constexpr std::array<const char *, 20>
      kNames{"HY", "HP", "TY", "TP", "KP", "AP", "L1", "L2", "L3", "L4",
             "L5", "L6", "L7", "R1", "R2", "R3", "R4", "R5", "R6", "R7"};
  return kNames.at(index);
}

const char *
projectionFlag(mcl::planned_hierarchical_step_otg::ProjectionComponent component) {
  using Component = mcl::planned_hierarchical_step_otg::ProjectionComponent;
  switch (component) {
  case Component::VelocityLimit:
    return "V";
  case Component::AccelerationLimit:
    return "A";
  case Component::JerkStoppingEnvelope:
    return "J";
  }
  return "?";
}

std::string retargetClampDetail(const RedAttemptSnapshot &attempt) {
  if (!attempt.retarget_clamp.clamped()) {
    return {};
  }

  constexpr std::array<const char *, 3> kAxisNames{"x", "y", "z"};
  std::ostringstream detail;
  detail << std::setprecision(9)
         << "retarget_clamp revision=" << attempt.retarget_clamp_target_revision
         << " components=" << attempt.retarget_clamp.clamped_component_count
         << " max_limit_ratio=" << attempt.retarget_clamp.maximum_limit_ratio;
  for (std::size_t index = 0U;
       index < attempt.retarget_clamp.clamped_component_count; ++index) {
    const auto &event = attempt.retarget_clamp.events[index];
    detail << " [" << (event.segment_index == 0U ? "left" : "right") << '.'
           << retargetClampComponentName(event.component) << '.'
           << kAxisNames.at(event.axis) << " original=" << event.original_value
           << " applied=" << event.applied_value << " limit=" << event.limit
           << ']';
  }
  return detail.str();
}

struct WorkerThreads {
  explicit WorkerThreads(mcl::WorkerStopController &stop_controller)
      : stop_controller(stop_controller) {}

  ~WorkerThreads() { join(); }

  void join() {
    if (joined) {
      return;
    }
    joined = true;
    stop_controller.requestStop();
    if (red.joinable()) {
      red.join();
    }
    if (yellow.joinable()) {
      yellow.join();
    }
  }

  mcl::WorkerStopController &stop_controller;
  std::thread red;
  std::thread yellow;
  bool joined{false};
};

std::vector<mcl::ArmTarget> armTargets(const TargetSnapshot &target) {
  return {
      {mcl::ArmSide::Left, target.left},
      {mcl::ArmSide::Right, target.right},
  };
}

TargetSnapshot targetSnapshot(const std::vector<mcl::ArmTarget> &targets,
                              std::uint64_t revision) {
  TargetSnapshot result;
  result.revision = revision;
  result.left = requireTarget(targets, mcl::ArmSide::Left).target_pose;
  result.right = requireTarget(targets, mcl::ArmSide::Right).target_pose;
  return result;
}

bool sameTargetPoses(const TargetSnapshot &target,
                     const std::vector<mcl::ArmTarget> &command_targets) {
  constexpr double kPoseComparisonTolerance = 1.0e-12;
  return target.left.matrix().isApprox(
             requireTarget(command_targets, mcl::ArmSide::Left)
                 .target_pose.matrix(),
             kPoseComparisonTolerance) &&
         target.right.matrix().isApprox(
             requireTarget(command_targets, mcl::ArmSide::Right)
                 .target_pose.matrix(),
             kPoseComparisonTolerance);
}

std::string faultSummary(const mcl::GroupedWorkerFault &fault) {
  std::ostringstream summary;
  summary << mcl::workerGroupName(fault.group) << ' '
          << mcl::workerFailureName(fault.failure)
          << " revision=" << fault.revision
          << " release_lateness_ms=" << fault.release_lateness_ms
          << " execution_ms=" << fault.execution_ms
          << " release_to_finish_ms=" << fault.release_to_finish_ms
          << " deadline_ms=" << fault.deadline_ms
          << " overrun_ms=" << fault.overrun_ms
          << " solver_ms=" << fault.solver_ms;
  if (!fault.detail.empty()) {
    summary << ' ' << fault.detail;
  }
  return summary.str();
}

std::string failureLayer(const mcl::GroupedWorkerFault &fault) {
  if (fault.failure == mcl::WorkerFailureKind::DeadlineMiss) {
    return "scheduler-deadline";
  }
  const std::array<std::pair<std::string_view, std::string_view>, 7> layers{
      {{"Cartesian replan", "cartesian-replan"},
       {"Cartesian planner step", "cartesian-step"},
       {"joint target projection", "joint-target-projection"},
       {"JointPlanner plan", "joint-plan"},
       {"JointPlanner step", "joint-step"},
       {"OTG execution-state FK", "otg-fk"},
       {"Red", "red-ik"}}};
  for (const auto &layer : layers) {
    if (fault.detail.find(layer.first) != std::string::npos) {
      return std::string{layer.second};
    }
  }
  return fault.group == mcl::WorkerGroup::Yellow ? "yellow-ik" : "worker";
}

std::string jsonText(const Json::Value &value) {
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "  ";
  return Json::writeString(builder, value) + "\n";
}

double maximumAbsolute(const Eigen::VectorXd &values) {
  return values.size() == 0 ? 0.0 : values.cwiseAbs().maxCoeff();
}

std::pair<double, double> poseError(const mcc::Pose &target,
                                    const mcc::Pose &actual) {
  return {
      (target.translation() - actual.translation()).norm(),
      Eigen::AngleAxisd(target.linear() * actual.linear().transpose()).angle()};
}

namespace proto = telemetry_proto;

int priorityNumber(mcc::PriorityLevel priority) {
  return static_cast<int>(priority) + 1;
}

int passNumber(mcc::HierarchicalSolvePass pass) {
  return static_cast<int>(pass) + 1;
}

const char *hierarchicalTaskKindName(mcc::HierarchicalTaskKind kind) {
  switch (kind) {
  case mcc::HierarchicalTaskKind::Position:
    return "position";
  case mcc::HierarchicalTaskKind::Orientation:
    return "orientation";
  case mcc::HierarchicalTaskKind::Posture:
    return "posture";
  }
  return "unknown";
}

const char *hierarchicalTaskTargetUnit(mcc::HierarchicalTaskKind kind) {
  return kind == mcc::HierarchicalTaskKind::Position ? "m" : "rad";
}

const char *hierarchicalTaskResidualUnit(mcc::HierarchicalTaskKind kind) {
  return kind == mcc::HierarchicalTaskKind::Position ? "m/s" : "rad/s";
}

double hierarchicalTaskTolerance(
    mcc::HierarchicalTaskKind kind, const SolverOptions &options) {
  return kind == mcc::HierarchicalTaskKind::Posture
             ? options.posture_preservation_tolerance
             : options.cartesian_preservation_tolerance;
}

proto::AttemptOutcome attemptOutcome(mcl::WorkerIterationOutcome outcome) {
  switch (outcome) {
  case mcl::WorkerIterationOutcome::Accepted:
    return proto::ACCEPTED;
  case mcl::WorkerIterationOutcome::Idle:
    return proto::IDLE;
  case mcl::WorkerIterationOutcome::RecoverableRejected:
    return proto::RECOVERABLE_REJECTED;
  case mcl::WorkerIterationOutcome::FatalRejected:
    return proto::FATAL_REJECTED;
  }
  return proto::UNSPECIFIED;
}

template <typename Message>
TelemetryRecord telemetryRecord(EventStamp stamp, const char *topic,
                                Message message) {
  return TelemetryRecord::encoded(
      stamp, topic, std::make_unique<Message>(std::move(message)));
}

proto::SolverTelemetry makeSolverTelemetry(
    const proto::SampleContext &context, const char *solver_kind,
    const SolverDiagnostics &diagnostics, const mcl::SolverDebug &debug,
    const SolverOptions &options) {
  proto::SolverTelemetry message;
  *message.mutable_context() = context;
  message.set_solver_kind(solver_kind);
  message.set_backend(debug.backend);
  message.set_joint_limit_policy(debug.joint_limit_policy);
  message.set_status(debug.disposition);
  message.set_native_status(debug.native_status);
  message.set_solve_time_ms(diagnostics.solve_time_ms);
  message.set_maximum_hard_violation(diagnostics.maximum_hard_violation);

  if (diagnostics.hierarchical) {
    if (diagnostics.hierarchy.highest_completed_priority.has_value()) {
      message.set_highest_completed_priority(
          priorityNumber(*diagnostics.hierarchy.highest_completed_priority));
    }
    if (diagnostics.hierarchy.same_tick_fallback_level.has_value()) {
      message.set_fallback_priority(
          priorityNumber(*diagnostics.hierarchy.same_tick_fallback_level));
    }
    for (const auto &source : diagnostics.hierarchy.passes) {
      auto *pass = message.add_passes();
      pass->set_label(hierarchicalPassName(source.pass));
      pass->set_priority(source.pass == mcc::HierarchicalSolvePass::Terminal
                             ? 0
                             : passNumber(source.pass));
      pass->set_pass(passNumber(source.pass));
      pass->set_attempted(source.attempted);
      pass->set_succeeded(source.succeeded);
      pass->set_status(qpStatusName(source.backend_status));
      pass->set_native_status(source.native_status);
      pass->set_solve_time_ms(source.solve_time_ms);
      pass->set_iterations(source.iterations);
      pass->set_warm_start_used(source.warm_start_used);
      pass->set_objective_value(source.objective_value);
      pass->set_primal_residual(source.primal_residual);
      pass->set_dual_residual(source.dual_residual);
      double maximum_violation = 0.0;
      for (const auto &violation : source.constraint_violations) {
        auto *output = pass->add_constraint_violations();
        output->set_label(
            violation.source_name + "." + violation.component_name + "." +
            hierarchicalBoundSideName(violation.bound_side));
        output->set_kind(hierarchicalConstraintKindName(violation.kind));
        output->set_bound_source(
            hierarchicalBoundSourceName(violation.bound_source));
        output->set_side(hierarchicalBoundSideName(violation.bound_side));
        output->set_source(violation.source_name);
        output->set_component(violation.component_name);
        output->set_unit(hierarchicalViolationUnit(violation));
        output->set_value(violation.value);
        output->set_lower(violation.lower);
        output->set_upper(violation.upper);
        output->set_violation(violation.violation);
        maximum_violation = std::max(maximum_violation, violation.violation);
      }
      pass->set_maximum_constraint_violation(maximum_violation);
    }
    for (const auto &source : diagnostics.hierarchy.tasks) {
      auto *task = message.add_tasks();
      task->set_name(source.name);
      task->set_kind(hierarchicalTaskKindName(source.kind));
      task->set_priority(priorityNumber(source.priority));
      task->set_enabled(source.enabled);
      task->set_target_error(source.target_error_norm);
      task->set_target_error_unit(hierarchicalTaskTargetUnit(source.kind));
      for (Eigen::Index index = 0; index < source.residual_optimum.size(); ++index) {
        auto *component = task->add_components();
        component->set_label(source.name + "[" + std::to_string(index) + "]");
        component->set_residual_optimum(source.residual_optimum[index]);
        if (index < source.actual_preservation_drift.size()) {
          component->set_preservation_drift(
              source.actual_preservation_drift[index]);
        }
        component->set_preservation_tolerance(
            hierarchicalTaskTolerance(source.kind, options));
        component->set_residual_unit(hierarchicalTaskResidualUnit(source.kind));
      }
    }
    for (const auto &source : diagnostics.hierarchy.task_scales) {
      auto *scale = message.add_task_scales();
      scale->set_label(source.name);
      scale->set_priority(priorityNumber(source.priority));
      scale->set_active(source.active);
      scale->set_scale(source.weighted_progress_scale);
      scale->set_preservation_drift(source.actual_preservation_drift);
      scale->set_degraded(source.degraded);
      scale->set_stuck(source.stuck);
    }
  } else {
    auto *pass = message.add_passes();
    pass->set_label("solve");
    pass->set_attempted(true);
    pass->set_succeeded(debug.disposition == "accepted");
    pass->set_status(debug.qp_status);
    pass->set_native_status(debug.native_status);
    pass->set_solve_time_ms(debug.qp_solve_time_ms);
    pass->set_iterations(debug.qp_iterations);
    pass->set_warm_start_used(debug.warm_start_used);
    pass->set_objective_value(debug.objective_value);
    pass->set_primal_residual(debug.primal_residual);
    pass->set_dual_residual(debug.dual_residual);
    pass->set_maximum_constraint_violation(debug.maximum_hard_violation);
    for (const auto &source : debug.task_scales) {
      auto *scale = message.add_task_scales();
      scale->set_label(source.name);
      scale->set_active(source.active);
      scale->set_scale(source.scale);
      scale->set_cost(source.cost);
      scale->set_degraded(source.degraded);
      scale->set_stuck(source.stuck);
    }
    for (const auto &source : debug.requirements) {
      auto *requirement = message.add_requirements();
      requirement->set_label(source.name);
      requirement->set_unit(source.unit);
      requirement->set_source(source.source);
      requirement->set_enabled(source.enabled);
      requirement->set_active(source.active);
      requirement->set_maximum_violation(source.maximum_violation);
      requirement->set_cost(source.cost);
    }
  }
  return message;
}

proto::CouplingTelemetry makeCouplingTelemetry(
    const proto::SampleContext &context, const SolverDiagnostics &diagnostics,
    const EventStamp &stamp) {
  proto::CouplingTelemetry message;
  *message.mutable_context() = context;
  message.set_producer("avoidance");
  message.set_consumer("ik");
  message.set_state(couplingStateName(diagnostics.coupling_state));
  message.set_source_attempt_revision(diagnostics.attempt_revision);
  message.set_source_value_revision(diagnostics.value_revision);
  message.set_consumed_value_revision(
      diagnostics.consumed_source_value_revision);
  const auto captured_time = diagnostics.captured_state_time_nanoseconds;
  if (captured_time >= 0 && stamp.run_time_ns >= static_cast<std::uint64_t>(captured_time)) {
    message.set_source_age_ms(
        static_cast<double>(stamp.run_time_ns - static_cast<std::uint64_t>(captured_time)) /
        1.0e6);
  }
  message.set_captured_state_sequence(diagnostics.captured_state_sequence);
  return message;
}

proto::WorkerTelemetry makeWorkerTelemetry(
    const proto::SampleContext &context, const char *role, double rate_hz,
    const mcl::PeriodicIterationTiming &timing,
    const mcl::PeriodicWorkerStatistics &statistics) {
  proto::WorkerTelemetry message;
  *message.mutable_context() = context;
  message.set_worker_role(role);
  message.set_configured_rate_hz(rate_hz);
  message.set_deadline_ms(timing.deadline_ms);
  message.set_release_lateness_ms(timing.release_lateness_ms);
  message.set_execution_ms(timing.execution_ms);
  message.set_solver_ms(statistics.latest_solver_ms);
  message.set_non_solver_ms(statistics.latest_non_solver_execution_ms);
  message.set_release_to_finish_ms(timing.release_to_finish_ms);
  message.set_overrun_ms(timing.overrun_ms);
  message.set_iteration_count(statistics.iteration_count);
  message.set_deadline_miss_count(statistics.deadline_miss_count);
  message.set_skipped_release_count(statistics.skipped_release_count);
  message.set_recoverable_rejection_count(
      statistics.recoverable_rejection_count);
  message.set_fatal_rejection_count(
      context.outcome() == proto::FATAL_REJECTED ? 1U : 0U);
  return message;
}

void setVector3(proto::Vector3 &output, const Eigen::Vector3d &value) {
  output.set_x(value.x());
  output.set_y(value.y());
  output.set_z(value.z());
}

void setCartesianError(proto::CartesianError &output,
                       const mcc::Pose &target, const mcc::Pose &actual) {
  const Eigen::Vector3d position = target.translation() - actual.translation();
  const Eigen::AngleAxisd rotation(target.linear() * actual.linear().transpose());
  const Eigen::Vector3d rotation_vector = rotation.axis() * rotation.angle();
  setVector3(*output.mutable_position_m(), position);
  setVector3(*output.mutable_rotation_vector_rad(), rotation_vector);
  output.set_position_norm_m(position.norm());
  output.set_rotation_norm_rad(rotation.angle());
}

proto::CartesianTracking makeCartesianTracking(
    const proto::SampleContext &context, const TargetSnapshot &goal,
    const TargetSnapshot &reference,
    const mcc::CartesianTrajectorySample &planner_sample,
    const mcc::Pose &raw_left, const mcc::Pose &raw_right,
    const mcc::Pose &executed_left, const mcc::Pose &executed_right) {
  proto::CartesianTracking message;
  *message.mutable_context() = context;
  const auto append = [&](const char *side, const mcc::Pose &arm_goal,
                          const mcc::Pose &arm_reference,
                          const mcc::Pose &raw, const mcc::Pose &executed,
                          std::size_t frame_index) {
    auto *arm = message.add_arms();
    arm->set_side(side);
    setCartesianError(*arm->mutable_goal_to_reference(), arm_goal, arm_reference);
    setCartesianError(*arm->mutable_reference_to_ik(), arm_reference, raw);
    setCartesianError(*arm->mutable_reference_to_execution(), arm_reference, executed);
    setCartesianError(*arm->mutable_ik_to_execution(), raw, executed);
    if (frame_index < planner_sample.frames.size()) {
      const auto &frame = planner_sample.frames[frame_index];
      setVector3(*arm->mutable_reference_linear_velocity_mps(),
                 frame.twist.head<3>());
      setVector3(*arm->mutable_reference_angular_velocity_radps(),
                 frame.twist.tail<3>());
      setVector3(*arm->mutable_reference_linear_acceleration_mps2(),
                 frame.acceleration.head<3>());
      setVector3(*arm->mutable_reference_angular_acceleration_radps2(),
                 frame.acceleration.tail<3>());
    }
  };
  append("left", goal.left, reference.left, raw_left, executed_left, 0U);
  append("right", goal.right, reference.right, raw_right, executed_right, 1U);
  return message;
}

proto::JointTracking makeJointTracking(
    const proto::SampleContext &context,
    const std::vector<std::string> &joint_names,
    const RedOutputSnapshot &output,
    const JointTargetLimits &limits) {
  proto::JointTracking message;
  *message.mutable_context() = context;
  std::vector<std::vector<proto::ProjectionKind>> projection_kinds(joint_names.size());
  for (const auto &event : output.projection.events) {
    proto::ProjectionKind kind = proto::PROJECTION_KIND_UNSPECIFIED;
    switch (event.component) {
    case ProjectionComponent::VelocityLimit:
      kind = proto::PROJECTION_KIND_VELOCITY_LIMIT;
      break;
    case ProjectionComponent::AccelerationLimit:
      kind = proto::PROJECTION_KIND_ACCELERATION_LIMIT;
      break;
    case ProjectionComponent::JerkStoppingEnvelope:
      kind = proto::PROJECTION_KIND_JERK_STOPPING_ENVELOPE;
      break;
    }
    projection_kinds.at(event.joint_index).push_back(kind);
  }
  for (std::size_t index = 0U; index < joint_names.size(); ++index) {
    auto *joint = message.add_joints();
    joint->set_name(joint_names[index]);
    joint->set_ik_position_rad(output.raw_ik_positions[index]);
    joint->set_ik_velocity_radps(output.raw_ik_velocities[index]);
    joint->set_raw_target_position_rad(output.raw_joint_target.positions.at(index));
    joint->set_raw_target_velocity_radps(output.raw_joint_target.velocities.at(index));
    joint->set_raw_target_acceleration_radps2(
        output.raw_joint_target.accelerations.at(index));
    joint->set_projected_target_position_rad(
        output.projected_joint_target.positions.at(index));
    joint->set_projected_target_velocity_radps(
        output.projected_joint_target.velocities.at(index));
    joint->set_projected_target_acceleration_radps2(
        output.projected_joint_target.accelerations.at(index));
    joint->set_execution_position_rad(output.state.positions[index]);
    joint->set_execution_velocity_radps(output.state.velocities[index]);
    joint->set_execution_acceleration_radps2(output.state.accelerations[index]);
    joint->set_execution_jerk_radps3(output.state.jerks[index]);
    joint->set_position_lower_rad(limits.position_lower.at(index));
    joint->set_position_upper_rad(limits.position_upper.at(index));
    joint->set_velocity_limit_radps(limits.max_velocity.at(index));
    joint->set_acceleration_limit_radps2(limits.max_acceleration.at(index));
    joint->set_jerk_limit_radps3(limits.max_jerk.at(index));
    joint->set_velocity_utilization(
        std::abs(output.state.velocities[index]) / limits.max_velocity.at(index));
    joint->set_acceleration_utilization(
        std::abs(output.state.accelerations[index]) /
        limits.max_acceleration.at(index));
    joint->set_jerk_utilization(
        std::abs(output.state.jerks[index]) / limits.max_jerk.at(index));
    joint->set_position_margin_rad(std::min(
        output.state.positions[index] - limits.position_lower.at(index),
        limits.position_upper.at(index) - output.state.positions[index]));
    for (const auto kind : projection_kinds[index]) {
      joint->add_projection_kinds(kind);
    }
  }
  return message;
}

proto::PlannerTelemetry makePlannerTelemetry(
    const proto::SampleContext &context, const char *kind,
    const char *algorithm, const char *synchronization,
    const char *operation, const mcc::PlanningDiagnostics &diagnostics,
    double sample_time_s) {
  proto::PlannerTelemetry message;
  *message.mutable_context() = context;
  message.set_planner_kind(kind);
  message.set_algorithm(algorithm);
  message.set_synchronization(synchronization);
  message.set_state(plannerStateName(diagnostics.state));
  message.set_operation(operation);
  message.set_duration_s(diagnostics.duration);
  message.set_sample_time_s(sample_time_s);
  message.set_sample_count(diagnostics.sample_count);
  message.set_calculation_time_ms(diagnostics.calculation_time_ms);
  return message;
}

proto::CollisionTelemetry makeCollisionTelemetry(
    const proto::SampleContext &context,
    const mcl::SelfCollisionDebug &collision) {
  proto::CollisionTelemetry message;
  *message.mutable_context() = context;
  message.set_minimum_distance_m(collision.minimum_distance_m);
  message.set_influence_distance_m(collision.influence_distance_m);
  message.set_minimum_before_m(collision.minimum_distance_before_m);
  message.set_minimum_after_m(collision.minimum_distance_after_m);
  message.set_margin_shortfall_m(collision.margin_shortfall_m);
  for (const auto &source : collision.pairs) {
    auto *pair = message.add_pairs();
    pair->set_label(source.first_link + "--" + source.second_link);
    pair->set_first_link(source.first_link);
    pair->set_second_link(source.second_link);
    pair->set_distance_before_m(source.distance_before_m);
    pair->set_distance_after_m(source.distance_after_m);
    pair->set_margin_shortfall_m(std::max(
        0.0, collision.minimum_distance_m - source.distance_after_m));
    pair->set_active(source.active);
  }
  return message;
}

template <typename Derived>
std::string traceEigenVector(const Eigen::MatrixBase<Derived> &values) {
  std::ostringstream output;
  output << '"' << std::setprecision(17);
  for (Eigen::Index index = 0; index < values.size(); ++index) {
    if (index != 0) {
      output << ';';
    }
    output << values[index];
  }
  output << '"';
  return output.str();
}

std::string traceStdVector(const std::vector<double> &values) {
  std::ostringstream output;
  output << '"' << std::setprecision(17);
  for (std::size_t index = 0U; index < values.size(); ++index) {
    if (index != 0U) {
      output << ';';
    }
    output << values[index];
  }
  output << '"';
  return output.str();
}

std::string tracePose(const mcc::Pose &pose) {
  const Eigen::Quaterniond orientation(pose.linear());
  std::ostringstream output;
  output << '"' << std::setprecision(17) << pose.translation().x() << ';'
         << pose.translation().y() << ';' << pose.translation().z() << ';'
         << orientation.x() << ';' << orientation.y() << ';' << orientation.z()
         << ';' << orientation.w() << '"';
  return output.str();
}

std::string traceProjectionEvents(
    const mcl::planned_hierarchical_step_otg::ProjectionDiagnostics &diagnostics,
    const std::vector<std::string> &joint_names) {
  std::ostringstream output;
  for (std::size_t index = 0U; index < diagnostics.events.size(); ++index) {
    if (index != 0U) {
      output << ';';
    }
    const auto &event = diagnostics.events[index];
    output << joint_names.at(event.joint_index) << ':'
           << mcl::planned_hierarchical_step_otg::projectionComponentName(
                  event.component)
           << ':' << std::setprecision(17) << event.original_value << "->"
           << event.applied_value;
  }
  return replay::csvEscape(output.str());
}

void appendCsvRow(std::ostringstream &output,
                  const std::vector<std::string> &fields) {
  for (std::size_t index = 0U; index < fields.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    output << fields[index];
  }
  output << '\n';
}

void applyReplayInitialState(const replay::LoadedReplay &loaded,
                             const mcl::R1RobotConfig &robot,
                             Eigen::VectorXd &positions,
                             Eigen::VectorXd &velocities) {
  if (!loaded.initial_joint_state.has_value()) {
    return;
  }
  const auto &source = *loaded.initial_joint_state;
  for (std::size_t index = 0; index < robot.joint_names.size(); ++index) {
    const auto iterator = std::find(source.names.begin(), source.names.end(),
                                    robot.joint_names[index]);
    if (iterator == source.names.end()) {
      throw std::runtime_error("initial JointState is missing " +
                               robot.joint_names[index]);
    }
    const std::size_t source_index =
        static_cast<std::size_t>(std::distance(source.names.begin(), iterator));
    positions(static_cast<Eigen::Index>(index)) =
        source.positions.at(source_index);
    // Replay starts a fresh accepted-state feedback chain; recorded velocity is
    // provenance only.
    velocities(static_cast<Eigen::Index>(index)) = 0.0;
  }
}

mcc::RobotState robotState(const StateSnapshot &state) {
  mcc::RobotState result;
  result.joint_positions = state.positions;
  result.joint_velocities = state.velocities;
  return result;
}

CapturedRobotState capturedState(const StateSnapshot &state) {
  return {robotState(state), state.sequence, state.monotonic_time_nanoseconds};
}

void addCartesianTargets(const CartesianHandles &handles,
                         const mcc::CartesianTrajectorySample &sample,
                         SolverRequest &request) {
  const auto &left = sample.frames.at(0);
  const auto &right = sample.frames.at(1);
  request.position_targets[0].position = left.pose.translation();
  request.position_targets[1].position = right.pose.translation();
  request.orientation_targets[0].orientation = left.pose.linear();
  request.orientation_targets[1].orientation = right.pose.linear();
  request.position_targets[0].feed_forward_velocity = left.twist.head<3>();
  request.position_targets[1].feed_forward_velocity = right.twist.head<3>();
  request.orientation_targets[0].feed_forward_angular_velocity =
      left.twist.tail<3>();
  request.orientation_targets[1].feed_forward_angular_velocity =
      right.twist.tail<3>();
  request.position_targets[0].handle = handles.left_position;
  request.position_targets[1].handle = handles.right_position;
  request.orientation_targets[0].handle = handles.left_orientation;
  request.orientation_targets[1].handle = handles.right_orientation;
}

void initializeCartesianRequest(const CartesianHandles &handles,
                                const mcc::FrameName &reference_frame_name,
                                SolverRequest &request) {
  request.reference_frame_name = reference_frame_name;
  request.position_targets.resize(2);
  request.orientation_targets.resize(2);
  request.position_targets[0].handle = handles.left_position;
  request.position_targets[1].handle = handles.right_position;
  request.orientation_targets[0].handle = handles.left_orientation;
  request.orientation_targets[1].handle = handles.right_orientation;
}

std::string statusDetail(const mcc::Status &status) {
  return status.message.empty() ? "solver returned a rejected result"
                                : status.message;
}

const mcc::RequirementDiagnostic *maximumViolatedHardRequirement(
    const mcc::OptimizationDiagnostics &diagnostics) {
  if (diagnostics.maximum_hard_violation <= 0.0) {
    return nullptr;
  }
  const mcc::RequirementDiagnostic *result = nullptr;
  double smallest_distance = std::numeric_limits<double>::infinity();
  for (const auto &requirement : diagnostics.requirements) {
    // Hard requirements do not accumulate a soft cost. Matching against the
    // independently computed maximum avoids selecting the soft coupling slot.
    if (!requirement.enabled || requirement.cost != 0.0) {
      continue;
    }
    const double distance = std::abs(requirement.maximum_violation -
                                     diagnostics.maximum_hard_violation);
    if (distance < smallest_distance) {
      smallest_distance = distance;
      result = &requirement;
    }
  }
  return result;
}

std::string rejectedAttemptDetail(const mcc::Status &status,
                                  const SolverDiagnostics &diagnostics) {
  if (diagnostics.hierarchical) {
    std::ostringstream output;
    output << statusDetail(status) << std::scientific << std::setprecision(9)
           << " maximum_hard_violation=" << diagnostics.maximum_hard_violation
           << " task_scales=[";
    for (std::size_t index = 0;
         index < diagnostics.hierarchy.task_scales.size(); ++index) {
      if (index != 0) {
        output << ',';
      }
      const auto &scale = diagnostics.hierarchy.task_scales[index];
      output << "{name=\"" << scale.name << "\",active=" << std::boolalpha
             << scale.active
             << ",weighted_progress_scale=" << scale.weighted_progress_scale
             << '}';
    }
    output << "] failed_pass_evidence=[";
    bool first_failed_pass = true;
    for (const auto &pass : diagnostics.hierarchy.passes) {
      if (!pass.attempted || pass.succeeded) {
        continue;
      }
      if (!first_failed_pass) {
        output << ',';
      }
      first_failed_pass = false;
      output << "{pass=" << hierarchicalPassName(pass.pass)
             << ",objective=" << pass.objective_value
             << ",primal_residual=" << pass.primal_residual
             << ",dual_residual=" << pass.dual_residual
             << ",last_iterate=" << std::boolalpha
             << pass.last_iterate_available
             << ",observed_violations=" << pass.constraint_violations.size();
      if (!pass.constraint_violations.empty()) {
        const auto &violation = pass.constraint_violations.front();
        output << ",largest={kind="
               << hierarchicalConstraintKindName(violation.kind)
               << ",bound_source="
               << hierarchicalBoundSourceName(violation.bound_source)
               << ",source=\"" << violation.source_name
               << "\",component=\"" << violation.component_name
               << "\",side=" << hierarchicalBoundSideName(violation.bound_side)
               << ",unit=" << hierarchicalViolationUnit(violation)
               << ",value=" << violation.value << ",lower=" << violation.lower
               << ",upper=" << violation.upper
               << ",violation=" << violation.violation << '}';
      }
      output << '}';
    }
    output << ']';
    return output.str();
  }
  const auto &kinematics = diagnostics.kinematics;
  const auto &optimization = kinematics.optimization;
  const auto *requirement = maximumViolatedHardRequirement(optimization);

  std::ostringstream output;
  output << statusDetail(status) << std::scientific << std::setprecision(9)
         << " maximum_hard_violation=" << optimization.maximum_hard_violation;
  if (requirement == nullptr) {
    output << " max_violated_requirement=<unavailable>"
           << " maximum_violation=<unavailable>"
           << " requirement_unit=<unavailable>"
           << " requirement_source=<unavailable>";
  } else {
    output << " max_violated_requirement=\"" << requirement->name << '"'
           << " maximum_violation=" << requirement->maximum_violation
           << " requirement_unit=\"" << requirement->unit << '"'
           << " requirement_source=\"" << requirement->source << '"';
  }

  output << " task_scales=[";
  for (std::size_t index = 0; index < optimization.task_scales.size();
       ++index) {
    if (index != 0) {
      output << ',';
    }
    const auto &scale = optimization.task_scales[index];
    output << "{name=\"" << scale.name << "\",active=" << std::boolalpha
           << scale.active << ",scale=" << scale.scale
           << ",degraded=" << scale.degraded << ",stuck=" << scale.stuck << '}';
  }
  output << "] position_errors=[";
  for (std::size_t index = 0; index < kinematics.position_errors.size();
       ++index) {
    if (index != 0) {
      output << ',';
    }
    const auto &error = kinematics.position_errors[index];
    output << "{frame=\"" << error.frame_name << "\",norm_m=" << error.norm_m
           << '}';
  }
  output << "] orientation_errors=[";
  for (std::size_t index = 0; index < kinematics.orientation_errors.size();
       ++index) {
    if (index != 0) {
      output << ',';
    }
    const auto &error = kinematics.orientation_errors[index];
    output << "{frame=\"" << error.frame_name
           << "\",norm_rad=" << error.norm_rad << '}';
  }
  output << "] saturated_joints=[";
  for (std::size_t index = 0; index < kinematics.saturated_joints.size();
       ++index) {
    if (index != 0) {
      output << ',';
    }
    output << '"' << kinematics.saturated_joints[index] << '"';
  }
  output << ']';
  return output.str();
}

TaskScaleSnapshot
taskScaleSnapshot(const mcc::HierarchicalTaskScaleDiagnostics &diagnostic) {
  return TaskScaleSnapshot{diagnostic.active,
                           diagnostic.weighted_progress_scale,
                           diagnostic.degraded, diagnostic.stuck};
}

void fillRedDiagnostics(const CartesianHandles &handles,
                        const SolverDiagnostics &diagnostics,
                        RedOutputSnapshot &output) {
  for (const auto &task : diagnostics.hierarchy.tasks) {
    if (task.kind == mcc::HierarchicalTaskKind::Position) {
      if (task.handle_value == handles.left_position.value) {
        output.left_position_error_m = task.target_error_norm;
      } else if (task.handle_value == handles.right_position.value) {
        output.right_position_error_m = task.target_error_norm;
      }
    } else if (task.kind == mcc::HierarchicalTaskKind::Orientation) {
      if (task.handle_value == handles.left_orientation.value) {
        output.left_orientation_error_rad = task.target_error_norm;
      } else if (task.handle_value == handles.right_orientation.value) {
        output.right_orientation_error_rad = task.target_error_norm;
      }
    }
  }
  const auto &scales = diagnostics.hierarchy.task_scales;
  output.left_scale = taskScaleSnapshot(scales.at(0));
  output.right_scale = taskScaleSnapshot(scales.at(1));
}

const char *taskScaleClassification(const TaskScaleSnapshot &scale) {
  if (!scale.active) {
    return "inactive";
  }
  if (scale.stuck) {
    return "stuck";
  }
  if (scale.degraded) {
    return "degraded";
  }
  return "full";
}

std::string taskScaleStatus(const RedOutputSnapshot &output) {
  std::ostringstream status;
  status << std::fixed << std::setprecision(3)
         << "scale L=" << output.left_scale.scale << '('
         << taskScaleClassification(output.left_scale)
         << ") R=" << output.right_scale.scale << '('
         << taskScaleClassification(output.right_scale) << ')';
  return status.str();
}

void fillSelfCollisionDebug(
    const StateSnapshot &input_state,
    const mcc::SelfCollisionDiagnostics &diagnostics,
    const mcl::planned_hierarchical_step_otg::SolverOptions &options,
    mcl::SelfCollisionDebug &output) {
  output.label = "Yellow self-collision";
  output.input_state_sequence = input_state.sequence;
  output.minimum_distance_m = options.minimum_collision_distance_m;
  output.influence_distance_m = options.collision_influence_distance_m;
  output.minimum_distance_before_m = diagnostics.minimum_distance_before_m;
  output.minimum_distance_after_m = diagnostics.minimum_distance_after_m;
  output.margin_shortfall_m = diagnostics.margin_shortfall_m;
  output.input_joint_positions = toStdVector(input_state.positions);
  output.pairs.resize(diagnostics.pairs.size());
  for (std::size_t index = 0; index < diagnostics.pairs.size(); ++index) {
    const auto &source = diagnostics.pairs[index];
    auto &destination = output.pairs[index];
    destination.first_link = source.link_pair.first_link;
    destination.second_link = source.link_pair.second_link;
    destination.distance_before_m = source.distance_before_m;
    destination.distance_after_m = source.distance_after_m;
    destination.active = source.active;
  }
}

} // namespace

namespace {

motion_control::viz::PoseSample
makeVisualizationPose(const char *channel, const std::string &reference_frame,
                      const Eigen::Isometry3d &pose) {
  const Eigen::Quaterniond orientation(pose.linear());
  motion_control::viz::PoseSample result;
  result.channel = channel;
  result.frame_id = reference_frame;
  result.pose.position_m = {pose.translation().x(), pose.translation().y(),
                            pose.translation().z()};
  result.pose.orientation_xyzw = {orientation.x(), orientation.y(),
                                  orientation.z(), orientation.w()};
  return result;
}

} // namespace

void appendPlanningRequestPoses(motion_control::viz::RenderBatch &frame,
                                const std::string &reference_frame,
                                const Eigen::Isometry3d &left_pose,
                                const Eigen::Isometry3d &right_pose) {
  namespace contract = contracts::mcl_planning_v1;
  frame.poses.reserve(frame.poses.size() + 2U);
  frame.poses.push_back(makeVisualizationPose(
      contract::kLeftCartesianReferenceTopic, reference_frame, left_pose));
  frame.poses.push_back(makeVisualizationPose(
      contract::kRightCartesianReferenceTopic, reference_frame, right_pose));
}

void appendOtgExecution(motion_control::viz::RenderBatch &batch,
                        const std::vector<std::string> &joint_names,
                        const std::vector<double> &positions,
                        const std::vector<double> &velocities,
                        const std::string &reference_frame,
                        const Eigen::Isometry3d &left_pose,
                        const Eigen::Isometry3d &right_pose) {
  namespace contract = contracts::mcl_execution_v1;
  batch.joint_states.push_back(motion_control::viz::JointStateSample{
      contract::kJointExecutionTopic, joint_names, positions,
      velocities});
  batch.poses.push_back(makeVisualizationPose(contract::kLeftCartesianExecutionTopic,
                                              reference_frame, left_pose));
  batch.poses.push_back(makeVisualizationPose(contract::kRightCartesianExecutionTopic,
                                              reference_frame, right_pose));
}

int runLoop(Options planned_options, const R1RobotConfig &robot,
            SolverRuntime &solver, const SolverHandles &handles,
            mcc::CartesianPlanner &cartesian_planner,
            mcc::JointPlanner &joint_planner,
            const JointTargetLimits &joint_otg_limits,
            const std::vector<std::size_t> &active_joint_full_indices,
            std::string &normal_exit_detail) {
  const auto &options = planned_options.interactive;
  const auto affinity_domain = mcl::CpuAffinityDomain::capture();
  affinity_domain.validate(kProgramId, "ui", kUiCpuAffinity);
  affinity_domain.validate(kProgramId, "red", kRedCpuAffinity);
  affinity_domain.validate(kProgramId, "yellow", kYellowCpuAffinity);
  const auto ui_affinity_binding =
      affinity_domain.bindCurrentThread(kProgramId, "ui", kUiCpuAffinity);
  std::optional<replay::LoadedReplay> loaded_replay;
  std::optional<replay::ReplaySource> replay_source;
  if (planned_options.replay.has_value()) {
    auto &replay_options = *planned_options.replay;
    if (!replay_options.output_dir_explicit) {
      const std::string run_id = replay_options.run_id.value_or(
          mcl::make_run_id(mcl::sha256_file(replay_options.input_path)));
      const std::filesystem::path output_root =
          replay_options.output_root.value_or(
              std::filesystem::path{"runs/mcl_planned_hierarchical_step_otg"});
      replay_options.output_dir = output_root / run_id;
    }
    loaded_replay = replay::loadReplay(replay_options);
    if (loaded_replay->timeline.timeline.empty()) {
      throw std::runtime_error("replay timeline is empty");
    }
    replay_source.emplace(*loaded_replay, replay_options.execution_mode,
                          replay_options.playback_rate,
                          planned_options.start_paused);
    replay::createOutputDirectory(replay_options.output_dir);
  }
  const auto &joint_names = robot.joint_names;
  const Eigen::VectorXd initial_positions = toEigen(robot.default_positions);
  StateSnapshot initial_state;
  initial_state.sequence = 1;
  initial_state.monotonic_time_nanoseconds = 1;
  initial_state.positions = initial_positions;
  initial_state.velocities.setZero(initial_positions.size());
  initial_state.accelerations.setZero(initial_positions.size());
  initial_state.jerks.setZero(initial_positions.size());
  if (loaded_replay.has_value()) {
    applyReplayInitialState(*loaded_replay, robot, initial_state.positions,
                            initial_state.velocities);
  }

  mcc::ForwardKinematicsRequest initial_fk_request;
  initial_fk_request.state = robotState(initial_state);
  initial_fk_request.frame_names = {robot.left_end_effector_frame,
                                    robot.right_end_effector_frame};
  initial_fk_request.reference_frame_name = robot.base_frame;
  mcc::ForwardKinematicsSolution initial_fk;
  mcc::ForwardKinematicsDiagnostics initial_fk_diagnostics;
  requireOk(solver.computeForwardKinematics(initial_fk_request, initial_fk,
                                            initial_fk_diagnostics),
            "Initial FK failed");

  TargetSnapshot warmup_target;
  warmup_target.revision = 0;
  warmup_target.left =
      requirePose(initial_fk.poses, robot.left_end_effector_frame).pose;
  warmup_target.right =
      requirePose(initial_fk.poses, robot.right_end_effector_frame).pose;
  TargetSnapshot first_replay_target = warmup_target;
  if (loaded_replay.has_value()) {
    const auto &first = replay_source->sourceFrame();
    first_replay_target.replay_source_index = replay_source->sourceIndex();
    first_replay_target.left =
        first.value.left.pose * robot.left_tcp_offset.inverse();
    first_replay_target.right =
        first.value.right.pose * robot.right_tcp_offset.inverse();
  }
  TargetSnapshot initial_target =
      loaded_replay.has_value() && planned_options.start_paused
          ? warmup_target
          : first_replay_target;
  initial_target.revision = 1;
  if (loaded_replay.has_value() && planned_options.start_paused) {
    initial_target.replay_joint_hold = true;
  }

  RedOutputSnapshot initial_output;
  initial_output.accepted_target = warmup_target;
  initial_output.source_goal = initial_target;
  initial_output.state = initial_state;
  initial_output.raw_ik_positions = initial_state.positions;
  initial_output.raw_ik_velocities = initial_state.velocities;
  initial_output.raw_joint_target.positions =
      toStdVector(initial_state.positions);
  initial_output.raw_joint_target.velocities =
      toStdVector(initial_state.velocities);
  initial_output.raw_joint_target.accelerations.assign(joint_names.size(), 0.0);
  initial_output.projected_joint_target = initial_output.raw_joint_target;
  initial_output.raw_left_pose = warmup_target.left;
  initial_output.raw_right_pose = warmup_target.right;
  initial_output.left_pose = warmup_target.left;
  initial_output.right_pose = warmup_target.right;
  initial_output.accepted_planner_sample.frames.resize(2);
  initial_output.accepted_planner_sample.frames[0].reference_frame_name =
      robot.base_frame;
  initial_output.accepted_planner_sample.frames[0].frame_name =
      robot.left_end_effector_frame;
  initial_output.accepted_planner_sample.frames[0].pose = warmup_target.left;
  initial_output.accepted_planner_sample.frames[1].reference_frame_name =
      robot.base_frame;
  initial_output.accepted_planner_sample.frames[1].frame_name =
      robot.right_end_effector_frame;
  initial_output.accepted_planner_sample.frames[1].pose = warmup_target.right;
  initial_output.planner_state = mcc::PlanningState::Finished;
  mcc::SelfCollisionDiagnostics initial_collision_diagnostics;
  mcl::SelfCollisionDebug initial_collision_debug;
  mcl::SolverDebug initial_yellow_solver_debug;

  // Warm all numerical workspaces and the coupling path before deadlines apply.
  solver.beginRun(1);
  {
    SolverSolution solution;
    SolverDiagnostics diagnostics;
    SolverRequest yellow;
    yellow.reference_frame_name = robot.base_frame;
    yellow.captured_state = capturedState(initial_state);
    auto status = solver.solveYellow(yellow, solution, diagnostics);
    if (!status.ok()) {
      throw std::runtime_error("Yellow warm-up failed: " +
                               rejectedAttemptDetail(status, diagnostics));
    }
    requireOk(
        solver.getSelfCollisionDiagnostics(handles.yellow_collision,
                                           initial_collision_diagnostics),
        "Failed to query Yellow self-collision diagnostics after warm-up");
    fillSelfCollisionDebug(initial_state, initial_collision_diagnostics,
                           options.solver, initial_collision_debug);
    initial_yellow_solver_debug = makeSolverDebug(
        "Yellow", diagnostics, solution.kinematics_solution.disposition);

    SolverRequest red;
    initializeCartesianRequest(handles.red, robot.base_frame, red);
    red.captured_state = capturedState(initial_state);
    addCartesianTargets(handles.red, initial_output.accepted_planner_sample,
                        red);
    status = solver.solveRed(red, solution, diagnostics);
    if (!status.ok()) {
      throw std::runtime_error("Red warm-up failed: " +
                               rejectedAttemptDetail(status, diagnostics));
    }
    initial_output.solve_time_ms = diagnostics.solve_time_ms;
    initial_output.iterations = diagnostics.iterations;
    initial_output.converged = diagnostics.converged;
    fillRedDiagnostics(handles.red, diagnostics, initial_output);
    initial_output.solver_debug = makeSolverDebug(
        "Red", diagnostics, solution.kinematics_solution.disposition);
  }
  solver.beginRun(2);

  RedAttemptSnapshot initial_red_attempt;
  initial_red_attempt.target = initial_target;
  initial_red_attempt.attempted_reference = warmup_target;
  initial_red_attempt.solver_debug = initial_output.solver_debug;

  const auto initial_red_affinity =
      affinity_domain.describe(kProgramId, "red", kRedCpuAffinity);
  const auto initial_yellow_affinity =
      affinity_domain.describe(kProgramId, "yellow", kYellowCpuAffinity);

  mcl::LatestValueMailbox<TargetSnapshot> target_to_red(initial_target);
  mcl::LatestValueMailbox<StateSnapshot> state_to_yellow(initial_state);
  mcl::LatestValueMailbox<RedOutputSnapshot> output_to_ui(initial_output);
  mcl::LatestValueMailbox<RedAttemptSnapshot> red_attempt_to_ui(
      initial_red_attempt);
  mcl::LatestValueMailbox<mcl::SelfCollisionDebug> collision_to_ui(
      initial_collision_debug);
  mcl::LatestValueMailbox<mcl::SolverDebug> yellow_solver_to_ui(
      initial_yellow_solver_debug);
  mcl::LatestValueMailbox<mcl::CpuAffinityBinding> red_affinity_to_ui(
      initial_red_affinity);
  mcl::LatestValueMailbox<mcl::CpuAffinityBinding> yellow_affinity_to_ui(
      initial_yellow_affinity);
  target_to_red.publish(initial_target);
  state_to_yellow.publish(initial_state);
  output_to_ui.publish(initial_output);
  red_attempt_to_ui.publish(initial_red_attempt);
  collision_to_ui.publish(initial_collision_debug);
  yellow_solver_to_ui.publish(initial_yellow_solver_debug);

  const auto presentation =
      mcl::makeArmPresentation(robot, mcl::foxgloveIkVisualizationChannels());
  mcl::TerminalFrontend terminal(
      {planned_options.source_mode == SourceMode::Teleop ||
           (planned_options.replay.has_value() &&
            planned_options.replay->terminal_input_enabled),
       options.presentation.enabled});
  mcl::KeyboardTargetSource input(terminal,
                                  planned_options.source_mode ==
                                          SourceMode::Replay
                                      ? mcl::KeyboardSourceMode::Replay
                                      : mcl::KeyboardSourceMode::Teleop,
                                  options.tui,
                                  {{mcl::ArmSide::Left, initial_target.left},
                                   {mcl::ArmSide::Right, initial_target.right}},
                                  true);
  mcl::PlannedGroupedTui tui(options.presentation);
  if (planned_options.source_mode == SourceMode::Replay) {
    input.setMotionInputEnabled(false, "Replay motion editing is disabled");
    if (planned_options.start_paused) {
      input.setPaused(true, "Replay timeline paused; press space to start");
    }
  }
  auto visualization_sink =
      mcl::createPreviewSink(options.visualization, kProgramId);
  const bool telemetry_enabled = options.visualization.enabled;
  std::unique_ptr<RunClock> telemetry_clock;
  std::unique_ptr<TelemetryEncoder> telemetry_encoder;
  std::unique_ptr<WorkerTelemetryQueue> red_telemetry;
  std::unique_ptr<WorkerTelemetryQueue> yellow_telemetry;
  if (telemetry_enabled) {
    telemetry_clock = std::make_unique<RunClock>();
    telemetry_encoder = std::make_unique<TelemetryEncoder>();
    red_telemetry = std::make_unique<WorkerTelemetryQueue>();
    yellow_telemetry = std::make_unique<WorkerTelemetryQueue>();
  }

  mcl::WorkerStopController stop_controller;
  mcl::GroupedFaultState fault;
  mcl::PeriodicWorkerDiagnostics red_worker_diagnostics;
  mcl::PeriodicWorkerDiagnostics yellow_worker_diagnostics;
  mcl::RollingPercentiles red_solve_time_percentiles;
  mcl::RollingPercentiles yellow_solve_time_percentiles;
  QpPassTimePercentiles red_qp_pass_time_percentiles;
  WorkerThreads workers(stop_controller);
  std::mutex replay_trace_mutex;
  std::ostringstream replay_trace;
  std::size_t replay_accepted_solve_count = 0;
  std::size_t replay_rejected_solve_count = 0;
  std::atomic<std::uint64_t> replay_last_consumed_revision{0};
  std::atomic<std::size_t> replay_settled_cycle_count{0U};
  std::atomic_bool replay_settled{false};
  if (planned_options.replay_trace_enabled) {
    replay_trace
        << "attempt,source_revision,original_logical_timestamp_ns,"
           "source_time_from_start_ns,"
           "projected_timestamp_ns,left_header_stamp_ns,left_log_time_"
           "ns,left_publish_time_ns,"
           "right_header_stamp_ns,right_log_time_ns,right_publish_time_"
           "ns,accepted,failure_layer,solver_status,joint_target_mode,"
           "solve_time_ms,maximum_hard_violation,goal_left_xyz,"
           "reference_left_xyz,"
           "reference_left_twist,reference_left_acceleration,raw_fk_left_pose,"
           "raw_fk_right_pose,otg_fk_left_pose,otg_fk_right_pose,"
           "raw_ik_positions,raw_ik_velocities,"
           "raw_target_positions,raw_target_velocities,raw_target_accelerations,"
           "projected_target_positions,projected_target_velocities,"
           "projected_target_accelerations,otg_positions,otg_velocities,"
           "otg_accelerations,otg_jerks,future_o1_startup,"
           "joint_planner_state,joint_trajectory_duration_s,"
           "joint_plan_time_ms,joint_step_time_ms,projection_events,"
           "projection_event_count,projection_cycle_count,deadline_miss_count,"
           "skipped_release_count,replay_settled_cycles\n";
  }

  visualization_sink->open();
  mcl::installRuntimeSignalHandlers();

  workers.yellow = std::thread([&]() {
    const auto affinity_binding = affinity_domain.bindCurrentThread(
        kProgramId, "yellow", kYellowCpuAffinity);
    yellow_affinity_to_ui.publish(affinity_binding);
    StateSnapshot state = initial_state;
    SolverRequest request;
    request.reference_frame_name = robot.base_frame;
    SolverSolution solution;
    SolverDiagnostics diagnostics;
    mcc::SelfCollisionDiagnostics collision_diagnostics =
        initial_collision_diagnostics;
    mcl::SelfCollisionDebug collision_debug = initial_collision_debug;
    mcl::SolverDebug solver_debug = initial_yellow_solver_debug;
    EventStamp telemetry_stamp;
    proto::SampleContext telemetry_context;
    mcl::PeriodicIterationObserver telemetry_observer;
    bool deadline_miss_active = false;
    if (telemetry_enabled) {
      telemetry_observer = [&](const mcl::WorkerIterationResult &result,
                               const mcl::PeriodicIterationTiming &timing,
                               const mcl::PeriodicWorkerStatistics &statistics) {
        auto context = telemetry_context;
        context.set_outcome(attemptOutcome(result.outcome));
        context.set_committed(result.outcome == mcl::WorkerIterationOutcome::Accepted);
        yellow_telemetry->tryPush(telemetryRecord(
            telemetry_stamp, contracts::mcl_telemetry_v1::kAvoidanceWorkerTopic,
            makeWorkerTelemetry(context, "avoidance", options.yellow_rate_hz,
                                timing, statistics)));
        const bool deadline_missed = timing.overrun_ms > 0.0;
        if (deadline_missed && !deadline_miss_active) {
          yellow_telemetry->tryPush(TelemetryRecord::log(
              telemetry_stamp, contracts::mcl_telemetry_v1::kEventsTopic,
              motion_control::viz::LogLevel::Warning,
              "avoidance-deadline-miss",
              "avoidance worker overrun_ms=" +
                  std::to_string(timing.overrun_ms)));
        }
        deadline_miss_active = deadline_missed;
      };
    }
    mcl::runPeriodicWorker(
        {mcl::WorkerGroup::Yellow, options.yellow_rate_hz,
         options.deadline_policy},
        stop_controller, fault, yellow_worker_diagnostics,
        [&](double, std::int64_t) {
          if (telemetry_enabled) {
            telemetry_stamp = telemetry_clock->sample();
          }
          state_to_yellow.readLatest(state);
          request.captured_state = capturedState(state);
          const auto status =
              solver.solveYellow(request, solution, diagnostics);
          yellow_solve_time_percentiles.record(diagnostics.solve_time_ms);
          const bool accepted = status.ok();
          if (accepted) {
            requireOk(
                solver.getSelfCollisionDiagnostics(handles.yellow_collision,
                                                   collision_diagnostics),
                "Failed to query accepted Yellow self-collision diagnostics");
            fillSelfCollisionDebug(state, collision_diagnostics, options.solver,
                                   collision_debug);
            collision_to_ui.publish(collision_debug);
          }
          updateSolverDebug(solver_debug, diagnostics,
                            accepted ? mcc::ResultDisposition::Accepted
                                     : mcc::ResultDisposition::Rejected);
          yellow_solver_to_ui.publish(solver_debug);
          if (telemetry_enabled) {
            telemetry_context = makeSampleContext(
                telemetry_stamp,
                accepted ? proto::ACCEPTED
                         : proto::FATAL_REJECTED,
                accepted, diagnostics.run_generation, 0U,
                diagnostics.attempt_revision, diagnostics.value_revision);
            yellow_telemetry->tryPush(telemetryRecord(
                telemetry_stamp,
                contracts::mcl_telemetry_v1::kAvoidanceSolverTopic,
                makeSolverTelemetry(telemetry_context, "avoidance",
                                    diagnostics, solver_debug,
                                    options.solver)));
            if (accepted) {
              yellow_telemetry->tryPush(telemetryRecord(
                  telemetry_stamp,
                  contracts::mcl_telemetry_v1::kCollisionTopic,
                  makeCollisionTelemetry(telemetry_context, collision_debug)));
            } else {
              yellow_telemetry->tryPush(TelemetryRecord::log(
                  telemetry_stamp, contracts::mcl_telemetry_v1::kEventsTopic,
                  motion_control::viz::LogLevel::Error, "avoidance-rejection",
                  rejectedAttemptDetail(status, diagnostics)));
            }
          }
          return mcl::WorkerIterationResult{
              accepted ? mcl::WorkerIterationOutcome::Accepted
                       : mcl::WorkerIterationOutcome::FatalRejected,
              diagnostics.attempt_revision, diagnostics.solve_time_ms,
              accepted ? std::string{}
                       : rejectedAttemptDetail(status, diagnostics)};
        },
        telemetry_observer);
  });

  workers.red = std::thread([&]() {
    const auto affinity_binding =
        affinity_domain.bindCurrentThread(kProgramId, "red", kRedCpuAffinity);
    red_affinity_to_ui.publish(affinity_binding);
    // Keep the accepted raw-IK reference independent from the jerk-limited
    // execution state. Red advances from ik_state, while JointPlanner and
    // Yellow advance from otg_state.
    StateSnapshot ik_state = initial_state;
    StateSnapshot otg_state = initial_state;
    TargetSnapshot target = initial_target;
    RedOutputSnapshot output = initial_output;
    SolverRequest request;
    initializeCartesianRequest(handles.red, robot.base_frame, request);
    SolverSolution solution;
    SolverDiagnostics diagnostics;
    RedAttemptSnapshot attempt = initial_red_attempt;
    std::optional<std::uint64_t> rejected_target_revision;
    mcc::PlanningDiagnostics planning_diagnostics;
    mcc::CartesianTrajectorySample accepted_planner_sample =
        initial_output.accepted_planner_sample;
    std::optional<mcc::CartesianTrajectorySample> staged_planner_sample;
    std::uint64_t planned_goal_revision = 0;

    mcl::planned_hierarchical_step_otg::JointTargetBuilder joint_target_builder(
        planned_options.joint_target, 1.0 / options.red_rate_hz,
        joint_names.size());
    mcl::planned_hierarchical_step_otg::ReplaySettlingCounter replay_settling(
        planned_options.replay_settling);
    EventStamp telemetry_stamp;
    proto::SampleContext telemetry_context;
    mcl::PeriodicIterationObserver telemetry_observer;
    bool deadline_miss_active = false;
    if (telemetry_enabled) {
      telemetry_observer = [&](const mcl::WorkerIterationResult &result,
                               const mcl::PeriodicIterationTiming &timing,
                               const mcl::PeriodicWorkerStatistics &statistics) {
        auto context = telemetry_context;
        context.set_outcome(attemptOutcome(result.outcome));
        context.set_committed(result.outcome == mcl::WorkerIterationOutcome::Accepted);
        red_telemetry->tryPush(telemetryRecord(
            telemetry_stamp, contracts::mcl_telemetry_v1::kControlWorkerTopic,
            makeWorkerTelemetry(context, "control", options.red_rate_hz,
                                timing, statistics)));
        const bool deadline_missed = timing.overrun_ms > 0.0;
        if (deadline_missed && !deadline_miss_active) {
          red_telemetry->tryPush(TelemetryRecord::log(
              telemetry_stamp, contracts::mcl_telemetry_v1::kEventsTopic,
              motion_control::viz::LogLevel::Warning,
              "control-deadline-miss",
              "control worker overrun_ms=" +
                  std::to_string(timing.overrun_ms)));
        }
        deadline_miss_active = deadline_missed;
      };
    }

    const auto makeReplayRow = [&](std::uint64_t attempt_revision) {
      std::vector<std::string> row(48U);
      row[0] = std::to_string(attempt_revision);
      row[14] = mcl::planned_hierarchical_step_otg::jointTargetModeName(
          planned_options.joint_target.mode);
      if (!loaded_replay.has_value() ||
          !target.replay_source_index.has_value()) {
        return row;
      }
      const std::size_t source_index = std::min(
          *target.replay_source_index,
          loaded_replay->timeline.timeline.size() - 1U);
      const auto &source = loaded_replay->timeline.timeline.at(source_index);
      row[1] = std::to_string(source.sequence);
      row[2] = std::to_string(source.original_logical_time_ns);
      row[3] = std::to_string(source.source_time_from_start_ns);
      row[4] = std::to_string(source.projected_time_ns);
      row[5] =
          replay::optionalTimestamp(source.value.left.time.header_stamp_ns);
      row[6] = replay::optionalTimestamp(source.value.left.time.log_time_ns);
      row[7] =
          replay::optionalTimestamp(source.value.left.time.publish_time_ns);
      row[8] =
          replay::optionalTimestamp(source.value.right.time.header_stamp_ns);
      row[9] = replay::optionalTimestamp(source.value.right.time.log_time_ns);
      row[10] =
          replay::optionalTimestamp(source.value.right.time.publish_time_ns);
      return row;
    };
    const auto recordFailure = [&](const std::string &layer,
                                   const std::string &detail) {
      if (telemetry_enabled) {
        red_telemetry->tryPush(TelemetryRecord::log(
            telemetry_stamp, contracts::mcl_telemetry_v1::kEventsTopic,
            motion_control::viz::LogLevel::Error, layer, detail));
      }
      if (!loaded_replay.has_value()) {
        return;
      }
      ++replay_rejected_solve_count;
      if (!planned_options.replay_trace_enabled) {
        return;
      }
      auto row = makeReplayRow(target.revision);
      row[11] = "false";
      row[12] = layer;
      row[13] = replay::csvEscape(detail);
      row[17] = traceEigenVector(target.left.translation());
      row[23] = tracePose(output.left_pose);
      row[24] = tracePose(output.right_pose);
      row[33] = traceEigenVector(otg_state.positions);
      row[34] = traceEigenVector(otg_state.velocities);
      row[35] = traceEigenVector(otg_state.accelerations);
      row[36] = traceEigenVector(otg_state.jerks);
      const auto worker_stats = red_worker_diagnostics.snapshot();
      row[45] = std::to_string(worker_stats.deadline_miss_count);
      row[46] = std::to_string(worker_stats.skipped_release_count);
      row[47] = std::to_string(replay_settled_cycle_count.load());
      std::lock_guard<std::mutex> lock(replay_trace_mutex);
      appendCsvRow(replay_trace, row);
    };

    mcl::runPeriodicWorker(
        {mcl::WorkerGroup::Red, options.red_rate_hz, options.deadline_policy},
        stop_controller, fault, red_worker_diagnostics,
        [&](double, std::int64_t sample_time_ns) {
          if (telemetry_enabled) {
            telemetry_stamp = telemetry_clock->sample();
            attempt.event_timestamp_ns = telemetry_stamp.timestamp_ns;
            attempt.run_time_ns = telemetry_stamp.run_time_ns;
          }
          if (target_to_red.readLatest(target) && loaded_replay.has_value()) {
            replay_last_consumed_revision.store(target.revision);
          }
          if (telemetry_enabled) {
            telemetry_context = makeSampleContext(
                telemetry_stamp, proto::IDLE, false, 2U,
                target.revision, diagnostics.attempt_revision,
                diagnostics.value_revision);
          }
          if (rejected_target_revision.has_value() &&
              target.revision == *rejected_target_revision) {
            return mcl::WorkerIterationResult{mcl::WorkerIterationOutcome::Idle,
                                              diagnostics.attempt_revision,
                                              0.0,
                                              {}};
          }
          rejected_target_revision.reset();
          if (target.revision != planned_goal_revision) {
            auto retarget_request = makeRetargetRequest(
                target.left, target.right, accepted_planner_sample, robot,
                planned_options.planning, options.red_rate_hz);
            attempt.retarget_clamp =
                mcl::planned_hierarchical_step_otg::clampRetargetCurrentState(
                    retarget_request);
            attempt.retarget_clamp_target_revision = target.revision;
            const auto planning_status = cartesian_planner.replan(
                retarget_request, planning_diagnostics);
            if (telemetry_enabled) {
              auto planner_context = telemetry_context;
              planner_context.set_outcome(
                  planning_status.ok() ? proto::ACCEPTED
                                       : proto::FATAL_REJECTED);
              red_telemetry->tryPush(telemetryRecord(
                  telemetry_stamp,
                  contracts::mcl_telemetry_v1::kCartesianPlannerTopic,
                  makePlannerTelemetry(
                      planner_context, "cartesian", "jerk_limited",
                      planningSynchronizationName(
                          planned_options.planning.cartesian_synchronization),
                      "replan", planning_diagnostics,
                      accepted_planner_sample.time_from_start)));
              if (attempt.retarget_clamp.clamped()) {
                red_telemetry->tryPush(TelemetryRecord::log(
                    telemetry_stamp,
                    contracts::mcl_telemetry_v1::kEventsTopic,
                    motion_control::viz::LogLevel::Warning,
                    "cartesian-projection", retargetClampDetail(attempt)));
              }
            }
            if (!planning_status.ok()) {
              attempt.state =
                  planned_options.source_mode == SourceMode::Teleop &&
                          planning_status.code == mcc::StatusCode::Infeasible
                      ? RedAttemptState::RecoverableRejected
                      : RedAttemptState::FatalRejected;
              attempt.target = target;
              attempt.detail =
                  "Cartesian replan failed: " + planning_status.message;
              recordFailure("cartesian-replan", attempt.detail);
              red_attempt_to_ui.publish(attempt);
              if (attempt.state == RedAttemptState::RecoverableRejected) {
                rejected_target_revision = target.revision;
              }
              if (telemetry_enabled) {
                telemetry_context.set_outcome(
                    attempt.state == RedAttemptState::RecoverableRejected
                        ? proto::RECOVERABLE_REJECTED
                        : proto::FATAL_REJECTED);
              }
              return mcl::WorkerIterationResult{
                  attempt.state == RedAttemptState::RecoverableRejected
                      ? mcl::WorkerIterationOutcome::RecoverableRejected
                      : mcl::WorkerIterationOutcome::FatalRejected,
                  target.revision, planning_diagnostics.calculation_time_ms,
                  attempt.detail};
            }
            planned_goal_revision = target.revision;
            staged_planner_sample.reset();
          }
          if (!staged_planner_sample.has_value()) {
            mcc::CartesianTrajectorySample next_sample;
            const auto planning_status =
                cartesian_planner.step(next_sample, planning_diagnostics);
            if (telemetry_enabled) {
              auto planner_context = telemetry_context;
              planner_context.set_outcome(
                  planning_status.ok() ? proto::ACCEPTED
                                       : proto::FATAL_REJECTED);
              red_telemetry->tryPush(telemetryRecord(
                  telemetry_stamp,
                  contracts::mcl_telemetry_v1::kCartesianPlannerTopic,
                  makePlannerTelemetry(
                      planner_context, "cartesian", "jerk_limited",
                      planningSynchronizationName(
                          planned_options.planning.cartesian_synchronization),
                      "step", planning_diagnostics,
                      planning_status.ok() ? next_sample.time_from_start
                                           : accepted_planner_sample.time_from_start)));
            }
            if (!planning_status.ok()) {
              attempt.state = RedAttemptState::FatalRejected;
              attempt.target = target;
              attempt.detail =
                  "Cartesian planner step failed: " + planning_status.message;
              recordFailure("cartesian-step", attempt.detail);
              red_attempt_to_ui.publish(attempt);
              if (telemetry_enabled) {
                telemetry_context.set_outcome(
                    proto::FATAL_REJECTED);
              }
              return mcl::WorkerIterationResult{
                  mcl::WorkerIterationOutcome::FatalRejected, target.revision,
                  planning_diagnostics.calculation_time_ms, attempt.detail};
            }
            staged_planner_sample = std::move(next_sample);
          }
          TargetSnapshot reference;
          reference.revision = target.revision;
          reference.left = staged_planner_sample->frames.at(0).pose;
          reference.right = staged_planner_sample->frames.at(1).pose;
          const mcc::CartesianTrajectorySample staged_for_attempt =
              *staged_planner_sample;
          attempt.attempted_reference = reference;
          request.captured_state = capturedState(ik_state);
          addCartesianTargets(handles.red, staged_for_attempt, request);
          const auto ik_status =
              solver.solveRed(request, solution, diagnostics);
          red_solve_time_percentiles.record(diagnostics.solve_time_ms);
          recordQpPassTimes(red_qp_pass_time_percentiles, diagnostics);
          const bool ik_accepted = ik_status.ok();
          if (telemetry_enabled) {
            telemetry_context = makeSampleContext(
                telemetry_stamp,
                ik_accepted ? proto::ACCEPTED
                            : proto::FATAL_REJECTED,
                false, diagnostics.run_generation, target.revision,
                diagnostics.attempt_revision, diagnostics.value_revision);
          }
          if (!ik_accepted) {
            attempt.state =
                planned_options.source_mode == SourceMode::Teleop &&
                        ik_status.code == mcc::StatusCode::Infeasible
                    ? RedAttemptState::RecoverableRejected
                    : RedAttemptState::FatalRejected;
            attempt.target = target;
            updateSolverDebug(attempt.solver_debug, diagnostics,
                              mcc::ResultDisposition::Rejected);
            attempt.detail = rejectedAttemptDetail(ik_status, diagnostics);
            if (telemetry_enabled) {
              telemetry_context.set_outcome(
                  attempt.state == RedAttemptState::RecoverableRejected
                      ? proto::RECOVERABLE_REJECTED
                      : proto::FATAL_REJECTED);
              red_telemetry->tryPush(telemetryRecord(
                  telemetry_stamp,
                  contracts::mcl_telemetry_v1::kIkSolverTopic,
                  makeSolverTelemetry(telemetry_context, "hierarchical_ik",
                                      diagnostics, attempt.solver_debug,
                                      options.solver)));
              red_telemetry->tryPush(telemetryRecord(
                  telemetry_stamp,
                  contracts::mcl_telemetry_v1::kAvoidanceToIkCouplingTopic,
                  makeCouplingTelemetry(telemetry_context, diagnostics,
                                        telemetry_stamp)));
              red_telemetry->tryPush(telemetryRecord(
                  telemetry_stamp,
                  contracts::mcl_telemetry_v1::kCartesianTrackingTopic,
                  makeCartesianTracking(
                      telemetry_context, target, reference,
                      staged_for_attempt, output.raw_left_pose,
                      output.raw_right_pose, output.left_pose,
                      output.right_pose)));
            }
            red_attempt_to_ui.publish(attempt);
            if (attempt.state == RedAttemptState::RecoverableRejected) {
              rejected_target_revision = target.revision;
            }
            recordFailure("red-ik", attempt.detail);
            return mcl::WorkerIterationResult{
                attempt.state == RedAttemptState::RecoverableRejected
                    ? mcl::WorkerIterationOutcome::RecoverableRejected
                    : mcl::WorkerIterationOutcome::FatalRejected,
                diagnostics.attempt_revision, diagnostics.solve_time_ms,
                attempt.detail};
          }

          updateSolverDebug(attempt.solver_debug, diagnostics,
                            solution.kinematics_solution.disposition);
          const auto raw_left_pose =
              requirePose(solution.kinematics_solution.solved_poses,
                          robot.left_end_effector_frame)
                  .pose;
          const auto raw_right_pose =
              requirePose(solution.kinematics_solution.solved_poses,
                          robot.right_end_effector_frame)
                  .pose;
          const auto publishIkEvidence = [&]() {
            if (!telemetry_enabled) {
              return;
            }
            red_telemetry->tryPush(telemetryRecord(
                telemetry_stamp,
                contracts::mcl_telemetry_v1::kIkSolverTopic,
                makeSolverTelemetry(telemetry_context, "hierarchical_ik",
                                    diagnostics, attempt.solver_debug,
                                    options.solver)));
            red_telemetry->tryPush(telemetryRecord(
                telemetry_stamp,
                contracts::mcl_telemetry_v1::kAvoidanceToIkCouplingTopic,
                makeCouplingTelemetry(telemetry_context, diagnostics,
                                      telemetry_stamp)));
          };
          if (telemetry_enabled) {
            if (diagnostics.hierarchy.same_tick_fallback_level.has_value()) {
              red_telemetry->tryPush(TelemetryRecord::log(
                  telemetry_stamp, contracts::mcl_telemetry_v1::kEventsTopic,
                  motion_control::viz::LogLevel::Warning, "ik-fallback",
                  "hierarchical IK used same-tick fallback priority " +
                      std::to_string(priorityNumber(
                          *diagnostics.hierarchy.same_tick_fallback_level))));
            }
          }

          const auto mapped_raw_ik =
              mcl::planned_hierarchical_step_otg::mapActiveIkToFull(
                  toStdVector(ik_state.positions), active_joint_full_indices,
                  toStdVector(solution.kinematics_solution.joint_positions),
                  toStdVector(solution.kinematics_solution.joint_velocities));
          const auto raw_target = joint_target_builder.preview(
              mapped_raw_ik.positions, mapped_raw_ik.velocities);
          mcl::planned_hierarchical_step_otg::ProjectionDiagnostics projection;
          auto projected_target =
              mcl::planned_hierarchical_step_otg::projectConfiguredLimits(
                  raw_target, joint_otg_limits, projection);
          if (target.replay_joint_hold) {
            // Synthetic replay frames keep the IK state machine running while
            // the executed joints remain at the q0 used to construct FK(q0).
            projected_target.positions = toStdVector(otg_state.positions);
            projected_target.velocities.assign(joint_names.size(), 0.0);
            projected_target.accelerations.assign(joint_names.size(), 0.0);
            projection = {};
          }
          if (telemetry_enabled && projection.projected()) {
            red_telemetry->tryPush(TelemetryRecord::log(
                telemetry_stamp, contracts::mcl_telemetry_v1::kEventsTopic,
                motion_control::viz::LogLevel::Warning,
                "joint-target-projection",
                "projected " + std::to_string(projection.events.size()) +
                    " joint target components"));
          }

          mcc::JointTrajectoryRequest joint_request;
          joint_request.joint_names = joint_names;
          joint_request.current.positions = toStdVector(otg_state.positions);
          joint_request.current.velocities = toStdVector(otg_state.velocities);
          joint_request.current.accelerations =
              toStdVector(otg_state.accelerations);
          joint_request.target.positions = projected_target.positions;
          joint_request.target.velocities = projected_target.velocities;
          joint_request.target.accelerations = projected_target.accelerations;
          joint_request.limits.position_lower = joint_otg_limits.position_lower;
          joint_request.limits.position_upper = joint_otg_limits.position_upper;
          joint_request.limits.max_velocity = joint_otg_limits.max_velocity;
          joint_request.limits.max_acceleration =
              joint_otg_limits.max_acceleration;
          joint_request.limits.max_jerk = joint_otg_limits.max_jerk;
          joint_request.sample_period = 1.0 / options.red_rate_hz;

          mcc::PlanningDiagnostics joint_plan_diagnostics;
          auto joint_status =
              joint_planner.plan(joint_request, joint_plan_diagnostics);
          if (!joint_status.ok()) {
            attempt.state = RedAttemptState::FatalRejected;
            attempt.target = target;
            attempt.detail =
                "JointPlanner plan failed: " + joint_status.message;
            recordFailure("joint-plan", attempt.detail);
            red_attempt_to_ui.publish(attempt);
            if (telemetry_enabled) {
              telemetry_context.set_outcome(
                  proto::FATAL_REJECTED);
              publishIkEvidence();
              red_telemetry->tryPush(telemetryRecord(
                  telemetry_stamp,
                  contracts::mcl_telemetry_v1::kJointPlannerTopic,
                  makePlannerTelemetry(
                      telemetry_context, "joint", "jerk_limited",
                      planningSynchronizationName(
                          planned_options.planning.joint_synchronization),
                      "plan", joint_plan_diagnostics, 0.0)));
              red_telemetry->tryPush(telemetryRecord(
                  telemetry_stamp,
                  contracts::mcl_telemetry_v1::kCartesianTrackingTopic,
                  makeCartesianTracking(
                      telemetry_context, target, reference,
                      staged_for_attempt, raw_left_pose, raw_right_pose,
                      output.left_pose, output.right_pose)));
            }
            return mcl::WorkerIterationResult{
                mcl::WorkerIterationOutcome::FatalRejected,
                diagnostics.attempt_revision,
                diagnostics.solve_time_ms +
                    joint_plan_diagnostics.calculation_time_ms,
                attempt.detail};
          }
          mcc::JointTrajectorySample joint_sample;
          mcc::PlanningDiagnostics joint_step_diagnostics;
          joint_status =
              joint_planner.step(joint_sample, joint_step_diagnostics);
          if (!joint_status.ok()) {
            attempt.state = RedAttemptState::FatalRejected;
            attempt.target = target;
            attempt.detail =
                "JointPlanner step failed: " + joint_status.message;
            recordFailure("joint-step", attempt.detail);
            red_attempt_to_ui.publish(attempt);
            if (telemetry_enabled) {
              telemetry_context.set_outcome(
                  proto::FATAL_REJECTED);
              publishIkEvidence();
              red_telemetry->tryPush(telemetryRecord(
                  telemetry_stamp,
                  contracts::mcl_telemetry_v1::kJointPlannerTopic,
                  makePlannerTelemetry(
                      telemetry_context, "joint", "jerk_limited",
                      planningSynchronizationName(
                          planned_options.planning.joint_synchronization),
                      "step", joint_step_diagnostics, 0.0)));
              red_telemetry->tryPush(telemetryRecord(
                  telemetry_stamp,
                  contracts::mcl_telemetry_v1::kCartesianTrackingTopic,
                  makeCartesianTracking(
                      telemetry_context, target, reference,
                      staged_for_attempt, raw_left_pose, raw_right_pose,
                      output.left_pose, output.right_pose)));
            }
            return mcl::WorkerIterationResult{
                mcl::WorkerIterationOutcome::FatalRejected,
                diagnostics.attempt_revision,
                diagnostics.solve_time_ms +
                    joint_plan_diagnostics.calculation_time_ms +
                    joint_step_diagnostics.calculation_time_ms,
                attempt.detail};
          }

          StateSnapshot candidate_ik_state = ik_state;
          candidate_ik_state.positions = toEigen(mapped_raw_ik.positions);
          candidate_ik_state.velocities = toEigen(mapped_raw_ik.velocities);
          candidate_ik_state.accelerations.setZero();
          candidate_ik_state.jerks.setZero();
          ++candidate_ik_state.sequence;
          candidate_ik_state.monotonic_time_nanoseconds =
              telemetry_enabled
                  ? static_cast<std::int64_t>(telemetry_stamp.run_time_ns)
                  : sample_time_ns;

          StateSnapshot candidate_otg_state = otg_state;
          candidate_otg_state.positions = toEigen(joint_sample.positions);
          candidate_otg_state.velocities = toEigen(joint_sample.velocities);
          candidate_otg_state.accelerations =
              toEigen(joint_sample.accelerations);
          candidate_otg_state.jerks = toEigen(joint_sample.jerks);
          ++candidate_otg_state.sequence;
          candidate_otg_state.monotonic_time_nanoseconds =
              telemetry_enabled
                  ? static_cast<std::int64_t>(telemetry_stamp.run_time_ns)
                  : sample_time_ns;
          mcc::ForwardKinematicsRequest executed_fk_request;
          executed_fk_request.state = robotState(candidate_otg_state);
          executed_fk_request.frame_names = {robot.left_end_effector_frame,
                                             robot.right_end_effector_frame};
          executed_fk_request.reference_frame_name = robot.base_frame;
          mcc::ForwardKinematicsSolution executed_fk;
          mcc::ForwardKinematicsDiagnostics executed_fk_diagnostics;
          const auto fk_status = solver.computeForwardKinematics(
              executed_fk_request, executed_fk, executed_fk_diagnostics);
          if (!fk_status.ok()) {
            attempt.state = RedAttemptState::FatalRejected;
            attempt.target = target;
            attempt.detail =
                "OTG execution-state FK failed: " + fk_status.message;
            recordFailure("otg-fk", attempt.detail);
            red_attempt_to_ui.publish(attempt);
            if (telemetry_enabled) {
              telemetry_context.set_outcome(
                  proto::FATAL_REJECTED);
              publishIkEvidence();
              red_telemetry->tryPush(telemetryRecord(
                  telemetry_stamp,
                  contracts::mcl_telemetry_v1::kCartesianTrackingTopic,
                  makeCartesianTracking(
                      telemetry_context, target, reference,
                      staged_for_attempt, raw_left_pose, raw_right_pose,
                      output.left_pose, output.right_pose)));
            }
            return mcl::WorkerIterationResult{
                mcl::WorkerIterationOutcome::FatalRejected,
                diagnostics.attempt_revision,
                diagnostics.solve_time_ms +
                    joint_plan_diagnostics.calculation_time_ms +
                    joint_step_diagnostics.calculation_time_ms,
                attempt.detail};
          }

          const auto executed_left_pose =
              requirePose(executed_fk.poses, robot.left_end_effector_frame)
                  .pose;
          const auto executed_right_pose =
              requirePose(executed_fk.poses, robot.right_end_effector_frame)
                  .pose;

          ik_state = std::move(candidate_ik_state);
          otg_state = std::move(candidate_otg_state);
          joint_target_builder.commit(mapped_raw_ik.positions, raw_target);
          state_to_yellow.publish(otg_state);
          accepted_planner_sample = *staged_planner_sample;
          staged_planner_sample.reset();

          output.revision = diagnostics.value_revision;
          output.event_timestamp_ns = telemetry_stamp.timestamp_ns;
          output.run_time_ns = telemetry_stamp.run_time_ns;
          output.accepted_target = reference;
          output.source_goal = target;
          output.accepted_planner_sample = accepted_planner_sample;
          output.planner_state = planning_diagnostics.state;
          output.cartesian_plan_diagnostics = planning_diagnostics;
          output.state = otg_state;
          output.raw_ik_positions = ik_state.positions;
          output.raw_ik_velocities = ik_state.velocities;
          output.raw_joint_target = raw_target;
          output.projected_joint_target = projected_target;
          output.projection = projection;
          output.projection_event_count += projection.events.size();
          output.projection_cycle_count += projection.projected() ? 1U : 0U;
          output.future_o1_startup = raw_target.future_o1_startup;
          output.joint_plan_diagnostics = joint_plan_diagnostics;
          output.joint_step_diagnostics = joint_step_diagnostics;
          output.raw_left_pose = raw_left_pose;
          output.raw_right_pose = raw_right_pose;
          output.left_pose = executed_left_pose;
          output.right_pose = executed_right_pose;
          output.solve_time_ms = diagnostics.solve_time_ms;
          output.iterations = diagnostics.iterations;
          output.converged = diagnostics.converged;
          fillRedDiagnostics(handles.red, diagnostics, output);
          std::tie(output.left_position_error_m,
                   output.left_orientation_error_rad) =
              poseError(reference.left, executed_left_pose);
          std::tie(output.right_position_error_m,
                   output.right_orientation_error_rad) =
              poseError(reference.right, executed_right_pose);
          updateSolverDebug(output.solver_debug, diagnostics,
                            solution.kinematics_solution.disposition);
          if (telemetry_enabled) {
            telemetry_context.set_outcome(proto::ACCEPTED);
            telemetry_context.set_committed(true);
            telemetry_context.set_value_revision(diagnostics.value_revision);
            attempt.solver_debug = output.solver_debug;
            publishIkEvidence();
            red_telemetry->tryPush(telemetryRecord(
                telemetry_stamp,
                contracts::mcl_telemetry_v1::kCartesianTrackingTopic,
                makeCartesianTracking(
                    telemetry_context, target, reference, staged_for_attempt,
                    raw_left_pose, raw_right_pose, executed_left_pose,
                    executed_right_pose)));
            red_telemetry->tryPush(telemetryRecord(
                telemetry_stamp,
                contracts::mcl_telemetry_v1::kJointTrackingTopic,
                makeJointTracking(telemetry_context, joint_names, output,
                                  joint_otg_limits)));
            auto joint_tick_diagnostics = joint_step_diagnostics;
            joint_tick_diagnostics.duration = joint_plan_diagnostics.duration;
            joint_tick_diagnostics.calculation_time_ms +=
                joint_plan_diagnostics.calculation_time_ms;
            red_telemetry->tryPush(telemetryRecord(
                telemetry_stamp,
                contracts::mcl_telemetry_v1::kJointPlannerTopic,
                makePlannerTelemetry(
                    telemetry_context, "joint", "jerk_limited",
                    planningSynchronizationName(
                        planned_options.planning.joint_synchronization),
                    "plan+step", joint_tick_diagnostics,
                    joint_sample.time_from_start)));
          }
          output_to_ui.publish(output);

          attempt.state = RedAttemptState::Accepted;
          attempt.target = target;
          attempt.solver_debug = output.solver_debug;
          attempt.detail.clear();
          red_attempt_to_ui.publish(attempt);

          if (loaded_replay.has_value()) {
            ++replay_accepted_solve_count;
            const std::size_t final_source_index =
                loaded_replay->timeline.timeline.size() - 1U;
            const bool settled = replay_settling.update(
                target.replay_source_index == final_source_index &&
                    replay_last_consumed_revision.load() >= target.revision,
                planning_diagnostics.state == mcc::PlanningState::Finished,
                output.left_position_error_m, output.left_orientation_error_rad,
                output.right_position_error_m,
                output.right_orientation_error_rad,
                maximumAbsolute(otg_state.velocities),
                maximumAbsolute(otg_state.accelerations));
            replay_settled_cycle_count.store(
                replay_settling.consecutiveCycles());
            replay_settled.store(settled);

            if (planned_options.replay_trace_enabled) {
              auto row = makeReplayRow(diagnostics.attempt_revision);
              row[11] = "true";
              row[12] = "";
              row[13] = "ok";
              row[15] = std::to_string(diagnostics.solve_time_ms);
              row[16] = std::to_string(diagnostics.maximum_hard_violation);
              row[17] = traceEigenVector(target.left.translation());
              row[18] = traceEigenVector(reference.left.translation());
              row[19] = traceEigenVector(staged_for_attempt.frames.at(0).twist);
              row[20] =
                  traceEigenVector(staged_for_attempt.frames.at(0).acceleration);
              row[21] = tracePose(raw_left_pose);
              row[22] = tracePose(raw_right_pose);
              row[23] = tracePose(executed_left_pose);
              row[24] = tracePose(executed_right_pose);
              row[25] = traceEigenVector(output.raw_ik_positions);
              row[26] = traceEigenVector(output.raw_ik_velocities);
              row[27] = traceStdVector(raw_target.positions);
              row[28] = traceStdVector(raw_target.velocities);
              row[29] = traceStdVector(raw_target.accelerations);
              row[30] = traceStdVector(projected_target.positions);
              row[31] = traceStdVector(projected_target.velocities);
              row[32] = traceStdVector(projected_target.accelerations);
              row[33] = traceEigenVector(otg_state.positions);
              row[34] = traceEigenVector(otg_state.velocities);
              row[35] = traceEigenVector(otg_state.accelerations);
              row[36] = traceEigenVector(otg_state.jerks);
              row[37] = raw_target.future_o1_startup ? "true" : "false";
              row[38] = plannerStateName(joint_step_diagnostics.state);
              row[39] = std::to_string(joint_plan_diagnostics.duration);
              row[40] =
                  std::to_string(joint_plan_diagnostics.calculation_time_ms);
              row[41] =
                  std::to_string(joint_step_diagnostics.calculation_time_ms);
              row[42] = traceProjectionEvents(projection, joint_names);
              row[43] = std::to_string(output.projection_event_count);
              row[44] = std::to_string(output.projection_cycle_count);
              const auto worker_stats = red_worker_diagnostics.snapshot();
              row[45] = std::to_string(worker_stats.deadline_miss_count);
              row[46] = std::to_string(worker_stats.skipped_release_count);
              row[47] = std::to_string(replay_settling.consecutiveCycles());
              std::lock_guard<std::mutex> lock(replay_trace_mutex);
              appendCsvRow(replay_trace, row);
            }
          }

          const double total_solver_time_ms =
              diagnostics.solve_time_ms +
              joint_plan_diagnostics.calculation_time_ms +
              joint_step_diagnostics.calculation_time_ms;
          return mcl::WorkerIterationResult{
              mcl::WorkerIterationOutcome::Accepted,
              diagnostics.attempt_revision,
              total_solver_time_ms,
              {}};
        },
        telemetry_observer);
  });

  mcl::SingleRateScheduler ui_scheduler(
      {options.ui_rate_hz, options.duration_s});
  TargetSnapshot published_target = initial_target;
  TargetSnapshot last_command_target = initial_target;
  std::vector<mcl::ArmTarget> latest_input_targets = armTargets(initial_target);
  if (planned_options.source_mode == SourceMode::Replay &&
      initial_target.replay_source_index.has_value()) {
    const auto &source = replay_source->sourceFrame();
    latest_input_targets = {
        {mcl::ArmSide::Left, source.value.left.pose},
        {mcl::ArmSide::Right, source.value.right.pose}};
  }
  std::optional<std::size_t> last_released_replay_source_index =
      initial_target.replay_source_index;
  RedOutputSnapshot latest_output = initial_output;
  RedAttemptSnapshot latest_red_attempt = initial_red_attempt;
  mcl::SelfCollisionDebug latest_collision_debug = initial_collision_debug;
  mcl::SolverDebug latest_yellow_solver_debug = initial_yellow_solver_debug;
  mcl::CpuAffinityBinding latest_red_affinity = initial_red_affinity;
  mcl::CpuAffinityBinding latest_yellow_affinity = initial_yellow_affinity;
  std::optional<mcl::RejectedTargetDebug> rejected_target;
  std::optional<RedAttemptSnapshot> last_recoverable_rejection;
  std::optional<mcl::GroupedWorkerFault> held_fault;
  std::optional<std::string> pending_fault_event;
  std::uint64_t handled_rejected_target_revision = 0;
  std::size_t publish_count = 0;
  bool replay_completed = false;
  std::string telemetry_run_id;
  if (telemetry_enabled) {
    telemetry_run_id =
        planned_options.replay.has_value()
            ? planned_options.replay->output_dir.filename().string()
            : mcl::make_run_id(mcl::sha256_file(options.urdf_path));
  }
  bool run_info_published = false;
  bool replay_eos_event_published = false;
  std::uint64_t last_run_info_run_time_ns = 0U;
  std::uint64_t last_reported_dropped_samples = 0U;
  double previous_write_time_ms = 0.0;
  mcl::IkDebugFrame frame;
  frame.joint_names = joint_names;
  frame.positions = robot.default_positions;
  frame.velocities.assign(joint_names.size(), 0.0);
  frame.forward_kinematics = {{mcl::ArmSide::Left, initial_output.left_pose},
                              {mcl::ArmSide::Right, initial_output.right_pose}};
  frame.selected_side = mcl::parseArmSide(options.tui.side);
  frame.solvers = {initial_output.solver_debug, initial_yellow_solver_debug};
  frame.solvers[0].ik_solve_time_percentiles =
      red_solve_time_percentiles.snapshot();
  updateQpPassTimePercentiles(frame.solvers[0],
                              red_qp_pass_time_percentiles);
  frame.solvers[1].ik_solve_time_percentiles =
      yellow_solve_time_percentiles.snapshot();
  frame.cpu_affinities = {mcl::makeCpuAffinityDebug(ui_affinity_binding),
                          mcl::makeCpuAffinityDebug(latest_red_affinity),
                          mcl::makeCpuAffinityDebug(latest_yellow_affinity)};
  frame.self_collisions = {initial_collision_debug};
  mcl::PlannedGroupedJointOtgTuiDebug tui_debug;
  tui_debug.source_mode = planned_options.source_mode == SourceMode::Replay
                              ? "mcap replay"
                              : "keyboard teleop";
  tui_debug.target_mode = mcl::planned_hierarchical_step_otg::jointTargetModeName(
      planned_options.joint_target.mode);
  tui_debug.feedback_topology = "split IK reference / OTG execution";
  tui_debug.cartesian_limits = {
      planned_options.planning.max_linear_velocity_mps,
      planned_options.planning.max_linear_acceleration_mps2,
      planned_options.planning.max_linear_jerk_mps3,
      planned_options.planning.max_angular_velocity_rps,
      planned_options.planning.max_angular_acceleration_rps2,
      planned_options.planning.max_angular_jerk_rps3};
  std::optional<std::uint64_t> pending_step_hold_red_iteration;
  bool pending_synthetic_replay_hold = false;

  const auto publishSyntheticReplayHold = [&](std::string status) {
    published_target.revision += 1U;
    published_target.replay_source_index.reset();
    published_target.replay_joint_hold = true;
    published_target.left = latest_output.left_pose;
    published_target.right = latest_output.right_pose;
    latest_input_targets = armTargets(published_target);
    last_command_target = published_target;
    target_to_red.publish(published_target);
    replay_settled.store(false);
    replay_settled_cycle_count.store(0U);
    input.setTargetPose(mcl::ArmSide::Left, published_target.left, status);
    input.setTargetPose(mcl::ArmSide::Right, published_target.right,
                        std::move(status));
  };
  const auto publishCurrentReplayFrame = [&](std::string status) {
    const auto &source = replay_source->sourceFrame();
    published_target.revision += 1U;
    published_target.replay_source_index = replay_source->sourceIndex();
    published_target.replay_joint_hold = false;
    published_target.left =
        source.value.left.pose * robot.left_tcp_offset.inverse();
    published_target.right =
        source.value.right.pose * robot.right_tcp_offset.inverse();
    latest_input_targets = {
        {mcl::ArmSide::Left, source.value.left.pose},
        {mcl::ArmSide::Right, source.value.right.pose}};
    last_released_replay_source_index = replay_source->sourceIndex();
    last_command_target = published_target;
    target_to_red.publish(published_target);
    input.setTargetPose(mcl::ArmSide::Left, published_target.left, status);
    input.setTargetPose(mcl::ArmSide::Right, published_target.right,
                        std::move(status));
  };
  const auto scheduleSingleFrameHold = [&](std::int64_t duration_ns) {
    const auto red_stats = red_worker_diagnostics.snapshot();
    pending_step_hold_red_iteration =
        red_stats.iteration_count + workerTicksForReplayDuration(
                                        duration_ns, options.red_rate_hz);
  };
  const auto currentReplayFrameDuration = [&]() {
    const std::size_t source_index = replay_source->sourceIndex();
    const auto &timeline = loaded_replay->timeline.timeline;
    if (source_index + 1U < timeline.size()) {
      return timeline.at(source_index + 1U).projected_time_ns -
             timeline.at(source_index).projected_time_ns;
    }
    return planned_options.replay->target_period_ns;
  };

  while (true) {
    const auto schedule = ui_scheduler.next();
    if (!schedule) {
      break;
    }

    output_to_ui.readLatest(latest_output);
    collision_to_ui.readLatest(latest_collision_debug);
    yellow_solver_to_ui.readLatest(latest_yellow_solver_debug);
    red_affinity_to_ui.readLatest(latest_red_affinity);
    yellow_affinity_to_ui.readLatest(latest_yellow_affinity);
    if (red_attempt_to_ui.readLatest(latest_red_attempt)) {
      if (latest_red_attempt.state == RedAttemptState::RecoverableRejected) {
        last_recoverable_rejection = latest_red_attempt;
        rejected_target = mcl::RejectedTargetDebug{
            latest_red_attempt.target.revision,
            armTargets(latest_red_attempt.target), latest_red_attempt.detail};
        if (latest_red_attempt.target.revision == published_target.revision &&
            latest_red_attempt.target.revision !=
                handled_rejected_target_revision) {
          handled_rejected_target_revision = latest_red_attempt.target.revision;
          input.setTargetPose(mcl::ArmSide::Left,
                              latest_output.accepted_target.left,
                              "Restoring last accepted Red target");
          input.setTargetPose(
              mcl::ArmSide::Right, latest_output.accepted_target.right,
              "Red target rejected; edit from the last accepted "
              "target to retry");
          last_command_target = latest_output.accepted_target;
        }
      } else if (latest_red_attempt.state == RedAttemptState::Accepted &&
                 rejected_target.has_value() &&
                 latest_red_attempt.target.revision >
                     rejected_target->revision) {
        rejected_target.reset();
        input.setStatus("Red accepted the new target; hierarchical IK resumed");
      } else if (latest_red_attempt.state == RedAttemptState::FatalRejected) {
        rejected_target = mcl::RejectedTargetDebug{
            latest_red_attempt.target.revision,
            armTargets(latest_red_attempt.target), latest_red_attempt.detail};
      }
    }

    if (!held_fault.has_value()) {
      if (const auto recorded_fault = fault.snapshot()) {
        held_fault = *recorded_fault;
        pending_fault_event = faultSummary(*recorded_fault);
        workers.join();
        input.setMotionInputEnabled(
            false, std::string{"FAULT HOLD: "} +
                       mcl::workerGroupName(recorded_fault->group) + " " +
                       mcl::workerFailureName(recorded_fault->failure));
      }
    }

    const auto input_update = input.poll(schedule->dt);
    for (const auto &event : input_update.navigation) {
      tui.handleNavigation(event);
    }
    if (!held_fault.has_value()) {
      if (const auto reset_side = input.consumeResetRequest()) {
        input.setTargetPose(
            *reset_side,
            *reset_side == mcl::ArmSide::Left ? latest_output.left_pose
                                              : latest_output.right_pose,
            std::string{"Reset "} + mcl::armSideName(*reset_side) +
                " target from latest Red output");
      }
    }
    if (input.stopRequested()) {
      break;
    }

    if (planned_options.source_mode == SourceMode::Replay &&
        !held_fault.has_value()) {
      bool skip_replay_advance = false;
      std::optional<std::int64_t> pending_step_start_ns;
      for (const auto control : input.consumeSourceControls()) {
        const bool was_paused = replay_source->paused();
        if (control == mcl::SourceControl::Step) {
          pending_step_hold_red_iteration.reset();
          const bool current_frame_has_not_been_released =
              replay_source->paused() &&
              (!last_released_replay_source_index.has_value() ||
               *last_released_replay_source_index !=
                   replay_source->sourceIndex());
          if (current_frame_has_not_been_released) {
            publishCurrentReplayFrame("Replay single-frame goal");
            scheduleSingleFrameHold(currentReplayFrameDuration());
            skip_replay_advance = true;
          } else {
            pending_step_start_ns =
                replay_source->sourceFrame().projected_time_ns;
            replay_source->applyControl(control);
          }
          continue;
        }

        replay_source->applyControl(control);
        const bool paused_now = replay_source->paused();
        const bool paused_by_control =
            control == mcl::SourceControl::Pause ||
            (control == mcl::SourceControl::TogglePause && !was_paused &&
             paused_now);
        const bool resumed_by_control =
            control == mcl::SourceControl::Resume ||
            (control == mcl::SourceControl::TogglePause && was_paused &&
             !paused_now);
        if (paused_by_control) {
          pending_step_hold_red_iteration.reset();
          pending_synthetic_replay_hold =
              published_target.replay_source_index.has_value();
          skip_replay_advance = true;
        } else if (resumed_by_control) {
          pending_step_hold_red_iteration.reset();
          pending_synthetic_replay_hold = false;
          publishCurrentReplayFrame("Replay resumed from current source frame");
          skip_replay_advance = true;
        } else if (control == mcl::SourceControl::Stop) {
          pending_step_hold_red_iteration.reset();
          pending_synthetic_replay_hold = false;
          skip_replay_advance = true;
        }
      }
      const bool worker_consumed_current =
          replay_last_consumed_revision.load() >= published_target.revision;
      const bool may_advance = planned_options.replay->execution_mode ==
                                   mcl::data::ExecutionMode::Realtime ||
                               worker_consumed_current;
      const auto advance =
          !skip_replay_advance && may_advance
              ? replay_source->advance(static_cast<std::int64_t>(
                    std::llround(1.0e9 / options.ui_rate_hz)))
              : replay::ReplayAdvance{};
      replay_source->waitForCurrentFrame();
      input.setPaused(replay_source->paused(), replay_source->status().detail);
      if (advance.frame_changed) {
        publishCurrentReplayFrame("Replay goal advanced");
        if (pending_step_start_ns.has_value()) {
          const std::int64_t frame_duration_ns =
              replay_source->sourceFrame().projected_time_ns -
              *pending_step_start_ns;
          scheduleSingleFrameHold(frame_duration_ns);
        }
      }
      if (pending_step_hold_red_iteration.has_value() &&
          replay_source->paused() &&
          red_worker_diagnostics.snapshot().iteration_count >=
              *pending_step_hold_red_iteration) {
        pending_synthetic_replay_hold = true;
        pending_step_hold_red_iteration.reset();
      }
      output_to_ui.readLatest(latest_output);
      const bool current_target_finished =
          latest_output.source_goal.revision == published_target.revision &&
          latest_output.planner_state == mcc::PlanningState::Finished;
      const bool execution_settled =
          latest_output.left_position_error_m <=
              planned_options.replay_settling.fk_position_m &&
          latest_output.right_position_error_m <=
              planned_options.replay_settling.fk_position_m &&
          latest_output.left_orientation_error_rad <=
              planned_options.replay_settling.fk_orientation_rad &&
          latest_output.right_orientation_error_rad <=
              planned_options.replay_settling.fk_orientation_rad &&
          maximumAbsolute(latest_output.state.velocities) <=
              planned_options.replay_settling.velocity_rad_per_s &&
          maximumAbsolute(latest_output.state.accelerations) <=
              planned_options.replay_settling.acceleration_rad_per_s2;
      // An instantaneous FK(current-q) retarget while q is moving requires a
      // stop-and-return Cartesian path. Finish the already released source
      // frame first, then make the settled execution state the synthetic q0.
      if (pending_synthetic_replay_hold && replay_source->paused() &&
          current_target_finished && execution_settled) {
        publishSyntheticReplayHold("Replay paused at settled FK");
        pending_synthetic_replay_hold = false;
      }
    }

    if (schedule->update_due) {
      if (planned_options.source_mode == SourceMode::Teleop &&
          !held_fault.has_value() && !input.paused() &&
          !sameTargetPoses(last_command_target, input.targets())) {
        published_target =
            targetSnapshot(input.targets(), published_target.revision + 1);
        latest_input_targets = input.targets();
        last_command_target = published_target;
        target_to_red.publish(published_target);
      }

      frame.input_targets = latest_input_targets;
      frame.targets = armTargets(latest_output.accepted_target);
      frame.forward_kinematics = {
          {mcl::ArmSide::Left, latest_output.left_pose},
          {mcl::ArmSide::Right, latest_output.right_pose}};
      frame.positions = toStdVector(latest_output.state.positions);
      frame.velocities = toStdVector(latest_output.state.velocities);
      const auto red_stats = red_worker_diagnostics.snapshot();
      const auto yellow_stats = yellow_worker_diagnostics.snapshot();
      frame.solvers = {latest_red_attempt.solver_debug,
                       latest_yellow_solver_debug};
      frame.solvers[0].ik_solve_time_percentiles =
          red_solve_time_percentiles.snapshot();
      updateQpPassTimePercentiles(frame.solvers[0],
                                  red_qp_pass_time_percentiles);
      frame.solvers[1].ik_solve_time_percentiles =
          yellow_solve_time_percentiles.snapshot();
      frame.cpu_affinities = {
          mcl::makeCpuAffinityDebug(ui_affinity_binding),
          mcl::makeCpuAffinityDebug(latest_red_affinity),
          mcl::makeCpuAffinityDebug(latest_yellow_affinity)};
      frame.workers = {
          {"Red", options.red_rate_hz, red_stats.iteration_count,
           red_stats.deadline_miss_count, red_stats.consecutive_deadline_misses,
           red_stats.skipped_release_count,
           red_stats.maximum_release_lateness_ms,
           red_stats.maximum_execution_ms,
           red_stats.maximum_release_to_finish_ms, red_stats.maximum_overrun_ms,
           red_stats.maximum_solver_ms, red_stats.recoverable_rejection_count,
           red_stats.maximum_non_solver_execution_ms,
           red_stats.latest_release_lateness_ms, red_stats.latest_execution_ms,
           red_stats.latest_release_to_finish_ms, red_stats.latest_overrun_ms,
           red_stats.latest_solver_ms,
           red_stats.latest_non_solver_execution_ms},
          {"Yellow", options.yellow_rate_hz, yellow_stats.iteration_count,
           yellow_stats.deadline_miss_count,
           yellow_stats.consecutive_deadline_misses,
           yellow_stats.skipped_release_count,
           yellow_stats.maximum_release_lateness_ms,
           yellow_stats.maximum_execution_ms,
           yellow_stats.maximum_release_to_finish_ms,
           yellow_stats.maximum_overrun_ms, yellow_stats.maximum_solver_ms,
           yellow_stats.recoverable_rejection_count,
           yellow_stats.maximum_non_solver_execution_ms,
           yellow_stats.latest_release_lateness_ms,
           yellow_stats.latest_execution_ms,
           yellow_stats.latest_release_to_finish_ms,
           yellow_stats.latest_overrun_ms, yellow_stats.latest_solver_ms,
           yellow_stats.latest_non_solver_execution_ms}};
      const std::string clamp_detail = retargetClampDetail(latest_red_attempt);
      if (held_fault.has_value()) {
        frame.runtime_state = mcl::IkRuntimeState::FaultHold;
        frame.ik_status = "fault hold " + taskScaleStatus(latest_output);
        frame.status = faultSummary(*held_fault);
      } else if (latest_red_attempt.state ==
                 RedAttemptState::RecoverableRejected) {
        frame.runtime_state = mcl::IkRuntimeState::RecoverableReject;
        frame.ik_status =
            "target rejected; output held " + taskScaleStatus(latest_output);
        frame.status = "Red target revision=" +
                       std::to_string(latest_red_attempt.target.revision) +
                       " rejected as infeasible; edit from the last accepted "
                       "target to retry";
      } else {
        frame.runtime_state = mcl::IkRuntimeState::Running;
        frame.ik_status =
            "running " + taskScaleStatus(latest_output) +
            " deadline_misses R=" +
            std::to_string(red_stats.deadline_miss_count) +
            " Y=" + std::to_string(yellow_stats.deadline_miss_count) +
            " skipped R=" + std::to_string(red_stats.skipped_release_count) +
            " Y=" + std::to_string(yellow_stats.skipped_release_count);
        frame.status = "Hierarchical IK running | skipped_releases R=" +
                       std::to_string(red_stats.skipped_release_count) + " Y=" +
                       std::to_string(yellow_stats.skipped_release_count) +
                       " recoverable_rejections R=" +
                       std::to_string(red_stats.recoverable_rejection_count);
      }
      if (!clamp_detail.empty()) {
        frame.ik_status +=
            " retarget_clamped=" +
            std::to_string(
                latest_red_attempt.retarget_clamp.clamped_component_count);
        frame.status += " | " + clamp_detail;
      }
      const double raw_otg_position_delta =
          (latest_output.raw_ik_positions - latest_output.state.positions)
              .cwiseAbs()
              .maxCoeff();
      const double raw_otg_velocity_delta =
          (latest_output.raw_ik_velocities - latest_output.state.velocities)
              .cwiseAbs()
              .maxCoeff();
      frame.ik_status +=
          " joint_otg=" +
          std::string{mcl::planned_hierarchical_step_otg::jointTargetModeName(
              planned_options.joint_target.mode)} +
          " feedback=split-ik-reference/otg-execution" + " state=" +
          plannerStateName(latest_output.joint_step_diagnostics.state) +
          " plan_ms=" +
          std::to_string(
              latest_output.joint_plan_diagnostics.calculation_time_ms) +
          " step_ms=" +
          std::to_string(
              latest_output.joint_step_diagnostics.calculation_time_ms) +
          " projection_events=" +
          std::to_string(latest_output.projection_event_count);
      frame.status +=
          " | JointPlanner " +
          std::string{
              mcl::planned_hierarchical_step_otg::planningSynchronizationName(
                  planned_options.planning.joint_synchronization)} +
          " per-tick plan+first-step duration_s=" +
          std::to_string(latest_output.joint_plan_diagnostics.duration) +
          " startup=" +
          (latest_output.future_o1_startup ? std::string{"true"} : "false") +
          " max_abs_v=" +
          std::to_string(maximumAbsolute(latest_output.state.velocities)) +
          " max_abs_a=" +
          std::to_string(maximumAbsolute(latest_output.state.accelerations)) +
          " raw_otg_max_dq=" + std::to_string(raw_otg_position_delta) +
          " raw_otg_max_dv=" + std::to_string(raw_otg_velocity_delta);
      frame.iterations = latest_red_attempt.solver_debug.ik_iterations;
      frame.converged = latest_red_attempt.solver_debug.converged;
      frame.solve_time_ms = latest_red_attempt.solver_debug.ik_solve_time_ms;
      frame.target_errors = {
          {mcl::ArmSide::Left, latest_output.left_position_error_m,
           latest_output.left_orientation_error_rad},
          {mcl::ArmSide::Right, latest_output.right_position_error_m,
           latest_output.right_orientation_error_rad}};
      mcl::CartesianPlannerDebug planner_debug;
      planner_debug.state = plannerStateName(latest_output.planner_state);
      planner_debug.sample_time_s =
          latest_output.accepted_planner_sample.time_from_start;
      const auto makePlannerArm = [&](mcl::ArmSide side, std::size_t index) {
        const bool left = side == mcl::ArmSide::Left;
        const auto &planner_frame =
            latest_output.accepted_planner_sample.frames.at(index);
        const auto &source_goal = left ? latest_output.source_goal.left
                                       : latest_output.source_goal.right;
        const auto &reference = left ? latest_output.accepted_target.left
                                     : latest_output.accepted_target.right;
        const auto &fk =
            left ? latest_output.left_pose : latest_output.right_pose;
        return mcl::PlannedArmDebug{
            side,
            source_goal,
            reference,
            fk,
            planner_frame.twist,
            planner_frame.acceleration,
            (reference.translation() - fk.translation()).norm(),
            Eigen::AngleAxisd(reference.linear() * fk.linear().transpose())
                .angle()};
      };
      planner_debug.arms = {makePlannerArm(mcl::ArmSide::Left, 0U),
                            makePlannerArm(mcl::ArmSide::Right, 1U)};
      frame.cartesian_planner = std::move(planner_debug);
      frame.self_collisions = {latest_collision_debug};
      frame.rejected_target = rejected_target;
      frame.paused = input.paused();
      frame.selected_side = input.selectedSide();

      tui_debug.left_task_scale = latest_output.left_scale.scale;
      tui_debug.right_task_scale = latest_output.right_scale.scale;
      tui_debug.cartesian_plan = {
          plannerStateName(latest_output.planner_state),
          latest_output.cartesian_plan_diagnostics.duration,
          latest_output.accepted_planner_sample.time_from_start,
          latest_output.cartesian_plan_diagnostics.sample_count,
          latest_output.cartesian_plan_diagnostics.calculation_time_ms};
      tui_debug.joint_plan = {
          plannerStateName(latest_output.joint_plan_diagnostics.state),
          latest_output.joint_plan_diagnostics.duration,
          1.0 / options.red_rate_hz,
          latest_output.joint_plan_diagnostics.sample_count,
          latest_output.joint_plan_diagnostics.calculation_time_ms};
      tui_debug.joint_step = {
          plannerStateName(latest_output.joint_step_diagnostics.state),
          latest_output.joint_step_diagnostics.duration,
          1.0 / options.red_rate_hz,
          latest_output.joint_step_diagnostics.sample_count,
          latest_output.joint_step_diagnostics.calculation_time_ms};
      tui_debug.startup = latest_output.future_o1_startup;
      tui_debug.projection_event_count = latest_output.projection_event_count;
      tui_debug.projection_cycle_count = latest_output.projection_cycle_count;
      tui_debug.modified_joint_count =
          latest_output.projection.modified_joint_count;
      tui_debug.raw_otg_max_position_delta = raw_otg_position_delta;
      tui_debug.raw_otg_max_velocity_delta = raw_otg_velocity_delta;
      tui_debug.maximum_absolute_velocity =
          maximumAbsolute(latest_output.state.velocities);
      tui_debug.maximum_absolute_acceleration =
          maximumAbsolute(latest_output.state.accelerations);
      tui_debug.maximum_absolute_jerk =
          maximumAbsolute(latest_output.state.jerks);
      tui_debug.clamp_target_revision =
          latest_red_attempt.retarget_clamp_target_revision;
      tui_debug.maximum_clamp_limit_ratio =
          latest_red_attempt.retarget_clamp.maximum_limit_ratio;

      std::vector<std::string> projection_flags(joint_names.size(), "-");
      tui_debug.projection_events.clear();
      tui_debug.projection_events.reserve(
          latest_output.projection.events.size());
      for (const auto &event : latest_output.projection.events) {
        auto &flags = projection_flags.at(event.joint_index);
        const std::string flag = projectionFlag(event.component);
        if (flags == "-") {
          flags = flag;
        } else if (flags.find(flag) == std::string::npos) {
          flags += flag;
        }
        tui_debug.projection_events.push_back(
            {joint_names.at(event.joint_index),
             mcl::planned_hierarchical_step_otg::projectionComponentName(
                 event.component),
             event.original_value, event.applied_value, event.limit});
      }

      tui_debug.joints.clear();
      tui_debug.joints.reserve(joint_names.size());
      for (std::size_t index = 0U; index < joint_names.size(); ++index) {
        tui_debug.joints.push_back(
            {joint_names[index], compactJointName(index),
             latest_output.raw_ik_positions[static_cast<Eigen::Index>(index)],
             latest_output.raw_ik_velocities[static_cast<Eigen::Index>(index)],
             latest_output.projected_joint_target.positions.at(index),
             latest_output.projected_joint_target.velocities.at(index),
             latest_output.projected_joint_target.accelerations.at(index),
             latest_output.state.positions[static_cast<Eigen::Index>(index)],
             latest_output.state.velocities[static_cast<Eigen::Index>(index)],
             latest_output.state
                 .accelerations[static_cast<Eigen::Index>(index)],
             latest_output.state.jerks[static_cast<Eigen::Index>(index)],
             joint_otg_limits.position_lower.at(index),
             joint_otg_limits.position_upper.at(index),
             joint_otg_limits.max_velocity.at(index),
             joint_otg_limits.max_acceleration.at(index),
             joint_otg_limits.max_jerk.at(index), projection_flags[index]});
      }

      constexpr std::array<const char *, 3> kAxisNames{"x", "y", "z"};
      tui_debug.clamp_events.clear();
      tui_debug.clamp_events.reserve(
          latest_red_attempt.retarget_clamp.clamped_component_count);
      for (std::size_t index = 0U;
           index < latest_red_attempt.retarget_clamp.clamped_component_count;
           ++index) {
        const auto &event = latest_red_attempt.retarget_clamp.events[index];
        tui_debug.clamp_events.push_back(
            {event.segment_index == 0U ? "left" : "right",
             retargetClampComponentName(event.component),
             kAxisNames.at(event.axis), event.original_value,
             event.applied_value, event.limit});
      }

      mcl::IkDebugFrame visualization_debug_frame = frame;
      visualization_debug_frame.targets = armTargets(latest_red_attempt.target);
      visualization_debug_frame.positions =
          toStdVector(latest_output.raw_ik_positions);
      visualization_debug_frame.velocities =
          toStdVector(latest_output.raw_ik_velocities);
      visualization_debug_frame.forward_kinematics = {
          {mcl::ArmSide::Left, latest_output.raw_left_pose},
          {mcl::ArmSide::Right, latest_output.raw_right_pose}};
      EventStamp emit_stamp{
          schedule->emit_time_ns,
          static_cast<std::uint64_t>(std::max<std::int64_t>(0, schedule->sample_time_ns)),
          0U};
      if (telemetry_enabled) {
        emit_stamp = telemetry_clock->sample();
      }
      auto visualization_frame = mcl::makeIkRenderBatch(
          visualization_debug_frame, presentation, emit_stamp.timestamp_ns);
      mcl::planned_hierarchical_step_otg::appendPlanningRequestPoses(
          visualization_frame, robot.base_frame,
          latest_red_attempt.attempted_reference.left,
          latest_red_attempt.attempted_reference.right);
      mcl::planned_hierarchical_step_otg::appendOtgExecution(
          visualization_frame, joint_names,
          toStdVector(latest_output.state.positions),
          toStdVector(latest_output.state.velocities), robot.base_frame,
          latest_output.left_pose, latest_output.right_pose);

      if (telemetry_enabled) {
        const auto output_timestamp = latest_output.event_timestamp_ns == 0U
                                          ? emit_stamp.timestamp_ns
                                          : latest_output.event_timestamp_ns;
        const auto attempt_timestamp = latest_red_attempt.event_timestamp_ns == 0U
                                           ? emit_stamp.timestamp_ns
                                           : latest_red_attempt.event_timestamp_ns;
        for (auto &pose : visualization_frame.poses) {
          if (pose.channel == contracts::mcl_state_v1::kLeftCartesianIkTopic ||
              pose.channel == contracts::mcl_state_v1::kRightCartesianIkTopic ||
              pose.channel == contracts::mcl_execution_v1::kLeftCartesianExecutionTopic ||
              pose.channel == contracts::mcl_execution_v1::kRightCartesianExecutionTopic) {
            pose.timestamp_ns = output_timestamp;
          } else if (
              pose.channel == contracts::mcl_planning_v1::kLeftCartesianReferenceTopic ||
              pose.channel == contracts::mcl_planning_v1::kRightCartesianReferenceTopic) {
            pose.timestamp_ns = attempt_timestamp;
          } else {
            pose.timestamp_ns = emit_stamp.timestamp_ns;
          }
        }
        for (auto &joints : visualization_frame.joint_states) {
          joints.timestamp_ns = output_timestamp;
        }

        const std::size_t queue_depth =
            red_telemetry->depth() + yellow_telemetry->depth();
        std::vector<TelemetryRecord> records;
        records.reserve(queue_depth);
        drainTelemetryQueue(*red_telemetry, records);
        drainTelemetryQueue(*yellow_telemetry, records);
        double oldest_sample_age_ms = 0.0;
        for (const auto &record : records) {
          if (emit_stamp.timestamp_ns >= record.stamp.timestamp_ns) {
            oldest_sample_age_ms = std::max(
                oldest_sample_age_ms,
                static_cast<double>(emit_stamp.timestamp_ns -
                                    record.stamp.timestamp_ns) /
                    1.0e6);
          }
        }
        auto encode_statistics = telemetry_encoder->append(
            std::move(records), emit_stamp, visualization_frame);

        const bool run_info_due =
            !run_info_published ||
            emit_stamp.run_time_ns - last_run_info_run_time_ns >= 1000000000ULL;
        if (run_info_due) {
          proto::RunInfo run_info;
          const auto info_context = makeSampleContext(
              emit_stamp, proto::ACCEPTED, false, 2U,
              published_target.revision,
              latest_red_attempt.solver_debug.grouped_attempt.has_value()
                  ? latest_red_attempt.solver_debug.grouped_attempt->attempt_revision
                  : 0U,
              latest_output.revision);
          *run_info.mutable_timestamp() = info_context.timestamp();
          run_info.set_run_id(telemetry_run_id);
          run_info.set_app_id(kProgramId);
          run_info.set_source_mode(
              planned_options.source_mode == SourceMode::Replay ? "replay" : "teleop");
          run_info.set_base_frame_id(robot.base_frame);
          for (const auto &name : joint_names) {
            run_info.add_joint_names(name);
          }
          run_info.set_control_rate_hz(options.red_rate_hz);
          run_info.set_avoidance_rate_hz(options.yellow_rate_hz);
          run_info.set_visualization_rate_hz(options.ui_rate_hz);
          run_info.set_schema_version(contracts::mcl_telemetry_v1::kSchemaVersion);
          const auto add_option = [&](const std::string &name,
                                      const std::string &value) {
            auto *option = run_info.add_resolved_options();
            option->set_name(name);
            option->set_value(value);
          };
          add_option("solver_kind", "hierarchical_ik");
          add_option("solver_backend", latest_red_attempt.solver_debug.backend);
          add_option("joint_limit_policy",
                     latest_red_attempt.solver_debug.joint_limit_policy);
          add_option(
              "joint_target_mode",
              jointTargetModeName(planned_options.joint_target.mode));
          add_option("cartesian_algorithm", "jerk_limited");
          add_option("joint_algorithm", "jerk_limited");
          add_option(
              "cartesian_synchronization",
              planningSynchronizationName(
                  planned_options.planning.cartesian_synchronization));
          add_option(
              "joint_synchronization",
              planningSynchronizationName(
                  planned_options.planning.joint_synchronization));
          add_option("deadline_policy",
                     options.deadline_policy == mcl::DeadlinePolicy::Strict
                         ? "strict"
                         : "monitor");
          add_option(
              "max_linear_velocity_mps",
              std::to_string(planned_options.planning.max_linear_velocity_mps));
          add_option(
              "max_linear_acceleration_mps2",
              std::to_string(
                  planned_options.planning.max_linear_acceleration_mps2));
          add_option(
              "max_linear_jerk_mps3",
              std::to_string(planned_options.planning.max_linear_jerk_mps3));
          add_option(
              "max_angular_velocity_radps",
              std::to_string(planned_options.planning.max_angular_velocity_rps));
          add_option(
              "max_angular_acceleration_radps2",
              std::to_string(
                  planned_options.planning.max_angular_acceleration_rps2));
          add_option(
              "max_angular_jerk_radps3",
              std::to_string(planned_options.planning.max_angular_jerk_rps3));
          add_option("joint_stream_profile_revision",
                     options.robot.joint_stream.source_revision);
          add_option("joint_stream_profile_path",
                     options.robot.joint_stream.source_path);
          add_option("joint_stream_profile_sha256",
                     options.robot.joint_stream.source_sha256);
          add_option("visualization_sink",
                     options.visualization.mcap_path.has_value()
                         ? "foxglove_websocket+mcap"
                         : "foxglove_websocket");
          telemetry_encoder->appendMessage(
              contracts::mcl_telemetry_v1::kRunInfoTopic, run_info,
              emit_stamp, emit_stamp, visualization_frame,
              encode_statistics);
          run_info_published = true;
          last_run_info_run_time_ns = emit_stamp.run_time_ns;
        }

        const bool will_finish_replay =
            loaded_replay.has_value() && replay_settled.load() &&
            replay_source->sourceIndex() + 1U ==
                loaded_replay->timeline.timeline.size();
        if (loaded_replay.has_value()) {
          const auto &source = replay_source->sourceFrame();
          const bool input_consumed =
              replay_last_consumed_revision.load() >= published_target.revision;
          const bool cartesian_finished =
              latest_output.planner_state == mcc::PlanningState::Finished;
          const bool pose_within =
              latest_output.left_position_error_m <=
                  planned_options.replay_settling.fk_position_m &&
              latest_output.right_position_error_m <=
                  planned_options.replay_settling.fk_position_m &&
              latest_output.left_orientation_error_rad <=
                  planned_options.replay_settling.fk_orientation_rad &&
              latest_output.right_orientation_error_rad <=
                  planned_options.replay_settling.fk_orientation_rad;
          const bool velocity_within =
              maximumAbsolute(latest_output.state.velocities) <=
              planned_options.replay_settling.velocity_rad_per_s;
          const bool acceleration_within =
              maximumAbsolute(latest_output.state.accelerations) <=
              planned_options.replay_settling.acceleration_rad_per_s2;
          proto::ReplayTelemetry replay_message;
          *replay_message.mutable_context() = makeSampleContext(
              emit_stamp, proto::ACCEPTED, false, 2U,
              published_target.revision,
              latest_red_attempt.solver_debug.grouped_attempt.has_value()
                  ? latest_red_attempt.solver_debug.grouped_attempt->attempt_revision
                  : 0U,
              latest_output.revision);
          replay_message.set_source_sequence(source.sequence);
          const auto set_source_time = [&](google::protobuf::Timestamp *output,
                                           const std::optional<std::int64_t> &value) {
            if (!value.has_value() || *value < 0) {
              return;
            }
            const auto timestamp_context = makeSampleContext(
                EventStamp{static_cast<std::uint64_t>(*value), 0U, 0U},
                proto::UNSPECIFIED, false, 0U, 0U, 0U, 0U);
            *output = timestamp_context.timestamp();
          };
          set_source_time(replay_message.mutable_source_header_time(),
                          source.value.left.time.header_stamp_ns);
          set_source_time(replay_message.mutable_source_log_time(),
                          source.value.left.time.log_time_ns);
          set_source_time(replay_message.mutable_source_publish_time(),
                          source.value.left.time.publish_time_ns);
          replay_message.set_source_time_from_start_ns(
              static_cast<std::uint64_t>(source.source_time_from_start_ns));
          replay_message.set_projected_time_ns(
              static_cast<std::uint64_t>(source.projected_time_ns));
          replay_message.set_playback_rate(planned_options.replay->playback_rate);
          replay_message.set_paused(replay_source->paused());
          replay_message.set_end_of_stream(
              replay_source->endOfStream() || will_finish_replay);
          replay_message.set_input_consumed(input_consumed);
          replay_message.set_cartesian_finished(cartesian_finished);
          replay_message.set_pose_error_within_tolerance(pose_within);
          replay_message.set_joint_velocity_within_tolerance(velocity_within);
          replay_message.set_joint_acceleration_within_tolerance(
              acceleration_within);
          replay_message.set_settled_cycles(
              replay_settled_cycle_count.load());
          replay_message.set_required_cycles(
              planned_options.replay_settling.required_cycles);
          telemetry_encoder->appendMessage(
              contracts::mcl_telemetry_v1::kReplayTopic, replay_message,
              emit_stamp, emit_stamp, visualization_frame,
              encode_statistics);
          if (will_finish_replay && !replay_eos_event_published) {
            visualization_frame.logs.push_back(motion_control::viz::LogSample{
                contracts::mcl_telemetry_v1::kEventsTopic,
                motion_control::viz::LogLevel::Info,
                "replay end of stream after settling",
                "replay-eos",
                {},
                0U,
                emit_stamp.timestamp_ns});
            replay_eos_event_published = true;
          }
        }

        if (pending_fault_event.has_value()) {
          visualization_frame.logs.push_back(motion_control::viz::LogSample{
              contracts::mcl_telemetry_v1::kEventsTopic,
              motion_control::viz::LogLevel::Fatal,
              *pending_fault_event,
              "fault-hold",
              {},
              0U,
              emit_stamp.timestamp_ns});
          pending_fault_event.reset();
        }

        const std::uint64_t dropped_samples =
            red_telemetry->dropped() + yellow_telemetry->dropped();
        if (dropped_samples > last_reported_dropped_samples) {
          visualization_frame.logs.push_back(motion_control::viz::LogSample{
              contracts::mcl_telemetry_v1::kEventsTopic,
              motion_control::viz::LogLevel::Warning,
              "telemetry queue dropped " +
                  std::to_string(dropped_samples -
                                 last_reported_dropped_samples) +
                  " samples",
              "telemetry-queue-drop",
              {},
              0U,
              emit_stamp.timestamp_ns});
          last_reported_dropped_samples = dropped_samples;
        }

        proto::TransportTelemetry transport;
        *transport.mutable_context() = makeSampleContext(
            emit_stamp, proto::ACCEPTED, false, 2U,
            published_target.revision,
            latest_red_attempt.solver_debug.grouped_attempt.has_value()
                ? latest_red_attempt.solver_debug.grouped_attempt->attempt_revision
                : 0U,
            latest_output.revision);
        transport.set_sink_kind(
            options.visualization.mcap_path.has_value()
                ? "foxglove_websocket+mcap"
                : "foxglove_websocket");
        transport.set_connected(true);
        transport.set_queue_depth(queue_depth);
        transport.set_queue_capacity(kTelemetryQueueCapacity * 2U);
        transport.set_dropped_samples(dropped_samples);
        transport.set_oldest_sample_age_ms(oldest_sample_age_ms);
        transport.set_encode_time_ms(encode_statistics.encode_time_ms);
        transport.set_write_time_ms(previous_write_time_ms);
        transport.set_serialized_bytes(encode_statistics.serialized_bytes);
        telemetry_encoder->appendMessage(
            contracts::mcl_telemetry_v1::kTransportTopic, transport,
            emit_stamp, emit_stamp, visualization_frame,
            encode_statistics);
      }

      const auto write_started = std::chrono::steady_clock::now();
      visualization_sink->write(visualization_frame);
      if (telemetry_enabled) {
        previous_write_time_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - write_started).count();
      }
      ++publish_count;

      if (planned_options.source_mode == SourceMode::Replay &&
          replay_settled.load()) {
        replay_source->markFrameProcessed();
        if (replay_source->endOfStream()) {
          replay_completed = true;
          break;
        }
      }
    }
    if (schedule->draw_due) {
      const mcl::PlannedGroupedTuiSnapshot tui_snapshot{
          &frame,
          &presentation,
          tui_debug,
          publish_count,
          visualization_sink->status(),
          kTitle,
          input.status()};
      tui.render(tui_snapshot);
    }
    if (planned_options.replay.has_value() &&
        planned_options.replay->execution_mode ==
            mcl::data::ExecutionMode::Batch) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    } else {
      ui_scheduler.sleep();
    }
  }

  workers.join();
  const auto recorded_fault =
      held_fault.has_value() ? held_fault : fault.snapshot();
  visualization_sink->flush();
  visualization_sink->close();

  const auto red_stats = red_worker_diagnostics.snapshot();
  if (planned_options.replay.has_value()) {
    const auto trace_path = planned_options.replay->output_dir / "trace.csv";
    std::string trace_sha256;
    if (planned_options.replay_trace_enabled) {
      std::lock_guard<std::mutex> lock(replay_trace_mutex);
      replay::writeTextFile(trace_path, replay_trace.str());
      trace_sha256 = mcl::sha256_file(trace_path);
    }
    replay::ReplayExecutionMetadata execution;
    execution.app = kProgramId;
    execution.topology =
        "planned-red-yellow-hierarchical-step+split-reference-joint-otg";
    execution.solver = "motion_control_core+CartesianPlanner+JointPlanner";
    execution.backend = "proxqp+ruckig";
    execution.red_rate_hz = options.red_rate_hz;
    execution.yellow_rate_hz = options.yellow_rate_hz;
    execution.consumed_frame_count = replay_source->consumedFrameCount();
    execution.dropped_frame_count = replay_source->droppedFrameCount();
    execution.accepted_count = replay_accepted_solve_count;
    execution.rejected_count = replay_rejected_solve_count;
    execution.deadline_miss_count =
        red_stats.deadline_miss_count + replay_source->deadlineMissCount();
    execution.resolved_config = {
        {"regularization", std::to_string(options.solver.regularization)},
        {"maximum_hard_violation",
         std::to_string(options.solver.maximum_accepted_hard_violation)},
        {"cartesian_progress_weight",
         std::to_string(options.solver.cartesian_progress_weight)},
        {"red_proxqp_absolute_tolerance",
         std::to_string(options.solver.red_proxqp_absolute_tolerance)},
        {"red_proxqp_primal_infeasibility_tolerance",
         std::to_string(
             options.solver.red_proxqp_primal_infeasibility_tolerance)},
        {"yellow_to_red_coupling_weight",
         std::to_string(options.solver.yellow_to_red_coupling_weight)},
        {"minimum_collision_distance_m",
         std::to_string(options.solver.minimum_collision_distance_m)},
        {"collision_influence_distance_m",
         std::to_string(options.solver.collision_influence_distance_m)},
        {"collision_weight", std::to_string(options.solver.collision_weight)},
        {"max_linear_velocity_mps",
         std::to_string(planned_options.planning.max_linear_velocity_mps)},
        {"max_angular_velocity_rps",
         std::to_string(planned_options.planning.max_angular_velocity_rps)},
        {"joint_target_mode",
         mcl::planned_hierarchical_step_otg::jointTargetModeName(
             planned_options.joint_target.mode)},
        {"replay_trace",
         planned_options.replay_trace_enabled ? "on" : "off"},
    };
    auto manifest =
        replay::makeReplayManifest(*planned_options.replay, *loaded_replay,
                                   execution, trace_sha256);
    if (!planned_options.replay_trace_enabled) {
      manifest["artifacts"].removeMember("trace.csv");
    }
    auto &joint_otg = manifest["joint_otg"];
    joint_otg["target_mode"] =
        mcl::planned_hierarchical_step_otg::jointTargetModeName(
            planned_options.joint_target.mode);
    joint_otg["sample_period_s"] = 1.0 / options.red_rate_hz;
    joint_otg["algorithm"] =
        mcl::planned_hierarchical_step_otg::jointPlanningAlgorithmName(
            planned_options.planning.joint_algorithm);
    joint_otg["synchronization"] =
        mcl::planned_hierarchical_step_otg::planningSynchronizationName(
            planned_options.planning.joint_synchronization);
    joint_otg["execution_semantics"] =
        "per Red tick JointPlanner::plan plus first JointPlanner::step sample; "
        "not persistent "
        "Ruckig.update equivalence";
    joint_otg["feedback_topology"] = "split-ik-reference-and-otg-execution";
    joint_otg["red_ik_feedback_source"] = "previous accepted raw IK P/V";
    joint_otg["joint_planner_feedback_source"] = "previous accepted OTG P/V/A";
    joint_otg["yellow_feedback_source"] = "previous accepted OTG P/V";
    joint_otg["future_o1_startup"] =
        "first two accepted live samples use latest P and zero V";
    joint_otg["future_o1_velocity_deadband_rad_per_s"] =
        planned_options.joint_target.future_o1_velocity_deadband_rad_per_s;
    joint_otg["profile_source_revision"] =
        options.robot.joint_stream.source_revision;
    joint_otg["profile_source_path"] =
        options.robot.joint_stream.source_path;
    joint_otg["profile_source_sha256"] =
        options.robot.joint_stream.source_sha256;
    joint_otg["profile_overrides"]["max_jerk_rad_per_s3"] =
        options.robot.joint_stream.max_jerk_rad_per_s3.front();
    joint_otg["profile_overrides"]["reason"] =
        options.robot.joint_stream.jerk_override_reason;
    joint_otg["position_limits_source"] = "runtime-loaded R1 URDF model";
    joint_otg["projection_event_count"] =
        Json::UInt64(latest_output.projection_event_count);
    joint_otg["projection_cycle_count"] =
        Json::UInt64(latest_output.projection_cycle_count);
    joint_otg["replay_settled_cycles"] =
        Json::UInt64(replay_settled_cycle_count.load());
    joint_otg["replay_required_settled_cycles"] =
        Json::UInt64(planned_options.replay_settling.required_cycles);
    joint_otg["replay_completion_thresholds"]["fk_position_m"] =
        planned_options.replay_settling.fk_position_m;
    joint_otg["replay_completion_thresholds"]["fk_orientation_rad"] =
        planned_options.replay_settling.fk_orientation_rad;
    joint_otg["replay_completion_thresholds"]["velocity_rad_per_s"] =
        planned_options.replay_settling.velocity_rad_per_s;
    joint_otg["replay_completion_thresholds"]["acceleration_rad_per_s2"] =
        planned_options.replay_settling.acceleration_rad_per_s2;
    joint_otg["deadline_miss_count"] =
        Json::UInt64(red_stats.deadline_miss_count);
    joint_otg["skipped_release_count"] =
        Json::UInt64(red_stats.skipped_release_count);
    joint_otg["failure_layer"] = recorded_fault.has_value()
                                     ? failureLayer(*recorded_fault)
                                     : std::string{};
    for (std::size_t index = 0U; index < joint_names.size(); ++index) {
      joint_otg["joint_names"].append(joint_names[index]);
      joint_otg["position_lower"].append(
          joint_otg_limits.position_lower[index]);
      joint_otg["position_upper"].append(
          joint_otg_limits.position_upper[index]);
      joint_otg["max_velocity_rad_per_s"].append(
          options.robot.joint_stream.max_velocity_rad_per_s[index]);
      joint_otg["max_acceleration_rad_per_s2"].append(
          options.robot.joint_stream.max_acceleration_rad_per_s2[index]);
      joint_otg["max_jerk_rad_per_s3"].append(
          options.robot.joint_stream.max_jerk_rad_per_s3[index]);
    }
    replay::writeTextFile(planned_options.replay->output_dir / "manifest.json",
                          jsonText(manifest));
    const auto status = replay::makeReplayStatus(
        *loaded_replay, execution,
        recorded_fault.has_value()
            ? "failed"
            : (replay_completed ? "succeeded" : "stopped"),
        recorded_fault.has_value() ? faultSummary(*recorded_fault)
                                   : std::string{});
    replay::writeTextFile(planned_options.replay->output_dir / "status.json",
                          jsonText(status));
  }

  if (recorded_fault.has_value()) {
    throw std::runtime_error(faultSummary(*recorded_fault));
  }
  if (red_stats.recoverable_rejection_count > 0) {
    std::ostringstream detail;
    detail << "recoverable_rejections="
           << red_stats.recoverable_rejection_count;
    if (last_recoverable_rejection.has_value()) {
      detail << " last_rejected_target_revision="
             << last_recoverable_rejection->target.revision;
      if (!last_recoverable_rejection->detail.empty()) {
        detail << " last_rejection=" << last_recoverable_rejection->detail;
      }
    }
    normal_exit_detail = detail.str();
  }
  return EXIT_SUCCESS;
}

} // namespace motion_control_lab::planned_hierarchical_step_otg
