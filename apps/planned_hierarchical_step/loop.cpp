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
#include "contracts/visualization/mcl_planning_v1.hpp"
#include "loop.hpp"
#include "motion_control_lab/run_artifacts.hpp"
#include "motion_control_lab/sha256.hpp"
#include "options.hpp"
#include "planning.hpp"
#include "solver.hpp"

namespace motion_control_lab::planned_hierarchical_step {
namespace {

namespace mcc = motion_control::core;
namespace mcl = motion_control_lab;
namespace replay = motion_control_lab::replay;

using mcl::toEigen;
using mcl::toStdVector;

constexpr const char *kProgramId = "mcl_planned_hierarchical_step";
constexpr const char *kTitle = "Motion Control Planned Hierarchical Step";
constexpr std::array<unsigned int, 1> kUiCpuAffinity{5};
constexpr std::array<unsigned int, 1> kRedCpuAffinity{6};
constexpr std::array<unsigned int, 1> kYellowCpuAffinity{7};
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
    output.objective_value = 0.0;
    output.primal_residual = 0.0;
    output.dual_residual = 0.0;
    output.maximum_hard_violation = diagnostics.maximum_hard_violation;
    output.qp_solve_time_ms = diagnostics.solve_time_ms;
    output.qp_iterations = diagnostics.iterations;
    output.active_set_size = 0;
    output.warm_start_used = last_pass->warm_start_used;
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
  mcc::Pose left{mcc::Pose::Identity()};
  mcc::Pose right{mcc::Pose::Identity()};
};

struct StateSnapshot {
  std::uint64_t sequence{0};
  std::int64_t monotonic_time_nanoseconds{0};
  Eigen::VectorXd positions;
  Eigen::VectorXd velocities;
};

struct TaskScaleSnapshot {
  bool active{false};
  double scale{1.0};
  bool degraded{false};
  bool stuck{false};
};

struct RedOutputSnapshot {
  std::uint64_t revision{0};
  TargetSnapshot accepted_target;
  TargetSnapshot source_goal;
  mcc::CartesianTrajectorySample accepted_planner_sample;
  mcc::PlanningState planner_state{mcc::PlanningState::Idle};
  StateSnapshot state;
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
  RedAttemptState state{RedAttemptState::Accepted};
  TargetSnapshot target;
  TargetSnapshot attempted_reference;
  mcl::SolverDebug solver_debug;
  mcl::planned_hierarchical_step::RetargetClampDiagnostics retarget_clamp;
  std::uint64_t retarget_clamp_target_revision{0U};
  std::string detail;
};

const char *retargetClampComponentName(
    mcl::planned_hierarchical_step::RetargetClampComponent component) {
  using Component = mcl::planned_hierarchical_step::RetargetClampComponent;
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

std::string jsonText(const Json::Value &value) {
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "  ";
  return Json::writeString(builder, value) + "\n";
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
    const mcl::planned_hierarchical_step::SolverOptions &options,
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

void appendPlanningRequestPoses(motion_control::viz::RenderBatch &frame,
                                const std::string &reference_frame,
                                const Eigen::Isometry3d &left_pose,
                                const Eigen::Isometry3d &right_pose) {
  namespace contract = contracts::mcl_planning_v1;
  const auto sample = [&](const char *channel, const Eigen::Isometry3d &pose) {
    const Eigen::Quaterniond orientation(pose.linear());
    motion_control::viz::PoseSample result;
    result.channel = channel;
    result.frame_id = reference_frame;
    result.pose.position_m = {pose.translation().x(), pose.translation().y(),
                              pose.translation().z()};
    result.pose.orientation_xyzw = {orientation.x(), orientation.y(),
                                    orientation.z(), orientation.w()};
    return result;
  };
  frame.poses.reserve(frame.poses.size() + 2U);
  frame.poses.push_back(sample(contract::kLeftCartesianReferenceTopic, left_pose));
  frame.poses.push_back(
      sample(contract::kRightCartesianReferenceTopic, right_pose));
}

int runLoop(Options planned_options, const R1RobotConfig &robot,
            SolverRuntime &solver, const SolverHandles &handles,
            mcc::CartesianPlanner &cartesian_planner,
            const std::vector<Eigen::Index> &active_joint_full_indices,
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
              std::filesystem::path{"runs/mcl_planned_hierarchical_step"});
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
  TargetSnapshot initial_target = warmup_target;
  initial_target.revision = 1;
  if (loaded_replay.has_value()) {
    const auto &first = replay_source->sourceFrame();
    initial_target.left =
        first.value.left.pose * robot.left_tcp_offset.inverse();
    initial_target.right =
        first.value.right.pose * robot.right_tcp_offset.inverse();
  }

  RedOutputSnapshot initial_output;
  initial_output.accepted_target = warmup_target;
  initial_output.source_goal = initial_target;
  initial_output.state = initial_state;
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

  mcl::WorkerStopController stop_controller;
  mcl::GroupedFaultState fault;
  mcl::PeriodicWorkerDiagnostics red_worker_diagnostics;
  mcl::PeriodicWorkerDiagnostics yellow_worker_diagnostics;
  mcl::RollingPercentiles red_solve_time_percentiles;
  mcl::RollingPercentiles yellow_solve_time_percentiles;
  WorkerThreads workers(stop_controller);
  std::mutex replay_trace_mutex;
  std::ostringstream replay_trace;
  std::size_t replay_accepted_solve_count = 0;
  std::size_t replay_rejected_solve_count = 0;
  std::atomic<std::uint64_t> replay_last_consumed_revision{0};
  replay_trace << "attempt,source_revision,original_logical_timestamp_ns,"
                  "source_time_from_start_ns,"
                  "projected_timestamp_ns,left_header_stamp_ns,left_log_time_"
                  "ns,left_publish_time_ns,"
                  "right_header_stamp_ns,right_log_time_ns,right_publish_time_"
                  "ns,accepted,solver_status,"
                  "solve_time_ms,maximum_hard_violation,goal_left_xyz,"
                  "reference_left_xyz,"
                  "reference_left_twist,reference_left_acceleration,actual_fk_"
                  "left_xyz,positions,velocities\n";

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
    mcl::runPeriodicWorker(
        {mcl::WorkerGroup::Yellow, options.yellow_rate_hz,
         options.deadline_policy},
        stop_controller, fault, yellow_worker_diagnostics,
        [&](double, std::int64_t) {
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
          return mcl::WorkerIterationResult{
              accepted ? mcl::WorkerIterationOutcome::Accepted
                       : mcl::WorkerIterationOutcome::FatalRejected,
              diagnostics.attempt_revision, diagnostics.solve_time_ms,
              accepted ? std::string{}
                       : rejectedAttemptDetail(status, diagnostics)};
        });
  });

  workers.red = std::thread([&]() {
    const auto affinity_binding =
        affinity_domain.bindCurrentThread(kProgramId, "red", kRedCpuAffinity);
    red_affinity_to_ui.publish(affinity_binding);
    StateSnapshot state = initial_state;
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
    const auto recordPlannerFailure = [&](const std::string &detail) {
      if (!loaded_replay.has_value()) {
        return;
      }
      ++replay_rejected_solve_count;
      const std::size_t source_index = std::min<std::size_t>(
          target.revision > 0U ? target.revision - 1U : 0U,
          loaded_replay->timeline.timeline.size() - 1U);
      const auto &source = loaded_replay->timeline.timeline.at(source_index);
      std::lock_guard<std::mutex> lock(replay_trace_mutex);
      replay_trace
          << target.revision << ',' << source.sequence << ','
          << source.original_logical_time_ns << ','
          << source.source_time_from_start_ns << ',' << source.projected_time_ns
          << ','
          << replay::optionalTimestamp(source.value.left.time.header_stamp_ns)
          << ','
          << replay::optionalTimestamp(source.value.left.time.log_time_ns)
          << ','
          << replay::optionalTimestamp(source.value.left.time.publish_time_ns)
          << ','
          << replay::optionalTimestamp(source.value.right.time.header_stamp_ns)
          << ','
          << replay::optionalTimestamp(source.value.right.time.log_time_ns)
          << ','
          << replay::optionalTimestamp(source.value.right.time.publish_time_ns)
          << ",false," << replay::csvEscape(detail) << ",,,"
          << traceEigenVector(target.left.translation()) << ",,,,"
          << traceEigenVector(output.left_pose.translation()) << ','
          << traceEigenVector(state.positions) << ','
          << traceEigenVector(state.velocities) << '\n';
    };
    mcl::runPeriodicWorker(
        {mcl::WorkerGroup::Red, options.red_rate_hz, options.deadline_policy},
        stop_controller, fault, red_worker_diagnostics,
        [&](double, std::int64_t sample_time_ns) {
          if (target_to_red.readLatest(target) && loaded_replay.has_value()) {
            replay_last_consumed_revision.store(target.revision);
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
                mcl::planned_hierarchical_step::clampRetargetCurrentState(
                    retarget_request);
            attempt.retarget_clamp_target_revision = target.revision;
            const auto planning_status = cartesian_planner.replan(
                retarget_request, planning_diagnostics);
            if (!planning_status.ok()) {
              attempt.state =
                  planned_options.source_mode == SourceMode::Teleop &&
                          planning_status.code == mcc::StatusCode::Infeasible
                      ? RedAttemptState::RecoverableRejected
                      : RedAttemptState::FatalRejected;
              attempt.target = target;
              attempt.detail =
                  "Cartesian replan failed: " + planning_status.message;
              recordPlannerFailure(attempt.detail);
              red_attempt_to_ui.publish(attempt);
              if (attempt.state == RedAttemptState::RecoverableRejected) {
                rejected_target_revision = target.revision;
              }
              return mcl::WorkerIterationResult{
                  attempt.state == RedAttemptState::RecoverableRejected
                      ? mcl::WorkerIterationOutcome::RecoverableRejected
                      : mcl::WorkerIterationOutcome::FatalRejected,
                  target.revision, 0.0, attempt.detail};
            }
            planned_goal_revision = target.revision;
            staged_planner_sample.reset();
          }
          if (!staged_planner_sample.has_value()) {
            mcc::CartesianTrajectorySample next_sample;
            const auto planning_status =
                cartesian_planner.step(next_sample, planning_diagnostics);
            if (!planning_status.ok()) {
              attempt.state = RedAttemptState::FatalRejected;
              attempt.target = target;
              attempt.detail =
                  "Cartesian planner step failed: " + planning_status.message;
              recordPlannerFailure(attempt.detail);
              red_attempt_to_ui.publish(attempt);
              return mcl::WorkerIterationResult{
                  mcl::WorkerIterationOutcome::FatalRejected, target.revision,
                  0.0, attempt.detail};
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
          request.captured_state = capturedState(state);
          addCartesianTargets(handles.red, staged_for_attempt, request);
          const auto status = solver.solveRed(request, solution, diagnostics);
          red_solve_time_percentiles.record(diagnostics.solve_time_ms);
          const bool accepted = status.ok();
          if (accepted) {
            for (std::size_t index = 0;
                 index < active_joint_full_indices.size(); ++index) {
              const auto full_index = active_joint_full_indices[index];
              state.positions(full_index) =
                  solution.kinematics_solution.joint_positions(
                      static_cast<Eigen::Index>(index));
              state.velocities(full_index) =
                  solution.kinematics_solution.joint_velocities(
                      static_cast<Eigen::Index>(index));
            }
            ++state.sequence;
            state.monotonic_time_nanoseconds = sample_time_ns;
            state_to_yellow.publish(state);

            output.revision = diagnostics.value_revision;
            output.accepted_target = reference;
            output.source_goal = target;
            accepted_planner_sample = *staged_planner_sample;
            output.accepted_planner_sample = accepted_planner_sample;
            output.planner_state = planning_diagnostics.state;
            staged_planner_sample.reset();
            output.state = state;
            output.left_pose =
                requirePose(solution.kinematics_solution.solved_poses,
                            robot.left_end_effector_frame)
                    .pose;
            output.right_pose =
                requirePose(solution.kinematics_solution.solved_poses,
                            robot.right_end_effector_frame)
                    .pose;
            output.solve_time_ms = diagnostics.solve_time_ms;
            output.iterations = diagnostics.iterations;
            output.converged = diagnostics.converged;
            fillRedDiagnostics(handles.red, diagnostics, output);
            updateSolverDebug(output.solver_debug, diagnostics,
                              solution.kinematics_solution.disposition);
            output_to_ui.publish(output);

            attempt.state = RedAttemptState::Accepted;
            attempt.target = target;
            attempt.solver_debug = output.solver_debug;
            attempt.detail.clear();
            red_attempt_to_ui.publish(attempt);
          } else {
            attempt.state = planned_options.source_mode == SourceMode::Teleop &&
                                    status.code == mcc::StatusCode::Infeasible
                                ? RedAttemptState::RecoverableRejected
                                : RedAttemptState::FatalRejected;
            attempt.target = target;
            updateSolverDebug(attempt.solver_debug, diagnostics,
                              mcc::ResultDisposition::Rejected);
            attempt.detail = rejectedAttemptDetail(status, diagnostics);
            red_attempt_to_ui.publish(attempt);
            if (attempt.state == RedAttemptState::RecoverableRejected) {
              rejected_target_revision = target.revision;
            }
          }
          if (loaded_replay.has_value()) {
            if (accepted) {
              ++replay_accepted_solve_count;
            } else {
              ++replay_rejected_solve_count;
            }
            const std::size_t source_index = std::min<std::size_t>(
                target.revision > 0U ? target.revision - 1U : 0U,
                loaded_replay->timeline.timeline.size() - 1U);
            const auto &source =
                loaded_replay->timeline.timeline.at(source_index);
            std::lock_guard<std::mutex> lock(replay_trace_mutex);
            replay_trace
                << diagnostics.attempt_revision << ',' << source.sequence << ','
                << source.original_logical_time_ns << ','
                << source.source_time_from_start_ns << ','
                << source.projected_time_ns << ','
                << replay::optionalTimestamp(
                       source.value.left.time.header_stamp_ns)
                << ','
                << replay::optionalTimestamp(source.value.left.time.log_time_ns)
                << ','
                << replay::optionalTimestamp(
                       source.value.left.time.publish_time_ns)
                << ','
                << replay::optionalTimestamp(
                       source.value.right.time.header_stamp_ns)
                << ','
                << replay::optionalTimestamp(
                       source.value.right.time.log_time_ns)
                << ','
                << replay::optionalTimestamp(
                       source.value.right.time.publish_time_ns)
                << ',' << std::boolalpha << accepted << ','
                << replay::csvEscape(accepted ? "ok" : attempt.detail) << ','
                << diagnostics.solve_time_ms << ','
                << diagnostics.maximum_hard_violation << ','
                << traceEigenVector(target.left.translation()) << ','
                << traceEigenVector(reference.left.translation()) << ','
                << traceEigenVector(staged_for_attempt.frames.at(0).twist)
                << ','
                << traceEigenVector(
                       staged_for_attempt.frames.at(0).acceleration)
                << ',' << traceEigenVector(output.left_pose.translation())
                << ',' << traceEigenVector(state.positions) << ','
                << traceEigenVector(state.velocities) << '\n';
          }
          return mcl::WorkerIterationResult{
              accepted ? mcl::WorkerIterationOutcome::Accepted
              : planned_options.source_mode == SourceMode::Teleop &&
                      status.code == mcc::StatusCode::Infeasible
                  ? mcl::WorkerIterationOutcome::RecoverableRejected
                  : mcl::WorkerIterationOutcome::FatalRejected,
              diagnostics.attempt_revision, diagnostics.solve_time_ms,
              accepted ? std::string{}
                       : rejectedAttemptDetail(status, diagnostics)};
        });
  });

  mcl::SingleRateScheduler ui_scheduler(
      {options.ui_rate_hz, options.duration_s});
  TargetSnapshot published_target = initial_target;
  TargetSnapshot last_command_target = initial_target;
  std::vector<mcl::ArmTarget> latest_input_targets = armTargets(initial_target);
  if (planned_options.source_mode == SourceMode::Replay) {
    const auto & source = replay_source->sourceFrame();
    latest_input_targets = {
      {mcl::ArmSide::Left, source.value.left.pose},
      {mcl::ArmSide::Right, source.value.right.pose}};
  }
  RedOutputSnapshot latest_output = initial_output;
  RedAttemptSnapshot latest_red_attempt = initial_red_attempt;
  mcl::SelfCollisionDebug latest_collision_debug = initial_collision_debug;
  mcl::SolverDebug latest_yellow_solver_debug = initial_yellow_solver_debug;
  mcl::CpuAffinityBinding latest_red_affinity = initial_red_affinity;
  mcl::CpuAffinityBinding latest_yellow_affinity = initial_yellow_affinity;
  std::optional<mcl::RejectedTargetDebug> rejected_target;
  std::optional<RedAttemptSnapshot> last_recoverable_rejection;
  std::optional<mcl::GroupedWorkerFault> held_fault;
  std::uint64_t handled_rejected_target_revision = 0;
  std::size_t publish_count = 0;
  bool replay_completed = false;
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
  frame.solvers[1].ik_solve_time_percentiles =
      yellow_solve_time_percentiles.snapshot();
  frame.cpu_affinities = {mcl::makeCpuAffinityDebug(ui_affinity_binding),
                          mcl::makeCpuAffinityDebug(latest_red_affinity),
                          mcl::makeCpuAffinityDebug(latest_yellow_affinity)};
  frame.self_collisions = {initial_collision_debug};

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
      for (const auto control : input.consumeSourceControls()) {
        replay_source->applyControl(control);
      }
      const bool worker_consumed_current =
          replay_last_consumed_revision.load() >= published_target.revision;
      const bool may_advance = planned_options.replay->execution_mode ==
                                   mcl::data::ExecutionMode::Realtime ||
                               worker_consumed_current;
      const auto advance =
          may_advance ? replay_source->advance(static_cast<std::int64_t>(
                            std::llround(1.0e9 / options.ui_rate_hz)))
                      : replay::ReplayAdvance{};
      replay_source->waitForCurrentFrame();
      input.setPaused(replay_source->paused(), replay_source->status().detail);
      if (advance.frame_changed) {
        const auto &source = replay_source->sourceFrame();
        latest_input_targets = {
            {mcl::ArmSide::Left, source.value.left.pose},
            {mcl::ArmSide::Right, source.value.right.pose}};
        published_target.revision = source.sequence + 1U;
        published_target.left =
            source.value.left.pose * robot.left_tcp_offset.inverse();
        published_target.right =
            source.value.right.pose * robot.right_tcp_offset.inverse();
        last_command_target = published_target;
        target_to_red.publish(published_target);
        input.setTargetPose(mcl::ArmSide::Left, published_target.left,
                            "Replay goal advanced");
        input.setTargetPose(mcl::ArmSide::Right, published_target.right,
                            "Replay goal advanced");
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
      if (planned_options.source_mode == SourceMode::Replay) {
        frame.replay_frame_progress = mcl::ReplayFrameProgressDebug{
            replay_source->sourceIndex(),
            loaded_replay->timeline.timeline.size()};
      }

      mcl::IkDebugFrame visualization_debug_frame = frame;
      visualization_debug_frame.targets = armTargets(latest_red_attempt.target);
      auto visualization_frame = mcl::makeIkRenderBatch(
          visualization_debug_frame, presentation, schedule->emit_time_ns);
      mcl::planned_hierarchical_step::appendPlanningRequestPoses(
          visualization_frame, robot.base_frame,
          latest_red_attempt.attempted_reference.left,
          latest_red_attempt.attempted_reference.right);
      visualization_sink->write(visualization_frame);
      ++publish_count;
      const mcl::PlannedGroupedTuiSnapshot tui_snapshot{
          &frame,
          &presentation,
          std::nullopt,
          publish_count,
          visualization_sink->status(),
          kTitle,
          input.status()};
      tui.render(tui_snapshot);

      if (planned_options.source_mode == SourceMode::Replay &&
          latest_output.source_goal.revision == published_target.revision &&
          latest_output.accepted_target.revision == published_target.revision &&
          latest_output.planner_state == mcc::PlanningState::Finished) {
        replay_source->markFrameProcessed();
        if (replay_source->endOfStream()) {
          replay_completed = true;
          break;
        }
      }
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
    {
      std::lock_guard<std::mutex> lock(replay_trace_mutex);
      replay::writeTextFile(trace_path, replay_trace.str());
    }
    replay::ReplayExecutionMetadata execution;
    execution.app = kProgramId;
    execution.topology = "planned-red-yellow-hierarchical-step";
    execution.solver = "motion_control_core+CartesianPlanner";
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
    };
    const auto manifest =
        replay::makeReplayManifest(*planned_options.replay, *loaded_replay,
                                   execution, mcl::sha256_file(trace_path));
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

} // namespace motion_control_lab::planned_hierarchical_step
