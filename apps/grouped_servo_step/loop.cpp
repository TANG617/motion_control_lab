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
#include "components/tui/standard_ik_tui.hpp"
#include "components/tui/tui_renderer.hpp"
#include "components/visualization/preview_projection.hpp"
#include "components/visualization/preview_transport.hpp"
#include "contracts/presentation/ik_app_snapshot.hpp"
#include "loop.hpp"
#include "motion_control_lab/run_artifacts.hpp"
#include "motion_control_lab/sha256.hpp"
#include "solver.hpp"

namespace motion_control_lab {
namespace {

namespace grouped_diagnostics = motion_control::core;

const char *
resultDispositionName(grouped_diagnostics::ResultDisposition value) {
  return grouped_diagnostics::isAccepted(value) ? "accepted" : "rejected";
}

const char *
jointLimitPolicyName(grouped_diagnostics::KinematicsJointLimitPolicy value) {
  switch (value) {
  case grouped_diagnostics::KinematicsJointLimitPolicy::ModelPositionOnly:
    return "model-position";
  case grouped_diagnostics::KinematicsJointLimitPolicy::
      ModelPositionAndVelocity:
    return "model-position+velocity";
  case grouped_diagnostics::KinematicsJointLimitPolicy::ExplicitRequirements:
    return "explicit-requirements";
  case grouped_diagnostics::KinematicsJointLimitPolicy::Unconstrained:
    return "unconstrained";
  }
  return "unknown";
}

const char *
terminationReasonName(grouped_diagnostics::IkTerminationReason value) {
  switch (value) {
  case grouped_diagnostics::IkTerminationReason::NotStarted:
    return "not-started";
  case grouped_diagnostics::IkTerminationReason::Converged:
    return "converged";
  case grouped_diagnostics::IkTerminationReason::NoEnabledConvergenceTasks:
    return "no-convergence-tasks";
  case grouped_diagnostics::IkTerminationReason::SingleIteration:
    return "single-iteration";
  case grouped_diagnostics::IkTerminationReason::Saturated:
    return "saturated";
  case grouped_diagnostics::IkTerminationReason::NoProgress:
    return "no-progress";
  case grouped_diagnostics::IkTerminationReason::IterationBudget:
    return "iteration-budget";
  case grouped_diagnostics::IkTerminationReason::SoftTimeBudget:
    return "soft-time-budget";
  case grouped_diagnostics::IkTerminationReason::HardConstraintViolation:
    return "hard-constraint-violation";
  case grouped_diagnostics::IkTerminationReason::InvalidNumericalSolution:
    return "invalid-numerical-solution";
  }
  return "unknown";
}

const char *qpBackendName(grouped_diagnostics::QpBackend value) {
  return value == grouped_diagnostics::QpBackend::ProxQp ? "proxqp"
                                                         : "eiquadprog";
}

const char *qpStatusName(grouped_diagnostics::QpSolveStatus value) {
  switch (value) {
  case grouped_diagnostics::QpSolveStatus::Optimal:
    return "optimal";
  case grouped_diagnostics::QpSolveStatus::PrimalInfeasible:
    return "primal-infeasible";
  case grouped_diagnostics::QpSolveStatus::DualInfeasible:
    return "dual-infeasible";
  case grouped_diagnostics::QpSolveStatus::MaximumIterations:
    return "maximum-iterations";
  case grouped_diagnostics::QpSolveStatus::NumericalFailure:
    return "numerical-failure";
  case grouped_diagnostics::QpSolveStatus::NotRun:
    return "not-run";
  }
  return "unknown";
}

void updateLocalSolverDebug(
    SolverDebug &output,
    const grouped_diagnostics::InverseKinematicsDiagnostics &diagnostics,
    grouped_diagnostics::ResultDisposition disposition) {
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
}

const char *
groupedRejectionReasonName(grouped_diagnostics::GroupedRejectionReason value) {
  switch (value) {
  case grouped_diagnostics::GroupedRejectionReason::None:
    return "none";
  case grouped_diagnostics::GroupedRejectionReason::SolverNotInitialized:
    return "solver-not-initialized";
  case grouped_diagnostics::GroupedRejectionReason::GroupUnavailable:
    return "group-unavailable";
  case grouped_diagnostics::GroupedRejectionReason::RunNotStarted:
    return "run-not-started";
  case grouped_diagnostics::GroupedRejectionReason::ConcurrentCall:
    return "concurrent-call";
  case grouped_diagnostics::GroupedRejectionReason::InvalidCapturedState:
    return "invalid-captured-state";
  case grouped_diagnostics::GroupedRejectionReason::InvalidTarget:
    return "invalid-target";
  case grouped_diagnostics::GroupedRejectionReason::SolverRejected:
    return "solver-rejected";
  case grouped_diagnostics::GroupedRejectionReason::RevisionOverflow:
    return "revision-overflow";
  }
  return "unknown";
}

const char *couplingStateName(grouped_diagnostics::GroupCouplingState value) {
  switch (value) {
  case grouped_diagnostics::GroupCouplingState::Unavailable:
    return "unavailable";
  case grouped_diagnostics::GroupCouplingState::WaitingForValue:
    return "waiting-for-value";
  case grouped_diagnostics::GroupCouplingState::Active:
    return "active";
  case grouped_diagnostics::GroupCouplingState::RejectedSource:
    return "rejected-source";
  }
  return "unknown";
}

} // namespace

void updateSolverDebug(
    SolverDebug &output,
    const motion_control::core::GroupedInverseKinematicsDiagnostics
        &diagnostics,
    motion_control::core::ResultDisposition disposition);

SolverDebug
makeSolverDebug(std::string label,
                const motion_control::core::GroupedInverseKinematicsDiagnostics
                    &diagnostics,
                motion_control::core::ResultDisposition disposition) {
  SolverDebug output;
  output.label = std::move(label);
  updateSolverDebug(output, diagnostics, disposition);
  return output;
}

void updateSolverDebug(
    SolverDebug &output,
    const motion_control::core::GroupedInverseKinematicsDiagnostics
        &diagnostics,
    motion_control::core::ResultDisposition disposition) {
  updateLocalSolverDebug(output, diagnostics.kinematics, disposition);
  if (!output.grouped_attempt.has_value()) {
    output.grouped_attempt.emplace();
  }
  auto &attempt = *output.grouped_attempt;
  attempt.rejection_reason =
      groupedRejectionReasonName(diagnostics.rejection_reason);
  attempt.run_generation = diagnostics.run_generation;
  attempt.attempt_revision = diagnostics.attempt_revision;
  attempt.value_revision = diagnostics.value_revision;
  attempt.attempt_accepted = diagnostics.attempt_accepted;
  attempt.has_accepted_value = diagnostics.has_accepted_value;
  attempt.coupling_state = couplingStateName(diagnostics.coupling_state);
  attempt.consumed_source_value_revision =
      diagnostics.consumed_source_value_revision;
  attempt.captured_state_sequence = diagnostics.captured_state_sequence;
  attempt.captured_state_time_nanoseconds =
      diagnostics.captured_state_time_nanoseconds;
}

} // namespace motion_control_lab

namespace {

namespace mcc = motion_control::core;
namespace mcl = motion_control_lab;
namespace replay = motion_control_lab::replay;

using mcl::grouped_servo_step::CartesianHandles;
using mcl::grouped_servo_step::LaunchOptions;
using mcl::grouped_servo_step::requireOk;
using mcl::grouped_servo_step::SolverHandles;
using mcl::grouped_servo_step::SourceMode;

using mcl::toEigen;
using mcl::toStdVector;

constexpr const char *kProgramId = "mcl_grouped_servo_step";
constexpr const char *kTitle = "Motion Control Grouped Dual-arm IK";
constexpr std::array<unsigned int, 1> kUiCpuAffinity{29};
constexpr std::array<unsigned int, 1> kRedCpuAffinity{31};
constexpr std::array<unsigned int, 1> kYellowCpuAffinity{30};
bool operationSucceeded(const mcc::Status &status) { return status.ok(); }

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
  mcl::SolverDebug solver_debug;
  std::string detail;
};

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

std::string traceEigenVector(const Eigen::VectorXd &values) {
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

mcc::RobotState robotState(const StateSnapshot &state) {
  mcc::RobotState result;
  result.joint_positions = state.positions;
  result.joint_velocities = state.velocities;
  return result;
}

mcc::CapturedRobotState capturedState(const StateSnapshot &state) {
  return mcc::CapturedRobotState{robotState(state), state.sequence,
                                 state.monotonic_time_nanoseconds};
}

void addCartesianTargets(const CartesianHandles &handles,
                         const TargetSnapshot &target,
                         mcc::GroupedInverseKinematicsRequest &request) {
  request.position_targets[0].position = target.left.translation();
  request.position_targets[1].position = target.right.translation();
  request.orientation_targets[0].orientation = target.left.linear();
  request.orientation_targets[1].orientation = target.right.linear();
  request.position_targets[0].handle = handles.left_position;
  request.position_targets[1].handle = handles.right_position;
  request.orientation_targets[0].handle = handles.left_orientation;
  request.orientation_targets[1].handle = handles.right_orientation;
}

void initializeCartesianRequest(const CartesianHandles &handles,
                                const mcc::FrameName &reference_frame_name,
                                mcc::GroupedInverseKinematicsRequest &request) {
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

std::string rejectedAttemptDetail(
    const mcc::Status &status,
    const mcc::GroupedInverseKinematicsDiagnostics &diagnostics) {
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
taskScaleSnapshot(const mcc::TaskScaleDiagnostic &diagnostic) {
  return TaskScaleSnapshot{diagnostic.active, diagnostic.scale,
                           diagnostic.degraded, diagnostic.stuck};
}

void fillRedDiagnostics(
    const CartesianHandles &handles,
    const mcc::GroupedInverseKinematicsDiagnostics &diagnostics,
    RedOutputSnapshot &output) {
  for (const auto &error : diagnostics.kinematics.position_errors) {
    if (error.handle.value == handles.left_position.value) {
      output.left_position_error_m = error.norm_m;
    } else if (error.handle.value == handles.right_position.value) {
      output.right_position_error_m = error.norm_m;
    }
  }
  for (const auto &error : diagnostics.kinematics.orientation_errors) {
    if (error.handle.value == handles.left_orientation.value) {
      output.left_orientation_error_rad = error.norm_rad;
    } else if (error.handle.value == handles.right_orientation.value) {
      output.right_orientation_error_rad = error.norm_rad;
    }
  }
  const auto &scales = diagnostics.kinematics.optimization.task_scales;
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
    const mcl::grouped_servo_step::SolverOptions &options,
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

int runLoopImpl(LaunchOptions launch, const mcl::R1RobotConfig &robot,
                mcc::GroupedKinematicsSolver &solver,
                const SolverHandles &handles,
                const std::vector<Eigen::Index> &active_joint_full_indices,
                std::string &normal_exit_detail) {
  const auto &options = launch.interactive;
  const auto affinity_domain = mcl::CpuAffinityDomain::capture();
  affinity_domain.validate(kProgramId, "ui", kUiCpuAffinity);
  affinity_domain.validate(kProgramId, "red", kRedCpuAffinity);
  affinity_domain.validate(kProgramId, "yellow", kYellowCpuAffinity);
  const auto ui_affinity_binding =
      affinity_domain.bindCurrentThread(kProgramId, "ui", kUiCpuAffinity);
  std::optional<replay::LoadedReplay> loaded_replay;
  std::optional<replay::ReplaySource> replay_source;
  if (launch.replay.has_value()) {
    if (!launch.replay->output_dir_explicit) {
      const std::string run_id = launch.replay->run_id.value_or(
          mcl::make_run_id(mcl::sha256_file(launch.replay->input_path)));
      const std::filesystem::path output_root =
          launch.replay->output_root.value_or(
              std::filesystem::path{"runs/mcl_grouped_servo_step"});
      launch.replay->output_dir = output_root / run_id;
    }
    loaded_replay = replay::loadReplay(*launch.replay);
    if (loaded_replay->timeline.timeline.empty()) {
      throw std::runtime_error("replay timeline is empty");
    }
    replay_source.emplace(*loaded_replay, launch.replay->execution_mode,
                          launch.replay->playback_rate);
    replay::createOutputDirectory(launch.replay->output_dir);
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
  requireOk(solver.computeForwardKinematics(mcc::SolverGroup::Red,
                                            initial_fk_request, initial_fk,
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
  initial_output.state = initial_state;
  initial_output.left_pose = warmup_target.left;
  initial_output.right_pose = warmup_target.right;
  mcc::SelfCollisionDiagnostics initial_collision_diagnostics;
  mcl::SelfCollisionDebug initial_collision_debug;
  mcl::SolverDebug initial_yellow_solver_debug;

  // Warm all numerical workspaces and the coupling path before deadlines apply.
  requireOk(solver.beginRun(1), "Failed to begin warm-up run");
  {
    mcc::GroupedInverseKinematicsSolution solution;
    mcc::GroupedInverseKinematicsDiagnostics diagnostics;
    mcc::GroupedInverseKinematicsRequest yellow;
    yellow.reference_frame_name = robot.base_frame;
    yellow.captured_state = capturedState(initial_state);
    auto status = solver.solveInverseKinematics(mcc::SolverGroup::Yellow,
                                                yellow, solution, diagnostics);
    if (!operationSucceeded(status) || !diagnostics.attempt_accepted) {
      throw std::runtime_error("Yellow warm-up failed: " +
                               rejectedAttemptDetail(status, diagnostics));
    }
    requireOk(
        solver.getSelfCollisionDiagnostics(handles.yellow_collision,
                                           initial_collision_diagnostics),
        "Failed to query Yellow self-collision diagnostics after warm-up");
    fillSelfCollisionDebug(initial_state, initial_collision_diagnostics,
                           options.solver, initial_collision_debug);
    initial_yellow_solver_debug = mcl::makeSolverDebug(
        "Yellow", diagnostics, solution.kinematics_solution.disposition);

    mcc::GroupedInverseKinematicsRequest red;
    initializeCartesianRequest(handles.red, robot.base_frame, red);
    red.captured_state = capturedState(initial_state);
    addCartesianTargets(handles.red, warmup_target, red);
    status = solver.solveInverseKinematics(mcc::SolverGroup::Red, red, solution,
                                           diagnostics);
    if (!operationSucceeded(status) || !diagnostics.attempt_accepted) {
      throw std::runtime_error("Red warm-up failed: " +
                               rejectedAttemptDetail(status, diagnostics));
    }
    initial_output.solve_time_ms = diagnostics.kinematics.solve_time_ms;
    initial_output.iterations = diagnostics.kinematics.iterations;
    initial_output.converged = diagnostics.kinematics.converged;
    fillRedDiagnostics(handles.red, diagnostics, initial_output);
    initial_output.solver_debug = mcl::makeSolverDebug(
        "Red", diagnostics, solution.kinematics_solution.disposition);
  }
  requireOk(solver.beginRun(2), "Failed to begin timed grouped run");

  RedAttemptSnapshot initial_red_attempt;
  initial_red_attempt.target = initial_target;
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
  const bool terminal_input_enabled =
      launch.source_mode == SourceMode::Teleop ||
      (launch.replay.has_value() && launch.replay->terminal_input_enabled);
  mcl::TerminalFrontend terminal({terminal_input_enabled, options.tui_enabled});
  mcl::KeyboardTargetSource input(terminal,
                                  launch.source_mode == SourceMode::Replay
                                      ? mcl::KeyboardSourceMode::Replay
                                      : mcl::KeyboardSourceMode::Teleop,
                                  options.tui,
                                  {{mcl::ArmSide::Left, initial_target.left},
                                   {mcl::ArmSide::Right, initial_target.right}},
                                  true);
  mcl::TuiRenderer tui(options.tui_enabled);
  if (launch.source_mode == SourceMode::Replay) {
    input.setMotionInputEnabled(false, "Replay motion editing is disabled");
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
                  "solve_time_ms,maximum_hard_violation,positions,velocities\n";

  visualization_sink->open();
  mcl::installRuntimeSignalHandlers();

  workers.yellow = std::thread([&]() {
    const auto affinity_binding = affinity_domain.bindCurrentThread(
        kProgramId, "yellow", kYellowCpuAffinity);
    yellow_affinity_to_ui.publish(affinity_binding);
    StateSnapshot state = initial_state;
    mcc::GroupedInverseKinematicsRequest request;
    request.reference_frame_name = robot.base_frame;
    mcc::GroupedInverseKinematicsSolution solution;
    mcc::GroupedInverseKinematicsDiagnostics diagnostics;
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
          const auto status = solver.solveInverseKinematics(
              mcc::SolverGroup::Yellow, request, solution, diagnostics);
          yellow_solve_time_percentiles.record(
              diagnostics.kinematics.solve_time_ms);
          const bool accepted =
              operationSucceeded(status) && diagnostics.attempt_accepted;
          if (accepted) {
            requireOk(
                solver.getSelfCollisionDiagnostics(handles.yellow_collision,
                                                   collision_diagnostics),
                "Failed to query accepted Yellow self-collision diagnostics");
            fillSelfCollisionDebug(state, collision_diagnostics, options.solver,
                                   collision_debug);
            collision_to_ui.publish(collision_debug);
          }
          mcl::updateSolverDebug(solver_debug, diagnostics,
                                 accepted ? mcc::ResultDisposition::Accepted
                                          : mcc::ResultDisposition::Rejected);
          yellow_solver_to_ui.publish(solver_debug);
          return mcl::WorkerIterationResult{
              accepted ? mcl::WorkerIterationOutcome::Accepted
                       : mcl::WorkerIterationOutcome::FatalRejected,
              diagnostics.attempt_revision,
              diagnostics.kinematics.solve_time_ms,
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
    mcc::GroupedInverseKinematicsRequest request;
    initializeCartesianRequest(handles.red, robot.base_frame, request);
    mcc::GroupedInverseKinematicsSolution solution;
    mcc::GroupedInverseKinematicsDiagnostics diagnostics;
    RedAttemptSnapshot attempt = initial_red_attempt;
    std::optional<std::uint64_t> rejected_target_revision;
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
          request.captured_state = capturedState(state);
          addCartesianTargets(handles.red, target, request);
          const auto status = solver.solveInverseKinematics(
              mcc::SolverGroup::Red, request, solution, diagnostics);
          red_solve_time_percentiles.record(
              diagnostics.kinematics.solve_time_ms);
          const bool accepted =
              operationSucceeded(status) && diagnostics.attempt_accepted;
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
            output.accepted_target = target;
            output.state = state;
            output.left_pose =
                requirePose(solution.kinematics_solution.solved_poses,
                            robot.left_end_effector_frame)
                    .pose;
            output.right_pose =
                requirePose(solution.kinematics_solution.solved_poses,
                            robot.right_end_effector_frame)
                    .pose;
            output.solve_time_ms = diagnostics.kinematics.solve_time_ms;
            output.iterations = diagnostics.kinematics.iterations;
            output.converged = diagnostics.kinematics.converged;
            fillRedDiagnostics(handles.red, diagnostics, output);
            mcl::updateSolverDebug(output.solver_debug, diagnostics,
                                   solution.kinematics_solution.disposition);
            output_to_ui.publish(output);

            attempt.state = RedAttemptState::Accepted;
            attempt.target = target;
            attempt.solver_debug = output.solver_debug;
            attempt.detail.clear();
            red_attempt_to_ui.publish(attempt);
          } else {
            attempt.state = launch.source_mode == SourceMode::Teleop &&
                                    status.code == mcc::StatusCode::Infeasible
                                ? RedAttemptState::RecoverableRejected
                                : RedAttemptState::FatalRejected;
            attempt.target = target;
            mcl::updateSolverDebug(attempt.solver_debug, diagnostics,
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
                << diagnostics.kinematics.solve_time_ms << ','
                << diagnostics.kinematics.optimization.maximum_hard_violation
                << ',' << traceEigenVector(state.positions) << ','
                << traceEigenVector(state.velocities) << '\n';
          }
          return mcl::WorkerIterationResult{
              accepted ? mcl::WorkerIterationOutcome::Accepted
              : launch.source_mode == SourceMode::Teleop &&
                      status.code == mcc::StatusCode::Infeasible
                  ? mcl::WorkerIterationOutcome::RecoverableRejected
                  : mcl::WorkerIterationOutcome::FatalRejected,
              diagnostics.attempt_revision,
              diagnostics.kinematics.solve_time_ms,
              accepted ? std::string{}
                       : rejectedAttemptDetail(status, diagnostics)};
        });
  });

  mcl::SingleRateScheduler ui_scheduler(
      {options.ui_rate_hz, options.duration_s});
  TargetSnapshot published_target = initial_target;
  TargetSnapshot last_command_target = initial_target;
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
  std::optional<mcl::IkRuntimeState> last_visualized_runtime_state;
  std::uint64_t last_visualized_rejected_target_revision = 0;
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
        input.setStatus("Red accepted the new target; grouped IK resumed");
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

    if (launch.source_mode == SourceMode::Replay && !held_fault.has_value()) {
      for (const auto control : input.consumeSourceControls()) {
        replay_source->applyControl(control);
      }
      const bool worker_consumed_current =
          replay_last_consumed_revision.load() >= published_target.revision;
      const bool may_advance =
          launch.replay->execution_mode == mcl::data::ExecutionMode::Realtime ||
          worker_consumed_current;
      const auto advance =
          may_advance ? replay_source->advance(static_cast<std::int64_t>(
                            std::llround(1.0e9 / options.ui_rate_hz)))
                      : replay::ReplayAdvance{};
      replay_source->waitForCurrentFrame();
      input.setPaused(replay_source->paused(), replay_source->status().detail);
      if (advance.frame_changed) {
        const auto &source = replay_source->sourceFrame();
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
      if (launch.source_mode == SourceMode::Teleop && !held_fault.has_value() &&
          !input.paused() &&
          !sameTargetPoses(last_command_target, input.targets())) {
        published_target =
            targetSnapshot(input.targets(), published_target.revision + 1);
        last_command_target = published_target;
        target_to_red.publish(published_target);
      }

      frame.targets = input.targets();
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
        frame.status = "Grouped IK running | skipped_releases R=" +
                       std::to_string(red_stats.skipped_release_count) + " Y=" +
                       std::to_string(yellow_stats.skipped_release_count) +
                       " recoverable_rejections R=" +
                       std::to_string(red_stats.recoverable_rejection_count);
      }
      frame.iterations = latest_red_attempt.solver_debug.ik_iterations;
      frame.converged = latest_red_attempt.solver_debug.converged;
      frame.solve_time_ms = latest_red_attempt.solver_debug.ik_solve_time_ms;
      frame.target_errors = {
          {mcl::ArmSide::Left, latest_output.left_position_error_m,
           latest_output.left_orientation_error_rad},
          {mcl::ArmSide::Right, latest_output.right_position_error_m,
           latest_output.right_orientation_error_rad}};
      frame.self_collisions = {latest_collision_debug};
      frame.rejected_target = rejected_target;
      frame.paused = input.paused();
      frame.selected_side = input.selectedSide();

      const std::uint64_t rejected_revision =
          rejected_target.has_value() ? rejected_target->revision : 0;
      const bool visualization_state_changed =
          !last_visualized_runtime_state.has_value() ||
          frame.runtime_state != *last_visualized_runtime_state ||
          rejected_revision != last_visualized_rejected_target_revision;
      if (frame.runtime_state == mcl::IkRuntimeState::Running ||
          visualization_state_changed) {
        visualization_sink->write(mcl::makeIkRenderBatch(
            frame, presentation, schedule->emit_time_ns));
        last_visualized_runtime_state = frame.runtime_state;
        last_visualized_rejected_target_revision = rejected_revision;
        ++publish_count;
      }
      tui.render(mcl::makeStandardIkTuiDocument(
          frame, presentation, publish_count, visualization_sink->status(),
          kTitle, input.status()));

      if (launch.source_mode == SourceMode::Replay &&
          latest_output.accepted_target.revision == published_target.revision) {
        replay_source->markFrameProcessed();
        if (replay_source->endOfStream()) {
          replay_completed = true;
          break;
        }
      }
    }
    if (launch.replay.has_value() &&
        launch.replay->execution_mode == mcl::data::ExecutionMode::Batch) {
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
  if (launch.replay.has_value()) {
    const auto trace_path = launch.replay->output_dir / "trace.csv";
    {
      std::lock_guard<std::mutex> lock(replay_trace_mutex);
      replay::writeTextFile(trace_path, replay_trace.str());
    }
    replay::ReplayExecutionMetadata execution;
    execution.app = kProgramId;
    execution.topology = "red-yellow-grouped-servo-step";
    execution.solver = "motion_control_core";
    execution.backend = "proxqp";
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
        {"yellow_to_red_coupling_weight",
         std::to_string(options.solver.yellow_to_red_coupling_weight)},
        {"minimum_collision_distance_m",
         std::to_string(options.solver.minimum_collision_distance_m)},
        {"collision_influence_distance_m",
         std::to_string(options.solver.collision_influence_distance_m)},
        {"collision_weight", std::to_string(options.solver.collision_weight)},
    };
    const auto manifest =
        replay::makeReplayManifest(*launch.replay, *loaded_replay, execution,
                                   mcl::sha256_file(trace_path));
    replay::writeTextFile(launch.replay->output_dir / "manifest.json",
                          jsonText(manifest));
    const auto status = replay::makeReplayStatus(
        *loaded_replay, execution,
        recorded_fault.has_value()
            ? "failed"
            : (replay_completed ? "succeeded" : "stopped"),
        recorded_fault.has_value() ? faultSummary(*recorded_fault)
                                   : std::string{});
    replay::writeTextFile(launch.replay->output_dir / "status.json",
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

} // namespace

namespace motion_control_lab::grouped_servo_step {

int runLoop(LaunchOptions launch, const R1RobotConfig &robot,
            mcc::GroupedKinematicsSolver &solver, const SolverHandles &handles,
            const std::vector<Eigen::Index> &active_joint_full_indices,
            std::string &normal_exit_detail) {
  return runLoopImpl(std::move(launch), robot, solver, handles,
                     active_joint_full_indices, normal_exit_detail);
}

} // namespace motion_control_lab::grouped_servo_step
