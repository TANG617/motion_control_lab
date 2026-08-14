#include "apps/dual_arm_replay_ik/replay_ik_engine.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <memory>
#include <motion_control_core/motion_control_core.hpp>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace motion_control_lab::replay
{
namespace
{
namespace mcc = motion_control::core;

void requireOk(const mcc::Status & status, const std::string &)
{
  if (!status.ok()) {
    throw std::runtime_error(status.message);
  }
}

Eigen::VectorXd toEigen(const std::vector<double> & values)
{
  return Eigen::Map<const Eigen::VectorXd>(values.data(), static_cast<Eigen::Index>(values.size()));
}

std::vector<double> toVector(const Eigen::VectorXd & values)
{
  return {values.data(), values.data() + values.size()};
}

std::vector<double> positionsByJointName(
  const data::StampedJointState & joint_state, const std::vector<std::string> & expected_names)
{
  std::unordered_map<std::string, double> positions;
  positions.reserve(joint_state.names.size());
  for (std::size_t index = 0; index < joint_state.names.size(); ++index) {
    positions.emplace(joint_state.names[index], joint_state.positions.at(index));
  }

  std::vector<double> result;
  result.reserve(expected_names.size());
  for (const auto & name : expected_names) {
    result.push_back(positions.at(name));
  }
  return result;
}

mcc::RobotState makeState(
  const std::vector<double> & positions, const std::vector<double> & velocities)
{
  mcc::RobotState result;
  result.joint_positions = toEigen(positions);
  result.joint_velocities = toEigen(velocities);
  return result;
}

std::int64_t monotonicNanoseconds(data::ReplayClock::TimePoint value)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(value.time_since_epoch()).count();
}

std::string joinPositions(const std::vector<double> & positions)
{
  std::ostringstream output;
  output << std::setprecision(17);
  for (std::size_t index = 0; index < positions.size(); ++index) {
    if (index > 0) {
      output << ';';
    }
    output << positions[index];
  }
  return output.str();
}

struct EndEffectorFk
{
  Eigen::Isometry3d left{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d right{Eigen::Isometry3d::Identity()};
};

EndEffectorFk computeEndEffectorFk(
  mcc::GroupedKinematicsSolver & solver, const R1RobotConfig & robot,
  const std::vector<double> & positions, const std::vector<double> & velocities)
{
  mcc::ForwardKinematicsRequest request;
  request.state = makeState(positions, velocities);
  request.frame_names = {robot.left_end_effector_frame, robot.right_end_effector_frame};
  request.reference_frame_name = robot.base_frame;
  mcc::ForwardKinematicsSolution solution;
  mcc::ForwardKinematicsDiagnostics diagnostics;
  requireOk(
    solver.computeForwardKinematics(mcc::SolverGroup::Red, request, solution, diagnostics),
    "visualization FK failed");
  const auto findPose = [&](const std::string & frame_name) -> const mcc::FramePose & {
    const auto found = std::find_if(
      solution.poses.begin(), solution.poses.end(),
      [&](const mcc::FramePose & pose) { return pose.frame_name == frame_name; });
    return *found;
  };
  return {
    findPose(robot.left_end_effector_frame).pose,
    findPose(robot.right_end_effector_frame).pose};
}

ReplayIkVisualizationSample makeVisualizationSample(
  std::uint64_t sequence, std::int64_t sample_time_ns, const data::DualArmFrame & frame,
  const R1RobotConfig & robot, const EndEffectorFk & fk, const std::vector<double> & positions,
  const std::vector<double> & velocities, std::string status, bool paused, bool accepted,
  double solve_time_ms)
{
  ReplayIkVisualizationSample result;
  result.sequence = sequence;
  result.sample_time_ns = sample_time_ns;
  result.left_target_frame_id = frame.left.frame_id;
  result.left_input_target = frame.left.pose;
  result.right_target_frame_id = frame.right.frame_id;
  result.right_input_target = frame.right.pose;
  result.forward_kinematics_frame_id = robot.base_frame;
  result.left_end_effector_fk = fk.left;
  result.right_end_effector_fk = fk.right;
  result.joint_names = robot.joint_names;
  result.positions = positions;
  result.velocities = velocities;
  result.status = std::move(status);
  result.paused = paused;
  result.solve_accepted = accepted;
  result.solve_time_ms = solve_time_ms;
  return result;
}

std::string statusCodeString(mcc::StatusCode code)
{
  switch (code) {
    case mcc::StatusCode::Ok:
      return "ok";
    case mcc::StatusCode::InvalidInput:
      return "invalid_input";
    case mcc::StatusCode::InvalidState:
      return "invalid_state";
    case mcc::StatusCode::InvalidTarget:
      return "invalid_target";
    case mcc::StatusCode::NotInitialized:
      return "not_initialized";
    case mcc::StatusCode::Infeasible:
      return "infeasible";
    case mcc::StatusCode::SolverError:
      return "solver_error";
  }
  return "unknown";
}

}  // namespace

bool ReplayIkCaseResult::completed() const
{
  return frames_attempted == frames_planned && rejected_solves == 0;
}

std::string replayIkTraceHeader()
{
  return "sequence,original_logical_timestamp_ns,source_time_from_start_ns,"
         "projected_timestamp_ns,scheduled_monotonic_time_ns,actual_solve_start_ns,"
         "actual_solve_end_ns,lateness_ns,deadline_missed,"
         "left_header_stamp_ns,left_log_time_ns,left_publish_time_ns,"
         "right_header_stamp_ns,right_log_time_ns,right_publish_time_ns,"
         "solve_accepted,solve_status,solve_time_ms,joint_positions\n";
}

ReplayIkCaseResult executeReplayIkCase(
  const ReplayOptions & options, const ReplayIkExecutionConfig & execution_config)
{
  ReplayIkCaseResult result;
  result.next_visualization_sequence = execution_config.first_visualization_sequence;
  result.loaded = loadReplay(options);
  const auto & first_frame = result.loaded.timeline.timeline.at(0).value;
  result.frames_planned = result.loaded.timeline.timeline.size();

  const auto & robot = r1RobotConfig();
  mcc::RobotModelDescription model_description;
  model_description.urdf_path = options.urdf_path.string();
  model_description.kinematics_reference_frame = robot.base_frame;
  model_description.joint_names = robot.joint_names;
  std::shared_ptr<const mcc::RobotModel> model;
  requireOk(mcc::RobotModel::load(model_description, model), "failed to load robot model");

  mcc::KinematicsSolverConfig solver_config;
  solver_config.mode = mcc::IkSolveMode::ServoStep;
  solver_config.servo_period = static_cast<double>(options.servo_period_ns) / 1.0e9;
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
  requireOk(builder.configure(model, robot.joint_names, grouped_config), "failed to configure IK");

  mcc::PositionTaskConfig left_position_config;
  left_position_config.name = "replay-left-position";
  left_position_config.enforcement = mcc::HardEnforcement{};
  mcc::GroupedPositionTaskHandle left_position;
  requireOk(
    builder.addPositionTask(
      mcc::SolverGroup::Red, robot.left_end_effector_frame, left_position_config, left_position),
    "failed to add left position task");

  mcc::OrientationTaskConfig left_orientation_config;
  left_orientation_config.name = "replay-left-orientation";
  left_orientation_config.enforcement = mcc::HardEnforcement{};
  mcc::GroupedOrientationTaskHandle left_orientation;
  requireOk(
    builder.addOrientationTask(
      mcc::SolverGroup::Red, robot.left_end_effector_frame, left_orientation_config, left_orientation),
    "failed to add left orientation task");

  mcc::PositionTaskConfig right_position_config;
  right_position_config.name = "replay-right-position";
  right_position_config.enforcement = mcc::HardEnforcement{};
  mcc::GroupedPositionTaskHandle right_position;
  requireOk(
    builder.addPositionTask(
      mcc::SolverGroup::Red, robot.right_end_effector_frame, right_position_config, right_position),
    "failed to add right position task");

  mcc::OrientationTaskConfig right_orientation_config;
  right_orientation_config.name = "replay-right-orientation";
  right_orientation_config.enforcement = mcc::HardEnforcement{};
  mcc::GroupedOrientationTaskHandle right_orientation;
  requireOk(
    builder.addOrientationTask(
      mcc::SolverGroup::Red, robot.right_end_effector_frame, right_orientation_config, right_orientation),
    "failed to add right orientation task");

  mcc::JointPositionLimitConfig position_limit_config;
  position_limit_config.margin = 1.0e-3;
  position_limit_config.enforcement = mcc::HardEnforcement{};
  mcc::GroupedJointPositionLimitHandle position_limits;
  requireOk(
    builder.addJointPositionLimits(mcc::SolverGroup::Red, position_limit_config, position_limits),
    "failed to add position limits");

  mcc::JointVelocityLimitConfig velocity_limit_config;
  velocity_limit_config.enforcement = mcc::HardEnforcement{};
  mcc::GroupedJointVelocityLimitHandle velocity_limits;
  requireOk(
    builder.addJointVelocityLimits(mcc::SolverGroup::Red, velocity_limit_config, velocity_limits),
    "failed to add velocity limits");

  mcc::GroupedKinematicsSolver solver;
  requireOk(builder.finalize(solver), "failed to finalize IK");
  requireOk(solver.beginRun(1), "failed to begin IK run");

  result.initial_positions =
    result.loaded.initial_joint_state.has_value()
      ? positionsByJointName(*result.loaded.initial_joint_state, robot.joint_names)
      : robot.default_positions;
  std::vector<double> positions = result.initial_positions;
  std::vector<double> velocities(positions.size(), 0.0);
  const auto fixed_positions = positions;
  const auto fixed_velocities = velocities;
  data::ReplayClock clock(options.execution_mode, options.playback_rate);

  if (execution_config.before_replay) {
    execution_config.before_replay();
  }

  if (execution_config.initial_frame_gate || execution_config.visualization_callback) {
    const auto initial_fk =
      computeEndEffectorFk(solver, robot, result.initial_positions, fixed_velocities);
    auto sample = makeVisualizationSample(
      result.next_visualization_sequence++, 0, first_frame, robot, initial_fk,
      result.initial_positions, fixed_velocities,
      execution_config.initial_frame_gate ? "waiting_for_space" : "initialized",
      static_cast<bool>(execution_config.initial_frame_gate), false, 0.0);
    if (execution_config.initial_frame_gate) {
      execution_config.initial_frame_gate(sample);
    } else {
      execution_config.visualization_callback(sample);
    }
  }

  const auto run_start = clock.now();
  std::ostringstream trace;
  trace << replayIkTraceHeader();
  trace << std::setprecision(17);

  for (const auto & frame : result.loaded.timeline.timeline) {
    const auto scheduled = clock.deadline(run_start, frame.projected_time_ns);
    clock.waitUntil(scheduled);
    const auto solve_start = clock.now();
    const auto lateness =
      options.execution_mode == data::ExecutionMode::Realtime
        ? std::max<std::int64_t>(
            0,
            std::chrono::duration_cast<std::chrono::nanoseconds>(solve_start - scheduled).count())
        : 0;
    const bool deadline_missed =
      options.execution_mode == data::ExecutionMode::Realtime && lateness > 0;
    result.deadline_misses += deadline_missed ? 1U : 0U;

    const auto & state_positions =
      options.state_policy == StatePolicy::PreviousSolution ? positions : fixed_positions;
    const auto & state_velocities =
      options.state_policy == StatePolicy::PreviousSolution ? velocities : fixed_velocities;
    mcc::GroupedInverseKinematicsRequest request;
    request.reference_frame_name = frame.value.left.frame_id;
    request.captured_state.state = makeState(state_positions, state_velocities);
    request.captured_state.sequence = frame.sequence + 1;
    request.captured_state.monotonic_time_nanoseconds =
      std::max<std::int64_t>(1, monotonicNanoseconds(scheduled));
    const Eigen::Isometry3d left_end_effector_target =
      frame.value.left.pose * robot.left_tcp_offset.inverse();
    const Eigen::Isometry3d right_end_effector_target =
      frame.value.right.pose * robot.right_tcp_offset.inverse();
    request.position_targets.push_back(
      {left_position, left_end_effector_target.translation(), true});
    request.orientation_targets.push_back(
      {left_orientation, left_end_effector_target.linear(), true});
    request.position_targets.push_back(
      {right_position, right_end_effector_target.translation(), true});
    request.orientation_targets.push_back(
      {right_orientation, right_end_effector_target.linear(), true});

    mcc::GroupedInverseKinematicsSolution solution;
    mcc::GroupedInverseKinematicsDiagnostics diagnostics;
    const auto status =
      solver.solveInverseKinematics(mcc::SolverGroup::Red, request, solution, diagnostics);
    const auto solve_end = clock.now();
    const bool accepted = status.ok() && diagnostics.attempt_accepted;
    std::vector<double> output_positions = state_positions;
    std::vector<double> output_velocities = state_velocities;
    if (accepted) {
      output_positions = toVector(solution.kinematics_solution.joint_positions);
      output_velocities = toVector(solution.kinematics_solution.joint_velocities);
      ++result.accepted_solves;
      if (options.state_policy == StatePolicy::PreviousSolution) {
        positions = output_positions;
        velocities = output_velocities;
      }
    } else {
      ++result.rejected_solves;
      if (!result.first_failure_frame.has_value()) {
        result.first_failure_frame = frame.sequence;
        result.first_failure_code =
          status.ok() ? "attempt_rejected" : statusCodeString(status.code);
        result.first_failure_message = status.ok() ? "solver attempt was rejected" : status.message;
      }
    }
    ++result.frames_attempted;

    const std::string status_text =
      status.ok() ? (accepted ? "ok" : "attempt_rejected") : status.message;
    if (execution_config.visualization_callback) {
      const auto output_fk =
        computeEndEffectorFk(solver, robot, output_positions, output_velocities);
      execution_config.visualization_callback(makeVisualizationSample(
        result.next_visualization_sequence++, frame.projected_time_ns, frame.value, robot,
        output_fk, output_positions, output_velocities, status_text, false, accepted,
        diagnostics.kinematics.solve_time_ms));
    }

    trace << frame.sequence << ',' << frame.original_logical_time_ns << ','
          << frame.source_time_from_start_ns << ',' << frame.projected_time_ns << ','
          << monotonicNanoseconds(scheduled) << ',' << monotonicNanoseconds(solve_start) << ','
          << monotonicNanoseconds(solve_end) << ',' << lateness << ','
          << (deadline_missed ? "true" : "false") << ','
          << optionalTimestamp(frame.value.left.time.header_stamp_ns) << ','
          << optionalTimestamp(frame.value.left.time.log_time_ns) << ','
          << optionalTimestamp(frame.value.left.time.publish_time_ns) << ','
          << optionalTimestamp(frame.value.right.time.header_stamp_ns) << ','
          << optionalTimestamp(frame.value.right.time.log_time_ns) << ','
          << optionalTimestamp(frame.value.right.time.publish_time_ns) << ','
          << (accepted ? "true" : "false") << ',' << csvEscape(status_text) << ','
          << diagnostics.kinematics.solve_time_ms << ','
          << csvEscape(joinPositions(output_positions)) << '\n';

    if (!accepted && execution_config.stop_on_first_error) {
      break;
    }
  }

  result.trace_csv = trace.str();
  return result;
}

}  // namespace motion_control_lab::replay
