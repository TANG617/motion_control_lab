#include "apps/dual_arm_replay_ik/replay_support.hpp"

#include "motion_control_lab/sha256.hpp"

#include <motion_control_core/motion_control_core.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
namespace mcc = motion_control::core;
namespace mcl = motion_control_lab;
namespace replay = motion_control_lab::replay;

struct R1ReplayConfig
{
  std::string base_frame{"base_link"};
  std::string left_end_effector{"left_arm_ee_link"};
  std::string right_end_effector{"right_arm_ee_link"};
  std::vector<std::string> joint_names{
    "head_yaw_joint", "head_pitch_joint", "torso_yaw_joint", "torso_pitch_joint",
    "knee_pitch_joint", "ankle_pitch_joint",
    "left_arm_joint1", "left_arm_joint2", "left_arm_joint3", "left_arm_joint4",
    "left_arm_joint5", "left_arm_joint6", "left_arm_joint7",
    "right_arm_joint1", "right_arm_joint2", "right_arm_joint3", "right_arm_joint4",
    "right_arm_joint5", "right_arm_joint6", "right_arm_joint7"};
  std::vector<double> initial_positions{
    0.0, 0.31, 0.0, 0.5, 0.5, -0.5,
    0.9, -1.38, -1.57, -1.4, -0.45, 0.0, 0.0,
    -0.9, 1.38, 1.57, 1.4, 0.45, 0.0, 0.0};
};

void requireOk(const mcc::Status & status, const std::string & operation)
{
  if (!status.ok()) {
    throw std::runtime_error(operation + ": " + status.message);
  }
}

Eigen::VectorXd toEigen(const std::vector<double> & values)
{
  return Eigen::Map<const Eigen::VectorXd>(
    values.data(), static_cast<Eigen::Index>(values.size()));
}

std::vector<double> toVector(const Eigen::VectorXd & values)
{
  return {values.data(), values.data() + values.size()};
}

mcc::RobotState makeState(
  const std::vector<double> & positions,
  const std::vector<double> & velocities)
{
  mcc::RobotState result;
  result.joint_positions = toEigen(positions);
  result.joint_velocities = toEigen(velocities);
  return result;
}

std::int64_t monotonicNanoseconds(mcl::data::ReplayClock::TimePoint value)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    value.time_since_epoch()).count();
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

std::string jsonString(const Json::Value & value)
{
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "  ";
  return Json::writeString(builder, value) + "\n";
}

int run(int argc, char ** argv)
{
  const auto options = replay::parseReplayOptions(argc, argv, true);
  if (options.help) {
    std::cout << replay::replayHelp(argv[0], true);
    return EXIT_SUCCESS;
  }

  // Input I/O, chunk decompression, CDR/CSV decoding, validation, pairing and
  // timestamp projection all complete before the solve clock starts.
  const auto loaded = replay::loadReplay(options);
  if (loaded.timeline.timeline.empty()) {
    throw std::runtime_error("replay timeline contains no paired frames");
  }

  const R1ReplayConfig robot;
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
      mcc::SolverGroup::Red, robot.left_end_effector,
      left_position_config, left_position),
    "failed to add left position task");

  mcc::OrientationTaskConfig left_orientation_config;
  left_orientation_config.name = "replay-left-orientation";
  left_orientation_config.enforcement = mcc::HardEnforcement{};
  mcc::GroupedOrientationTaskHandle left_orientation;
  requireOk(
    builder.addOrientationTask(
      mcc::SolverGroup::Red, robot.left_end_effector,
      left_orientation_config, left_orientation),
    "failed to add left orientation task");

  mcc::PositionTaskConfig right_position_config;
  right_position_config.name = "replay-right-position";
  right_position_config.enforcement = mcc::HardEnforcement{};
  mcc::GroupedPositionTaskHandle right_position;
  requireOk(
    builder.addPositionTask(
      mcc::SolverGroup::Red, robot.right_end_effector,
      right_position_config, right_position),
    "failed to add right position task");

  mcc::OrientationTaskConfig right_orientation_config;
  right_orientation_config.name = "replay-right-orientation";
  right_orientation_config.enforcement = mcc::HardEnforcement{};
  mcc::GroupedOrientationTaskHandle right_orientation;
  requireOk(
    builder.addOrientationTask(
      mcc::SolverGroup::Red, robot.right_end_effector,
      right_orientation_config, right_orientation),
    "failed to add right orientation task");

  mcc::JointPositionLimitConfig position_limit_config;
  position_limit_config.margin = 1.0e-3;
  position_limit_config.enforcement = mcc::HardEnforcement{};
  mcc::GroupedJointPositionLimitHandle position_limits;
  requireOk(
    builder.addJointPositionLimits(
      mcc::SolverGroup::Red, position_limit_config, position_limits),
    "failed to add position limits");

  mcc::JointVelocityLimitConfig velocity_limit_config;
  velocity_limit_config.enforcement = mcc::HardEnforcement{};
  mcc::GroupedJointVelocityLimitHandle velocity_limits;
  requireOk(
    builder.addJointVelocityLimits(
      mcc::SolverGroup::Red, velocity_limit_config, velocity_limits),
    "failed to add velocity limits");

  mcc::GroupedKinematicsSolver solver;
  requireOk(builder.finalize(solver), "failed to finalize IK");
  requireOk(solver.beginRun(1), "failed to begin IK run");

  replay::createOutputDirectory(options.output_dir);
  std::vector<double> positions = robot.initial_positions;
  std::vector<double> velocities(positions.size(), 0.0);
  const auto fixed_positions = positions;
  const auto fixed_velocities = velocities;
  mcl::data::ReplayClock clock(options.execution_mode, options.playback_rate);
  const auto run_start = clock.now();
  std::size_t deadline_misses = 0;
  std::size_t accepted_count = 0;

  std::ostringstream trace;
  trace << "sequence,original_logical_timestamp_ns,source_time_from_start_ns,"
           "projected_timestamp_ns,scheduled_monotonic_time_ns,actual_solve_start_ns,"
           "actual_solve_end_ns,lateness_ns,deadline_missed,"
           "left_header_stamp_ns,left_log_time_ns,left_publish_time_ns,"
           "right_header_stamp_ns,right_log_time_ns,right_publish_time_ns,"
           "solve_accepted,solve_status,solve_time_ms,joint_positions\n";
  trace << std::setprecision(17);

  for (const auto & frame : loaded.timeline.timeline) {
    const auto scheduled = clock.deadline(run_start, frame.projected_time_ns);
    clock.waitUntil(scheduled);
    const auto solve_start = clock.now();
    const auto lateness = options.execution_mode == mcl::data::ExecutionMode::Realtime
      ? std::max<std::int64_t>(
          0, std::chrono::duration_cast<std::chrono::nanoseconds>(
            solve_start - scheduled).count())
      : 0;
    const bool deadline_missed =
      options.execution_mode == mcl::data::ExecutionMode::Realtime && lateness > 0;
    deadline_misses += deadline_missed ? 1U : 0U;

    const auto & state_positions = options.state_policy == replay::StatePolicy::PreviousSolution
      ? positions : fixed_positions;
    const auto & state_velocities = options.state_policy == replay::StatePolicy::PreviousSolution
      ? velocities : fixed_velocities;
    mcc::GroupedInverseKinematicsRequest request;
    request.reference_frame_name = frame.value.left.frame_id;
    request.captured_state.state = makeState(state_positions, state_velocities);
    request.captured_state.sequence = frame.sequence + 1;
    request.captured_state.monotonic_time_nanoseconds =
      std::max<std::int64_t>(1, monotonicNanoseconds(scheduled));
    request.position_targets.push_back(
      {left_position, frame.value.left.pose.translation(), true});
    request.orientation_targets.push_back(
      {left_orientation, frame.value.left.pose.linear(), true});
    request.position_targets.push_back(
      {right_position, frame.value.right.pose.translation(), true});
    request.orientation_targets.push_back(
      {right_orientation, frame.value.right.pose.linear(), true});

    mcc::GroupedInverseKinematicsSolution solution;
    mcc::GroupedInverseKinematicsDiagnostics diagnostics;
    const auto status = solver.solveInverseKinematics(
      mcc::SolverGroup::Red, request, solution, diagnostics);
    const auto solve_end = clock.now();
    const bool accepted = status.ok() && diagnostics.attempt_accepted &&
      solution.kinematics_solution.joint_positions.size() ==
      static_cast<Eigen::Index>(robot.joint_names.size());
    std::vector<double> output_positions = state_positions;
    if (accepted) {
      output_positions = toVector(solution.kinematics_solution.joint_positions);
      ++accepted_count;
      if (options.state_policy == replay::StatePolicy::PreviousSolution) {
        positions = output_positions;
        if (solution.kinematics_solution.joint_velocities.size() ==
            static_cast<Eigen::Index>(robot.joint_names.size())) {
          velocities = toVector(solution.kinematics_solution.joint_velocities);
        } else {
          velocities.assign(positions.size(), 0.0);
        }
      }
    }

    const std::string status_text = status.ok()
      ? (accepted ? "ok" : "attempt_rejected")
      : status.message;
    trace
      << frame.sequence << ','
      << frame.original_logical_time_ns << ','
      << frame.source_time_from_start_ns << ','
      << frame.projected_time_ns << ','
      << monotonicNanoseconds(scheduled) << ','
      << monotonicNanoseconds(solve_start) << ','
      << monotonicNanoseconds(solve_end) << ','
      << lateness << ','
      << (deadline_missed ? "true" : "false") << ','
      << replay::optionalTimestamp(frame.value.left.time.header_stamp_ns) << ','
      << replay::optionalTimestamp(frame.value.left.time.log_time_ns) << ','
      << replay::optionalTimestamp(frame.value.left.time.publish_time_ns) << ','
      << replay::optionalTimestamp(frame.value.right.time.header_stamp_ns) << ','
      << replay::optionalTimestamp(frame.value.right.time.log_time_ns) << ','
      << replay::optionalTimestamp(frame.value.right.time.publish_time_ns) << ','
      << (accepted ? "true" : "false") << ','
      << replay::csvEscape(status_text) << ','
      << diagnostics.kinematics.solve_time_ms << ','
      << replay::csvEscape(joinPositions(output_positions)) << '\n';
  }

  const auto trace_path = options.output_dir / "trace.csv";
  replay::writeTextFile(trace_path, trace.str());
  auto manifest = replay::makeReplayManifest(
    options, loaded, deadline_misses, accepted_count, mcl::sha256_file(trace_path));
  manifest["robot_model"]["urdf_path"] =
    std::filesystem::absolute(options.urdf_path).lexically_normal().string();
  manifest["robot_model"]["urdf_sha256"] = mcl::sha256_file(options.urdf_path);
  manifest["solver"]["profile"] = "RedOnly";
  manifest["solver"]["servo_mode"] = "ServoStep";
  manifest["initial_state"]["source"] = "r1_replay_config.v1";
  for (const auto & name : robot.joint_names) {
    manifest["initial_state"]["joint_names"].append(name);
  }
  for (const double position : robot.initial_positions) {
    manifest["initial_state"]["joint_positions"].append(position);
  }
  replay::writeTextFile(options.output_dir / "manifest.json", jsonString(manifest));

  std::cout
    << "frames=" << loaded.timeline.timeline.size()
    << " accepted=" << accepted_count
    << " deadline_misses=" << deadline_misses
    << " output=" << options.output_dir.string() << '\n';
  return accepted_count == loaded.timeline.timeline.size() ? EXIT_SUCCESS : EXIT_FAILURE;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    return run(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << "mcl_dual_arm_replay_ik: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
