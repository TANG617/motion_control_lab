#include "apps/dual_arm_replay_ik/replay_support.hpp"

#include "contracts/visualization/foxglove_ik_v1.hpp"
#include "e02_build_config.hpp"
#include "motion_control_lab/run_artifacts.hpp"
#include "motion_control_lab/sha256.hpp"

#include <motion_control_core/motion_control_core.hpp>

#if MCL_WITH_REPLAY_VISUALIZATION
#include <motion_control_viz/foxglove_frame_sink.hpp>
#include <motion_control_viz/frame.hpp>
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unordered_map>
#include <unistd.h>
#include <utility>
#include <vector>

namespace
{
namespace mcc = motion_control::core;
namespace mcl = motion_control_lab;
namespace replay = motion_control_lab::replay;
namespace visualization_contract =
  motion_control_lab::contracts::foxglove_ik_v1;
#if MCL_WITH_REPLAY_VISUALIZATION
namespace mcv = motion_control::viz;
#endif

Eigen::Isometry3d makeR1TcpOffset()
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d{0.0, 0.0, 0.1};
  return result;
}

struct R1ReplayConfig
{
  std::string base_frame{"base_link"};
  std::string left_end_effector{"left_arm_ee_link"};
  std::string right_end_effector{"right_arm_ee_link"};
  Eigen::Isometry3d left_tcp_offset{makeR1TcpOffset()};
  Eigen::Isometry3d right_tcp_offset{makeR1TcpOffset()};
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

std::vector<double> positionsByJointName(
  const mcl::data::StampedJointState & joint_state,
  const std::vector<std::string> & expected_names)
{
  if (joint_state.names.size() != joint_state.positions.size()) {
    throw std::runtime_error(
            "initial JointState names and positions have different lengths");
  }
  std::unordered_map<std::string, double> positions;
  positions.reserve(joint_state.names.size());
  for (std::size_t index = 0; index < joint_state.names.size(); ++index) {
    if (joint_state.names[index].empty()) {
      throw std::runtime_error("initial JointState contains an empty joint name");
    }
    const auto inserted = positions.emplace(joint_state.names[index], joint_state.positions[index]);
    if (!inserted.second) {
      throw std::runtime_error(
              "initial JointState contains duplicate joint: " + joint_state.names[index]);
    }
  }

  std::vector<double> result;
  result.reserve(expected_names.size());
  for (const auto & name : expected_names) {
    const auto found = positions.find(name);
    if (found == positions.end()) {
      throw std::runtime_error("initial JointState is missing joint: " + name);
    }
    result.push_back(found->second);
  }
  return result;
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

Json::Value isometryJson(const Eigen::Isometry3d & value)
{
  Json::Value result;
  result["translation"]["x"] = value.translation().x();
  result["translation"]["y"] = value.translation().y();
  result["translation"]["z"] = value.translation().z();
  const Eigen::Quaterniond orientation(value.linear());
  result["orientation_xyzw"]["x"] = orientation.x();
  result["orientation_xyzw"]["y"] = orientation.y();
  result["orientation_xyzw"]["z"] = orientation.z();
  result["orientation_xyzw"]["w"] = orientation.w();
  return result;
}

class RawTerminalSession
{
public:
  RawTerminalSession()
  {
    if (!::isatty(STDIN_FILENO)) {
      throw std::runtime_error(
              "--wait-for-space requires an interactive TTY; use --no-wait-for-space");
    }
    if (::tcgetattr(STDIN_FILENO, &original_) != 0) {
      throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
    }
    termios raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ECHO | ICANON));
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    if (::tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) != 0) {
      throw std::runtime_error("tcsetattr raw failed: " + std::string(std::strerror(errno)));
    }
    active_ = true;
  }

  RawTerminalSession(const RawTerminalSession &) = delete;
  RawTerminalSession & operator=(const RawTerminalSession &) = delete;

  ~RawTerminalSession()
  {
    if (active_) {
      ::tcsetattr(STDIN_FILENO, TCSAFLUSH, &original_);
    }
  }

  std::optional<char> readKeyFor(int timeout_ms) const
  {
    pollfd input{};
    input.fd = STDIN_FILENO;
    input.events = POLLIN;
    while (true) {
      const int ready = ::poll(&input, 1, timeout_ms);
      if (ready == 0) {
        return std::nullopt;
      }
      if (ready > 0) {
        break;
      }
      if (errno != EINTR) {
        throw std::runtime_error("failed to poll start key");
      }
    }

    char key = '\0';
    while (true) {
      const ssize_t count = ::read(STDIN_FILENO, &key, 1);
      if (count == 1) {
        return key;
      }
      if (count < 0 && errno == EINTR) {
        continue;
      }
      throw std::runtime_error("failed to read start key");
    }
  }

private:
  termios original_{};
  bool active_{};
};

void waitForSpace(const std::function<void()> & publish_waiting_frame)
{
  RawTerminalSession terminal;
  std::cout << "Ready. Press Space to start replay (Esc or q cancels)." << std::flush;
  while (true) {
    publish_waiting_frame();
    const auto key = terminal.readKeyFor(250);
    if (!key.has_value()) {
      continue;
    }
    if (*key == ' ') {
      std::cout << "\nReplay started.\n";
      return;
    }
    if (*key == '\033' || *key == 'q' || *key == 'Q') {
      std::cout << '\n';
      throw std::runtime_error("replay cancelled before start");
    }
  }
}

void resolveExperimentOutput(replay::ReplayOptions & options)
{
  const std::filesystem::path definition_path{
    std::string(mcl::e02::build_config::kDefinitionPath)};
  const std::string definition_sha256 = mcl::sha256_file(definition_path);
  if (definition_sha256 != std::string(mcl::e02::build_config::kDefinitionSha256)) {
    throw std::runtime_error(
            "E02 definition changed after configuration; rerun CMake before replay");
  }
  if (options.output_dir_explicit) {
    return;
  }
  const std::filesystem::path output_root = options.output_root.value_or(
    std::filesystem::path{std::string(mcl::e02::build_config::kDefaultOutputRoot)});
  const std::string run_id = options.run_id.value_or(mcl::make_run_id(definition_sha256));
  options.output_dir = output_root / run_id;
}

#if MCL_WITH_REPLAY_VISUALIZATION
std::uint64_t wallClockNanoseconds()
{
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
}

mcv::Pose3d visualizationPose(const Eigen::Isometry3d & pose)
{
  Eigen::Quaterniond orientation(pose.linear());
  orientation.normalize();
  return {
    {pose.translation().x(), pose.translation().y(), pose.translation().z()},
    {orientation.x(), orientation.y(), orientation.z(), orientation.w()}};
}

struct EndEffectorFk
{
  Eigen::Isometry3d left{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d right{Eigen::Isometry3d::Identity()};
};

EndEffectorFk computeEndEffectorFk(
  mcc::GroupedKinematicsSolver & solver,
  const R1ReplayConfig & robot,
  const std::vector<double> & positions,
  const std::vector<double> & velocities)
{
  mcc::ForwardKinematicsRequest request;
  request.state = makeState(positions, velocities);
  request.frame_names = {robot.left_end_effector, robot.right_end_effector};
  request.reference_frame_name = robot.base_frame;
  mcc::ForwardKinematicsSolution solution;
  mcc::ForwardKinematicsDiagnostics diagnostics;
  requireOk(
    solver.computeForwardKinematics(
      mcc::SolverGroup::Red, request, solution, diagnostics),
    "visualization FK failed");
  const auto findPose = [&](const std::string & frame_name) -> const mcc::FramePose & {
      const auto found = std::find_if(
        solution.poses.begin(), solution.poses.end(),
        [&](const mcc::FramePose & pose) { return pose.frame_name == frame_name; });
      if (found == solution.poses.end()) {
        throw std::runtime_error("visualization FK omitted frame: " + frame_name);
      }
      return *found;
    };
  return {
    findPose(robot.left_end_effector).pose,
    findPose(robot.right_end_effector).pose};
}

mcv::VisualizationFrame makeReplayVisualizationFrame(
  const std::string & run_id,
  std::uint64_t sequence,
  std::int64_t sample_time_ns,
  const std::string & left_target_frame_id,
  const Eigen::Isometry3d & left_input_target,
  const std::string & right_target_frame_id,
  const Eigen::Isometry3d & right_input_target,
  const std::string & forward_kinematics_frame_id,
  const Eigen::Isometry3d & left_end_effector_fk,
  const Eigen::Isometry3d & right_end_effector_fk,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & positions,
  const std::vector<double> & velocities,
  const std::string & status,
  bool paused,
  bool solve_accepted,
  double solve_time_ms)
{
  mcv::VisualizationFrame result;
  result.run_id = run_id;
  result.sequence = sequence;
  result.sample_time_ns = sample_time_ns;
  result.sample_clock = "canonical_replay";
  result.emit_time_ns = wallClockNanoseconds();
  result.status = status;
  result.paused = paused;
  result.poses = {
    {"left_input_target", visualization_contract::kLeftTargetPose, left_target_frame_id,
      visualizationPose(left_input_target)},
    {"right_input_target", visualization_contract::kRightTargetPose, right_target_frame_id,
      visualizationPose(right_input_target)},
    {"left_end_effector_fk", visualization_contract::kLeftEndEffectorPose,
      forward_kinematics_frame_id,
      visualizationPose(left_end_effector_fk)},
    {"right_end_effector_fk", visualization_contract::kRightEndEffectorPose,
      forward_kinematics_frame_id,
      visualizationPose(right_end_effector_fk)}};
  result.joints = mcv::JointStateFrame{
    visualization_contract::kJointStates, joint_names, positions, velocities};
  result.diagnostics = {
    {"ik.accepted", solve_accepted ? 1.0 : 0.0, "bool"},
    {"ik.solve_time", solve_time_ms, "ms"}};
  return result;
}
#endif

int run(int argc, char ** argv)
{
  auto options = replay::parseReplayOptions(argc, argv, true);
  if (options.help) {
    std::cout << replay::replayHelp(argv[0], true);
    return EXIT_SUCCESS;
  }
  resolveExperimentOutput(options);
#if !MCL_WITH_REPLAY_VISUALIZATION
  if (options.visualize) {
    throw std::runtime_error(
            "--visualize is unavailable in this build; configure with "
            "-DMCL_BUILD_DUAL_ARM_REPLAY_VISUALIZATION=ON");
  }
#endif

  const std::filesystem::path definition_path{
    std::string(mcl::e02::build_config::kDefinitionPath)};
  const std::string definition_sha256 = mcl::sha256_file(definition_path);
  const Json::Value resolved_definition = mcl::load_json_file(definition_path);

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

  const std::vector<double> initial_positions = loaded.initial_joint_state.has_value()
    ? positionsByJointName(*loaded.initial_joint_state, robot.joint_names)
    : robot.initial_positions;
  replay::createOutputDirectory(options.output_dir);
  std::vector<double> positions = initial_positions;
  std::vector<double> velocities(positions.size(), 0.0);
  const auto fixed_positions = positions;
  const auto fixed_velocities = velocities;
  mcl::data::ReplayClock clock(options.execution_mode, options.playback_rate);
  std::size_t deadline_misses = 0;
  std::size_t accepted_count = 0;
  std::function<void()> publish_waiting_frame = []() {};

#if MCL_WITH_REPLAY_VISUALIZATION
  std::unique_ptr<mcv::FoxgloveFrameSink> visualization_sink;
  if (options.visualize) {
    mcv::FoxgloveFrameSinkOptions sink_options;
    sink_options.server_name = "mcl_dual_arm_replay_ik";
    sink_options.host = options.visualization_host;
    sink_options.port = options.visualization_port;
    if (options.record_visualization_mcap) {
      sink_options.mcap_path = options.output_dir / "visualization.mcap";
    }
    visualization_sink = std::make_unique<mcv::FoxgloveFrameSink>(std::move(sink_options));
    visualization_sink->open({options.output_dir.filename().string(), "mcl_dual_arm_replay_ik"});

    const auto first_frame = loaded.timeline.timeline.at(0);
    const auto initial_fk = computeEndEffectorFk(
      solver, robot, initial_positions, fixed_velocities);
    publish_waiting_frame = [&, first_frame, initial_fk]() {
        visualization_sink->write(makeReplayVisualizationFrame(
          options.output_dir.filename().string(),
          0,
          0,
          first_frame.value.left.frame_id,
          first_frame.value.left.pose,
          first_frame.value.right.frame_id,
          first_frame.value.right.pose,
          robot.base_frame,
          initial_fk.left,
          initial_fk.right,
          robot.joint_names,
          initial_positions,
          fixed_velocities,
          options.wait_for_space ? "waiting_for_space" : "initialized",
          options.wait_for_space,
          false,
          0.0));
      };
    publish_waiting_frame();
    std::cout << "Foxglove: " << visualization_sink->status() << '\n';
  }
#endif

  if (options.wait_for_space) {
    waitForSpace(publish_waiting_frame);
  }
  // The replay clock origin is deliberately established after the optional
  // human start gate, so waiting in Foxglove cannot create artificial lateness.
  const auto run_start = clock.now();

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
    const auto status = solver.solveInverseKinematics(
      mcc::SolverGroup::Red, request, solution, diagnostics);
    const auto solve_end = clock.now();
    const bool accepted = status.ok() && diagnostics.attempt_accepted &&
      solution.kinematics_solution.joint_positions.size() ==
      static_cast<Eigen::Index>(robot.joint_names.size());
    std::vector<double> output_positions = state_positions;
    std::vector<double> output_velocities = state_velocities;
    if (accepted) {
      output_positions = toVector(solution.kinematics_solution.joint_positions);
      if (solution.kinematics_solution.joint_velocities.size() ==
          static_cast<Eigen::Index>(robot.joint_names.size())) {
        output_velocities = toVector(solution.kinematics_solution.joint_velocities);
      } else {
        output_velocities.assign(output_positions.size(), 0.0);
      }
      ++accepted_count;
      if (options.state_policy == replay::StatePolicy::PreviousSolution) {
        positions = output_positions;
        velocities = output_velocities;
      }
    }

    const std::string status_text = status.ok()
      ? (accepted ? "ok" : "attempt_rejected")
      : status.message;
#if MCL_WITH_REPLAY_VISUALIZATION
    if (visualization_sink) {
      const auto output_fk = computeEndEffectorFk(
        solver, robot, output_positions, output_velocities);
      visualization_sink->write(makeReplayVisualizationFrame(
        options.output_dir.filename().string(),
        frame.sequence + 1,
        frame.projected_time_ns,
        frame.value.left.frame_id,
        frame.value.left.pose,
        frame.value.right.frame_id,
        frame.value.right.pose,
        robot.base_frame,
        output_fk.left,
        output_fk.right,
        robot.joint_names,
        output_positions,
        output_velocities,
        status_text,
        false,
        accepted,
        diagnostics.kinematics.solve_time_ms));
    }
#endif
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

#if MCL_WITH_REPLAY_VISUALIZATION
  if (visualization_sink) {
    visualization_sink->flush();
    visualization_sink->close();
  }
#endif

  const auto trace_path = options.output_dir / "trace.csv";
  replay::writeTextFile(trace_path, trace.str());
  const bool completed = accepted_count == loaded.timeline.timeline.size();
  Json::Value run_status;
  run_status["schema_version"] = "dual_arm_replay_status.v1";
  run_status["status"] = completed ? "completed" : "failed";
  run_status["frames"] = Json::UInt64(loaded.timeline.timeline.size());
  run_status["accepted_solves"] = Json::UInt64(accepted_count);
  run_status["rejected_solves"] = Json::UInt64(
    loaded.timeline.timeline.size() - accepted_count);
  run_status["deadline_misses"] = Json::UInt64(deadline_misses);
  const auto status_path = options.output_dir / "status.json";
  replay::writeTextFile(status_path, jsonString(run_status));

  auto manifest = replay::makeReplayManifest(
    options, loaded, deadline_misses, accepted_count, mcl::sha256_file(trace_path));
  manifest["run_id"] = options.output_dir.filename().string();
  manifest["status"] = completed ? "completed" : "failed";
  manifest["experiment"]["id"] = "E02";
  manifest["experiment"]["formal_default_output"] = !options.output_dir_explicit;
  manifest["experiment"]["definition_path"] =
    std::filesystem::absolute(definition_path).lexically_normal().string();
  manifest["experiment"]["definition_sha256"] = definition_sha256;
  manifest["experiment"]["resolved_definition"] = resolved_definition;
  manifest["source_control"]["revision"] =
    std::string(mcl::e02::build_config::kSourceRevision);
  manifest["source_control"]["dirty"] = mcl::e02::build_config::kSourceDirty;
  manifest["environment"]["runtime"] = std::string(mcl::e02::build_config::kRuntime);
  manifest["artifacts"]["status.json"]["sha256"] = mcl::sha256_file(status_path);
  manifest["robot_model"]["urdf_path"] =
    std::filesystem::absolute(options.urdf_path).lexically_normal().string();
  manifest["robot_model"]["urdf_sha256"] = mcl::sha256_file(options.urdf_path);
  manifest["solver"]["profile"] = "RedOnly";
  manifest["solver"]["servo_mode"] = "ServoStep";
  manifest["target_pose"]["input_semantics"] = "tcp";
  manifest["target_pose"]["solver_semantics"] = "end_effector";
  manifest["target_pose"]["conversion"] = "end_effector_target=tcp_target*tcp_offset.inverse()";
  manifest["target_pose"]["tcp_offsets"]["left"] = isometryJson(robot.left_tcp_offset);
  manifest["target_pose"]["tcp_offsets"]["right"] = isometryJson(robot.right_tcp_offset);
  manifest["initial_state"]["source"] = loaded.initial_joint_state.has_value()
    ? "mcap_first_joint_state" : "r1_replay_config.v1";
  manifest["initial_state"]["velocity_source"] = "zero";
  if (loaded.initial_joint_state.has_value()) {
    manifest["initial_state"]["stream"] = *options.initial_joint_state_stream;
    manifest["initial_state"]["decoder"] = loaded.initial_joint_state_decoder;
    manifest["initial_state"]["sample_index"] = Json::UInt64(0);
    if (loaded.initial_joint_state->time.header_stamp_ns.has_value()) {
      manifest["initial_state"]["header_stamp_ns"] = Json::Int64(
        *loaded.initial_joint_state->time.header_stamp_ns);
    }
    if (loaded.initial_joint_state->time.log_time_ns.has_value()) {
      manifest["initial_state"]["log_time_ns"] = Json::Int64(
        *loaded.initial_joint_state->time.log_time_ns);
    }
    if (loaded.initial_joint_state->time.publish_time_ns.has_value()) {
      manifest["initial_state"]["publish_time_ns"] = Json::Int64(
        *loaded.initial_joint_state->time.publish_time_ns);
    }
  }
  for (const auto & name : robot.joint_names) {
    manifest["initial_state"]["joint_names"].append(name);
  }
  for (const double position : initial_positions) {
    manifest["initial_state"]["joint_positions"].append(position);
  }
  manifest["visualization"]["enabled"] = options.visualize;
  manifest["visualization"]["wait_for_space"] = options.wait_for_space;
  if (options.visualize) {
    manifest["visualization"]["contract"] = visualization_contract::kSchemaVersion;
    manifest["visualization"]["host"] = options.visualization_host;
    manifest["visualization"]["port"] = options.visualization_port;
    for (const auto * channel : visualization_contract::kRequiredChannels) {
      manifest["visualization"]["channels"].append(channel);
    }
  }
  if (options.record_visualization_mcap) {
    const auto visualization_mcap = options.output_dir / "visualization.mcap";
    manifest["visualization"]["mcap"] = "visualization.mcap";
    manifest["artifacts"]["visualization.mcap"]["sha256"] =
      mcl::sha256_file(visualization_mcap);
  }
  replay::writeTextFile(options.output_dir / "manifest.json", jsonString(manifest));

  std::cout
    << "frames=" << loaded.timeline.timeline.size()
    << " accepted=" << accepted_count
    << " deadline_misses=" << deadline_misses
    << " output=" << options.output_dir.string() << '\n';
  return completed ? EXIT_SUCCESS : EXIT_FAILURE;
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
