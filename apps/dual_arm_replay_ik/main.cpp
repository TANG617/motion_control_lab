#include "apps/dual_arm_replay_ik/replay_ik_engine.hpp"
#include "cpu_affinity.hpp"
#include "contracts/visualization/foxglove_ik_v1.hpp"
#include "e02_build_config.hpp"
#include "motion_control_lab/run_artifacts.hpp"
#include "motion_control_lab/sha256.hpp"

#if MCL_WITH_REPLAY_VISUALIZATION
#include <motion_control_viz/foxglove_frame_sink.hpp>
#include <motion_control_viz/frame.hpp>
#endif

#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Geometry>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

namespace
{
namespace mcl = motion_control_lab;
namespace replay = motion_control_lab::replay;
namespace visualization_contract = motion_control_lab::contracts::foxglove_ik_v1;
#if MCL_WITH_REPLAY_VISUALIZATION
namespace mcv = motion_control::viz;
#endif

constexpr const char * kProgramId = "mcl_dual_arm_replay_ik";
constexpr std::array<unsigned int, 1> kMainCpuAffinity{8};

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

Json::Value cpuAffinityJson(const mcl::CpuAffinityBinding & binding)
{
  Json::Value result;
  result["enabled"] = binding.enabled;
  result["roles"][binding.role]["requested_cpus"] = Json::Value(Json::arrayValue);
  result["roles"][binding.role]["effective_cpus"] = Json::Value(Json::arrayValue);
  for (const auto cpu : binding.requested_cpus) {
    result["roles"][binding.role]["requested_cpus"].append(cpu);
  }
  for (const auto cpu : binding.effective_cpus) {
    result["roles"][binding.role]["effective_cpus"].append(cpu);
  }
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
  const std::filesystem::path definition_path{std::string(mcl::e02::build_config::kDefinitionPath)};
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
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::system_clock::now().time_since_epoch())
                                      .count());
}

mcv::Pose3d visualizationPose(const Eigen::Isometry3d & pose)
{
  Eigen::Quaterniond orientation(pose.linear());
  orientation.normalize();
  return {
    {pose.translation().x(), pose.translation().y(), pose.translation().z()},
    {orientation.x(), orientation.y(), orientation.z(), orientation.w()}};
}

mcv::VisualizationFrame makeReplayVisualizationFrame(
  const std::string & run_id, const replay::ReplayIkVisualizationSample & sample)
{
  mcv::VisualizationFrame result;
  result.run_id = run_id;
  result.sequence = sample.sequence;
  result.sample_time_ns = sample.sample_time_ns;
  result.sample_clock = "canonical_replay";
  result.emit_time_ns = wallClockNanoseconds();
  result.status = sample.status;
  result.paused = sample.paused;
  result.poses = {
    {"left_input_target", visualization_contract::kLeftTargetPose, sample.left_target_frame_id,
     visualizationPose(sample.left_input_target)},
    {"right_input_target", visualization_contract::kRightTargetPose, sample.right_target_frame_id,
     visualizationPose(sample.right_input_target)},
    {"left_end_effector_fk", visualization_contract::kLeftEndEffectorPose,
     sample.forward_kinematics_frame_id, visualizationPose(sample.left_end_effector_fk)},
    {"right_end_effector_fk", visualization_contract::kRightEndEffectorPose,
     sample.forward_kinematics_frame_id, visualizationPose(sample.right_end_effector_fk)}};
  result.joints = mcv::JointStateFrame{
    visualization_contract::kJointStates, sample.joint_names, sample.positions, sample.velocities};
  result.diagnostics = {
    {"ik.accepted", sample.solve_accepted ? 1.0 : 0.0, "bool"},
    {"ik.solve_time", sample.solve_time_ms, "ms"}};
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
  const auto affinity_domain = mcl::CpuAffinityDomain::capture();
  const auto affinity_binding =
    affinity_domain.bindCurrentThread(kProgramId, "main", kMainCpuAffinity);
  resolveExperimentOutput(options);
#if !MCL_WITH_REPLAY_VISUALIZATION
  if (options.visualize) {
    throw std::runtime_error(
      "--visualize is unavailable in this build; configure with "
      "-DMCL_BUILD_DUAL_ARM_REPLAY_VISUALIZATION=ON");
  }
#endif

  const std::filesystem::path definition_path{std::string(mcl::e02::build_config::kDefinitionPath)};
  const std::string definition_sha256 = mcl::sha256_file(definition_path);
  const Json::Value resolved_definition = mcl::load_json_file(definition_path);
  bool output_created = false;
  const auto ensureOutputDirectory = [&]() {
    if (!output_created) {
      replay::createOutputDirectory(options.output_dir);
      output_created = true;
    }
  };

  replay::ReplayIkExecutionConfig execution_config;
  execution_config.stop_on_first_error = true;
  execution_config.before_replay = ensureOutputDirectory;
#if MCL_WITH_REPLAY_VISUALIZATION
  std::unique_ptr<mcv::FoxgloveFrameSink> visualization_sink;
  const auto publish = [&](const replay::ReplayIkVisualizationSample & sample) {
    ensureOutputDirectory();
    if (!visualization_sink) {
      mcv::FoxgloveFrameSinkOptions sink_options;
      sink_options.server_name = "mcl_dual_arm_replay_ik";
      sink_options.host = options.visualization_host;
      sink_options.port = options.visualization_port;
      if (options.record_visualization_mcap) {
        sink_options.mcap_path = options.output_dir / "visualization.mcap";
      }
      visualization_sink = std::make_unique<mcv::FoxgloveFrameSink>(std::move(sink_options));
      visualization_sink->open({options.output_dir.filename().string(), "mcl_dual_arm_replay_ik"});
      std::cout << "Foxglove: " << visualization_sink->status() << '\n';
    }
    visualization_sink->write(
      makeReplayVisualizationFrame(options.output_dir.filename().string(), sample));
  };
  if (options.visualize) {
    execution_config.visualization_callback = publish;
  }
#endif
  if (options.wait_for_space) {
    execution_config.initial_frame_gate = [&](const replay::ReplayIkVisualizationSample & sample) {
#if MCL_WITH_REPLAY_VISUALIZATION
      const auto publish_waiting = [&]() {
        if (options.visualize) {
          publish(sample);
        }
      };
#else
      const auto publish_waiting = []() {};
#endif
      publish_waiting();
      waitForSpace(publish_waiting);
    };
  }

  auto result = replay::executeReplayIkCase(options, execution_config);
#if MCL_WITH_REPLAY_VISUALIZATION
  if (visualization_sink) {
    visualization_sink->flush();
    visualization_sink->close();
  }
#endif
  ensureOutputDirectory();

  const auto trace_path = options.output_dir / "trace.csv";
  replay::writeTextFile(trace_path, result.trace_csv);
  Json::Value run_status;
  run_status["schema_version"] = "dual_arm_replay_status.v1";
  run_status["status"] = result.completed() ? "completed" : "failed";
  run_status["frames"] = Json::UInt64(result.frames_planned);
  run_status["accepted_solves"] = Json::UInt64(result.accepted_solves);
  run_status["rejected_solves"] = Json::UInt64(result.rejected_solves);
  run_status["deadline_misses"] = Json::UInt64(result.deadline_misses);
  const auto status_path = options.output_dir / "status.json";
  replay::writeTextFile(status_path, jsonString(run_status));

  auto manifest = replay::makeReplayManifest(
    options, result.loaded, result.deadline_misses, result.accepted_solves,
    mcl::sha256_file(trace_path));
  manifest["run_id"] = options.output_dir.filename().string();
  manifest["status"] = result.completed() ? "completed" : "failed";
  manifest["experiment"]["id"] = "E02";
  manifest["experiment"]["formal_default_output"] = !options.output_dir_explicit;
  manifest["experiment"]["definition_path"] =
    std::filesystem::absolute(definition_path).lexically_normal().string();
  manifest["experiment"]["definition_sha256"] = definition_sha256;
  manifest["experiment"]["resolved_definition"] = resolved_definition;
  manifest["source_control"]["revision"] = std::string(mcl::e02::build_config::kSourceRevision);
  manifest["source_control"]["dirty"] = mcl::e02::build_config::kSourceDirty;
  manifest["environment"]["runtime"] = std::string(mcl::e02::build_config::kRuntime);
  manifest["execution"]["cpu_affinity"] = cpuAffinityJson(affinity_binding);
  manifest["artifacts"]["status.json"]["sha256"] = mcl::sha256_file(status_path);
  manifest["robot_model"]["urdf_path"] =
    std::filesystem::absolute(options.urdf_path).lexically_normal().string();
  manifest["robot_model"]["urdf_sha256"] = mcl::sha256_file(options.urdf_path);
  manifest["solver"]["profile"] = "RedOnly";
  manifest["solver"]["servo_mode"] = "ServoStep";
  manifest["target_pose"]["input_semantics"] = "tcp";
  manifest["target_pose"]["solver_semantics"] = "end_effector";
  manifest["target_pose"]["conversion"] = "end_effector_target=tcp_target*tcp_offset.inverse()";
  const auto & robot = mcl::r1RobotConfig();
  manifest["target_pose"]["tcp_offsets"]["left"] = isometryJson(robot.left_tcp_offset);
  manifest["target_pose"]["tcp_offsets"]["right"] = isometryJson(robot.right_tcp_offset);
  manifest["initial_state"]["source"] = result.loaded.initial_joint_state.has_value()
                                          ? "mcap_first_joint_state"
                                          : "r1_replay_config.v1";
  manifest["initial_state"]["velocity_source"] = "zero";
  if (result.loaded.initial_joint_state.has_value()) {
    manifest["initial_state"]["stream"] = *options.initial_joint_state_stream;
    manifest["initial_state"]["decoder"] = result.loaded.initial_joint_state_decoder;
    manifest["initial_state"]["sample_index"] = Json::UInt64(0);
    if (result.loaded.initial_joint_state->time.header_stamp_ns.has_value()) {
      manifest["initial_state"]["header_stamp_ns"] =
        Json::Int64(*result.loaded.initial_joint_state->time.header_stamp_ns);
    }
    if (result.loaded.initial_joint_state->time.log_time_ns.has_value()) {
      manifest["initial_state"]["log_time_ns"] =
        Json::Int64(*result.loaded.initial_joint_state->time.log_time_ns);
    }
    if (result.loaded.initial_joint_state->time.publish_time_ns.has_value()) {
      manifest["initial_state"]["publish_time_ns"] =
        Json::Int64(*result.loaded.initial_joint_state->time.publish_time_ns);
    }
  }
  for (const auto & name : robot.joint_names) {
    manifest["initial_state"]["joint_names"].append(name);
  }
  for (const double position : result.initial_positions) {
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
    manifest["artifacts"]["visualization.mcap"]["sha256"] = mcl::sha256_file(visualization_mcap);
  }
  replay::writeTextFile(options.output_dir / "manifest.json", jsonString(manifest));

  std::cout << "frames=" << result.frames_planned << " accepted=" << result.accepted_solves
            << " deadline_misses=" << result.deadline_misses
            << " output=" << options.output_dir.string() << '\n';
  return result.completed() ? EXIT_SUCCESS : EXIT_FAILURE;
}

}  // namespace

int main(int argc, char ** argv)
{
  return run(argc, argv);
}
