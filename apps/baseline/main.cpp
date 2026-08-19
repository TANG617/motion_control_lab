#include "baseline_config.hpp"
#include "baseline_solver.hpp"
#include "app_options.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "adapters/replay/replay_support.hpp"
#include "console/tui_console.hpp"
#include "cpu_affinity.hpp"
#include "ik_app_utils.hpp"
#include "motion_control_lab/run_artifacts.hpp"
#include "motion_control_lab/sha256.hpp"
#include "r1_interactive_config.hpp"
#include "r1_robot_config.hpp"
#include "runtime/interactive_scheduler.hpp"
#include "runtime/rolling_percentiles.hpp"
#include "sinks/ik_render_batch.hpp"
#include "sinks/preview_sink_factory.hpp"

namespace
{

namespace baseline = motion_control_lab::baseline;
namespace mcl = motion_control_lab;
namespace replay = motion_control_lab::replay;

constexpr const char * kProgramId = "mcl_baseline";
constexpr const char * kTitle = "PlaCo Production-Static Baseline";
constexpr std::array<unsigned int, 1> kMainCpuAffinity{8};

using baseline::ReplayAppOptions;
using baseline::parseReplayOptions;
using baseline::parseTeleopOptions;
using baseline::printTopLevelUsage;

std::string sourceSummary()
{
  return baseline::productionStaticConfig().source_revision.substr(0, 12);
}

std::string traceVector(const std::vector<double> & values)
{
  std::ostringstream output;
  output << '"' << std::fixed << std::setprecision(12);
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {
      output << ';';
    }
    const double canonical_value = std::fabs(values[index]) < 5.0e-13 ? 0.0 : values[index];
    output << canonical_value;
  }
  output << '"';
  return output.str();
}

std::string traceScalar(double value)
{
  const double canonical_value = std::fabs(value) < 5.0e-13 ? 0.0 : value;
  std::ostringstream output;
  output << std::fixed << std::setprecision(12) << canonical_value;
  return output.str();
}

std::string tracePose(const mcl::Pose & pose)
{
  const Eigen::Quaterniond orientation{pose.rotation()};
  return traceVector({pose.translation().x(), pose.translation().y(), pose.translation().z(),
                      orientation.x(), orientation.y(), orientation.z(), orientation.w()});
}

std::string jsonText(const Json::Value & value)
{
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "  ";
  return Json::writeString(builder, value) + "\n";
}

Json::Value baselineManifest(const replay::ReplayOptions & options,
                             const replay::LoadedReplay & loaded,
                             const replay::ReplayExecutionMetadata & execution,
                             const std::string & trace_sha256, const std::string & config_sha256)
{
  auto manifest = replay::makeReplayManifest(options, loaded, execution, trace_sha256);
  manifest["baseline"]["schema_version"] = baseline::productionStaticConfig().schema_version;
  manifest["baseline"]["source_revision"] = baseline::productionStaticConfig().source_revision;
  manifest["baseline"]["config_sha256"] = config_sha256;
  manifest["artifacts"]["baseline_config.json"]["sha256"] = config_sha256;
  return manifest;
}

Json::Value baselineStatus(const replay::LoadedReplay & loaded,
                           const replay::ReplayExecutionMetadata & execution,
                           const std::string & state, const std::string & config_sha256,
                           const std::string & error = {})
{
  auto status = replay::makeReplayStatus(loaded, execution, state, error);
  status["baseline_config_sha256"] = config_sha256;
  status["source_revision"] = baseline::productionStaticConfig().source_revision;
  return status;
}

std::string taskSummary()
{
  return "source=" + sourceSummary() +
         "; tasks: position(x2)=scaled/9, orientation(x2)=soft/4, "
         "posture(x12)=soft/{0.1,0.4,0.6,0.9,1.0}, internal-reg=soft/1e-6, "
         "kinetic=soft/0.1, absolute-limits=hard/1";
}

template <typename Solver>
int runTeleopLoop(const baseline::TeleopOptions & options, Solver & solver)
{
  const auto & robot = mcl::r1RobotConfig();
  const auto affinity_domain = mcl::CpuAffinityDomain::capture();
  const auto affinity_binding =
      affinity_domain.bindCurrentThread(kProgramId, "main", kMainCpuAffinity);
  const auto presentation = mcl::makeArmPresentation(robot, mcl::foxgloveIkVisualizationChannels());

  const auto initial_left_tcp = solver.currentTcpPose(mcl::ArmSide::Left);
  const auto initial_right_tcp = solver.currentTcpPose(mcl::ArmSide::Right);
  std::vector<mcl::ArmTarget> initial_targets{
      {mcl::ArmSide::Left, initial_left_tcp},
      {mcl::ArmSide::Right, initial_right_tcp},
  };
  mcl::TuiConsole tui(options.tui, options.rate_hz,
                      std::string{kTitle} + " [source " + sourceSummary() + "]", presentation,
                      initial_targets, true, options.tui_enabled);
  auto visualization_sink = mcl::createPreviewSink(options.visualization, kProgramId);

  mcl::installInteractiveSignalHandlers();
  mcl::InteractiveScheduler scheduler({options.rate_hz, options.duration_s});
  mcl::RollingPercentiles solve_time_percentiles;
  mcl::SolverRunCounters counters;
  std::size_t publish_count = 0;

  mcl::IkDebugFrame frame;
  frame.run_id = "placo-production-static-" + sourceSummary();
  frame.targets = initial_targets;
  frame.forward_kinematics = {{mcl::ArmSide::Left, initial_left_tcp},
                              {mcl::ArmSide::Right, initial_right_tcp}};
  frame.joint_names = robot.joint_names;
  frame.positions = solver.positions();
  frame.velocities = solver.velocities();
  frame.selected_side = mcl::parseArmSide(options.tui.side);
  frame.cpu_affinities = {mcl::makeCpuAffinityDebug(affinity_binding)};

  visualization_sink->open();
  while (const auto schedule = scheduler.next()) {
    tui.poll();
    if (const auto reset_side = tui.consumeResetRequest()) {
      tui.setTargetPose(*reset_side, solver.currentTcpPose(*reset_side),
                        std::string{"Reset "} + mcl::armSideName(*reset_side) +
                            " target from current TCP");
    }
    const auto & command = tui.command();
    if (command.stop_requested) {
      break;
    }

    if (schedule->update_due && !command.paused) {
      ++counters.attempts;
      auto result = solver.solve(command.targets);
      ++counters.accepted;
      solve_time_percentiles.record(result.solve_time_ms);
      result.solver_debug.ik_solve_time_percentiles = solve_time_percentiles.snapshot();
      result.solver_debug.run_counters = counters;
      result.solver_debug.native_status = taskSummary();

      frame.targets = command.targets;
      frame.forward_kinematics = result.tcp_forward_kinematics;
      frame.positions = result.positions;
      frame.velocities = result.velocities;
      frame.ik_status = result.status_name;
      frame.iterations = result.iterations;
      frame.converged = result.converged;
      frame.solve_time_ms = result.solve_time_ms;
      frame.target_errors = result.tcp_target_errors;
      frame.solvers = {std::move(result.solver_debug)};
      frame.status = result.status_name + "; frame scale=" + std::to_string(result.frame_scale) +
                     "; source=" + sourceSummary();
      frame.paused = command.paused;
      frame.selected_side = command.selected_side;
      auto visualization_debug_frame = frame;
      visualization_debug_frame.forward_kinematics = result.end_effector_forward_kinematics;
      visualization_debug_frame.target_errors = result.end_effector_target_errors;
      visualization_sink->write(
          mcl::makeIkRenderBatch(
            visualization_debug_frame, presentation, schedule->emit_time_ns));
      ++publish_count;
    }

    if (schedule->draw_due) {
      frame.paused = tui.command().paused;
      frame.selected_side = tui.command().selected_side;
      tui.render(frame, publish_count, visualization_sink->status());
    }
    scheduler.sleep();
  }

  visualization_sink->flush();
  visualization_sink->close();
  return EXIT_SUCCESS;
}

int runTeleop(int argc, char ** argv)
{
  const auto options = parseTeleopOptions(argc, argv);
  baseline::BaselineSolver solver(options.urdf_path);
  return runTeleopLoop(options, solver);
}

int runReplayLoop(ReplayAppOptions options, baseline::BaselineSolver & solver,
                  const replay::LoadedReplay & loaded)
{
  if (loaded.timeline.timeline.empty()) {
    throw std::runtime_error("replay timeline is empty");
  }
  if (!options.replay.output_dir_explicit) {
    const std::string run_id = options.replay.run_id.value_or(
        mcl::make_run_id(mcl::sha256_file(options.replay.input_path)));
    options.replay.output_dir =
        options.replay.output_root.value_or(std::filesystem::path{"runs/baseline"}) / run_id;
  }
  replay::createOutputDirectory(options.replay.output_dir);

  const std::string config_text = baseline::productionStaticConfigJsonText();
  const std::string config_sha256 = mcl::sha256_text(config_text);
  replay::writeTextFile(options.replay.output_dir / "baseline_config.json", config_text);

  const auto & robot = mcl::r1RobotConfig();
  const auto & first = loaded.timeline.timeline.at(0);
  std::vector<mcl::ArmTarget> targets{
      {mcl::ArmSide::Left, first.value.left.pose},
      {mcl::ArmSide::Right, first.value.right.pose},
  };
  const auto presentation = mcl::makeArmPresentation(robot, mcl::foxgloveIkVisualizationChannels());
  mcl::TuiConsole tui(baseline::TeleopOptions{}.tui,
                      baseline::productionStaticConfig().control_rate_hz,
                      std::string{kTitle} + " Replay [source " + sourceSummary() + "]",
                      presentation, targets, true, options.replay.ui_mode == "tui",
                      mcl::TuiControlMode::Replay);
  tui.setMotionInputEnabled(false, "Replay motion editing is disabled");

  mcl::PreviewSinkOptions sink_options;
  sink_options.host = options.replay.visualization_host;
  sink_options.port = options.replay.visualization_port;
  sink_options.mcap_path = options.replay.visualization_mcap_path;
  auto visualization_sink = mcl::createPreviewSink(sink_options, kProgramId);
  visualization_sink->open();
  mcl::installInteractiveSignalHandlers();

  replay::ReplayExecutionMetadata execution;
  execution.app = kProgramId;
  execution.topology = "ordinary-target-solve-production-static";
  execution.solver = "placo";
  execution.backend = "eiquadprog";
  execution.rate_hz = baseline::productionStaticConfig().control_rate_hz;
  execution.consumed_frame_count = 1U;

  std::ostringstream trace;
  trace << "attempt,source_sequence,original_logical_timestamp_ns,source_time_"
           "from_start_ns,"
           "projected_timestamp_ns,left_header_stamp_ns,left_log_time_ns,left_"
           "publish_time_ns,"
           "right_header_stamp_ns,right_log_time_ns,right_publish_time_ns,"
           "accepted,solver_status,"
           "termination_reason,iterations,converged,frame_scale,solve_time_ms,"
           "maximum_hard_violation,left_ee_position_error_m,left_ee_"
           "orientation_error_rad,"
           "right_ee_position_error_m,right_ee_orientation_error_rad,left_tcp_"
           "position_error_m,"
           "left_tcp_orientation_error_rad,right_tcp_position_error_m,right_"
           "tcp_orientation_error_rad,"
           "left_tcp_target,left_ee_target,right_tcp_target,right_ee_target,"
           "left_ee_fk,right_ee_fk,"
           "left_tcp_fk,right_tcp_fk,positions,velocities\n";

  const std::int64_t solver_period_ns = baseline::kTargetPeriodNs;
  const auto run_start = std::chrono::steady_clock::now();
  std::int64_t solver_time_ns = 0;
  std::int64_t timeline_time_ns = 0;
  std::size_t source_index = 0;
  std::size_t attempt = 0;
  std::size_t publish_count = 0;
  std::string replay_state{"running"};

  try {
    while (true) {
      if (options.replay.execution_mode == mcl::data::ExecutionMode::Realtime) {
        const auto deadline = run_start + std::chrono::nanoseconds(solver_time_ns);
        std::this_thread::sleep_until(deadline);
        if (std::chrono::steady_clock::now() > deadline) {
          ++execution.deadline_miss_count;
        }
      }
      tui.poll();
      if (tui.command().stop_requested) {
        replay_state = "stopped";
        break;
      }

      const bool single_step = tui.consumeSingleStepRequest();
      if (!tui.command().paused || single_step) {
        if (single_step) {
          if (source_index + 1U < loaded.timeline.timeline.size()) {
            timeline_time_ns = loaded.timeline.timeline.at(source_index + 1U).projected_time_ns;
          }
        } else if (attempt != 0U) {
          timeline_time_ns += static_cast<std::int64_t>(
              std::llround(static_cast<double>(solver_period_ns) * options.replay.playback_rate));
        }
        std::size_t next_index = source_index;
        while (next_index + 1U < loaded.timeline.timeline.size() &&
               loaded.timeline.timeline.at(next_index + 1U).projected_time_ns <= timeline_time_ns) {
          ++next_index;
        }
        if (next_index > source_index) {
          execution.dropped_frame_count += next_index - source_index - 1U;
          ++execution.consumed_frame_count;
          source_index = next_index;
          const auto & source = loaded.timeline.timeline.at(source_index);
          targets[0].target_pose = source.value.left.pose;
          targets[1].target_pose = source.value.right.pose;
          tui.setTargetPose(mcl::ArmSide::Left, targets[0].target_pose, "Replay TCP goal advanced");
          tui.setTargetPose(mcl::ArmSide::Right, targets[1].target_pose,
                            "Replay TCP goal advanced");
        }
      }

      auto result = solver.solve(targets);
      ++attempt;
      ++execution.accepted_count;
      const auto & source = loaded.timeline.timeline.at(source_index);
      const auto & left_ee_error = result.end_effector_target_errors.at(0);
      const auto & right_ee_error = result.end_effector_target_errors.at(1);
      const auto & left_tcp_error = result.tcp_target_errors.at(0);
      const auto & right_tcp_error = result.tcp_target_errors.at(1);
      trace << attempt << ',' << source.sequence << ',' << source.original_logical_time_ns << ','
            << source.source_time_from_start_ns << ',' << source.projected_time_ns << ','
            << replay::optionalTimestamp(source.value.left.time.header_stamp_ns) << ','
            << replay::optionalTimestamp(source.value.left.time.log_time_ns) << ','
            << replay::optionalTimestamp(source.value.left.time.publish_time_ns) << ','
            << replay::optionalTimestamp(source.value.right.time.header_stamp_ns) << ','
            << replay::optionalTimestamp(source.value.right.time.log_time_ns) << ','
            << replay::optionalTimestamp(source.value.right.time.publish_time_ns) << ",true,"
            << result.status_name << ',' << result.termination_reason << ',' << result.iterations
            << ',' << std::boolalpha << result.converged << ',' << traceScalar(result.frame_scale)
            << ',' << std::setprecision(17) << result.solve_time_ms << ','
            << traceScalar(result.maximum_hard_violation) << ','
            << traceScalar(left_ee_error.position_m) << ','
            << traceScalar(left_ee_error.orientation_rad) << ','
            << traceScalar(right_ee_error.position_m) << ','
            << traceScalar(right_ee_error.orientation_rad) << ','
            << traceScalar(left_tcp_error.position_m) << ','
            << traceScalar(left_tcp_error.orientation_rad) << ','
            << traceScalar(right_tcp_error.position_m) << ','
            << traceScalar(right_tcp_error.orientation_rad) << ','
            << tracePose(targets[0].target_pose) << ','
            << tracePose(result.internal_end_effector_targets[0].target_pose) << ','
            << tracePose(targets[1].target_pose) << ','
            << tracePose(result.internal_end_effector_targets[1].target_pose) << ','
            << tracePose(result.end_effector_forward_kinematics[0].pose) << ','
            << tracePose(result.end_effector_forward_kinematics[1].pose) << ','
            << tracePose(result.tcp_forward_kinematics[0].pose) << ','
            << tracePose(result.tcp_forward_kinematics[1].pose) << ','
            << traceVector(result.positions) << ',' << traceVector(result.velocities) << '\n';

      result.solver_debug.native_status = taskSummary();
      mcl::IkDebugFrame frame;
      frame.run_id = options.replay.output_dir.filename().string();
      frame.targets = targets;
      frame.forward_kinematics = result.tcp_forward_kinematics;
      frame.joint_names = robot.joint_names;
      frame.positions = result.positions;
      frame.velocities = result.velocities;
      frame.solvers = {result.solver_debug};
      frame.target_errors = result.tcp_target_errors;
      frame.iterations = result.iterations;
      frame.converged = result.converged;
      frame.solve_time_ms = result.solve_time_ms;
      frame.ik_status = result.status_name;
      frame.status = "Replay source=" + std::to_string(source.sequence) +
                     " frame scale=" + std::to_string(result.frame_scale) +
                     " config=" + config_sha256.substr(0, 12);
      frame.paused = tui.command().paused;
      auto visualization_debug_frame = frame;
      visualization_debug_frame.forward_kinematics = result.end_effector_forward_kinematics;
      visualization_debug_frame.target_errors = result.end_effector_target_errors;
      visualization_sink->write(mcl::makeIkRenderBatch(
          visualization_debug_frame, presentation,
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::system_clock::now().time_since_epoch())
              .count()));
      tui.render(frame, publish_count, visualization_sink->status());
      ++publish_count;

      if (source_index + 1U == loaded.timeline.timeline.size() && !tui.command().paused) {
        replay_state = "succeeded";
        break;
      }
      solver_time_ns += solver_period_ns;
    }
  } catch (const std::exception & error) {
    ++execution.rejected_count;
    replay_state = "failed";
    const auto trace_path = options.replay.output_dir / "trace.csv";
    replay::writeTextFile(trace_path, trace.str());
    replay::writeTextFile(
        options.replay.output_dir / "status.json",
        jsonText(baselineStatus(loaded, execution, replay_state, config_sha256, error.what())));
    replay::writeTextFile(options.replay.output_dir / "manifest.json",
                          jsonText(baselineManifest(options.replay, loaded, execution,
                                                    mcl::sha256_file(trace_path), config_sha256)));
    visualization_sink->close();
    throw;
  }

  const auto trace_path = options.replay.output_dir / "trace.csv";
  replay::writeTextFile(trace_path, trace.str());
  replay::writeTextFile(options.replay.output_dir / "status.json",
                        jsonText(baselineStatus(loaded, execution, replay_state, config_sha256)));
  replay::writeTextFile(options.replay.output_dir / "manifest.json",
                        jsonText(baselineManifest(options.replay, loaded, execution,
                                                  mcl::sha256_file(trace_path), config_sha256)));
  visualization_sink->flush();
  visualization_sink->close();
  return EXIT_SUCCESS;
}

int runReplay(int argc, char ** argv)
{
  auto options = parseReplayOptions(argc, argv);
  const auto loaded = replay::loadReplay(options.replay);
  baseline::BaselineSolver solver(options.replay.urdf_path.string());
  return runReplayLoop(std::move(options), solver, loaded);
}

int run(int argc, char ** argv)
{
  if (argc < 2 || std::string{argv[1]} == "--help" || std::string{argv[1]} == "-h") {
    printTopLevelUsage(argv[0]);
    return EXIT_SUCCESS;
  }
  const std::string mode{argv[1]};
  if (mode == "teleop") {
    return runTeleop(argc - 1, argv + 1);
  }
  if (mode == "replay") {
    return runReplay(argc - 1, argv + 1);
  }
  throw std::runtime_error("expected subcommand 'teleop' or 'replay'");
}

} // namespace

int main(int argc, char ** argv)
{
  try {
    return run(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << kProgramId << ": " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
