#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "components/app_helpers/app_helpers.hpp"
#include "components/replay/replay_source.hpp"
#include "components/robot/r1/r1_robot_config.hpp"
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

namespace {

namespace mcl = motion_control_lab;
namespace replay = motion_control_lab::replay;

using mcl::servo_step::AppOptions;
using mcl::servo_step::MccBackend;
using mcl::servo_step::MccServoSolver;
using mcl::servo_step::PlacoServoSolver;
using mcl::servo_step::ReplayAppOptions;

constexpr const char *kProgramId = "mcl_servo_step";
constexpr const char *kTitle = "Dual-arm IK — ServoStep";
constexpr std::array<unsigned int, 1> kMainCpuAffinity{31};

template <typename Solver>
int runInteractive(const AppOptions &app_options, const std::string &solver_id,
                   const std::string &solver_title, Solver &solver) {
  const auto &options = app_options.interactive;
  const auto &robot = mcl::r1RobotConfig();
  const auto affinity_domain = mcl::CpuAffinityDomain::capture();
  const auto affinity_binding =
      affinity_domain.bindCurrentThread(kProgramId, "main", kMainCpuAffinity);

  const auto presentation =
      mcl::makeArmPresentation(robot, mcl::foxgloveIkVisualizationChannels());
  const auto initial_left_fk = solver.currentPose(mcl::ArmSide::Left);
  const auto initial_right_fk = solver.currentPose(mcl::ArmSide::Right);
  const std::string title = std::string{kTitle} + " [" + solver_title + "]";
  mcl::TerminalFrontend terminal({true, options.tui_enabled});
  mcl::KeyboardTargetSource input(terminal, mcl::KeyboardSourceMode::Teleop,
                                  options.tui,
                                  {{mcl::ArmSide::Left, initial_left_fk},
                                   {mcl::ArmSide::Right, initial_right_fk}},
                                  true);
  mcl::TuiRenderer tui(options.tui_enabled);
  auto visualization_sink =
      mcl::createPreviewSink(options.visualization, kProgramId);

  mcl::installRuntimeSignalHandlers();
  mcl::SingleRateScheduler scheduler({options.rate_hz, options.duration_s});
  mcl::RollingPercentiles solve_time_percentiles;
  mcl::SolverRunCounters run_counters;
  std::size_t publish_count = 0;
  const std::string run_id = "interactive-preview-" + solver_id;

  mcl::IkDebugFrame latest_frame;
  latest_frame.run_id = run_id;
  latest_frame.targets = input.targets();
  latest_frame.forward_kinematics = {{mcl::ArmSide::Left, initial_left_fk},
                                     {mcl::ArmSide::Right, initial_right_fk}};
  latest_frame.joint_names = robot.joint_names;
  latest_frame.positions = solver.positions();
  latest_frame.velocities = solver.velocities();
  latest_frame.selected_side = mcl::parseArmSide(options.tui.side);
  latest_frame.cpu_affinities = {mcl::makeCpuAffinityDebug(affinity_binding)};

  visualization_sink->open();
  while (const auto schedule = scheduler.next()) {
    const auto input_update = input.poll(schedule->dt);
    for (const auto &event : input_update.navigation) {
      tui.handleNavigation(event);
    }
    if (const auto reset_side = input.consumeResetRequest()) {
      input.setTargetPose(*reset_side, solver.currentPose(*reset_side),
                          std::string{"Reset "} +
                              mcl::armSideName(*reset_side) +
                              " target from current FK");
    }

    if (input.stopRequested()) {
      break;
    }

    if (schedule->update_due && !input.paused()) {
      ++run_counters.attempts;
      auto result = solver.solve(input.targets());
      ++run_counters.accepted;
      solve_time_percentiles.record(result.solve_time_ms);
      result.solver_debug.ik_solve_time_percentiles =
          solve_time_percentiles.snapshot();
      result.solver_debug.run_counters = run_counters;

      latest_frame.targets = input.targets();
      latest_frame.forward_kinematics = std::move(result.forward_kinematics);
      latest_frame.positions = std::move(result.positions);
      latest_frame.velocities = std::move(result.velocities);
      latest_frame.ik_status = "ok";
      latest_frame.iterations = result.iterations;
      latest_frame.converged = result.converged;
      latest_frame.solve_time_ms = result.solve_time_ms;
      latest_frame.solvers = {std::move(result.solver_debug)};
      latest_frame.target_errors = std::move(result.target_errors);
      latest_frame.status = "IK accepted [" + solver_id + "]";
      latest_frame.paused = input.paused();
      latest_frame.selected_side = input.selectedSide();
      visualization_sink->write(mcl::makeIkRenderBatch(
          latest_frame, presentation, schedule->emit_time_ns));
      ++publish_count;
    }

    if (schedule->draw_due) {
      latest_frame.paused = input.paused();
      latest_frame.selected_side = input.selectedSide();
      tui.render(mcl::makeStandardIkTuiDocument(
          latest_frame, presentation, publish_count,
          visualization_sink->status(), title, input.status()));
    }
    scheduler.sleep();
  }

  visualization_sink->flush();
  visualization_sink->close();
  return EXIT_SUCCESS;
}

std::pair<std::vector<double>, std::vector<double>>
replayInitialState(const replay::LoadedReplay &loaded,
                   const mcl::R1RobotConfig &robot) {
  if (!loaded.initial_joint_state.has_value()) {
    return {robot.default_positions,
            std::vector<double>(robot.joint_names.size(), 0.0)};
  }
  const auto &source = *loaded.initial_joint_state;
  std::vector<double> positions(robot.joint_names.size(), 0.0);
  std::vector<double> velocities(robot.joint_names.size(), 0.0);
  for (std::size_t index = 0; index < robot.joint_names.size(); ++index) {
    const auto iterator = std::find(source.names.begin(), source.names.end(),
                                    robot.joint_names[index]);
    if (iterator == source.names.end()) {
      throw std::runtime_error("initial JointState is missing " +
                               robot.joint_names[index]);
    }
    const std::size_t source_index =
        static_cast<std::size_t>(std::distance(source.names.begin(), iterator));
    positions[index] = source.positions.at(source_index);
    // Replay starts a fresh accepted-state feedback chain; recorded velocity is
    // provenance only.
    velocities[index] = 0.0;
  }
  return {std::move(positions), std::move(velocities)};
}

std::string jsonText(const Json::Value &value) {
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "  ";
  return Json::writeString(builder, value) + "\n";
}

std::string traceVector(const std::vector<double> &values) {
  std::ostringstream output;
  output << '"' << std::setprecision(17);
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {
      output << ';';
    }
    output << values[index];
  }
  output << '"';
  return output.str();
}

template <typename Solver>
int runReplayWithSolver(ReplayAppOptions options, const std::string &solver_id,
                        const std::string &solver_title, Solver &solver,
                        const replay::LoadedReplay &loaded) {
  if (loaded.timeline.timeline.empty()) {
    throw std::runtime_error("replay timeline is empty");
  }
  if (!options.replay.output_dir_explicit) {
    const std::string run_id = options.replay.run_id.value_or(
        mcl::make_run_id(mcl::sha256_file(options.replay.input_path)));
    const std::filesystem::path output_root =
        options.replay.output_root.value_or(
            std::filesystem::path{"experiments/E02_dual_arm_replay_ik/runs"});
    options.replay.output_dir = output_root / run_id;
  }
  replay::createOutputDirectory(options.replay.output_dir);

  const auto &robot = mcl::r1RobotConfig();
  replay::ReplaySource replay_source(loaded, options.replay.execution_mode,
                                     options.replay.playback_rate);
  const auto &first = replay_source.sourceFrame();
  std::vector<mcl::ArmTarget> targets{
      {mcl::ArmSide::Left,
       first.value.left.pose * robot.left_tcp_offset.inverse()},
      {mcl::ArmSide::Right,
       first.value.right.pose * robot.right_tcp_offset.inverse()}};
  const auto presentation =
      mcl::makeArmPresentation(robot, mcl::foxgloveIkVisualizationChannels());
  const bool tui_enabled = options.replay.ui_mode == "tui";
  const std::string title =
      std::string{kTitle} + " Replay [" + solver_title + "]";
  mcl::TerminalFrontend terminal(
      {options.replay.terminal_input_enabled, tui_enabled});
  mcl::KeyRouter key_router;
  mcl::KeyboardTeleop keyboard(mcl::KeyboardSourceMode::Replay);
  mcl::TuiRenderer tui(tui_enabled);

  mcl::PreviewSinkOptions sink_options;
  sink_options.enabled = options.replay.visualization_enabled;
  sink_options.host = options.replay.visualization_host;
  sink_options.port = options.replay.visualization_port;
  sink_options.mcap_path = options.replay.visualization_mcap_path;
  auto visualization_sink = mcl::createPreviewSink(sink_options, kProgramId);
  visualization_sink->open();
  mcl::installRuntimeSignalHandlers();

  replay::ReplayExecutionMetadata execution;
  execution.app = kProgramId;
  execution.topology = "ordinary-servo-step";
  execution.solver = solver_id;
  execution.backend =
      solver_id == "mcc"
          ? (options.backend == MccBackend::Proxqp ? "proxqp" : "eiquadprog")
          : "eiquadprog";
  execution.rate_hz = options.rate_hz;
  execution.consumed_frame_count = 1U;
  execution.resolved_config = {
      {"regularization", std::to_string(options.algorithm.regularization)},
      {"position_tolerance_m",
       std::to_string(options.algorithm.position_tolerance_m)},
      {"orientation_tolerance_rad",
       std::to_string(options.algorithm.orientation_tolerance_rad)},
      {"joint_position_margin_rad",
       std::to_string(options.algorithm.joint_position_margin_rad)},
  };

  std::ostringstream trace;
  trace << "attempt,source_revision,original_logical_timestamp_ns,source_time_"
           "from_start_ns,"
           "projected_timestamp_ns,left_header_stamp_ns,left_log_time_ns,left_"
           "publish_time_ns,"
           "right_header_stamp_ns,right_log_time_ns,right_publish_time_ns,"
           "accepted,solver_status,"
           "solve_time_ms,maximum_hard_violation,positions,velocities\n";
  const std::int64_t solver_period_ns =
      static_cast<std::int64_t>(std::llround(1.0e9 / options.rate_hz));
  std::size_t attempt = 0;
  std::size_t publish_count = 0;
  std::string replay_state{"running"};

  try {
    while (true) {
      for (const auto &event : terminal.poll()) {
        if (key_router.route(event) == mcl::KeyRoute::Navigation) {
          tui.handleNavigation(event);
        } else {
          const auto action = keyboard.handle(event);
          if (action.source_control.has_value()) {
            replay_source.applyControl(*action.source_control);
          }
        }
      }
      if (replay_source.stopped()) {
        replay_state = "stopped";
        break;
      }

      if (attempt != 0U) {
        replay_source.advance(solver_period_ns);
      }
      replay_source.waitForCurrentFrame();
      execution.deadline_miss_count = replay_source.deadlineMissCount();
      execution.dropped_frame_count = replay_source.droppedFrameCount();
      execution.consumed_frame_count = replay_source.consumedFrameCount();
      const auto &replay_frame = replay_source.frame();
      targets[0].target_pose =
          replay_frame.targets[0].target_pose * robot.left_tcp_offset.inverse();
      targets[1].target_pose = replay_frame.targets[1].target_pose *
                               robot.right_tcp_offset.inverse();

      const auto result = solver.solve(targets);
      ++attempt;
      ++execution.accepted_count;
      const auto &source = replay_source.sourceFrame();
      trace
          << attempt << ',' << source.sequence << ','
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
          << ",true,ok," << result.solve_time_ms << ','
          << result.solver_debug.maximum_hard_violation << ','
          << traceVector(result.positions) << ','
          << traceVector(result.velocities) << '\n';

      mcl::IkDebugFrame frame;
      frame.run_id = options.replay.output_dir.filename().string();
      frame.targets = targets;
      frame.forward_kinematics = result.forward_kinematics;
      frame.joint_names = robot.joint_names;
      frame.positions = result.positions;
      frame.velocities = result.velocities;
      frame.solvers = {result.solver_debug};
      frame.target_errors = result.target_errors;
      frame.iterations = result.iterations;
      frame.converged = result.converged;
      frame.solve_time_ms = result.solve_time_ms;
      frame.ik_status = "replay accepted";
      frame.status =
          "Replay source revision=" + std::to_string(source.sequence) +
          " dropped=" + std::to_string(execution.dropped_frame_count);
      frame.paused = replay_source.paused();
      visualization_sink->write(mcl::makeIkRenderBatch(
          frame, presentation,
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::system_clock::now().time_since_epoch())
              .count()));
      tui.render(mcl::makeStandardIkTuiDocument(
          frame, presentation, publish_count, visualization_sink->status(),
          title, replay_source.status().detail));
      ++publish_count;

      replay_source.markFrameProcessed();
      if (replay_source.endOfStream()) {
        replay_state = "succeeded";
        break;
      }
    }
  } catch (const std::exception &error) {
    const auto &source = replay_source.sourceFrame();
    trace << attempt + 1U << ',' << source.sequence << ','
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
          << ",false," << replay::csvEscape(error.what()) << ",,,"
          << traceVector(solver.positions()) << ','
          << traceVector(solver.velocities()) << '\n';
    ++execution.rejected_count;
    replay_state = "failed";
    const auto trace_path = options.replay.output_dir / "trace.csv";
    replay::writeTextFile(trace_path, trace.str());
    replay::writeTextFile(options.replay.output_dir / "status.json",
                          jsonText(replay::makeReplayStatus(
                              loaded, execution, replay_state, error.what())));
    const auto manifest = replay::makeReplayManifest(
        options.replay, loaded, execution, mcl::sha256_file(trace_path));
    replay::writeTextFile(options.replay.output_dir / "manifest.json",
                          jsonText(manifest));
    visualization_sink->close();
    throw;
  }

  const auto trace_path = options.replay.output_dir / "trace.csv";
  replay::writeTextFile(trace_path, trace.str());
  replay::writeTextFile(
      options.replay.output_dir / "status.json",
      jsonText(replay::makeReplayStatus(loaded, execution, replay_state)));
  const auto manifest = replay::makeReplayManifest(
      options.replay, loaded, execution, mcl::sha256_file(trace_path));
  replay::writeTextFile(options.replay.output_dir / "manifest.json",
                        jsonText(manifest));
  visualization_sink->flush();
  visualization_sink->close();
  return EXIT_SUCCESS;
}

} // namespace

namespace motion_control_lab::servo_step {

std::pair<std::vector<double>, std::vector<double>>
makeReplayInitialState(const replay::LoadedReplay &loaded,
                       const R1RobotConfig &robot) {
  return replayInitialState(loaded, robot);
}

int runLoop(const AppOptions &options, const std::string &solver_id,
            const std::string &solver_title, MccServoSolver &solver) {
  return runInteractive(options, solver_id, solver_title, solver);
}

int runLoop(const AppOptions &options, const std::string &solver_id,
            const std::string &solver_title, PlacoServoSolver &solver) {
  return runInteractive(options, solver_id, solver_title, solver);
}

int runReplayLoop(ReplayAppOptions options, const std::string &solver_id,
                  const std::string &solver_title, MccServoSolver &solver,
                  const replay::LoadedReplay &loaded) {
  return runReplayWithSolver(std::move(options), solver_id, solver_title,
                             solver, loaded);
}

int runReplayLoop(ReplayAppOptions options, const std::string &solver_id,
                  const std::string &solver_title, PlacoServoSolver &solver,
                  const replay::LoadedReplay &loaded) {
  return runReplayWithSolver(std::move(options), solver_id, solver_title,
                             solver, loaded);
}

} // namespace motion_control_lab::servo_step
