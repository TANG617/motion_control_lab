#include "components/teleop/keyboard/keyboard_target_source.hpp"
#include "components/tui/tui_renderer.hpp"
#include "contracts/presentation/tui_document.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <unistd.h>

namespace
{

constexpr std::string_view kExpectedException = "intentional TUI exception";

motion_control_lab::TuiDocument makeDocument(bool fault_hold, const std::string & status)
{
  namespace mcl = motion_control_lab;
  const auto makeSheet = [](const std::string & title, const std::string & marker,
                            std::size_t row_count = 1U, std::size_t column = 0U,
                            std::size_t row = 0U) {
    mcl::TuiSection section;
    section.title = title;
    mcl::TuiTable table;
    table.columns = {{"Metric", mcl::TuiTableAlignment::Left},
                     {"Value", mcl::TuiTableAlignment::Right}};
    for (std::size_t index = 0U; index < row_count; ++index) {
      table.rows.push_back({"row-" + std::to_string(index),
                            index + 1U == row_count ? marker : std::to_string(index)});
    }
    table.style = mcl::TuiTableStyle::Compact;
    section.tables.push_back(std::move(table));
    section.column = column;
    section.row = row;
    section.style = mcl::TuiSectionStyle::Panel;
    return section;
  };
  mcl::TuiDocument document;
  document.title = "FTXUI PTY test";
  document.subtitle = "renderer accepts TuiDocument only";
  document.status = status;
  document.help_lines = {"Keyboard help", "navigation and source input are external"};
  mcl::TuiPage overview;
  overview.title = "Overview";
  overview.column_weights = {3, 2};
  overview.rows = {{{3, 2}, 3}, {{2, 3}, 2}};
  overview.sections = {makeSheet("System", "system-ready", 8U),
                       makeSheet("Solver", "maximum hard violation", 5U, 1U),
                       makeSheet("Joint execution", "overview-bottom-marker", 7U, 0U, 1U),
                       makeSheet("Runtime", "runtime-ready", 5U, 1U, 1U)};
  document.pages.push_back(std::move(overview));

  mcl::TuiPage cartesian;
  cartesian.title = "Cartesian Planning";
  cartesian.sections = {makeSheet("Cartesian targets", "cartesian-bottom-marker", 12U)};
  document.pages.push_back(std::move(cartesian));

  mcl::TuiPage joint_plan;
  joint_plan.title = "Joint Planning";
  joint_plan.sections = {makeSheet("IK to OTG chain", "joint-plan-bottom-marker", 20U)};
  document.pages.push_back(std::move(joint_plan));

  mcl::TuiPage solver;
  solver.title = "Solver and Quadratic Programming";
  solver.sections = {makeSheet("IK calculation percentiles", "solver-bottom-marker", 12U)};
  solver.sections.front().lines = {
    "90th percentile [ms] 95th percentile [ms] 99th percentile [ms] 0.160"};
  document.pages.push_back(std::move(solver));

  mcl::TuiPage joints;
  joints.title = "Joint State";
  joints.sections = {makeSheet("Executed joint state", "joints-bottom-marker", 20U)};
  joints.sections.front().lines = {"head_yaw left_arm_joint7"};
  document.pages.push_back(std::move(joints));

  mcl::TuiPage runtime;
  runtime.title = "Runtime";
  runtime.column_weights = {3, 2};
  runtime.sections = {makeSheet("Worker timing", "runtime-bottom-marker", 10U),
                      makeSheet("Processor affinity", "events-ready", 8U, 1U)};
  runtime.sections.back().lines = {"bound disabled 4101 requested processors "
                                   "effective processors release lateness "
                                   "Non-Quadratic Programming"};
  document.pages.push_back(std::move(runtime));

  mcl::TuiPage events;
  events.title = "Events";
  events.sections = {makeSheet("Current state", "events-bottom-marker", 15U)};
  events.sections.front().lines = {"Attempts Accepted Rejected"};
  document.pages.push_back(std::move(events));
  if (fault_hold) {
    auto & lines = document.pages.front().sections.front().lines;
    lines.insert(lines.end(), {"TARGET REJECTED", "FAULT HOLD", "rejected target revision",
                               "recoverable rejects"});
  }
  return document;
}

int run(bool throw_after_render, bool fault_hold, bool replay_controls, bool replay_start_paused)
{
  namespace mcl = motion_control_lab;
  mcl::CartesianTeleopOptions options;
  options.side = "left";
  options.step_m = 0.005;
  options.min_step_m = 0.001;
  options.max_step_m = 0.5;
  options.rotation_step_deg = 5.0;

  const std::array<int, 3> terminal_flags_before{::fcntl(STDIN_FILENO, F_GETFL, 0),
                                                 ::fcntl(STDOUT_FILENO, F_GETFL, 0),
                                                 ::fcntl(STDERR_FILENO, F_GETFL, 0)};
  mcl::TerminalFrontend terminal({true, true});
  const std::array<int, 3> terminal_flags_after{::fcntl(STDIN_FILENO, F_GETFL, 0),
                                                ::fcntl(STDOUT_FILENO, F_GETFL, 0),
                                                ::fcntl(STDERR_FILENO, F_GETFL, 0)};
  if (terminal_flags_after != terminal_flags_before) {
    throw std::runtime_error("TerminalFrontend changed shared PTY file status flags");
  }
  mcl::KeyboardTargetSource input(
      terminal, replay_controls ? mcl::KeyboardSourceMode::Replay : mcl::KeyboardSourceMode::Teleop,
      options,
      {{mcl::ArmSide::Left, mcl::Pose::Identity()}, {mcl::ArmSide::Right, mcl::Pose::Identity()}},
      true);
  mcl::TuiRenderer tui(true);
  if (replay_controls || fault_hold) {
    input.setMotionInputEnabled(false, fault_hold ? "FAULT HOLD: Red deadline miss"
                                                  : "Replay motion editing is disabled");
  }
  if (replay_start_paused) {
    input.setPaused(true, "Replay timeline paused; press space to start");
  }

  auto document = makeDocument(fault_hold, input.status());
  tui.render(document);
  if (throw_after_render) {
    throw std::runtime_error(std::string{kExpectedException});
  }

  bool single_step_requested = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!input.stopRequested() && std::chrono::steady_clock::now() < deadline) {
    const auto update = input.poll(0.01);
    for (const auto & event : update.navigation) {
      tui.handleNavigation(event);
    }
    for (const auto control : input.consumeSourceControls()) {
      if (control == mcl::SourceControl::Step) {
        single_step_requested = true;
      }
    }
    document.status = input.status();
    tui.render(document);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const bool expected_paused = !fault_hold;
  if (!input.stopRequested() || input.selectedSide() != mcl::ArmSide::Right ||
      input.targets().size() != 2U || input.paused() != expected_paused ||
      single_step_requested != replay_controls) {
    std::cerr << "final input state: stop=" << input.stopRequested()
              << " side=" << mcl::armSideName(input.selectedSide())
              << " targets=" << input.targets().size() << " paused=" << input.paused()
              << " step=" << single_step_requested << '\n';
    return EXIT_FAILURE;
  }

  const auto & left = input.targets()[0].target_pose.translation();
  const auto & right = input.targets()[1].target_pose.translation();
  constexpr double kTolerance = 1e-12;
  const bool motion_disabled = fault_hold || replay_controls;
  const double expected_left_x = motion_disabled ? 0.0 : 0.005;
  const double expected_right_y = motion_disabled ? 0.0 : -0.010;
  const double expected_right_z = motion_disabled ? 0.0 : 0.020;
  if (std::abs(left.x() - expected_left_x) > kTolerance ||
      std::abs(right.y() - expected_right_y) > kTolerance ||
      std::abs(right.z() - expected_right_z) > kTolerance) {
    std::cerr << "final target: left_x=" << left.x() << " right_y=" << right.y()
              << " right_z=" << right.z() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char ** argv)
{
  const bool throw_after_render = argc == 2 && std::string_view(argv[1]) == "--throw-after-render";
  const bool fault_hold = argc == 2 && std::string_view(argv[1]) == "--fault-hold";
  const bool replay_controls = argc == 2 && (std::string_view(argv[1]) == "--replay" ||
                                             std::string_view(argv[1]) == "--replay-start-paused");
  const bool replay_start_paused =
      argc == 2 && std::string_view(argv[1]) == "--replay-start-paused";
  try {
    return run(throw_after_render, fault_hold, replay_controls, replay_start_paused);
  } catch (const std::exception & error) {
    std::cerr << "test_tui_console: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
