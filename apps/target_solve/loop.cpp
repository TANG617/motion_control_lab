#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "components/app_helpers/app_helpers.hpp"
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

namespace {

namespace mcl = motion_control_lab;

using mcl::target_solve::AppOptions;
using mcl::target_solve::MccTargetSolver;
using mcl::target_solve::PlacoTargetSolver;

constexpr const char *kProgramId = "mcl_target_solve";
constexpr const char *kTitle = "Dual-arm IK — TargetSolve";
constexpr std::array<unsigned int, 1> kMainCpuAffinity{8};

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
      if (result.accepted) {
        ++run_counters.accepted;
      } else {
        ++run_counters.rejected;
      }
      solve_time_percentiles.record(result.solve_time_ms);
      result.solver_debug.ik_solve_time_percentiles =
          solve_time_percentiles.snapshot();
      result.solver_debug.run_counters = run_counters;

      latest_frame.targets = input.targets();
      latest_frame.ik_status = result.ik_status;
      latest_frame.iterations = result.iterations;
      latest_frame.converged = result.converged;
      latest_frame.solve_time_ms = result.solve_time_ms;
      latest_frame.solvers = {std::move(result.solver_debug)};
      latest_frame.target_errors = std::move(result.target_errors);
      latest_frame.status = result.status;
      if (result.accepted) {
        latest_frame.forward_kinematics = std::move(result.forward_kinematics);
        latest_frame.positions = std::move(result.positions);
        latest_frame.velocities = std::move(result.velocities);
      }
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

} // namespace

namespace motion_control_lab::target_solve {

int runLoop(const AppOptions &options, const std::string &solver_id,
            const std::string &solver_title, MccTargetSolver &solver) {
  return runInteractive(options, solver_id, solver_title, solver);
}

int runLoop(const AppOptions &options, const std::string &solver_id,
            const std::string &solver_title, PlacoTargetSolver &solver) {
  return runInteractive(options, solver_id, solver_title, solver);
}

} // namespace motion_control_lab::target_solve
