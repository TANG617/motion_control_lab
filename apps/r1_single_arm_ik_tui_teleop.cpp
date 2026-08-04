#include "config/r1_ik_options.hpp"
#include "runtime/interactive_runner.hpp"
#include "runtime/r1_ik_solver_session.hpp"
#include "sinks/visualization_sink_factory.hpp"
#include "teleop/tui_teleop_source.hpp"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{

std::atomic_bool stop_requested{false};

void handleSignal(int)
{
  stop_requested.store(true);
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = motion_control_lab::parseR1IkOptions(argc, argv);
    if (!std::filesystem::exists(options.urdf_path)) {
      throw std::runtime_error("URDF does not exist: " + options.urdf_path);
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    motion_control_lab::R1IkSolverSession solver_session(options.urdf_path);
    const auto side = motion_control_lab::parseArmSide(options.side);
    std::vector<motion_control_lab::ArmTarget> initial_targets{
      motion_control_lab::ArmTarget{side, solver_session.currentTargetPose(side)}};

    motion_control_lab::TuiTeleopSource teleop_source(
      options,
      std::move(initial_targets),
      false);
    auto visualization_sink = motion_control_lab::createVisualizationSink(options);
    visualization_sink->open({"interactive-preview", "r1_single_arm_ik_tui_teleop"});

    motion_control_lab::InteractiveRunner runner(
      motion_control_lab::InteractiveRunnerOptions{
        options.rate_hz,
        options.duration_s,
        []() { return stop_requested.load(); }},
      solver_session,
      teleop_source,
      *visualization_sink);
    runner.run();
    visualization_sink->flush();
    visualization_sink->close();

    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "r1_single_arm_ik_tui_teleop: " << error.what() << "\n";
    return EXIT_FAILURE;
  }
}
