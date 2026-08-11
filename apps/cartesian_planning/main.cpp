#include "cartesian_planning.hpp"

#include <motion_control_core/motion_control_core.hpp>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace
{

namespace mcc = motion_control::core;
namespace mcp = motion_control_lab::cartesian_planning;

void requireOk(const mcc::Status & status, const std::string & action)
{
  if (!status.ok()) {
    throw std::runtime_error(action + ": " + status.message);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = mcp::parseAppOptions(argc, argv);
    if (options.mcap_path.has_value() && std::filesystem::exists(*options.mcap_path)) {
      throw std::runtime_error(
              "refusing to overwrite MCAP: " + options.mcap_path->string());
    }
    const auto request = mcp::loadRequest(options.request_path);

    mcc::CartesianMoveLinePlanner planner;
    mcc::CartesianTrajectory trajectory;
    mcc::PlanningDiagnostics diagnostics;
    requireOk(
      planner.generate(request, trajectory, diagnostics),
      "Cartesian MoveLine planning failed");

    const auto outputs = mcp::prepareOutputPaths(options.output_dir, options.force);
    mcp::writeTrajectoryCsv(outputs.trajectory_csv, trajectory);
    mcp::renderTrajectoryPlots(outputs, trajectory);

    std::cout << "Cartesian MoveLine planning succeeded\n"
              << "  duration: " << trajectory.duration << " s\n"
              << "  samples: " << diagnostics.sample_count << '\n'
              << "  calculation: " << diagnostics.calculation_time_ms << " ms\n"
              << "  CSV: " << std::filesystem::absolute(outputs.trajectory_csv) << '\n'
              << "  path plot: " << std::filesystem::absolute(outputs.path_plot) << '\n'
              << "  profiles plot: " << std::filesystem::absolute(outputs.profiles_plot) << '\n';

    if (options.live) {
      std::cout << "  Foxglove: ws://" << options.host << ':' << options.port << '\n'
                << (options.once ? "Playing once\n" : "Looping until Ctrl-C\n");
      mcp::playTrajectory(options, request, trajectory);
    }
    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "mcl_cartesian_planning: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
