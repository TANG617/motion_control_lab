#include "cartesian_planning.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace
{

namespace mcc = motion_control::core;
namespace mcp = motion_control_lab::cartesian_planning;

void requireOk(const mcc::Status & status)
{
  if (!status.ok()) {
    throw std::runtime_error(status.message);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  const auto options = mcp::parseAppOptions(argc, argv);
  const auto request = mcp::loadRequest(options.request_path);

  mcc::CartesianPlanner planner;
  mcc::CartesianTrajectory trajectory;
  mcc::PlanningDiagnostics diagnostics;
  requireOk(planner.generate(request, trajectory, diagnostics));

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
}
