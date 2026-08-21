#include "options.hpp"
#include "planning.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace {

namespace app = motion_control_lab::plot_core_planning;
namespace mcc = motion_control::core;

void requireOk(const mcc::Status &status) {
  if (!status.ok()) {
    throw std::runtime_error(status.message);
  }
}

} // namespace

int main(int argc, char **argv) {
  const auto options = app::parseOptions(argc, argv);
  std::filesystem::create_directories(options.output_dir);
  app::initializePlotBackend();

  mcc::CartesianPlanner cartesian_planner;
  mcc::CartesianTrajectory cartesian_trajectory;
  mcc::PlanningDiagnostics cartesian_diagnostics;
  requireOk(cartesian_planner.generate(app::makeCartesianRequest(),
                                       cartesian_trajectory,
                                       cartesian_diagnostics));

  mcc::JointPlannerConfig joint_config;
  joint_config.algorithm = mcc::JointTrajectoryAlgorithm::JerkLimited;
  joint_config.synchronization = mcc::TrajectorySynchronization::Phase;
  mcc::JointPlanner joint_planner(joint_config);
  mcc::JointTrajectory joint_trajectory;
  mcc::PlanningDiagnostics joint_diagnostics;
  requireOk(joint_planner.generate(app::makeJointRequest(), joint_trajectory,
                                   joint_diagnostics));

  const auto cartesian_path =
      options.output_dir / "cartesian_move_line_planning.png";
  const auto joint_path = options.output_dir / "joint_trajectory_planning.png";
  app::saveCartesianPlot(cartesian_trajectory, cartesian_path);
  app::saveJointPlot(joint_trajectory, joint_path);

  std::cout << "Wrote " << std::filesystem::absolute(cartesian_path) << '\n'
            << "Wrote " << std::filesystem::absolute(joint_path) << '\n';
  return EXIT_SUCCESS;
}
