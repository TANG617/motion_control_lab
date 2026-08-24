#include <cstdlib>
#include <iostream>

#include "components/robot/r1/r1_robot_config.hpp"
#include "loop.hpp"
#include "options.hpp"
#include "solver.hpp"

namespace {

namespace app = motion_control_lab::target;

constexpr const char *kProgramId = "mcl_target";

int run(int argc, char **argv) {
  const auto options = app::parseOptions(argc, argv);
  const auto &robot = motion_control_lab::r1RobotConfig();
  if (options.solver == app::SolverKind::Mcc) {
    app::MccTargetSolver solver(options.interactive.urdf_path, robot,
                                options.backend, options.algorithm);
    return app::runLoop(options, "mcc", app::mccSolverTitle(options.backend),
                        solver);
  }
  app::PlacoTargetSolver solver(options.interactive.urdf_path, robot,
                                options.algorithm);
  return app::runLoop(options, "placo", "PlaCo/eiquadprog", solver);
}

} // namespace

int main(int argc, char **argv) {
  try {
    return run(argc, argv);
  } catch (const std::exception &error) {
    std::cerr << kProgramId << ": " << error.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << kProgramId << ": non-standard exception\n";
    return EXIT_FAILURE;
  }
}
