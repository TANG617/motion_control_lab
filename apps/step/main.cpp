#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "adapters/replay/replay_support.hpp"
#include "components/robot/r1/r1_robot_config.hpp"
#include "loop.hpp"
#include "options.hpp"
#include "solver.hpp"

namespace {

namespace app = motion_control_lab::step;
namespace replay = motion_control_lab::replay;

constexpr const char *kProgramId = "mcl_step";

int runTeleop(int argc, char **argv) {
  const auto options = app::parseOptions(argc, argv);
  const auto &robot = motion_control_lab::r1RobotConfig();
  if (options.solver == app::SolverKind::Mcc) {
    const std::string solver_title = app::mccSolverTitle(options.backend);
    app::MccServoSolver solver(options.interactive.urdf_path,
                               options.interactive.rate_hz, robot,
                               options.backend, options.algorithm);
    return app::runLoop(options, "mcc", solver_title, solver);
  }
  app::PlacoServoSolver solver(options.interactive.urdf_path,
                               options.interactive.rate_hz, robot,
                               options.algorithm);
  return app::runLoop(options, "placo", "PlaCo/eiquadprog", solver);
}

int runReplay(int argc, char **argv, int process_argc, char **process_argv) {
  auto options = app::parseReplayOptions(argc, argv);
  options.replay.original_argv.assign(process_argv,
                                      process_argv + process_argc);
  const auto loaded = replay::loadReplay(options.replay);
  const auto &robot = motion_control_lab::r1RobotConfig();
  const auto [initial_positions, initial_velocities] =
      app::makeReplayInitialState(loaded, robot);
  if (options.solver == app::SolverKind::Mcc) {
    const std::string solver_title = app::mccSolverTitle(options.backend);
    app::MccServoSolver solver(options.replay.urdf_path.string(),
                               options.rate_hz, robot, options.backend,
                               options.algorithm, initial_positions,
                               initial_velocities);
    return app::runReplayLoop(std::move(options), "mcc", solver_title, solver,
                              loaded);
  }
  app::PlacoServoSolver solver(options.replay.urdf_path.string(),
                               options.rate_hz, robot, options.algorithm,
                               initial_positions, initial_velocities);
  return app::runReplayLoop(std::move(options), "placo", "PlaCo/eiquadprog",
                            solver, loaded);
}

int run(int argc, char **argv) {
  if (argc < 2 || std::string{argv[1]} == "--help" ||
      std::string{argv[1]} == "-h") {
    app::printTopLevelUsage(argv[0]);
    return EXIT_SUCCESS;
  }
  const std::string mode{argv[1]};
  if (mode == "teleop") {
    return runTeleop(argc - 1, argv + 1);
  }
  if (mode == "replay") {
    return runReplay(argc - 1, argv + 1, argc, argv);
  }
  throw std::runtime_error("expected subcommand 'teleop' or 'replay'");
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
