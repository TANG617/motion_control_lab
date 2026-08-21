#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "adapters/replay/replay_support.hpp"
#include "loop.hpp"
#include "options.hpp"
#include "solver.hpp"

namespace {

namespace app = motion_control_lab::baseline;
namespace replay = motion_control_lab::replay;

constexpr const char *kProgramId = "mcl_baseline";

int runTeleop(int argc, char **argv) {
  const auto options = app::parseTeleopOptions(argc, argv);
  app::BaselineSolver solver(options.urdf_path);
  return app::runLoop(options, solver);
}

int runReplay(int argc, char **argv, int process_argc, char **process_argv) {
  auto options = app::parseReplayOptions(argc, argv);
  options.replay.original_argv.assign(process_argv,
                                      process_argv + process_argc);
  const auto loaded = replay::loadReplay(options.replay);
  app::BaselineSolver solver(options.replay.urdf_path.string());
  return app::runReplayLoop(std::move(options), solver, loaded);
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
  }
}
