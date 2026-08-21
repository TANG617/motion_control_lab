#include "loop.hpp"
#include "options.hpp"

#include <cstdlib>
#include <exception>
#include <iostream>

int main(int argc, char **argv) {
  namespace replay = motion_control_lab::replay;
  namespace app = motion_control_lab::replay_plan;
  try {
    auto options = app::parseOptions(argc, argv);
    if (options.help) {
      std::cout << replay::replayHelp(argv[0], false);
      return EXIT_SUCCESS;
    }
    app::runLoop(options);
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    std::cerr << "mcl_replay_plan: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
