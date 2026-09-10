#include "options.hpp"
#include "loop.hpp"
#include <cstdlib>
#include <exception>
#include <iostream>

int main(int argc, char ** argv) {
  namespace app = motion_control_lab::psibot_teleop;
  try {
    const auto options = app::parseOptions(argc, argv);
    if (options.help) {
      std::cout << app::helpText();
      return EXIT_SUCCESS;
    }
    return app::runLoop(options);
  } catch (const std::exception & error) {
    // The loop has unwound and restored the terminal; never continue on failure.
    std::cerr << "mcl_psibot_teleop: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
