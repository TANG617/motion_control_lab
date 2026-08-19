#include "../app_options.hpp"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace app = motion_control_lab::baseline;

int main() {
  char program[] = "mcl_baseline";
  char urdf_option[] = "--urdf";
  char urdf[] = "/tmp/robot.urdf";
  char *defaults_argv[]{program, urdf_option, urdf};
  const auto defaults = app::parseTeleopOptions(3, defaults_argv);
  if (defaults.rate_hz != 100.0 || defaults.duration_s != 0.0 ||
      !defaults.tui_enabled || defaults.tui.step_m != 0.005 ||
      defaults.visualization.port != 8765) {
    return EXIT_FAILURE;
  }

  char ui_option[] = "--ui";
  char ui[] = "none";
  char side_option[] = "--side";
  char side[] = "right";
  char *custom_argv[]{program, urdf_option, urdf, ui_option,
                      ui,      side_option, side};
  const auto custom = app::parseTeleopOptions(7, custom_argv);
  if (custom.tui_enabled || custom.tui.side != "right") {
    return EXIT_FAILURE;
  }

  auto expectFailure = [&](int argc, char **argv) {
    try {
      (void)app::parseTeleopOptions(argc, argv);
      return false;
    } catch (const std::runtime_error &) {
      return true;
    }
  };
  char rate_option[] = "--rate";
  char rate[] = "50";
  char *rate_argv[]{program, urdf_option, urdf, rate_option, rate};
  char unknown[] = "--unknown";
  char *unknown_argv[]{program, urdf_option, urdf, unknown};
  char *missing_argv[]{program, urdf_option, urdf, side_option};
  char bad_ui[] = "bad";
  char *bad_ui_argv[]{program, urdf_option, urdf, ui_option, bad_ui};
  if (!expectFailure(5, rate_argv) || !expectFailure(4, unknown_argv) ||
      !expectFailure(4, missing_argv) || !expectFailure(5, bad_ui_argv)) {
    return EXIT_FAILURE;
  }

  std::ostringstream help;
  auto *original = std::cout.rdbuf(help.rdbuf());
  app::printTeleopUsage(program);
  std::cout.rdbuf(original);
  return help.str().find("MOTION_CONTROL_URDF") != std::string::npos &&
                 help.str().find("fixed at 100") != std::string::npos
             ? EXIT_SUCCESS
             : EXIT_FAILURE;
}
