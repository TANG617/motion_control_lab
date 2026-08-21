#include "../app_options.hpp"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace app = motion_control_lab::single_arm_servo_step;

int main() {
  char program[] = "mcl_single_arm_servo_step";
  char urdf_option[] = "--urdf";
  char urdf[] = "/tmp/robot.urdf";
  char *defaults_argv[]{program, urdf_option, urdf};
  const auto defaults = app::parseAppOptions(3, defaults_argv);
  if (defaults.rate_hz != 100.0 || defaults.duration_s != 0.0 ||
      !defaults.tui_enabled || defaults.tui.step_m != 0.005 ||
      defaults.tui.min_step_m != 0.001 || defaults.tui.max_step_m != 0.5 ||
      defaults.visualization.port != 8765) {
    return EXIT_FAILURE;
  }

  char rate_option[] = "--rate";
  char rate[] = "50";
  char ui_option[] = "--ui";
  char ui[] = "none";
  char side_option[] = "--side";
  char side[] = "right";
  char regularization_option[] = "--regularization";
  char regularization[] = "2e-8";
  char *custom_argv[]{program,   urdf_option, urdf,        rate_option, rate,
                      ui_option, ui,          side_option, side,
                      regularization_option, regularization};
  const auto custom = app::parseAppOptions(11, custom_argv);
  if (custom.rate_hz != 50.0 || custom.tui_enabled ||
      custom.tui.side != "right" || custom.regularization != 2.0e-8) {
    return EXIT_FAILURE;
  }

  auto expectFailure = [&](int argc, char **argv) {
    try {
      (void)app::parseAppOptions(argc, argv);
      return false;
    } catch (const std::runtime_error &) {
      return true;
    }
  };
  char unknown[] = "--unknown";
  char *unknown_argv[]{program, urdf_option, urdf, unknown};
  char *missing_argv[]{program, urdf_option, urdf, rate_option};
  char bad_ui[] = "invalid";
  char *bad_ui_argv[]{program, urdf_option, urdf, ui_option, bad_ui};
  char min_option[] = "--min-step-m";
  char min_value[] = "0.01";
  char max_option[] = "--max-step-m";
  char max_value[] = "0.001";
  char *bad_range_argv[]{program,   urdf_option, urdf,     min_option,
                         min_value, max_option,  max_value};
  if (!expectFailure(4, unknown_argv) || !expectFailure(4, missing_argv) ||
      !expectFailure(5, bad_ui_argv) || !expectFailure(7, bad_range_argv)) {
    return EXIT_FAILURE;
  }

  std::ostringstream help;
  auto *original = std::cout.rdbuf(help.rdbuf());
  app::printUsage(program);
  std::cout.rdbuf(original);
  return help.str().find("MOTION_CONTROL_URDF") != std::string::npos &&
                 help.str().find("default: 100") != std::string::npos &&
                 help.str().find("default: 0.005") != std::string::npos
             ? EXIT_SUCCESS
             : EXIT_FAILURE;
}
