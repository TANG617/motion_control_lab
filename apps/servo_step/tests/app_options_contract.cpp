#include "../app_options.hpp"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace app = motion_control_lab::servo_step;

int main() {
  char program[] = "mcl_servo_step";
  char urdf_option[] = "--urdf";
  char urdf[] = "/tmp/robot.urdf";
  char *defaults_argv[]{program, urdf_option, urdf};
  const auto defaults = app::parseAppOptions(3, defaults_argv);
  if (defaults.interactive.rate_hz != 100.0 ||
      defaults.solver != app::SolverKind::Mcc ||
      defaults.backend != app::MccBackend::Proxqp) {
    return EXIT_FAILURE;
  }

  char solver_option[] = "--solver";
  char solver[] = "placo";
  char backend_option[] = "--backend";
  char backend[] = "eiquadprog";
  char rate_option[] = "--rate";
  char rate[] = "50";
  char regularization_option[] = "--regularization";
  char regularization[] = "2e-4";
  char *custom_argv[]{program,       urdf_option, urdf,
                      solver_option, solver,      backend_option,
                      backend,       rate_option, rate, regularization_option,
                      regularization};
  const auto custom = app::parseAppOptions(11, custom_argv);
  if (custom.solver != app::SolverKind::Placo ||
      custom.backend != app::MccBackend::Eiquadprog ||
      custom.interactive.rate_hz != 50.0 || custom.algorithm.regularization != 2.0e-4) {
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
  char *missing_argv[]{program, urdf_option, urdf, solver_option};
  char bad_backend[] = "bad";
  char *bad_backend_argv[]{program, urdf_option, urdf, backend_option,
                           bad_backend};
  char ui_option[] = "--ui";
  char bad_ui[] = "bad";
  char *bad_ui_argv[]{program, urdf_option, urdf, ui_option, bad_ui};
  if (!expectFailure(4, unknown_argv) || !expectFailure(4, missing_argv) ||
      !expectFailure(5, bad_backend_argv) || !expectFailure(5, bad_ui_argv)) {
    return EXIT_FAILURE;
  }

  std::ostringstream help;
  auto *original = std::cout.rdbuf(help.rdbuf());
  app::printTeleopUsage(program);
  std::cout.rdbuf(original);
  return help.str().find("MOTION_CONTROL_URDF") != std::string::npos &&
                 help.str().find("default: 100") != std::string::npos
             ? EXIT_SUCCESS
             : EXIT_FAILURE;
}
