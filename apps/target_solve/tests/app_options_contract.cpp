#include "../options.hpp"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace app = motion_control_lab::target_solve;

int main() {
  char program[] = "mcl_target_solve";
  char urdf_option[] = "--urdf";
  char urdf[] = "/tmp/robot.urdf";
  char *defaults_argv[]{program, urdf_option, urdf};
  const auto defaults = app::parseOptions(3, defaults_argv);
  if (defaults.interactive.rate_hz != 100.0 ||
      defaults.solver != app::SolverKind::Mcc ||
      defaults.backend != app::MccBackend::Proxqp) {
    return EXIT_FAILURE;
  }

  char solver_option[] = "--solver";
  char solver[] = "placo";
  char backend_option[] = "--backend";
  char backend[] = "eiquadprog";
  char ui_option[] = "--ui";
  char ui[] = "none";
  char iterations_option[] = "--maximum-iterations";
  char iterations[] = "42";
  char *custom_argv[]{program,       urdf_option, urdf,
                      solver_option, solver,      backend_option,
                      backend,       ui_option,   ui, iterations_option,
                      iterations};
  const auto custom = app::parseOptions(11, custom_argv);
  if (custom.solver != app::SolverKind::Placo ||
      custom.backend != app::MccBackend::Eiquadprog ||
      custom.interactive.tui_enabled || custom.algorithm.maximum_iterations != 42) {
    return EXIT_FAILURE;
  }

  auto expectFailure = [&](int argc, char **argv) {
    try {
      (void)app::parseOptions(argc, argv);
      return false;
    } catch (const std::runtime_error &) {
      return true;
    }
  };
  char unknown[] = "--unknown";
  char *unknown_argv[]{program, urdf_option, urdf, unknown};
  char *missing_argv[]{program, urdf_option, urdf, backend_option};
  char bad_solver[] = "bad";
  char *bad_solver_argv[]{program, urdf_option, urdf, solver_option,
                          bad_solver};
  char step_option[] = "--step-m";
  char step[] = "1.0";
  char *bad_step_argv[]{program, urdf_option, urdf, step_option, step};
  if (!expectFailure(4, unknown_argv) || !expectFailure(4, missing_argv) ||
      !expectFailure(5, bad_solver_argv) || !expectFailure(5, bad_step_argv)) {
    return EXIT_FAILURE;
  }

  std::ostringstream help;
  auto *original = std::cout.rdbuf(help.rdbuf());
  app::printUsage(program);
  std::cout.rdbuf(original);
  return help.str().find("MOTION_CONTROL_URDF") != std::string::npos &&
                 help.str().find("default: 100") != std::string::npos
             ? EXIT_SUCCESS
             : EXIT_FAILURE;
}
