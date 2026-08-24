#include "../options.hpp"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace app = motion_control_lab::planned_hierarchical_step;

int main() {
  char program[] = "mcl_planned_hierarchical_step";
  char urdf_option[] = "--urdf";
  char urdf[] = "/tmp/robot.urdf";
  char *defaults_argv[]{program, urdf_option, urdf};
  const auto defaults = app::parseHierarchicalOptions(3, defaults_argv);
  if (defaults.red_rate_hz != 100.0 || defaults.yellow_rate_hz != 20.0 ||
      defaults.ui_rate_hz != 100.0 ||
      defaults.deadline_policy != motion_control_lab::DeadlinePolicy::Strict) {
    return EXIT_FAILURE;
  }

  char red_option[] = "--red-rate";
  char red[] = "200";
  char yellow_option[] = "--yellow-rate";
  char yellow[] = "40";
  char ui_option[] = "--ui";
  char ui[] = "none";
  char tolerance_option[] = "--red-proxqp-absolute-tolerance";
  char tolerance[] = "3e-5";
  char *custom_argv[]{program,       urdf_option, urdf,      red_option, red,
                      yellow_option, yellow,      ui_option, ui,
                      tolerance_option, tolerance};
  const auto custom = app::parseHierarchicalOptions(11, custom_argv);
  if (custom.red_rate_hz != 200.0 || custom.yellow_rate_hz != 40.0 ||
      custom.presentation.enabled || custom.solver.red_proxqp_absolute_tolerance != 3.0e-5) {
    return EXIT_FAILURE;
  }

  auto expectFailure = [&](int argc, char **argv) {
    try {
      (void)app::parseHierarchicalOptions(argc, argv);
      return false;
    } catch (const std::runtime_error &) {
      return true;
    }
  };
  char unknown[] = "--unknown";
  char *unknown_argv[]{program, urdf_option, urdf, unknown};
  char *missing_argv[]{program, urdf_option, urdf, red_option};
  char policy_option[] = "--deadline-policy";
  char bad_policy[] = "ignore";
  char *bad_policy_argv[]{program, urdf_option, urdf, policy_option,
                          bad_policy};
  char equal[] = "20";
  char *bad_rate_argv[]{program, urdf_option,   urdf, red_option,
                        equal,   yellow_option, equal};
  if (!expectFailure(4, unknown_argv) || !expectFailure(4, missing_argv) ||
      !expectFailure(5, bad_policy_argv) || !expectFailure(7, bad_rate_argv)) {
    return EXIT_FAILURE;
  }

  std::ostringstream help;
  auto *original = std::cout.rdbuf(help.rdbuf());
  app::printPlannedUsage(program, app::SourceMode::Replay);
  std::cout.rdbuf(original);
  return help.str().find("default: 100") != std::string::npos &&
                 help.str().find("default: 20") != std::string::npos &&
                 help.str().find("--start-paused") != std::string::npos
             ? EXIT_SUCCESS
             : EXIT_FAILURE;
}
