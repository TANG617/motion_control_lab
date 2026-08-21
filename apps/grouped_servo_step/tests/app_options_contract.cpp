#include "../app_options.hpp"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace app = motion_control_lab::grouped_servo_step;

int main() {
  char program[] = "mcl_grouped_servo_step";
  char urdf_option[] = "--urdf";
  char urdf[] = "/tmp/robot.urdf";
  char *defaults_argv[]{program, urdf_option, urdf};
  const auto defaults = app::parseGroupedOptions(3, defaults_argv);
  if (defaults.red_rate_hz != 1000.0 || defaults.yellow_rate_hz != 100.0 ||
      defaults.ui_rate_hz != 100.0 ||
      defaults.deadline_policy != motion_control_lab::DeadlinePolicy::Strict) {
    return EXIT_FAILURE;
  }

  char red_option[] = "--red-rate";
  char red[] = "800";
  char yellow_option[] = "--yellow-rate";
  char yellow[] = "80";
  char policy_option[] = "--deadline-policy";
  char policy[] = "monitor";
  char regularization_option[] = "--regularization";
  char regularization[] = "2e-4";
  char *custom_argv[]{program,    urdf_option,   urdf,
                      red_option, red,           yellow_option,
                      yellow,     policy_option, policy, regularization_option,
                      regularization};
  const auto custom = app::parseGroupedOptions(11, custom_argv);
  if (custom.red_rate_hz != 800.0 || custom.yellow_rate_hz != 80.0 ||
      custom.deadline_policy != motion_control_lab::DeadlinePolicy::Monitor ||
      custom.solver.regularization != 2.0e-4) {
    return EXIT_FAILURE;
  }

  auto expectFailure = [&](int argc, char **argv) {
    try {
      (void)app::parseGroupedOptions(argc, argv);
      return false;
    } catch (const std::runtime_error &) {
      return true;
    }
  };
  char unknown[] = "--green-rate";
  char value[] = "10";
  char *unknown_argv[]{program, urdf_option, urdf, unknown, value};
  char *missing_argv[]{program, urdf_option, urdf, red_option};
  char bad_policy[] = "ignore";
  char *bad_policy_argv[]{program, urdf_option, urdf, policy_option,
                          bad_policy};
  char equal[] = "100";
  char *bad_rate_argv[]{program, urdf_option,   urdf, red_option,
                        equal,   yellow_option, equal};
  if (!expectFailure(5, unknown_argv) || !expectFailure(4, missing_argv) ||
      !expectFailure(5, bad_policy_argv) || !expectFailure(7, bad_rate_argv)) {
    return EXIT_FAILURE;
  }

  std::ostringstream help;
  auto *original = std::cout.rdbuf(help.rdbuf());
  app::printGroupedUsage(program);
  std::cout.rdbuf(original);
  return help.str().find("default: 1000") != std::string::npos &&
                 help.str().find("default: 100") != std::string::npos
             ? EXIT_SUCCESS
             : EXIT_FAILURE;
}
