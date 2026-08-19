#include "../app_options.hpp"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace app = motion_control_lab::planned_grouped_step_otg;

int main() {
  char program[] = "mcl_planned_grouped_step_otg";
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
  char red[] = "200";
  char yellow_option[] = "--yellow-rate";
  char yellow[] = "40";
  char ui_option[] = "--ui";
  char ui[] = "none";
  char *custom_argv[]{program,       urdf_option, urdf,      red_option, red,
                      yellow_option, yellow,      ui_option, ui};
  const auto custom = app::parseGroupedOptions(9, custom_argv);
  if (custom.red_rate_hz != 200.0 || custom.yellow_rate_hz != 40.0 ||
      custom.tui_enabled) {
    return EXIT_FAILURE;
  }

  char teleop[] = "teleop";
  char mode_option[] = "--joint-target-mode";
  char mode[] = "ik-pv";
  char *planned_argv[]{program, teleop, urdf_option, urdf, mode_option, mode};
  const auto planned = app::parsePlannedOptions(6, planned_argv);
  if (planned.joint_target_mode != app::JointTargetMode::IkPv ||
      planned.interactive.red_rate_hz != 1000.0 ||
      planned.interactive.yellow_rate_hz != 100.0) {
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
  return help.str().find("default: 1000") != std::string::npos &&
                 help.str().find("default: 100") != std::string::npos &&
                 help.str().find("--start-paused") != std::string::npos &&
                 help.str().find("--joint-target-mode") != std::string::npos
             ? EXIT_SUCCESS
             : EXIT_FAILURE;
}
