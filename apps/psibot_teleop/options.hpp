#pragma once

#include "contracts/input/cartesian_teleop_options.hpp"
#include <string>

namespace motion_control_lab::psibot_teleop {
enum class Mode { Single, Wbc };
inline const char * modeName(Mode mode) { return mode == Mode::Single ? "single" : "wbc"; }
struct Options {
  std::string address{"127.0.0.1:6060"};
  std::string auth{"admin:admin"};
  Mode mode{Mode::Single};
  CartesianTeleopOptions teleop;
  bool tui{true};
  bool help{false};
  double ui_hz{30};
  double feedback_hz{20};
  double send_hz{50};
  double duration_s{0};
  double connect_timeout_s{5};
  double velocity{0.1}, acceleration{0.5}, jerk{1.0};
  double joint_step_deg{1.0};
  std::string log_dir;
};
Options parseOptions(int argc, char ** argv);
std::string helpText();
}  // namespace motion_control_lab::psibot_teleop
