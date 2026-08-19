#pragma once

#include <string>

#include "console/tui_teleop_options.hpp"
#include "sinks/preview_sink_options.hpp"

namespace motion_control_lab::single_arm_servo_step {

struct AppOptions {
  std::string urdf_path;
  double rate_hz{100.0};
  double duration_s{0.0};
  bool tui_enabled{true};
  TuiTeleopOptions tui{"left", 0.005, 0.001, 0.5, 5.0};
  PreviewSinkOptions visualization{"127.0.0.1", 8765, std::nullopt};
};

void printUsage(const char *program);

AppOptions parseAppOptions(int argc, char **argv);

} // namespace motion_control_lab::single_arm_servo_step
