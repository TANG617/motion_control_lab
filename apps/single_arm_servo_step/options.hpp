#pragma once

#include <string>

#include "components/visualization/preview_sink_options.hpp"
#include "contracts/input/cartesian_teleop_options.hpp"

namespace motion_control_lab::single_arm_servo_step {

struct AppOptions {
  std::string urdf_path;
  double rate_hz{100.0};
  double duration_s{0.0};
  bool tui_enabled{true};
  CartesianTeleopOptions tui{"left", 0.005, 0.001, 0.5, 5.0};
  PreviewSinkOptions visualization{true, "127.0.0.1", 8765, std::nullopt};
  double regularization{1.0e-8};
  int maximum_iterations{1};
  double position_tolerance_m{1.0e-4};
  double orientation_tolerance_rad{1.0e-4};
  double joint_position_margin_rad{0.0};
};

void printUsage(const char *program);

AppOptions parseOptions(int argc, char **argv);

} // namespace motion_control_lab::single_arm_servo_step
