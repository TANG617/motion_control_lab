#pragma once

#include <cstddef>
#include <string>

#include "components/tui/planned_grouped_tui.hpp"
#include "components/visualization/preview_sink_options.hpp"
#include "contracts/input/cartesian_teleop_options.hpp"

namespace motion_control_lab::hierarchical_inverse_dynamics_torque_sim {

struct Options {
  std::string urdf_path;
  std::string mujoco_path;
  double duration_seconds{0.1};
  double retarget_x_m{0.01};
  bool admittance_enabled{false};
  bool start_paused{false};
  std::size_t single_step_count{0};
  bool keyboard_enabled{false};
  CartesianTeleopOptions keyboard;
  double ui_rate_hz{100.0};
  PlannedGroupedTuiConfig presentation;
  PreviewSinkOptions visualization{true, "127.0.0.1", 8765, std::nullopt};
  bool viewer_enabled{true};
};

void printUsage(const char *program);
Options parseOptions(int argc, char **argv);

} // namespace motion_control_lab::hierarchical_inverse_dynamics_torque_sim
