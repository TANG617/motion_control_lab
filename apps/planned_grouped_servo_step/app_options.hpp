#pragma once

#include <optional>
#include <string>

#include "adapters/replay/replay_support.hpp"
#include "console/tui_teleop_options.hpp"
#include "runtime/grouped_worker.hpp"
#include "sinks/visualization_sink_options.hpp"

namespace motion_control_lab::planned_grouped_servo_step {

enum class SourceMode {
  Teleop,
  Replay,
};

struct GroupedOptions {
  std::string urdf_path;
  double red_rate_hz{100.0};
  double yellow_rate_hz{20.0};
  double ui_rate_hz{100.0};
  DeadlinePolicy deadline_policy{DeadlinePolicy::Strict};
  double duration_s{0.0};
  bool tui_enabled{true};
  TuiTeleopOptions tui{"left", 0.005, 0.001, 0.5, 5.0};
  VisualizationSinkOptions visualization{"127.0.0.1", 8765, std::nullopt};
};

struct PlanningLimitOptions {
  double max_linear_velocity_mps{0.8};
  double max_linear_acceleration_mps2{4.0};
  double max_linear_jerk_mps3{20.0};
  double max_angular_velocity_rps{1.0};
  double max_angular_acceleration_rps2{2.0};
  double max_angular_jerk_rps3{10.0};
};

struct PlannedOptions {
  SourceMode source_mode{SourceMode::Teleop};
  GroupedOptions interactive;
  PlanningLimitOptions planning;
  std::optional<replay::ReplayOptions> replay;
  bool start_paused{false};
};

void printGroupedUsage(const char *program);

void printPlannedUsage(const char *program, SourceMode source_mode);

GroupedOptions parseGroupedOptions(int argc, char **argv);

PlannedOptions parsePlannedOptions(int argc, char **argv);

} // namespace motion_control_lab::planned_grouped_servo_step
