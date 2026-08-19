#pragma once

#include <optional>
#include <string>

#include "adapters/replay/replay_support.hpp"
#include "console/tui_teleop_options.hpp"
#include "runtime/grouped_worker.hpp"
#include "sinks/visualization_sink_options.hpp"

namespace motion_control_lab::grouped_servo_step {

enum class SourceMode {
  Teleop,
  Replay,
};

struct GroupedOptions {
  std::string urdf_path;
  double red_rate_hz{1000.0};
  double yellow_rate_hz{100.0};
  double ui_rate_hz{100.0};
  DeadlinePolicy deadline_policy{DeadlinePolicy::Strict};
  double duration_s{0.0};
  bool tui_enabled{true};
  TuiTeleopOptions tui{"left", 0.005, 0.001, 0.5, 5.0};
  VisualizationSinkOptions visualization{"127.0.0.1", 8765, std::nullopt};
};

struct LaunchOptions {
  SourceMode source_mode{SourceMode::Teleop};
  GroupedOptions interactive;
  std::optional<replay::ReplayOptions> replay;
};

void printGroupedUsage(const char *program);

void printTopLevelUsage(const char *program);

GroupedOptions parseGroupedOptions(int argc, char **argv);

LaunchOptions parseLaunchOptions(int argc, char **argv);

} // namespace motion_control_lab::grouped_servo_step
