#pragma once

#include <cstdint>
#include <string>

#include "adapters/replay/replay_support.hpp"
#include "console/tui_teleop_options.hpp"
#include "sinks/visualization_sink_options.hpp"

namespace motion_control_lab::baseline {

inline constexpr std::int64_t kTargetPeriodNs = 10'000'000;

struct TeleopOptions {
  std::string urdf_path;
  double rate_hz{100.0};
  double duration_s{0.0};
  bool tui_enabled{true};
  TuiTeleopOptions tui{"left", 0.005, 0.001, 0.5, 5.0};
  VisualizationSinkOptions visualization{"127.0.0.1", 8765, std::nullopt};
};

struct ReplayAppOptions {
  replay::ReplayOptions replay;
};

void printTopLevelUsage(const char *program);

void printTeleopUsage(const char *program);

void printReplayUsage(const char *program);

TeleopOptions parseTeleopOptions(int argc, char **argv);

ReplayAppOptions parseReplayOptions(int argc, char **argv);

} // namespace motion_control_lab::baseline
