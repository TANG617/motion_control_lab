#pragma once

#include <cstdint>
#include <string>

#include "adapters/replay/replay_support.hpp"
#include "components/visualization/preview_sink_options.hpp"
#include "contracts/input/cartesian_teleop_options.hpp"

namespace motion_control_lab::baseline {

inline constexpr std::int64_t kTargetPeriodNs = 10'000'000;

struct TeleopOptions {
  std::string urdf_path;
  double rate_hz{100.0};
  double duration_s{0.0};
  bool tui_enabled{true};
  CartesianTeleopOptions tui{"left", 0.005, 0.001, 0.5, 5.0};
  PreviewSinkOptions visualization{true, "127.0.0.1", 8765, std::nullopt};
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
