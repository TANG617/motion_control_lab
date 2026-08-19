#pragma once

#include <string>

#include "adapters/replay/replay_support.hpp"
#include "console/tui_teleop_options.hpp"
#include "sinks/visualization_sink_options.hpp"

namespace motion_control_lab::servo_step {

enum class SolverKind {
  Mcc,
  Placo,
};

enum class MccBackend {
  Proxqp,
  Eiquadprog,
};

struct TeleopOptions {
  std::string urdf_path;
  double rate_hz{100.0};
  double duration_s{0.0};
  bool tui_enabled{true};
  TuiTeleopOptions tui{"left", 0.005, 0.001, 0.5, 5.0};
  VisualizationSinkOptions visualization{"127.0.0.1", 8765, std::nullopt};
};

struct AppOptions {
  SolverKind solver{SolverKind::Mcc};
  MccBackend backend{MccBackend::Proxqp};
  TeleopOptions interactive;
};

struct ReplayAppOptions {
  SolverKind solver{SolverKind::Mcc};
  MccBackend backend{MccBackend::Proxqp};
  double rate_hz{100.0};
  replay::ReplayOptions replay;
};

void printTeleopUsage(const char *program);

void printTopLevelUsage(const char *program);

AppOptions parseAppOptions(int argc, char **argv);

ReplayAppOptions parseReplayAppOptions(int argc, char **argv);

} // namespace motion_control_lab::servo_step
