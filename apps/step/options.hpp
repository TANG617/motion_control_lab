#pragma once

#include <string>

#include "adapters/replay/replay_support.hpp"
#include "components/visualization/preview_sink_options.hpp"
#include "contracts/input/cartesian_teleop_options.hpp"

namespace motion_control_lab::step {

enum class SolverKind {
  Mcc,
  Placo,
};

enum class MccBackend {
  Proxqp,
  Eiquadprog,
};

struct AlgorithmOptions {
  double regularization{1.0e-4};
  double position_tolerance_m{1.0e-4};
  double orientation_tolerance_rad{1.0e-4};
  double joint_position_margin_rad{1.0e-3};
};

struct TeleopOptions {
  std::string urdf_path;
  double rate_hz{100.0};
  double duration_s{0.0};
  bool tui_enabled{true};
  CartesianTeleopOptions tui{"left", 0.005, 0.001, 0.5, 5.0};
  PreviewSinkOptions visualization{true, "127.0.0.1", 8765, std::nullopt};
};

struct AppOptions {
  SolverKind solver{SolverKind::Mcc};
  MccBackend backend{MccBackend::Proxqp};
  AlgorithmOptions algorithm;
  TeleopOptions interactive;
};

struct ReplayAppOptions {
  SolverKind solver{SolverKind::Mcc};
  MccBackend backend{MccBackend::Proxqp};
  AlgorithmOptions algorithm;
  double rate_hz{100.0};
  replay::ReplayOptions replay;
};

void printTeleopUsage(const char *program);

void printTopLevelUsage(const char *program);

AppOptions parseOptions(int argc, char **argv);

ReplayAppOptions parseReplayOptions(int argc, char **argv);

} // namespace motion_control_lab::step
