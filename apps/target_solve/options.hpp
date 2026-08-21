#pragma once

#include <string>

#include "components/visualization/preview_sink_options.hpp"
#include "contracts/input/cartesian_teleop_options.hpp"

namespace motion_control_lab::target_solve {

enum class SolverKind {
  Mcc,
  Placo,
};

enum class MccBackend {
  Proxqp,
  Eiquadprog,
};

struct AlgorithmOptions {
  int maximum_iterations{10000};
  double soft_solve_time_budget_ms{100.0};
  double position_tolerance_m{1.0e-4};
  double orientation_tolerance_rad{1.0e-4};
  double minimum_position_improvement_m{1.0e-8};
  double minimum_orientation_improvement_rad{1.0e-8};
  double joint_position_margin_rad{1.0e-3};
  double posture_weight{1.0e-5};
  double regularization{1.0e-4};
  double proxqp_absolute_tolerance{1.0e-8};
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

void printUsage(const char *program);

AppOptions parseOptions(int argc, char **argv);

} // namespace motion_control_lab::target_solve
