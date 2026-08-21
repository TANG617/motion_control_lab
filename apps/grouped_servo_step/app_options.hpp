#pragma once

#include <optional>
#include <string>

#include "adapters/replay/replay_support.hpp"
#include "contracts/input/cartesian_teleop_options.hpp"
#include "components/scheduler/grouped_worker.hpp"
#include "components/visualization/preview_sink_options.hpp"

namespace motion_control_lab::grouped_servo_step {

enum class SourceMode {
  Teleop,
  Replay,
};

struct SolverOptions {
  double regularization{1.0e-4};
  double position_tolerance_m{1.0e-4};
  double orientation_tolerance_rad{1.0e-4};
  double maximum_accepted_hard_violation{5.0e-4};
  double joint_position_margin_rad{1.0e-2};
  double cartesian_progress_weight{3.0};
  double red_proxqp_absolute_tolerance{1.0e-6};
  double yellow_posture_weight{1.0};
  double yellow_to_red_coupling_weight{10.0};
  double minimum_collision_distance_m{0.3};
  double collision_influence_distance_m{0.35};
  double collision_damping_gain_per_s{2.0};
  double collision_weight{100.0};
};

struct GroupedOptions {
  std::string urdf_path;
  double red_rate_hz{1000.0};
  double yellow_rate_hz{100.0};
  double ui_rate_hz{100.0};
  DeadlinePolicy deadline_policy{DeadlinePolicy::Strict};
  double duration_s{0.0};
  bool tui_enabled{true};
  CartesianTeleopOptions tui{"left", 0.005, 0.001, 0.5, 5.0};
  PreviewSinkOptions visualization{true, "127.0.0.1", 8765, std::nullopt};
  SolverOptions solver;
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
