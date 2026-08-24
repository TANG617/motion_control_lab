#pragma once

#include <optional>
#include <string>

#include "adapters/replay/replay_support.hpp"
#include "components/scheduler/grouped_worker.hpp"
#include "components/tui/planned_grouped_tui.hpp"
#include "components/visualization/preview_sink_options.hpp"
#include "contracts/input/cartesian_teleop_options.hpp"

namespace motion_control_lab::planned_hierarchical_step {

enum class SourceMode {
  Teleop,
  Replay,
};

struct SolverOptions {
  double regularization{1.0e-10};
  double position_tolerance_m{1.0e-4};
  double orientation_tolerance_rad{1.0e-4};
  double maximum_accepted_hard_violation{5.0e-4};
  double joint_position_margin_rad{1.0e-2};
  double cartesian_progress_weight{100.0};
  double red_proxqp_absolute_tolerance{2.0e-5};
  double red_proxqp_primal_infeasibility_tolerance{1.0e-12};
  double yellow_posture_weight{1.0};
  double yellow_to_red_coupling_weight{1.0};
  double minimum_collision_distance_m{0.1};
  double collision_influence_distance_m{0.15};
  double collision_damping_gain_per_s{2.0};
  double collision_weight{10.0};
};

struct HierarchicalOptions {
  std::string urdf_path;
  double red_rate_hz{100.0};
  double yellow_rate_hz{20.0};
  double ui_rate_hz{100.0};
  DeadlinePolicy deadline_policy{DeadlinePolicy::Strict};
  double duration_s{0.0};
  PlannedGroupedTuiConfig presentation;
  CartesianTeleopOptions tui{"left", 0.005, 0.001, 0.5, 5.0};
  PreviewSinkOptions visualization{true, "127.0.0.1", 8765, std::nullopt};
  SolverOptions solver;
};

struct PlanningLimitOptions {
  double max_linear_velocity_mps{0.8};
  double max_linear_acceleration_mps2{4.0};
  double max_linear_jerk_mps3{20.0};
  double max_angular_velocity_rps{1.0};
  double max_angular_acceleration_rps2{2.0};
  double max_angular_jerk_rps3{10.0};
};

struct Options {
  SourceMode source_mode{SourceMode::Teleop};
  HierarchicalOptions interactive;
  PlanningLimitOptions planning;
  std::optional<replay::ReplayOptions> replay;
  bool start_paused{false};
};

void printHierarchicalUsage(const char *program);

void printPlannedUsage(const char *program, SourceMode source_mode);

HierarchicalOptions parseHierarchicalOptions(int argc, char **argv);

Options parseOptions(int argc, char **argv);

} // namespace motion_control_lab::planned_hierarchical_step
