#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

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
  double minimum_position_improvement_m{1.0e-8};
  double minimum_orientation_improvement_rad{1.0e-8};
  double maximum_accepted_hard_violation{5.0e-4};
  double joint_position_margin_rad{1.0e-2};
  bool joint_position_braking_velocity_envelope_enabled{true};
  double cartesian_progress_weight{100.0};
  double cartesian_preservation_tolerance{5.0e-4};
  double scale_preservation_tolerance{1.0e-4};
  double posture_preservation_tolerance{1.0e-5};
  int yellow_maximum_iterations{1};
  double red_proxqp_absolute_tolerance{2.0e-5};
  double red_proxqp_primal_infeasibility_tolerance{1.0e-12};
  bool red_proxqp_warm_start_enabled{false};
  double yellow_posture_weight{1.0};
  double yellow_to_red_coupling_weight{1.0};
  double minimum_collision_distance_m{0.1};
  double collision_influence_distance_m{0.15};
  double collision_damping_gain_per_s{2.0};
  double collision_weight{10.0};
};

struct CollisionLinkPairOptions {
  std::string first_link;
  std::string second_link;
};

struct RobotOptions {
  std::vector<std::string> inactive_joint_names{"knee_pitch_joint",
                                                 "ankle_pitch_joint"};
  std::array<double, 20> maximum_joint_accelerations_rad_per_s2{
      6.0,  6.0,  6.0,  4.0,   4.0,   4.0,   10.10, 10.10, 12.42, 12.48,
      16.2, 16.2, 16.2, 10.10, 10.10, 12.42, 12.48, 16.2,  16.2,  16.2};
  std::vector<CollisionLinkPairOptions> self_collision_link_pairs{
      {"left_arm_link4", "body_link4"},
      {"right_arm_link4", "body_link4"},
      {"left_arm_link7", "right_arm_link4"},
      {"right_arm_link7", "left_arm_link4"}};
  std::vector<std::string> collision_mesh_search_paths;
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
  RobotOptions robot;
  SolverOptions solver;
};

enum class PlanningSynchronization {
  Time,
  Phase,
};

struct PlanningOptions {
  double max_linear_velocity_mps{0.8};
  double max_linear_acceleration_mps2{4.0};
  double max_linear_jerk_mps3{20.0};
  double max_angular_velocity_rps{1.0};
  double max_angular_acceleration_rps2{2.0};
  double max_angular_jerk_rps3{10.0};
  PlanningSynchronization synchronization{PlanningSynchronization::Time};
};

struct Options {
  SourceMode source_mode{SourceMode::Teleop};
  HierarchicalOptions interactive;
  PlanningOptions planning;
  std::optional<replay::ReplayOptions> replay;
  bool start_paused{false};
};

void printHierarchicalUsage(const char *program);

void printPlannedUsage(const char *program, SourceMode source_mode);

HierarchicalOptions parseHierarchicalOptions(int argc, char **argv);

Options parseOptions(int argc, char **argv);

} // namespace motion_control_lab::planned_hierarchical_step
