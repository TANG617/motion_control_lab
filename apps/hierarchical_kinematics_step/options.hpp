#pragma once

#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "adapters/replay/replay_support.hpp"
#include "components/scheduler/grouped_worker.hpp"
#include "components/tui/planned_grouped_tui.hpp"
#include "components/visualization/preview_sink_options.hpp"
#include "contracts/input/cartesian_teleop_options.hpp"

namespace motion_control_lab::hierarchical_kinematics_step {

enum class Profile {
  Hierarchical,
  Planned,
  PlannedOtg,
  PlannedOtgNullspace,
  PlannedOtgNullspaceAdmittanceKinematicSim,
};

const char *profileName(Profile profile);
Profile parseProfile(const std::string &value);

struct ProfileCapabilities {
  bool cartesian_planning{false};
  bool joint_otg{false};
  bool nullspace{false};
  bool admittance{false};
  bool kinematic_simulation{false};
  bool telemetry{false};
};

ProfileCapabilities profileCapabilities(Profile profile);

enum class SourceMode {
  Teleop,
  Replay,
};

enum class JointTargetMode {
  FutureO1Pv,
  IkPv,
};

inline const char *jointTargetModeName(JointTargetMode mode) {
  switch (mode) {
  case JointTargetMode::FutureO1Pv:
    return "future-o1-pv";
  case JointTargetMode::IkPv:
    return "ik-pv";
  }
  return "unknown";
}

enum class PlanningSynchronization {
  None,
  Time,
  Phase,
};

inline const char *planningSynchronizationName(PlanningSynchronization value) {
  switch (value) {
  case PlanningSynchronization::None:
    return "None";
  case PlanningSynchronization::Time:
    return "Time";
  case PlanningSynchronization::Phase:
    return "Phase";
  }
  return "unknown";
}

enum class JointPlanningAlgorithm {
  JerkLimited,
};

inline const char *jointPlanningAlgorithmName(JointPlanningAlgorithm value) {
  switch (value) {
  case JointPlanningAlgorithm::JerkLimited:
    return "JerkLimited";
  }
  return "unknown";
}

struct SolverOptions {
  double regularization{1.0e-10};
  double position_tolerance_m{1.0e-4};
  double orientation_tolerance_rad{1.0e-4};
  double minimum_position_improvement_m{1.0e-8};
  double minimum_orientation_improvement_rad{1.0e-8};
  double maximum_accepted_hard_violation{1.0e-4};
  double joint_position_margin_rad{1.0e-2};
  bool joint_position_braking_velocity_envelope_enabled{false};
  bool red_joint_acceleration_limits_enabled{false};

  // The first three profiles retain the original shared Cartesian scale and
  // Secondary posture-coupling topology.  The two null-space profiles use the
  // explicit Primary position / Secondary orientation / Tertiary posture
  // topology below.
  double legacy_cartesian_progress_weight{100.0};
  double legacy_cartesian_preservation_tolerance{5.0e-4};
  double legacy_scale_preservation_tolerance{1.0e-4};
  double legacy_posture_preservation_tolerance{1.0e-5};
  double legacy_yellow_posture_weight{1.0};
  double legacy_yellow_to_red_coupling_weight{1.0};
  double legacy_minimum_collision_distance_m{0.1};
  double legacy_collision_influence_distance_m{0.15};
  double legacy_collision_damping_gain_per_s{2.0};
  double legacy_collision_weight{10.0};
  double red_primary_task_tcp_position_progress_weight{100.0};
  double red_primary_task_tcp_position_preservation_tolerance_mps{5.0e-4};
  double red_primary_task_tcp_position_progress_preservation_tolerance{1.0e-4};
  double red_secondary_task_tcp_orientation_progress_weight{100.0};
  double red_secondary_task_tcp_orientation_preservation_tolerance_radps{5.0e-4};
  double red_secondary_task_tcp_orientation_progress_preservation_tolerance{1.0e-4};
  double red_tertiary_task_yellow_posture_coupling_preservation_tolerance{1.0e-5};
  double red_tertiary_task_link4_position_weight{100.0};
  double red_tertiary_task_link4_position_servo_gain_per_s{10.0};
  double red_tertiary_task_link4_position_preservation_tolerance_mps{5.0e-4};
  int yellow_maximum_iterations{1};
  int red_proxqp_maximum_iterations{200};
  double red_proxqp_absolute_tolerance{2.0e-5};
  double red_proxqp_primal_infeasibility_tolerance{1.0e-12};
  bool red_proxqp_warm_start_enabled{false};
  double yellow_task_posture_preference_weight{1.0};
  double yellow_task_posture_preference_servo_gain_per_s{10.0};
  std::vector<std::pair<std::string, double>>
      yellow_task_posture_preference_joint_weight_multipliers;
  double red_tertiary_task_yellow_posture_coupling_weight{1.0};
  double red_tertiary_task_yellow_posture_coupling_servo_gain_per_s{10.0};
  std::vector<std::pair<std::string, double>>
      red_tertiary_task_yellow_posture_coupling_joint_weight_multipliers;
  double yellow_constraints_self_collision_avoidance_minimum_distance_m{0.1};
  double yellow_constraints_self_collision_avoidance_influence_distance_m{0.15};
  double yellow_constraints_self_collision_avoidance_damping_gain_per_s{2.0};
  double yellow_constraints_self_collision_avoidance_weight{10.0};
};

struct CollisionLinkPairOptions {
  std::string first_link;
  std::string second_link;
};

struct JointStreamProfileOptions {
  std::string source_revision{"42ed3ce3a19f5a7346874a31ec659c0298751137"};
  std::string source_path{
      "products/synrobot/modules/control/motion_control/config/robots/"
      "psi_r1.yaml"};
  std::string source_sha256{
      "895416681f8fd41138f3be280b0f7a330ca004f46951b12b3fea2a230e779d1b"};
  std::string jerk_override_reason{"OTG Lab fixed jerk override"};
  std::array<std::string, 20> joint_names{
      "head_yaw_joint",    "head_pitch_joint", "torso_yaw_joint",
      "torso_pitch_joint", "knee_pitch_joint", "ankle_pitch_joint",
      "left_arm_joint1",   "left_arm_joint2",  "left_arm_joint3",
      "left_arm_joint4",   "left_arm_joint5",  "left_arm_joint6",
      "left_arm_joint7",   "right_arm_joint1", "right_arm_joint2",
      "right_arm_joint3",  "right_arm_joint4", "right_arm_joint5",
      "right_arm_joint6",  "right_arm_joint7"};
  std::array<double, 20> position_lower_rad{
      -1.5708, -0.3491, -3.082251406, -1.9199, 0.0,     -1.0821, -3.1067,
      -2.0944, -3.1067, -2.5307,      -3.1067, -1.0472, -1.0472, -3.1067,
      -2.0944, -3.1067, -1.0472,      -3.1067, -1.0472, -1.0472};
  std::array<double, 20> position_upper_rad{
      1.5708, 0.3491, 2.932153093, 1.117, 2.6529, 0.0,    3.1067,
      2.0944, 3.1067, 1.0472,      3.1067, 1.0472, 1.0472, 3.1067,
      2.0944, 3.1067, 2.5307,      3.1067, 1.0472, 1.0472};
  std::array<double, 20> max_velocity_rad_per_s{
      3.0, 3.0, 3.0, 3.0,  3.0,  3.0,  5.05, 5.05, 5.71, 5.24,
      4.1, 4.1, 4.1, 5.05, 5.05, 5.71, 5.24, 4.1,  4.1,  4.1};
  std::array<double, 20> max_acceleration_rad_per_s2{
      9.0,  9.0,  9.0,  6.0,   6.0,   6.0,   15.15, 15.15, 18.63, 18.72,
      24.3, 24.3, 24.3, 15.15, 15.15, 18.63, 18.72, 24.3,  24.3,  24.3};
  std::array<double, 20> max_jerk_rad_per_s3{
      3200.0, 3200.0, 3200.0, 3200.0, 3200.0, 3200.0, 3200.0,
      3200.0, 3200.0, 3200.0, 3200.0, 3200.0, 3200.0, 3200.0,
      3200.0, 3200.0, 3200.0, 3200.0, 3200.0, 3200.0};
};

struct RobotOptions {
  std::string profile_provenance;
  std::string base_frame{"base_link"};
  std::string left_end_effector_frame{"left_arm_ee_link"};
  std::string right_end_effector_frame{"right_arm_ee_link"};
  std::string left_link4_frame{"left_arm_link4"};
  std::string right_link4_frame{"right_arm_link4"};
  Eigen::Isometry3d left_tcp_offset{[] {
    auto value = Eigen::Isometry3d::Identity();
    value.translation() = Eigen::Vector3d{0.0, 0.0, 0.1};
    return value;
  }()};
  Eigen::Isometry3d right_tcp_offset{[] {
    auto value = Eigen::Isometry3d::Identity();
    value.translation() = Eigen::Vector3d{0.0, 0.0, 0.1};
    return value;
  }()};
  std::vector<std::string> joint_names{
      "head_yaw_joint",    "head_pitch_joint", "torso_yaw_joint",
      "torso_pitch_joint", "knee_pitch_joint", "ankle_pitch_joint",
      "left_arm_joint1",   "left_arm_joint2",  "left_arm_joint3",
      "left_arm_joint4",   "left_arm_joint5",  "left_arm_joint6",
      "left_arm_joint7",   "right_arm_joint1", "right_arm_joint2",
      "right_arm_joint3",  "right_arm_joint4", "right_arm_joint5",
      "right_arm_joint6",  "right_arm_joint7"};
  std::vector<double> default_positions{
      0.0, 0.31, 0.0, 0.5, 0.5, -0.5, 0.9, -1.38, -1.57, -1.4,
      -0.45, 0.0, 0.0, -0.9, 1.38, 1.57, 1.4, 0.45, 0.0, 0.0};
  std::vector<std::size_t> left_arm_joint_indices{6, 7, 8, 9, 10, 11, 12};
  std::vector<std::size_t> right_arm_joint_indices{13, 14, 15, 16, 17, 18,
                                                    19};
  std::vector<double> effort_limits{
      16.0, 16.0, 450.0, 450.0, 600.0, 600.0, 108.0, 108.0, 66.0, 66.0,
      19.0, 19.0, 19.0, 108.0, 108.0, 66.0, 66.0, 19.0, 19.0, 19.0};
  std::vector<std::string> inactive_joint_names{};
  std::vector<CollisionLinkPairOptions> self_collision_link_pairs{
      {"left_arm_link4", "body_link4"},
      {"right_arm_link4", "body_link4"},
      {"left_arm_link7", "right_arm_link4"},
      {"right_arm_link7", "left_arm_link4"}};
  std::vector<std::string> collision_mesh_search_paths;
  JointStreamProfileOptions joint_stream;
};

// App-local alias used by the existing solver/planning/presentation surface.
// The values are owned by Options::interactive.robot, not by the shared R1
// singleton used by the other Lab applications.
using R1RobotConfig = RobotOptions;

struct AdmittanceOptions {
  bool angular_enabled{false};
  double linear_environment_stiffness_n_per_m{200.0};
  double linear_environment_damping_ns_per_m{20.0};
  double maximum_force_n{20.0};
  double angular_environment_stiffness_nm_per_rad{10.0};
  double angular_environment_damping_nms_per_rad{1.0};
  double maximum_torque_nm{5.0};
  double wrench_filter_alpha{0.08};
};

struct SimulationOptions {
  std::string mujoco_model_path;
  bool viewer_enabled{true};
};

struct HierarchicalOptions {
  std::string urdf_path;
  double red_rate_hz{1000.0};
  double yellow_rate_hz{100.0};
  double ui_rate_hz{100.0};
  DeadlinePolicy deadline_policy{DeadlinePolicy::Strict};
  double duration_s{0.0};
  PlannedGroupedTuiConfig presentation;
  CartesianTeleopOptions tui{"left", 0.005, 0.001, 0.5, 5.0};
  PreviewSinkOptions visualization{true, "127.0.0.1", 8765, std::nullopt};
  RobotOptions robot;
  SolverOptions solver;
  AdmittanceOptions admittance;
  SimulationOptions simulation;
};

struct PlanningOptions {
  double max_linear_velocity_mps{0.8};
  double max_linear_acceleration_mps2{4.0};
  double max_linear_jerk_mps3{20.0};
  double max_angular_velocity_rps{1.0};
  double max_angular_acceleration_rps2{2.0};
  double max_angular_jerk_rps3{10.0};
  PlanningSynchronization cartesian_synchronization{
      PlanningSynchronization::Time};
  JointPlanningAlgorithm joint_algorithm{JointPlanningAlgorithm::JerkLimited};
  PlanningSynchronization joint_synchronization{
      PlanningSynchronization::Phase};
};

struct JointTargetOptions {
  JointTargetMode mode{JointTargetMode::FutureO1Pv};
  double future_o1_velocity_deadband_rad_per_s{1.0e-10};
};

struct ReplaySettlingOptions {
  double fk_position_m{1.0e-4};
  double fk_orientation_rad{1.0e-4};
  double velocity_rad_per_s{5.0e-3};
  double acceleration_rad_per_s2{1.0};
  std::size_t required_cycles{50U};
};

struct Options {
  Profile profile{Profile::PlannedOtgNullspaceAdmittanceKinematicSim};
  SourceMode source_mode{SourceMode::Teleop};
  HierarchicalOptions interactive;
  PlanningOptions planning;
  JointTargetOptions joint_target;
  ReplaySettlingOptions replay_settling;
  std::optional<replay::ReplayOptions> replay;
  bool replay_trace_enabled{true};
  bool replay_elbow_teleop_enabled{false};
  bool start_paused{false};
  bool dump_resolved_options{false};
  std::string launcher_argv_json;
  std::vector<std::string> binary_argv;
};

Options profileDefaults(Profile profile);

std::string resolvedOptionsJson(const Options &options);

void printHierarchicalUsage(const char *program);

void printPlannedUsage(const char *program, SourceMode source_mode);

HierarchicalOptions parseHierarchicalOptions(
    int argc, char **argv, HierarchicalOptions defaults = {},
    Profile profile = Profile::PlannedOtgNullspaceAdmittanceKinematicSim);

Options parseOptions(int argc, char **argv);

} // namespace motion_control_lab::hierarchical_kinematics_step
