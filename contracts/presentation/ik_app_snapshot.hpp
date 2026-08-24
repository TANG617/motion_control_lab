#pragma once

#include "contracts/input/input_contract.hpp"
#include "contracts/runtime/runtime_contract.hpp"

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace motion_control_lab
{

struct ArmTargetError
{
  ArmSide side{ArmSide::Left};
  double position_m{0.0};
  double orientation_rad{0.0};
};

struct ArmForwardKinematics
{
  ArmSide side{ArmSide::Left};
  Pose pose{Pose::Identity()};
};

struct SelfCollisionPairDebug
{
  std::string first_link;
  std::string second_link;
  double distance_before_m{0.0};
  double distance_after_m{0.0};
  bool active{false};
};

struct SelfCollisionDebug
{
  std::string label{"self-collision"};
  std::uint64_t input_state_sequence{0};
  double minimum_distance_m{0.0};
  double influence_distance_m{0.0};
  double minimum_distance_before_m{0.0};
  double minimum_distance_after_m{0.0};
  double margin_shortfall_m{0.0};
  std::vector<double> input_joint_positions;
  std::vector<SelfCollisionPairDebug> pairs;
};

struct TaskScaleDebug
{
  std::string name;
  bool active{false};
  double scale{1.0};
  double cost{0.0};
  bool degraded{false};
  bool stuck{false};
};

struct RequirementDebug
{
  std::string name;
  std::string unit;
  std::string source;
  bool enabled{false};
  bool active{false};
  double maximum_violation{0.0};
  double cost{0.0};
};

struct GroupedAttemptDebug
{
  std::string rejection_reason;
  std::uint64_t run_generation{0};
  std::uint64_t attempt_revision{0};
  std::uint64_t value_revision{0};
  std::string coupling_state;
  std::uint64_t consumed_source_value_revision{0};
  std::uint64_t captured_state_sequence{0};
  std::int64_t captured_state_time_nanoseconds{0};
};

struct SolverRunCounters
{
  std::uint64_t attempts{0};
  std::uint64_t accepted{0};
  std::uint64_t rejected{0};
};

struct QpPassDebug
{
  std::string label;
  bool attempted{false};
  bool succeeded{false};
  std::string status;
  std::string native_status;
  double solve_time_ms{0.0};
  int iterations{0};
  bool warm_start_used{false};
};

struct SolverDebug
{
  std::string label{"IK"};
  std::string disposition;
  std::string joint_limit_policy;
  std::string termination_reason;
  int ik_iterations{0};
  bool converged{false};
  double ik_solve_time_ms{0.0};
  RollingPercentilesSnapshot ik_solve_time_percentiles;
  std::vector<std::string> saturated_joints;
  std::string backend;
  std::string qp_status;
  std::string native_status;
  bool has_qp_diagnostics{true};
  double objective_value{0.0};
  double primal_residual{0.0};
  double dual_residual{0.0};
  double maximum_hard_violation{0.0};
  double qp_solve_time_ms{0.0};
  int qp_iterations{0};
  int active_set_size{0};
  bool warm_start_used{false};
  std::vector<QpPassDebug> qp_passes;
  std::vector<TaskScaleDebug> task_scales;
  std::vector<RequirementDebug> requirements;
  std::optional<SolverRunCounters> run_counters;
  std::optional<GroupedAttemptDebug> grouped_attempt;
};

struct WorkerDebug
{
  std::string label;
  double configured_rate_hz{0.0};
  std::uint64_t iteration_count{0};
  std::uint64_t deadline_miss_count{0};
  std::uint64_t consecutive_deadline_misses{0};
  std::uint64_t skipped_release_count{0};
  double maximum_release_lateness_ms{0.0};
  double maximum_execution_ms{0.0};
  double maximum_release_to_finish_ms{0.0};
  double maximum_overrun_ms{0.0};
  double maximum_solver_ms{0.0};
  std::uint64_t recoverable_rejection_count{0};
  double maximum_non_solver_execution_ms{0.0};
  double latest_release_lateness_ms{0.0};
  double latest_execution_ms{0.0};
  double latest_release_to_finish_ms{0.0};
  double latest_overrun_ms{0.0};
  double latest_solver_ms{0.0};
  double latest_non_solver_execution_ms{0.0};
};

struct CpuAffinityDebug
{
  std::string role;
  bool enabled{false};
  std::int64_t thread_id{-1};
  std::vector<unsigned int> requested_cpus;
  std::vector<unsigned int> effective_cpus;
};

enum class IkRuntimeState
{
  Running,
  RecoverableReject,
  FaultHold,
};

struct RejectedTargetDebug
{
  std::uint64_t revision{0};
  std::vector<ArmTarget> targets;
  std::string detail;
};

struct PlannedArmDebug
{
  ArmSide side{ArmSide::Left};
  Pose source_goal{Pose::Identity()};
  Pose reference{Pose::Identity()};
  Pose forward_kinematics{Pose::Identity()};
  Eigen::Matrix<double, 6, 1> reference_twist{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> reference_acceleration{Eigen::Matrix<double, 6, 1>::Zero()};
  double tracking_position_error_m{0.0};
  double tracking_orientation_error_rad{0.0};
};

struct CartesianPlannerDebug
{
  std::string state{"idle"};
  double sample_time_s{0.0};
  std::vector<PlannedArmDebug> arms;
};

struct ArmPresentation
{
  ArmSide side{ArmSide::Left};
  std::string target_channel;
  std::string forward_kinematics_channel;
  std::vector<std::size_t> joint_indices;
};

struct InteractiveIkPresentation
{
  std::string base_frame_id;
  std::string joint_state_channel;
  std::vector<ArmPresentation> arms;
};

inline const ArmPresentation * findArmPresentation(
  const InteractiveIkPresentation & presentation, ArmSide side)
{
  for (const auto & arm : presentation.arms) {
    if (arm.side == side) {
      return &arm;
    }
  }
  return nullptr;
}

struct IkDebugFrame
{
  std::string run_id{"interactive-preview"};
  std::vector<ArmTarget> targets;
  std::vector<ArmForwardKinematics> forward_kinematics;
  JointNames joint_names;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::string ik_status{"not solved yet"};
  int iterations{0};
  bool converged{false};
  double solve_time_ms{0.0};
  std::vector<ArmTargetError> target_errors;
  std::vector<SolverDebug> solvers;
  std::vector<WorkerDebug> workers;
  std::vector<CpuAffinityDebug> cpu_affinities;
  std::vector<SelfCollisionDebug> self_collisions;
  std::optional<RejectedTargetDebug> rejected_target;
  std::optional<CartesianPlannerDebug> cartesian_planner;
  std::string status{"Ready"};
  IkRuntimeState runtime_state{IkRuntimeState::Running};
  bool paused{false};
  ArmSide selected_side{ArmSide::Left};
};

}  // namespace motion_control_lab
