#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace motion_control_lab
{

using Pose = Eigen::Isometry3d;
using JointNames = std::vector<std::string>;

enum class ArmSide
{
  Left,
  Right,
};

inline const char * armSideName(ArmSide side)
{
  return side == ArmSide::Left ? "left" : "right";
}

inline ArmSide parseArmSide(const std::string & side)
{
  if (side == "left") {
    return ArmSide::Left;
  }
  if (side == "right") {
    return ArmSide::Right;
  }
  throw std::runtime_error("side must be either 'left' or 'right'");
}

struct ArmTarget
{
  ArmSide side{ArmSide::Left};
  Pose target_pose{Pose::Identity()};
};

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
  bool attempt_accepted{false};
  bool has_accepted_value{false};
  std::string coupling_state;
  std::uint64_t consumed_source_value_revision{0};
  std::uint64_t captured_state_sequence{0};
  std::int64_t captured_state_time_nanoseconds{0};
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
  std::vector<std::string> saturated_joints;
  std::string backend;
  std::string qp_status;
  std::string native_status;
  double objective_value{0.0};
  double primal_residual{0.0};
  double dual_residual{0.0};
  double maximum_hard_violation{0.0};
  double qp_solve_time_ms{0.0};
  int qp_iterations{0};
  int active_set_size{0};
  bool warm_start_used{false};
  std::vector<TaskScaleDebug> task_scales;
  std::vector<RequirementDebug> requirements;
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
  const InteractiveIkPresentation & presentation,
  ArmSide side)
{
  for (const auto & arm : presentation.arms) {
    if (arm.side == side) {
      return &arm;
    }
  }
  return nullptr;
}

struct TargetCommand
{
  std::vector<ArmTarget> targets;
  ArmSide selected_side{ArmSide::Left};
  bool paused{false};
  bool stop_requested{false};
  std::string status{"Ready"};
};

struct IkDebugFrame
{
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
  std::vector<SelfCollisionDebug> self_collisions;
  std::string status{"Ready"};
  bool paused{false};
  ArmSide selected_side{ArmSide::Left};
};

}  // namespace motion_control_lab
