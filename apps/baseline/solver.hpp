#pragma once

#include <json/json.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace motion_control_lab::baseline {

struct SourceDigest {
  std::string path;
  std::string sha256;
};

struct TaskSpec {
  std::string name;
  std::string type;
  std::string priority;
  double weight{};
  std::vector<std::string> joints;
};

struct ConstraintSpec {
  std::string name;
  std::string type;
  std::string priority;
  double weight{};
  std::vector<std::string> joints;
};

struct ProductionStaticConfig {
  std::string schema_version;
  std::string source_revision;
  std::vector<SourceDigest> source_digests;

  std::string base_frame;
  std::string left_end_effector_frame;
  std::string right_end_effector_frame;
  std::vector<double> left_tcp_offset_xyz;
  std::vector<double> right_tcp_offset_xyz;

  std::vector<std::string> joint_names;
  std::vector<std::string> active_joint_names;
  std::vector<std::string> masked_joint_names;
  std::vector<double> initial_positions;
  std::vector<double> lower_limits;
  std::vector<double> upper_limits;
  std::vector<double> velocity_limits;
  std::vector<double> posture_joint_weights;
  double posture_profile_default_weight{};

  bool use_sparsity{};
  bool rewrite_equalities{};
  double problem_regularization{};
  double solver_dt{};
  double control_rate_hz{};
  double control_dt_s{};
  double soft_limit_margin{};
  int maximum_iterations{};
  double soft_solve_time_budget_ms{};
  double position_tolerance_m{};
  double orientation_tolerance_rad{};
  double minimum_position_improvement_m{};
  double minimum_orientation_improvement_rad{};

  std::vector<TaskSpec> tasks;
  std::vector<ConstraintSpec> constraints;
  std::vector<std::string> disabled_features;
};

const ProductionStaticConfig &productionStaticConfig();

std::size_t jointIndex(const ProductionStaticConfig &config,
                       const std::string &joint_name);
bool isActiveJoint(const ProductionStaticConfig &config,
                   const std::string &joint_name);
double limitedLower(const ProductionStaticConfig &config,
                    std::size_t joint_index);
double limitedUpper(const ProductionStaticConfig &config,
                    std::size_t joint_index);

const TaskSpec &taskSpec(const ProductionStaticConfig &config,
                         const std::string &name);
const ConstraintSpec &constraintSpec(const ProductionStaticConfig &config,
                                     const std::string &name);

Json::Value productionStaticConfigJson();
std::string productionStaticConfigJsonText();

} // namespace motion_control_lab::baseline

#include <Eigen/Geometry>

#include <string>
#include <vector>

#include "contracts/presentation/ik_app_snapshot.hpp"
#include "placo/kinematics/kinematics_solver.h"
#include "placo/model/robot_wrapper.h"

namespace motion_control_lab::baseline {

enum class SolveStatus {
  Converged,
  Saturated,
  BestEffort,
};

struct CartesianError {
  double position_m{};
  double orientation_rad{};
};

struct BaselineSolveResult {
  bool accepted{true};
  SolveStatus status{SolveStatus::BestEffort};
  std::string status_name;
  std::string termination_reason;
  int iterations{};
  bool converged{};
  double frame_scale{1.0};
  double solve_time_ms{};
  double maximum_hard_violation{};

  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<std::string> saturated_joints;
  std::vector<ArmForwardKinematics> end_effector_forward_kinematics;
  std::vector<ArmForwardKinematics> tcp_forward_kinematics;
  std::vector<ArmTarget> internal_end_effector_targets;
  std::vector<ArmTargetError> end_effector_target_errors;
  std::vector<ArmTargetError> tcp_target_errors;
  SolverDebug solver_debug;
};

class BaselineSolver {
public:
  explicit BaselineSolver(const std::string &urdf_path);

  const std::vector<double> &positions() const;
  const std::vector<double> &velocities() const;
  Pose currentEndEffectorPose(ArmSide side);
  Pose currentTcpPose(ArmSide side);
  BaselineSolveResult solve(const std::vector<ArmTarget> &public_tcp_targets);

private:
  Pose framePose(placo::model::RobotWrapper &robot, ArmSide side) const;
  Pose tcpOffset(ArmSide side) const;
  void setAcceptedStateOnRobot(placo::model::RobotWrapper &robot) const;
  void updateTaskTargets(const std::vector<ArmTarget> &internal_targets);
  std::vector<ArmTarget>
  toInternalTargets(const std::vector<ArmTarget> &public_tcp_targets) const;

  const ProductionStaticConfig &config_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  placo::model::RobotWrapper robot_;
  placo::model::RobotWrapper fk_robot_;
  placo::kinematics::KinematicsSolver solver_;
  placo::kinematics::PositionTask *left_position_task_{nullptr};
  placo::kinematics::OrientationTask *left_orientation_task_{nullptr};
  placo::kinematics::PositionTask *right_position_task_{nullptr};
  placo::kinematics::OrientationTask *right_orientation_task_{nullptr};
};

const char *solveStatusName(SolveStatus status);

} // namespace motion_control_lab::baseline
