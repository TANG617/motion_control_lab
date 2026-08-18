#pragma once

#include "baseline_config.hpp"

#include <Eigen/Geometry>

#include <string>
#include <vector>

#include "placo/kinematics/kinematics_solver.h"
#include "placo/model/robot_wrapper.h"
#include "runtime/interactive_types.hpp"

namespace motion_control_lab::baseline
{

enum class SolveStatus
{
  Converged,
  Saturated,
  BestEffort,
};

struct CartesianError
{
  double position_m{};
  double orientation_rad{};
};

struct BaselineSolveResult
{
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

class BaselineSolver
{
public:
  explicit BaselineSolver(const std::string & urdf_path);

  const std::vector<double> & positions() const;
  const std::vector<double> & velocities() const;
  Pose currentEndEffectorPose(ArmSide side);
  Pose currentTcpPose(ArmSide side);
  BaselineSolveResult solve(const std::vector<ArmTarget> & public_tcp_targets);

private:
  Pose framePose(placo::model::RobotWrapper & robot, ArmSide side) const;
  Pose tcpOffset(ArmSide side) const;
  void setAcceptedStateOnRobot(placo::model::RobotWrapper & robot) const;
  void updateTaskTargets(const std::vector<ArmTarget> & internal_targets);
  std::vector<ArmTarget> toInternalTargets(const std::vector<ArmTarget> & public_tcp_targets) const;

  const ProductionStaticConfig & config_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  placo::model::RobotWrapper robot_;
  placo::model::RobotWrapper fk_robot_;
  placo::kinematics::KinematicsSolver solver_;
  placo::kinematics::PositionTask * left_position_task_{nullptr};
  placo::kinematics::OrientationTask * left_orientation_task_{nullptr};
  placo::kinematics::PositionTask * right_position_task_{nullptr};
  placo::kinematics::OrientationTask * right_orientation_task_{nullptr};
};

const char * solveStatusName(SolveStatus status);

} // namespace motion_control_lab::baseline
