#pragma once
#include "motion_control_core/kinematics/solver.hpp"
#include "motion_control_core/planning/joint/constraints.hpp"
#include "options.hpp"
#include <array>
namespace motion_control_lab::joint_path_planning {
struct NamedGoal {
  mcc::JointNames names;
  Eigen::VectorXd positions;
  bool accepted{false};
  std::optional<mcc::JointPathPoseConstraint> pose_constraint;
  mcc::InverseKinematicsDiagnostics diagnostics;
};
class Solver {
public:
  Solver(std::shared_ptr<const mcc::RobotModel> model,
         const mcc::RobotState &initial_state, bool whole_body = true);
  NamedGoal solveGoal(const mcc::RobotState &, ArmSide, const Pose &tcp);
  Pose tcp(const mcc::RobotState &, ArmSide, bool check_limits = true);

private:
  bool whole_body_;
  std::array<mcc::PositionTaskHandle, 2> held_positions_;
  std::array<mcc::OrientationTaskHandle, 2> held_rotations_;
  std::array<mcc::KinematicsSolver, 2> ik_;
  mcc::KinematicsSolver fk_;
  std::array<mcc::PositionTaskHandle, 2> positions_;
  std::array<mcc::OrientationTaskHandle, 2> rotations_;
  std::array<mcc::TaskScaleGroupHandle, 2> scales_;
  std::array<mcc::PostureTaskHandle, 2> postures_;
  std::array<Eigen::VectorXd, 2> initial_postures_;
};
Pose configuredGoal(const Json::Value &, const Pose &current);
} // namespace motion_control_lab::joint_path_planning
