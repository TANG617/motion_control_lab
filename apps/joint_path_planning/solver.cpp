#include "solver.hpp"
#include <algorithm>
namespace motion_control_lab::joint_path_planning {
namespace {
int index(ArmSide side) { return side == ArmSide::Left ? 0 : 1; }
std::string frame(ArmSide side) {
  return side == ArmSide::Left ? r1RobotConfig().left_end_effector_frame
                               : r1RobotConfig().right_end_effector_frame;
}
Pose offset(ArmSide side) {
  return side == ArmSide::Left ? r1RobotConfig().left_tcp_offset
                               : r1RobotConfig().right_tcp_offset;
}
} // namespace
Solver::Solver(std::shared_ptr<const mcc::RobotModel> model,
               const mcc::RobotState &initial_state, bool whole_body)
    : whole_body_(whole_body) {
  for (int a = 0; a < 2; ++a) {
    const auto side = a == 0 ? ArmSide::Left : ArmSide::Right;
    mcc::JointNames names;
    if (whole_body_) {
      names = {"torso_pitch_joint", "torso_yaw_joint"};
      for (const auto *prefix : {"left", "right"})
        for (int j = 1; j <= 7; ++j)
          names.push_back(std::string(prefix) + "_arm_joint" +
                          std::to_string(j));
    } else {
      for (int j = 1; j <= 7; ++j)
        names.push_back(std::string(armSideName(side)) + "_arm_joint" +
                        std::to_string(j));
    }
    mcc::KinematicsSolverBuilder b;
    mcc::KinematicsSolverConfig c;
    c.execution = mcc::TargetSolveOptions{};
    if (whole_body_) {
      c.convergence.position_tolerance_m = 1e-5;
      c.convergence.orientation_tolerance_rad = 1e-4;
    }
    require(b.configure(model, names, c));
    mcc::TaskScaleGroupConfig scale;
    scale.name = std::string(armSideName(side)) + "-cartesian-progress";
    scale.progress_weight = kIkProgressWeight;
    require(b.addTaskScaleGroup(scale, scales_[a]));
    mcc::PositionTaskConfig p;
    mcc::OrientationTaskConfig r;
    p.enforcement = mcc::ScaledEnforcement{scales_[a]};
    r.enforcement = mcc::ScaledEnforcement{scales_[a]};
    require(b.addPositionTask(frame(side), p, positions_[a]));
    require(b.addOrientationTask(frame(side), r, rotations_[a]));
    if (whole_body_) {
      const auto held_side = a == 0 ? ArmSide::Right : ArmSide::Left;
      mcc::PositionTaskConfig hp;
      mcc::OrientationTaskConfig hr;
      hp.enforcement = mcc::HardEnforcement{};
      hr.enforcement = mcc::HardEnforcement{};
      require(b.addPositionTask(frame(held_side), hp, held_positions_[a]));
      require(b.addOrientationTask(frame(held_side), hr, held_rotations_[a]));
    }
    mcc::PostureTaskConfig posture;
    posture.name = std::string(armSideName(side)) + "-initial-posture";
    posture.enforcement =
        mcc::squaredL2Penalty(kIkPostureWeight, static_cast<int>(names.size()));
    require(b.addPostureTask(posture, postures_[a]));
    require(b.setPostureConvergence(postures_[a], std::nullopt));
    initial_postures_[a].resize(names.size());
    for (std::size_t j = 0; j < names.size(); ++j) {
      const auto index = std::find(model->jointNames().begin(),
                                   model->jointNames().end(), names[j]) -
                         model->jointNames().begin();
      initial_postures_[a][j] = initial_state.joint_positions[index];
    }
    require(b.finalize(ik_[a]));
  }
  mcc::KinematicsSolverBuilder b;
  mcc::KinematicsSolverConfig c;
  require(b.configure(model, ik_[0].activeJointNames(), c));
  require(b.finalize(fk_));
}
NamedGoal Solver::solveGoal(const mcc::RobotState &state, ArmSide side,
                            const Pose &tcp_goal) {
  const int a = index(side);
  const Pose ee = tcp_goal * offset(side).inverse();
  mcc::InverseKinematicsRequest request;
  request.state = state;
  request.reference_frame_name = r1RobotConfig().base_frame;
  request.position_targets = {{positions_[a], ee.translation(), true}};
  request.orientation_targets = {{rotations_[a], ee.rotation(), true}};
  request.posture_targets = {{postures_[a], initial_postures_[a], true}};
  mcc::InverseKinematicsSolution result;
  NamedGoal goal;
  if (whole_body_) {
    const auto held_side =
        side == ArmSide::Left ? ArmSide::Right : ArmSide::Left;
    const Pose anchor = tcp(state, held_side);
    const Pose held_ee = anchor * offset(held_side).inverse();
    request.position_targets.push_back(
        {held_positions_[a], held_ee.translation(), true});
    request.orientation_targets.push_back(
        {held_rotations_[a], held_ee.rotation(), true});
    mcc::JointPathPoseConstraint c;
    c.frame = frame(held_side);
    c.frame_T_control = offset(held_side);
    c.model_root_T_target = anchor;
    goal.pose_constraint = c;
  }
  require(ik_[a].solveInverseKinematics(request, result, goal.diagnostics));
  goal.accepted =
      mcc::isAccepted(result.disposition) && goal.diagnostics.converged;
  for (const auto &e : goal.diagnostics.position_errors)
    goal.accepted &= std::isfinite(e.norm_m) && e.norm_m <= 1e-3;
  for (const auto &e : goal.diagnostics.orientation_errors)
    goal.accepted &= std::isfinite(e.norm_rad) && e.norm_rad <= 1e-2;
  goal.names = ik_[a].activeJointNames();
  if (goal.accepted)
    goal.positions = result.joint_positions;
  return goal;
}
Pose Solver::tcp(const mcc::RobotState &state, ArmSide side,
                 bool check_limits) {
  mcc::ForwardKinematicsRequest request;
  request.state = state;
  request.frame_names = {frame(side)};
  request.reference_frame_name = r1RobotConfig().base_frame;
  mcc::ForwardKinematicsSolution out;
  mcc::ForwardKinematicsDiagnostics d;
  if (!check_limits)
    request.joint_position_validation =
        mcc::JointPositionValidation::UncheckedJointPositionLimits;
  require(fk_.computeForwardKinematics(request, out, d));
  return out.poses.front().pose * offset(side);
}
Pose configuredGoal(const Json::Value &v, const Pose &current) {
  Pose p = current;
  if (v.isMember("xyz"))
    for (int i = 0; i < 3; ++i)
      p.translation()[i] = v["xyz"][i].asDouble();
  if (v.isMember("delta"))
    for (int i = 0; i < 3; ++i)
      p.translation()[i] += v["delta"][i].asDouble();
  if (v.isMember("quaternion_xyzw")) {
    const auto &q = v["quaternion_xyzw"];
    p.linear() = Eigen::Quaterniond(q[3].asDouble(), q[0].asDouble(),
                                    q[1].asDouble(), q[2].asDouble())
                     .toRotationMatrix();
  }
  return p;
}
} // namespace motion_control_lab::joint_path_planning
