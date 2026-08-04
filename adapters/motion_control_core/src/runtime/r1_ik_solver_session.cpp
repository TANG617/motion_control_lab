#include "runtime/r1_ik_solver_session.hpp"

#include "config/constants.hpp"

#include <stdexcept>

namespace motion_control_lab
{
namespace
{

const char * targetFrameForSide(ArmSide side)
{
  return side == ArmSide::Left ? kLeftTargetFrame : kRightTargetFrame;
}

ArmSide sideForTargetFrame(const std::string & frame_name)
{
  if (frame_name == kLeftTargetFrame) {
    return ArmSide::Left;
  }
  if (frame_name == kRightTargetFrame) {
    return ArmSide::Right;
  }
  throw std::runtime_error("Unknown R1 target frame: " + frame_name);
}

IkSolveStatusCode mapStatusCode(mcc::StatusCode code)
{
  switch (code) {
    case mcc::StatusCode::Ok:
      return IkSolveStatusCode::Ok;
    case mcc::StatusCode::InvalidInput:
      return IkSolveStatusCode::InvalidInput;
    case mcc::StatusCode::InvalidState:
      return IkSolveStatusCode::InvalidState;
    case mcc::StatusCode::InvalidTarget:
      return IkSolveStatusCode::InvalidTarget;
    case mcc::StatusCode::NotInitialized:
      return IkSolveStatusCode::NotInitialized;
    case mcc::StatusCode::Infeasible:
      return IkSolveStatusCode::Infeasible;
    case mcc::StatusCode::Saturated:
      return IkSolveStatusCode::Saturated;
    case mcc::StatusCode::BestEffort:
      return IkSolveStatusCode::BestEffort;
    case mcc::StatusCode::SolverError:
      return IkSolveStatusCode::SolverError;
  }
  return IkSolveStatusCode::SolverError;
}

}  // namespace

R1IkSolverSession::R1IkSolverSession(const std::string & urdf_path)
: joint_names_(r1JointNames()),
  positions_(r1InitialPositions()),
  velocities_(positions_.size(), 0.0)
{
  auto status = solver_.configureModel(
    makeR1ModelDescription(urdf_path),
    joint_names_,
    makeR1IkConfig(),
    positions_);
  if (!status.ok()) {
    throw std::runtime_error("Failed to configure solver model: " + status.message);
  }

  mcc::PositionTaskConfig position_task;
  position_task.priority = mcc::TaskPriority::Hard;
  mcc::OrientationTaskConfig orientation_task;
  orientation_task.priority = mcc::TaskPriority::Hard;
  for (const char * frame : {kLeftTargetFrame, kRightTargetFrame}) {
    status = solver_.addPositionTask(frame, position_task);
    if (status.ok()) {
      status = solver_.addOrientationTask(frame, orientation_task);
    }
    if (!status.ok()) {
      throw std::runtime_error(
        std::string("Failed to register R1 tasks for ") + frame + ": " + status.message);
    }
  }
  status = solver_.finalizeConfiguration();
  if (!status.ok()) {
    throw std::runtime_error("Failed to finalize solver configuration: " + status.message);
  }
}

std::string_view R1IkSolverSession::backendId() const
{
  return "motion_control_core";
}

const JointNames & R1IkSolverSession::jointNames() const
{
  return joint_names_;
}

const std::vector<double> & R1IkSolverSession::positions() const
{
  return positions_;
}

const std::vector<double> & R1IkSolverSession::velocities() const
{
  return velocities_;
}

void R1IkSolverSession::reset(const JointState & state)
{
  if (state.names != joint_names_) {
    throw std::invalid_argument("R1 IK reset joint names do not match the backend joint order");
  }
  if (state.positions.size() != joint_names_.size()) {
    throw std::invalid_argument("R1 IK reset positions do not match the backend joint count");
  }
  if (!state.velocities.empty() && state.velocities.size() != joint_names_.size()) {
    throw std::invalid_argument("R1 IK reset velocities do not match the backend joint count");
  }

  positions_ = state.positions;
  velocities_ = state.velocities.empty()
    ? std::vector<double>(joint_names_.size(), 0.0)
    : state.velocities;
}

Pose R1IkSolverSession::currentTargetPose(ArmSide side) const
{
  mcc::ForwardKinematicsSolution solution;
  mcc::ForwardKinematicsDiagnostics diagnostics;
  const auto status = solver_.solveForwardKinematics(
    positions_,
    solution,
    diagnostics,
    mcc::JointPositionValidation::LimitChecked);
  if (!status.ok()) {
    throw std::runtime_error("FK failed: " + status.message);
  }
  return findPose(solution.poses, targetFrameForSide(side)).target_pose;
}

std::vector<ArmTarget> R1IkSolverSession::currentTargetPoses(
  const std::vector<ArmSide> & sides) const
{
  mcc::ForwardKinematicsSolution solution;
  mcc::ForwardKinematicsDiagnostics diagnostics;
  const auto status = solver_.solveForwardKinematics(
    positions_,
    solution,
    diagnostics,
    mcc::JointPositionValidation::LimitChecked);
  if (!status.ok()) {
    throw std::runtime_error("FK failed: " + status.message);
  }

  std::vector<ArmTarget> targets;
  targets.reserve(sides.size());
  for (const auto side : sides) {
    targets.push_back(ArmTarget{
      side,
      findPose(solution.poses, targetFrameForSide(side)).target_pose});
  }
  return targets;
}

IkSolveResult R1IkSolverSession::solveTargets(
  const std::vector<ArmTarget> & targets,
  double dt)
{
  auto complete_targets = currentTargetPoses({ArmSide::Left, ArmSide::Right});
  for (const auto & requested_target : targets) {
    auto & complete_target = requested_target.side == ArmSide::Left ?
      complete_targets[0] : complete_targets[1];
    complete_target = requested_target;
  }

  mcc::InverseKinematicsRequest request;
  request.current_state.positions = positions_;
  request.current_state.velocities = velocities_;
  request.dt = dt;
  request.seed_positions = positions_;
  request.targets.reserve(complete_targets.size());
  for (const auto & target : complete_targets) {
    mcc::EndEffectorTarget core_target;
    core_target.frame_name = targetFrameForSide(target.side);
    core_target.target_pose = target.target_pose;
    request.targets.push_back(core_target);
  }

  mcc::InverseKinematicsSolution core_solution;
  mcc::InverseKinematicsDiagnostics core_diagnostics;
  const auto core_status = solver_.solveInverseKinematics(
    request,
    core_solution,
    core_diagnostics);
  if (core_status.ok() && core_solution.joints.positions.size() == joint_names_.size()) {
    positions_ = core_solution.joints.positions;
    if (core_solution.joints.velocities.size() == joint_names_.size()) {
      velocities_ = core_solution.joints.velocities;
    } else {
      velocities_.assign(joint_names_.size(), 0.0);
    }
  }

  IkSolveResult result;
  result.status = IkSolveStatus{mapStatusCode(core_status.code), core_status.message};
  result.joint_state = JointState{joint_names_, positions_, velocities_};
  result.solved_targets.reserve(core_solution.solved_poses.size());
  for (const auto & solved_pose : core_solution.solved_poses) {
    result.solved_targets.push_back(
      ArmTarget{sideForTargetFrame(solved_pose.frame_name), solved_pose.target_pose});
  }
  result.diagnostics.saturated_joints = core_diagnostics.saturated_joints;
  result.diagnostics.iterations = core_diagnostics.iterations;
  result.diagnostics.converged = core_diagnostics.converged;
  result.diagnostics.solve_time_ms = core_diagnostics.solve_time_ms;
  result.diagnostics.errors.reserve(core_diagnostics.errors.size());
  for (const auto & error : core_diagnostics.errors) {
    result.diagnostics.errors.push_back(ArmTargetError{
      sideForTargetFrame(error.frame_name),
      error.position_m,
      error.orientation_rad});
  }
  return result;
}

}  // namespace motion_control_lab
