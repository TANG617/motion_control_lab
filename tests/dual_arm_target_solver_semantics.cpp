#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "../apps/target/solver.hpp"
#include "placo/model/robot_wrapper.h"
#include "placo/problem/qp_error.h"

namespace mcl = motion_control_lab;
using motion_control_lab::target::MccTargetSolver;
using motion_control_lab::target::MccBackend;
using motion_control_lab::target::PlacoTargetSolver;
using motion_control_lab::target::TargetSolveResult;

namespace
{

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

const motion_control_lab::Pose & poseForSide(
  const std::vector<motion_control_lab::ArmForwardKinematics> & poses,
  motion_control_lab::ArmSide side)
{
  const auto found = std::find_if(
    poses.begin(), poses.end(),
    [side](const motion_control_lab::ArmForwardKinematics & pose) { return pose.side == side; });
  require(found != poses.end(), "missing arm FK");
  return found->pose;
}

bool isTargetTerminationReason(const std::string & reason)
{
  return reason == "converged" || reason == "no-progress" || reason == "soft-time-budget" ||
         reason == "iteration-budget";
}

void validatePositionLimits(
  const TargetSolveResult & result, placo::model::RobotWrapper & model,
  const motion_control_lab::R1RobotConfig & robot)
{
  require(result.positions.size() == robot.joint_names.size(), "position size mismatch");
  require(result.velocities.size() == robot.joint_names.size(), "velocity size mismatch");
  for (std::size_t index = 0; index < robot.joint_names.size(); ++index) {
    const auto position_limits = model.get_joint_limits(robot.joint_names[index]);
    require(
      result.positions[index] >= position_limits.first +
      motion_control_lab::target::AlgorithmOptions{}.joint_position_margin_rad - 1.0e-8 &&
        result.positions[index] <= position_limits.second -
          motion_control_lab::target::AlgorithmOptions{}.joint_position_margin_rad + 1.0e-8,
      robot.joint_names[index] + " violated its position limit margin");
    require(result.velocities[index] == 0.0, "TargetSolve output velocity must be zero");
  }
}

template <typename Solver>
void exerciseTargetSolver(
  Solver & solver, placo::model::RobotWrapper & model,
  const motion_control_lab::R1RobotConfig & robot, const std::string & expected_label,
  const std::string & expected_backend)
{
  std::vector<motion_control_lab::ArmTarget> targets{
    {motion_control_lab::ArmSide::Left, solver.currentPose(motion_control_lab::ArmSide::Left)},
    {motion_control_lab::ArmSide::Right, solver.currentPose(motion_control_lab::ArmSide::Right)}};

  const auto initial = solver.solve(targets);
  require(initial.accepted, "initial target was not accepted");
  require(initial.solver_debug.label == expected_label, "unexpected solver label");
  require(initial.solver_debug.backend == expected_backend, "unexpected QP backend");
  require(
    initial.iterations >= 1 &&
      initial.iterations <= motion_control_lab::target::AlgorithmOptions{}.maximum_iterations,
    "initial target iteration count is invalid");
  require(
    isTargetTerminationReason(initial.solver_debug.termination_reason),
    "initial target has an invalid termination reason");
  validatePositionLimits(initial, model, robot);

  targets.front().target_pose.translation().x() += 0.005;
  const auto stepped = solver.solve(targets);
  require(stepped.accepted, "5 mm target was not accepted");
  require(
    stepped.iterations >= 1 &&
      stepped.iterations <= motion_control_lab::target::AlgorithmOptions{}.maximum_iterations,
    "5 mm target iteration count is invalid");
  require(
    isTargetTerminationReason(stepped.solver_debug.termination_reason),
    "5 mm target has an invalid termination reason");
  require(stepped.solve_time_ms >= 0.0, "solve time must be non-negative");
  require(stepped.target_errors.size() == 2U, "missing Cartesian errors");
  for (const auto & error : stepped.target_errors) {
    require(
      std::isfinite(error.position_m) && std::isfinite(error.orientation_rad),
      "Cartesian error must be finite");
  }
  validatePositionLimits(stepped, model, robot);
  require(
    poseForSide(stepped.forward_kinematics, motion_control_lab::ArmSide::Left)
      .matrix()
      .isApprox(solver.currentPose(motion_control_lab::ArmSide::Left).matrix(), 1.0e-10),
    "left FK is not coherent with accepted state");
  require(
    poseForSide(stepped.forward_kinematics, motion_control_lab::ArmSide::Right)
      .matrix()
      .isApprox(solver.currentPose(motion_control_lab::ArmSide::Right).matrix(), 1.0e-10),
    "right FK is not coherent with accepted state");
}

void validateMccInfeasibleHold(MccTargetSolver & solver)
{
  std::vector<motion_control_lab::ArmTarget> targets{
    {motion_control_lab::ArmSide::Left, solver.currentPose(motion_control_lab::ArmSide::Left)},
    {motion_control_lab::ArmSide::Right, solver.currentPose(motion_control_lab::ArmSide::Right)}};
  targets.front().target_pose.translation().x() += 100.0;
  const auto accepted_positions = solver.positions();
  const auto rejected = solver.solve(targets);
  require(!rejected.accepted, "MCC did not reject the infeasible target");
  require(
    solver.positions() == accepted_positions, "MCC changed accepted state after infeasible target");
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    require(argc == 2, "usage: dual_arm_target_solver_semantics <urdf>");
    const std::string urdf_path = argv[1];
    const auto & robot = motion_control_lab::r1RobotConfig();

    placo::model::RobotWrapper limit_model(
      urdf_path,
      placo::model::RobotWrapper::IGNORE_COLLISIONS | placo::model::RobotWrapper::IGNORE_GEOMETRY);
    const mcl::target::AlgorithmOptions algorithm;
    MccTargetSolver mcc_proxqp_solver(urdf_path, robot, MccBackend::Proxqp, algorithm);
    MccTargetSolver mcc_eiquadprog_solver(urdf_path, robot, MccBackend::Eiquadprog, algorithm);
    PlacoTargetSolver placo_solver(urdf_path, robot, algorithm);
    require(
      mcc_proxqp_solver.currentPose(motion_control_lab::ArmSide::Left)
        .matrix()
        .isApprox(placo_solver.currentPose(motion_control_lab::ArmSide::Left).matrix(), 1.0e-6),
      "MCC/ProxQP and PlaCo left initial FK differ");
    require(
      mcc_proxqp_solver.currentPose(motion_control_lab::ArmSide::Right)
        .matrix()
        .isApprox(placo_solver.currentPose(motion_control_lab::ArmSide::Right).matrix(), 1.0e-6),
      "MCC/ProxQP and PlaCo right initial FK differ");
    require(
      mcc_eiquadprog_solver.currentPose(motion_control_lab::ArmSide::Left)
        .matrix()
        .isApprox(placo_solver.currentPose(motion_control_lab::ArmSide::Left).matrix(), 1.0e-6),
      "MCC/eiquadprog and PlaCo left initial FK differ");
    require(
      mcc_eiquadprog_solver.currentPose(motion_control_lab::ArmSide::Right)
        .matrix()
        .isApprox(placo_solver.currentPose(motion_control_lab::ArmSide::Right).matrix(), 1.0e-6),
      "MCC/eiquadprog and PlaCo right initial FK differ");

    exerciseTargetSolver(mcc_proxqp_solver, limit_model, robot, "MCC/ProxQP", "proxqp");
    exerciseTargetSolver(mcc_eiquadprog_solver, limit_model, robot, "MCC/eiquadprog", "eiquadprog");
    exerciseTargetSolver(placo_solver, limit_model, robot, "PlaCo/eiquadprog", "eiquadprog");

    validateMccInfeasibleHold(mcc_proxqp_solver);
    validateMccInfeasibleHold(mcc_eiquadprog_solver);

    std::vector<motion_control_lab::ArmTarget> infeasible_targets{
      {motion_control_lab::ArmSide::Left,
       placo_solver.currentPose(motion_control_lab::ArmSide::Left)},
      {motion_control_lab::ArmSide::Right,
       placo_solver.currentPose(motion_control_lab::ArmSide::Right)}};
    infeasible_targets.front().target_pose.translation().x() += 100.0;
    bool placo_threw_qp_error = false;
    try {
      (void)placo_solver.solve(infeasible_targets);
    } catch (const placo::problem::QPError &) {
      placo_threw_qp_error = true;
    }
    require(placo_threw_qp_error, "PlaCo QPError was not propagated");
    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "dual_arm_target_solver_semantics: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
