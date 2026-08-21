#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "../apps/servo_step/solver.hpp"
#include "placo/model/robot_wrapper.h"

namespace mcl = motion_control_lab;
using motion_control_lab::servo_step::MccBackend;
using motion_control_lab::servo_step::MccServoSolver;
using motion_control_lab::servo_step::PlacoServoSolver;
using motion_control_lab::servo_step::ServoSolveResult;

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

void validatePositionAndVelocityLimits(
  const ServoSolveResult & result, placo::model::RobotWrapper & model,
  const motion_control_lab::R1RobotConfig & robot)
{
  require(result.positions.size() == robot.joint_names.size(), "position size mismatch");
  require(result.velocities.size() == robot.joint_names.size(), "velocity size mismatch");
  for (std::size_t index = 0; index < robot.joint_names.size(); ++index) {
    const auto & joint_name = robot.joint_names[index];
    const auto position_limits = model.get_joint_limits(joint_name);
    require(
      result.positions[index] >= position_limits.first +
          mcl::servo_step::AlgorithmOptions{}.joint_position_margin_rad - 1.0e-8 &&
        result.positions[index] <= position_limits.second -
          mcl::servo_step::AlgorithmOptions{}.joint_position_margin_rad + 1.0e-8,
      joint_name + " violated its position limit margin");
    const int velocity_index = model.get_joint_v_offset(joint_name);
    require(velocity_index >= 0, joint_name + " has no velocity index");
    const double velocity_limit = model.model.velocityLimit[velocity_index];
    require(
      std::abs(result.velocities[index]) <= velocity_limit + 1.0e-8,
      joint_name + " violated its velocity limit");
  }
}

template <typename Solver>
void exerciseServoSolver(
  Solver & solver, placo::model::RobotWrapper & model,
  const motion_control_lab::R1RobotConfig & robot, double rate_hz,
  const std::string & expected_label, const std::string & expected_backend)
{
  std::vector<motion_control_lab::ArmTarget> targets{
    {motion_control_lab::ArmSide::Left, solver.currentPose(motion_control_lab::ArmSide::Left)},
    {motion_control_lab::ArmSide::Right, solver.currentPose(motion_control_lab::ArmSide::Right)}};

  const auto initial = solver.solve(targets);
  require(initial.solver_debug.label == expected_label, "unexpected solver label");
  require(initial.solver_debug.backend == expected_backend, "unexpected QP backend");
  require(initial.iterations == 1, "ServoStep must perform exactly one iteration");
  require(initial.solve_time_ms >= 0.0, "solve time must be non-negative");
  require(initial.target_errors.size() == 2U, "missing Cartesian errors");
  for (const auto & error : initial.target_errors) {
    require(
      std::isfinite(error.position_m) && std::isfinite(error.orientation_rad),
      "Cartesian error must be finite");
  }
  validatePositionAndVelocityLimits(initial, model, robot);
  require(
    poseForSide(initial.forward_kinematics, motion_control_lab::ArmSide::Left)
      .matrix()
      .isApprox(solver.currentPose(motion_control_lab::ArmSide::Left).matrix(), 1.0e-10),
    "left FK is not coherent with accepted state");
  require(
    poseForSide(initial.forward_kinematics, motion_control_lab::ArmSide::Right)
      .matrix()
      .isApprox(solver.currentPose(motion_control_lab::ArmSide::Right).matrix(), 1.0e-10),
    "right FK is not coherent with accepted state");

  const auto previous_positions = solver.positions();
  targets.front().target_pose.translation().x() += 0.005;
  const auto stepped = solver.solve(targets);
  validatePositionAndVelocityLimits(stepped, model, robot);
  require(stepped.iterations == 1, "5 mm ServoStep used more than one iteration");
  for (std::size_t index = 0; index < stepped.positions.size(); ++index) {
    const double expected_velocity =
      (stepped.positions[index] - previous_positions[index]) * rate_hz;
    require(
      std::abs(stepped.velocities[index] - expected_velocity) <= 1.0e-6,
      robot.joint_names[index] + " velocity is not delta/dt");
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    require(argc == 2, "usage: dual_arm_servo_solver_semantics <urdf>");
    const std::string urdf_path = argv[1];
    constexpr double rate_hz = 20.0;
    const auto & robot = motion_control_lab::r1RobotConfig();

    placo::model::RobotWrapper limit_model(
      urdf_path,
      placo::model::RobotWrapper::IGNORE_COLLISIONS | placo::model::RobotWrapper::IGNORE_GEOMETRY);
    const mcl::servo_step::AlgorithmOptions algorithm;
    MccServoSolver mcc_proxqp_solver(
      urdf_path, rate_hz, robot, MccBackend::Proxqp, algorithm);
    MccServoSolver mcc_eiquadprog_solver(
      urdf_path, rate_hz, robot, MccBackend::Eiquadprog, algorithm);
    PlacoServoSolver placo_solver(urdf_path, rate_hz, robot, algorithm);
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

    exerciseServoSolver(
      mcc_proxqp_solver, limit_model, robot, rate_hz, "MCC/ProxQP", "proxqp");
    exerciseServoSolver(
      mcc_eiquadprog_solver, limit_model, robot, rate_hz, "MCC/eiquadprog", "eiquadprog");
    exerciseServoSolver(
      placo_solver, limit_model, robot, rate_hz, "PlaCo/eiquadprog", "eiquadprog");
    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "dual_arm_servo_solver_semantics: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
