#include "baseline_config.hpp"
#include "baseline_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace baseline = motion_control_lab::baseline;
namespace mcl = motion_control_lab;

namespace
{

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

const mcl::Pose & poseForSide(const std::vector<mcl::ArmForwardKinematics> & poses,
                              mcl::ArmSide side)
{
  const auto found = std::find_if(poses.begin(), poses.end(),
                                  [side](const auto & pose) { return pose.side == side; });
  require(found != poses.end(), "missing arm pose");
  return found->pose;
}

void validateMaskedJoints(const baseline::ProductionStaticConfig & config,
                          const std::vector<double> & positions)
{
  for (const auto & joint_name : config.masked_joint_names) {
    const auto index = baseline::jointIndex(config, joint_name);
    require(positions[index] == config.initial_positions[index],
            joint_name + " did not remain latched");
  }
}

void validateActiveLimits(const baseline::ProductionStaticConfig & config,
                          const std::vector<double> & positions)
{
  for (const auto & joint_name : config.active_joint_names) {
    const auto index = baseline::jointIndex(config, joint_name);
    require(positions[index] >= baseline::limitedLower(config, index) - 1.0e-8,
            joint_name + " violated the lower production margin");
    require(positions[index] <= baseline::limitedUpper(config, index) + 1.0e-8,
            joint_name + " violated the upper production margin");
  }
}

} // namespace

int main(int argc, char ** argv)
{
  try {
    require(argc == 2, "usage: baseline_solver_semantics <urdf>");
    const auto & config = baseline::productionStaticConfig();
    baseline::BaselineSolver solver(argv[1]);

    std::vector<mcl::ArmTarget> targets{
        {mcl::ArmSide::Left, solver.currentTcpPose(mcl::ArmSide::Left)},
        {mcl::ArmSide::Right, solver.currentTcpPose(mcl::ArmSide::Right)},
    };
    const auto initial_positions = solver.positions();
    const auto initial = solver.solve(targets);
    require(initial.accepted, "initial target was not accepted");
    require(initial.status == baseline::SolveStatus::Converged, "initial solve did not converge");
    require(initial.iterations == 0, "initial target must use the zero-iteration path");
    require(initial.positions == initial_positions, "zero-iteration solve changed joint state");
    require(initial.frame_scale == 1.0, "zero-iteration scale must be one");
    validateMaskedJoints(config, initial.positions);
    validateActiveLimits(config, initial.positions);

    for (std::size_t index = 0; index < targets.size(); ++index) {
      const auto side = index == 0U ? mcl::ArmSide::Left : mcl::ArmSide::Right;
      const auto reconstructed =
          initial.internal_end_effector_targets[index].target_pose *
          (solver.currentTcpPose(side).inverse() * solver.currentEndEffectorPose(side)).inverse();
      require(reconstructed.matrix().isApprox(targets[index].target_pose.matrix(), 1.0e-10),
              "TCP to end-effector target conversion mismatch");
    }

    targets[0].target_pose.translation().x() += 0.005;
    targets[1].target_pose.translation().x() += 0.005;
    const auto before_step = solver.positions();
    const auto stepped = solver.solve(targets);
    require(stepped.accepted, "reachable target was not accepted");
    require(stepped.iterations >= 1 && stepped.iterations <= 20, "iteration count mismatch");
    require(stepped.frame_scale >= 0.0 && stepped.frame_scale <= 1.0, "frame scale out of range");
    require(stepped.end_effector_target_errors.size() == 2U, "missing EE errors");
    require(stepped.tcp_target_errors.size() == 2U, "missing TCP errors");
    require(stepped.maximum_hard_violation <= 1.0e-8, "hard limit violation is too large");
    validateMaskedJoints(config, stepped.positions);
    validateActiveLimits(config, stepped.positions);
    for (const auto & joint_name : config.active_joint_names) {
      const auto index = baseline::jointIndex(config, joint_name);
      const double expected_velocity =
          (stepped.positions[index] - before_step[index]) / config.control_dt_s;
      require(std::fabs(stepped.velocities[index] - expected_velocity) <= 1.0e-10,
              joint_name + " velocity does not use the 100 Hz control dt");
    }
    require(poseForSide(stepped.end_effector_forward_kinematics, mcl::ArmSide::Left)
                .matrix()
                .isApprox(solver.currentEndEffectorPose(mcl::ArmSide::Left).matrix(), 1.0e-10),
            "accepted left EE FK is incoherent");
    require(poseForSide(stepped.tcp_forward_kinematics, mcl::ArmSide::Right)
                .matrix()
                .isApprox(solver.currentTcpPose(mcl::ArmSide::Right).matrix(), 1.0e-10),
            "accepted right TCP FK is incoherent");

    baseline::BaselineSolver best_effort_solver(argv[1]);
    std::vector<mcl::ArmTarget> orientation_targets{
        {mcl::ArmSide::Left, best_effort_solver.currentTcpPose(mcl::ArmSide::Left)},
        {mcl::ArmSide::Right, best_effort_solver.currentTcpPose(mcl::ArmSide::Right)},
    };
    for (auto & target : orientation_targets) {
      target.target_pose.linear() =
          Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
          target.target_pose.rotation();
    }
    const auto best_effort = best_effort_solver.solve(orientation_targets);
    require(best_effort.accepted, "successful BestEffort solve was not accepted");
    require(best_effort.status == baseline::SolveStatus::BestEffort,
            "orientation-only target did not exercise BestEffort acceptance: " +
                best_effort.status_name + ", scale=" + std::to_string(best_effort.frame_scale) +
                ", saturated=" + std::to_string(best_effort.saturated_joints.size()));
    require(!best_effort.converged, "BestEffort target unexpectedly converged");
    require(best_effort.iterations >= 1 && best_effort.iterations <= config.maximum_iterations,
            "BestEffort iteration budget mismatch");
    require(best_effort.termination_reason == "no-progress" ||
                best_effort.termination_reason == "soft-time-budget" ||
                best_effort.termination_reason == "iteration-budget",
            "BestEffort termination reason is not a production target_solve exit");
    validateMaskedJoints(config, best_effort.positions);
    validateActiveLimits(config, best_effort.positions);

    auto far_targets = targets;
    far_targets[0].target_pose.translation().x() += 5.0;
    far_targets[1].target_pose.translation().x() += 5.0;
    const auto far = solver.solve(far_targets);
    require(far.accepted, "successful best-effort/scaled solve was not accepted");
    require(!far.converged, "far target unexpectedly converged");
    require(far.status == baseline::SolveStatus::Saturated ||
                far.status == baseline::SolveStatus::BestEffort,
            "far target has the wrong production acceptance status");
    require(far.frame_scale < 0.999, "far target did not exercise scaled position tasks");
    validateMaskedJoints(config, far.positions);
    validateActiveLimits(config, far.positions);
    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "baseline_solver_semantics: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
