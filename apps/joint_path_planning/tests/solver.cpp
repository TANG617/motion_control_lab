#include "solver.hpp"
#include "planning.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace app = motion_control_lab::joint_path_planning;
namespace mcc = motion_control::core;

int main(int argc, char **argv) {
  if (argc != 3)
    throw std::runtime_error("expected URDF and scene paths");
  mcc::RobotModelDescription description;
  description.urdf_path = argv[1];
  description.kinematics_reference_frame =
      motion_control_lab::r1RobotConfig().base_frame;
  std::shared_ptr<const mcc::RobotModel> model;
  app::require(mcc::RobotModel::load(description, model));
  app::Options options;
  options.scene = app::readJson(argv[2]);
  const auto initial = app::initialState(*model, options);
  app::Solver solver(model, initial, false);
  const auto check = [](bool value, const char *message) {
    if (!value)
      throw std::runtime_error(message);
  };
  const auto &names = model->jointNames();
  // Reuse one solver across changed seeds, arm switches and repeated requests.
  for (int attempt = 0; attempt < 4; ++attempt) {
    const auto side = attempt % 2 ? motion_control_lab::ArmSide::Right
                                  : motion_control_lab::ArmSide::Left;
    const std::string prefix = motion_control_lab::armSideName(side);
    auto seed = initial;
    const auto joint =
        std::find(names.begin(), names.end(), prefix + "_arm_joint1") -
        names.begin();
    seed.joint_positions[joint] += attempt < 2 ? 0.04 : 0.08;
    const auto goal = solver.solveGoal(seed, side, solver.tcp(seed, side));
    check(goal.accepted, "reachable endpoint rejected");
    check(goal.diagnostics.optimization.task_scales.size() == 1,
          "expected one arm-local group");
    check(goal.diagnostics.optimization.task_scales[0].name ==
              prefix + "-cartesian-progress",
          "incorrect arm scale group");
    check(goal.diagnostics.position_errors.size() == 1 &&
              goal.diagnostics.orientation_errors.size() == 1,
          "expected position and orientation tasks");
    check(goal.diagnostics.posture_errors.size() == 1, "missing posture task");
    const auto &error = goal.diagnostics.posture_errors.front();
    check(error.role == mcc::PostureTaskRole::Regularization,
          "posture must not gate convergence");
    double expected_error = 0.0;
    for (std::size_t j = 0; j < goal.names.size(); ++j) {
      const auto full =
          std::find(names.begin(), names.end(), goal.names[j]) - names.begin();
      expected_error =
          std::max(expected_error,
                   std::abs(goal.positions[j] - initial.joint_positions[full]));
    }
    check(expected_error > 0.01,
          "probe did not move away from initial posture");
    check(std::abs(error.maximum_absolute_error_rad - expected_error) < 1e-10,
          "posture reference changed with request seed");
  }
  app::Solver whole(model, initial, true);
  for (const auto side : {motion_control_lab::ArmSide::Left,
                          motion_control_lab::ArmSide::Right}) {
    const auto held = side == motion_control_lab::ArmSide::Left
                          ? motion_control_lab::ArmSide::Right
                          : motion_control_lab::ArmSide::Left;
    auto target = whole.tcp(initial, side);
    target.translation().z() += 0.03;
    const auto goal = whole.solveGoal(initial, side, target);
    check(goal.accepted, "whole-body endpoint rejected");
    check(goal.names.size() == 16, "expected waist and both arms");
    check(goal.names[0] == "torso_pitch_joint" &&
              goal.names[1] == "torso_yaw_joint",
          "waist missing");
    check(goal.diagnostics.position_errors.size() == 2 &&
              goal.diagnostics.orientation_errors.size() == 2,
          "missing held tasks");
    auto end = initial;
    for (std::size_t i = 0; i < goal.names.size(); ++i)
      end.joint_positions[std::find(names.begin(), names.end(), goal.names[i]) -
                          names.begin()] = goal.positions[i];
    const auto actual = whole.tcp(end, held);
    const auto anchor = whole.tcp(initial, held);
    check((actual.translation() - anchor.translation()).norm() < 1e-5,
          "held TCP position drift");
    check(Eigen::AngleAxisd(anchor.linear().transpose() * actual.linear())
                  .angle() < 1e-4,
          "held TCP orientation drift");
    check(goal.pose_constraint.has_value(), "missing path constraint");
    check(goal.pose_constraint->model_root_T_target.isApprox(anchor, 1e-12),
          "anchor must use request start");
  }
  std::cout << "Single-arm posture and both 16-DOF held-TCP solves verified\n";
}
