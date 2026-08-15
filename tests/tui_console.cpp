#include "config/interactive_ik_options.hpp"
#include "runtime/interactive_types.hpp"
#include "teleop/tui_teleop_source.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <thread>

namespace
{

constexpr std::string_view kExpectedException = "intentional TUI exception";

int run(bool throw_after_render, bool fault_hold)
{
  namespace mcl = motion_control_lab;

  mcl::TuiTeleopOptions options;
  options.side = "left";
  options.step_m = 0.005;
  options.min_step_m = 0.001;
  options.max_step_m = 0.5;
  options.rotation_step_deg = 5.0;

  mcl::InteractiveIkPresentation presentation;
  presentation.base_frame_id = "base_link";
  presentation.joint_state_channel = "/test/joints";
  presentation.arms = {
    {mcl::ArmSide::Left, "/test/left_target", "/test/left_fk", {0}},
    {mcl::ArmSide::Right, "/test/right_target", "/test/right_fk", {1}},
  };

  mcl::TuiTeleopSource tui(
    options, 100.0, "FTXUI PTY test", presentation,
    {
      {mcl::ArmSide::Left, mcl::Pose::Identity()},
      {mcl::ArmSide::Right, mcl::Pose::Identity()},
    },
    true);

  mcl::IkDebugFrame frame;
  frame.targets = tui.command().targets;
  frame.forward_kinematics = {
    {mcl::ArmSide::Left, mcl::Pose::Identity()},
    {mcl::ArmSide::Right, mcl::Pose::Identity()},
  };
  frame.joint_names = {"left_joint", "right_joint"};
  frame.positions = {0.0, 0.0};
  frame.velocities = {0.0, 0.0};
  frame.ik_status = "test";
  frame.converged = true;
  frame.status = "diagnostic frame ready";

  mcl::SolverDebug solver;
  solver.label = "Red";
  solver.disposition = "accepted";
  solver.joint_limit_policy = "explicit-requirements";
  solver.termination_reason = "single-iteration";
  solver.ik_iterations = 1;
  solver.converged = true;
  solver.ik_solve_time_ms = 0.125;
  solver.backend = "proxqp";
  solver.qp_status = "optimal";
  solver.native_status = "PROXQP_SOLVED";
  solver.objective_value = 1.25;
  solver.primal_residual = 1.0e-9;
  solver.dual_residual = 2.0e-9;
  solver.maximum_hard_violation = 3.0e-9;
  solver.qp_solve_time_ms = 0.100;
  solver.qp_iterations = 4;
  solver.warm_start_used = true;
  solver.task_scales = {{"left-cartesian", true, 1.0, 0.0, false, false}};
  solver.requirements = {{"joint-position-limits", "rad", "joint limits", true, true,
                          3.0e-9, 0.0}};
  frame.solvers = {solver};
  frame.workers = {{"Red", 1000.0, 120, 2, 0, 3, 0.010, 0.200, 0.210, 0.0, 0.125}};

  mcl::SelfCollisionDebug collision;
  collision.label = "Yellow self-collision";
  collision.input_state_sequence = 119;
  collision.minimum_distance_m = 0.3;
  collision.influence_distance_m = 0.35;
  collision.minimum_distance_before_m = 0.062;
  collision.minimum_distance_after_m = 0.067;
  collision.margin_shortfall_m = 0.233;
  collision.input_joint_positions = frame.positions;
  collision.pairs = {{"left_arm_link4", "body_link4", 0.062, 0.067, true}};
  frame.self_collisions = {collision};

  if (fault_hold) {
    frame.runtime_state = mcl::IkRuntimeState::RecoverableReject;
    auto rejected_targets = frame.targets;
    rejected_targets[0].target_pose.translation().x() += 0.050;
    frame.rejected_target = mcl::RejectedTargetDebug{
      42, std::move(rejected_targets), "QP is primal infeasible (test)"};
    frame.workers.front().recoverable_rejection_count = 2;
    frame.status = "Red target revision=42 rejected as infeasible";
  }

  tui.render(frame, 1U, "test sink");
  if (throw_after_render) {
    throw std::runtime_error(std::string(kExpectedException));
  }
  if (fault_hold) {
    frame.runtime_state = mcl::IkRuntimeState::FaultHold;
    frame.status = "Red deadline miss revision=42";
    tui.setMotionInputEnabled(false, "FAULT HOLD: Red deadline miss");
    tui.render(frame, 2U, "test sink");
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  std::size_t publish_count = 1;
  while (!tui.command().stop_requested && std::chrono::steady_clock::now() < deadline) {
    tui.poll();
    tui.render(frame, ++publish_count, "test sink");
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const auto & command = tui.command();
  if (!command.stop_requested || command.selected_side != mcl::ArmSide::Right ||
      command.targets.size() != 2U || command.paused == fault_hold) {
    return EXIT_FAILURE;
  }

  const auto & left = command.targets[0].target_pose.translation();
  const auto & right = command.targets[1].target_pose.translation();
  constexpr double kTolerance = 1e-12;
  const double expected_left_x = fault_hold ? 0.0 : 0.005;
  const double expected_right_y = fault_hold ? 0.0 : -0.010;
  const double expected_right_z = fault_hold ? 0.0 : 0.020;
  if (std::abs(left.x() - expected_left_x) > kTolerance ||
      std::abs(right.y() - expected_right_y) > kTolerance ||
      std::abs(right.z() - expected_right_z) > kTolerance) {
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char ** argv)
{
  const bool throw_after_render =
    argc == 2 && std::string_view(argv[1]) == "--throw-after-render";
  const bool fault_hold = argc == 2 && std::string_view(argv[1]) == "--fault-hold";
  try {
    return run(throw_after_render, fault_hold);
  } catch (const std::exception & error) {
    std::cerr << "test_tui_console: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
