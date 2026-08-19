#include "console/tui_console.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <thread>

#include "runtime/interactive_types.hpp"

namespace
{

constexpr std::string_view kExpectedException = "intentional TUI exception";

int run(bool throw_after_render, bool fault_hold, bool replay_controls, bool replay_start_paused)
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
    {mcl::ArmSide::Left, "/test/left_target", "/test/left_fk", {6, 7, 8, 9, 10, 11, 12}},
    {mcl::ArmSide::Right, "/test/right_target", "/test/right_fk", {13, 14, 15, 16, 17, 18, 19}},
  };

  mcl::TuiConsole tui(
    options, 100.0, "FTXUI PTY test", presentation,
    {
      {mcl::ArmSide::Left, mcl::Pose::Identity()},
      {mcl::ArmSide::Right, mcl::Pose::Identity()},
    },
    true, true, replay_controls ? mcl::TuiControlMode::Replay : mcl::TuiControlMode::Teleop);
  if (replay_controls) {
    tui.setMotionInputEnabled(false, "Replay motion editing is disabled");
    if (replay_start_paused) {
      tui.setPaused(true, "Replay timeline paused; press space to start");
    }
  }

  mcl::IkDebugFrame frame;
  frame.targets = tui.command().targets;
  frame.forward_kinematics = {
    {mcl::ArmSide::Left, mcl::Pose::Identity()},
    {mcl::ArmSide::Right, mcl::Pose::Identity()},
  };
  frame.joint_names = {
    "head_yaw_joint",   "head_pitch_joint",  "torso_yaw_joint",  "torso_pitch_joint",
    "knee_pitch_joint", "ankle_pitch_joint", "left_arm_joint1",  "left_arm_joint2",
    "left_arm_joint3",  "left_arm_joint4",   "left_arm_joint5",  "left_arm_joint6",
    "left_arm_joint7",  "right_arm_joint1",  "right_arm_joint2", "right_arm_joint3",
    "right_arm_joint4", "right_arm_joint5",  "right_arm_joint6", "right_arm_joint7"};
  frame.positions = {0.0, 0.1, 0.2, 0.3,  0.4,  0.5,  0.6,  0.7,  0.8,  0.9,
                     1.0, 1.1, 1.2, -0.6, -0.7, -0.8, -0.9, -1.0, -1.1, -1.2};
  frame.velocities = {0.00, 0.01, 0.02, 0.03,  0.04,  0.05,  0.06,  0.07,  0.08,  0.09,
                      0.10, 0.11, 0.12, -0.06, -0.07, -0.08, -0.09, -0.10, -0.11, -0.12};
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
  solver.ik_solve_time_percentiles = {5000, 4096, 4096, 0.130, 0.140, 0.160};
  solver.run_counters = mcl::SolverRunCounters{5001, 5000, 1};
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
  solver.saturated_joints = {"left_arm_joint4"};
  solver.task_scales = {{"left-cartesian", true, 1.0, 0.0, false, false}};
  solver.requirements = {{"joint-position-limits", "rad", "joint limits", true, true, 3.0e-9, 0.0}};
  frame.solvers = {solver};
  frame.workers = {{"Red", 1000.0, 120, 2, 0, 3, 0.010, 0.200, 0.210, 0.0, 0.125}};
  frame.workers.front().maximum_non_solver_execution_ms = 0.075;
  frame.workers.front().latest_release_lateness_ms = 0.004;
  frame.workers.front().latest_execution_ms = 0.180;
  frame.workers.front().latest_release_to_finish_ms = 0.184;
  frame.workers.front().latest_overrun_ms = 0.0;
  frame.workers.front().latest_solver_ms = 0.120;
  frame.workers.front().latest_non_solver_execution_ms = 0.060;
  frame.cpu_affinities = {
    {"ui", true, 4101, {16}, {16}}, {"red", true, 4102, {8}, {8}}, {"yellow", false, -1, {10}, {}}};

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
    frame.rejected_target =
      mcl::RejectedTargetDebug{42, std::move(rejected_targets), "QP is primal infeasible (test)"};
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
  const bool single_step_requested = tui.consumeSingleStepRequest();
  const bool expected_paused = !fault_hold;
  if (
    !command.stop_requested || command.selected_side != mcl::ArmSide::Right ||
    command.targets.size() != 2U || command.paused != expected_paused ||
    single_step_requested != replay_controls) {
    return EXIT_FAILURE;
  }

  const auto & left = command.targets[0].target_pose.translation();
  const auto & right = command.targets[1].target_pose.translation();
  constexpr double kTolerance = 1e-12;
  const bool motion_disabled = fault_hold || replay_controls;
  const double expected_left_x = motion_disabled ? 0.0 : 0.005;
  const double expected_right_y = motion_disabled ? 0.0 : -0.010;
  const double expected_right_z = motion_disabled ? 0.0 : 0.020;
  if (
    std::abs(left.x() - expected_left_x) > kTolerance ||
    std::abs(right.y() - expected_right_y) > kTolerance ||
    std::abs(right.z() - expected_right_z) > kTolerance) {
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char ** argv)
{
  const bool throw_after_render = argc == 2 && std::string_view(argv[1]) == "--throw-after-render";
  const bool fault_hold = argc == 2 && std::string_view(argv[1]) == "--fault-hold";
  const bool replay_controls =
    argc == 2 &&
    (std::string_view(argv[1]) == "--replay" ||
     std::string_view(argv[1]) == "--replay-start-paused");
  const bool replay_start_paused =
    argc == 2 && std::string_view(argv[1]) == "--replay-start-paused";
  try {
    return run(throw_after_render, fault_hold, replay_controls, replay_start_paused);
  } catch (const std::exception & error) {
    std::cerr << "test_tui_console: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
