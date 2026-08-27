#include "../options.hpp"
#include "../planning.hpp"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace app = motion_control_lab::planned_hierarchical_step_otg_nullspace;

int main() {
  char program[] = "mcl_planned_hierarchical_step_otg_nullspace";
  char urdf_option[] = "--urdf";
  char urdf[] = "/tmp/robot.urdf";
  char *defaults_argv[]{program, urdf_option, urdf};
  const auto defaults = app::parseHierarchicalOptions(3, defaults_argv);
  if (defaults.red_rate_hz != 1000.0 || defaults.yellow_rate_hz != 100.0 ||
      defaults.ui_rate_hz != 100.0 ||
      defaults.deadline_policy != motion_control_lab::DeadlinePolicy::Strict ||
      defaults.solver.cartesian_preservation_tolerance != 5.0e-4 ||
      defaults.solver.scale_preservation_tolerance != 1.0e-4 ||
      defaults.solver.posture_preservation_tolerance != 1.0e-5 ||
      defaults.solver.elbow_task_weight != 100.0 ||
      defaults.solver.elbow_servo_gain_per_s != 10.0 ||
      defaults.solver.elbow_preservation_tolerance_mps != 5.0e-4 ||
      defaults.solver.yellow_maximum_iterations != 1 ||
      defaults.solver.red_proxqp_maximum_iterations != 200 ||
      !defaults.robot.inactive_joint_names.empty() ||
      defaults.robot.joint_stream.joint_names.size() != 20U ||
      defaults.robot.joint_stream.max_acceleration_rad_per_s2[10] != 24.3 ||
      defaults.robot.self_collision_link_pairs.size() != 4U ||
      defaults.robot.collision_mesh_search_paths.size() != 1U) {
    return EXIT_FAILURE;
  }

  char red_option[] = "--red-rate";
  char red[] = "200";
  char yellow_option[] = "--yellow-rate";
  char yellow[] = "40";
  char ui_option[] = "--ui";
  char ui[] = "none";
  char collision_option[] = "--collision-weight";
  char collision[] = "12";
  char elbow_option[] = "--elbow-task-weight";
  char elbow[] = "125";
  char *custom_argv[]{program,       urdf_option, urdf,      red_option, red,
                      yellow_option, yellow,      ui_option, ui,
                      collision_option, collision, elbow_option, elbow};
  const auto custom = app::parseHierarchicalOptions(13, custom_argv);
  if (custom.red_rate_hz != 200.0 || custom.yellow_rate_hz != 40.0 ||
      custom.presentation.enabled || custom.solver.collision_weight != 12.0 ||
      custom.solver.elbow_task_weight != 125.0) {
    return EXIT_FAILURE;
  }

  char teleop[] = "teleop";
  char mode_option[] = "--joint-target-mode";
  char mode[] = "ik-pv";
  char *planned_argv[]{program, teleop, urdf_option, urdf, mode_option, mode};
  const auto planned = app::parseOptions(6, planned_argv);
  if (planned.joint_target.mode != app::JointTargetMode::IkPv ||
      !planned.replay_trace_enabled ||
      planned.replay_elbow_teleop_enabled ||
      planned.interactive.red_rate_hz != 1000.0 ||
      planned.interactive.yellow_rate_hz != 100.0 ||
      planned.planning.cartesian_synchronization !=
          app::PlanningSynchronization::Time ||
      planned.planning.joint_synchronization !=
          app::PlanningSynchronization::Phase) {
    return EXIT_FAILURE;
  }

  char replay[] = "replay";
  char input_option[] = "--input";
  char input[] = "/tmp/input.mcap";
  char left_stream_option[] = "--left-stream";
  char left_stream[] = "/left";
  char right_stream_option[] = "--right-stream";
  char right_stream[] = "/right";
  char target_period_option[] = "--target-period-ms";
  char target_period[] = "10";
  char replay_trace_option[] = "--replay-trace";
  char replay_trace_off[] = "off";
  char *replay_argv[]{program,
                      replay,
                      urdf_option,
                      urdf,
                      input_option,
                      input,
                      left_stream_option,
                      left_stream,
                      right_stream_option,
                      right_stream,
                      target_period_option,
                      target_period,
                      replay_trace_option,
                      replay_trace_off};
  const auto replay_options = app::parseOptions(14, replay_argv);
  if (replay_options.source_mode != app::SourceMode::Replay ||
      replay_options.replay_trace_enabled ||
      replay_options.replay_elbow_teleop_enabled) {
    return EXIT_FAILURE;
  }
  char replay_trace_invalid[] = "sometimes";
  replay_argv[13] = replay_trace_invalid;
  bool invalid_replay_trace_rejected = false;
  try {
    (void)app::parseOptions(14, replay_argv);
  } catch (const std::runtime_error &) {
    invalid_replay_trace_rejected = true;
  }
  if (!invalid_replay_trace_rejected) {
    return EXIT_FAILURE;
  }
  replay_argv[13] = replay_trace_off;

  char execution_option[] = "--execution-mode";
  char realtime[] = "realtime";
  char terminal_input_option[] = "--terminal-input";
  char on[] = "on";
  char elbow_teleop_option[] = "--replay-elbow-teleop";
  char *interactive_replay_argv[]{
      program,               replay,
      urdf_option,           urdf,
      input_option,          input,
      left_stream_option,    left_stream,
      right_stream_option,   right_stream,
      target_period_option,  target_period,
      execution_option,      realtime,
      terminal_input_option, on,
      elbow_teleop_option,   on};
  const auto interactive_replay = app::parseOptions(18, interactive_replay_argv);
  if (!interactive_replay.replay_elbow_teleop_enabled ||
      !interactive_replay.replay->terminal_input_enabled ||
      interactive_replay.replay->execution_mode !=
          motion_control_lab::data::ExecutionMode::Realtime) {
    return EXIT_FAILURE;
  }

  auto expectPlannedFailure = [&](int argc, char **argv) {
    try {
      (void)app::parseOptions(argc, argv);
      return false;
    } catch (const std::runtime_error &) {
      return true;
    }
  };
  char off[] = "off";
  char batch[] = "batch";
  interactive_replay_argv[15] = off;
  if (!expectPlannedFailure(18, interactive_replay_argv)) {
    return EXIT_FAILURE;
  }
  interactive_replay_argv[15] = on;
  interactive_replay_argv[13] = batch;
  if (!expectPlannedFailure(18, interactive_replay_argv)) {
    return EXIT_FAILURE;
  }
  interactive_replay_argv[13] = realtime;
  char sometimes[] = "sometimes";
  interactive_replay_argv[17] = sometimes;
  if (!expectPlannedFailure(18, interactive_replay_argv)) {
    return EXIT_FAILURE;
  }
  interactive_replay_argv[17] = on;
  char *teleop_elbow_argv[]{program, teleop, urdf_option, urdf,
                            elbow_teleop_option, on};
  if (!expectPlannedFailure(6, teleop_elbow_argv)) {
    return EXIT_FAILURE;
  }

  auto expectFailure = [&](int argc, char **argv) {
    try {
      (void)app::parseHierarchicalOptions(argc, argv);
      return false;
    } catch (const std::runtime_error &) {
      return true;
    }
  };
  char unknown[] = "--unknown";
  char *unknown_argv[]{program, urdf_option, urdf, unknown};
  char *missing_argv[]{program, urdf_option, urdf, red_option};
  char policy_option[] = "--deadline-policy";
  char bad_policy[] = "ignore";
  char *bad_policy_argv[]{program, urdf_option, urdf, policy_option,
                          bad_policy};
  char equal[] = "20";
  char *bad_rate_argv[]{program, urdf_option,   urdf, red_option,
                        equal,   yellow_option, equal};
  if (!expectFailure(4, unknown_argv) || !expectFailure(4, missing_argv) ||
      !expectFailure(5, bad_policy_argv) || !expectFailure(7, bad_rate_argv)) {
    return EXIT_FAILURE;
  }

  std::ostringstream help;
  auto *original = std::cout.rdbuf(help.rdbuf());
  app::printPlannedUsage(program, app::SourceMode::Replay);
  std::cout.rdbuf(original);
  return help.str().find("default: 1000") != std::string::npos &&
                 help.str().find("default: 100") != std::string::npos &&
                 help.str().find("--start-paused") != std::string::npos &&
                 help.str().find("--replay-elbow-teleop") !=
                     std::string::npos &&
                 help.str().find("--replay-trace") != std::string::npos &&
                 help.str().find("--elbow-task-weight") != std::string::npos &&
                 help.str().find("c: switch TCP/link4") != std::string::npos &&
                 help.str().find("--joint-target-mode") != std::string::npos
             ? EXIT_SUCCESS
             : EXIT_FAILURE;
}
