#include "options.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <initializer_list>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "components/robot/r1/r1_robot_config.hpp"
#include "components/tui/tui_help.hpp"

namespace motion_control_lab::planned_hierarchical_step_otg_nullspace {
namespace {

std::string requireValue(int &index, int argc, char **argv,
                         const std::string &option) {
  if (index + 1 >= argc) {
    throw std::runtime_error(option + " requires a value");
  }
  return argv[++index];
}

double parsePositiveDouble(const std::string &name, const std::string &value) {
  const double parsed = std::stod(value);
  if (parsed <= 0.0 || !std::isfinite(parsed)) {
    throw std::runtime_error(name + " must be a positive finite value");
  }
  return parsed;
}

bool optionIn(const std::string &option,
              std::initializer_list<const char *> candidates) {
  return std::any_of(
      candidates.begin(), candidates.end(),
      [&](const char *candidate) { return option == candidate; });
}

bool parseOnOff(const std::string &argument, const std::string &value) {
  if (value == "on") {
    return true;
  }
  if (value == "off") {
    return false;
  }
  throw std::runtime_error(argument + " must be either 'on' or 'off'");
}

bool parseSolverOption(const std::string &argument, const std::string &value,
                       SolverOptions &options) {
  const double parsed = parsePositiveDouble(argument, value);
  if (argument == "--regularization")
    options.regularization = parsed;
  else if (argument == "--position-tolerance-m")
    options.position_tolerance_m = parsed;
  else if (argument == "--orientation-tolerance-rad")
    options.orientation_tolerance_rad = parsed;
  else if (argument == "--maximum-hard-violation")
    options.maximum_accepted_hard_violation = parsed;
  else if (argument == "--joint-position-margin-rad")
    options.joint_position_margin_rad = parsed;
  else if (argument == "--red-primary-task-cartesian-progress-weight")
    options.red_primary_task_cartesian_progress_weight = parsed;
  else if (argument == "--red-secondary-task-link4-position-weight")
    options.red_secondary_task_link4_position_weight = parsed;
  else if (argument ==
           "--red-secondary-task-link4-position-servo-gain-per-s")
    options.red_secondary_task_link4_position_servo_gain_per_s = parsed;
  else if (argument ==
           "--red-secondary-task-link4-position-preservation-tolerance-mps")
    options.red_secondary_task_link4_position_preservation_tolerance_mps =
        parsed;
  else if (argument == "--red-proxqp-absolute-tolerance")
    options.red_proxqp_absolute_tolerance = parsed;
  else if (argument == "--red-proxqp-primal-infeasibility-tolerance")
    options.red_proxqp_primal_infeasibility_tolerance = parsed;
  else if (argument == "--yellow-task-posture-preference-weight")
    options.yellow_task_posture_preference_weight = parsed;
  else if (argument ==
           "--red-secondary-task-yellow-posture-coupling-weight")
    options.red_secondary_task_yellow_posture_coupling_weight = parsed;
  else if (argument ==
           "--yellow-constraints-self-collision-avoidance-minimum-distance-m")
    options.yellow_constraints_self_collision_avoidance_minimum_distance_m =
        parsed;
  else if (argument ==
           "--yellow-constraints-self-collision-avoidance-influence-distance-m")
    options.yellow_constraints_self_collision_avoidance_influence_distance_m =
        parsed;
  else if (argument ==
           "--yellow-constraints-self-collision-avoidance-damping-gain-per-s")
    options.yellow_constraints_self_collision_avoidance_damping_gain_per_s =
        parsed;
  else if (argument ==
           "--yellow-constraints-self-collision-avoidance-weight")
    options.yellow_constraints_self_collision_avoidance_weight = parsed;
  else
    return false;
  return true;
}

void validate(const HierarchicalOptions &options) {
  if (!(options.red_rate_hz > options.yellow_rate_hz)) {
    throw std::runtime_error("rates must satisfy red > yellow > 0");
  }
  if (options.tui.side != "left" && options.tui.side != "right") {
    throw std::runtime_error("side must be either 'left' or 'right'");
  }
  if (options.tui.max_step_m < options.tui.min_step_m) {
    throw std::runtime_error(
        "--max-step-m must be greater than or equal to --min-step-m");
  }
  if (options.tui.step_m < options.tui.min_step_m ||
      options.tui.step_m > options.tui.max_step_m) {
    throw std::runtime_error(
        "--step-m must be inside [--min-step-m, --max-step-m]");
  }
  if (options.urdf_path.empty()) {
    throw std::runtime_error(std::string{"--urdf is required unless "} +
                             kMotionControlUrdfEnvironmentVariable + " is set");
  }
  if (options.solver
          .yellow_constraints_self_collision_avoidance_influence_distance_m <=
      options.solver
          .yellow_constraints_self_collision_avoidance_minimum_distance_m) {
    throw std::runtime_error(
        "--yellow-constraints-self-collision-avoidance-influence-distance-m "
        "must exceed "
        "--yellow-constraints-self-collision-avoidance-minimum-distance-m");
  }
}

std::string collisionMeshSearchRoot(const std::filesystem::path &urdf_path) {
  const auto canonical_urdf = std::filesystem::weakly_canonical(urdf_path);
  return canonical_urdf.parent_path().parent_path().parent_path().string();
}

} // namespace

void printHierarchicalUsage(const char *program) {
  const HierarchicalOptions defaults;
  std::cout
      << "Usage: " << program << " [options]\n\n"
      << "Options:\n"
      << "  --side <left|right> Initial selected arm side (default: "
      << defaults.tui.side << ")\n"
      << "  --urdf <path>       Robot URDF path (or $"
      << kMotionControlUrdfEnvironmentVariable << ")\n"
      << "  --host <address>    WebSocket bind address (default: "
      << defaults.visualization.host << ")\n"
      << "  --port <port>       WebSocket port (default: "
      << defaults.visualization.port << ")\n"
      << "  --red-rate <hz>     Red servo rate/deadline (default: "
      << defaults.red_rate_hz << ")\n"
      << "  --yellow-rate <hz>  Yellow proposal rate/deadline (default: "
      << defaults.yellow_rate_hz << ")\n"
      << "  --ui-rate <hz>      TUI and visualization rate (default: "
      << defaults.ui_rate_hz << ")\n"
      << "  --ui <tui|none>     User interface mode (default: tui)\n"
      << "  --viz <foxglove|none> Visualization transport (default: foxglove)\n"
      << "  --deadline-policy <strict|monitor> Deadline handling (default: "
         "strict)\n"
      << "  --duration <sec>    Stop after seconds; 0 runs until Ctrl-C "
         "(default: "
      << defaults.duration_s << ")\n"
      << "  --step-m <meters>   Cartesian increment per keypress (default: "
      << defaults.tui.step_m << ")\n"
      << "  --min-step-m <m>    Minimum step size (default: "
      << defaults.tui.min_step_m << ")\n"
      << "  --max-step-m <m>    Maximum step size (default: "
      << defaults.tui.max_step_m << ")\n"
      << "  --rotation-step-deg <deg> TCP rotation step (default: "
      << defaults.tui.rotation_step_deg << ")\n"
      << "  --mcap <path>       Write MCAP from the UI thread when Foxglove is "
         "enabled\n"
      << "  --no-mcap           Disable MCAP output (default)\n"
      << "  --regularization <value>                 QP regularization\n"
      << "  --position-tolerance-m <value>           Position tolerance\n"
      << "  --orientation-tolerance-rad <value>      Orientation tolerance\n"
      << "  --maximum-hard-violation <value>         App acceptance tolerance\n"
      << "  --joint-position-margin-rad <value>      Joint limit margin\n"
      << "  --red-primary-task-cartesian-progress-weight <value> Cartesian "
         "progress weight\n"
      << "  --red-secondary-task-link4-position-weight <value> Secondary "
         "link4 weight "
         "(default: "
      << defaults.solver.red_secondary_task_link4_position_weight << ")\n"
      << "  --red-secondary-task-link4-position-servo-gain-per-s <value> "
         "Secondary link4 gain "
         "(default: "
      << defaults.solver.red_secondary_task_link4_position_servo_gain_per_s
      << ")\n"
      << "  --red-secondary-task-link4-position-preservation-tolerance-mps "
         "<value> Link4 preservation tolerance (default: "
      << defaults.solver
             .red_secondary_task_link4_position_preservation_tolerance_mps
      << ")\n"
      << "  --red-proxqp-absolute-tolerance <value>  Red QP tolerance\n"
      << "  --red-proxqp-primal-infeasibility-tolerance <value> Red "
         "certificate tolerance\n"
      << "  --yellow-task-posture-preference-weight <value> Yellow posture "
         "preference weight\n"
      << "  --red-secondary-task-yellow-posture-coupling-weight <value> "
         "Yellow-to-Red coupling weight\n"
      << "  --yellow-constraints-self-collision-avoidance-minimum-distance-m "
         "<value> Collision minimum distance\n"
      << "  --yellow-constraints-self-collision-avoidance-influence-distance-m "
         "<value> Collision influence distance\n"
      << "  --yellow-constraints-self-collision-avoidance-damping-gain-per-s "
         "<value> Collision damping gain\n"
      << "  --yellow-constraints-self-collision-avoidance-weight <value> "
         "Collision weight\n"
      << "  --help              Show this help text\n\n"
      << "Rates must satisfy red > yellow > 0. Each group period is its "
         "deadline.\n\n";
  std::cout
      << "Keyboard controls:\n"
      << "  1..8/F1..F7/Tab/BackTab: navigate pages; h or ?: help\n"
      << "  PageUp/PageDown/Home/End: scroll the current page\n"
      << "  left/right arrows: select arm; c: switch TCP/link4 focus\n"
      << "  w/s: +x/-x, a/d: +y/-y, q/e: +z/-z\n"
      << "  n: cycle TCP rotation axis, i/u: rotate TCP\n"
      << "  up/down arrows: double/halve step; m: enter step size\n"
      << "  r: reset current target; x: clear held link4 or exit; Esc: exit\n";
}

void printPlannedUsage(const char *program, SourceMode source_mode) {
  printHierarchicalUsage(program);
  const Options defaults;
  std::cout << "\nOnline Cartesian replan limits (per "
               "reference-frame/rotation-vector axis):\n"
            << "  --max-linear-velocity-mps <value>       (default: "
            << defaults.planning.max_linear_velocity_mps << ")\n"
            << "  --max-linear-acceleration-mps2 <value>  (default: "
            << defaults.planning.max_linear_acceleration_mps2 << ")\n"
            << "  --max-linear-jerk-mps3 <value>          (default: "
            << defaults.planning.max_linear_jerk_mps3 << ")\n"
            << "  --max-angular-velocity-rps <value>      (default: "
            << defaults.planning.max_angular_velocity_rps << ")\n"
            << "  --max-angular-acceleration-rps2 <value> (default: "
            << defaults.planning.max_angular_acceleration_rps2 << ")\n"
            << "  --max-angular-jerk-rps3 <value>         (default: "
            << defaults.planning.max_angular_jerk_rps3 << ")\n"
            << "  --joint-target-mode <future-o1-pv|ik-pv> (default: "
            << jointTargetModeName(defaults.joint_target.mode) << ")\n";
  if (source_mode == SourceMode::Replay) {
    std::cout << "\nReplay startup:\n"
              << "  --start-paused  Hold the replay at timeline zero until "
                 "space is pressed\n"
              << "  --replay-elbow-teleop <on|off> Allow realtime link4 "
                 "Secondary target editing (default: off)\n"
              << "  --replay-trace <on|off> Write detailed per-Red-tick "
                 "trace.csv (default: on)\n";
  }
}

HierarchicalOptions parseHierarchicalOptions(int argc, char **argv) {
  HierarchicalOptions options;
  if (const char *configured_urdf =
          std::getenv(kMotionControlUrdfEnvironmentVariable)) {
    options.urdf_path = configured_urdf;
  }
  for (int index = 1; index < argc; ++index) {
    const std::string argument{argv[index]};
    if (argument == "--help" || argument == "-h") {
      printHierarchicalUsage(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else if (argument == "--side") {
      options.tui.side = requireValue(index, argc, argv, argument);
    } else if (argument == "--urdf") {
      options.urdf_path = requireValue(index, argc, argv, argument);
    } else if (argument == "--host") {
      options.visualization.host = requireValue(index, argc, argv, argument);
    } else if (argument == "--port") {
      const long port = std::stol(requireValue(index, argc, argv, argument));
      if (port <= 0 || port > 65535) {
        throw std::runtime_error("port must be in [1, 65535]");
      }
      options.visualization.port = static_cast<std::uint16_t>(port);
    } else if (argument == "--red-rate") {
      options.red_rate_hz = parsePositiveDouble(
          "red rate", requireValue(index, argc, argv, argument));
    } else if (argument == "--yellow-rate") {
      options.yellow_rate_hz = parsePositiveDouble(
          "yellow rate", requireValue(index, argc, argv, argument));
    } else if (argument == "--ui-rate") {
      options.ui_rate_hz = parsePositiveDouble(
          "UI rate", requireValue(index, argc, argv, argument));
    } else if (argument == "--ui") {
      const auto value = requireValue(index, argc, argv, argument);
      if (value == "tui") {
        options.presentation.enabled = true;
      } else if (value == "none") {
        options.presentation.enabled = false;
      } else {
        throw std::runtime_error("ui must be either 'tui' or 'none'");
      }
    } else if (argument == "--viz") {
      const auto value = requireValue(index, argc, argv, argument);
      if (value == "foxglove") {
        options.visualization.enabled = true;
      } else if (value == "none") {
        options.visualization.enabled = false;
      } else {
        throw std::runtime_error("viz must be either 'foxglove' or 'none'");
      }
    } else if (argument == "--deadline-policy") {
      const auto value = requireValue(index, argc, argv, argument);
      if (value == "strict") {
        options.deadline_policy = DeadlinePolicy::Strict;
      } else if (value == "monitor") {
        options.deadline_policy = DeadlinePolicy::Monitor;
      } else {
        throw std::runtime_error(
            "--deadline-policy must be either 'strict' or 'monitor'");
      }
    } else if (argument == "--duration") {
      options.duration_s = std::stod(requireValue(index, argc, argv, argument));
      if (options.duration_s < 0.0 || !std::isfinite(options.duration_s)) {
        throw std::runtime_error("duration must be finite and non-negative");
      }
    } else if (argument == "--step-m") {
      options.tui.step_m = parsePositiveDouble(
          "step", requireValue(index, argc, argv, argument));
    } else if (argument == "--min-step-m") {
      options.tui.min_step_m = parsePositiveDouble(
          "minimum step", requireValue(index, argc, argv, argument));
    } else if (argument == "--max-step-m") {
      options.tui.max_step_m = parsePositiveDouble(
          "maximum step", requireValue(index, argc, argv, argument));
    } else if (argument == "--rotation-step-deg") {
      options.tui.rotation_step_deg = parsePositiveDouble(
          "rotation step", requireValue(index, argc, argv, argument));
    } else if (optionIn(
                   argument,
                   {"--regularization", "--position-tolerance-m",
                    "--orientation-tolerance-rad", "--maximum-hard-violation",
                    "--joint-position-margin-rad",
                    "--red-primary-task-cartesian-progress-weight",
                    "--red-secondary-task-link4-position-weight",
                    "--red-secondary-task-link4-position-servo-gain-per-s",
                    "--red-secondary-task-link4-position-preservation-tolerance-mps",
                    "--red-proxqp-absolute-tolerance",
                    "--red-proxqp-primal-infeasibility-tolerance",
                    "--yellow-task-posture-preference-weight",
                    "--red-secondary-task-yellow-posture-coupling-weight",
                    "--yellow-constraints-self-collision-avoidance-minimum-distance-m",
                    "--yellow-constraints-self-collision-avoidance-influence-distance-m",
                    "--yellow-constraints-self-collision-avoidance-damping-gain-per-s",
                    "--yellow-constraints-self-collision-avoidance-weight"})) {
      parseSolverOption(argument, requireValue(index, argc, argv, argument),
                        options.solver);
    } else if (argument == "--mcap") {
      options.visualization.mcap_path =
          std::filesystem::path{requireValue(index, argc, argv, argument)};
    } else if (argument == "--no-mcap") {
      options.visualization.mcap_path.reset();
    } else {
      throw std::runtime_error("unknown option: " + argument);
    }
  }
  validate(options);
  options.robot.collision_mesh_search_paths = {
      collisionMeshSearchRoot(options.urdf_path)};
  return options;
}

Options parseOptions(int argc, char **argv) {
  if (argc < 2 || std::string{argv[1]} == "--help" ||
      std::string{argv[1]} == "-h") {
    std::cout << "Usage: " << argv[0] << " <teleop|replay> [options]\n\n"
              << "  teleop  Edit source Cartesian goals and replan online "
                 "(default UI: tui)\n"
              << "  replay  Replan paired MCAP/CSV goals; --target-period-ms "
                 "is required\n";
    std::exit(EXIT_SUCCESS);
  }
  Options result;
  const std::string mode{argv[1]};
  if (mode != "teleop" && mode != "replay") {
    throw std::runtime_error("expected subcommand 'teleop' or 'replay'");
  }
  result.source_mode =
      mode == "replay" ? SourceMode::Replay : SourceMode::Teleop;
  if (argc >= 3 &&
      (std::string{argv[2]} == "--help" || std::string{argv[2]} == "-h")) {
    printPlannedUsage(argv[0], result.source_mode);
    if (result.source_mode == SourceMode::Replay) {
      std::cout << '\n' << replay::replayHelp(argv[0], true);
    }
    std::exit(EXIT_SUCCESS);
  }

  std::vector<char *> hierarchical_arguments{argv[0]};
  std::vector<char *> replay_arguments{argv[0]};
  for (int index = 2; index < argc; ++index) {
    const std::string argument{argv[index]};
    auto planningValue = [&](double &destination) {
      destination = parsePositiveDouble(
          argument, requireValue(index, argc, argv, argument));
    };
    if (argument == "--max-linear-velocity-mps") {
      planningValue(result.planning.max_linear_velocity_mps);
    } else if (argument == "--max-linear-acceleration-mps2") {
      planningValue(result.planning.max_linear_acceleration_mps2);
    } else if (argument == "--max-linear-jerk-mps3") {
      planningValue(result.planning.max_linear_jerk_mps3);
    } else if (argument == "--max-angular-velocity-rps") {
      planningValue(result.planning.max_angular_velocity_rps);
    } else if (argument == "--max-angular-acceleration-rps2") {
      planningValue(result.planning.max_angular_acceleration_rps2);
    } else if (argument == "--max-angular-jerk-rps3") {
      planningValue(result.planning.max_angular_jerk_rps3);
    } else if (argument == "--start-paused") {
      if (result.source_mode != SourceMode::Replay) {
        throw std::runtime_error("--start-paused is only valid with replay");
      }
      result.start_paused = true;
    } else if (argument == "--replay-trace") {
      if (result.source_mode != SourceMode::Replay) {
        throw std::runtime_error("--replay-trace is only valid with replay");
      }
      result.replay_trace_enabled =
          parseOnOff(argument, requireValue(index, argc, argv, argument));
    } else if (argument == "--replay-elbow-teleop") {
      if (result.source_mode != SourceMode::Replay) {
        throw std::runtime_error(
            "--replay-elbow-teleop is only valid with replay");
      }
      result.replay_elbow_teleop_enabled =
          parseOnOff(argument, requireValue(index, argc, argv, argument));
    } else if (argument == "--joint-target-mode") {
      const auto value = requireValue(index, argc, argv, argument);
      if (value == "future-o1-pv") {
        result.joint_target.mode = JointTargetMode::FutureO1Pv;
      } else if (value == "ik-pv") {
        result.joint_target.mode = JointTargetMode::IkPv;
      } else {
        throw std::runtime_error(
            "--joint-target-mode must be either 'future-o1-pv' or 'ik-pv'");
      }
    } else if (result.source_mode == SourceMode::Teleop) {
      hierarchical_arguments.push_back(argv[index]);
    } else {
      const bool shared_value = optionIn(
          argument, {"--urdf", "--ui", "--viz", "--host", "--port", "--mcap"});
      const bool hierarchical_value = optionIn(
          argument,
          {"--red-rate", "--yellow-rate", "--ui-rate", "--deadline-policy",
           "--duration", "--regularization", "--position-tolerance-m",
           "--orientation-tolerance-rad", "--maximum-hard-violation",
           "--joint-position-margin-rad",
           "--red-primary-task-cartesian-progress-weight",
           "--red-secondary-task-link4-position-weight",
           "--red-secondary-task-link4-position-servo-gain-per-s",
           "--red-secondary-task-link4-position-preservation-tolerance-mps",
           "--red-proxqp-absolute-tolerance",
           "--red-proxqp-primal-infeasibility-tolerance",
           "--yellow-task-posture-preference-weight",
           "--red-secondary-task-yellow-posture-coupling-weight",
           "--yellow-constraints-self-collision-avoidance-minimum-distance-m",
           "--yellow-constraints-self-collision-avoidance-influence-distance-m",
           "--yellow-constraints-self-collision-avoidance-damping-gain-per-s",
           "--yellow-constraints-self-collision-avoidance-weight"});
      const bool replay_value =
          optionIn(argument, {"--input",
                              "--input-format",
                              "--left-stream",
                              "--right-stream",
                              "--initial-joint-state-stream",
                              "--csv-mapping",
                              "--timestamp-source",
                              "--target-period-ms",
                              "--pairing-policy",
                              "--nearest-tolerance-ms",
                              "--unmatched-policy",
                              "--execution-mode",
                              "--playback-rate",
                              "--output-dir",
                              "--output-root",
                              "--run-id",
                              "--viz-host",
                              "--viz-port",
                              "--terminal-input",
                              "--launcher"});
      if (argument == "--no-mcap") {
        hierarchical_arguments.push_back(argv[index]);
        replay_arguments.push_back(argv[index]);
        continue;
      }
      if (!shared_value && !hierarchical_value && !replay_value) {
        throw std::runtime_error("unknown option: " + argument);
      }
      if (index + 1 >= argc) {
        throw std::runtime_error(argument + " requires a value");
      }
      if (shared_value || hierarchical_value) {
        hierarchical_arguments.push_back(argv[index]);
        hierarchical_arguments.push_back(argv[index + 1]);
      }
      if (shared_value || replay_value) {
        replay_arguments.push_back(argv[index]);
        replay_arguments.push_back(argv[index + 1]);
      }
      ++index;
    }
  }
  result.interactive = parseHierarchicalOptions(
      static_cast<int>(hierarchical_arguments.size()), hierarchical_arguments.data());
  if (result.source_mode == SourceMode::Replay) {
    result.replay =
        replay::parseReplayOptions(static_cast<int>(replay_arguments.size()),
                                   replay_arguments.data(), true);
    result.replay->original_argv.assign(argv, argv + argc);
    result.interactive.visualization.enabled =
        result.replay->visualization_enabled;
    if (result.start_paused && !result.replay->terminal_input_enabled) {
      throw std::runtime_error("--start-paused requires --terminal-input on");
    }
    if (result.replay_elbow_teleop_enabled &&
        !result.replay->terminal_input_enabled) {
      throw std::runtime_error(
          "--replay-elbow-teleop on requires --terminal-input on");
    }
    if (result.replay_elbow_teleop_enabled &&
        result.replay->execution_mode != data::ExecutionMode::Realtime) {
      throw std::runtime_error(
          "--replay-elbow-teleop on requires --execution-mode realtime");
    }
  }
  return result;
}

} // namespace motion_control_lab::planned_hierarchical_step_otg_nullspace
