#include "app_options.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <initializer_list>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "console/tui_help.hpp"
#include "r1_robot_config.hpp"

namespace motion_control_lab::grouped_servo_step {
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

void validate(const GroupedOptions &options) {
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
}

} // namespace

void printGroupedUsage(const char *program) {
  const GroupedOptions defaults;
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
      << "  --help              Show this help text\n\n"
      << "Rates must satisfy red > yellow > 0. Each group period is its "
         "deadline.\n\n";
  printTuiControlsHelp(std::cout);
}

void printTopLevelUsage(const char *program) {
  std::cout << "Usage: " << program << " <teleop|replay> [options]\n\n"
            << "  teleop  Edit live Cartesian goals (default UI: tui)\n"
            << "  replay  Consume paired MCAP/CSV goals; --target-period-ms is "
               "required\n\n"
            << "Run '" << program << " teleop --help' or '" << program
            << " replay --help' for mode-specific options.\n";
}

GroupedOptions parseGroupedOptions(int argc, char **argv) {
  GroupedOptions options;
  if (const char *configured_urdf =
          std::getenv(kMotionControlUrdfEnvironmentVariable)) {
    options.urdf_path = configured_urdf;
  }
  for (int index = 1; index < argc; ++index) {
    const std::string argument{argv[index]};
    if (argument == "--help" || argument == "-h") {
      printGroupedUsage(argv[0]);
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
        options.tui_enabled = true;
      } else if (value == "none") {
        options.tui_enabled = false;
      } else {
        throw std::runtime_error("ui must be either 'tui' or 'none'");
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
  return options;
}

LaunchOptions parseLaunchOptions(int argc, char **argv) {
  if (argc < 2 || std::string{argv[1]} == "--help" ||
      std::string{argv[1]} == "-h") {
    printTopLevelUsage(argv[0]);
    std::exit(EXIT_SUCCESS);
  }
  const std::string mode{argv[1]};
  if (mode == "teleop") {
    LaunchOptions result;
    result.interactive = parseGroupedOptions(argc - 1, argv + 1);
    return result;
  }
  if (mode != "replay") {
    throw std::runtime_error("expected subcommand 'teleop' or 'replay'");
  }
  if (argc >= 3 &&
      (std::string{argv[2]} == "--help" || std::string{argv[2]} == "-h")) {
    printGroupedUsage(argv[0]);
    std::cout << '\n' << replay::replayHelp(argv[0], true);
    std::exit(EXIT_SUCCESS);
  }

  std::vector<char *> grouped_arguments{argv[0]};
  std::vector<char *> replay_arguments{argv[0]};
  for (int index = 2; index < argc; ++index) {
    const std::string argument{argv[index]};
    const bool shared_value =
        optionIn(argument, {"--urdf", "--ui", "--host", "--port", "--mcap"});
    const bool grouped_value =
        optionIn(argument, {"--red-rate", "--yellow-rate", "--ui-rate",
                            "--deadline-policy", "--duration"});
    const bool replay_value = optionIn(
        argument,
        {"--input", "--input-format", "--left-stream", "--right-stream",
         "--initial-joint-state-stream", "--csv-mapping", "--timestamp-source",
         "--target-period-ms", "--pairing-policy", "--nearest-tolerance-ms",
         "--unmatched-policy", "--execution-mode", "--playback-rate",
         "--output-dir", "--output-root", "--run-id", "--viz-host",
         "--viz-port"});
    if (argument == "--no-mcap") {
      grouped_arguments.push_back(argv[index]);
      replay_arguments.push_back(argv[index]);
      continue;
    }
    if (!shared_value && !grouped_value && !replay_value) {
      throw std::runtime_error("unknown option: " + argument);
    }
    if (index + 1 >= argc) {
      throw std::runtime_error(argument + " requires a value");
    }
    if (shared_value || grouped_value) {
      grouped_arguments.push_back(argv[index]);
      grouped_arguments.push_back(argv[index + 1]);
    }
    if (shared_value || replay_value) {
      replay_arguments.push_back(argv[index]);
      replay_arguments.push_back(argv[index + 1]);
    }
    ++index;
  }

  LaunchOptions result;
  result.source_mode = SourceMode::Replay;
  result.interactive = parseGroupedOptions(
      static_cast<int>(grouped_arguments.size()), grouped_arguments.data());
  result.replay = replay::parseReplayOptions(
      static_cast<int>(replay_arguments.size()), replay_arguments.data(), true);
  return result;
}

} // namespace motion_control_lab::grouped_servo_step
