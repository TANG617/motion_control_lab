#include "app_options.hpp"

#include "baseline_config.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "console/tui_help.hpp"
#include "r1_robot_config.hpp"

namespace motion_control_lab::baseline {
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

void validate(const TeleopOptions &options) {
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

void printTopLevelUsage(const char *program) {
  std::cout
      << "Usage: " << program << " <teleop|replay> [options]\n\n"
      << "  teleop  Run the fixed 100 Hz production-static PlaCo baseline\n"
      << "  replay  Replay paired TCP targets at the fixed 10 ms period\n\n"
      << "This executable is PlaCo-only; --solver and --backend are not "
         "supported.\n";
}

void printTeleopUsage(const char *program) {
  TeleopOptions defaults;
  defaults.rate_hz = productionStaticConfig().control_rate_hz;
  std::cout
      << "Usage: " << program << " [options]\n\n"
      << "Options:\n"
      << "  --urdf <path>              Robot URDF (required unless $"
      << kMotionControlUrdfEnvironmentVariable << " is set)\n"
      << "  --side <left|right>        Initially selected arm (default: "
      << defaults.tui.side << ")\n"
      << "  --ui <tui|none>            User interface mode (default: tui)\n"
      << "  --duration <seconds>       Stop after duration; zero runs until "
         "stopped "
         "(default: "
      << defaults.duration_s << ")\n"
      << "  --step-m <meters>          Cartesian key increment (default: "
      << defaults.tui.step_m << ")\n"
      << "  --min-step-m <meters>      Minimum Cartesian increment (default: "
      << defaults.tui.min_step_m << ")\n"
      << "  --max-step-m <meters>      Maximum Cartesian increment (default: "
      << defaults.tui.max_step_m << ")\n"
      << "  --rotation-step-deg <deg>  TCP local-axis rotation increment "
         "(default: "
      << defaults.tui.rotation_step_deg << ")\n"
      << "  --host <address>           Foxglove bind address (default: "
      << defaults.visualization.host << ")\n"
      << "  --port <port>              Foxglove port (default: "
      << defaults.visualization.port << ")\n"
      << "  --mcap <path>              Record visualization MCAP\n"
      << "  --no-mcap                  Disable visualization MCAP (default)\n"
      << "  --help                     Show this text\n\n"
      << "The control rate is fixed at " << defaults.rate_hz
      << " Hz. --rate is rejected.\n\n";
  printTuiControlsHelp(std::cout);
}

void printReplayUsage(const char *program) {
  std::cout
      << "Usage: " << program << " [options]\n\n"
      << "  --urdf <path>                    Robot URDF (required)\n"
      << "  --input <path>                   MCAP or CSV input (required)\n"
      << "  --input-format mcap|csv          Physical source backend\n"
      << "  --left-stream <name>             Left logical TCP stream\n"
      << "  --right-stream <name>            Right logical TCP stream\n"
      << "  --csv-mapping <json>             Optional CSV column mapping\n"
      << "  --timestamp-source <source>      header_stamp, log_time, "
         "publish_time, "
         "or csv_timestamp\n"
      << "  --target-period-ms 10            Required production parity "
         "period\n"
      << "  --pairing-policy exact|nearest   Original-time pairing\n"
      << "  --nearest-tolerance-ms <ms>      Nearest pairing tolerance\n"
      << "  --unmatched-policy error|drop_with_diagnostics\n"
      << "  --execution-mode batch|realtime\n"
      << "  --playback-rate <rate>           Positive replay rate\n"
      << "  --output-dir <path>              Exact new artifact directory\n"
      << "  --output-root <path>             Parent of an auto-named run\n"
      << "  --run-id <id>                    Override auto-generated run ID\n"
      << "  --ui tui|none                    User interface mode\n"
      << "  --host <address>                 Foxglove bind address\n"
      << "  --port <port>                    Foxglove port\n"
      << "  --mcap <path>                    Record visualization MCAP\n"
      << "  --no-mcap                        Disable visualization MCAP\n"
      << "  --help                           Show this text\n\n"
      << "Replay always starts from the frozen production initial pose; "
         "--initial-joint-state-stream is rejected.\n";
}

TeleopOptions parseTeleopOptions(int argc, char **argv) {
  TeleopOptions options;
  options.rate_hz = productionStaticConfig().control_rate_hz;
  if (const char *configured_urdf =
          std::getenv(kMotionControlUrdfEnvironmentVariable)) {
    options.urdf_path = configured_urdf;
  }
  for (int index = 1; index < argc; ++index) {
    const std::string argument{argv[index]};
    if (argument == "--help" || argument == "-h") {
      printTeleopUsage(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else if (argument == "--rate") {
      throw std::runtime_error(
          "--rate is not supported; the baseline is fixed at 100 Hz");
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
    } else if (argument == "--ui") {
      const auto value = requireValue(index, argc, argv, argument);
      if (value == "tui") {
        options.tui_enabled = true;
      } else if (value == "none") {
        options.tui_enabled = false;
      } else {
        throw std::runtime_error("ui must be either 'tui' or 'none'");
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

ReplayAppOptions parseReplayOptions(int argc, char **argv) {
  for (int index = 1; index < argc; ++index) {
    const std::string argument{argv[index]};
    if (argument == "--help" || argument == "-h") {
      printReplayUsage(argv[0]);
      std::exit(EXIT_SUCCESS);
    }
    if (argument == "--initial-joint-state-stream") {
      throw std::runtime_error("--initial-joint-state-stream is not supported; "
                               "the baseline uses the production initial pose");
    }
  }

  ReplayAppOptions result;
  result.replay = replay::parseReplayOptions(argc, argv, true);
  if (result.replay.target_period_ns != kTargetPeriodNs) {
    throw std::runtime_error(
        "--target-period-ms must be exactly 10 for the production baseline");
  }
  result.replay.initial_joint_state_stream.reset();
  return result;
}

} // namespace motion_control_lab::baseline
