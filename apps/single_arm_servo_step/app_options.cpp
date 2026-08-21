#include "app_options.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "components/robot/r1/r1_robot_config.hpp"
#include "components/tui/tui_help.hpp"

namespace motion_control_lab::single_arm_servo_step {
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

double parseNonnegativeDouble(const std::string &name, const std::string &value) {
  const double parsed = std::stod(value);
  if (parsed < 0.0 || !std::isfinite(parsed)) {
    throw std::runtime_error(name + " must be a finite non-negative value");
  }
  return parsed;
}

void validate(const AppOptions &options) {
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

void printUsage(const char *program) {
  const AppOptions defaults;
  std::cout
      << "Usage: " << program << " [options]\n\n"
      << "Options:\n"
      << "  --side <left|right> Initial arm side (default: "
      << defaults.tui.side << ")\n"
      << "  --urdf <path>       Robot URDF path (or $"
      << kMotionControlUrdfEnvironmentVariable << ")\n"
      << "  --host <address>    WebSocket bind address (default: "
      << defaults.visualization.host << ")\n"
      << "  --port <port>       WebSocket port (default: "
      << defaults.visualization.port << ")\n"
      << "  --rate <hz>         IK and publish rate in Hz (default: "
      << defaults.rate_hz << ")\n"
      << "  --ui <tui|none>     User interface mode (default: tui)\n"
      << "  --viz <foxglove|none> Visualization transport (default: foxglove)\n"
      << "  --duration <sec>    Stop after seconds; 0 runs until Ctrl-C "
         "(default: "
      << defaults.duration_s << ")\n"
      << "  --step-m <meters>   Cartesian increment per keypress (default: "
      << defaults.tui.step_m << ")\n"
      << "  --min-step-m <m>    Minimum step size (default: "
      << defaults.tui.min_step_m << ")\n"
      << "  --max-step-m <m>    Maximum step size (default: "
      << defaults.tui.max_step_m << ")\n"
      << "  --rotation-step-deg <deg> TCP local-axis rotation step (default: "
      << defaults.tui.rotation_step_deg << ")\n"
      << "  --regularization <value> QP regularization (default: "
      << defaults.regularization << ")\n"
      << "  --maximum-iterations <count> Servo solver iterations (default: "
      << defaults.maximum_iterations << ")\n"
      << "  --position-tolerance-m <value> Position tolerance (default: "
      << defaults.position_tolerance_m << ")\n"
      << "  --orientation-tolerance-rad <value> Orientation tolerance (default: "
      << defaults.orientation_tolerance_rad << ")\n"
      << "  --joint-position-margin-rad <value> Joint limit margin (default: "
      << defaults.joint_position_margin_rad << ")\n"
      << "  --mcap <path>       Write MCAP output to path when Foxglove is "
         "enabled\n"
      << "  --no-mcap           Disable MCAP output (default)\n"
      << "  --help              Show this help text\n\n";
  printTuiControlsHelp(std::cout);
}

AppOptions parseAppOptions(int argc, char **argv) {
  AppOptions options;
  if (const char *configured_urdf =
          std::getenv(kMotionControlUrdfEnvironmentVariable)) {
    options.urdf_path = configured_urdf;
  }
  for (int index = 1; index < argc; ++index) {
    const std::string argument{argv[index]};
    if (argument == "--help" || argument == "-h") {
      printUsage(argv[0]);
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
    } else if (argument == "--rate") {
      options.rate_hz = parsePositiveDouble(
          "rate", requireValue(index, argc, argv, argument));
    } else if (argument == "--ui") {
      const auto value = requireValue(index, argc, argv, argument);
      if (value == "tui") {
        options.tui_enabled = true;
      } else if (value == "none") {
        options.tui_enabled = false;
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
    } else if (argument == "--regularization") {
      options.regularization = parsePositiveDouble(
          "regularization", requireValue(index, argc, argv, argument));
    } else if (argument == "--maximum-iterations") {
      options.maximum_iterations = std::stoi(requireValue(index, argc, argv, argument));
      if (options.maximum_iterations <= 0) {
        throw std::runtime_error("maximum iterations must be positive");
      }
    } else if (argument == "--position-tolerance-m") {
      options.position_tolerance_m = parsePositiveDouble(
          "position tolerance", requireValue(index, argc, argv, argument));
    } else if (argument == "--orientation-tolerance-rad") {
      options.orientation_tolerance_rad = parsePositiveDouble(
          "orientation tolerance", requireValue(index, argc, argv, argument));
    } else if (argument == "--joint-position-margin-rad") {
      options.joint_position_margin_rad = parseNonnegativeDouble(
          "joint position margin", requireValue(index, argc, argv, argument));
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

} // namespace motion_control_lab::single_arm_servo_step
