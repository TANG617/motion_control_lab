#include "config/interactive_ik_options.hpp"

#include "config/constants.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace motion_control_lab
{
namespace
{

uint16_t parsePort(const std::string & value)
{
  const long parsed = std::stol(value);
  if (parsed <= 0 || parsed > 65535) {
    throw std::runtime_error("port must be in [1, 65535]");
  }
  return static_cast<uint16_t>(parsed);
}

double parsePositiveDouble(const std::string & name, const std::string & value)
{
  const double parsed = std::stod(value);
  if (parsed <= 0.0 || !std::isfinite(parsed)) {
    throw std::runtime_error(name + " must be a positive finite value");
  }
  return parsed;
}

std::string parseSide(const std::string & value)
{
  if (value == "left" || value == "right") {
    return value;
  }
  throw std::runtime_error("side must be either 'left' or 'right'");
}

}  // namespace

void printInteractiveIkUsage(const char * program)
{
  std::cout
    << "Usage: " << program << " [options]\n\n"
    << "Options:\n"
    << "  --side <left|right> Initial/selected arm side (default: left)\n"
    << "  --urdf <path>       Robot URDF path (or $" << kUrdfEnvironmentVariable << ")\n"
    << "  --host <address>    WebSocket bind address (default: 127.0.0.1)\n"
    << "  --port <port>       WebSocket port (default: 8765)\n"
    << "  --rate <hz>         IK and publish rate in Hz (default: 20)\n"
    << "  --duration <sec>    Stop after seconds; 0 runs until Ctrl-C (default: 0)\n"
    << "  --step-m <meters>   Cartesian increment per keypress (default: 0.01)\n"
    << "  --min-step-m <m>    Minimum step size (default: 0.001)\n"
    << "  --max-step-m <m>    Maximum step size (default: 0.1)\n"
    << "  --rotation-step-deg <deg>\n"
    << "                      TCP local-axis rotation step (default: 5)\n"
    << "  --mcap <path>       Write MCAP output to path when Foxglove is enabled\n"
    << "  --no-mcap           Disable MCAP output (default)\n"
    << "  --help              Show this help text\n\n"
    << "TUI controls:\n"
    << "  w/s: +x/-x, a/d: +y/-y, q/e: +z/-z\n"
    << "  n: cycle TCP rotation axis, i: clockwise, u: counter-clockwise\n"
    << "  left/right arrows: switch arm when the app controls both arms\n"
    << "  up/down arrows: double/halve step size\n"
    << "  m: enter step size, r: reset target from current FK\n"
    << "  space: pause/resume publishing, h: show/hide help, x or Esc: exit\n";
}

InteractiveIkOptions parseInteractiveIkOptions(int argc, char ** argv)
{
  InteractiveIkOptions options;
  if (const char * configured_urdf = std::getenv(kUrdfEnvironmentVariable)) {
    options.urdf_path = configured_urdf;
  }
  for (int i = 1; i < argc; ++i) {
    const std::string arg{argv[i]};
    auto requireValue = [&](const std::string & option) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(option + " requires a value");
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      printInteractiveIkUsage(argv[0]);
      std::exit(0);
    } else if (arg == "--side") {
      options.tui.side = parseSide(requireValue(arg));
    } else if (arg == "--urdf") {
      options.urdf_path = requireValue(arg);
    } else if (arg == "--host") {
      options.visualization.host = requireValue(arg);
    } else if (arg == "--port") {
      options.visualization.port = parsePort(requireValue(arg));
    } else if (arg == "--rate") {
      options.rate_hz = parsePositiveDouble("rate", requireValue(arg));
    } else if (arg == "--duration") {
      const double duration = std::stod(requireValue(arg));
      if (duration < 0.0 || !std::isfinite(duration)) {
        throw std::runtime_error("duration must be finite and non-negative");
      }
      options.duration_s = duration;
    } else if (arg == "--step-m") {
      options.tui.step_m = parsePositiveDouble("step", requireValue(arg));
    } else if (arg == "--min-step-m") {
      options.tui.min_step_m = parsePositiveDouble("minimum step", requireValue(arg));
    } else if (arg == "--max-step-m") {
      options.tui.max_step_m = parsePositiveDouble("maximum step", requireValue(arg));
    } else if (arg == "--rotation-step-deg") {
      options.tui.rotation_step_deg = parsePositiveDouble("rotation step", requireValue(arg));
    } else if (arg == "--mcap") {
      options.visualization.mcap_path = std::filesystem::path{requireValue(arg)};
    } else if (arg == "--no-mcap") {
      options.visualization.mcap_path.reset();
    } else {
      throw std::runtime_error("unknown option: " + arg);
    }
  }
  if (options.tui.max_step_m < options.tui.min_step_m) {
    throw std::runtime_error("--max-step-m must be greater than or equal to --min-step-m");
  }
  if (options.tui.step_m < options.tui.min_step_m ||
      options.tui.step_m > options.tui.max_step_m) {
    throw std::runtime_error("--step-m must be inside [--min-step-m, --max-step-m]");
  }
  if (options.urdf_path.empty()) {
    throw std::runtime_error(
            std::string{"--urdf is required unless "} +
            kUrdfEnvironmentVariable + " is set");
  }
  return options;
}

void printGroupedInteractiveIkUsage(const char * program)
{
  std::cout
    << "Usage: " << program << " [options]\n\n"
    << "Options:\n"
    << "  --side <left|right> Initial selected arm side (default: left)\n"
    << "  --urdf <path>       Robot URDF path (or $" << kUrdfEnvironmentVariable << ")\n"
    << "  --host <address>    WebSocket bind address (default: 127.0.0.1)\n"
    << "  --port <port>       WebSocket port (default: 8765)\n"
    << "  --red-rate <hz>     Red servo rate/deadline (default: 1000)\n"
    << "  --yellow-rate <hz>  Yellow proposal rate/deadline (default: 100)\n"
    << "  --green-rate <hz>   Green proposal rate/deadline (default: 10)\n"
    << "  --ui-rate <hz>      TUI and visualization rate (default: 20)\n"
    << "  --duration <sec>    Stop after seconds; 0 runs until Ctrl-C (default: 0)\n"
    << "  --step-m <meters>   Cartesian increment per keypress (default: 0.01)\n"
    << "  --min-step-m <m>    Minimum step size (default: 0.001)\n"
    << "  --max-step-m <m>    Maximum step size (default: 0.1)\n"
    << "  --rotation-step-deg <deg> TCP rotation step (default: 5)\n"
    << "  --mcap <path>       Write MCAP from the UI thread when Foxglove is enabled\n"
    << "  --no-mcap           Disable MCAP output (default)\n"
    << "  --help              Show this help text\n\n"
    << "Rates must satisfy red > yellow > green > 0. Each group period is its deadline.\n";
}

GroupedInteractiveIkOptions parseGroupedInteractiveIkOptions(int argc, char ** argv)
{
  GroupedInteractiveIkOptions options;
  if (const char * configured_urdf = std::getenv(kUrdfEnvironmentVariable)) {
    options.urdf_path = configured_urdf;
  }
  for (int i = 1; i < argc; ++i) {
    const std::string arg{argv[i]};
    auto requireValue = [&](const std::string & option) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(option + " requires a value");
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      printGroupedInteractiveIkUsage(argv[0]);
      std::exit(0);
    } else if (arg == "--side") {
      options.tui.side = parseSide(requireValue(arg));
    } else if (arg == "--urdf") {
      options.urdf_path = requireValue(arg);
    } else if (arg == "--host") {
      options.visualization.host = requireValue(arg);
    } else if (arg == "--port") {
      options.visualization.port = parsePort(requireValue(arg));
    } else if (arg == "--red-rate") {
      options.red_rate_hz = parsePositiveDouble("red rate", requireValue(arg));
    } else if (arg == "--yellow-rate") {
      options.yellow_rate_hz = parsePositiveDouble("yellow rate", requireValue(arg));
    } else if (arg == "--green-rate") {
      options.green_rate_hz = parsePositiveDouble("green rate", requireValue(arg));
    } else if (arg == "--ui-rate") {
      options.ui_rate_hz = parsePositiveDouble("UI rate", requireValue(arg));
    } else if (arg == "--duration") {
      const double duration = std::stod(requireValue(arg));
      if (duration < 0.0 || !std::isfinite(duration)) {
        throw std::runtime_error("duration must be finite and non-negative");
      }
      options.duration_s = duration;
    } else if (arg == "--step-m") {
      options.tui.step_m = parsePositiveDouble("step", requireValue(arg));
    } else if (arg == "--min-step-m") {
      options.tui.min_step_m = parsePositiveDouble("minimum step", requireValue(arg));
    } else if (arg == "--max-step-m") {
      options.tui.max_step_m = parsePositiveDouble("maximum step", requireValue(arg));
    } else if (arg == "--rotation-step-deg") {
      options.tui.rotation_step_deg = parsePositiveDouble("rotation step", requireValue(arg));
    } else if (arg == "--mcap") {
      options.visualization.mcap_path = std::filesystem::path{requireValue(arg)};
    } else if (arg == "--no-mcap") {
      options.visualization.mcap_path.reset();
    } else {
      throw std::runtime_error("unknown option: " + arg);
    }
  }

  if (!(options.red_rate_hz > options.yellow_rate_hz &&
      options.yellow_rate_hz > options.green_rate_hz))
  {
    throw std::runtime_error("rates must satisfy red > yellow > green > 0");
  }
  if (options.tui.max_step_m < options.tui.min_step_m) {
    throw std::runtime_error("--max-step-m must be greater than or equal to --min-step-m");
  }
  if (options.tui.step_m < options.tui.min_step_m ||
      options.tui.step_m > options.tui.max_step_m)
  {
    throw std::runtime_error("--step-m must be inside [--min-step-m, --max-step-m]");
  }
  if (options.urdf_path.empty()) {
    throw std::runtime_error(
            std::string{"--urdf is required unless "} +
            kUrdfEnvironmentVariable + " is set");
  }
  return options;
}

}  // namespace motion_control_lab
