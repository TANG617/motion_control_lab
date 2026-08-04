#include "config/r1_ik_options.hpp"

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

void printR1IkUsage(const char * program)
{
  std::cout
    << "Usage: " << program << " [options]\n\n"
    << "Options:\n"
    << "  --side <left|right> Initial/selected arm side (default: left)\n"
    << "  --urdf <path>        R1 URDF path (or $" << kR1UrdfEnvironmentVariable << ")\n"
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

R1IkOptions parseR1IkOptions(int argc, char ** argv)
{
  R1IkOptions options;
  if (const char * configured_urdf = std::getenv(kR1UrdfEnvironmentVariable)) {
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
      printR1IkUsage(argv[0]);
      std::exit(0);
    } else if (arg == "--side") {
      options.side = parseSide(requireValue(arg));
    } else if (arg == "--urdf") {
      options.urdf_path = requireValue(arg);
    } else if (arg == "--host") {
      options.host = requireValue(arg);
    } else if (arg == "--port") {
      options.port = parsePort(requireValue(arg));
    } else if (arg == "--rate") {
      options.rate_hz = parsePositiveDouble("rate", requireValue(arg));
    } else if (arg == "--duration") {
      const double duration = std::stod(requireValue(arg));
      if (duration < 0.0 || !std::isfinite(duration)) {
        throw std::runtime_error("duration must be finite and non-negative");
      }
      options.duration_s = duration;
    } else if (arg == "--step-m") {
      options.step_m = parsePositiveDouble("step", requireValue(arg));
    } else if (arg == "--min-step-m") {
      options.min_step_m = parsePositiveDouble("minimum step", requireValue(arg));
    } else if (arg == "--max-step-m") {
      options.max_step_m = parsePositiveDouble("maximum step", requireValue(arg));
    } else if (arg == "--rotation-step-deg") {
      options.rotation_step_deg = parsePositiveDouble("rotation step", requireValue(arg));
    } else if (arg == "--mcap") {
      options.mcap_path = std::filesystem::path{requireValue(arg)};
    } else if (arg == "--no-mcap") {
      options.mcap_path.reset();
    } else {
      throw std::runtime_error("unknown option: " + arg);
    }
  }
  if (options.max_step_m < options.min_step_m) {
    throw std::runtime_error("--max-step-m must be greater than or equal to --min-step-m");
  }
  if (options.step_m < options.min_step_m || options.step_m > options.max_step_m) {
    throw std::runtime_error("--step-m must be inside [--min-step-m, --max-step-m]");
  }
  if (options.urdf_path.empty()) {
    throw std::runtime_error(
            std::string{"--urdf is required unless "} +
            kR1UrdfEnvironmentVariable + " is set");
  }
  return options;
}

}  // namespace motion_control_lab
