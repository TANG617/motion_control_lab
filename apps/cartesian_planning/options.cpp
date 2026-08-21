#include "options.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace motion_control_lab::cartesian_planning {
namespace {

double parsePositiveCliDouble(const std::string &option,
                              const std::string &value) {
  (void)option;
  return std::stod(value);
}

double parseNonNegativeCliDouble(const std::string &option,
                                 const std::string &value) {
  (void)option;
  return std::stod(value);
}

} // namespace

void printUsage(const char *program) {
  std::cout
      << "Usage: " << program
      << " --request <json> --output-dir <dir> [options]\n\n"
      << "Options:\n"
      << "  --request <path>       Cartesian MoveLine request JSON (required)\n"
      << "  --output-dir <path>    CSV and PNG output directory (required)\n"
      << "  --host <address>       Foxglove bind address (default: 127.0.0.1)\n"
      << "  --port <port>          Foxglove port (default: 8765)\n"
      << "  --playback-rate <x>    Positive playback speed multiplier "
         "(default: 1)\n"
      << "  --loop-delay <sec>     Delay between loops (default: 1)\n"
      << "  --once                 Play once and exit\n"
      << "  --no-live              Generate CSV/PNG without Foxglove playback\n"
      << "  --mcap <path>          Record playback to a new MCAP file\n"
      << "  --force                Overwrite existing CSV/PNG outputs\n"
      << "  --help                 Show this help text\n";
}

AppOptions parseOptions(int argc, char **argv) {
  AppOptions options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument{argv[index]};
    auto requireValue = [&](const std::string &option) -> std::string {
      if (index + 1 >= argc) {
        throw std::runtime_error(option + " requires a value");
      }
      return argv[++index];
    };

    if (argument == "--help" || argument == "-h") {
      printUsage(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else if (argument == "--request") {
      options.request_path = requireValue(argument);
    } else if (argument == "--output-dir") {
      options.output_dir = requireValue(argument);
    } else if (argument == "--host") {
      options.host = requireValue(argument);
    } else if (argument == "--port") {
      const std::string port_text = requireValue(argument);
      options.port = static_cast<std::uint16_t>(std::stol(port_text));
    } else if (argument == "--playback-rate") {
      options.playback_rate =
          parsePositiveCliDouble(argument, requireValue(argument));
    } else if (argument == "--loop-delay") {
      options.loop_delay_s =
          parseNonNegativeCliDouble(argument, requireValue(argument));
    } else if (argument == "--once") {
      options.once = true;
    } else if (argument == "--no-live") {
      options.live = false;
    } else if (argument == "--mcap") {
      options.mcap_path = std::filesystem::path{requireValue(argument)};
    } else if (argument == "--force") {
      options.force = true;
    } else {
      throw std::runtime_error("unknown option: " + argument);
    }
  }

  if (options.request_path.empty()) {
    throw std::runtime_error("--request is required");
  }
  if (options.output_dir.empty()) {
    throw std::runtime_error("--output-dir is required");
  }
  if (!options.live && options.mcap_path.has_value()) {
    throw std::runtime_error("--mcap cannot be used with --no-live");
  }
  return options;
}

} // namespace motion_control_lab::cartesian_planning
