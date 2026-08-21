#include "options.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "components/robot/r1/r1_robot_config.hpp"
#include "components/tui/tui_help.hpp"

namespace motion_control_lab::target_solve {
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

double parseNonnegativeDouble(const std::string &name,
                              const std::string &value) {
  const double parsed = std::stod(value);
  if (parsed < 0.0 || !std::isfinite(parsed)) {
    throw std::runtime_error(name + " must be a finite non-negative value");
  }
  return parsed;
}

bool parseAlgorithmOption(const std::string &argument, const std::string &value,
                          AlgorithmOptions &options) {
  if (argument == "--maximum-iterations") {
    options.maximum_iterations = std::stoi(value);
    if (options.maximum_iterations <= 0)
      throw std::runtime_error("maximum iterations must be positive");
  } else if (argument == "--solve-time-budget-ms") {
    options.soft_solve_time_budget_ms =
        parsePositiveDouble("solve time budget", value);
  } else if (argument == "--position-tolerance-m") {
    options.position_tolerance_m =
        parsePositiveDouble("position tolerance", value);
  } else if (argument == "--orientation-tolerance-rad") {
    options.orientation_tolerance_rad =
        parsePositiveDouble("orientation tolerance", value);
  } else if (argument == "--minimum-position-improvement-m") {
    options.minimum_position_improvement_m =
        parseNonnegativeDouble("position improvement", value);
  } else if (argument == "--minimum-orientation-improvement-rad") {
    options.minimum_orientation_improvement_rad =
        parseNonnegativeDouble("orientation improvement", value);
  } else if (argument == "--joint-position-margin-rad") {
    options.joint_position_margin_rad =
        parseNonnegativeDouble("joint position margin", value);
  } else if (argument == "--posture-weight") {
    options.posture_weight = parsePositiveDouble("posture weight", value);
  } else if (argument == "--regularization") {
    options.regularization = parsePositiveDouble("regularization", value);
  } else if (argument == "--proxqp-absolute-tolerance") {
    options.proxqp_absolute_tolerance =
        parsePositiveDouble("ProxQP absolute tolerance", value);
  } else {
    return false;
  }
  return true;
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

void printUsage(const char *program) {
  const AppOptions defaults;
  const auto &interactive = defaults.interactive;
  std::cout
      << "Usage: " << program << " [options]\n\n"
      << "Options:\n"
      << "  --side <left|right> Initial/selected arm side (default: "
      << interactive.tui.side << ")\n"
      << "  --urdf <path>       Robot URDF path (or $"
      << kMotionControlUrdfEnvironmentVariable << ")\n"
      << "  --host <address>    WebSocket bind address (default: "
      << interactive.visualization.host << ")\n"
      << "  --port <port>       WebSocket port (default: "
      << interactive.visualization.port << ")\n"
      << "  --rate <hz>         IK and publish rate in Hz (default: "
      << interactive.rate_hz << ")\n"
      << "  --ui <tui|none>     User interface mode (default: tui)\n"
      << "  --viz <foxglove|none> Visualization transport (default: foxglove)\n"
      << "  --duration <sec>    Stop after seconds; 0 runs until Ctrl-C "
         "(default: "
      << interactive.duration_s << ")\n"
      << "  --step-m <meters>   Cartesian increment per keypress (default: "
      << interactive.tui.step_m << ")\n"
      << "  --min-step-m <m>    Minimum step size (default: "
      << interactive.tui.min_step_m << ")\n"
      << "  --max-step-m <m>    Maximum step size (default: "
      << interactive.tui.max_step_m << ")\n"
      << "  --rotation-step-deg <deg> TCP local-axis rotation step (default: "
      << interactive.tui.rotation_step_deg << ")\n"
      << "  --mcap <path>       Write MCAP output to path when Foxglove is "
         "enabled\n"
      << "  --no-mcap           Disable MCAP output (default)\n"
      << "  --solver <mcc|placo>              IK implementation (default: "
         "mcc)\n"
      << "  --backend <proxqp|eiquadprog>     MCC QP backend (default: proxqp; "
         "ignored for placo)\n"
      << "  --maximum-iterations <count>      TargetSolve iteration budget\n"
      << "  --solve-time-budget-ms <value>    Soft solve time budget\n"
      << "  --position-tolerance-m <value>    Position tolerance\n"
      << "  --orientation-tolerance-rad <value> Orientation tolerance\n"
      << "  --minimum-position-improvement-m <value> No-progress threshold\n"
      << "  --minimum-orientation-improvement-rad <value> No-progress "
         "threshold\n"
      << "  --joint-position-margin-rad <value> Joint limit margin\n"
      << "  --posture-weight <value>          Posture regularization weight\n"
      << "  --regularization <value>          QP regularization\n"
      << "  --proxqp-absolute-tolerance <value> ProxQP tolerance\n"
      << "  --help              Show this help text\n\n";
  printTuiControlsHelp(std::cout);
}

AppOptions parseOptions(int argc, char **argv) {
  AppOptions result;
  auto &options = result.interactive;
  if (const char *configured_urdf =
          std::getenv(kMotionControlUrdfEnvironmentVariable)) {
    options.urdf_path = configured_urdf;
  }
  for (int index = 1; index < argc; ++index) {
    const std::string argument{argv[index]};
    if (argument == "--help" || argument == "-h") {
      printUsage(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else if (argument == "--solver") {
      const auto value = requireValue(index, argc, argv, argument);
      if (value == "mcc") {
        result.solver = SolverKind::Mcc;
      } else if (value == "placo") {
        result.solver = SolverKind::Placo;
      } else {
        throw std::runtime_error("--solver must be either 'mcc' or 'placo'");
      }
    } else if (argument == "--backend") {
      const auto value = requireValue(index, argc, argv, argument);
      if (value == "proxqp") {
        result.backend = MccBackend::Proxqp;
      } else if (value == "eiquadprog") {
        result.backend = MccBackend::Eiquadprog;
      } else {
        throw std::runtime_error(
            "--backend must be either 'proxqp' or 'eiquadprog'");
      }
    } else if (argument == "--maximum-iterations" ||
               argument == "--solve-time-budget-ms" ||
               argument == "--position-tolerance-m" ||
               argument == "--orientation-tolerance-rad" ||
               argument == "--minimum-position-improvement-m" ||
               argument == "--minimum-orientation-improvement-rad" ||
               argument == "--joint-position-margin-rad" ||
               argument == "--posture-weight" ||
               argument == "--regularization" ||
               argument == "--proxqp-absolute-tolerance") {
      parseAlgorithmOption(argument, requireValue(index, argc, argv, argument),
                           result.algorithm);
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
  return result;
}

} // namespace motion_control_lab::target_solve
