#include "options.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace motion_control_lab::hierarchical_inverse_dynamics_torque_sim {

void printUsage(const char *program) {
  std::cout
      << "Usage: " << program
      << " [teleop] [--duration SEC] [--retarget-x M] [--admittance]"
         " [--paused] [--single-steps N] [--side left|right]"
         " [--step-m M] [--min-step-m M] [--max-step-m M]"
         " [--rotation-step-deg DEG] [--urdf PATH]"
         " [--mujoco-model PATH]\n"
         "  --ui <tui|none>       Full-screen TUI (default: tui)\n"
         "  --viz <foxglove|none> Foxglove transport (default: foxglove)\n"
         "  --host ADDRESS        Foxglove bind address (default: 127.0.0.1)\n"
         "  --port PORT           Foxglove port (default: 8765)\n"
         "  --mcap PATH           Also record the Foxglove stream to MCAP\n"
         "  --ui-rate HZ          TUI, Foxglove and viewer rate (default: "
         "100)\n"
         "  --mujoco-viewer / --no-mujoco-viewer (default: viewer)\n"
         "Keyboard: Left/Right select arm; W/S X; A/D Y; Q/E Z;"
         " N axis; I/U rotate; Up/Down step; R reset; Space pause;"
         " X or Esc exit. MuJoCo: Ctrl+drag a hand marker.\n";
}

Options parseOptions(int argc, char **argv) {
  Options options;
  options.urdf_path = MCL_R1_MUJOCO_URDF_PATH;
  options.mujoco_path = MCL_R1_MUJOCO_MODEL_PATH;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      printUsage(argv[0]);
      std::exit(EXIT_SUCCESS);
    }
    if (argument == "teleop") {
      options.keyboard_enabled = true;
      continue;
    }
    const auto value = [&](const char *name) -> std::string {
      if (++index >= argc) {
        throw std::invalid_argument(std::string{name} + " requires a value");
      }
      return argv[index];
    };
    if (argument == "--duration") {
      options.duration_seconds = std::stod(value("--duration"));
    } else if (argument == "--retarget-x") {
      options.retarget_x_m = std::stod(value("--retarget-x"));
    } else if (argument == "--single-steps") {
      options.single_step_count = std::stoul(value("--single-steps"));
    } else if (argument == "--urdf") {
      options.urdf_path = value("--urdf");
    } else if (argument == "--mujoco" || argument == "--mujoco-model") {
      options.mujoco_path = value("--mujoco-model");
    } else if (argument == "--side") {
      options.keyboard.side = value("--side");
    } else if (argument == "--step" || argument == "--step-m") {
      options.keyboard.step_m = std::stod(value("--step-m"));
    } else if (argument == "--min-step" || argument == "--min-step-m") {
      options.keyboard.min_step_m = std::stod(value("--min-step-m"));
    } else if (argument == "--max-step" || argument == "--max-step-m") {
      options.keyboard.max_step_m = std::stod(value("--max-step-m"));
    } else if (argument == "--rotation-step-deg") {
      options.keyboard.rotation_step_deg =
          std::stod(value("--rotation-step-deg"));
    } else if (argument == "--admittance") {
      options.admittance_enabled = true;
    } else if (argument == "--paused") {
      options.start_paused = true;
    } else if (argument == "--ui-rate") {
      options.ui_rate_hz = std::stod(value("--ui-rate"));
    } else if (argument == "--ui") {
      const auto mode = value("--ui");
      if (mode != "tui" && mode != "none") {
        throw std::invalid_argument("--ui must be tui or none");
      }
      options.presentation.enabled = mode == "tui";
    } else if (argument == "--viz") {
      const auto mode = value("--viz");
      if (mode != "foxglove" && mode != "none") {
        throw std::invalid_argument("--viz must be foxglove or none");
      }
      options.visualization.enabled = mode == "foxglove";
    } else if (argument == "--host") {
      options.visualization.host = value("--host");
    } else if (argument == "--port") {
      const int port = std::stoi(value("--port"));
      if (port <= 0 || port > 65535) {
        throw std::invalid_argument("--port must be in [1, 65535]");
      }
      options.visualization.port = static_cast<std::uint16_t>(port);
    } else if (argument == "--mcap") {
      options.visualization.mcap_path = value("--mcap");
    } else if (argument == "--no-mcap") {
      options.visualization.mcap_path.reset();
    } else if (argument == "--mujoco-viewer") {
      options.viewer_enabled = true;
    } else if (argument == "--no-mujoco-viewer") {
      options.viewer_enabled = false;
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  if (options.urdf_path.empty() || options.mujoco_path.empty() ||
      !std::isfinite(options.duration_seconds) ||
      options.duration_seconds < 0.0 || !std::isfinite(options.retarget_x_m) ||
      !std::isfinite(options.ui_rate_hz) || options.ui_rate_hz <= 0.0 ||
      options.visualization.host.empty() || options.visualization.port == 0) {
    throw std::invalid_argument("invalid torque-simulation options");
  }
  return options;
}

} // namespace motion_control_lab::hierarchical_inverse_dynamics_torque_sim
