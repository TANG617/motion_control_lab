#include "options.hpp"

namespace motion_control_lab::replay_plan {

Options parseOptions(int argc, char **argv) {
  return replay::parseReplayOptions(argc, argv, false);
}

} // namespace motion_control_lab::replay_plan
