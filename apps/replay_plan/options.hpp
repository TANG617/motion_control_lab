#pragma once

#include "adapters/replay/replay_support.hpp"

namespace motion_control_lab::replay_plan {

using Options = replay::ReplayOptions;

Options parseOptions(int argc, char **argv);

} // namespace motion_control_lab::replay_plan
