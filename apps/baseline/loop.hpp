#pragma once

#include "adapters/replay/replay_support.hpp"
#include "options.hpp"
#include "solver.hpp"

namespace motion_control_lab::baseline {

int runLoop(const TeleopOptions &options, BaselineSolver &solver);

int runReplayLoop(ReplayAppOptions options, BaselineSolver &solver,
                  const replay::LoadedReplay &loaded);

} // namespace motion_control_lab::baseline
