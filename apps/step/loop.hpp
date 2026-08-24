#pragma once

#include <string>
#include <utility>
#include <vector>

#include "adapters/replay/replay_support.hpp"
#include "options.hpp"
#include "solver.hpp"

namespace motion_control_lab::step {

std::pair<std::vector<double>, std::vector<double>>
makeReplayInitialState(const replay::LoadedReplay &loaded,
                       const R1RobotConfig &robot);

int runLoop(const AppOptions &options, const std::string &solver_id,
            const std::string &solver_title, MccServoSolver &solver);
int runLoop(const AppOptions &options, const std::string &solver_id,
            const std::string &solver_title, PlacoServoSolver &solver);

int runReplayLoop(ReplayAppOptions options, const std::string &solver_id,
                  const std::string &solver_title, MccServoSolver &solver,
                  const replay::LoadedReplay &loaded);
int runReplayLoop(ReplayAppOptions options, const std::string &solver_id,
                  const std::string &solver_title, PlacoServoSolver &solver,
                  const replay::LoadedReplay &loaded);

} // namespace motion_control_lab::step
