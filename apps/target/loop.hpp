#pragma once

#include <string>

#include "options.hpp"
#include "solver.hpp"

namespace motion_control_lab::target {

int runLoop(const AppOptions &options, const std::string &solver_id,
            const std::string &solver_title, MccTargetSolver &solver);

int runLoop(const AppOptions &options, const std::string &solver_id,
            const std::string &solver_title, PlacoTargetSolver &solver);

} // namespace motion_control_lab::target
