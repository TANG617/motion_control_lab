#pragma once

#include <motion_control_core/motion_control_core.hpp>

#include "components/robot/r1/r1_robot_config.hpp"
#include "options.hpp"
#include "solver.hpp"

namespace motion_control_lab::hierarchical_inverse_dynamics_torque_sim {

int runLoop(const Options &options, const R1RobotConfig &robot,
            Runtime &runtime, const Handles &handles);

} // namespace motion_control_lab::hierarchical_inverse_dynamics_torque_sim
