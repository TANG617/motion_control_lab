#pragma once

#include <motion_control_core/motion_control_core.hpp>

#include "components/robot/r1/r1_robot_config.hpp"
#include "options.hpp"
#include "solver.hpp"

namespace motion_control_lab::single_arm_step {

int runLoop(const AppOptions &options, const R1RobotConfig &robot,
            ArmSide controlled_side, mcc::KinematicsSolver &solver,
            const SolverHandles &handles);

} // namespace motion_control_lab::single_arm_step
