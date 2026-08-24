#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

#include <motion_control_core/motion_control_core.hpp>

#include "components/robot/r1/r1_robot_config.hpp"
#include "options.hpp"
#include "solver.hpp"

namespace motion_control_lab::hierarchical_step {

int runLoop(LaunchOptions launch, const R1RobotConfig &robot,
            SolverRuntime &solver, const SolverHandles &handles,
            const std::vector<Eigen::Index> &active_joint_full_indices,
            std::string &normal_exit_detail);

} // namespace motion_control_lab::hierarchical_step
