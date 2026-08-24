#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

#include <motion_control_core/motion_control_core.hpp>
#include <motion_control_viz/render_batch.hpp>

#include "components/robot/r1/r1_robot_config.hpp"
#include "options.hpp"
#include "solver.hpp"

namespace motion_control_lab::planned_hierarchical_step {

int runLoop(Options options, const R1RobotConfig &robot, SolverRuntime &solver,
            const SolverHandles &handles,
            motion_control::core::CartesianPlanner &cartesian_planner,
            const std::vector<Eigen::Index> &active_joint_full_indices,
            std::string &normal_exit_detail);

void appendPlanningRequestPoses(motion_control::viz::RenderBatch &frame,
                                const std::string &reference_frame,
                                const Eigen::Isometry3d &left_pose,
                                const Eigen::Isometry3d &right_pose);

} // namespace motion_control_lab::planned_hierarchical_step
