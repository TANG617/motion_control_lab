#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <string>
#include <vector>

#include <motion_control_core/motion_control_core.hpp>
#include <motion_control_viz/render_batch.hpp>

#include "components/robot/r1/r1_robot_config.hpp"
#include "options.hpp"
#include "planning.hpp"
#include "solver.hpp"

namespace motion_control_lab::planned_hierarchical_step_otg_nullspace {

struct Link4TargetSnapshot;

int runLoop(Options options, const R1RobotConfig &robot, SolverRuntime &solver,
            const SolverHandles &handles,
            motion_control::core::CartesianPlanner &cartesian_planner,
            motion_control::core::JointPlanner &joint_planner,
            const JointTargetLimits &joint_limits,
            const std::vector<std::size_t> &active_joint_full_indices,
            std::string &normal_exit_detail);

void appendPlanningRequestPoses(motion_control::viz::RenderBatch &frame,
                                const std::string &reference_frame,
                                const Eigen::Isometry3d &left_pose,
                                const Eigen::Isometry3d &right_pose);

void appendOtgExecution(motion_control::viz::RenderBatch &batch,
                        const std::vector<std::string> &joint_names,
                        const std::vector<double> &positions,
                        const std::vector<double> &velocities,
                        const std::string &reference_frame,
                        const Eigen::Isometry3d &left_pose,
                        const Eigen::Isometry3d &right_pose);

void appendNullspaceElbowScene(motion_control::viz::RenderBatch &batch,
                               const std::string &reference_frame,
                               const Link4TargetSnapshot &target,
                               const Eigen::Isometry3d &raw_left_pose,
                               const Eigen::Isometry3d &raw_right_pose,
                               const Eigen::Isometry3d &executed_left_pose,
                               const Eigen::Isometry3d &executed_right_pose);

} // namespace motion_control_lab::planned_hierarchical_step_otg_nullspace
