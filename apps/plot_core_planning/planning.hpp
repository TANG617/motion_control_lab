#pragma once

#include <filesystem>

#include <motion_control_core/planning/cartesian_planner.hpp>
#include <motion_control_core/planning/joint_planner.hpp>

namespace motion_control_lab::plot_core_planning {

void initializePlotBackend();

motion_control::core::CartesianLineRequest makeCartesianRequest();
motion_control::core::JointTrajectoryRequest makeJointRequest();

void saveCartesianPlot(
    const motion_control::core::CartesianTrajectory &trajectory,
    const std::filesystem::path &path);

void saveJointPlot(const motion_control::core::JointTrajectory &trajectory,
                   const std::filesystem::path &path);

} // namespace motion_control_lab::plot_core_planning
