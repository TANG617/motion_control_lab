#pragma once

#include <filesystem>
#include <motion_control_core/planning/cartesian_planner.hpp>

namespace motion_control_lab::cartesian_planning {

struct OutputPaths {
  std::filesystem::path trajectory_csv;
  std::filesystem::path path_plot;
  std::filesystem::path profiles_plot;
};

motion_control::core::CartesianLineRequest
loadRequest(const std::filesystem::path &path);

OutputPaths prepareOutputPaths(const std::filesystem::path &output_dir,
                               bool force);

void writeTrajectoryCsv(
    const std::filesystem::path &path,
    const motion_control::core::CartesianTrajectory &trajectory);

void renderTrajectoryPlots(
    const OutputPaths &paths,
    const motion_control::core::CartesianTrajectory &trajectory);

} // namespace motion_control_lab::cartesian_planning
