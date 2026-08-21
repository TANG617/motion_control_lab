#pragma once

#include <cstdint>
#include <vector>

#include <motion_control_core/planning/cartesian_planner.hpp>
#include <motion_control_viz/render_batch.hpp>

#include "options.hpp"

namespace motion_control_lab::cartesian_planning {

std::vector<motion_control::viz::LineStrip3d>
makeStaticScene(const motion_control::core::CartesianLineRequest &request);

motion_control::viz::RenderBatch makePlaybackFrame(
    const motion_control::core::CartesianTrajectorySample &sample,
    const std::vector<motion_control::viz::LineStrip3d> &static_scene,
    bool include_static_scene, std::uint64_t timestamp_ns);

void playTrajectory(
    const AppOptions &options,
    const motion_control::core::CartesianLineRequest &request,
    const motion_control::core::CartesianTrajectory &trajectory);

} // namespace motion_control_lab::cartesian_planning
