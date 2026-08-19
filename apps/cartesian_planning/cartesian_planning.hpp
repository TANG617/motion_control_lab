#pragma once

#include <motion_control_core/planning/cartesian_planner.hpp>
#include <motion_control_viz/render_batch.hpp>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace motion_control_lab::cartesian_planning
{

struct AppOptions
{
  std::filesystem::path request_path;
  std::filesystem::path output_dir;
  std::string host{"127.0.0.1"};
  std::uint16_t port{8765};
  double playback_rate{1.0};
  double loop_delay_s{1.0};
  bool once{false};
  bool live{true};
  bool force{false};
  std::optional<std::filesystem::path> mcap_path;
};

struct OutputPaths
{
  std::filesystem::path trajectory_csv;
  std::filesystem::path path_plot;
  std::filesystem::path profiles_plot;
};

void printUsage(const char * program);

AppOptions parseAppOptions(int argc, char ** argv);

motion_control::core::CartesianLineRequest loadRequest(
  const std::filesystem::path & path);

OutputPaths prepareOutputPaths(
  const std::filesystem::path & output_dir,
  bool force);

void writeTrajectoryCsv(
  const std::filesystem::path & path,
  const motion_control::core::CartesianTrajectory & trajectory);

void renderTrajectoryPlots(
  const OutputPaths & paths,
  const motion_control::core::CartesianTrajectory & trajectory);

std::vector<motion_control::viz::LineStrip3d> makeStaticScene(
  const motion_control::core::CartesianLineRequest & request);

motion_control::viz::RenderBatch makePlaybackFrame(
  const motion_control::core::CartesianTrajectorySample & sample,
  const std::vector<motion_control::viz::LineStrip3d> & static_scene,
  bool include_static_scene,
  std::uint64_t timestamp_ns);

void playTrajectory(
  const AppOptions & options,
  const motion_control::core::CartesianLineRequest & request,
  const motion_control::core::CartesianTrajectory & trajectory);

}  // namespace motion_control_lab::cartesian_planning
