#include "cartesian_planning.hpp"

#include <motion_control_core/motion_control_core.hpp>

#include <json/json.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>

namespace
{

namespace fs = std::filesystem;
namespace mcc = motion_control::core;
namespace mcp = motion_control_lab::cartesian_planning;

Json::Value loadJson(const fs::path & path)
{
  std::ifstream input(path);
  Json::CharReaderBuilder builder;
  Json::Value value;
  std::string errors;
  if (!input || !Json::parseFromStream(builder, input, &value, &errors)) {
    throw std::runtime_error("test failed to load JSON: " + errors);
  }
  return value;
}

void writeJson(const fs::path & path, const Json::Value & value)
{
  std::ofstream output(path, std::ios::trunc);
  Json::StreamWriterBuilder builder;
  output << Json::writeString(builder, value);
  if (!output) {
    throw std::runtime_error("test failed to write JSON");
  }
}

bool rejects(const std::function<void()> & operation)
{
  try {
    operation();
  } catch (const std::runtime_error &) {
    return true;
  }
  return false;
}

bool fileIsNonEmpty(const fs::path & path)
{
  return fs::is_regular_file(path) && fs::file_size(path) > 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    return EXIT_FAILURE;
  }
  const fs::path example_path{argv[1]};
  const fs::path output_dir{argv[2]};
  fs::remove_all(output_dir);
  fs::create_directories(output_dir);

  try {
    char program[] = "mcl_cartesian_planning";
    char request_option[] = "--request";
    char request_value[] = "/tmp/request.json";
    char output_option[] = "--output-dir";
    char output_value[] = "/tmp/output";
    char rate_option[] = "--playback-rate";
    char rate_value[] = "2.5";
    char no_live_option[] = "--no-live";
    char * app_argv[]{
      program,
      request_option,
      request_value,
      output_option,
      output_value,
      rate_option,
      rate_value,
      no_live_option};
    const auto options = mcp::parseAppOptions(8, app_argv);
    if (options.request_path != request_value || options.output_dir != output_value ||
      options.playback_rate != 2.5 || options.live)
    {
      return EXIT_FAILURE;
    }

    const auto request = mcp::loadRequest(example_path);
    if (request.reference_frame_name != "base_link" || request.segments.size() != 2 ||
      request.maximum_sample_count != 100000 ||
      request.synchronization != mcc::TrajectorySynchronization::Time ||
      request.segments[0].frame_name != "left_arm_ee_link" ||
      request.segments[1].current_twist.norm() != 0.0)
    {
      return EXIT_FAILURE;
    }

    mcc::CartesianPlanner planner;
    mcc::CartesianTrajectory trajectory;
    mcc::PlanningDiagnostics diagnostics;
    const auto status = planner.generate(request, trajectory, diagnostics);
    if (!status.ok() || trajectory.samples.empty() ||
      diagnostics.sample_count != trajectory.samples.size() ||
      trajectory.samples.front().frames.size() != 2 ||
      trajectory.samples.back().time_from_start != trajectory.duration ||
      !trajectory.samples.front().frames[0].pose.isApprox(request.segments[0].start_pose) ||
      !trajectory.samples.back().frames[0].pose.isApprox(request.segments[0].target_pose))
    {
      return EXIT_FAILURE;
    }

    const auto static_scene = mcp::makeStaticScene(request);
    if (static_scene.size() != 14) {
      return EXIT_FAILURE;
    }
    auto stationary_request = request;
    stationary_request.segments.resize(1);
    stationary_request.segments[0].target_pose =
      stationary_request.segments[0].start_pose;
    if (mcp::makeStaticScene(stationary_request).size() != 6) {
      return EXIT_FAILURE;
    }
    const auto playback = mcp::makePlaybackFrame(
      trajectory.samples.front(), static_scene, true, 9, 10, 11);
    if (playback.sequence != 9 || playback.poses.size() != 2 ||
      playback.line_strips.size() != static_scene.size() + 6 ||
      playback.poses[0].channel != "/mc/cartesian/pose/left_arm_ee_link")
    {
      return EXIT_FAILURE;
    }

    const auto paths = mcp::prepareOutputPaths(output_dir / "render", false);
    mcp::writeTrajectoryCsv(paths.trajectory_csv, trajectory);
    if (!fileIsNonEmpty(paths.trajectory_csv) ||
      !rejects([&]() {mcp::prepareOutputPaths(output_dir / "render", false);}))
    {
      return EXIT_FAILURE;
    }
    (void)mcp::prepareOutputPaths(output_dir / "render", true);

    const Json::Value example = loadJson(example_path);
    auto defaults = example;
    defaults.removeMember("maximum_sample_count");
    defaults.removeMember("synchronization");
    const auto defaults_path = output_dir / "defaults.json";
    writeJson(defaults_path, defaults);
    const auto default_request = mcp::loadRequest(defaults_path);
    if (default_request.maximum_sample_count != mcc::kDefaultMaximumSampleCount ||
      default_request.synchronization != mcc::TrajectorySynchronization::Time)
    {
      return EXIT_FAILURE;
    }

    auto phase = example;
    phase["synchronization"] = "phase";
    const auto phase_path = output_dir / "phase.json";
    writeJson(phase_path, phase);
    if (mcp::loadRequest(phase_path).synchronization !=
      mcc::TrajectorySynchronization::Phase)
    {
      return EXIT_FAILURE;
    }

  } catch (const std::exception &) {
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
