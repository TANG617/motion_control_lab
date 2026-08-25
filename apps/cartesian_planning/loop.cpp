#include "loop.hpp"

#include "components/visualization/preview_transport.hpp"
#include "contracts/visualization/mcl_planning_v1.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace motion_control_lab::cartesian_planning {
namespace {

namespace mcc = motion_control::core;
namespace mcv = motion_control::viz;
constexpr const char *kSceneChannel = "/mcl/cartesian/scene";
constexpr double kAxisLengthM = 0.05;

std::atomic_bool stop_requested{false};

void signalHandler(int) { stop_requested.store(true); }

std::array<double, 4> paletteColor(std::size_t index) {
  constexpr std::array<std::array<double, 4>, 8> palette{
      {{{0.121, 0.466, 0.705, 1.0}},
       {{1.000, 0.498, 0.054, 1.0}},
       {{0.172, 0.627, 0.172, 1.0}},
       {{0.839, 0.153, 0.157, 1.0}},
       {{0.580, 0.404, 0.741, 1.0}},
       {{0.549, 0.337, 0.294, 1.0}},
       {{0.890, 0.467, 0.761, 1.0}},
       {{0.498, 0.498, 0.498, 1.0}}}};
  return palette[index % palette.size()];
}

mcv::ColorRgba toColor(const std::array<double, 4> &value) {
  return {value[0], value[1], value[2], value[3]};
}

std::array<double, 3> toPoint(const Eigen::Vector3d &value) {
  return {value.x(), value.y(), value.z()};
}

void appendAxisTriad(std::vector<mcv::LineStrip3d> &lines,
                     const std::string &entity_prefix,
                     const std::string &frame_id, const mcc::Pose &pose,
                     double alpha) {
  constexpr std::array<std::array<double, 4>, 3> axis_colors{
      {{{1.0, 0.0, 0.0, 1.0}}, {{0.0, 1.0, 0.0, 1.0}}, {{0.0, 0.4, 1.0, 1.0}}}};
  const Eigen::Vector3d origin = pose.translation();
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const Eigen::Vector3d end =
        origin +
        kAxisLengthM * pose.linear().col(static_cast<Eigen::Index>(axis));
    auto color = axis_colors[axis];
    color[3] = alpha;
    lines.push_back(
        mcv::LineStrip3d{entity_prefix + "/" + std::string{"xyz"[axis]},
                         kSceneChannel,
                         frame_id,
                         {toPoint(origin), toPoint(end)},
                         toColor(color),
                         3.0,
                         true});
  }
}

std::uint64_t wallTimeNs() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

const char *referenceTopic(const std::string &frame_name) {
  namespace contract = motion_control_lab::contracts::mcl_planning_v1;
  if (frame_name.find("left") != std::string::npos) {
    return contract::kLeftCartesianReferenceTopic;
  }
  if (frame_name.find("right") != std::string::npos) {
    return contract::kRightCartesianReferenceTopic;
  }
  throw std::runtime_error(
      "Cartesian playback frame cannot be mapped to MCL left/right reference topic: " +
      frame_name);
}

void waitUntil(const std::chrono::steady_clock::time_point &deadline) {
  constexpr auto poll_interval = std::chrono::milliseconds(20);
  while (!stop_requested.load()) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return;
    }
    std::this_thread::sleep_until(std::min(deadline, now + poll_interval));
  }
}

} // namespace

std::vector<mcv::LineStrip3d>
makeStaticScene(const mcc::CartesianLineRequest &request) {
  std::vector<mcv::LineStrip3d> lines;
  for (std::size_t index = 0; index < request.segments.size(); ++index) {
    const auto &segment = request.segments[index];
    const Eigen::Vector3d start = segment.start_pose.translation();
    const Eigen::Vector3d target = segment.target_pose.translation();
    if ((target - start).norm() > 1.0e-12) {
      lines.push_back(mcv::LineStrip3d{segment.frame_name + "/path",
                                       kSceneChannel,
                                       request.reference_frame_name,
                                       {toPoint(start), toPoint(target)},
                                       toColor(paletteColor(index)),
                                       4.0,
                                       true});
    }
    appendAxisTriad(lines, segment.frame_name + "/start",
                    request.reference_frame_name, segment.start_pose, 0.55);
    appendAxisTriad(lines, segment.frame_name + "/target",
                    request.reference_frame_name, segment.target_pose, 0.75);
  }
  return lines;
}

mcv::RenderBatch
makePlaybackFrame(const mcc::CartesianTrajectorySample &sample,
                  const std::vector<mcv::LineStrip3d> &static_scene,
                  bool include_static_scene, std::uint64_t timestamp_ns) {
  mcv::RenderBatch visualization;
  visualization.timestamp_ns = timestamp_ns;
  if (include_static_scene) {
    visualization.line_strips = static_scene;
  }
  visualization.poses.reserve(sample.frames.size());
  for (const auto &frame : sample.frames) {
    Eigen::Quaterniond orientation(frame.pose.linear());
    orientation.normalize();
    visualization.poses.push_back(mcv::PoseSample{
        referenceTopic(frame.frame_name), frame.reference_frame_name,
        mcv::Pose3d{toPoint(frame.pose.translation()),
                    {orientation.x(), orientation.y(), orientation.z(),
                     orientation.w()}}});
    appendAxisTriad(visualization.line_strips, frame.frame_name + "/current",
                    frame.reference_frame_name, frame.pose, 1.0);
  }
  return visualization;
}

void playTrajectory(const AppOptions &options,
                    const mcc::CartesianLineRequest &request,
                    const mcc::CartesianTrajectory &trajectory) {
  PreviewSinkOptions sink_options;
  sink_options.host = options.host;
  sink_options.port = options.port;
  sink_options.mcap_path = options.mcap_path;
  auto sink = createPreviewSink(sink_options, "mcl_cartesian_planning");
  sink->open();

  stop_requested.store(false);
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  const auto static_scene = makeStaticScene(request);
  do {
    const auto loop_start = std::chrono::steady_clock::now();
    bool first_sample = true;
    for (const auto &sample : trajectory.samples) {
      const auto offset = std::chrono::duration<double>(sample.time_from_start /
                                                        options.playback_rate);
      waitUntil(loop_start +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    offset));
      if (stop_requested.load()) {
        break;
      }
      sink->write(
          makePlaybackFrame(sample, static_scene, first_sample, wallTimeNs()));
      first_sample = false;
    }
    sink->flush();
    if (options.once || stop_requested.load()) {
      break;
    }
    waitUntil(std::chrono::steady_clock::now() +
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>(options.loop_delay_s)));
  } while (!stop_requested.load());
  sink->close();
}

} // namespace motion_control_lab::cartesian_planning
