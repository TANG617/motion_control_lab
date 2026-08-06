#include "sinks/ik_visualization.hpp"

#include <Eigen/Geometry>

#include <stdexcept>
#include <string>
#include <utility>

namespace motion_control_lab
{

motion_control::viz::VisualizationFrame makeIkVisualizationFrame(
  const IkDebugFrame & frame,
  const InteractiveIkPresentation & presentation,
  std::uint64_t sequence,
  std::int64_t sample_time_ns,
  std::uint64_t emit_time_ns)
{
  namespace mcv = motion_control::viz;

  mcv::VisualizationFrame visualization;
  visualization.run_id = "interactive-preview";
  visualization.sequence = sequence;
  visualization.sample_time_ns = sample_time_ns;
  visualization.sample_clock = "interactive_steady";
  visualization.emit_time_ns = emit_time_ns;
  visualization.status = frame.status;
  visualization.paused = frame.paused;

  visualization.poses.reserve(frame.targets.size());
  for (const auto & target : frame.targets) {
    const auto * arm = findArmPresentation(presentation, target.side);
    if (arm == nullptr) {
      throw std::runtime_error(
              std::string{"visualization presentation is missing the "} +
              armSideName(target.side) + " arm");
    }
    const Eigen::Quaterniond orientation(target.target_pose.linear());
    mcv::NamedPose pose;
    pose.entity_id = std::string{armSideName(target.side)} + "_target";
    pose.channel = arm->target_channel;
    pose.frame_id = presentation.base_frame_id;
    pose.pose.position_m = {
      target.target_pose.translation().x(),
      target.target_pose.translation().y(),
      target.target_pose.translation().z()};
    pose.pose.orientation_xyzw = {
      orientation.x(), orientation.y(), orientation.z(), orientation.w()};
    visualization.poses.push_back(std::move(pose));
  }

  visualization.joints = mcv::JointStateFrame{
    presentation.joint_state_channel,
    frame.joint_names,
    frame.positions,
    frame.velocities};
  visualization.diagnostics = {
    {"ik.iterations", static_cast<double>(frame.iterations), "count"},
    {"ik.converged", frame.converged ? 1.0 : 0.0, "bool"},
    {"ik.solve_time", frame.solve_time_ms, "ms"}};
  for (const auto & error : frame.target_errors) {
    const std::string prefix = std::string{"ik."} + armSideName(error.side);
    visualization.diagnostics.push_back(
      {prefix + ".position_error", error.position_m, "m"});
    visualization.diagnostics.push_back(
      {prefix + ".orientation_error", error.orientation_rad, "rad"});
  }
  return visualization;
}

}  // namespace motion_control_lab
