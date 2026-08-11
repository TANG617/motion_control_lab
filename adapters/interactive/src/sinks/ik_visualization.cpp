#include "sinks/ik_visualization.hpp"

#include <Eigen/Geometry>

#include <algorithm>
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

  visualization.poses.reserve(presentation.arms.size() * 2);
  for (const auto & arm : presentation.arms) {
    const auto target = std::find_if(
      frame.targets.begin(), frame.targets.end(),
      [&arm](const ArmTarget & value) { return value.side == arm.side; });
    if (target == frame.targets.end()) {
      throw std::runtime_error(
              std::string{"visualization frame is missing the "} +
              armSideName(arm.side) + " target");
    }
    const Eigen::Quaterniond orientation(target->target_pose.linear());
    mcv::NamedPose pose;
    pose.entity_id = std::string{armSideName(arm.side)} + "_target";
    pose.channel = arm.target_channel;
    pose.frame_id = presentation.base_frame_id;
    pose.pose.position_m = {
      target->target_pose.translation().x(),
      target->target_pose.translation().y(),
      target->target_pose.translation().z()};
    pose.pose.orientation_xyzw = {
      orientation.x(), orientation.y(), orientation.z(), orientation.w()};
    visualization.poses.push_back(std::move(pose));

    const auto fk = std::find_if(
      frame.forward_kinematics.begin(), frame.forward_kinematics.end(),
      [&arm](const ArmForwardKinematics & value) { return value.side == arm.side; });
    if (fk == frame.forward_kinematics.end()) {
      throw std::runtime_error(
              std::string{"visualization frame is missing the "} +
              armSideName(arm.side) + " FK output");
    }
    const Eigen::Quaterniond fk_orientation(fk->pose.linear());
    mcv::NamedPose fk_pose;
    fk_pose.entity_id = std::string{armSideName(arm.side)} + "_end_effector_fk";
    fk_pose.channel = arm.forward_kinematics_channel;
    fk_pose.frame_id = presentation.base_frame_id;
    fk_pose.pose.position_m = {
      fk->pose.translation().x(),
      fk->pose.translation().y(),
      fk->pose.translation().z()};
    fk_pose.pose.orientation_xyzw = {
      fk_orientation.x(), fk_orientation.y(), fk_orientation.z(), fk_orientation.w()};
    visualization.poses.push_back(std::move(fk_pose));
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
