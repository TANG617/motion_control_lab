#pragma once

#include "elbow_reference.hpp"
#include <motion_control_viz/render_batch.hpp>

namespace motion_control_lab::hierarchical_kinematics_step {

inline constexpr char kRecordedElbowAngleTopic[] = "/mcl/elbow_reference/left/angle";
inline constexpr char kRecordedElbowSceneTopic[] = "/mcl/elbow_reference/left/scene";

// Updated only when Red accepts a result. No allocations or transport in Red.
class ElbowReferenceAngleTracker {
public:
  double update(const ElbowPrediction &prediction);

private:
  bool initialized_{false};
  std::uint64_t generation_{0}, sequence_{0};
  double wrapped_{0}, unwrapped_{0};
};

struct ElbowReferenceVisualizationSnapshot {
  bool valid{false};
  ElbowConsumption consumed;
  double angle_unwrapped_rad{0};
  Eigen::Vector3d shoulder{Eigen::Vector3d::Zero()};
  Eigen::Vector3d wrist{Eigen::Vector3d::Zero()};
  ElbowCircle circle{};
  double left_scale{0}, right_scale{0};
  double left_tcp_position_error_m{0}, right_tcp_position_error_m{0};
  double left_tcp_orientation_error_rad{0}, right_tcp_orientation_error_rad{0};
};

ElbowReferenceVisualizationSnapshot makeElbowReferenceVisualizationSnapshot(
    const ElbowConsumption &consumed, const ElbowGeometry &geometry,
    const Eigen::Vector3d &shoulder, const Eigen::Isometry3d &ee_reference,
    ElbowReferenceAngleTracker &tracker);

// UI/visualization thread only. Retains the app's normal preview/MCAP sink.
void appendElbowReferenceVisualization(motion_control::viz::RenderBatch &batch,
                            const std::string &frame, const std::string &source,
                            const ElbowReferenceVisualizationSnapshot &snapshot);
} // namespace motion_control_lab::hierarchical_kinematics_step
