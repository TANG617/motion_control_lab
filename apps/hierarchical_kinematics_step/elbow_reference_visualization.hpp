#pragma once

#include "elbow_reference.hpp"
#include <motion_control_viz/render_batch.hpp>

namespace motion_control_lab::hierarchical_kinematics_step {

inline constexpr char kRecordedElbowAngleTopic[] = "/mcl/elbow_reference/left/angle";
inline constexpr char kRecordedElbowSceneTopic[] = "/mcl/elbow_reference/left/scene";

// Updated only when Red accepts a result. No allocations or transport in Red.
class ElbowReferenceAngleTracker {
public:
  double update(const ElbowPrediction &prediction, std::size_t side = 0);

private:
  bool initialized_{false};
  std::uint64_t generation_{0}, sequence_{0};
  double wrapped_{0}, unwrapped_{0};
};

struct ElbowArmVisualization {
  double angle_unwrapped_rad{0};
  Eigen::Vector3d shoulder{Eigen::Vector3d::Zero()};
  Eigen::Vector3d wrist{Eigen::Vector3d::Zero()};
  ElbowCircle circle{};
};
struct ElbowReferenceVisualizationSnapshot {
  bool valid{false};
  ElbowConsumption consumed;
  std::array<ElbowArmVisualization,2> arms{};
  double left_scale{0}, right_scale{0};
  double left_tcp_position_error_m{0}, right_tcp_position_error_m{0};
  double left_tcp_orientation_error_rad{0}, right_tcp_orientation_error_rad{0};
};

ElbowReferenceVisualizationSnapshot makeElbowReferenceVisualizationSnapshot(
    const ElbowConsumption &consumed, const std::array<ElbowGeometry,2> &geometry,
    const std::array<Eigen::Vector3d,2> &shoulder, const ArmPoses &ee_reference,
    const Eigen::Isometry3d &reference_from_base,
    std::array<ElbowReferenceAngleTracker,2> &tracker);

// UI/visualization thread only. Retains the app's normal preview/MCAP sink.
void appendElbowReferenceVisualization(motion_control::viz::RenderBatch &batch,
                            const std::string &frame, const std::string &source,
                            const ElbowReferenceVisualizationSnapshot &snapshot);
} // namespace motion_control_lab::hierarchical_kinematics_step
