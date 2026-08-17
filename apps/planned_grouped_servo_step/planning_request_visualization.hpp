#pragma once

#include <Eigen/Geometry>

#include <motion_control_viz/frame.hpp>

#include <string>

#include "contracts/visualization/foxglove_planned_grouped_servo_step_v1.hpp"

namespace motion_control_lab::planned_grouped_servo_step {

inline motion_control::viz::NamedPose
makePlanningRequestPose(const char *entity_id, const char *channel,
                        const std::string &reference_frame,
                        const Eigen::Isometry3d &pose) {
  const Eigen::Quaterniond orientation(pose.linear());
  motion_control::viz::NamedPose result;
  result.entity_id = entity_id;
  result.channel = channel;
  result.frame_id = reference_frame;
  result.pose.position_m = {pose.translation().x(), pose.translation().y(),
                            pose.translation().z()};
  result.pose.orientation_xyzw = {orientation.x(), orientation.y(),
                                  orientation.z(), orientation.w()};
  return result;
}

inline void
appendPlanningRequestPoses(motion_control::viz::VisualizationFrame &frame,
                           const std::string &reference_frame,
                           const Eigen::Isometry3d &left_pose,
                           const Eigen::Isometry3d &right_pose) {
  namespace contract =
      motion_control_lab::contracts::foxglove_planned_grouped_servo_step_v1;
  frame.poses.reserve(frame.poses.size() + 2U);
  frame.poses.push_back(makePlanningRequestPose(
      contract::kLeftPlanningRequestEntity, contract::kLeftPlanningRequestPose,
      reference_frame, left_pose));
  frame.poses.push_back(makePlanningRequestPose(
      contract::kRightPlanningRequestEntity,
      contract::kRightPlanningRequestPose, reference_frame, right_pose));
}

} // namespace motion_control_lab::planned_grouped_servo_step
