#pragma once

#include <Eigen/Geometry>

#include <motion_control_viz/render_batch.hpp>

#include <string>

#include "contracts/visualization/foxglove_planned_grouped_step_otg_v1.hpp"

namespace motion_control_lab::planned_grouped_step_otg {

inline motion_control::viz::PoseSample
makePlanningRequestPose(const char *channel,
                        const std::string &reference_frame,
                        const Eigen::Isometry3d &pose) {
  const Eigen::Quaterniond orientation(pose.linear());
  motion_control::viz::PoseSample result;
  result.channel = channel;
  result.frame_id = reference_frame;
  result.pose.position_m = {pose.translation().x(), pose.translation().y(),
                            pose.translation().z()};
  result.pose.orientation_xyzw = {orientation.x(), orientation.y(),
                                  orientation.z(), orientation.w()};
  return result;
}

inline void
appendPlanningRequestPoses(motion_control::viz::RenderBatch &frame,
                           const std::string &reference_frame,
                           const Eigen::Isometry3d &left_pose,
                           const Eigen::Isometry3d &right_pose) {
  namespace contract =
      motion_control_lab::contracts::foxglove_planned_grouped_step_otg_v1;
  frame.poses.reserve(frame.poses.size() + 2U);
  frame.poses.push_back(makePlanningRequestPose(
      contract::kLeftPlanningRequestTopic, reference_frame, left_pose));
  frame.poses.push_back(makePlanningRequestPose(
      contract::kRightPlanningRequestTopic, reference_frame, right_pose));
}

inline void appendOtgExecution(
    motion_control::viz::RenderBatch &batch,
    const std::vector<std::string> &joint_names,
    const std::vector<double> &positions,
    const std::vector<double> &velocities,
    const std::string &reference_frame,
    const Eigen::Isometry3d &left_pose,
    const Eigen::Isometry3d &right_pose) {
  namespace contract =
      motion_control_lab::contracts::foxglove_planned_grouped_step_otg_v1;
  batch.joint_states.push_back(motion_control::viz::JointStateSample{
      contract::kJointOtgExecutionStateTopic, joint_names, positions, velocities});
  batch.poses.push_back(makePlanningRequestPose(
      contract::kLeftOtgFkTopic, reference_frame, left_pose));
  batch.poses.push_back(makePlanningRequestPose(
      contract::kRightOtgFkTopic, reference_frame, right_pose));
}

} // namespace motion_control_lab::planned_grouped_step_otg
