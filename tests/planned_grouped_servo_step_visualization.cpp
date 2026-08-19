#include <Eigen/Geometry>

#include <cstdlib>

#include "apps/planned_grouped_servo_step/planning_request_visualization.hpp"
#include "contracts/visualization/foxglove_ik_v1.hpp"
#include "contracts/visualization/foxglove_planned_grouped_servo_step_v1.hpp"
#include "runtime/interactive_types.hpp"
#include "sinks/ik_render_batch.hpp"
#include "tests/visualization_contract_conformance.hpp"

int main() {
  namespace mcl = motion_control_lab;
  namespace contract = mcl::contracts::foxglove_planned_grouped_servo_step_v1;

  mcl::InteractiveIkPresentation presentation;
  presentation.base_frame_id = "base_link";
  presentation.joint_state_channel =
      mcl::contracts::foxglove_ik_v1::kIkOutputJointStateTopic;
  presentation.arms = {{mcl::ArmSide::Left,
                        mcl::contracts::foxglove_ik_v1::kLeftInputTargetTopic,
                        mcl::contracts::foxglove_ik_v1::kLeftFkOutputTopic,
                        {}},
                       {mcl::ArmSide::Right,
                        mcl::contracts::foxglove_ik_v1::kRightInputTargetTopic,
                        mcl::contracts::foxglove_ik_v1::kRightFkOutputTopic,
                        {}}};

  mcl::IkDebugFrame debug_frame;
  debug_frame.targets = {{mcl::ArmSide::Left, mcl::Pose::Identity()},
                         {mcl::ArmSide::Right, mcl::Pose::Identity()}};
  debug_frame.forward_kinematics = {
      {mcl::ArmSide::Left, mcl::Pose::Identity()},
      {mcl::ArmSide::Right, mcl::Pose::Identity()}};

  auto visualization =
      mcl::makeIkRenderBatch(debug_frame, presentation, 3U);
  if (visualization.poses.size() != 4U) {
    return EXIT_FAILURE;
  }

  Eigen::Isometry3d left_request = Eigen::Isometry3d::Identity();
  left_request.translation() = Eigen::Vector3d{1.0, 2.0, 3.0};
  left_request.linear() =
      Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Isometry3d right_request = Eigen::Isometry3d::Identity();
  right_request.translation() = Eigen::Vector3d{4.0, 5.0, 6.0};
  right_request.linear() =
      Eigen::AngleAxisd(-0.5, Eigen::Vector3d::UnitY()).toRotationMatrix();

  mcl::planned_grouped_servo_step::appendPlanningRequestPoses(
      visualization, "base_link", left_request, right_request);

  if (visualization.poses.size() != 6U) {
    return EXIT_FAILURE;
  }
  const auto &left = visualization.poses.at(4);
  const auto &right = visualization.poses.at(5);
  const Eigen::Quaterniond left_orientation(left_request.linear());
  const Eigen::Quaterniond right_orientation(right_request.linear());
  if (left.channel != contract::kLeftPlanningRequestTopic ||
      left.frame_id != "base_link" || left.pose.position_m.at(0) != 1.0 ||
      left.pose.position_m.at(1) != 2.0 || left.pose.position_m.at(2) != 3.0 ||
      left.pose.orientation_xyzw.at(0) != left_orientation.x() ||
      left.pose.orientation_xyzw.at(1) != left_orientation.y() ||
      left.pose.orientation_xyzw.at(2) != left_orientation.z() ||
      left.pose.orientation_xyzw.at(3) != left_orientation.w() ||
      right.channel != contract::kRightPlanningRequestTopic ||
      right.frame_id != "base_link" || right.pose.position_m.at(0) != 4.0 ||
      right.pose.position_m.at(1) != 5.0 ||
      right.pose.position_m.at(2) != 6.0 ||
      right.pose.orientation_xyzw.at(0) != right_orientation.x() ||
      right.pose.orientation_xyzw.at(1) != right_orientation.y() ||
      right.pose.orientation_xyzw.at(2) != right_orientation.z() ||
      right.pose.orientation_xyzw.at(3) != right_orientation.w() ||
      !mcl::tests::requiredChannelsPresent(visualization, contract::kChannels)) {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
