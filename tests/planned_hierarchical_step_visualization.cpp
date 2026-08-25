#include <Eigen/Geometry>

#include <cstdlib>

#include "apps/planned_hierarchical_step/loop.hpp"
#include "contracts/visualization/mcl_planning_v1.hpp"
#include "contracts/visualization/mcl_state_v1.hpp"
#include "contracts/presentation/ik_app_snapshot.hpp"
#include "components/visualization/preview_projection.hpp"
#include "tests/visualization_contract_conformance.hpp"

int main() {
  namespace mcl = motion_control_lab;
  namespace contract = mcl::contracts::mcl_planning_v1;

  mcl::InteractiveIkPresentation presentation;
  presentation.base_frame_id = "base_link";
  presentation.joint_state_channel =
      mcl::contracts::mcl_state_v1::kJointIkTopic;
  presentation.arms = {{mcl::ArmSide::Left,
                        mcl::contracts::mcl_state_v1::kLeftCartesianInputTopic,
                        mcl::contracts::mcl_state_v1::kLeftCartesianGoalTopic,
                        mcl::contracts::mcl_state_v1::kLeftCartesianIkTopic,
                        {}},
                       {mcl::ArmSide::Right,
                        mcl::contracts::mcl_state_v1::kRightCartesianInputTopic,
                        mcl::contracts::mcl_state_v1::kRightCartesianGoalTopic,
                        mcl::contracts::mcl_state_v1::kRightCartesianIkTopic,
                        {}}};

  mcl::IkDebugFrame debug_frame;
  debug_frame.targets = {{mcl::ArmSide::Left, mcl::Pose::Identity()},
                         {mcl::ArmSide::Right, mcl::Pose::Identity()}};
  debug_frame.input_targets = debug_frame.targets;
  debug_frame.forward_kinematics = {
      {mcl::ArmSide::Left, mcl::Pose::Identity()},
      {mcl::ArmSide::Right, mcl::Pose::Identity()}};

  auto visualization =
      mcl::makeIkRenderBatch(debug_frame, presentation, 3U);
  if (visualization.poses.size() != 6U) {
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

  mcl::planned_hierarchical_step::appendPlanningRequestPoses(
      visualization, "base_link", left_request, right_request);

  if (visualization.poses.size() != 8U) {
    return EXIT_FAILURE;
  }
  const auto &left = visualization.poses.at(6);
  const auto &right = visualization.poses.at(7);
  const Eigen::Quaterniond left_orientation(left_request.linear());
  const Eigen::Quaterniond right_orientation(right_request.linear());
  if (left.channel != contract::kLeftCartesianReferenceTopic ||
      left.frame_id != "base_link" || left.pose.position_m.at(0) != 1.0 ||
      left.pose.position_m.at(1) != 2.0 || left.pose.position_m.at(2) != 3.0 ||
      left.pose.orientation_xyzw.at(0) != left_orientation.x() ||
      left.pose.orientation_xyzw.at(1) != left_orientation.y() ||
      left.pose.orientation_xyzw.at(2) != left_orientation.z() ||
      left.pose.orientation_xyzw.at(3) != left_orientation.w() ||
      right.channel != contract::kRightCartesianReferenceTopic ||
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
