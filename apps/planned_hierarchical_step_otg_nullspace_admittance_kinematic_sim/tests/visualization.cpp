#include <Eigen/Geometry>

#include <cstdlib>
#include <string>
#include <vector>

#include "../loop.hpp"
#include "../nullspace.hpp"
#include "components/visualization/preview_projection.hpp"
#include "contracts/presentation/ik_app_snapshot.hpp"
#include "contracts/visualization/mcl_execution_v1.hpp"
#include "contracts/visualization/mcl_nullspace_v1.hpp"
#include "contracts/visualization/mcl_planning_v1.hpp"
#include "contracts/visualization/mcl_state_v1.hpp"
#include "tests/visualization_contract_conformance.hpp"

int main() {
  namespace mcl = motion_control_lab;
  namespace execution_contract = mcl::contracts::mcl_execution_v1;
  namespace planning_contract = mcl::contracts::mcl_planning_v1;

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
  mcl::IkDebugFrame raw;
  raw.joint_names = {"j0"};
  raw.positions = {1.0};
  raw.velocities = {2.0};
  raw.targets = {{mcl::ArmSide::Left, mcl::Pose::Identity()},
                 {mcl::ArmSide::Right, mcl::Pose::Identity()}};
  raw.input_targets = raw.targets;
  raw.forward_kinematics = {{mcl::ArmSide::Left, mcl::Pose::Identity()},
                            {mcl::ArmSide::Right, mcl::Pose::Identity()}};
  auto batch = mcl::makeIkRenderBatch(raw, presentation, 13U);
  mcl::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim::appendPlanningRequestPoses(
      batch, "base_link", mcl::Pose::Identity(), mcl::Pose::Identity());
  mcl::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim::appendOtgExecution(
      batch, {"j0"}, {3.0}, {4.0}, "base_link", mcl::Pose::Identity(),
      mcl::Pose::Identity());
  mcl::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim::Link4TargetSnapshot elbow;
  elbow.left = Eigen::Vector3d{0.1, 0.2, 0.3};
  elbow.right = Eigen::Vector3d{-0.1, -0.2, 0.4};
  elbow.left_enabled = true;
  auto raw_left = mcl::Pose::Identity();
  raw_left.translation() = Eigen::Vector3d{0.11, 0.2, 0.3};
  auto raw_right = mcl::Pose::Identity();
  raw_right.translation() = Eigen::Vector3d{-0.11, -0.2, 0.4};
  auto executed_left = mcl::Pose::Identity();
  executed_left.translation() = Eigen::Vector3d{0.12, 0.2, 0.3};
  auto executed_right = mcl::Pose::Identity();
  executed_right.translation() = Eigen::Vector3d{-0.12, -0.2, 0.4};
  mcl::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim::appendNullspaceElbowScene(
      batch, "base_link", elbow, raw_left, raw_right, executed_left,
      executed_right);

  const auto &left_target = batch.spheres.at(0);
  const auto &right_target = batch.spheres.at(3);

  return batch.timestamp_ns == 13U && batch.joint_states.size() == 2U &&
                 batch.spheres.size() == 6U && batch.line_strips.size() == 2U &&
                 left_target.entity_id == "left_link4_target" &&
                 left_target.channel ==
                     mcl::contracts::mcl_nullspace_v1::kElbowSceneTopic &&
                 left_target.center_m == std::array<double, 3>{0.1, 0.2, 0.3} &&
                 left_target.diameter_m == 0.04 &&
                 left_target.color.alpha == 0.95 &&
                 right_target.color.alpha == 0.0 &&
                 batch.line_strips.at(0).points_m.at(0) ==
                     std::array<double, 3>{0.1, 0.2, 0.3} &&
                 batch.joint_states.at(0).channel ==
                     mcl::contracts::mcl_state_v1::kJointIkTopic &&
                 batch.poses.at(6).channel ==
                     planning_contract::kLeftCartesianReferenceTopic &&
                 batch.joint_states.at(1).channel ==
                     execution_contract::kJointExecutionTopic &&
                 batch.poses.at(8).channel ==
                     execution_contract::kLeftCartesianExecutionTopic &&
                 batch.poses.at(9).channel ==
                     execution_contract::kRightCartesianExecutionTopic &&
                 mcl::tests::requiredChannelsPresent(
                     batch, mcl::contracts::mcl_state_v1::kChannels) &&
                 mcl::tests::requiredChannelsPresent(
                     batch, planning_contract::kChannels) &&
                 mcl::tests::requiredChannelsPresent(
                     batch, execution_contract::kChannels) &&
                 mcl::tests::requiredChannelsPresent(
                     batch, mcl::contracts::mcl_nullspace_v1::kChannels)
             ? EXIT_SUCCESS
             : EXIT_FAILURE;
}
