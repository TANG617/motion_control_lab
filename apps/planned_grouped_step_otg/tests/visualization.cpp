#include <Eigen/Geometry>

#include <cstdlib>
#include <string>
#include <vector>

#include "../planning_request_visualization.hpp"
#include "contracts/visualization/foxglove_ik_v1.hpp"
#include "contracts/visualization/foxglove_planned_grouped_step_otg_v1.hpp"
#include "runtime/interactive_types.hpp"
#include "sinks/ik_render_batch.hpp"
#include "tests/visualization_contract_conformance.hpp"

int main() {
  namespace mcl = motion_control_lab;
  namespace contract = mcl::contracts::foxglove_planned_grouped_step_otg_v1;

  mcl::InteractiveIkPresentation presentation;
  presentation.base_frame_id = "base_link";
  presentation.joint_state_channel =
      mcl::contracts::foxglove_ik_v1::kIkOutputJointStateTopic;
  presentation.arms = {
      {mcl::ArmSide::Left, mcl::contracts::foxglove_ik_v1::kLeftInputTargetTopic,
       mcl::contracts::foxglove_ik_v1::kLeftFkOutputTopic, {}},
      {mcl::ArmSide::Right, mcl::contracts::foxglove_ik_v1::kRightInputTargetTopic,
       mcl::contracts::foxglove_ik_v1::kRightFkOutputTopic, {}}};
  mcl::IkDebugFrame raw;
  raw.joint_names = {"j0"};
  raw.positions = {1.0};
  raw.velocities = {2.0};
  raw.targets = {{mcl::ArmSide::Left, mcl::Pose::Identity()},
                 {mcl::ArmSide::Right, mcl::Pose::Identity()}};
  raw.forward_kinematics = {{mcl::ArmSide::Left, mcl::Pose::Identity()},
                            {mcl::ArmSide::Right, mcl::Pose::Identity()}};
  auto batch = mcl::makeIkRenderBatch(raw, presentation, 13U);
  mcl::planned_grouped_step_otg::appendPlanningRequestPoses(
      batch, "base_link", mcl::Pose::Identity(), mcl::Pose::Identity());
  mcl::planned_grouped_step_otg::appendOtgExecution(
      batch,
      {"j0"}, {3.0}, {4.0}, "base_link", mcl::Pose::Identity(),
      mcl::Pose::Identity());

  return batch.timestamp_ns == 13U && batch.joint_states.size() == 2U &&
             batch.joint_states.at(0).channel ==
                 mcl::contracts::foxglove_ik_v1::kIkOutputJointStateTopic &&
             batch.poses.at(4).channel == contract::kLeftPlanningRequestTopic &&
             batch.joint_states.at(1).channel == contract::kJointOtgExecutionStateTopic &&
             batch.poses.at(6).channel == contract::kLeftOtgFkTopic &&
             batch.poses.at(7).channel == contract::kRightOtgFkTopic &&
             mcl::tests::requiredChannelsPresent(
               batch, mcl::contracts::foxglove_ik_v1::kChannels) &&
             mcl::tests::requiredChannelsPresent(batch, contract::kChannels)
         ? EXIT_SUCCESS
         : EXIT_FAILURE;
}
