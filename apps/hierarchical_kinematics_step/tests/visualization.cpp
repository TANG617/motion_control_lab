#include <Eigen/Geometry>

#include <cstdlib>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
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

namespace {
void checkCenterOfMass() {
  namespace app = motion_control_lab::hierarchical_kinematics_step;
  namespace mcc = motion_control::core;
  namespace mcv = motion_control::viz;
  const auto require = [](bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
  };
  const auto near = [&](const std::array<double, 3> &actual, const Eigen::Vector3d &expected) {
    require((Eigen::Vector3d(actual.data()) - expected).norm() < 1e-10,
            "CoM scene position differs from analytic value");
  };
  // 2 kg at (1,0,0), 3 kg at (2+qx,qy,1). Wheel supports at (+/-3,+/-2,0).
  // Reference links have no mass; moving the reference must not change the geometry.
  const auto path = std::filesystem::temp_directory_path() /
      ("mcl-com-" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()) + ".urdf");
  struct RemoveFile {
    std::filesystem::path path;
    ~RemoveFile() { std::filesystem::remove(path); }
  } cleanup{path};
  {
    std::ofstream file(path);
    file << R"(<robot name="com">
      <link name="base"><inertial><origin xyz="1 0 0"/><mass value="2"/>
        <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
      <link name="slider_x"/>
      <link name="slider"><inertial><origin xyz="0 0 1"/><mass value="3"/>
        <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
      <joint name="slide_x" type="prismatic"><parent link="base"/><child link="slider_x"/>
        <origin xyz="2 0 0"/><axis xyz="1 0 0"/>
        <limit lower="-2" upper="2" effort="100" velocity="2"/></joint>
      <joint name="slide_y" type="prismatic"><parent link="slider_x"/><child link="slider"/>
        <axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="100" velocity="2"/></joint>
      <link name="translated"/><joint name="translated_fixed" type="fixed">
        <parent link="base"/><child link="translated"/><origin xyz="0.7 -0.4 0.3"/></joint>
      <link name="rotated"/><joint name="rotated_fixed" type="fixed">
        <parent link="base"/><child link="rotated"/>
        <origin xyz="0.7 -0.4 0.3" rpy="0.2 -0.4 0.6"/></joint>
      <link name="moving"/><joint name="reference_angle" type="revolute">
        <parent link="slider"/><child link="moving"/>
        <origin xyz="0.2 0.1 0.3" rpy="0.3 -0.2 0.4"/><axis xyz="0 0 1"/>
        <limit lower="-3" upper="3" effort="10" velocity="2"/></joint>
      <link name="wheel_front_left_link"/><joint name="fl" type="fixed">
        <parent link="base"/><child link="wheel_front_left_link"/><origin xyz="3 2 0.1"/></joint>
      <link name="wheel_front_right_link"/><joint name="fr" type="fixed">
        <parent link="base"/><child link="wheel_front_right_link"/><origin xyz="3 -2 0.1"/></joint>
      <link name="wheel_back_right_link"/><joint name="rr" type="fixed">
        <parent link="base"/><child link="wheel_back_right_link"/><origin xyz="-3 -2 0.1"/></joint>
      <link name="wheel_back_left_link"/><joint name="rl" type="fixed">
        <parent link="base"/><child link="wheel_back_left_link"/><origin xyz="-3 2 0.1"/></joint>
      </robot>)";
  }
  const auto make = [&](const std::string &reference) {
    app::RobotOptions robot;
    robot.base_frame = reference;
    robot.support_visualization.reference_frame = "base";
    robot.joint_names = {"slide_x", "slide_y", "reference_angle"};
    robot.default_positions = {0, 0, 0};
    mcc::RobotModelDescription description;
    description.urdf_path = path.string();
    description.joint_names = robot.joint_names;
    description.kinematics_reference_frame = reference;
    std::shared_ptr<const mcc::RobotModel> model;
    app::requireOk(mcc::RobotModel::load(description, model), "test CoM model");
    return app::makeCenterOfMassVisualization(model, robot);
  };
  auto visualization = make("base");
  require(!visualization->frame_query, "default support reference retained per-frame FK");
  const Eigen::VectorXd executed = Eigen::Vector3d{1, 0, 0};
  mcv::RenderBatch batch;
  batch.timestamp_ns = 13U;
  batch.joint_states.push_back({"/mcl/joints/ik", {"slide_x", "slide_y", "reference_angle"},
                                {-1, 0, 0}, {0, 0, 0}});
  app::appendCenterOfMassScene(batch, visualization.get(), executed);
  require(batch.spheres.size() == 2 && batch.line_strips.size() == 2 && batch.timestamp_ns == 13U,
          "CoM scene must contain four entities sharing one batch");
  const auto &marker = batch.spheres.at(0);
  require(marker.entity_id == "whole_robot_com" && marker.diameter_m == 0.06 &&
          marker.color.red == 1.0 && marker.color.green == 0.55 &&
          marker.color.blue == 0.0 && marker.color.alpha == 0.95, "CoM marker changed");
  require(batch.spheres.at(1).entity_id == "whole_robot_com_projection" &&
          batch.spheres.at(1).diameter_m == 0.04, "projection marker changed");
  require(batch.line_strips.at(0).entity_id == "four_wheel_support" &&
          batch.line_strips.at(0).thickness == 3 && batch.line_strips.at(0).scale_invariant &&
          batch.line_strips.at(0).points_m.size() == 5 &&
          batch.line_strips.at(0).points_m.front() == batch.line_strips.at(0).points_m.back(),
          "support outline is not closed");
  require(batch.line_strips.at(1).entity_id == "whole_robot_com_projection_line" &&
          batch.line_strips.at(1).thickness == 2 && batch.line_strips.at(1).scale_invariant,
          "projection line changed");
  near(marker.center_m, {2.2, 0, 0.6});
  near(batch.spheres.at(1).center_m, {2.2, 0, 0});
  near(batch.line_strips.at(1).points_m.at(0), {2.2, 0, 0.6});
  near(batch.line_strips.at(1).points_m.at(1), {2.2, 0, 0});
  for (const auto &sphere : batch.spheres)
    require(sphere.frame_id == "base" && sphere.channel == "/mcl/dynamics/com/scene",
            "CoM sphere frame or topic differs");
  for (const auto &line : batch.line_strips)
    require(line.frame_id == "base" && line.channel == "/mcl/dynamics/com/scene",
            "CoM line frame or topic differs");
  mcv::RenderBatch held;
  held.timestamp_ns = 14U;
  app::appendCenterOfMassScene(held, visualization.get(), executed);
  require(held.spheres.at(1).center_m == batch.spheres.at(1).center_m &&
          held.spheres.at(1).color.green == batch.spheres.at(1).color.green &&
          held.line_strips.at(0).points_m == batch.line_strips.at(0).points_m,
          "held CoM scene changed");

  const auto rotation = [](double roll, double pitch, double yaw) -> Eigen::Matrix3d {
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  };
  struct Case { double x, y; bool inside; };
  const std::vector<Case> cases{
      {0,0,true}, {2.9,1.9,true}, {3,0,false}, {-3,0,false}, {0,2,false}, {0,-2,false},
      {3,2,false}, {3,-2,false}, {-3,2,false}, {-3,-2,false},
      {3.1,0,false}, {-3.1,0,false}, {0,2.1,false}, {0,-2.1,false},
      {3-0.5e-9,0,false}, {3-2e-9,0,true}};
  const std::array<Eigen::Vector3d,4> corners{
      Eigen::Vector3d{3,2,0}, {3,-2,0}, {-3,-2,0}, {-3,2,0}};
  for (const auto &reference : {"base", "translated", "rotated", "moving"}) {
    auto view = make(reference);
    for (const auto &sample : cases) {
      Eigen::VectorXd q = Eigen::Vector3d{
          (sample.x - 1.6) / 0.6, sample.y / 0.6, 0.7 + 0.1 * sample.y};
      Eigen::Isometry3d base_from_reference = Eigen::Isometry3d::Identity();
      if (std::string(reference) == "translated" || std::string(reference) == "rotated")
        base_from_reference.translation() = Eigen::Vector3d{0.7,-0.4,0.3};
      if (std::string(reference) == "rotated")
        base_from_reference.linear() = rotation(0.2,-0.4,0.6);
      if (std::string(reference) == "moving") {
        base_from_reference.translation() = Eigen::Vector3d{2.2+q[0],q[1]+0.1,0.3};
        base_from_reference.linear() = rotation(0.3,-0.2,0.4) * rotation(0,0,q[2]);
      }
      const Eigen::Isometry3d ref_from_base = base_from_reference.inverse();
      mcv::RenderBatch rendered;
      app::appendCenterOfMassScene(rendered, view.get(), q);
      near(rendered.spheres.at(0).center_m, ref_from_base * Eigen::Vector3d{sample.x,sample.y,0.6});
      near(rendered.spheres.at(1).center_m, ref_from_base * Eigen::Vector3d{sample.x,sample.y,0});
      for (std::size_t i = 0; i < corners.size(); ++i)
        near(rendered.line_strips.at(0).points_m.at(i), ref_from_base * corners[i]);
      const auto &projection = rendered.spheres.at(1);
      require(projection.frame_id == reference &&
              (projection.color.green == 1.0) == sample.inside &&
              (projection.color.red == 1.0) == !sample.inside &&
              rendered.line_strips.at(1).color.green == projection.color.green,
              "projection color or reference differs from expected geometry");
    }
  }
  // Interleaved independent caches must not affect a held execution state.
  mcv::RenderBatch unchanged;
  app::appendCenterOfMassScene(unchanged, visualization.get(), executed);
  require(unchanged.spheres.at(1).center_m == batch.spheres.at(1).center_m,
          "independent visualization caches interfered");
  mcv::RenderBatch disabled;
  app::appendCenterOfMassScene(disabled, nullptr, Eigen::VectorXd{});
  require(disabled.spheres.empty() && disabled.line_strips.empty(), "disabled CoM emitted geometry");
  app::CenterOfMassVisualization invalid;
  bool failed = false;
  try { app::appendCenterOfMassScene(disabled, &invalid, executed); }
  catch (const std::runtime_error &) { failed = true; }
  require(failed && disabled.spheres.empty() && disabled.line_strips.empty(),
          "failed CoM query published partial geometry");
  auto invalid_frame = make("moving");
  invalid_frame->support.reference_frame = "missing_frame";
  failed = false;
  try { app::appendCenterOfMassScene(disabled, invalid_frame.get(), executed); }
  catch (const std::runtime_error &) { failed = true; }
  require(failed && disabled.spheres.empty() && disabled.line_strips.empty(),
          "failed reference FK published partial geometry");
}

} // namespace

int main() {
  checkCenterOfMass();
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
  mcl::hierarchical_kinematics_step::appendPlanningRequestPoses(
      batch, "base_link", mcl::Pose::Identity(), mcl::Pose::Identity());
  mcl::hierarchical_kinematics_step::appendOtgExecution(
      batch, {"j0"}, {3.0}, {4.0}, "base_link", mcl::Pose::Identity(),
      mcl::Pose::Identity());
  mcl::hierarchical_kinematics_step::Link4TargetSnapshot elbow;
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
  mcl::hierarchical_kinematics_step::appendNullspaceElbowScene(
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
