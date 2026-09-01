#include "../solver.hpp"
#include "../planning.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace app = motion_control_lab::hierarchical_kinematics_step;

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

const motion_control::core::FramePose &
requirePose(const std::vector<motion_control::core::FramePose> &poses,
            const std::string &name) {
  return *std::find_if(poses.begin(), poses.end(), [&](const auto &pose) {
    return pose.frame_name == name;
  });
}

std::vector<double> values(const Eigen::VectorXd &vector) {
  return {vector.data(), vector.data() + vector.size()};
}

Eigen::VectorXd eigen(const std::vector<double> &vector) {
  return Eigen::Map<const Eigen::VectorXd>(
      vector.data(), static_cast<Eigen::Index>(vector.size()));
}

} // namespace

int main(int argc, char **argv) {
  try {
    const app::RobotOptions robot;
    const app::RobotOptions options;
    const auto names = app::activeJointNames(robot, options);
    const auto indices = app::activeJointFullIndices(robot, options);
    require(names.size() == 20U && indices.size() == 20U,
            "wrong active joint count");
    for (std::size_t index = 0; index < indices.size(); ++index) {
      require(names[index] ==
                  robot.joint_names[static_cast<std::size_t>(indices[index])],
              "active joint mapping mismatch");
    }
    require(std::find(indices.begin(), indices.end(), 4U) != indices.end() &&
                std::find(indices.begin(), indices.end(), 5U) != indices.end(),
            "default active joint policy unexpectedly excludes waist joints");

    auto custom_options = options;
    custom_options.inactive_joint_names = {"head_yaw_joint"};
    const auto custom_names = app::activeJointNames(robot, custom_options);
    const auto custom_indices =
        app::activeJointFullIndices(robot, custom_options);
    require(custom_names.size() == 19U && custom_indices.size() == 19U,
            "custom inactive joint policy was not applied");
    require(custom_indices.front() == 1U,
            "custom inactive joint mapping mismatch");
    require(std::find(custom_indices.begin(), custom_indices.end(), 4U) !=
                custom_indices.end() &&
                std::find(custom_indices.begin(), custom_indices.end(), 5U) !=
                    custom_indices.end(),
            "custom inactive joint policy retained hidden waist behavior");

    require(argc == 2, "solver topology test requires an R1 URDF path");
    app::Options app_options;
    app_options.interactive.urdf_path = argv[1];
    const auto urdf = std::filesystem::weakly_canonical(argv[1]);
    app_options.interactive.robot.collision_mesh_search_paths = {
        urdf.parent_path().parent_path().parent_path().string()};
    const auto model = app::loadRobotModel(robot, app_options);
    const auto collision_model = app::loadCollisionModel(model, app_options);
    app::SolverRuntime runtime;
    app::SolverHandles handles;
    app::configureSolver(runtime, handles, model, names, collision_model, robot,
                         app_options);

    motion_control::core::RobotState state;
    state.joint_positions = Eigen::Map<const Eigen::VectorXd>(
        robot.default_positions.data(),
        static_cast<Eigen::Index>(robot.default_positions.size()));
    state.joint_velocities.setZero(state.joint_positions.size());
    motion_control::core::ForwardKinematicsRequest fk_request;
    fk_request.state = state;
    fk_request.reference_frame_name = robot.base_frame;
    fk_request.frame_names = {robot.left_end_effector_frame,
                              robot.right_end_effector_frame,
                              robot.left_link4_frame,
                              robot.right_link4_frame};
    motion_control::core::ForwardKinematicsSolution initial_fk;
    motion_control::core::ForwardKinematicsDiagnostics fk_diagnostics;
    app::requireOk(runtime.computeForwardKinematics(
                       fk_request, initial_fk, fk_diagnostics),
                   "initial FK");

    const auto initial_left_tcp =
        requirePose(initial_fk.poses, robot.left_end_effector_frame).pose;
    const auto initial_right_tcp =
        requirePose(initial_fk.poses, robot.right_end_effector_frame).pose;
    const auto initial_left_link4 =
        requirePose(initial_fk.poses, robot.left_link4_frame).pose.translation();
    const auto initial_right_link4 =
        requirePose(initial_fk.poses, robot.right_link4_frame).pose.translation();

    runtime.beginRun(1);
    app::SolverRequest yellow;
    yellow.reference_frame_name = robot.base_frame;
    yellow.captured_state = {state, 1U, 1};
    app::SolverSolution yellow_solution;
    app::SolverDiagnostics yellow_diagnostics;
    app::requireOk(runtime.solveYellow(yellow, yellow_solution,
                                       yellow_diagnostics),
                   "Yellow solve");
    require(yellow_diagnostics.kinematics.posture_errors.size() == 1U,
            "Yellow posture diagnostics missing");
    const auto &yellow_posture_error =
        yellow_diagnostics.kinematics.posture_errors.front();
    require(yellow_posture_error.handle == handles.yellow_posture,
            "Yellow posture diagnostic handle mismatch");
    require(yellow_posture_error.enabled,
            "Yellow posture diagnostic is disabled");
    require(yellow_posture_error.role ==
                motion_control::core::PostureTaskRole::Regularization,
            "Yellow posture diagnostic role mismatch");
    const auto yellow_posture_requirement = std::find_if(
        yellow_diagnostics.kinematics.optimization.requirements.begin(),
        yellow_diagnostics.kinematics.optimization.requirements.end(),
        [](const auto &requirement) {
          return requirement.name == "yellow/task/posture-preference";
        });
    require(yellow_posture_requirement !=
                    yellow_diagnostics.kinematics.optimization.requirements.end() &&
                yellow_posture_requirement->enabled,
            "Yellow posture requirement is not enabled");

    auto displaced_state = state;
    displaced_state.joint_positions(0) += 0.1;
    yellow.captured_state = {displaced_state, 2U, 2};
    app::requireOk(runtime.solveYellow(yellow, yellow_solution,
                                       yellow_diagnostics),
                   "displaced Yellow solve");
    require(yellow_solution.kinematics_solution.joint_velocities(0) < 0.0 &&
                yellow_solution.kinematics_solution.joint_positions(0) <
                    displaced_state.joint_positions(0),
            "Yellow posture preference did not restore the nominal pose");

    yellow.captured_state = {state, 3U, 3};
    app::requireOk(runtime.solveYellow(yellow, yellow_solution,
                                       yellow_diagnostics),
                   "reset Yellow solve");

    app::SolverRequest red;
    red.reference_frame_name = robot.base_frame;
    red.captured_state = {state, 1U, 1};
    red.position_targets = {
        {handles.red.left_position, initial_left_tcp.translation()},
        {handles.red.right_position, initial_right_tcp.translation()},
        {handles.red_left_link4,
         initial_left_link4 + 0.005 * Eigen::Vector3d::UnitX(), true},
        {handles.red_right_link4, initial_right_link4, false}};
    red.orientation_targets = {
        {handles.red.left_orientation, initial_left_tcp.linear()},
        {handles.red.right_orientation, initial_right_tcp.linear()}};
    app::SolverSolution red_solution;
    app::SolverDiagnostics red_diagnostics;
    app::requireOk(runtime.solveRed(red, red_solution, red_diagnostics),
                   "Red solve");

    const auto &passes = red_diagnostics.hierarchy.passes;
    require(passes[0].attempted && passes[0].succeeded,
            "Primary pass did not succeed");
    require(passes[1].attempted && passes[1].succeeded,
            "Secondary pass did not succeed");
    require(passes[2].attempted && passes[2].succeeded,
            "Tertiary pass did not succeed");
    require(!passes[3].attempted, "Terminal pass must remain disabled");
    require(red_diagnostics.hierarchy.highest_completed_priority ==
                motion_control::core::PriorityLevel::Tertiary,
            "highest completed priority is not Tertiary");

    std::size_t enabled_primary = 0U;
    std::size_t enabled_secondary = 0U;
    std::size_t enabled_tertiary = 0U;
    bool left_link4_enabled = false;
    bool right_link4_disabled = false;
    bool yellow_posture_enabled = false;
    for (const auto &task : red_diagnostics.hierarchy.tasks) {
      if (task.enabled &&
          task.priority == motion_control::core::PriorityLevel::Primary) {
        ++enabled_primary;
      }
      if (task.enabled &&
          task.priority == motion_control::core::PriorityLevel::Secondary) {
        ++enabled_secondary;
      }
      if (task.enabled &&
          task.priority == motion_control::core::PriorityLevel::Tertiary) {
        ++enabled_tertiary;
      }
      if (task.handle_value == handles.red_left_link4.value) {
        left_link4_enabled = task.enabled;
      }
      if (task.handle_value == handles.red_right_link4.value) {
        right_link4_disabled = !task.enabled;
      }
      if (task.handle_value == handles.red_yellow_posture.value) {
        yellow_posture_enabled = task.enabled;
      }
    }
    require(enabled_primary == 2U,
            "Primary must contain the two TCP position tasks");
    require(enabled_secondary == 2U,
            "Secondary must contain the two TCP orientation tasks");
    require(enabled_tertiary == 2U,
            "Tertiary must contain one link4 and the Yellow posture target");
    require(left_link4_enabled && right_link4_disabled &&
                yellow_posture_enabled,
            "Tertiary task enablement mismatch");

    for (const bool exercise_left : {true, false}) {
      runtime.beginRun(exercise_left ? 2U : 3U);
      red.position_targets[2] = {
          handles.red_left_link4,
          initial_left_link4 + 0.005 * Eigen::Vector3d::UnitX(), exercise_left};
      red.position_targets[3] = {handles.red_right_link4,
                                 initial_right_link4 +
                                     0.005 * Eigen::Vector3d::UnitX(),
                                 !exercise_left};
      std::vector<double> ik_positions = robot.default_positions;
      std::vector<double> ik_velocities(robot.joint_names.size(), 0.0);
      std::vector<double> otg_positions = robot.default_positions;
      std::vector<double> otg_velocities(robot.joint_names.size(), 0.0);
      std::vector<double> otg_accelerations(robot.joint_names.size(), 0.0);
      app::JointTargetBuilder target_builder(
          app_options.joint_target, 1.0 / app_options.interactive.red_rate_hz,
          robot.joint_names.size());
      motion_control::core::JointPlanner joint_planner(
          app::makeJointPlannerConfig(app_options.planning));
      const auto joint_limits = app::makeJointTargetLimits(
          robot, app_options.interactive.robot.joint_stream);
      double maximum_primary_drift = 0.0;
      double maximum_secondary_drift = 0.0;

      for (std::uint64_t tick = 1; tick <= 400; ++tick) {
        motion_control::core::RobotState yellow_state;
        yellow_state.joint_positions = eigen(otg_positions);
        yellow_state.joint_velocities = eigen(otg_velocities);
        yellow.captured_state = {yellow_state, tick,
                                 static_cast<std::int64_t>(tick)};
        app::requireOk(
            runtime.solveYellow(yellow, yellow_solution, yellow_diagnostics),
            "iterated Yellow solve");

        motion_control::core::RobotState ik_state;
        ik_state.joint_positions = eigen(ik_positions);
        ik_state.joint_velocities = eigen(ik_velocities);
        red.captured_state = {ik_state, tick, static_cast<std::int64_t>(tick)};
        app::requireOk(runtime.solveRed(red, red_solution, red_diagnostics),
                       "iterated Red solve");
        require(red_diagnostics.hierarchy.passes[0].succeeded &&
                    red_diagnostics.hierarchy.passes[1].succeeded &&
                    red_diagnostics.hierarchy.passes[2].succeeded &&
                    !red_diagnostics.hierarchy.passes[3].attempted,
                "iterated solve changed the three-pass hierarchy");
        for (const auto &task : red_diagnostics.hierarchy.tasks) {
          if (task.enabled &&
              task.priority == motion_control::core::PriorityLevel::Primary &&
              task.actual_preservation_drift.size() != 0) {
            maximum_primary_drift =
                std::max(maximum_primary_drift,
                         task.actual_preservation_drift.maxCoeff());
          }
          if (task.enabled &&
              task.priority == motion_control::core::PriorityLevel::Secondary &&
              task.actual_preservation_drift.size() != 0) {
            maximum_secondary_drift =
                std::max(maximum_secondary_drift,
                         task.actual_preservation_drift.maxCoeff());
          }
        }

        const auto raw_target = app::mapActiveIkToFull(
            ik_positions, indices,
            values(red_solution.kinematics_solution.joint_positions),
            values(red_solution.kinematics_solution.joint_velocities));
        const auto target =
            target_builder.preview(raw_target.positions, raw_target.velocities);
        app::ProjectionDiagnostics projection;
        const auto projected =
            app::projectConfiguredLimits(target, joint_limits, projection);

        motion_control::core::JointTrajectoryRequest joint_request;
        joint_request.joint_names = robot.joint_names;
        joint_request.current = {otg_positions, otg_velocities,
                                 otg_accelerations};
        joint_request.target = {projected.positions, projected.velocities,
                                projected.accelerations};
        joint_request.limits = {
            joint_limits.position_lower, joint_limits.position_upper,
            joint_limits.max_velocity, joint_limits.max_acceleration,
            joint_limits.max_jerk};
        joint_request.sample_period = 1.0 / app_options.interactive.red_rate_hz;
        motion_control::core::PlanningDiagnostics plan_diagnostics;
        app::requireOk(joint_planner.plan(joint_request, plan_diagnostics),
                       "iterated JointPlanner plan");
        motion_control::core::JointTrajectorySample sample;
        motion_control::core::PlanningDiagnostics step_diagnostics;
        app::requireOk(joint_planner.step(sample, step_diagnostics),
                       "iterated JointPlanner step");

        target_builder.commit(raw_target.positions, projected);
        ik_positions = raw_target.positions;
        ik_velocities = raw_target.velocities;
        otg_positions = sample.positions;
        otg_velocities = sample.velocities;
        otg_accelerations = sample.accelerations;
      }

      motion_control::core::ForwardKinematicsRequest final_fk_request;
      final_fk_request.state.joint_positions = eigen(otg_positions);
      final_fk_request.state.joint_velocities = eigen(otg_velocities);
      final_fk_request.reference_frame_name = robot.base_frame;
      final_fk_request.frame_names = fk_request.frame_names;
      motion_control::core::ForwardKinematicsSolution final_fk;
      app::requireOk(runtime.computeForwardKinematics(final_fk_request,
                                                      final_fk, fk_diagnostics),
                     "final executed FK");
      const auto final_left_tcp =
          requirePose(final_fk.poses, robot.left_end_effector_frame).pose;
      const auto final_right_tcp =
          requirePose(final_fk.poses, robot.right_end_effector_frame).pose;
      const auto final_link4 =
          requirePose(final_fk.poses, exercise_left ? robot.left_link4_frame
                                                    : robot.right_link4_frame)
              .pose.translation();
      const double initial_link4_error = 0.005;
      const double final_link4_error =
          ((exercise_left ? initial_left_link4 : initial_right_link4) +
           0.005 * Eigen::Vector3d::UnitX() - final_link4)
              .norm();
      const double maximum_tcp_position_error = std::max(
          (initial_left_tcp.translation() - final_left_tcp.translation())
              .norm(),
          (initial_right_tcp.translation() - final_right_tcp.translation())
              .norm());
      const double maximum_tcp_orientation_error =
          std::max(Eigen::AngleAxisd(initial_left_tcp.linear() *
                                     final_left_tcp.linear().transpose())
                       .angle(),
                   Eigen::AngleAxisd(initial_right_tcp.linear() *
                                     final_right_tcp.linear().transpose())
                       .angle());
      require(final_link4_error < initial_link4_error,
              "executed link4 target error did not decrease");
      require((eigen(otg_positions) - state.joint_positions).norm() > 1.0e-4,
              "null-space objective did not change the executed joints");
      require(maximum_tcp_position_error <=
                      app_options.interactive.solver
                          .red_primary_task_tcp_position_preservation_tolerance_mps &&
                  maximum_tcp_orientation_error <=
                      app_options.interactive.solver
                          .red_secondary_task_tcp_orientation_preservation_tolerance_radps,
              "executed TCP drift exceeded its hierarchy preservation tolerance");
      require(
          maximum_primary_drift <=
              app_options.interactive.solver
                  .red_primary_task_tcp_position_preservation_tolerance_mps,
          "a lower pass changed a Primary position residual beyond tolerance");
      require(
          maximum_secondary_drift <=
              app_options.interactive.solver
                  .red_secondary_task_tcp_orientation_preservation_tolerance_radps,
          "Tertiary changed a Secondary orientation residual beyond tolerance");
    }
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    std::cerr << "planned hierarchical Step OTG solver test failed: " << error.what()
              << '\n';
    return EXIT_FAILURE;
  }
}
