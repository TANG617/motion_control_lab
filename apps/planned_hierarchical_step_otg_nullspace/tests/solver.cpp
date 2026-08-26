#include "../solver.hpp"
#include "../planning.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace app = motion_control_lab::planned_hierarchical_step_otg_nullspace;

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
    const auto &robot = motion_control_lab::r1RobotConfig();
    const app::RobotOptions options;
    const auto names = app::activeJointNames(robot, options);
    const auto indices = app::activeJointFullIndices(robot, options);
    require(names.size() == 18U && indices.size() == 18U,
            "wrong active joint count");
    for (std::size_t index = 0; index < indices.size(); ++index) {
      require(names[index] ==
                  robot.joint_names[static_cast<std::size_t>(indices[index])],
              "active joint mapping mismatch");
      require(indices[index] != 4 && indices[index] != 5,
              "waist fixed joints were not excluded");
    }

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
    require(!passes[2].attempted, "Tertiary pass must remain disabled");
    require(!passes[3].attempted, "Terminal pass must remain disabled");
    require(red_diagnostics.hierarchy.highest_completed_priority ==
                motion_control::core::PriorityLevel::Secondary,
            "highest completed priority is not Secondary");

    std::size_t enabled_primary = 0U;
    std::size_t enabled_secondary = 0U;
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
    require(enabled_primary == 4U,
            "Primary must contain two TCP position and orientation pairs");
    require(enabled_secondary == 2U,
            "Secondary must contain one link4 and the Yellow posture target");
    require(left_link4_enabled && right_link4_disabled &&
                yellow_posture_enabled,
            "Secondary task enablement mismatch");

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
          *model, robot, app_options.interactive.robot.joint_stream);
      double maximum_primary_drift = 0.0;

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
                    !red_diagnostics.hierarchy.passes[2].attempted &&
                    !red_diagnostics.hierarchy.passes[3].attempted,
                "iterated solve changed the two-pass hierarchy");
        for (const auto &task : red_diagnostics.hierarchy.tasks) {
          if (task.enabled &&
              task.priority == motion_control::core::PriorityLevel::Primary &&
              task.actual_preservation_drift.size() != 0) {
            maximum_primary_drift =
                std::max(maximum_primary_drift,
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
                          .cartesian_preservation_tolerance &&
                  maximum_tcp_orientation_error <=
                      app_options.interactive.solver
                          .cartesian_preservation_tolerance,
              "executed TCP drift exceeded the Primary preservation tolerance");
      require(
          maximum_primary_drift <=
              app_options.interactive.solver.cartesian_preservation_tolerance,
          "Secondary changed a Primary residual beyond tolerance");
    }
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    std::cerr << "planned hierarchical Step OTG solver test failed: " << error.what()
              << '\n';
    return EXIT_FAILURE;
  }
}
