#include "../solver.hpp"
#include "../elbow_reference.hpp"
#include "../planning.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
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

    const auto hierarchical_defaults =
        app::profileDefaults(app::Profile::Hierarchical);
    const auto nullspace_defaults =
        app::profileDefaults(app::Profile::PlannedOtgNullspace);
    const auto hierarchical_red =
        app::makeRedSolverConfig(hierarchical_defaults);
    const auto hierarchical_yellow =
        app::makeYellowSolverConfig(hierarchical_defaults);
    const auto nullspace_red = app::makeRedSolverConfig(nullspace_defaults);
    require(hierarchical_red.qp.backend ==
                    motion_control::core::QpBackend::ProxQp &&
                hierarchical_red.qp.regularization == 1.0e-8 &&
                hierarchical_red.qp.proxqp.maximum_iterations == 1000 &&
                std::holds_alternative<motion_control::core::ServoStepOptions>(
                    hierarchical_yellow.execution) &&
                std::get<motion_control::core::ServoStepOptions>(
                    hierarchical_yellow.execution).servo_period ==
                    1.0 / hierarchical_defaults.interactive.yellow_rate_hz &&
                hierarchical_yellow.qp.regularization == 1.0e-4 &&
                hierarchical_yellow.qp.proxqp.maximum_iterations == 1000 &&
                nullspace_red.qp.proxqp.maximum_iterations == 200,
            "profile QP defaults were not applied to solver configs");

    auto tuned = app::profileDefaults(app::Profile::PlannedOtg);
    tuned.interactive.solver.red_qp_regularization = 3.0e-8;
    tuned.interactive.solver.red_proxqp_maximum_iterations = 321;
    tuned.interactive.solver.red_proxqp_absolute_tolerance = 4.0e-6;
    tuned.interactive.solver.red_proxqp_relative_tolerance = 5.0e-5;
    tuned.interactive.solver.red_proxqp_primal_infeasibility_tolerance =
        6.0e-12;
    tuned.interactive.solver.red_proxqp_warm_start_enabled = true;
    tuned.interactive.solver.yellow_qp_regularization = 7.0e-8;
    tuned.interactive.solver.yellow_proxqp_maximum_iterations = 654;
    tuned.interactive.solver.yellow_proxqp_absolute_tolerance = 8.0e-7;
    tuned.interactive.solver.yellow_proxqp_relative_tolerance = 9.0e-6;
    tuned.interactive.solver.yellow_proxqp_primal_infeasibility_tolerance =
        1.0e-9;
    tuned.interactive.solver.yellow_proxqp_warm_start_enabled = false;
    const auto tuned_red = app::makeRedSolverConfig(tuned);
    const auto tuned_yellow = app::makeYellowSolverConfig(tuned);
    require(tuned_red.qp.regularization == 3.0e-8 &&
                tuned_red.qp.proxqp.maximum_iterations == 321 &&
                tuned_red.qp.proxqp.absolute_tolerance == 4.0e-6 &&
                tuned_red.qp.proxqp.relative_tolerance == 5.0e-5 &&
                tuned_red.qp.proxqp.primal_infeasibility_tolerance ==
                    std::optional<double>{6.0e-12} &&
                tuned_red.qp.proxqp.warm_start_enabled &&
                tuned_yellow.qp.regularization == 7.0e-8 &&
                tuned_yellow.qp.proxqp.maximum_iterations == 654 &&
                tuned_yellow.qp.proxqp.absolute_tolerance == 8.0e-7 &&
                tuned_yellow.qp.proxqp.relative_tolerance == 9.0e-6 &&
                tuned_yellow.qp.proxqp.primal_infeasibility_tolerance ==
                    std::optional<double>{1.0e-9} &&
                !tuned_yellow.qp.proxqp.warm_start_enabled,
            "explicit QP options were not applied to solver configs");

    require(argc == 2, "solver topology test requires an R1 URDF path");
    app::Options app_options;
    app_options.interactive.urdf_path = argv[1];
    const auto urdf = std::filesystem::weakly_canonical(argv[1]);
    app_options.interactive.robot.collision_mesh_search_paths = {
        urdf.parent_path().string()};
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
                              robot.left_link4_frame, robot.right_link4_frame,
                              robot.left_shoulder_frame, robot.left_wrist_frame};
    motion_control::core::ForwardKinematicsSolution initial_fk;
    motion_control::core::ForwardKinematicsDiagnostics fk_diagnostics;
    app::requireOk(runtime.computeForwardKinematics(fk_request, initial_fk,
                                                    fk_diagnostics),
                   "initial FK");

    const auto initial_left_tcp =
        requirePose(initial_fk.poses, robot.left_end_effector_frame).pose;
    const auto initial_right_tcp =
        requirePose(initial_fk.poses, robot.right_end_effector_frame).pose;
    const auto initial_left_link4 =
        requirePose(initial_fk.poses, robot.left_link4_frame)
            .pose.translation();
    const auto initial_right_link4 =
        requirePose(initial_fk.poses, robot.right_link4_frame)
            .pose.translation();
    const auto arm_geometry = app::ElbowGeometry::fromFk(
        requirePose(initial_fk.poses, robot.left_shoulder_frame).pose,
        requirePose(initial_fk.poses, robot.left_link4_frame).pose,
        requirePose(initial_fk.poses, robot.left_wrist_frame).pose, initial_left_tcp);
    const auto swivel=arm_geometry.angle(
        requirePose(initial_fk.poses,robot.left_shoulder_frame).pose.translation(),
        requirePose(initial_fk.poses,robot.left_wrist_frame).pose.translation(),initial_left_link4);
    require((arm_geometry.target(requirePose(initial_fk.poses,robot.left_shoulder_frame).pose.translation(),
        initial_left_tcp,swivel.x(),swivel.y())-initial_left_link4).norm()<1e-9,
        "R1 public-FK arm angle roundtrip");


    runtime.beginRun(1);
    app::SolverRequest yellow;
    yellow.reference_frame_name = robot.base_frame;
    yellow.captured_state = {state, 1U, 1};
    app::SolverSolution yellow_solution;
    app::SolverDiagnostics yellow_diagnostics;
    app::requireOk(
        runtime.solveYellow(yellow, yellow_solution, yellow_diagnostics),
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
    require(yellow_posture_error.tolerance_rad == 0.0,
            "Yellow posture must not introduce a convergence criterion");
    const auto yellow_posture_requirement = std::find_if(
        yellow_diagnostics.kinematics.optimization.requirements.begin(),
        yellow_diagnostics.kinematics.optimization.requirements.end(),
        [](const auto &requirement) {
          return requirement.name == "yellow/task/posture-preference";
        });
    require(
        yellow_posture_requirement !=
                    yellow_diagnostics.kinematics.optimization.requirements.end() &&
                yellow_posture_requirement->enabled,
            "Yellow posture requirement is not enabled");

    auto displaced_state = state;
    displaced_state.joint_positions(0) += 0.1;
    yellow.captured_state = {displaced_state, 2U, 2};
    app::requireOk(
        runtime.solveYellow(yellow, yellow_solution, yellow_diagnostics),
                   "displaced Yellow solve");
    require(yellow_solution.kinematics_solution.joint_velocities(0) < 0.0 &&
                yellow_solution.kinematics_solution.joint_positions(0) <
                    displaced_state.joint_positions(0),
            "Yellow posture preference did not restore the nominal pose");

    yellow.captured_state = {state, 3U, 3};
    app::requireOk(
        runtime.solveYellow(yellow, yellow_solution, yellow_diagnostics),
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
    require(!passes[2].attempted,
            "Tertiary pass must remain inactive in the two-level topology");
    require(!passes[3].attempted, "Terminal pass must remain disabled");
    require(red_diagnostics.hierarchy.highest_completed_priority ==
                motion_control::core::PriorityLevel::Secondary,
            "highest completed priority is not Secondary");
    require(red_diagnostics.hierarchy.selected_priority ==
                motion_control::core::PriorityLevel::Secondary,
            "selected priority is not Secondary");
    require(red_diagnostics.hierarchy.solution_quality ==
                motion_control::core::HierarchicalSolutionQuality::Converged,
            "normal two-pass solve is not marked converged");

    std::size_t enabled_primary = 0U;
    std::size_t enabled_primary_scaled = 0U;
    std::size_t enabled_secondary = 0U;
    std::size_t soft_secondary_orientation = 0U;
    std::size_t enabled_tertiary = 0U;
    bool left_link4_enabled = false;
    bool right_link4_disabled = false;
    bool yellow_posture_enabled = false;
    for (const auto &task : red_diagnostics.hierarchy.tasks) {
      if (task.enabled &&
          task.priority == motion_control::core::PriorityLevel::Primary) {
        ++enabled_primary;
        if (task.enforcement ==
            motion_control::core::HierarchicalTaskEnforcement::Scaled) {
          ++enabled_primary_scaled;
        }
      }
      if (task.enabled &&
          task.priority == motion_control::core::PriorityLevel::Secondary) {
        ++enabled_secondary;
        if (task.kind ==
                motion_control::core::HierarchicalTaskKind::Orientation &&
            task.enforcement ==
                motion_control::core::HierarchicalTaskEnforcement::Soft) {
          ++soft_secondary_orientation;
        }
      }
      if (task.enabled &&
          task.priority == motion_control::core::PriorityLevel::Tertiary) {
        ++enabled_tertiary;
      }
      if (task.kind == motion_control::core::HierarchicalTaskKind::Position &&
          task.handle_value == handles.red_left_link4.value) {
        left_link4_enabled = task.enabled;
      }
      if (task.kind == motion_control::core::HierarchicalTaskKind::Position &&
          task.handle_value == handles.red_right_link4.value) {
        right_link4_disabled = !task.enabled;
      }
      if (task.kind == motion_control::core::HierarchicalTaskKind::Posture &&
          task.handle_value == handles.red_yellow_posture.value) {
        yellow_posture_enabled = task.enabled;
      }
    }
    require(enabled_primary == 2U,
            "Primary must contain only the two TCP position tasks");
    require(enabled_primary_scaled == 2U,
            "both Primary position tasks must use scaled enforcement");
    require(enabled_secondary == 4U,
            "Secondary must contain soft TCP orientation, one link4 and Yellow "
            "posture");
    require(soft_secondary_orientation == 2U,
            "both orientation tasks must be soft and owned by Secondary");
    require(enabled_tertiary == 0U,
            "Tertiary must not contain an enabled task");
    require(left_link4_enabled && right_link4_disabled &&
                yellow_posture_enabled,
            "Secondary task enablement mismatch");
    const auto &scales = red_diagnostics.hierarchy.task_scales;
    require(
        scales.size() == 2U && scales[0].active && scales[1].active &&
            scales[0].priority ==
                motion_control::core::PriorityLevel::Primary &&
            scales[1].priority ==
                motion_control::core::PriorityLevel::Primary &&
            scales[0].name == "red-primary/task/left-tcp-position-progress" &&
            scales[1].name == "red-primary/task/right-tcp-position-progress",
        "Primary must expose exactly one active position scale per arm");

    auto conflicting = red;
    for (auto &target : conflicting.orientation_targets) {
      target.feed_forward_angular_velocity = 100.0 * Eigen::Vector3d::UnitX();
    }
    app::requireOk(runtime.solveRed(conflicting, red_solution, red_diagnostics),
                   "infeasible angular velocity must remain a soft objective");
    require(red_diagnostics.hierarchy.passes[0].succeeded &&
                red_diagnostics.hierarchy.passes[1].succeeded &&
                !red_diagnostics.hierarchy.passes[2].attempted &&
                !red_diagnostics.hierarchy.passes[3].attempted,
            "conflicting orientation must complete exactly two passes");
    bool orientation_left_residual = false;
    for (const auto &task : red_diagnostics.hierarchy.tasks) {
      if (task.kind ==
          motion_control::core::HierarchicalTaskKind::Orientation) {
        orientation_left_residual |= task.residual_norm > 1.0;
      }
      if (task.enabled &&
          task.priority == motion_control::core::PriorityLevel::Primary) {
        require(
            task.actual_preservation_drift.maxCoeff() <=
                app_options.interactive.solver
                    .red_primary_task_tcp_position_preservation_tolerance_mps,
            "soft orientation changed the Primary position optimum");
      }
    }
    require(orientation_left_residual,
            "unachievable orientation must leave a residual");
    require(red_diagnostics.maximum_hard_violation <=
                app_options.interactive.solver.maximum_accepted_hard_violation,
            "soft orientation must not relax shared joint bounds");

    // The app must submit a complete request even before Yellow has a value.
    runtime.beginRun(2);
    app::requireOk(runtime.solveRed(red, red_solution, red_diagnostics),
                   "Red without a Yellow value");
    require(red_diagnostics.coupling_state == app::CouplingState::WaitingForValue,
            "Red must keep coupling disabled before the first Yellow value");

    auto invalid_yellow = yellow;
    invalid_yellow.position_targets.emplace_back(
        motion_control::core::PositionTaskHandle{999}, Eigen::Vector3d::Zero());
    require(runtime.solveYellow(invalid_yellow, yellow_solution, yellow_diagnostics).code ==
                motion_control::core::StatusCode::InvalidTarget,
            "Yellow must reject an unknown target");
    app::requireOk(runtime.solveYellow(yellow, yellow_solution, yellow_diagnostics),
                   "Yellow recovery with the fixed posture request");
    require(yellow_diagnostics.kinematics.posture_errors.size() == 1U &&
                yellow_diagnostics.kinematics.posture_errors.front().enabled,
            "Yellow recovery lost the posture target");

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
      motion_control::core::JointPtpTrajectoryGenerator joint_generator(
          app::makeJointPtpTrajectoryConfig(app_options.planning));
      const auto joint_limits = app::makeJointTargetLimits(
          robot, app_options.interactive.robot.joint_stream);
      double maximum_primary_position_drift = 0.0;

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
            if (task.kind ==
                motion_control::core::HierarchicalTaskKind::Position) {
              maximum_primary_position_drift =
                  std::max(maximum_primary_position_drift,
                         task.actual_preservation_drift.maxCoeff());
            }
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

        motion_control::core::JointPtpTrajectoryRequest joint_request;
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
        motion_control::core::TrajectoryGenerationDiagnostics plan_diagnostics;
        app::requireOk(joint_generator.prepare(joint_request, plan_diagnostics),
                       "iterated JointPtpTrajectoryGenerator prepare");
        motion_control::core::JointTrajectorySample sample;
        motion_control::core::TrajectoryGenerationDiagnostics step_diagnostics;
        app::requireOk(joint_generator.step(sample, step_diagnostics),
                       "iterated JointPtpTrajectoryGenerator step");

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
                      .red_primary_task_tcp_position_preservation_tolerance_mps,
              "executed TCP position drift exceeded its hierarchy tolerance");
      require(std::isfinite(maximum_tcp_orientation_error),
              "soft orientation tracking produced a non-finite error");
      require(maximum_primary_position_drift <=
              app_options.interactive.solver
                  .red_primary_task_tcp_position_preservation_tolerance_mps,
              "Secondary changed a Primary position residual beyond tolerance");
    }
    for (const auto profile : {app::Profile::PlannedOtgNullspace, app::Profile::Planned, app::Profile::PostureReferenceTask, app::Profile::PostureReferenceTaskLeft, app::Profile::PostureReferenceTaskRight}) {
      auto pose_options = app::profileDefaults(profile);
      pose_options.interactive.red_rate_hz = 1000;
      pose_options.interactive.solver.hqp_layout="pose-primary";
      pose_options.interactive.robot.inactive_joint_names={"head_yaw_joint","head_pitch_joint","torso_yaw_joint","torso_pitch_joint","knee_pitch_joint","ankle_pitch_joint"};
      pose_options.interactive.urdf_path=argv[1];
      const auto active=app::activeJointNames(robot,pose_options.interactive.robot);
      require(active.size()==14,"pose-primary experiment must use 14 arm joints");
      app::SolverRuntime pose_runtime;app::SolverHandles h;
      app::configureSolver(pose_runtime,h,model,active,collision_model,robot,pose_options);
      app::SolverRequest r;r.reference_frame_name=robot.base_frame;r.captured_state={state,1U,1};
      r.position_targets={{h.red.left_position,initial_left_tcp.translation()},
          {h.red.right_position,initial_right_tcp.translation()},
          {h.red_left_link4,initial_left_link4+Eigen::Vector3d(.005,0,0),true},
          {h.red_right_link4,initial_right_link4,false}};
      if(app::isPostureReferenceProfile(profile)) {
        r.position_targets[2].enabled=pose_options.interactive.elbow_reference.enabled[0];
        r.position_targets[3].enabled=pose_options.interactive.elbow_reference.enabled[1];
        r.position_targets[3].position+=Eigen::Vector3d(.005,0,0);
      }
      r.orientation_targets={{h.red.left_orientation,initial_left_tcp.linear()},
          {h.red.right_orientation,initial_right_tcp.linear()}};
      app::SolverRequest y;y.reference_frame_name=robot.base_frame;y.captured_state={state,1U,1};
      app::SolverSolution solution;app::SolverDiagnostics d;
      app::requireOk(pose_runtime.solveYellow(y,solution,d),"pose-primary Yellow");
      for(int test_case=0;test_case<3;++test_case) {
        if(test_case==1)r.position_targets[2].position+=Eigen::Vector3d(0,.02,0);
        if(test_case==2) {
          r.orientation_targets[0].orientation=Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitX()).toRotationMatrix()*initial_left_tcp.linear();
          r.position_targets[0].feed_forward_velocity=Eigen::Vector3d(.05,.02,0);
        }
        app::requireOk(pose_runtime.solveRed(r,solution,d),"pose-primary solve");
        std::size_t primary=0,tertiary=0;
        for(const auto &task:d.hierarchy.tasks) {
          if(task.handle_value==h.red_left_link4.value || task.handle_value==h.red_right_link4.value) {
            const int a=task.handle_value==h.red_left_link4.value?0:1;
            require(task.enabled==r.position_targets[2+a].enabled,"native solver task mask matches the selected side");
            require(task.priority==motion_control::core::PriorityLevel::Secondary,"posture remains Secondary");
          }
          if(task.priority==motion_control::core::PriorityLevel::Primary) {
            ++primary;
            require(task.enforcement==motion_control::core::HierarchicalTaskEnforcement::Scaled,"all pose Primary tasks must be scaled");
            const double tolerance=task.kind==motion_control::core::HierarchicalTaskKind::Position ?
                pose_options.interactive.solver.red_primary_task_tcp_position_preservation_tolerance_mps :
                pose_options.interactive.solver.red_primary_task_tcp_orientation_preservation_tolerance_radps;
            if(task.actual_preservation_drift.size())require(task.actual_preservation_drift.maxCoeff()<=tolerance+1e-8,"Primary residual preservation");
          }
          if(task.priority==motion_control::core::PriorityLevel::Tertiary)++tertiary;
        }
        require(primary==4 && tertiary==0,"pose-primary topology requires four Primary tasks and no Tertiary");
        require(d.hierarchy.task_scales.size()==2,"pose-primary requires two arm scale groups");
        for(const auto &scale:d.hierarchy.task_scales)
          require(scale.actual_preservation_drift<=pose_options.interactive.solver.red_primary_task_tcp_cartesian_progress_preservation_tolerance+1e-8,"scale preservation");
        require(d.maximum_hard_violation<=pose_options.interactive.solver.maximum_accepted_hard_violation,"shared hard limits");
        if(test_case<2)require(d.hierarchy.passes[1].attempted && d.hierarchy.passes[1].succeeded,"Secondary must execute for elbow perturbation");
        if(test_case==2)require(d.hierarchy.task_scales[0].weighted_progress_scale<d.hierarchy.task_scales[1].weighted_progress_scale,"orientation-limited left arm must have independent scale");
        if(test_case==2) {
          auto candidate=state;
          const auto active_indices=app::activeJointFullIndices(robot,pose_options.interactive.robot);
          for(std::size_t i=0;i<active_indices.size();++i)
            candidate.joint_positions[active_indices[i]]=solution.kinematics_solution.joint_positions[i];
          auto fk=fk_request;fk.state=candidate;
          motion_control::core::ForwardKinematicsSolution result;
          app::requireOk(pose_runtime.computeForwardKinematics(fk,result,fk_diagnostics),"pose-primary correction FK");
          const Eigen::Vector3d velocity=(requirePose(result.poses,robot.left_end_effector_frame).pose.translation()-initial_left_tcp.translation())*pose_options.interactive.red_rate_hz;
          const Eigen::Vector3d expected=d.hierarchy.task_scales[0].weighted_progress_scale*Eigen::Vector3d(.05,.02,0);
          require((velocity-expected).norm()<.003,"position correction must share the orientation-limited arm scale");
        }
      }
    }
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    std::cerr << "planned hierarchical Step OTG solver test failed: "
              << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
