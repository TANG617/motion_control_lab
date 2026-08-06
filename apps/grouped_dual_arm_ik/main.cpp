#include "ik_app_utils.hpp"
#include "r1_robot_config.hpp"

#include "config/interactive_ik_options.hpp"
#include "runtime/grouped_worker.hpp"
#include "runtime/interactive_scheduler.hpp"
#include "runtime/interactive_types.hpp"
#include "runtime/latest_value_mailbox.hpp"
#include "sinks/ik_visualization.hpp"
#include "sinks/visualization_sink_factory.hpp"
#include "teleop/tui_teleop_source.hpp"

#include <motion_control_core/motion_control_core.hpp>

#include <Eigen/Core>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{

namespace mcc = motion_control::core;
namespace mcl = motion_control_lab;

using mcl::requireOk;
using mcl::requirePose;
using mcl::requireTarget;
using mcl::toEigen;
using mcl::toStdVector;

constexpr const char * kProgramId = "mcl_grouped_dual_arm_ik";
constexpr const char * kTitle = "Motion Control Grouped Dual-arm IK";
constexpr const char *kJointStateChannel = "/mc/ik/joint_states";
constexpr const char *kLeftTargetChannel = "/mc/ik/target/left_pose";
constexpr const char *kRightTargetChannel = "/mc/ik/target/right_pose";

bool acceptedStatus(const mcc::Status & status)
{
  return status.ok() || status.code == mcc::StatusCode::BestEffort ||
         status.code == mcc::StatusCode::Saturated;
}

struct TargetSnapshot
{
  std::uint64_t revision{0};
  mcc::Pose left{mcc::Pose::Identity()};
  mcc::Pose right{mcc::Pose::Identity()};
};

struct StateSnapshot
{
  std::uint64_t sequence{0};
  std::int64_t monotonic_time_nanoseconds{0};
  Eigen::VectorXd positions;
  Eigen::VectorXd velocities;
};

struct RedOutputSnapshot
{
  std::uint64_t revision{0};
  StateSnapshot state;
  mcc::Pose left_pose{mcc::Pose::Identity()};
  mcc::Pose right_pose{mcc::Pose::Identity()};
  double solve_time_ms{0.0};
  int iterations{0};
  bool converged{false};
  double left_position_error_m{0.0};
  double left_orientation_error_rad{0.0};
  double right_position_error_m{0.0};
  double right_orientation_error_rad{0.0};
};

struct CartesianHandles
{
  mcc::GroupedPositionTaskHandle left_position;
  mcc::GroupedOrientationTaskHandle left_orientation;
  mcc::GroupedPositionTaskHandle right_position;
  mcc::GroupedOrientationTaskHandle right_orientation;
};

struct GroupedHandles
{
  CartesianHandles red;
  CartesianHandles yellow;
  mcc::GroupedPostureTaskHandle green_posture;
};

mcc::RobotState robotState(const StateSnapshot & state)
{
  mcc::RobotState result;
  result.joint_positions = state.positions;
  result.joint_velocities = state.velocities;
  return result;
}

mcc::CapturedRobotState capturedState(const StateSnapshot & state)
{
  return mcc::CapturedRobotState{
    robotState(state), state.sequence, state.monotonic_time_nanoseconds};
}

void addCartesianTargets(
  const CartesianHandles & handles,
  const TargetSnapshot & target,
  mcc::GroupedInverseKinematicsRequest & request)
{
  request.position_targets[0].position = target.left.translation();
  request.position_targets[1].position = target.right.translation();
  request.orientation_targets[0].orientation = target.left.linear();
  request.orientation_targets[1].orientation = target.right.linear();
  request.position_targets[0].handle = handles.left_position;
  request.position_targets[1].handle = handles.right_position;
  request.orientation_targets[0].handle = handles.left_orientation;
  request.orientation_targets[1].handle = handles.right_orientation;
}

void initializeCartesianRequest(
  const CartesianHandles & handles,
  mcc::GroupedInverseKinematicsRequest & request)
{
  request.position_targets.resize(2);
  request.orientation_targets.resize(2);
  request.position_targets[0].handle = handles.left_position;
  request.position_targets[1].handle = handles.right_position;
  request.orientation_targets[0].handle = handles.left_orientation;
  request.orientation_targets[1].handle = handles.right_orientation;
}

CartesianHandles addCartesianTasks(
  mcc::GroupedKinematicsSolverBuilder & builder,
  mcc::SolverGroup group,
  const std::string & prefix,
  const mcl::R1RobotConfig & robot)
{
  CartesianHandles handles;
  mcc::PositionTaskConfig position;
  position.enforcement = mcc::HardEnforcement{};
  position.name = prefix + "-left-position";
  requireOk(
    builder.addPositionTask(
      group, robot.left_end_effector_frame, position, handles.left_position),
    "Failed to register " + position.name);
  position.name = prefix + "-right-position";
  requireOk(
    builder.addPositionTask(
      group, robot.right_end_effector_frame, position, handles.right_position),
    "Failed to register " + position.name);

  mcc::OrientationTaskConfig orientation;
  orientation.enforcement = mcc::HardEnforcement{};
  orientation.name = prefix + "-left-orientation";
  requireOk(
    builder.addOrientationTask(
      group, robot.left_end_effector_frame, orientation, handles.left_orientation),
    "Failed to register " + orientation.name);
  orientation.name = prefix + "-right-orientation";
  requireOk(
    builder.addOrientationTask(
      group, robot.right_end_effector_frame, orientation, handles.right_orientation),
    "Failed to register " + orientation.name);
  return handles;
}

void addExplicitLimits(
  mcc::GroupedKinematicsSolverBuilder & builder,
  mcc::SolverGroup group)
{
  mcc::JointPositionLimitConfig position;
  position.enforcement = mcc::HardEnforcement{};
  mcc::GroupedJointPositionLimitHandle position_handle;
  requireOk(
    builder.addJointPositionLimits(group, position, position_handle),
    "Failed to register grouped joint-position limits");

  mcc::JointVelocityLimitConfig velocity;
  velocity.mode = mcc::JointVelocityLimitMode::VelocityOnly;
  velocity.enforcement = mcc::HardEnforcement{};
  mcc::GroupedJointVelocityLimitHandle velocity_handle;
  requireOk(
    builder.addJointVelocityLimits(group, velocity, velocity_handle),
    "Failed to register grouped joint-velocity limits");
}

std::string statusDetail(const mcc::Status & status)
{
  return status.message.empty() ? "solver returned a rejected result" : status.message;
}

void fillRedErrors(
  const CartesianHandles & handles,
  const mcc::GroupedInverseKinematicsDiagnostics & diagnostics,
  RedOutputSnapshot & output)
{
  for (const auto & error : diagnostics.kinematics.position_errors) {
    if (error.handle.value == handles.left_position.value) {
      output.left_position_error_m = error.norm_m;
    } else if (error.handle.value == handles.right_position.value) {
      output.right_position_error_m = error.norm_m;
    }
  }
  for (const auto & error : diagnostics.kinematics.orientation_errors) {
    if (error.handle.value == handles.left_orientation.value) {
      output.left_orientation_error_rad = error.norm_rad;
    } else if (error.handle.value == handles.right_orientation.value) {
      output.right_orientation_error_rad = error.norm_rad;
    }
  }
}

int run(int argc, char ** argv)
{
  const auto options = mcl::parseGroupedInteractiveIkOptions(argc, argv);
  if (!std::filesystem::exists(options.urdf_path)) {
    throw std::runtime_error("URDF does not exist: " + options.urdf_path);
  }

  const auto & robot = mcl::r1RobotConfig();
  const auto & joint_names = robot.joint_names;
  const Eigen::VectorXd initial_positions = toEigen(robot.default_positions);
  StateSnapshot initial_state;
  initial_state.positions = initial_positions;
  initial_state.velocities.setZero(initial_positions.size());

  mcc::RobotModelDescription model_description;
  model_description.urdf_path = options.urdf_path;
  model_description.base_frame = robot.base_frame;
  model_description.joint_names = joint_names;
  model_description.end_effector_names = {
    robot.left_end_effector_frame,
    robot.right_end_effector_frame};
  std::shared_ptr<const mcc::RobotModel> model;
  requireOk(mcc::RobotModel::load(model_description, model), "Failed to load robot model");

  mcc::GroupedKinematicsSolverConfig solver_config;
  solver_config.profile = mcc::SolverProfile::RedYellowGreen;
  solver_config.red.mode = mcc::IkSolveMode::ServoStep;
  solver_config.red.maximum_iterations = 1;
  solver_config.red.maximum_solve_time_ms = 1000.0 / options.red_rate_hz;
  solver_config.yellow.mode = mcc::IkSolveMode::TargetSolve;
  solver_config.yellow.maximum_iterations = 80;
  solver_config.yellow.maximum_solve_time_ms = 1000.0 / options.yellow_rate_hz;
  solver_config.green.mode = mcc::IkSolveMode::TargetSolve;
  solver_config.green.maximum_iterations = 80;
  solver_config.green.maximum_solve_time_ms = 1000.0 / options.green_rate_hz;
  for (auto * config : {&solver_config.red, &solver_config.yellow, &solver_config.green}) {
    config->qp.backend = mcc::QpBackend::ProxQp;
    config->qp.regularization = 1.0e-8;
    config->position_tolerance_m = 1.0e-4;
    config->orientation_tolerance_rad = 1.0e-4;
    config->minimum_position_improvement_m = 1.0e-8;
    config->minimum_orientation_improvement_rad = 1.0e-8;
  }

  mcc::GroupedKinematicsSolverBuilder builder;
  requireOk(
    builder.configure(model, joint_names, solver_config, initial_positions),
    "Failed to configure grouped IK builder");
  GroupedHandles handles;
  handles.red = addCartesianTasks(builder, mcc::SolverGroup::Red, "red", robot);
  handles.yellow = addCartesianTasks(builder, mcc::SolverGroup::Yellow, "yellow", robot);
  mcc::PostureTaskConfig green_posture;
  green_posture.name = "green-initial-posture";
  green_posture.enforcement = mcc::squaredL2Penalty(1.0, 1);
  green_posture.reference_positions = initial_positions;
  requireOk(
    builder.addPostureTask(
      mcc::SolverGroup::Green, green_posture, handles.green_posture),
    "Failed to register Green posture task");
  addExplicitLimits(builder, mcc::SolverGroup::Red);
  addExplicitLimits(builder, mcc::SolverGroup::Yellow);
  addExplicitLimits(builder, mcc::SolverGroup::Green);

  mcc::GroupedKinematicsSolver solver;
  requireOk(builder.finalize(solver), "Failed to finalize grouped IK solver");

  mcc::ForwardKinematicsSolution initial_fk;
  mcc::ForwardKinematicsDiagnostics initial_fk_diagnostics;
  requireOk(
    solver.solveForwardKinematics(
      mcc::SolverGroup::Red,
      robotState(initial_state),
      initial_fk,
      initial_fk_diagnostics,
      mcc::JointPositionValidation::LimitChecked),
    "Initial FK failed");

  TargetSnapshot initial_target;
  initial_target.revision = 1;
  initial_target.left = requirePose(initial_fk.poses, robot.left_end_effector_frame).pose;
  initial_target.right = requirePose(initial_fk.poses, robot.right_end_effector_frame).pose;

  // Warm all numerical workspaces and the coupling path before deadlines apply.
  requireOk(solver.beginRun(1), "Failed to begin warm-up run");
  {
    mcc::GroupedInverseKinematicsSolution solution;
    mcc::GroupedInverseKinematicsDiagnostics diagnostics;
    mcc::GroupedInverseKinematicsRequest green;
    green.current_state = capturedState(initial_state);
    green.dt = 1.0 / options.green_rate_hz;
    green.posture_targets.push_back(
      {handles.green_posture, initial_positions, true});
    auto status = solver.solveInverseKinematics(
      mcc::SolverGroup::Green, green, solution, diagnostics);
    if (!acceptedStatus(status) || !diagnostics.attempt_accepted) {
      throw std::runtime_error("Green warm-up failed: " + statusDetail(status));
    }

    mcc::GroupedInverseKinematicsRequest yellow;
    initializeCartesianRequest(handles.yellow, yellow);
    yellow.current_state = capturedState(initial_state);
    yellow.dt = 1.0 / options.yellow_rate_hz;
    addCartesianTargets(handles.yellow, initial_target, yellow);
    status = solver.solveInverseKinematics(
      mcc::SolverGroup::Yellow, yellow, solution, diagnostics);
    if (!acceptedStatus(status) || !diagnostics.attempt_accepted) {
      throw std::runtime_error("Yellow warm-up failed: " + statusDetail(status));
    }

    mcc::GroupedInverseKinematicsRequest red;
    initializeCartesianRequest(handles.red, red);
    red.current_state = capturedState(initial_state);
    red.dt = 1.0 / options.red_rate_hz;
    addCartesianTargets(handles.red, initial_target, red);
    status = solver.solveInverseKinematics(
      mcc::SolverGroup::Red, red, solution, diagnostics);
    if (!acceptedStatus(status) || !diagnostics.attempt_accepted) {
      throw std::runtime_error("Red warm-up failed: " + statusDetail(status));
    }
  }
  requireOk(solver.beginRun(2), "Failed to begin timed grouped run");

  RedOutputSnapshot initial_output;
  initial_output.state = initial_state;
  initial_output.left_pose = initial_target.left;
  initial_output.right_pose = initial_target.right;
  mcl::LatestValueMailbox<TargetSnapshot> target_to_red(initial_target);
  mcl::LatestValueMailbox<TargetSnapshot> target_to_yellow(initial_target);
  mcl::LatestValueMailbox<StateSnapshot> state_to_yellow(initial_state);
  mcl::LatestValueMailbox<StateSnapshot> state_to_green(initial_state);
  mcl::LatestValueMailbox<RedOutputSnapshot> output_to_ui(initial_output);
  target_to_red.publish(initial_target);
  target_to_yellow.publish(initial_target);
  state_to_yellow.publish(initial_state);
  state_to_green.publish(initial_state);
  output_to_ui.publish(initial_output);

  const auto presentation = mcl::makeArmPresentation(
    robot,
    {kJointStateChannel, kLeftTargetChannel, kRightTargetChannel});
  mcl::TuiTeleopSource tui(
    options.tui,
    options.ui_rate_hz,
    kTitle,
    presentation,
    {
      {mcl::ArmSide::Left, initial_target.left},
      {mcl::ArmSide::Right, initial_target.right}
    },
    true);
  auto visualization_sink = mcl::createVisualizationSink(options.visualization, kProgramId);

  std::atomic_bool stop_requested{false};
  mcl::GroupedFaultState fault;
  std::thread red_thread;
  std::thread yellow_thread;
  std::thread green_thread;
  auto joinWorkers = [&]() {
      stop_requested.store(true, std::memory_order_release);
      if (red_thread.joinable()) {
        red_thread.join();
      }
      if (yellow_thread.joinable()) {
        yellow_thread.join();
      }
      if (green_thread.joinable()) {
        green_thread.join();
      }
    };

  bool sink_open = false;
  try {
    visualization_sink->open({"interactive-preview", kProgramId});
    sink_open = true;
    mcl::installInteractiveSignalHandlers();

    green_thread = std::thread([&]() {
      StateSnapshot state = initial_state;
      mcc::GroupedInverseKinematicsRequest request;
      request.posture_targets.push_back(
        {handles.green_posture, initial_positions, true});
      mcc::GroupedInverseKinematicsSolution solution;
      mcc::GroupedInverseKinematicsDiagnostics diagnostics;
      mcl::runPeriodicWorker(
        {mcl::WorkerGroup::Green, options.green_rate_hz},
        stop_requested,
        fault,
        [&](double dt, std::int64_t) {
          state_to_green.readLatest(state);
          request.current_state = capturedState(state);
          request.dt = dt;
          const auto status = solver.solveInverseKinematics(
            mcc::SolverGroup::Green, request, solution, diagnostics);
          return mcl::WorkerIterationResult{
            diagnostics.attempt_accepted,
            diagnostics.attempt_revision,
            diagnostics.kinematics.solve_time_ms,
            diagnostics.attempt_accepted ? std::string{} : statusDetail(status)};
        });
    });

    yellow_thread = std::thread([&]() {
      StateSnapshot state = initial_state;
      TargetSnapshot target = initial_target;
      mcc::GroupedInverseKinematicsRequest request;
      initializeCartesianRequest(handles.yellow, request);
      mcc::GroupedInverseKinematicsSolution solution;
      mcc::GroupedInverseKinematicsDiagnostics diagnostics;
      mcl::runPeriodicWorker(
        {mcl::WorkerGroup::Yellow, options.yellow_rate_hz},
        stop_requested,
        fault,
        [&](double dt, std::int64_t) {
          state_to_yellow.readLatest(state);
          target_to_yellow.readLatest(target);
          request.current_state = capturedState(state);
          request.dt = dt;
          addCartesianTargets(handles.yellow, target, request);
          const auto status = solver.solveInverseKinematics(
            mcc::SolverGroup::Yellow, request, solution, diagnostics);
          return mcl::WorkerIterationResult{
            diagnostics.attempt_accepted,
            diagnostics.attempt_revision,
            diagnostics.kinematics.solve_time_ms,
            diagnostics.attempt_accepted ? std::string{} : statusDetail(status)};
        });
    });

    red_thread = std::thread([&]() {
      StateSnapshot state = initial_state;
      TargetSnapshot target = initial_target;
      RedOutputSnapshot output = initial_output;
      mcc::GroupedInverseKinematicsRequest request;
      initializeCartesianRequest(handles.red, request);
      mcc::GroupedInverseKinematicsSolution solution;
      mcc::GroupedInverseKinematicsDiagnostics diagnostics;
      mcl::runPeriodicWorker(
        {mcl::WorkerGroup::Red, options.red_rate_hz},
        stop_requested,
        fault,
        [&](double dt, std::int64_t sample_time_ns) {
          target_to_red.readLatest(target);
          request.current_state = capturedState(state);
          request.dt = dt;
          addCartesianTargets(handles.red, target, request);
          const auto status = solver.solveInverseKinematics(
            mcc::SolverGroup::Red, request, solution, diagnostics);
          if (diagnostics.attempt_accepted) {
            state.positions = solution.value.joint_positions;
            state.velocities = solution.value.joint_velocities;
            ++state.sequence;
            state.monotonic_time_nanoseconds = sample_time_ns;
            state_to_yellow.publish(state);
            state_to_green.publish(state);

            output.revision = diagnostics.value_revision;
            output.state = state;
            output.left_pose = requirePose(
              solution.value.solved_poses, robot.left_end_effector_frame).pose;
            output.right_pose = requirePose(
              solution.value.solved_poses, robot.right_end_effector_frame).pose;
            output.solve_time_ms = diagnostics.kinematics.solve_time_ms;
            output.iterations = diagnostics.kinematics.iterations;
            output.converged = diagnostics.kinematics.converged;
            fillRedErrors(handles.red, diagnostics, output);
            output_to_ui.publish(output);
          }
          return mcl::WorkerIterationResult{
            diagnostics.attempt_accepted,
            diagnostics.attempt_revision,
            diagnostics.kinematics.solve_time_ms,
            diagnostics.attempt_accepted ? std::string{} : statusDetail(status)};
        });
    });

    mcl::InteractiveScheduler ui_scheduler({options.ui_rate_hz, options.duration_s});
    TargetSnapshot published_target = initial_target;
    RedOutputSnapshot latest_output = initial_output;
    std::size_t publish_count = 0;
    std::int64_t last_sample_time_ns = 0;
    std::uint64_t last_emit_time_ns = 0;
    mcl::IkDebugFrame frame;
    frame.joint_names = joint_names;
    frame.positions = robot.default_positions;
    frame.velocities.assign(joint_names.size(), 0.0);
    frame.selected_side = mcl::parseArmSide(options.tui.side);

    while (!stop_requested.load(std::memory_order_acquire)) {
      const auto schedule = ui_scheduler.next();
      if (!schedule) {
        break;
      }
      tui.poll();
      output_to_ui.readLatest(latest_output);
      if (const auto reset_side = tui.consumeResetRequest()) {
        tui.setTargetPose(
          *reset_side,
          *reset_side == mcl::ArmSide::Left ?
          latest_output.left_pose : latest_output.right_pose,
          std::string{"Reset "} + mcl::armSideName(*reset_side) +
          " target from latest Red output");
      }
      const auto & command = tui.command();
      if (command.stop_requested) {
        break;
      }

      if (schedule->update_due) {
        if (!command.paused) {
          published_target.revision++;
          published_target.left = requireTarget(command.targets, mcl::ArmSide::Left).target_pose;
          published_target.right = requireTarget(command.targets, mcl::ArmSide::Right).target_pose;
          target_to_red.publish(published_target);
          target_to_yellow.publish(published_target);
        }

        frame.targets = command.targets;
        frame.positions = toStdVector(latest_output.state.positions);
        frame.velocities = toStdVector(latest_output.state.velocities);
        frame.ik_status = fault.triggered() ? "fault" : "running";
        frame.iterations = latest_output.iterations;
        frame.converged = latest_output.converged;
        frame.solve_time_ms = latest_output.solve_time_ms;
        frame.target_errors = {
          {mcl::ArmSide::Left,
            latest_output.left_position_error_m,
            latest_output.left_orientation_error_rad},
          {mcl::ArmSide::Right,
            latest_output.right_position_error_m,
            latest_output.right_orientation_error_rad}};
        frame.status = command.status;
        frame.paused = command.paused;
        frame.selected_side = command.selected_side;

        visualization_sink->write(mcl::makeIkVisualizationFrame(
          frame,
          presentation,
          publish_count,
          schedule->sample_time_ns,
          schedule->emit_time_ns));
        last_sample_time_ns = schedule->sample_time_ns;
        last_emit_time_ns = schedule->emit_time_ns;
        ++publish_count;
        tui.render(frame, publish_count, visualization_sink->status());
      }
      ui_scheduler.sleep();
    }

    joinWorkers();
    if (const auto recorded_fault = fault.snapshot()) {
      output_to_ui.readLatest(latest_output);
      frame.positions = toStdVector(latest_output.state.positions);
      frame.velocities = toStdVector(latest_output.state.velocities);
      frame.ik_status = "fault";
      frame.status =
        std::string{mcl::workerGroupName(recorded_fault->group)} + " " +
        mcl::workerFailureName(recorded_fault->failure) +
        " revision=" + std::to_string(recorded_fault->revision) +
        " release_lateness_ms=" + std::to_string(recorded_fault->release_lateness_ms) +
        " execution_ms=" + std::to_string(recorded_fault->execution_ms) +
        " release_to_finish_ms=" + std::to_string(recorded_fault->release_to_finish_ms) +
        " deadline_ms=" + std::to_string(recorded_fault->deadline_ms) +
        " overrun_ms=" + std::to_string(recorded_fault->overrun_ms) +
        " solver_ms=" + std::to_string(recorded_fault->solver_ms) +
        (recorded_fault->detail.empty() ? "" : " " + recorded_fault->detail);
      visualization_sink->write(mcl::makeIkVisualizationFrame(
        frame,
        presentation,
        publish_count,
        last_sample_time_ns,
        last_emit_time_ns));
      std::cerr << kProgramId << ": " << frame.status << "\n";
    }

    visualization_sink->flush();
    visualization_sink->close();
    sink_open = false;
  } catch (...) {
    joinWorkers();
    if (sink_open) {
      try {
        visualization_sink->close();
      } catch (...) {
      }
    }
    throw;
  }

  return fault.triggered() ? EXIT_FAILURE : EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    return run(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << kProgramId << ": " << error.what() << "\n";
    return EXIT_FAILURE;
  }
}
