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

#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
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
constexpr const char * kLeftElbowFrame = "left_arm_link4";
constexpr const char * kRightElbowFrame = "right_arm_link4";
constexpr double kYellowCartesianWeight = 10.0;
constexpr double kYellowElbowWeight = 3.0;
constexpr double kYellowElbowOutwardOffsetM = 2.5;

bool operationSucceeded(const mcc::Status & status)
{
  return status.ok();
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
  mcc::GroupedPositionTaskHandle yellow_left_elbow;
  mcc::GroupedPositionTaskHandle yellow_right_elbow;
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
  const mcc::FrameName & reference_frame_name,
  mcc::GroupedInverseKinematicsRequest & request)
{
  request.reference_frame_name = reference_frame_name;
  request.position_targets.resize(2);
  request.orientation_targets.resize(2);
  request.position_targets[0].handle = handles.left_position;
  request.position_targets[1].handle = handles.right_position;
  request.orientation_targets[0].handle = handles.left_orientation;
  request.orientation_targets[1].handle = handles.right_orientation;
}

void initializeYellowRequest(
  const GroupedHandles & handles,
  const mcc::FrameName & reference_frame_name,
  mcc::GroupedInverseKinematicsRequest & request)
{
  initializeCartesianRequest(handles.yellow, reference_frame_name, request);
  request.position_targets.resize(4);
  request.position_targets[2].handle = handles.yellow_left_elbow;
  request.position_targets[3].handle = handles.yellow_right_elbow;
}

void addYellowTargets(
  const GroupedHandles & handles,
  const TargetSnapshot & target,
  const Eigen::Vector3d & left_elbow_target,
  const Eigen::Vector3d & right_elbow_target,
  mcc::GroupedInverseKinematicsRequest & request)
{
  addCartesianTargets(handles.yellow, target, request);
  request.position_targets[2].position = left_elbow_target;
  request.position_targets[3].position = right_elbow_target;
}

CartesianHandles addCartesianTasks(
  mcc::GroupedKinematicsSolverBuilder & builder,
  mcc::SolverGroup group,
  const std::string & prefix,
  const mcl::R1RobotConfig & robot,
  const mcc::RequirementEnforcement & position_enforcement,
  const mcc::RequirementEnforcement & orientation_enforcement)
{
  CartesianHandles handles;
  mcc::PositionTaskConfig position;
  position.enforcement = position_enforcement;
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
  orientation.enforcement = orientation_enforcement;
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

const mcc::RequirementDiagnostic * maximumViolatedHardRequirement(
  const mcc::OptimizationDiagnostics & diagnostics)
{
  if (diagnostics.maximum_hard_violation <= 0.0) {
    return nullptr;
  }
  const mcc::RequirementDiagnostic * result = nullptr;
  double smallest_distance = std::numeric_limits<double>::infinity();
  for (const auto & requirement : diagnostics.requirements) {
    // Hard requirements do not accumulate a soft cost. Matching against the
    // independently computed maximum avoids selecting the soft coupling slot.
    if (!requirement.enabled || requirement.cost != 0.0) {
      continue;
    }
    const double distance = std::abs(
      requirement.maximum_violation - diagnostics.maximum_hard_violation);
    if (distance < smallest_distance) {
      smallest_distance = distance;
      result = &requirement;
    }
  }
  return result;
}

std::string rejectedAttemptDetail(
  const mcc::Status & status,
  const mcc::GroupedInverseKinematicsDiagnostics & diagnostics)
{
  const auto & kinematics = diagnostics.kinematics;
  const auto & optimization = kinematics.optimization;
  const auto * requirement = maximumViolatedHardRequirement(optimization);

  std::ostringstream output;
  output << statusDetail(status) << std::scientific << std::setprecision(9)
         << " maximum_hard_violation=" << optimization.maximum_hard_violation;
  if (requirement == nullptr) {
    output << " max_violated_requirement=<unavailable>"
           << " maximum_violation=<unavailable>"
           << " requirement_unit=<unavailable>"
           << " requirement_source=<unavailable>";
  } else {
    output << " max_violated_requirement=\"" << requirement->name << '"'
           << " maximum_violation=" << requirement->maximum_violation
           << " requirement_unit=\"" << requirement->unit << '"'
           << " requirement_source=\"" << requirement->source << '"';
  }

  output << " position_errors=[";
  for (std::size_t index = 0; index < kinematics.position_errors.size(); ++index) {
    if (index != 0) {
      output << ',';
    }
    const auto & error = kinematics.position_errors[index];
    output << "{frame=\"" << error.frame_name << "\",norm_m=" << error.norm_m << '}';
  }
  output << "] orientation_errors=[";
  for (std::size_t index = 0; index < kinematics.orientation_errors.size(); ++index) {
    if (index != 0) {
      output << ',';
    }
    const auto & error = kinematics.orientation_errors[index];
    output << "{frame=\"" << error.frame_name << "\",norm_rad=" << error.norm_rad << '}';
  }
  output << "] saturated_joints=[";
  for (std::size_t index = 0; index < kinematics.saturated_joints.size(); ++index) {
    if (index != 0) {
      output << ',';
    }
    output << '"' << kinematics.saturated_joints[index] << '"';
  }
  output << ']';
  return output.str();
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
  initial_state.sequence = 1;
  initial_state.monotonic_time_nanoseconds = 1;
  initial_state.positions = initial_positions;
  initial_state.velocities.setZero(initial_positions.size());

  mcc::RobotModelDescription model_description;
  model_description.urdf_path = options.urdf_path;
  model_description.kinematics_reference_frame = robot.base_frame;
  model_description.joint_names = joint_names;
  std::shared_ptr<const mcc::RobotModel> model;
  requireOk(mcc::RobotModel::load(model_description, model), "Failed to load robot model");

  mcc::GroupedKinematicsSolverConfig solver_config;
  solver_config.profile = mcc::GroupedSolverProfile::RedYellowGreen;
  solver_config.red.mode = mcc::IkSolveMode::ServoStep;
  solver_config.red.servo_period = 1.0 / options.red_rate_hz;
  solver_config.red.maximum_iterations = 1;
  solver_config.red.soft_solve_time_budget_ms = 1000.0 / options.red_rate_hz;
  solver_config.yellow.mode = mcc::IkSolveMode::ServoStep;
  solver_config.yellow.servo_period = 1.0 / options.yellow_rate_hz;
  solver_config.yellow.maximum_iterations = 1;
  solver_config.yellow.soft_solve_time_budget_ms = 1000.0 / options.yellow_rate_hz;
  solver_config.green.mode = mcc::IkSolveMode::ServoStep;
  solver_config.green.servo_period = 1.0 / options.green_rate_hz;
  solver_config.green.maximum_iterations = 1;
  solver_config.green.soft_solve_time_budget_ms = 1000.0 / options.green_rate_hz;
  for (auto * config : {&solver_config.red, &solver_config.yellow, &solver_config.green}) {
    config->joint_limit_policy = mcc::KinematicsJointLimitPolicy::ExplicitRequirements;
    config->qp.backend = mcc::QpBackend::ProxQp;
    config->qp.regularization = 1.0e-8;
    config->position_tolerance_m = 1.0e-4;
    config->orientation_tolerance_rad = 1.0e-4;
    config->minimum_position_improvement_m = 1.0e-8;
    config->minimum_orientation_improvement_rad = 1.0e-8;
  }

  mcc::GroupedKinematicsSolverBuilder builder;
  requireOk(
    builder.configure(model, joint_names, solver_config),
    "Failed to configure grouped IK builder");
  GroupedHandles handles;
  handles.red = addCartesianTasks(
    builder,
    mcc::SolverGroup::Red,
    "red",
    robot,
    mcc::HardEnforcement{},
    mcc::HardEnforcement{});
  handles.yellow = addCartesianTasks(
    builder,
    mcc::SolverGroup::Yellow,
    "yellow",
    robot,
    mcc::squaredL2Penalty(kYellowCartesianWeight, 3),
    mcc::squaredL2Penalty(kYellowCartesianWeight, 3));
  mcc::PositionTaskConfig yellow_left_elbow;
  yellow_left_elbow.name = "yellow-left-elbow-outward";
  yellow_left_elbow.enforcement = mcc::squaredL2Penalty(kYellowElbowWeight, 3);
  requireOk(
    builder.addPositionTask(
      mcc::SolverGroup::Yellow,
      kLeftElbowFrame,
      yellow_left_elbow,
      handles.yellow_left_elbow),
    "Failed to register Yellow left-elbow task");
  mcc::PositionTaskConfig yellow_right_elbow;
  yellow_right_elbow.name = "yellow-right-elbow-outward";
  yellow_right_elbow.enforcement = mcc::squaredL2Penalty(kYellowElbowWeight, 3);
  requireOk(
    builder.addPositionTask(
      mcc::SolverGroup::Yellow,
      kRightElbowFrame,
      yellow_right_elbow,
      handles.yellow_right_elbow),
    "Failed to register Yellow right-elbow task");
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

  mcc::ForwardKinematicsRequest initial_fk_request;
  initial_fk_request.state = robotState(initial_state);
  initial_fk_request.frame_names = {
    robot.left_end_effector_frame,
    robot.right_end_effector_frame,
    kLeftElbowFrame,
    kRightElbowFrame};
  initial_fk_request.reference_frame_name = robot.base_frame;
  mcc::ForwardKinematicsSolution initial_fk;
  mcc::ForwardKinematicsDiagnostics initial_fk_diagnostics;
  requireOk(
    solver.computeForwardKinematics(
      mcc::SolverGroup::Red,
      initial_fk_request,
      initial_fk,
      initial_fk_diagnostics),
    "Initial FK failed");

  TargetSnapshot initial_target;
  initial_target.revision = 1;
  initial_target.left = requirePose(initial_fk.poses, robot.left_end_effector_frame).pose;
  initial_target.right = requirePose(initial_fk.poses, robot.right_end_effector_frame).pose;
  Eigen::Vector3d left_elbow_target =
    requirePose(initial_fk.poses, kLeftElbowFrame).pose.translation();
  Eigen::Vector3d right_elbow_target =
    requirePose(initial_fk.poses, kRightElbowFrame).pose.translation();
  left_elbow_target.y() += kYellowElbowOutwardOffsetM;
  right_elbow_target.y() -= kYellowElbowOutwardOffsetM;

  // Warm all numerical workspaces and the coupling path before deadlines apply.
  requireOk(solver.beginRun(1), "Failed to begin warm-up run");
  {
    mcc::GroupedInverseKinematicsSolution solution;
    mcc::GroupedInverseKinematicsDiagnostics diagnostics;
    mcc::GroupedInverseKinematicsRequest green;
    green.reference_frame_name = robot.base_frame;
    green.captured_state = capturedState(initial_state);
    auto status = solver.solveInverseKinematics(
      mcc::SolverGroup::Green, green, solution, diagnostics);
    if (!operationSucceeded(status) || !diagnostics.attempt_accepted) {
      throw std::runtime_error(
              "Green warm-up failed: " + rejectedAttemptDetail(status, diagnostics));
    }

    mcc::GroupedInverseKinematicsRequest yellow;
    initializeYellowRequest(handles, robot.base_frame, yellow);
    yellow.captured_state = capturedState(initial_state);
    addYellowTargets(
      handles, initial_target, left_elbow_target, right_elbow_target, yellow);
    status = solver.solveInverseKinematics(
      mcc::SolverGroup::Yellow, yellow, solution, diagnostics);
    if (!operationSucceeded(status) || !diagnostics.attempt_accepted) {
      throw std::runtime_error(
              "Yellow warm-up failed: " + rejectedAttemptDetail(status, diagnostics));
    }

    mcc::GroupedInverseKinematicsRequest red;
    initializeCartesianRequest(handles.red, robot.base_frame, red);
    red.captured_state = capturedState(initial_state);
    addCartesianTargets(handles.red, initial_target, red);
    status = solver.solveInverseKinematics(
      mcc::SolverGroup::Red, red, solution, diagnostics);
    if (!operationSucceeded(status) || !diagnostics.attempt_accepted) {
      throw std::runtime_error(
              "Red warm-up failed: " + rejectedAttemptDetail(status, diagnostics));
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
    robot, mcl::foxgloveIkVisualizationChannels());
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

  mcl::WorkerStopController stop_controller;
  mcl::GroupedFaultState fault;
  mcl::PeriodicWorkerDiagnostics red_worker_diagnostics;
  mcl::PeriodicWorkerDiagnostics yellow_worker_diagnostics;
  mcl::PeriodicWorkerDiagnostics green_worker_diagnostics;
  std::thread red_thread;
  std::thread yellow_thread;
  std::thread green_thread;
  auto joinWorkers = [&]() {
      stop_controller.requestStop();
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
      request.reference_frame_name = robot.base_frame;
      mcc::GroupedInverseKinematicsSolution solution;
      mcc::GroupedInverseKinematicsDiagnostics diagnostics;
      mcl::runPeriodicWorker(
        {mcl::WorkerGroup::Green, options.green_rate_hz, options.deadline_policy},
        stop_controller,
        fault,
        green_worker_diagnostics,
        [&](double, std::int64_t) {
          state_to_green.readLatest(state);
          request.captured_state = capturedState(state);
          const auto status = solver.solveInverseKinematics(
            mcc::SolverGroup::Green, request, solution, diagnostics);
          return mcl::WorkerIterationResult{
            diagnostics.attempt_accepted,
            diagnostics.attempt_revision,
            diagnostics.kinematics.solve_time_ms,
            diagnostics.attempt_accepted ? std::string{} :
            rejectedAttemptDetail(status, diagnostics)};
        });
    });

    yellow_thread = std::thread([&]() {
      StateSnapshot state = initial_state;
      TargetSnapshot target = initial_target;
      mcc::GroupedInverseKinematicsRequest request;
      initializeYellowRequest(handles, robot.base_frame, request);
      mcc::GroupedInverseKinematicsSolution solution;
      mcc::GroupedInverseKinematicsDiagnostics diagnostics;
      mcl::runPeriodicWorker(
        {mcl::WorkerGroup::Yellow, options.yellow_rate_hz, options.deadline_policy},
        stop_controller,
        fault,
        yellow_worker_diagnostics,
        [&](double, std::int64_t) {
          state_to_yellow.readLatest(state);
          target_to_yellow.readLatest(target);
          request.captured_state = capturedState(state);
          addYellowTargets(
            handles, target, left_elbow_target, right_elbow_target, request);
          const auto status = solver.solveInverseKinematics(
            mcc::SolverGroup::Yellow, request, solution, diagnostics);
          return mcl::WorkerIterationResult{
            diagnostics.attempt_accepted,
            diagnostics.attempt_revision,
            diagnostics.kinematics.solve_time_ms,
            diagnostics.attempt_accepted ? std::string{} :
            rejectedAttemptDetail(status, diagnostics)};
        });
    });

    red_thread = std::thread([&]() {
      StateSnapshot state = initial_state;
      TargetSnapshot target = initial_target;
      RedOutputSnapshot output = initial_output;
      mcc::GroupedInverseKinematicsRequest request;
      initializeCartesianRequest(handles.red, robot.base_frame, request);
      mcc::GroupedInverseKinematicsSolution solution;
      mcc::GroupedInverseKinematicsDiagnostics diagnostics;
      mcl::runPeriodicWorker(
        {mcl::WorkerGroup::Red, options.red_rate_hz, options.deadline_policy},
        stop_controller,
        fault,
        red_worker_diagnostics,
        [&](double, std::int64_t sample_time_ns) {
          target_to_red.readLatest(target);
          request.captured_state = capturedState(state);
          addCartesianTargets(handles.red, target, request);
          const auto status = solver.solveInverseKinematics(
            mcc::SolverGroup::Red, request, solution, diagnostics);
          if (diagnostics.attempt_accepted) {
            state.positions = solution.kinematics_solution.joint_positions;
            state.velocities = solution.kinematics_solution.joint_velocities;
            ++state.sequence;
            state.monotonic_time_nanoseconds = sample_time_ns;
            state_to_yellow.publish(state);
            state_to_green.publish(state);

            output.revision = diagnostics.value_revision;
            output.state = state;
            output.left_pose = requirePose(
              solution.kinematics_solution.solved_poses,
              robot.left_end_effector_frame).pose;
            output.right_pose = requirePose(
              solution.kinematics_solution.solved_poses,
              robot.right_end_effector_frame).pose;
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
            diagnostics.attempt_accepted ? std::string{} :
            rejectedAttemptDetail(status, diagnostics)};
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
    frame.forward_kinematics = {
      {mcl::ArmSide::Left, initial_output.left_pose},
      {mcl::ArmSide::Right, initial_output.right_pose}};
    frame.selected_side = mcl::parseArmSide(options.tui.side);

    while (!stop_controller.stopRequested()) {
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
        frame.forward_kinematics = {
          {mcl::ArmSide::Left, latest_output.left_pose},
          {mcl::ArmSide::Right, latest_output.right_pose}};
        frame.positions = toStdVector(latest_output.state.positions);
        frame.velocities = toStdVector(latest_output.state.velocities);
        const auto red_stats = red_worker_diagnostics.snapshot();
        const auto yellow_stats = yellow_worker_diagnostics.snapshot();
        const auto green_stats = green_worker_diagnostics.snapshot();
        frame.ik_status = fault.triggered() ? "fault" :
          "running deadline_misses R=" + std::to_string(red_stats.deadline_miss_count) +
          " Y=" + std::to_string(yellow_stats.deadline_miss_count) +
          " G=" + std::to_string(green_stats.deadline_miss_count) +
          " skipped R=" + std::to_string(red_stats.skipped_release_count) +
          " Y=" + std::to_string(yellow_stats.skipped_release_count) +
          " G=" + std::to_string(green_stats.skipped_release_count);
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
        frame.status = command.status +
          " | skipped_releases R=" + std::to_string(red_stats.skipped_release_count) +
          " Y=" + std::to_string(yellow_stats.skipped_release_count) +
          " G=" + std::to_string(green_stats.skipped_release_count);
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
      frame.forward_kinematics = {
        {mcl::ArmSide::Left, latest_output.left_pose},
        {mcl::ArmSide::Right, latest_output.right_pose}};
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
