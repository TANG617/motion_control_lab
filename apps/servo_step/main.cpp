#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <motion_control_core/motion_control_core.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "adapters/replay/replay_support.hpp"
#include "app_options.hpp"
#include "console/tui_console.hpp"
#include "cpu_affinity.hpp"
#include "ik_app_utils.hpp"
#include "motion_control_lab/run_artifacts.hpp"
#include "motion_control_lab/sha256.hpp"
#include "placo/kinematics/kinematics_solver.h"
#include "placo/model/robot_wrapper.h"
#include "r1_interactive_config.hpp"
#include "r1_robot_config.hpp"
#include "runtime/interactive_scheduler.hpp"
#include "runtime/interactive_types.hpp"
#include "runtime/rolling_percentiles.hpp"
#include "sinks/ik_render_batch.hpp"
#include "sinks/preview_sink_factory.hpp"

namespace
{

namespace mcc = motion_control::core;
namespace mcl = motion_control_lab;
namespace replay = motion_control_lab::replay;

using mcl::servo_step::AppOptions;
using mcl::servo_step::MccBackend;
using mcl::servo_step::ReplayAppOptions;
using mcl::servo_step::SolverKind;
using mcl::servo_step::parseAppOptions;
using mcl::servo_step::parseReplayAppOptions;
using mcl::servo_step::printTopLevelUsage;

constexpr const char * kProgramId = "mcl_servo_step";
constexpr const char * kTitle = "Dual-arm IK — ServoStep";
constexpr double kPositionToleranceM = 1.0e-4;
constexpr double kOrientationToleranceRad = 1.0e-4;
constexpr double kJointPositionMargin = 1.0e-3;
constexpr double kNumericalRegularization = 1.0e-4;
constexpr std::array<unsigned int, 1> kMainCpuAffinity{31};

struct ServoSolveResult
{
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<mcl::ArmForwardKinematics> forward_kinematics;
  std::vector<mcl::ArmTargetError> target_errors;
  mcl::SolverDebug solver_debug;
  int iterations{0};
  bool converged{false};
  double solve_time_ms{0.0};
};

void throwIfError(const mcc::Status & status)
{
  if (!status.ok()) {
    throw std::runtime_error(status.message);
  }
}

const mcc::Pose & poseForFrame(
  const std::vector<mcc::FramePose> & poses, const std::string & frame_name)
{
  return std::find_if(
           poses.begin(), poses.end(),
           [&](const mcc::FramePose & pose) { return pose.frame_name == frame_name; })
    ->pose;
}

mcl::Pose toPose(const Eigen::Affine3d & affine)
{
  mcl::Pose pose = mcl::Pose::Identity();
  pose.matrix() = affine.matrix();
  return pose;
}

double orientationError(const mcl::Pose & target, const mcl::Pose & actual)
{
  return Eigen::AngleAxisd(target.linear() * actual.linear().transpose()).angle();
}

const char * mccSolverTitle(MccBackend backend)
{
  return backend == MccBackend::Proxqp ? "MCC/ProxQP" : "MCC/eiquadprog";
}

mcc::QpBackend mccQpBackend(MccBackend backend)
{
  if (backend == MccBackend::Proxqp) {
    return mcc::QpBackend::ProxQp;
  }
  return mcc::QpBackend::Eiquadprog;
}

class MccServoSolver
{
public:
  MccServoSolver(
    const std::string & urdf_path, double rate_hz, const mcl::R1RobotConfig & robot,
    MccBackend backend,
    const std::vector<double> & initial_positions = {},
    const std::vector<double> & initial_velocities = {})
  : robot_(robot),
    backend_(backend),
    positions_(initial_positions.empty() ? robot.default_positions : initial_positions),
    velocities_(
      initial_velocities.empty() ? std::vector<double>(positions_.size(), 0.0) : initial_velocities)
  {
    mcc::RobotModelDescription model_description;
    model_description.urdf_path = urdf_path;
    model_description.kinematics_reference_frame = robot.base_frame;
    model_description.joint_names = robot.joint_names;
    throwIfError(mcc::RobotModel::load(model_description, model_));

    mcc::KinematicsSolverConfig solver_config;
    solver_config.mode = mcc::IkSolveMode::ServoStep;
    solver_config.servo_period = 1.0 / rate_hz;
    solver_config.joint_limit_policy = mcc::KinematicsJointLimitPolicy::ExplicitRequirements;
    solver_config.qp.backend = mccQpBackend(backend_);
    solver_config.qp.regularization = kNumericalRegularization;
    solver_config.maximum_iterations = 1;
    solver_config.soft_solve_time_budget_ms = 100.0;
    solver_config.position_tolerance_m = kPositionToleranceM;
    solver_config.orientation_tolerance_rad = kOrientationToleranceRad;
    solver_config.minimum_position_improvement_m = 1.0e-8;
    solver_config.minimum_orientation_improvement_rad = 1.0e-8;

    mcc::KinematicsSolverBuilder builder;
    throwIfError(builder.configure(model_, robot.joint_names, solver_config));

    mcc::PositionTaskConfig left_position_config;
    left_position_config.name = "left-position";
    left_position_config.enforcement = mcc::HardEnforcement{};
    throwIfError(builder.addPositionTask(
      robot.left_end_effector_frame, left_position_config, left_position_task_));

    mcc::OrientationTaskConfig left_orientation_config;
    left_orientation_config.name = "left-orientation";
    left_orientation_config.enforcement = mcc::HardEnforcement{};
    throwIfError(builder.addOrientationTask(
      robot.left_end_effector_frame, left_orientation_config, left_orientation_task_));

    mcc::PositionTaskConfig right_position_config;
    right_position_config.name = "right-position";
    right_position_config.enforcement = mcc::HardEnforcement{};
    throwIfError(builder.addPositionTask(
      robot.right_end_effector_frame, right_position_config, right_position_task_));

    mcc::OrientationTaskConfig right_orientation_config;
    right_orientation_config.name = "right-orientation";
    right_orientation_config.enforcement = mcc::HardEnforcement{};
    throwIfError(builder.addOrientationTask(
      robot.right_end_effector_frame, right_orientation_config, right_orientation_task_));

    mcc::JointPositionLimitConfig joint_limit_config;
    joint_limit_config.margin = kJointPositionMargin;
    joint_limit_config.enforcement = mcc::HardEnforcement{};
    mcc::JointPositionLimitHandle joint_limits;
    throwIfError(builder.addJointPositionLimits(joint_limit_config, joint_limits));

    mcc::JointVelocityLimitConfig velocity_limit_config;
    velocity_limit_config.enforcement = mcc::HardEnforcement{};
    mcc::JointVelocityLimitHandle velocity_limits;
    throwIfError(builder.addJointVelocityLimits(velocity_limit_config, velocity_limits));

    throwIfError(builder.finalize(solver_));
  }

  const std::vector<double> & positions() const { return positions_; }
  const std::vector<double> & velocities() const { return velocities_; }

  mcl::Pose currentPose(mcl::ArmSide side)
  {
    mcc::ForwardKinematicsRequest request;
    request.state = mcl::makeRobotState(positions_, velocities_);
    request.frame_names = {mcl::frameForSide(robot_, side)};
    request.reference_frame_name = robot_.base_frame;
    mcc::ForwardKinematicsSolution solution;
    mcc::ForwardKinematicsDiagnostics diagnostics;
    throwIfError(solver_.computeForwardKinematics(request, solution, diagnostics));
    return solution.poses.at(0).pose;
  }

  ServoSolveResult solve(const std::vector<mcl::ArmTarget> & targets)
  {
    const auto & left_target = targets.at(0);
    const auto & right_target = targets.at(1);
    mcc::InverseKinematicsRequest request;
    request.reference_frame_name = robot_.base_frame;
    request.state = mcl::makeRobotState(positions_, velocities_);
    request.position_targets.push_back(
      {left_position_task_, left_target.target_pose.translation(), true});
    request.orientation_targets.push_back(
      {left_orientation_task_, left_target.target_pose.linear(), true});
    request.position_targets.push_back(
      {right_position_task_, right_target.target_pose.translation(), true});
    request.orientation_targets.push_back(
      {right_orientation_task_, right_target.target_pose.linear(), true});

    mcc::InverseKinematicsSolution solution;
    mcc::InverseKinematicsDiagnostics diagnostics;
    const auto status = solver_.solveInverseKinematics(request, solution, diagnostics);
    throwIfError(status);
    if (!mcc::isAccepted(solution.disposition)) {
      throw std::runtime_error("IK candidate rejected");
    }

    positions_ = mcl::toStdVector(solution.joint_positions);
    velocities_ = mcl::toStdVector(solution.joint_velocities);

    ServoSolveResult result;
    result.positions = positions_;
    result.velocities = velocities_;
    result.forward_kinematics = {
      {mcl::ArmSide::Left, poseForFrame(solution.solved_poses, robot_.left_end_effector_frame)},
      {mcl::ArmSide::Right, poseForFrame(solution.solved_poses, robot_.right_end_effector_frame)}};
    result.solver_debug =
      mcl::makeSolverDebug(mccSolverTitle(backend_), diagnostics, solution.disposition);
    result.iterations = diagnostics.iterations;
    result.converged = diagnostics.converged;
    result.solve_time_ms = diagnostics.solve_time_ms;

    mcl::ArmTargetError left_error;
    left_error.side = mcl::ArmSide::Left;
    mcl::ArmTargetError right_error;
    right_error.side = mcl::ArmSide::Right;
    for (const auto & error : diagnostics.position_errors) {
      if (error.handle.value == left_position_task_.value) {
        left_error.position_m = error.norm_m;
      } else if (error.handle.value == right_position_task_.value) {
        right_error.position_m = error.norm_m;
      }
    }
    for (const auto & error : diagnostics.orientation_errors) {
      if (error.handle.value == left_orientation_task_.value) {
        left_error.orientation_rad = error.norm_rad;
      } else if (error.handle.value == right_orientation_task_.value) {
        right_error.orientation_rad = error.norm_rad;
      }
    }
    result.target_errors = {left_error, right_error};
    return result;
  }

private:
  const mcl::R1RobotConfig & robot_;
  MccBackend backend_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::shared_ptr<const mcc::RobotModel> model_;
  mcc::KinematicsSolver solver_;
  mcc::PositionTaskHandle left_position_task_;
  mcc::OrientationTaskHandle left_orientation_task_;
  mcc::PositionTaskHandle right_position_task_;
  mcc::OrientationTaskHandle right_orientation_task_;
};

class PlacoServoSolver
{
public:
  PlacoServoSolver(
    const std::string & urdf_path, double rate_hz, const mcl::R1RobotConfig & robot,
    const std::vector<double> & initial_positions = {},
    const std::vector<double> & initial_velocities = {})
  : robot_config_(robot),
    positions_(initial_positions.empty() ? robot.default_positions : initial_positions),
    velocities_(
      initial_velocities.empty() ? std::vector<double>(positions_.size(), 0.0)
                                 : initial_velocities),
    robot_(
      urdf_path,
      placo::model::RobotWrapper::IGNORE_COLLISIONS | placo::model::RobotWrapper::IGNORE_GEOMETRY),
    solver_(robot_)
  {
    robot_.reset();
    const std::set<std::string> controlled_joints(
      robot.joint_names.begin(), robot.joint_names.end());
    for (const auto & joint_name : robot_.joint_names()) {
      if (controlled_joints.count(joint_name) == 0U) {
        solver_.mask_dof(joint_name);
      }
    }
    solver_.mask_fbase(true);
    solver_.problem.rewrite_equalities = false;
    solver_.dt = 1.0 / rate_hz;
    solver_.problem.regularization = kNumericalRegularization;
    solver_.enable_joint_limits(true);
    solver_.enable_velocity_limits(true);

    for (std::size_t index = 0; index < robot.joint_names.size(); ++index) {
      const auto & joint_name = robot.joint_names[index];
      const auto limits = robot_.get_joint_limits(joint_name);
      robot_.set_joint_limits(
        joint_name, limits.first + kJointPositionMargin, limits.second - kJointPositionMargin);
      robot_.set_joint(joint_name, positions_[index]);
      robot_.set_joint_velocity(joint_name, velocities_[index]);
    }
    robot_.update_kinematics();

    const auto left_pose = currentPose(mcl::ArmSide::Left);
    const auto right_pose = currentPose(mcl::ArmSide::Right);
    left_position_task_ =
      &solver_.add_position_task(robot.left_end_effector_frame, left_pose.translation());
    left_position_task_->configure("left-position", "hard", 1.0);
    left_orientation_task_ =
      &solver_.add_orientation_task(robot.left_end_effector_frame, left_pose.linear());
    left_orientation_task_->configure("left-orientation", "hard", 1.0);
    right_position_task_ =
      &solver_.add_position_task(robot.right_end_effector_frame, right_pose.translation());
    right_position_task_->configure("right-position", "hard", 1.0);
    right_orientation_task_ =
      &solver_.add_orientation_task(robot.right_end_effector_frame, right_pose.linear());
    right_orientation_task_->configure("right-orientation", "hard", 1.0);
  }

  const std::vector<double> & positions() const { return positions_; }
  const std::vector<double> & velocities() const { return velocities_; }

  mcl::Pose currentPose(mcl::ArmSide side)
  {
    return toPose(robot_.get_T_world_frame(mcl::frameForSide(robot_config_, side)));
  }

  ServoSolveResult solve(const std::vector<mcl::ArmTarget> & targets)
  {
    const auto started = std::chrono::steady_clock::now();
    const auto & left_target = targets.at(0).target_pose;
    const auto & right_target = targets.at(1).target_pose;
    left_position_task_->target_world = left_target.translation();
    left_orientation_task_->R_world_frame = left_target.linear();
    right_position_task_->target_world = right_target.translation();
    right_orientation_task_->R_world_frame = right_target.linear();

    (void)solver_.solve(true);
    robot_.update_kinematics();

    for (std::size_t index = 0; index < robot_config_.joint_names.size(); ++index) {
      positions_[index] = robot_.get_joint(robot_config_.joint_names[index]);
      velocities_[index] = robot_.get_joint_velocity(robot_config_.joint_names[index]);
    }

    const auto left_fk = currentPose(mcl::ArmSide::Left);
    const auto right_fk = currentPose(mcl::ArmSide::Right);
    mcl::ArmTargetError left_error;
    left_error.side = mcl::ArmSide::Left;
    left_error.position_m = (left_target.translation() - left_fk.translation()).norm();
    left_error.orientation_rad = orientationError(left_target, left_fk);
    mcl::ArmTargetError right_error;
    right_error.side = mcl::ArmSide::Right;
    right_error.position_m = (right_target.translation() - right_fk.translation()).norm();
    right_error.orientation_rad = orientationError(right_target, right_fk);

    const bool converged = left_error.position_m <= kPositionToleranceM &&
                           right_error.position_m <= kPositionToleranceM &&
                           left_error.orientation_rad <= kOrientationToleranceRad &&
                           right_error.orientation_rad <= kOrientationToleranceRad;
    const double solve_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();

    ServoSolveResult result;
    result.positions = positions_;
    result.velocities = velocities_;
    result.forward_kinematics = {{mcl::ArmSide::Left, left_fk}, {mcl::ArmSide::Right, right_fk}};
    result.target_errors = {left_error, right_error};
    result.iterations = 1;
    result.converged = converged;
    result.solve_time_ms = solve_time_ms;
    result.solver_debug.label = "PlaCo/eiquadprog";
    result.solver_debug.disposition = "accepted";
    result.solver_debug.joint_limit_policy = "model-position+velocity (margin 1e-3)";
    result.solver_debug.termination_reason = converged ? "converged" : "single-iteration";
    result.solver_debug.ik_iterations = result.iterations;
    result.solver_debug.converged = converged;
    result.solver_debug.ik_solve_time_ms = solve_time_ms;
    result.solver_debug.backend = "eiquadprog";
    result.solver_debug.qp_status = "solved";
    result.solver_debug.has_qp_diagnostics = false;
    return result;
  }

private:
  const mcl::R1RobotConfig & robot_config_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  placo::model::RobotWrapper robot_;
  placo::kinematics::KinematicsSolver solver_;
  placo::kinematics::PositionTask * left_position_task_{nullptr};
  placo::kinematics::OrientationTask * left_orientation_task_{nullptr};
  placo::kinematics::PositionTask * right_position_task_{nullptr};
  placo::kinematics::OrientationTask * right_orientation_task_{nullptr};
};

template <typename Solver>
int runInteractive(
  const AppOptions & app_options, const std::string & solver_id, const std::string & solver_title,
  Solver & solver)
{
  const auto & options = app_options.interactive;
  const auto & robot = mcl::r1RobotConfig();
  const auto affinity_domain = mcl::CpuAffinityDomain::capture();
  const auto affinity_binding =
    affinity_domain.bindCurrentThread(kProgramId, "main", kMainCpuAffinity);

  const auto presentation = mcl::makeArmPresentation(robot, mcl::foxgloveIkVisualizationChannels());
  const auto initial_left_fk = solver.currentPose(mcl::ArmSide::Left);
  const auto initial_right_fk = solver.currentPose(mcl::ArmSide::Right);
  mcl::TuiConsole tui(
    options.tui, options.rate_hz, std::string{kTitle} + " [" + solver_title + "]", presentation,
    {{mcl::ArmSide::Left, initial_left_fk}, {mcl::ArmSide::Right, initial_right_fk}}, true,
    options.tui_enabled);
  auto visualization_sink = mcl::createPreviewSink(options.visualization, kProgramId);

  mcl::installInteractiveSignalHandlers();
  mcl::InteractiveScheduler scheduler({options.rate_hz, options.duration_s});
  mcl::RollingPercentiles solve_time_percentiles;
  mcl::SolverRunCounters run_counters;
  std::size_t publish_count = 0;
  const std::string run_id = "interactive-preview-" + solver_id;

  mcl::IkDebugFrame latest_frame;
  latest_frame.run_id = run_id;
  latest_frame.targets = tui.command().targets;
  latest_frame.forward_kinematics = {
    {mcl::ArmSide::Left, initial_left_fk}, {mcl::ArmSide::Right, initial_right_fk}};
  latest_frame.joint_names = robot.joint_names;
  latest_frame.positions = solver.positions();
  latest_frame.velocities = solver.velocities();
  latest_frame.selected_side = mcl::parseArmSide(options.tui.side);
  latest_frame.cpu_affinities = {mcl::makeCpuAffinityDebug(affinity_binding)};

  visualization_sink->open();
  while (const auto schedule = scheduler.next()) {
    tui.poll();
    if (const auto reset_side = tui.consumeResetRequest()) {
      tui.setTargetPose(
        *reset_side, solver.currentPose(*reset_side),
        std::string{"Reset "} + mcl::armSideName(*reset_side) + " target from current FK");
    }

    const auto & command = tui.command();
    if (command.stop_requested) {
      break;
    }

    if (schedule->update_due && !command.paused) {
      ++run_counters.attempts;
      auto result = solver.solve(command.targets);
      ++run_counters.accepted;
      solve_time_percentiles.record(result.solve_time_ms);
      result.solver_debug.ik_solve_time_percentiles = solve_time_percentiles.snapshot();
      result.solver_debug.run_counters = run_counters;

      latest_frame.targets = command.targets;
      latest_frame.forward_kinematics = std::move(result.forward_kinematics);
      latest_frame.positions = std::move(result.positions);
      latest_frame.velocities = std::move(result.velocities);
      latest_frame.ik_status = "ok";
      latest_frame.iterations = result.iterations;
      latest_frame.converged = result.converged;
      latest_frame.solve_time_ms = result.solve_time_ms;
      latest_frame.solvers = {std::move(result.solver_debug)};
      latest_frame.target_errors = std::move(result.target_errors);
      latest_frame.status = "IK accepted [" + solver_id + "]";
      latest_frame.paused = command.paused;
      latest_frame.selected_side = command.selected_side;
      visualization_sink->write(mcl::makeIkRenderBatch(
        latest_frame, presentation, schedule->emit_time_ns));
      ++publish_count;
    }

    if (schedule->draw_due) {
      latest_frame.paused = tui.command().paused;
      latest_frame.selected_side = tui.command().selected_side;
      tui.render(latest_frame, publish_count, visualization_sink->status());
    }
    scheduler.sleep();
  }

  visualization_sink->flush();
  visualization_sink->close();
  return EXIT_SUCCESS;
}

std::pair<std::vector<double>, std::vector<double>> replayInitialState(
  const replay::LoadedReplay & loaded, const mcl::R1RobotConfig & robot)
{
  if (!loaded.initial_joint_state.has_value()) {
    return {robot.default_positions, std::vector<double>(robot.joint_names.size(), 0.0)};
  }
  const auto & source = *loaded.initial_joint_state;
  std::vector<double> positions(robot.joint_names.size(), 0.0);
  std::vector<double> velocities(robot.joint_names.size(), 0.0);
  for (std::size_t index = 0; index < robot.joint_names.size(); ++index) {
    const auto iterator =
      std::find(source.names.begin(), source.names.end(), robot.joint_names[index]);
    if (iterator == source.names.end()) {
      throw std::runtime_error("initial JointState is missing " + robot.joint_names[index]);
    }
    const std::size_t source_index =
      static_cast<std::size_t>(std::distance(source.names.begin(), iterator));
    positions[index] = source.positions.at(source_index);
    // Replay starts a fresh accepted-state feedback chain; recorded velocity is
    // provenance only.
    velocities[index] = 0.0;
  }
  return {std::move(positions), std::move(velocities)};
}

std::string jsonText(const Json::Value & value)
{
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "  ";
  return Json::writeString(builder, value) + "\n";
}

std::string traceVector(const std::vector<double> & values)
{
  std::ostringstream output;
  output << '"' << std::setprecision(17);
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {
      output << ';';
    }
    output << values[index];
  }
  output << '"';
  return output.str();
}

template <typename Solver>
int runReplayWithSolver(
  ReplayAppOptions options, const std::string & solver_id, const std::string & solver_title,
  Solver & solver, const replay::LoadedReplay & loaded)
{
  if (loaded.timeline.timeline.empty()) {
    throw std::runtime_error("replay timeline is empty");
  }
  if (!options.replay.output_dir_explicit) {
    const std::string run_id =
      options.replay.run_id.value_or(mcl::make_run_id(mcl::sha256_file(options.replay.input_path)));
    const std::filesystem::path output_root = options.replay.output_root.value_or(
      std::filesystem::path{"experiments/E02_dual_arm_replay_ik/runs"});
    options.replay.output_dir = output_root / run_id;
  }
  replay::createOutputDirectory(options.replay.output_dir);

  const auto & robot = mcl::r1RobotConfig();
  const auto & first = loaded.timeline.timeline.at(0);
  std::vector<mcl::ArmTarget> targets{
    {mcl::ArmSide::Left, first.value.left.pose * robot.left_tcp_offset.inverse()},
    {mcl::ArmSide::Right, first.value.right.pose * robot.right_tcp_offset.inverse()}};
  const auto presentation = mcl::makeArmPresentation(robot, mcl::foxgloveIkVisualizationChannels());
  mcl::TuiConsole tui(
    {}, options.rate_hz, std::string{kTitle} + " Replay [" + solver_title + "]", presentation,
    targets, true, options.replay.ui_mode == "tui", mcl::TuiControlMode::Replay);
  tui.setMotionInputEnabled(false, "Replay motion editing is disabled");

  mcl::PreviewSinkOptions sink_options;
  sink_options.host = options.replay.visualization_host;
  sink_options.port = options.replay.visualization_port;
  sink_options.mcap_path = options.replay.visualization_mcap_path;
  auto visualization_sink = mcl::createPreviewSink(sink_options, kProgramId);
  visualization_sink->open();
  mcl::installInteractiveSignalHandlers();

  replay::ReplayExecutionMetadata execution;
  execution.app = kProgramId;
  execution.topology = "ordinary-servo-step";
  execution.solver = solver_id;
  execution.backend = solver_id == "mcc"
                        ? (options.backend == MccBackend::Proxqp ? "proxqp" : "eiquadprog")
                        : "eiquadprog";
  execution.rate_hz = options.rate_hz;
  execution.consumed_frame_count = 1U;

  std::ostringstream trace;
  trace << "attempt,source_revision,original_logical_timestamp_ns,source_time_"
           "from_start_ns,"
           "projected_timestamp_ns,left_header_stamp_ns,left_log_time_ns,left_"
           "publish_time_ns,"
           "right_header_stamp_ns,right_log_time_ns,right_publish_time_ns,"
           "accepted,solver_status,"
           "solve_time_ms,maximum_hard_violation,positions,velocities\n";
  const std::int64_t solver_period_ns =
    static_cast<std::int64_t>(std::llround(1.0e9 / options.rate_hz));
  const auto run_start = std::chrono::steady_clock::now();
  std::int64_t solver_time_ns = 0;
  std::int64_t timeline_time_ns = 0;
  std::size_t source_index = 0;
  std::size_t attempt = 0;
  std::size_t publish_count = 0;
  std::string replay_state{"running"};

  try {
    while (true) {
      if (options.replay.execution_mode == mcl::data::ExecutionMode::Realtime) {
        const auto deadline = run_start + std::chrono::nanoseconds(solver_time_ns);
        std::this_thread::sleep_until(deadline);
        if (std::chrono::steady_clock::now() > deadline) {
          ++execution.deadline_miss_count;
        }
      }
      tui.poll();
      if (tui.command().stop_requested) {
        replay_state = "stopped";
        break;
      }

      const bool single_step = tui.consumeSingleStepRequest();
      if (!tui.command().paused || single_step) {
        if (single_step) {
          if (source_index + 1U < loaded.timeline.timeline.size()) {
            timeline_time_ns = loaded.timeline.timeline.at(source_index + 1U).projected_time_ns;
          }
        } else {
          timeline_time_ns +=
            attempt == 0U
              ? 0
              : static_cast<std::int64_t>(std::llround(
                  static_cast<double>(solver_period_ns) * options.replay.playback_rate));
        }
        std::size_t next_index = source_index;
        while (next_index + 1U < loaded.timeline.timeline.size() &&
               loaded.timeline.timeline.at(next_index + 1U).projected_time_ns <= timeline_time_ns) {
          ++next_index;
        }
        if (next_index > source_index) {
          execution.dropped_frame_count += next_index - source_index - 1U;
          execution.consumed_frame_count += 1U;
          source_index = next_index;
          const auto & source = loaded.timeline.timeline.at(source_index);
          targets[0].target_pose = source.value.left.pose * robot.left_tcp_offset.inverse();
          targets[1].target_pose = source.value.right.pose * robot.right_tcp_offset.inverse();
          tui.setTargetPose(mcl::ArmSide::Left, targets[0].target_pose, "Replay goal advanced");
          tui.setTargetPose(mcl::ArmSide::Right, targets[1].target_pose, "Replay goal advanced");
        }
      }

      const auto result = solver.solve(targets);
      ++attempt;
      ++execution.accepted_count;
      const auto & source = loaded.timeline.timeline.at(source_index);
      trace << attempt << ',' << source.sequence << ',' << source.original_logical_time_ns << ','
            << source.source_time_from_start_ns << ',' << source.projected_time_ns << ','
            << replay::optionalTimestamp(source.value.left.time.header_stamp_ns) << ','
            << replay::optionalTimestamp(source.value.left.time.log_time_ns) << ','
            << replay::optionalTimestamp(source.value.left.time.publish_time_ns) << ','
            << replay::optionalTimestamp(source.value.right.time.header_stamp_ns) << ','
            << replay::optionalTimestamp(source.value.right.time.log_time_ns) << ','
            << replay::optionalTimestamp(source.value.right.time.publish_time_ns) << ",true,ok,"
            << result.solve_time_ms << ',' << result.solver_debug.maximum_hard_violation << ','
            << traceVector(result.positions) << ',' << traceVector(result.velocities) << '\n';

      mcl::IkDebugFrame frame;
      frame.run_id = options.replay.output_dir.filename().string();
      frame.targets = targets;
      frame.forward_kinematics = result.forward_kinematics;
      frame.joint_names = robot.joint_names;
      frame.positions = result.positions;
      frame.velocities = result.velocities;
      frame.solvers = {result.solver_debug};
      frame.target_errors = result.target_errors;
      frame.iterations = result.iterations;
      frame.converged = result.converged;
      frame.solve_time_ms = result.solve_time_ms;
      frame.ik_status = "replay accepted";
      frame.status = "Replay source revision=" + std::to_string(source.sequence) +
                     " dropped=" + std::to_string(execution.dropped_frame_count);
      frame.paused = tui.command().paused;
      visualization_sink->write(mcl::makeIkRenderBatch(
        frame, presentation,
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count()));
      tui.render(frame, publish_count, visualization_sink->status());
      ++publish_count;

      if (source_index + 1U == loaded.timeline.timeline.size() && !tui.command().paused) {
        replay_state = "succeeded";
        break;
      }
      solver_time_ns += solver_period_ns;
    }
  } catch (const std::exception & error) {
    const auto & source = loaded.timeline.timeline.at(source_index);
    trace << attempt + 1U << ',' << source.sequence << ',' << source.original_logical_time_ns << ','
          << source.source_time_from_start_ns << ',' << source.projected_time_ns << ','
          << replay::optionalTimestamp(source.value.left.time.header_stamp_ns) << ','
          << replay::optionalTimestamp(source.value.left.time.log_time_ns) << ','
          << replay::optionalTimestamp(source.value.left.time.publish_time_ns) << ','
          << replay::optionalTimestamp(source.value.right.time.header_stamp_ns) << ','
          << replay::optionalTimestamp(source.value.right.time.log_time_ns) << ','
          << replay::optionalTimestamp(source.value.right.time.publish_time_ns) << ",false,"
          << replay::csvEscape(error.what()) << ",,," << traceVector(solver.positions()) << ','
          << traceVector(solver.velocities()) << '\n';
    ++execution.rejected_count;
    replay_state = "failed";
    const auto trace_path = options.replay.output_dir / "trace.csv";
    replay::writeTextFile(trace_path, trace.str());
    replay::writeTextFile(
      options.replay.output_dir / "status.json",
      jsonText(replay::makeReplayStatus(loaded, execution, replay_state, error.what())));
    const auto manifest =
      replay::makeReplayManifest(options.replay, loaded, execution, mcl::sha256_file(trace_path));
    replay::writeTextFile(options.replay.output_dir / "manifest.json", jsonText(manifest));
    visualization_sink->close();
    throw;
  }

  const auto trace_path = options.replay.output_dir / "trace.csv";
  replay::writeTextFile(trace_path, trace.str());
  replay::writeTextFile(
    options.replay.output_dir / "status.json",
    jsonText(replay::makeReplayStatus(loaded, execution, replay_state)));
  const auto manifest =
    replay::makeReplayManifest(options.replay, loaded, execution, mcl::sha256_file(trace_path));
  replay::writeTextFile(options.replay.output_dir / "manifest.json", jsonText(manifest));
  visualization_sink->flush();
  visualization_sink->close();
  return EXIT_SUCCESS;
}

int runReplay(int argc, char ** argv)
{
  auto options = parseReplayAppOptions(argc, argv);
  const auto loaded = replay::loadReplay(options.replay);
  const auto & robot = mcl::r1RobotConfig();
  const auto [initial_positions, initial_velocities] = replayInitialState(loaded, robot);
  if (options.solver == SolverKind::Mcc) {
    const std::string solver_title = mccSolverTitle(options.backend);
    MccServoSolver solver(
      options.replay.urdf_path.string(), options.rate_hz, robot, options.backend,
      initial_positions, initial_velocities);
    return runReplayWithSolver(std::move(options), "mcc", solver_title, solver, loaded);
  }
  PlacoServoSolver solver(
    options.replay.urdf_path.string(), options.rate_hz, robot, initial_positions,
    initial_velocities);
  return runReplayWithSolver(std::move(options), "placo", "PlaCo/eiquadprog", solver, loaded);
}

int runTeleop(int argc, char ** argv)
{
  const auto app_options = parseAppOptions(argc, argv);
  const auto & robot = mcl::r1RobotConfig();
  if (app_options.solver == SolverKind::Mcc) {
    MccServoSolver solver(
      app_options.interactive.urdf_path, app_options.interactive.rate_hz, robot,
      app_options.backend);
    return runInteractive(app_options, "mcc", mccSolverTitle(app_options.backend), solver);
  }
  PlacoServoSolver solver(
    app_options.interactive.urdf_path, app_options.interactive.rate_hz, robot);
  return runInteractive(app_options, "placo", "PlaCo/eiquadprog", solver);
}

int run(int argc, char ** argv)
{
  if (argc < 2 || std::string{argv[1]} == "--help" || std::string{argv[1]} == "-h") {
    printTopLevelUsage(argv[0]);
    return EXIT_SUCCESS;
  }
  const std::string mode{argv[1]};
  if (mode == "teleop") {
    return runTeleop(argc - 1, argv + 1);
  }
  if (mode == "replay") {
    return runReplay(argc - 1, argv + 1);
  }
  throw std::runtime_error("expected subcommand 'teleop' or 'replay'");
}

}  // namespace

#ifndef MCL_SERVO_STEP_TESTING
int main(int argc, char ** argv)
{
  try {
    return run(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << kProgramId << ": " << error.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << kProgramId << ": non-standard exception\n";
    return EXIT_FAILURE;
  }
}
#endif
