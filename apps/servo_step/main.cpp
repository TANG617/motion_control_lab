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
#include "components/app_helpers/app_helpers.hpp"
#include "components/robot/r1/r1_robot_config.hpp"
#include "components/replay/replay_source.hpp"
#include "components/scheduler/rolling_percentiles.hpp"
#include "components/scheduler/single_rate_scheduler.hpp"
#include "components/teleop/keyboard/keyboard_target_source.hpp"
#include "components/tui/tui_renderer.hpp"
#include "components/visualization/preview_projection.hpp"
#include "components/visualization/preview_transport.hpp"
#include "contracts/presentation/ik_app_snapshot.hpp"
#include "diagnostics_projection.hpp"
#include "motion_control_lab/run_artifacts.hpp"
#include "motion_control_lab/sha256.hpp"
#include "placo/kinematics/kinematics_solver.h"
#include "placo/model/robot_wrapper.h"
#include "components/tui/standard_ik_tui.hpp"

namespace
{

namespace mcc = motion_control::core;
namespace mcl = motion_control_lab;
namespace replay = motion_control_lab::replay;

mcc::RobotState makeRobotState(
  const std::vector<double> & positions, const std::vector<double> & velocities)
{
  mcc::RobotState state;
  state.joint_positions = mcl::toEigen(positions);
  state.joint_velocities = mcl::toEigen(velocities);
  return state;
}

using mcl::servo_step::AppOptions;
using mcl::servo_step::MccBackend;
using mcl::servo_step::ReplayAppOptions;
using mcl::servo_step::SolverKind;
using mcl::servo_step::parseAppOptions;
using mcl::servo_step::parseReplayAppOptions;
using mcl::servo_step::printTopLevelUsage;

constexpr const char * kProgramId = "mcl_servo_step";
constexpr const char * kTitle = "Dual-arm IK — ServoStep";
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
    MccBackend backend, const mcl::servo_step::AlgorithmOptions & algorithm,
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
    solver_config.qp.regularization = algorithm.regularization;
    solver_config.maximum_iterations = 1;
    solver_config.soft_solve_time_budget_ms = 100.0;
    solver_config.position_tolerance_m = algorithm.position_tolerance_m;
    solver_config.orientation_tolerance_rad = algorithm.orientation_tolerance_rad;
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
    joint_limit_config.margin = algorithm.joint_position_margin_rad;
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
    request.state = makeRobotState(positions_, velocities_);
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
    request.state = makeRobotState(positions_, velocities_);
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
    const mcl::servo_step::AlgorithmOptions & algorithm,
    const std::vector<double> & initial_positions = {},
    const std::vector<double> & initial_velocities = {})
  : robot_config_(robot),
    algorithm_(algorithm),
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
    solver_.problem.regularization = algorithm_.regularization;
    solver_.enable_joint_limits(true);
    solver_.enable_velocity_limits(true);

    for (std::size_t index = 0; index < robot.joint_names.size(); ++index) {
      const auto & joint_name = robot.joint_names[index];
      const auto limits = robot_.get_joint_limits(joint_name);
      robot_.set_joint_limits(
        joint_name, limits.first + algorithm_.joint_position_margin_rad,
        limits.second - algorithm_.joint_position_margin_rad);
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

    const bool converged = left_error.position_m <= algorithm_.position_tolerance_m &&
                           right_error.position_m <= algorithm_.position_tolerance_m &&
                           left_error.orientation_rad <= algorithm_.orientation_tolerance_rad &&
                           right_error.orientation_rad <= algorithm_.orientation_tolerance_rad;
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
  mcl::servo_step::AlgorithmOptions algorithm_;
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
  const std::string title = std::string{kTitle} + " [" + solver_title + "]";
  mcl::TerminalFrontend terminal({true, options.tui_enabled});
  mcl::KeyboardTargetSource input(
    terminal, mcl::KeyboardSourceMode::Teleop, options.tui,
    {{mcl::ArmSide::Left, initial_left_fk}, {mcl::ArmSide::Right, initial_right_fk}}, true);
  mcl::TuiRenderer tui(options.tui_enabled);
  auto visualization_sink = mcl::createPreviewSink(options.visualization, kProgramId);

  mcl::installRuntimeSignalHandlers();
  mcl::SingleRateScheduler scheduler({options.rate_hz, options.duration_s});
  mcl::RollingPercentiles solve_time_percentiles;
  mcl::SolverRunCounters run_counters;
  std::size_t publish_count = 0;
  const std::string run_id = "interactive-preview-" + solver_id;

  mcl::IkDebugFrame latest_frame;
  latest_frame.run_id = run_id;
  latest_frame.targets = input.targets();
  latest_frame.forward_kinematics = {
    {mcl::ArmSide::Left, initial_left_fk}, {mcl::ArmSide::Right, initial_right_fk}};
  latest_frame.joint_names = robot.joint_names;
  latest_frame.positions = solver.positions();
  latest_frame.velocities = solver.velocities();
  latest_frame.selected_side = mcl::parseArmSide(options.tui.side);
  latest_frame.cpu_affinities = {mcl::makeCpuAffinityDebug(affinity_binding)};

  visualization_sink->open();
  while (const auto schedule = scheduler.next()) {
    const auto input_update = input.poll(schedule->dt);
    for (const auto & event : input_update.navigation) {
      tui.handleNavigation(event);
    }
    if (const auto reset_side = input.consumeResetRequest()) {
      input.setTargetPose(
        *reset_side, solver.currentPose(*reset_side),
        std::string{"Reset "} + mcl::armSideName(*reset_side) + " target from current FK");
    }

    if (input.stopRequested()) {
      break;
    }

    if (schedule->update_due && !input.paused()) {
      ++run_counters.attempts;
      auto result = solver.solve(input.targets());
      ++run_counters.accepted;
      solve_time_percentiles.record(result.solve_time_ms);
      result.solver_debug.ik_solve_time_percentiles = solve_time_percentiles.snapshot();
      result.solver_debug.run_counters = run_counters;

      latest_frame.targets = input.targets();
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
      latest_frame.paused = input.paused();
      latest_frame.selected_side = input.selectedSide();
      visualization_sink->write(mcl::makeIkRenderBatch(
        latest_frame, presentation, schedule->emit_time_ns));
      ++publish_count;
    }

    if (schedule->draw_due) {
      latest_frame.paused = input.paused();
      latest_frame.selected_side = input.selectedSide();
      tui.render(mcl::makeStandardIkTuiDocument(
        latest_frame, presentation, publish_count, visualization_sink->status(), title,
        input.status()));
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
  replay::ReplaySource replay_source(
    loaded, options.replay.execution_mode, options.replay.playback_rate);
  const auto & first = replay_source.sourceFrame();
  std::vector<mcl::ArmTarget> targets{
    {mcl::ArmSide::Left, first.value.left.pose * robot.left_tcp_offset.inverse()},
    {mcl::ArmSide::Right, first.value.right.pose * robot.right_tcp_offset.inverse()}};
  const auto presentation = mcl::makeArmPresentation(robot, mcl::foxgloveIkVisualizationChannels());
  const bool tui_enabled = options.replay.ui_mode == "tui";
  const std::string title = std::string{kTitle} + " Replay [" + solver_title + "]";
  mcl::TerminalFrontend terminal({options.replay.terminal_input_enabled, tui_enabled});
  mcl::KeyRouter key_router;
  mcl::KeyboardTeleop keyboard(mcl::KeyboardSourceMode::Replay);
  mcl::TuiRenderer tui(tui_enabled);

  mcl::PreviewSinkOptions sink_options;
  sink_options.enabled = options.replay.visualization_enabled;
  sink_options.host = options.replay.visualization_host;
  sink_options.port = options.replay.visualization_port;
  sink_options.mcap_path = options.replay.visualization_mcap_path;
  auto visualization_sink = mcl::createPreviewSink(sink_options, kProgramId);
  visualization_sink->open();
  mcl::installRuntimeSignalHandlers();

  replay::ReplayExecutionMetadata execution;
  execution.app = kProgramId;
  execution.topology = "ordinary-servo-step";
  execution.solver = solver_id;
  execution.backend = solver_id == "mcc"
                        ? (options.backend == MccBackend::Proxqp ? "proxqp" : "eiquadprog")
                        : "eiquadprog";
  execution.rate_hz = options.rate_hz;
  execution.consumed_frame_count = 1U;
  execution.resolved_config = {
    {"regularization", std::to_string(options.algorithm.regularization)},
    {"position_tolerance_m", std::to_string(options.algorithm.position_tolerance_m)},
    {"orientation_tolerance_rad", std::to_string(options.algorithm.orientation_tolerance_rad)},
    {"joint_position_margin_rad", std::to_string(options.algorithm.joint_position_margin_rad)},
  };

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
  std::size_t attempt = 0;
  std::size_t publish_count = 0;
  std::string replay_state{"running"};

  try {
    while (true) {
      for (const auto & event : terminal.poll()) {
        if (key_router.route(event) == mcl::KeyRoute::Navigation) {
          tui.handleNavigation(event);
        } else {
          const auto action = keyboard.handle(event);
          if (action.source_control.has_value()) {
            replay_source.applyControl(*action.source_control);
          }
        }
      }
      if (replay_source.stopped()) {
        replay_state = "stopped";
        break;
      }

      if (attempt != 0U) {
        replay_source.advance(solver_period_ns);
      }
      replay_source.waitForCurrentFrame();
      execution.deadline_miss_count = replay_source.deadlineMissCount();
      execution.dropped_frame_count = replay_source.droppedFrameCount();
      execution.consumed_frame_count = replay_source.consumedFrameCount();
      const auto & replay_frame = replay_source.frame();
      targets[0].target_pose = replay_frame.targets[0].target_pose * robot.left_tcp_offset.inverse();
      targets[1].target_pose = replay_frame.targets[1].target_pose * robot.right_tcp_offset.inverse();

      const auto result = solver.solve(targets);
      ++attempt;
      ++execution.accepted_count;
      const auto & source = replay_source.sourceFrame();
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
      frame.paused = replay_source.paused();
      visualization_sink->write(mcl::makeIkRenderBatch(
        frame, presentation,
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count()));
      tui.render(mcl::makeStandardIkTuiDocument(
        frame, presentation, publish_count, visualization_sink->status(), title,
        replay_source.status().detail));
      ++publish_count;

      replay_source.markFrameProcessed();
      if (replay_source.endOfStream()) {
        replay_state = "succeeded";
        break;
      }
    }
  } catch (const std::exception & error) {
    const auto & source = replay_source.sourceFrame();
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

int runReplay(int argc, char ** argv, int process_argc, char ** process_argv)
{
  auto options = parseReplayAppOptions(argc, argv);
  options.replay.original_argv.assign(process_argv, process_argv + process_argc);
  const auto loaded = replay::loadReplay(options.replay);
  const auto & robot = mcl::r1RobotConfig();
  const auto [initial_positions, initial_velocities] = replayInitialState(loaded, robot);
  if (options.solver == SolverKind::Mcc) {
    const std::string solver_title = mccSolverTitle(options.backend);
    MccServoSolver solver(
      options.replay.urdf_path.string(), options.rate_hz, robot, options.backend,
      options.algorithm,
      initial_positions, initial_velocities);
    return runReplayWithSolver(std::move(options), "mcc", solver_title, solver, loaded);
  }
  PlacoServoSolver solver(
    options.replay.urdf_path.string(), options.rate_hz, robot, options.algorithm, initial_positions,
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
      app_options.backend, app_options.algorithm);
    return runInteractive(app_options, "mcc", mccSolverTitle(app_options.backend), solver);
  }
  PlacoServoSolver solver(
    app_options.interactive.urdf_path, app_options.interactive.rate_hz, robot,
    app_options.algorithm);
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
    return runReplay(argc - 1, argv + 1, argc, argv);
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
