#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <motion_control_core/motion_control_core.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "app_options.hpp"
#include "components/app_helpers/app_helpers.hpp"
#include "components/robot/r1/r1_robot_config.hpp"
#include "components/scheduler/rolling_percentiles.hpp"
#include "components/scheduler/single_rate_scheduler.hpp"
#include "components/teleop/keyboard/keyboard_target_source.hpp"
#include "components/tui/tui_renderer.hpp"
#include "components/visualization/preview_projection.hpp"
#include "components/visualization/preview_transport.hpp"
#include "contracts/presentation/ik_app_snapshot.hpp"
#include "diagnostics_projection.hpp"
#include "placo/kinematics/kinematics_solver.h"
#include "placo/model/robot_wrapper.h"
#include "components/tui/standard_ik_tui.hpp"

namespace
{

namespace mcc = motion_control::core;
namespace mcl = motion_control_lab;

mcc::RobotState makeRobotState(
  const std::vector<double> & positions, const std::vector<double> & velocities)
{
  mcc::RobotState state;
  state.joint_positions = mcl::toEigen(positions);
  state.joint_velocities = mcl::toEigen(velocities);
  return state;
}

using mcl::target_solve::AppOptions;
using mcl::target_solve::MccBackend;
using mcl::target_solve::SolverKind;
using mcl::target_solve::parseAppOptions;

constexpr const char * kProgramId = "mcl_target_solve";
constexpr const char * kTitle = "Dual-arm IK — TargetSolve";
constexpr std::array<unsigned int, 1> kMainCpuAffinity{8};

struct TargetSolveResult
{
  bool accepted{false};
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<mcl::ArmForwardKinematics> forward_kinematics;
  std::vector<mcl::ArmTargetError> target_errors;
  mcl::SolverDebug solver_debug;
  std::string ik_status;
  std::string status;
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

double elapsedMilliseconds(const std::chrono::steady_clock::time_point & started)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
    .count();
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

class MccTargetSolver
{
public:
  MccTargetSolver(
    const std::string & urdf_path, const mcl::R1RobotConfig & robot, MccBackend backend,
    const mcl::target_solve::AlgorithmOptions & algorithm)
  : robot_(robot),
    backend_(backend),
    positions_(robot.default_positions),
    velocities_(positions_.size(), 0.0)
  {
    mcc::RobotModelDescription model_description;
    model_description.urdf_path = urdf_path;
    model_description.kinematics_reference_frame = robot.base_frame;
    model_description.joint_names = robot.joint_names;
    throwIfError(mcc::RobotModel::load(model_description, model_));

    mcc::KinematicsSolverConfig solver_config;
    solver_config.mode = mcc::IkSolveMode::TargetSolve;
    solver_config.servo_period = 0.0;
    solver_config.joint_limit_policy = mcc::KinematicsJointLimitPolicy::ExplicitRequirements;
    solver_config.qp.backend = mccQpBackend(backend_);
    solver_config.qp.regularization = algorithm.regularization;
    solver_config.maximum_iterations = algorithm.maximum_iterations;
    solver_config.soft_solve_time_budget_ms = algorithm.soft_solve_time_budget_ms;
    solver_config.position_tolerance_m = algorithm.position_tolerance_m;
    solver_config.orientation_tolerance_rad = algorithm.orientation_tolerance_rad;
    solver_config.minimum_position_improvement_m = algorithm.minimum_position_improvement_m;
    solver_config.minimum_orientation_improvement_rad = algorithm.minimum_orientation_improvement_rad;
    if (backend_ == MccBackend::Proxqp) {
      solver_config.qp.proxqp.absolute_tolerance = algorithm.proxqp_absolute_tolerance;
    }

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

    mcc::PostureTaskConfig posture_config;
    posture_config.name = "initial-posture";
    posture_config.enforcement =
      mcc::squaredL2Penalty(algorithm.posture_weight, static_cast<int>(robot.joint_names.size()));
    posture_config.reference_positions = mcl::toEigen(positions_);
    posture_config.role = mcc::PostureTaskRole::Regularization;
    mcc::PostureTaskHandle posture_task;
    throwIfError(builder.addPostureTask(posture_config, posture_task));

    mcc::JointPositionLimitConfig joint_limit_config;
    joint_limit_config.margin = algorithm.joint_position_margin_rad;
    joint_limit_config.enforcement = mcc::HardEnforcement{};
    mcc::JointPositionLimitHandle joint_limits;
    throwIfError(builder.addJointPositionLimits(joint_limit_config, joint_limits));

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

  TargetSolveResult solve(const std::vector<mcl::ArmTarget> & targets)
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

    TargetSolveResult result;
    result.iterations = diagnostics.iterations;
    result.solve_time_ms = diagnostics.solve_time_ms;
    result.solver_debug =
      mcl::makeSolverDebug(mccSolverTitle(backend_), diagnostics, solution.disposition);
    if (status.code == mcc::StatusCode::Infeasible) {
      result.accepted = false;
      result.ik_status = "rejected: infeasible";
      result.status = status.message;
      return result;
    }

    throwIfError(status);
    if (!mcc::isAccepted(solution.disposition)) {
      throw std::runtime_error("IK candidate rejected");
    }

    positions_ = mcl::toStdVector(solution.joint_positions);
    result.accepted = true;
    result.positions = positions_;
    result.velocities = velocities_;
    result.forward_kinematics = {
      {mcl::ArmSide::Left, poseForFrame(solution.solved_poses, robot_.left_end_effector_frame)},
      {mcl::ArmSide::Right, poseForFrame(solution.solved_poses, robot_.right_end_effector_frame)}};
    result.ik_status = "ok";
    result.status = "IK accepted [mcc]";
    result.converged = diagnostics.converged;

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

class PlacoTargetSolver
{
public:
  PlacoTargetSolver(
    const std::string & urdf_path, const mcl::R1RobotConfig & robot,
    const mcl::target_solve::AlgorithmOptions & algorithm)
  : algorithm_(algorithm),
    robot_config_(robot),
    positions_(robot.default_positions),
    velocities_(positions_.size(), 0.0),
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
    solver_.dt = 0.0;
    solver_.problem.regularization = algorithm_.regularization;
    solver_.enable_joint_limits(true);
    solver_.enable_velocity_limits(false);

    for (std::size_t index = 0; index < robot.joint_names.size(); ++index) {
      const auto & joint_name = robot.joint_names[index];
      const auto limits = robot_.get_joint_limits(joint_name);
      robot_.set_joint_limits(
        joint_name, limits.first + algorithm_.joint_position_margin_rad,
        limits.second - algorithm_.joint_position_margin_rad);
      robot_.set_joint(joint_name, positions_[index]);
      robot_.set_joint_velocity(joint_name, 0.0);
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

    posture_task_ = &solver_.add_joints_task();
    for (std::size_t index = 0; index < robot.joint_names.size(); ++index) {
      posture_task_->set_joint(robot.joint_names[index], positions_[index]);
    }
    posture_task_->configure("initial-posture", "soft", algorithm_.posture_weight);
  }

  const std::vector<double> & positions() const { return positions_; }
  const std::vector<double> & velocities() const { return velocities_; }

  mcl::Pose currentPose(mcl::ArmSide side)
  {
    return toPose(robot_.get_T_world_frame(mcl::frameForSide(robot_config_, side)));
  }

  TargetSolveResult solve(const std::vector<mcl::ArmTarget> & targets)
  {
    const auto started = std::chrono::steady_clock::now();
    const auto & left_target = targets.at(0).target_pose;
    const auto & right_target = targets.at(1).target_pose;
    left_position_task_->target_world = left_target.translation();
    left_orientation_task_->R_world_frame = left_target.linear();
    right_position_task_->target_world = right_target.translation();
    right_orientation_task_->R_world_frame = right_target.linear();

    double previous_position_error = std::numeric_limits<double>::infinity();
    double previous_orientation_error = std::numeric_limits<double>::infinity();
    mcl::ArmTargetError left_error;
    left_error.side = mcl::ArmSide::Left;
    mcl::ArmTargetError right_error;
    right_error.side = mcl::ArmSide::Right;
    mcl::Pose left_fk = currentPose(mcl::ArmSide::Left);
    mcl::Pose right_fk = currentPose(mcl::ArmSide::Right);
    std::string termination_reason = "iteration-budget";
    int iterations = 0;
    bool converged = false;

    for (int iteration = 0; iteration < algorithm_.maximum_iterations; ++iteration) {
      (void)solver_.solve(true);
      robot_.update_kinematics();
      ++iterations;

      left_fk = currentPose(mcl::ArmSide::Left);
      right_fk = currentPose(mcl::ArmSide::Right);
      left_error.position_m = (left_target.translation() - left_fk.translation()).norm();
      left_error.orientation_rad = orientationError(left_target, left_fk);
      right_error.position_m = (right_target.translation() - right_fk.translation()).norm();
      right_error.orientation_rad = orientationError(right_target, right_fk);

      const double maximum_position_error = std::max(left_error.position_m, right_error.position_m);
      const double maximum_orientation_error =
        std::max(left_error.orientation_rad, right_error.orientation_rad);
      converged = maximum_position_error <= algorithm_.position_tolerance_m &&
                  maximum_orientation_error <= algorithm_.orientation_tolerance_rad;
      if (converged) {
        termination_reason = "converged";
        break;
      }
      if (iteration > 0) {
        const double position_improvement = previous_position_error - maximum_position_error;
        const double orientation_improvement =
          previous_orientation_error - maximum_orientation_error;
        if (
          std::abs(position_improvement) < algorithm_.minimum_position_improvement_m &&
          std::abs(orientation_improvement) < algorithm_.minimum_orientation_improvement_rad) {
          termination_reason = "no-progress";
          break;
        }
      }
      previous_position_error = maximum_position_error;
      previous_orientation_error = maximum_orientation_error;
      if (elapsedMilliseconds(started) >= algorithm_.soft_solve_time_budget_ms) {
        termination_reason = "soft-time-budget";
        break;
      }
    }

    for (std::size_t index = 0; index < robot_config_.joint_names.size(); ++index) {
      positions_[index] = robot_.get_joint(robot_config_.joint_names[index]);
      velocities_[index] = 0.0;
    }
    const double solve_time_ms = elapsedMilliseconds(started);

    TargetSolveResult result;
    result.accepted = true;
    result.positions = positions_;
    result.velocities = velocities_;
    result.forward_kinematics = {{mcl::ArmSide::Left, left_fk}, {mcl::ArmSide::Right, right_fk}};
    result.target_errors = {left_error, right_error};
    result.ik_status = "ok";
    result.status = "IK accepted [placo]";
    result.iterations = iterations;
    result.converged = converged;
    result.solve_time_ms = solve_time_ms;
    result.solver_debug.label = "PlaCo/eiquadprog";
    result.solver_debug.disposition = "accepted";
    result.solver_debug.joint_limit_policy = "model-position (margin 1e-3)";
    result.solver_debug.termination_reason = termination_reason;
    result.solver_debug.ik_iterations = iterations;
    result.solver_debug.converged = converged;
    result.solver_debug.ik_solve_time_ms = solve_time_ms;
    result.solver_debug.backend = "eiquadprog";
    result.solver_debug.qp_status = "solved";
    result.solver_debug.has_qp_diagnostics = false;
    return result;
  }

private:
  mcl::target_solve::AlgorithmOptions algorithm_;
  const mcl::R1RobotConfig & robot_config_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  placo::model::RobotWrapper robot_;
  placo::kinematics::KinematicsSolver solver_;
  placo::kinematics::PositionTask * left_position_task_{nullptr};
  placo::kinematics::OrientationTask * left_orientation_task_{nullptr};
  placo::kinematics::PositionTask * right_position_task_{nullptr};
  placo::kinematics::OrientationTask * right_orientation_task_{nullptr};
  placo::kinematics::JointsTask * posture_task_{nullptr};
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
      if (result.accepted) {
        ++run_counters.accepted;
      } else {
        ++run_counters.rejected;
      }
      solve_time_percentiles.record(result.solve_time_ms);
      result.solver_debug.ik_solve_time_percentiles = solve_time_percentiles.snapshot();
      result.solver_debug.run_counters = run_counters;

      latest_frame.targets = input.targets();
      latest_frame.ik_status = result.ik_status;
      latest_frame.iterations = result.iterations;
      latest_frame.converged = result.converged;
      latest_frame.solve_time_ms = result.solve_time_ms;
      latest_frame.solvers = {std::move(result.solver_debug)};
      latest_frame.target_errors = std::move(result.target_errors);
      latest_frame.status = result.status;
      if (result.accepted) {
        latest_frame.forward_kinematics = std::move(result.forward_kinematics);
        latest_frame.positions = std::move(result.positions);
        latest_frame.velocities = std::move(result.velocities);
      }
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

int run(int argc, char ** argv)
{
  const auto app_options = parseAppOptions(argc, argv);
  const auto & robot = mcl::r1RobotConfig();
  if (app_options.solver == SolverKind::Mcc) {
    MccTargetSolver solver(
      app_options.interactive.urdf_path, robot, app_options.backend, app_options.algorithm);
    return runInteractive(app_options, "mcc", mccSolverTitle(app_options.backend), solver);
  }
  PlacoTargetSolver solver(app_options.interactive.urdf_path, robot, app_options.algorithm);
  return runInteractive(app_options, "placo", "PlaCo/eiquadprog", solver);
}

}  // namespace

#ifndef MCL_TARGET_SOLVE_TESTING
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
