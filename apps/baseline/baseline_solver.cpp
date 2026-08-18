#include "baseline_solver.hpp"

#include <pinocchio/spatial/explog.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace motion_control_lab::baseline
{
namespace
{

constexpr int kRobotWrapperFlags =
    placo::model::RobotWrapper::IGNORE_COLLISIONS | placo::model::RobotWrapper::IGNORE_GEOMETRY;

class AbsoluteJointLimitsConstraint : public placo::kinematics::Constraint
{
public:
  explicit AbsoluteJointLimitsConstraint(std::vector<std::string> joint_names)
      : joint_names_(std::move(joint_names))
  {
  }

  void add_constraint(placo::problem::Problem & problem) override
  {
    placo::problem::Expression expression;
    expression.A = Eigen::MatrixXd::Zero(joint_names_.size(), solver->N);
    expression.b = Eigen::VectorXd::Zero(joint_names_.size());
    Eigen::VectorXd lower(joint_names_.size());
    Eigen::VectorXd upper(joint_names_.size());

    for (std::size_t index = 0; index < joint_names_.size(); ++index) {
      const auto & joint_name = joint_names_[index];
      const auto [joint_lower, joint_upper] = solver->robot.get_joint_limits(joint_name);
      expression.A(static_cast<int>(index), solver->robot.get_joint_v_offset(joint_name)) = 1.0;
      expression.b[static_cast<int>(index)] = solver->robot.get_joint(joint_name);
      lower[static_cast<int>(index)] = joint_lower;
      upper[static_cast<int>(index)] = joint_upper;
    }

    const auto priority = this->priority == Prioritized::Priority::Hard
                              ? placo::problem::ProblemConstraint::Hard
                              : placo::problem::ProblemConstraint::Soft;
    problem.add_constraint(expression <= upper).configure(priority, weight);
    problem.add_constraint(lower <= expression).configure(priority, weight);
  }

private:
  std::vector<std::string> joint_names_;
};

struct ErrorSummary
{
  CartesianError left;
  CartesianError right;

  double maximumPosition() const
  {
    return std::max(left.position_m, right.position_m);
  }
  double maximumOrientation() const
  {
    return std::max(left.orientation_rad, right.orientation_rad);
  }
};

CartesianError poseError(const Pose & target, const Pose & solved)
{
  CartesianError result;
  result.position_m = (target.translation() - solved.translation()).norm();
  result.orientation_rad =
      pinocchio::log3(target.rotation() * solved.rotation().transpose()).norm();
  return result;
}

ArmTargetError armError(ArmSide side, const CartesianError & error)
{
  return {side, error.position_m, error.orientation_rad};
}

double elapsedMilliseconds(const std::chrono::steady_clock::time_point & started)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
      .count();
}

bool converged(const ProductionStaticConfig & config, const ErrorSummary & errors)
{
  return errors.left.position_m <= config.position_tolerance_m &&
         errors.right.position_m <= config.position_tolerance_m &&
         errors.left.orientation_rad <= config.orientation_tolerance_rad &&
         errors.right.orientation_rad <= config.orientation_tolerance_rad;
}

} // namespace

const char * solveStatusName(SolveStatus status)
{
  switch (status) {
  case SolveStatus::Converged:
    return "Converged";
  case SolveStatus::Saturated:
    return "Saturated";
  case SolveStatus::BestEffort:
    return "BestEffort";
  }
  return "BestEffort";
}

BaselineSolver::BaselineSolver(const std::string & urdf_path)
    : config_(productionStaticConfig()), positions_(config_.initial_positions),
      velocities_(config_.joint_names.size(), 0.0), robot_(urdf_path, kRobotWrapperFlags),
      fk_robot_(urdf_path, kRobotWrapperFlags), solver_(robot_)
{
  robot_.reset();
  fk_robot_.reset();
  solver_.mask_fbase(true);

  const std::set<std::string> active_joints(config_.active_joint_names.begin(),
                                            config_.active_joint_names.end());
  for (const auto & joint_name : robot_.joint_names()) {
    if (active_joints.count(joint_name) == 0U) {
      solver_.mask_dof(joint_name);
    }
  }

  solver_.problem.use_sparsity = config_.use_sparsity;
  solver_.problem.rewrite_equalities = config_.rewrite_equalities;
  solver_.problem.regularization = config_.problem_regularization;
  solver_.enable_joint_limits(false);
  solver_.enable_velocity_limits(false);
  solver_.dt = config_.solver_dt;

  for (std::size_t index = 0; index < config_.joint_names.size(); ++index) {
    const auto & joint_name = config_.joint_names[index];
    const double lower = limitedLower(config_, index);
    const double upper = limitedUpper(config_, index);
    robot_.set_joint_limits(joint_name, lower, upper);
    fk_robot_.set_joint_limits(joint_name, lower, upper);
    robot_.set_velocity_limit(joint_name, config_.velocity_limits[index]);
    fk_robot_.set_velocity_limit(joint_name, config_.velocity_limits[index]);
    robot_.set_joint(joint_name, positions_[index]);
    fk_robot_.set_joint(joint_name, positions_[index]);
  }
  robot_.update_kinematics();
  fk_robot_.update_kinematics();

  const auto & internal_regularization = taskSpec(config_, "internal_regularization");
  solver_.add_regularization_task(internal_regularization.weight)
      .configure(internal_regularization.name, internal_regularization.priority,
                 internal_regularization.weight);

  const auto & absolute_limits = constraintSpec(config_, "absolute_joint_limits");
  solver_.add_constraint(new AbsoluteJointLimitsConstraint(absolute_limits.joints))
      .configure(absolute_limits.name, absolute_limits.priority, absolute_limits.weight);

  const auto initial_left = framePose(robot_, ArmSide::Left);
  const auto initial_right = framePose(robot_, ArmSide::Right);
  const auto & left_position = taskSpec(config_, "left_frame_position");
  left_position_task_ =
      &solver_.add_position_task(config_.left_end_effector_frame, initial_left.translation());
  left_position_task_->configure(left_position.name, left_position.priority, left_position.weight);

  const auto & left_orientation = taskSpec(config_, "left_frame_orientation");
  left_orientation_task_ =
      &solver_.add_orientation_task(config_.left_end_effector_frame, initial_left.rotation());
  left_orientation_task_->configure(left_orientation.name, left_orientation.priority,
                                    left_orientation.weight);

  const auto & right_position = taskSpec(config_, "right_frame_position");
  right_position_task_ =
      &solver_.add_position_task(config_.right_end_effector_frame, initial_right.translation());
  right_position_task_->configure(right_position.name, right_position.priority,
                                  right_position.weight);

  const auto & right_orientation = taskSpec(config_, "right_frame_orientation");
  right_orientation_task_ =
      &solver_.add_orientation_task(config_.right_end_effector_frame, initial_right.rotation());
  right_orientation_task_->configure(right_orientation.name, right_orientation.priority,
                                     right_orientation.weight);

  for (const auto & task : config_.tasks) {
    if (task.type != "JointsTask") {
      continue;
    }
    auto & posture_task = solver_.add_joints_task();
    posture_task.configure(task.name, task.priority, task.weight);
    for (const auto & joint_name : task.joints) {
      posture_task.set_joint(joint_name,
                             config_.initial_positions[jointIndex(config_, joint_name)]);
    }
  }

  const auto & kinetic_energy = taskSpec(config_, "kinetic_energy");
  solver_.add_kinetic_energy_regularization_task(kinetic_energy.weight)
      .configure(kinetic_energy.name, kinetic_energy.priority, kinetic_energy.weight);
}

const std::vector<double> & BaselineSolver::positions() const
{
  return positions_;
}

const std::vector<double> & BaselineSolver::velocities() const
{
  return velocities_;
}

Pose BaselineSolver::framePose(placo::model::RobotWrapper & robot, ArmSide side) const
{
  const auto & frame =
      side == ArmSide::Left ? config_.left_end_effector_frame : config_.right_end_effector_frame;
  Pose result = Pose::Identity();
  result.matrix() = robot.get_T_world_frame(frame).matrix();
  return result;
}

Pose BaselineSolver::tcpOffset(ArmSide side) const
{
  const auto & xyz =
      side == ArmSide::Left ? config_.left_tcp_offset_xyz : config_.right_tcp_offset_xyz;
  Pose result = Pose::Identity();
  result.translation() = Eigen::Vector3d{xyz[0], xyz[1], xyz[2]};
  return result;
}

Pose BaselineSolver::currentEndEffectorPose(ArmSide side)
{
  return framePose(fk_robot_, side);
}

Pose BaselineSolver::currentTcpPose(ArmSide side)
{
  return currentEndEffectorPose(side) * tcpOffset(side);
}

void BaselineSolver::setAcceptedStateOnRobot(placo::model::RobotWrapper & robot) const
{
  for (std::size_t index = 0; index < config_.joint_names.size(); ++index) {
    robot.set_joint(config_.joint_names[index], positions_[index]);
  }
}

std::vector<ArmTarget>
BaselineSolver::toInternalTargets(const std::vector<ArmTarget> & public_tcp_targets) const
{
  return {
      {ArmSide::Left, public_tcp_targets.at(0).target_pose * tcpOffset(ArmSide::Left).inverse()},
      {ArmSide::Right, public_tcp_targets.at(1).target_pose * tcpOffset(ArmSide::Right).inverse()},
  };
}

void BaselineSolver::updateTaskTargets(const std::vector<ArmTarget> & internal_targets)
{
  left_position_task_->target_world = internal_targets.at(0).target_pose.translation();
  left_orientation_task_->R_world_frame = internal_targets.at(0).target_pose.rotation();
  right_position_task_->target_world = internal_targets.at(1).target_pose.translation();
  right_orientation_task_->R_world_frame = internal_targets.at(1).target_pose.rotation();
}

BaselineSolveResult BaselineSolver::solve(const std::vector<ArmTarget> & public_tcp_targets)
{
  const auto started = std::chrono::steady_clock::now();
  const auto previous_positions = positions_;
  setAcceptedStateOnRobot(robot_);
  robot_.update_kinematics();
  const auto internal_targets = toInternalTargets(public_tcp_targets);
  updateTaskTargets(internal_targets);

  auto currentErrors = [&]() {
    const auto left = framePose(robot_, ArmSide::Left);
    const auto right = framePose(robot_, ArmSide::Right);
    return ErrorSummary{
        poseError(internal_targets[0].target_pose, left),
        poseError(internal_targets[1].target_pose, right),
    };
  };

  ErrorSummary errors = currentErrors();
  ErrorSummary previous_errors = errors;
  int iterations = 0;
  std::string termination_reason = "initial-converged";

  if (!converged(config_, errors)) {
    termination_reason = "iteration-budget";
    for (int iteration = 0; iteration < config_.maximum_iterations; ++iteration) {
      solver_.dt = config_.solver_dt;
      solver_.solve(true);
      robot_.update_kinematics();
      iterations = iteration + 1;
      errors = currentErrors();

      if (converged(config_, errors)) {
        termination_reason = "converged";
        break;
      }

      const bool improved_position = previous_errors.maximumPosition() - errors.maximumPosition() >
                                     config_.minimum_position_improvement_m;
      const bool improved_orientation =
          previous_errors.maximumOrientation() - errors.maximumOrientation() >
          config_.minimum_orientation_improvement_rad;
      if (!improved_position && !improved_orientation) {
        termination_reason = "no-progress";
        break;
      }
      previous_errors = errors;

      if (elapsedMilliseconds(started) >= config_.soft_solve_time_budget_ms) {
        termination_reason = "soft-time-budget";
        break;
      }
    }
  }

  std::vector<double> published_positions = previous_positions;
  std::vector<double> published_velocities(config_.joint_names.size(), 0.0);
  std::vector<std::string> saturated_joints;
  for (const auto & joint_name : config_.active_joint_names) {
    const std::size_t index = jointIndex(config_, joint_name);
    const double raw_position = robot_.get_joint(joint_name);
    if (!std::isfinite(raw_position)) {
      throw std::runtime_error("PlaCo returned a non-finite joint position for " + joint_name);
    }
    const double published_position =
        std::clamp(raw_position, config_.lower_limits[index], config_.upper_limits[index]);
    published_positions[index] = published_position;
    published_velocities[index] =
        (published_position - previous_positions[index]) / config_.control_dt_s;
    if (std::fabs(published_position - config_.lower_limits[index]) <= 1.0e-3 ||
        std::fabs(config_.upper_limits[index] - published_position) <= 1.0e-3 ||
        std::fabs(raw_position - published_position) > 1.0e-6) {
      saturated_joints.push_back(joint_name);
    }
  }
  std::sort(saturated_joints.begin(), saturated_joints.end());

  positions_ = std::move(published_positions);
  velocities_ = std::move(published_velocities);
  setAcceptedStateOnRobot(fk_robot_);
  fk_robot_.update_kinematics();

  const Pose left_ee = framePose(fk_robot_, ArmSide::Left);
  const Pose right_ee = framePose(fk_robot_, ArmSide::Right);
  const Pose left_tcp = left_ee * tcpOffset(ArmSide::Left);
  const Pose right_tcp = right_ee * tcpOffset(ArmSide::Right);
  errors = {
      poseError(internal_targets[0].target_pose, left_ee),
      poseError(internal_targets[1].target_pose, right_ee),
  };
  const ErrorSummary tcp_errors{
      poseError(public_tcp_targets.at(0).target_pose, left_tcp),
      poseError(public_tcp_targets.at(1).target_pose, right_tcp),
  };

  const bool is_converged = converged(config_, errors);
  const double frame_scale = solver_.has_scaling ? solver_.scale : 1.0;
  SolveStatus status = SolveStatus::BestEffort;
  if (is_converged) {
    status = SolveStatus::Converged;
  } else if (!saturated_joints.empty() || frame_scale < 0.999) {
    status = SolveStatus::Saturated;
  }

  double maximum_hard_violation = 0.0;
  for (const auto & joint_name : config_.active_joint_names) {
    const std::size_t index = jointIndex(config_, joint_name);
    maximum_hard_violation = std::max(maximum_hard_violation,
                                      std::max(limitedLower(config_, index) - positions_[index],
                                               positions_[index] - limitedUpper(config_, index)));
  }

  BaselineSolveResult result;
  result.status = status;
  result.status_name = solveStatusName(status);
  result.termination_reason = termination_reason;
  result.iterations = iterations;
  result.converged = is_converged;
  result.frame_scale = frame_scale;
  result.solve_time_ms = elapsedMilliseconds(started);
  result.maximum_hard_violation = maximum_hard_violation;
  result.positions = positions_;
  result.velocities = velocities_;
  result.saturated_joints = saturated_joints;
  result.end_effector_forward_kinematics = {{ArmSide::Left, left_ee}, {ArmSide::Right, right_ee}};
  result.tcp_forward_kinematics = {{ArmSide::Left, left_tcp}, {ArmSide::Right, right_tcp}};
  result.internal_end_effector_targets = internal_targets;
  result.end_effector_target_errors = {armError(ArmSide::Left, errors.left),
                                       armError(ArmSide::Right, errors.right)};
  result.tcp_target_errors = {armError(ArmSide::Left, tcp_errors.left),
                              armError(ArmSide::Right, tcp_errors.right)};

  result.solver_debug.label = "PlaCo production-static baseline";
  result.solver_debug.disposition = result.status_name;
  result.solver_debug.joint_limit_policy = "absolute-position (margin 0.08 rad)";
  result.solver_debug.termination_reason = termination_reason;
  result.solver_debug.ik_iterations = iterations;
  result.solver_debug.converged = is_converged;
  result.solver_debug.ik_solve_time_ms = result.solve_time_ms;
  result.solver_debug.saturated_joints = saturated_joints;
  result.solver_debug.backend = "eiquadprog";
  result.solver_debug.qp_status = "solved";
  result.solver_debug.has_qp_diagnostics = false;
  result.solver_debug.maximum_hard_violation = maximum_hard_violation;
  result.solver_debug.task_scales = {
      {"frame_position", true, frame_scale, 0.0, frame_scale < 0.999, false}};
  return result;
}

} // namespace motion_control_lab::baseline
