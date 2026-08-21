#include "solver.hpp"

#include <stdexcept>
#include <string>

namespace motion_control_lab::single_arm_servo_step {

void requireOk(const mcc::Status &status) {
  if (!status.ok()) {
    throw std::runtime_error(status.message);
  }
}

std::shared_ptr<const mcc::RobotModel>
loadRobotModel(const R1RobotConfig &robot, const AppOptions &options) {
  mcc::RobotModelDescription description;
  description.urdf_path = options.urdf_path;
  description.kinematics_reference_frame = robot.base_frame;
  description.joint_names = robot.joint_names;
  std::shared_ptr<const mcc::RobotModel> model;
  requireOk(mcc::RobotModel::load(description, model));
  return model;
}

mcc::KinematicsSolverConfig makeSolverConfig(const AppOptions &options) {
  mcc::KinematicsSolverConfig config;
  config.mode = mcc::IkSolveMode::ServoStep;
  config.servo_period = 1.0 / options.rate_hz;
  config.joint_limit_policy =
      mcc::KinematicsJointLimitPolicy::ExplicitRequirements;
  config.qp.backend = mcc::QpBackend::ProxQp;
  config.qp.regularization = options.regularization;
  config.maximum_iterations = options.maximum_iterations;
  config.soft_solve_time_budget_ms = 100.0;
  config.position_tolerance_m = options.position_tolerance_m;
  config.orientation_tolerance_rad = options.orientation_tolerance_rad;
  config.minimum_position_improvement_m = 1.0e-8;
  config.minimum_orientation_improvement_rad = 1.0e-8;
  return config;
}

void configureSolver(mcc::KinematicsSolverBuilder &builder,
                     SolverHandles &handles, const R1RobotConfig &robot,
                     ArmSide controlled_side, const AppOptions &options) {
  const std::string &controlled_frame = controlled_side == ArmSide::Left
                                            ? robot.left_end_effector_frame
                                            : robot.right_end_effector_frame;

  mcc::PositionTaskConfig position;
  position.name = std::string{armSideName(controlled_side)} + "-position";
  position.enforcement = mcc::HardEnforcement{};
  requireOk(
      builder.addPositionTask(controlled_frame, position, handles.position));

  mcc::OrientationTaskConfig orientation;
  orientation.name = std::string{armSideName(controlled_side)} + "-orientation";
  orientation.enforcement = mcc::HardEnforcement{};
  requireOk(builder.addOrientationTask(controlled_frame, orientation,
                                       handles.orientation));

  mcc::JointPositionLimitConfig position_limits;
  position_limits.margin = options.joint_position_margin_rad;
  position_limits.enforcement = mcc::HardEnforcement{};
  mcc::JointPositionLimitHandle position_limit_handle;
  requireOk(
      builder.addJointPositionLimits(position_limits, position_limit_handle));

  mcc::JointVelocityLimitConfig velocity_limits;
  velocity_limits.enforcement = mcc::HardEnforcement{};
  mcc::JointVelocityLimitHandle velocity_limit_handle;
  requireOk(
      builder.addJointVelocityLimits(velocity_limits, velocity_limit_handle));
}

} // namespace motion_control_lab::single_arm_servo_step
