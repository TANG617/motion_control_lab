#include "solver.hpp"

#include <stdexcept>
#include <utility>

namespace motion_control_lab::hierarchical_inverse_dynamics_torque_sim {
namespace {

void requireOk(const mcc::Status &status, const char *operation) {
  if (!status.ok()) {
    throw std::runtime_error(std::string{operation} + ": " + status.message);
  }
}

mcc::SoftEnforcement soft(double weight, int rows) {
  return mcc::SoftEnforcement{
      mcc::QuadraticPenalty{weight, Eigen::VectorXd::Ones(rows)}};
}

} // namespace

Runtime configureSolver(const R1RobotConfig &robot, const Options &options,
                        Handles &handles) {
  Runtime runtime;
  mcc::RobotModelDescription description;
  description.urdf_path = options.urdf_path;
  description.kinematics_reference_frame = robot.base_frame;
  description.joint_names = robot.joint_names;
  description.root_joint = mcc::RootJointType::Fixed;
  requireOk(mcc::RobotModel::load(description, runtime.robot_model),
            "load R1 model");

  mcc::ActuationModelDescription actuation;
  for (std::size_t index = 0; index < robot.joint_names.size(); ++index) {
    actuation.actuators.push_back(mcc::ScalarActuatorDescription{
        robot.joint_names[index], -robot.effort_limits[index],
        robot.effort_limits[index]});
  }
  requireOk(mcc::ActuationModel::create(runtime.robot_model, actuation,
                                        runtime.actuation_model),
            "create R1 actuation model");

  mcc::HierarchicalInverseDynamicsSolverConfig config;
  config.control_period = 0.001;
  config.hard_tolerances.generalized_dynamics = 1.0e-3;
  config.hard_tolerances.joint_acceleration = 1.0e-3;
  config.hard_tolerances.actuator_effort = 1.0e-3;
  config.hard_tolerances.contact_acceleration = 1.0e-3;
  config.hard_tolerances.contact_wrench = 1.0e-3;
  config.qp.backend = mcc::QpBackend::ProxQp;
  config.qp.regularization = 1.0e-8;
  config.qp.proxqp.absolute_tolerance = 1.0e-4;
  config.qp.proxqp.maximum_iterations = 200;
  config.qp.proxqp.warm_start_enabled = false;
  mcc::HierarchicalInverseDynamicsSolverBuilder builder;
  requireOk(
      builder.configure(runtime.robot_model, runtime.actuation_model, config),
      "configure HID");

  constexpr double tracking_rate_per_s = 10.0;
  constexpr double kp = tracking_rate_per_s * tracking_rate_per_s;
  constexpr double kd = 2.0 * tracking_rate_per_s;
  mcc::PositionAccelerationTaskConfig position;
  position.position_gain_per_s2.setConstant(kp);
  position.velocity_gain_per_s.setConstant(kd);
  position.enforcement = soft(100.0, 3);
  position.name = "left-hand-position";
  requireOk(builder.addPositionTask(mcc::PriorityLevel::Primary,
                                    robot.left_end_effector_frame, position,
                                    Eigen::Vector3d::Constant(1.0e-4),
                                    handles.left_position),
            "add left position");
  position.name = "right-hand-position";
  requireOk(builder.addPositionTask(mcc::PriorityLevel::Primary,
                                    robot.right_end_effector_frame, position,
                                    Eigen::Vector3d::Constant(1.0e-4),
                                    handles.right_position),
            "add right position");

  mcc::OrientationAccelerationTaskConfig orientation;
  orientation.orientation_gain_per_s2.setConstant(kp);
  orientation.angular_velocity_gain_per_s.setConstant(kd);
  orientation.enforcement = soft(50.0, 3);
  orientation.name = "left-hand-orientation";
  requireOk(builder.addOrientationTask(
                mcc::PriorityLevel::Secondary, robot.left_end_effector_frame,
                orientation, Eigen::Vector3d::Constant(1.0e-4),
                handles.left_orientation),
            "add left orientation");
  orientation.name = "right-hand-orientation";
  requireOk(builder.addOrientationTask(
                mcc::PriorityLevel::Secondary, robot.right_end_effector_frame,
                orientation, Eigen::Vector3d::Constant(1.0e-4),
                handles.right_orientation),
            "add right orientation");

  mcc::PostureAccelerationTaskConfig posture;
  posture.name = "nominal-posture";
  posture.position_gain_per_s2.setConstant(20, kp);
  posture.velocity_gain_per_s.setConstant(20, kd);
  posture.enforcement = soft(1.0, 20);
  requireOk(builder.addPostureTask(mcc::PriorityLevel::Tertiary, posture,
                                   Eigen::VectorXd::Constant(20, 1.0e-3),
                                   handles.posture),
            "add posture");
  requireOk(builder.finalize(runtime.solver), "finalize HID");
  return runtime;
}

} // namespace motion_control_lab::hierarchical_inverse_dynamics_torque_sim
