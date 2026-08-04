#include "runtime/r1_robot_setup.hpp"

#include "config/constants.hpp"

#include <stdexcept>

namespace motion_control_lab
{

const JointNames & r1JointNames()
{
  static const JointNames names{
    "head_yaw_joint",
    "head_pitch_joint",
    "torso_yaw_joint",
    "torso_pitch_joint",
    "knee_pitch_joint",
    "ankle_pitch_joint",
    "left_arm_joint1",
    "left_arm_joint2",
    "left_arm_joint3",
    "left_arm_joint4",
    "left_arm_joint5",
    "left_arm_joint6",
    "left_arm_joint7",
    "right_arm_joint1",
    "right_arm_joint2",
    "right_arm_joint3",
    "right_arm_joint4",
    "right_arm_joint5",
    "right_arm_joint6",
    "right_arm_joint7"};
  return names;
}

std::vector<double> r1InitialPositions()
{
  return {
    0.0,
    0.31,
    0.0,
    0.5,
    0.5,
    -0.5,
    0.9,
    -1.38,
    -1.57,
    -1.4,
    -0.45,
    0.0,
    0.0,
    -0.9,
    1.38,
    1.57,
    1.4,
    0.45,
    0.0,
    0.0};
}

mcc::RobotModelDescription makeR1ModelDescription(const std::string & urdf_path)
{
  mcc::RobotModelDescription model;
  model.urdf_path = urdf_path;
  model.base_frame = kBaseFrame;
  model.joint_names = r1JointNames();
  model.end_effector_names = {"left_arm_ee_link", "right_arm_ee_link"};
  return model;
}

mcc::InverseKinematicsConfig makeR1IkConfig()
{
  mcc::InverseKinematicsConfig config;
  config.max_inner_iterations = 80;
  config.max_solve_time_ms = 100.0;
  config.position_tolerance_m = 1.0e-4;
  config.orientation_tolerance_rad = 1.0e-4;
  config.min_position_error_improvement_m = 1.0e-8;
  config.min_orientation_error_improvement_rad = 1.0e-8;
  config.constraints.soft_limit_margin = 0.0;
  config.qp.regularization = 1.0e-8;
  return config;
}

const mcc::EndEffectorTarget & findPose(
  const std::vector<mcc::EndEffectorTarget> & poses,
  const std::string & frame_name)
{
  for (const auto & pose : poses) {
    if (pose.frame_name == frame_name) {
      return pose;
    }
  }
  throw std::runtime_error("FK result did not contain frame: " + frame_name);
}

}  // namespace motion_control_lab
