#pragma once

#include "runtime/interactive_types.hpp"

#include <motion_control_core/motion_control_core.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>

namespace motion_control_lab
{

Eigen::VectorXd toEigen(const std::vector<double> & values);

std::vector<double> toStdVector(const Eigen::VectorXd & values);

void requireOk(
  const motion_control::core::Status & status,
  const std::string & operation);

motion_control::core::RobotState makeRobotState(
  const std::vector<double> & positions,
  const std::vector<double> & velocities);

const motion_control::core::FramePose & requirePose(
  const std::vector<motion_control::core::FramePose> & poses,
  const std::string & frame_name);

const ArmTarget & requireTarget(
  const std::vector<ArmTarget> & targets,
  ArmSide side);

}  // namespace motion_control_lab
