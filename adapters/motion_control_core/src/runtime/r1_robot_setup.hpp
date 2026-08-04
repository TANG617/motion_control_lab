#pragma once

#include <motion_control_lab/ik_solver_backend.hpp>

#include "motion_control_core/motion_control_core.hpp"

#include <string>
#include <vector>

namespace motion_control_lab
{

namespace mcc = motion_control::core;

const JointNames & r1JointNames();

std::vector<double> r1InitialPositions();

mcc::RobotModelDescription makeR1ModelDescription(const std::string & urdf_path);

mcc::InverseKinematicsConfig makeR1IkConfig();

const mcc::EndEffectorTarget & findPose(
  const std::vector<mcc::EndEffectorTarget> & poses,
  const std::string & frame_name);

}  // namespace motion_control_lab
