#pragma once

#include "runtime/interactive_types.hpp"

#include <motion_control_core/motion_control_core.hpp>

#include <Eigen/Core>

#include <vector>

namespace motion_control_lab
{

Eigen::VectorXd toEigen(const std::vector<double> & values);

std::vector<double> toStdVector(const Eigen::VectorXd & values);

motion_control::core::RobotState makeRobotState(
  const std::vector<double> & positions,
  const std::vector<double> & velocities);

SolverDebug makeSolverDebug(
  std::string label,
  const motion_control::core::InverseKinematicsDiagnostics & diagnostics,
  motion_control::core::ResultDisposition disposition);

SolverDebug makeSolverDebug(
  std::string label,
  const motion_control::core::GroupedInverseKinematicsDiagnostics & diagnostics,
  motion_control::core::ResultDisposition disposition);

void updateSolverDebug(
  SolverDebug & output,
  const motion_control::core::InverseKinematicsDiagnostics & diagnostics,
  motion_control::core::ResultDisposition disposition);

void updateSolverDebug(
  SolverDebug & output,
  const motion_control::core::GroupedInverseKinematicsDiagnostics & diagnostics,
  motion_control::core::ResultDisposition disposition);

}  // namespace motion_control_lab
