#pragma once

#include "contracts/presentation/ik_app_snapshot.hpp"

#include <motion_control_core/motion_control_core.hpp>

#include <string>

namespace motion_control_lab
{

SolverDebug makeSolverDebug(
  std::string label,
  const motion_control::core::InverseKinematicsDiagnostics & diagnostics,
  motion_control::core::ResultDisposition disposition);

void updateSolverDebug(
  SolverDebug & output,
  const motion_control::core::InverseKinematicsDiagnostics & diagnostics,
  motion_control::core::ResultDisposition disposition);

}  // namespace motion_control_lab

