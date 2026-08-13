#pragma once

#include <motion_control_core/motion_control_core.hpp>

#include <string>

namespace motion_control_lab
{

struct DualArmIkAppConfig
{
  std::string program_id;
  std::string title;
  motion_control::core::IkSolveMode solve_mode{
    motion_control::core::IkSolveMode::ServoStep};
};

struct DualArmIkSolverSetup
{
  motion_control::core::KinematicsSolverConfig solver_config;
  bool register_joint_velocity_limits{false};
};

DualArmIkSolverSetup makeDualArmIkSolverSetup(
  motion_control::core::IkSolveMode solve_mode,
  double rate_hz);

int runDualArmIkApp(
  int argc,
  char ** argv,
  const DualArmIkAppConfig & app_config);

}  // namespace motion_control_lab
