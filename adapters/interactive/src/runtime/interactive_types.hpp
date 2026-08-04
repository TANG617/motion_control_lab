#pragma once

#include <motion_control_lab/ik_solver_backend.hpp>

#include <string>
#include <vector>

namespace motion_control_lab
{

struct TargetCommand
{
  std::vector<ArmTarget> targets;
  ArmSide selected_side{ArmSide::Left};
  bool paused{false};
  bool stop_requested{false};
  std::string status{"Ready"};
};

struct IkDebugFrame
{
  std::string backend_id{"unknown"};
  std::vector<ArmTarget> targets;
  JointNames joint_names;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::string ik_status{"not solved yet"};
  int iterations{0};
  bool converged{false};
  double solve_time_ms{0.0};
  std::vector<ArmTargetError> target_errors;
  std::string status{"Ready"};
  bool paused{false};
  ArmSide selected_side{ArmSide::Left};
};

}  // namespace motion_control_lab
