#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace motion_control_lab
{

using Pose = Eigen::Isometry3d;
using JointNames = std::vector<std::string>;

enum class ArmSide
{
  Left,
  Right,
};

inline const char * armSideName(ArmSide side)
{
  return side == ArmSide::Left ? "left" : "right";
}

inline ArmSide parseArmSide(const std::string & side)
{
  if (side == "left") {
    return ArmSide::Left;
  }
  if (side == "right") {
    return ArmSide::Right;
  }
  throw std::runtime_error("side must be either 'left' or 'right'");
}

struct ArmTarget
{
  ArmSide side{ArmSide::Left};
  Pose target_pose{Pose::Identity()};
};

struct ArmTargetError
{
  ArmSide side{ArmSide::Left};
  double position_m{0.0};
  double orientation_rad{0.0};
};

struct ArmForwardKinematics
{
  ArmSide side{ArmSide::Left};
  Pose pose{Pose::Identity()};
};

struct ArmPresentation
{
  ArmSide side{ArmSide::Left};
  std::string target_channel;
  std::string forward_kinematics_channel;
  std::vector<std::size_t> joint_indices;
};

struct InteractiveIkPresentation
{
  std::string base_frame_id;
  std::string joint_state_channel;
  std::vector<ArmPresentation> arms;
};

inline const ArmPresentation * findArmPresentation(
  const InteractiveIkPresentation & presentation,
  ArmSide side)
{
  for (const auto & arm : presentation.arms) {
    if (arm.side == side) {
      return &arm;
    }
  }
  return nullptr;
}

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
  std::vector<ArmTarget> targets;
  std::vector<ArmForwardKinematics> forward_kinematics;
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
