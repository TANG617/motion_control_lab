#pragma once

#include <Eigen/Geometry>

#include <stdexcept>
#include <string>
#include <string_view>
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

enum class IkSolveStatusCode
{
  Ok,
  InvalidInput,
  InvalidState,
  InvalidTarget,
  NotInitialized,
  Infeasible,
  Saturated,
  BestEffort,
  SolverError,
};

struct IkSolveStatus
{
  IkSolveStatusCode code{IkSolveStatusCode::Ok};
  std::string message;

  bool ok() const
  {
    return code == IkSolveStatusCode::Ok;
  }
};

struct JointState
{
  JointNames names;
  std::vector<double> positions;
  std::vector<double> velocities;
};

struct IkSolveDiagnostics
{
  std::vector<ArmTargetError> errors;
  std::vector<std::string> saturated_joints;
  int iterations{0};
  bool converged{false};
  double solve_time_ms{0.0};
};

struct IkSolveResult
{
  IkSolveStatus status;
  JointState joint_state;
  std::vector<ArmTarget> solved_targets;
  IkSolveDiagnostics diagnostics;
};

// Backend-neutral R1 IK boundary. The interactive runner currently receives one
// MCC implementation. A future A/B runner can drive two implementations with
// the same targets and dt without depending on either solver's native types.
class IkSolverBackend
{
public:
  virtual ~IkSolverBackend() = default;

  virtual std::string_view backendId() const = 0;

  virtual const JointNames & jointNames() const = 0;

  virtual const std::vector<double> & positions() const = 0;

  virtual const std::vector<double> & velocities() const = 0;

  virtual void reset(const JointState & state) = 0;

  virtual Pose currentTargetPose(ArmSide side) const = 0;

  virtual std::vector<ArmTarget> currentTargetPoses(
    const std::vector<ArmSide> & sides) const = 0;

  virtual IkSolveResult solveTargets(
    const std::vector<ArmTarget> & targets,
    double dt) = 0;
};

}  // namespace motion_control_lab
