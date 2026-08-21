#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <optional>
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

inline const char * armSideName(ArmSide side) { return side == ArmSide::Left ? "left" : "right"; }

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

struct MotionTargetFrame
{
  std::vector<ArmTarget> targets;
  std::int64_t logical_time_ns{0};
  std::uint64_t revision{0};
  std::string source;
};

enum class InputState
{
  Running,
  Paused,
  EndOfStream,
  Fault,
  Stopped,
};

struct InputStatus
{
  InputState state{InputState::Running};
  std::string detail{"Ready"};
};

enum class SourceControl
{
  Pause,
  Resume,
  TogglePause,
  Step,
  Stop,
};

enum class TeleopIntentKind
{
  SelectArm,
  Translate,
  Rotate,
  IncreaseStep,
  DecreaseStep,
  SetStep,
  CycleRotationAxis,
  ResetTarget,
};

struct TeleopIntent
{
  TeleopIntentKind kind{TeleopIntentKind::Translate};
  ArmSide side{ArmSide::Left};
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
  double scalar{0.0};
  bool clockwise{false};
  bool discrete{true};
};

enum class KeyCode
{
  Character,
  ArrowLeft,
  ArrowRight,
  ArrowUp,
  ArrowDown,
  PageUp,
  PageDown,
  Home,
  End,
  Tab,
  BackTab,
  Function1,
  Function2,
  Function3,
  Function4,
  Function5,
  Function6,
  Function7,
  Enter,
  Escape,
};

struct KeyEvent
{
  KeyCode code{KeyCode::Character};
  char character{'\0'};

  static KeyEvent characterKey(char value) { return KeyEvent{KeyCode::Character, value}; }
};

struct KeyboardAction
{
  std::optional<TeleopIntent> teleop;
  std::optional<SourceControl> source_control;
  std::string status;
};

} // namespace motion_control_lab
