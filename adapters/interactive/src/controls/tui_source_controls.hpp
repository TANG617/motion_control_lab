#pragma once

#include "runtime/interactive_types.hpp"

namespace motion_control_lab
{

enum class TuiControlMode
{
  Teleop,
  Replay,
};

enum class TuiSourceControlAction
{
  Unhandled,
  Handled,
  Exit,
  ToggleHelp,
};

class TuiSourceControls
{
public:
  explicit TuiSourceControls(TuiControlMode mode);

  TuiControlMode mode() const noexcept;

  TuiSourceControlAction handleCharacter(
    char key, TargetCommand & command, bool & single_step_requested) const;

private:
  TuiControlMode mode_{TuiControlMode::Teleop};
};

}  // namespace motion_control_lab
