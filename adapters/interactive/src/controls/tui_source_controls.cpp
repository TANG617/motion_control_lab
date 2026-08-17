#include "controls/tui_source_controls.hpp"

namespace motion_control_lab
{

TuiSourceControls::TuiSourceControls(TuiControlMode mode) : mode_(mode) {}

TuiControlMode TuiSourceControls::mode() const noexcept { return mode_; }

TuiSourceControlAction TuiSourceControls::handleCharacter(
  char key, TargetCommand & command, bool & single_step_requested) const
{
  if (mode_ != TuiControlMode::Replay) {
    return TuiSourceControlAction::Unhandled;
  }
  switch (key) {
    case 'q':
    case 'x':
      return TuiSourceControlAction::Exit;
    case 'h':
      return TuiSourceControlAction::ToggleHelp;
    case ' ':
      command.paused = !command.paused;
      command.status = command.paused ? "Replay timeline paused" : "Replay timeline resumed";
      return TuiSourceControlAction::Handled;
    case '.':
      single_step_requested = true;
      command.paused = true;
      command.status = "Replay single-frame step requested";
      return TuiSourceControlAction::Handled;
    default:
      command.status = "Replay motion editing is disabled";
      return TuiSourceControlAction::Handled;
  }
}

}  // namespace motion_control_lab
