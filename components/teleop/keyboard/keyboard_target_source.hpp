#pragma once

#include "components/teleop/cartesian/cartesian_teleop.hpp"
#include "components/teleop/keyboard/keyboard_teleop.hpp"
#include "components/terminal_frontend/key_router.hpp"
#include "components/terminal_frontend/terminal_frontend.hpp"

#include <optional>
#include <string>
#include <vector>

namespace motion_control_lab
{

struct KeyboardTargetSourceUpdate
{
  std::vector<KeyEvent> navigation;
};

class KeyboardTargetSource
{
public:
  KeyboardTargetSource(
    TerminalFrontend & terminal,
    KeyboardSourceMode mode,
    CartesianTeleopOptions options,
    std::vector<ArmTarget> initial_targets,
    bool allow_side_switching);

  KeyboardTargetSourceUpdate poll(double dt);
  const MotionTargetFrame & targetFrame() const noexcept;
  const std::vector<ArmTarget> & targets() const noexcept;
  ArmSide selectedSide() const noexcept;
  bool paused() const noexcept;
  bool stopRequested() const noexcept;
  const std::string & status() const noexcept;
  void setStatus(std::string status);
  void setPaused(bool paused, std::string status);
  void setMotionInputEnabled(bool enabled, std::string status);
  void setTargetPose(ArmSide side, const Pose & pose, std::string status);
  std::optional<ArmSide> consumeResetRequest();
  std::vector<SourceControl> consumeSourceControls();

private:
  void apply(const KeyboardAction & action, double dt);

  TerminalFrontend & terminal_;
  KeyboardSourceMode mode_;
  KeyRouter router_;
  KeyboardTeleop keyboard_;
  CartesianTeleop cartesian_;
  InputStatus input_status_;
  bool paused_{false};
  bool stop_requested_{false};
  bool motion_input_enabled_{true};
  std::string motion_input_disabled_status_{"Motion controls disabled"};
  std::optional<ArmSide> reset_request_;
  std::vector<SourceControl> source_controls_;
};

}  // namespace motion_control_lab
