#pragma once

#include "contracts/input/input_contract.hpp"

#include <string>

namespace motion_control_lab
{

enum class KeyboardSourceMode
{
  Teleop,
  Replay,
};

class KeyboardTeleop
{
public:
  explicit KeyboardTeleop(KeyboardSourceMode mode);

  KeyboardAction handle(const KeyEvent & event);
  bool capturingText() const noexcept { return entering_step_; }

private:
  KeyboardAction handleCharacter(char key);

  KeyboardSourceMode mode_{KeyboardSourceMode::Teleop};
  bool entering_step_{false};
  std::string step_buffer_;
};

}  // namespace motion_control_lab
