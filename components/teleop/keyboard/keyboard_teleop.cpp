#include "components/teleop/keyboard/keyboard_teleop.hpp"

#include <cctype>

namespace motion_control_lab
{

KeyboardTeleop::KeyboardTeleop(KeyboardSourceMode mode) : mode_(mode) {}

KeyboardAction KeyboardTeleop::handle(const KeyEvent & event)
{
  if (entering_step_) {
    if (event.code == KeyCode::Escape) {
      entering_step_ = false;
      step_buffer_.clear();
      return KeyboardAction{std::nullopt, std::nullopt, "Step unchanged"};
    }
    if (event.code == KeyCode::Enter) {
      entering_step_ = false;
      if (step_buffer_.empty()) {
        return KeyboardAction{std::nullopt, std::nullopt, "Step unchanged"};
      }
      TeleopIntent intent;
      intent.kind = TeleopIntentKind::SetStep;
      intent.scalar = std::stod(step_buffer_);
      step_buffer_.clear();
      return KeyboardAction{intent, std::nullopt, {}};
    }
    if (event.code == KeyCode::Character) {
      const char value = event.character;
      if (std::isdigit(static_cast<unsigned char>(value)) || value == '.' || value == '-' ||
          value == '+' || value == 'e' || value == 'E') {
        step_buffer_.push_back(value);
      }
      return KeyboardAction{std::nullopt, std::nullopt, "Translation step: " + step_buffer_};
    }
    return {};
  }

  if (event.code == KeyCode::Escape) {
    return KeyboardAction{std::nullopt, SourceControl::Stop, "Exiting"};
  }
  if (event.code == KeyCode::ArrowLeft || event.code == KeyCode::ArrowRight) {
    TeleopIntent intent;
    intent.kind = TeleopIntentKind::SelectArm;
    intent.side = event.code == KeyCode::ArrowLeft ? ArmSide::Left : ArmSide::Right;
    return KeyboardAction{intent, std::nullopt, {}};
  }
  if (event.code == KeyCode::ArrowUp || event.code == KeyCode::ArrowDown) {
    TeleopIntent intent;
    intent.kind = event.code == KeyCode::ArrowUp
      ? TeleopIntentKind::IncreaseStep
      : TeleopIntentKind::DecreaseStep;
    return KeyboardAction{intent, std::nullopt, {}};
  }
  if (event.code != KeyCode::Character) {
    return {};
  }
  return handleCharacter(static_cast<char>(
    std::tolower(static_cast<unsigned char>(event.character))));
}

KeyboardAction KeyboardTeleop::handleCharacter(char key)
{
  if (mode_ == KeyboardSourceMode::Replay) {
    switch (key) {
      case 'q':
      case 'x':
        return KeyboardAction{std::nullopt, SourceControl::Stop, "Exiting"};
      case ' ':
        return KeyboardAction{
          std::nullopt, SourceControl::TogglePause, "Replay pause toggled"};
      case '.':
        return KeyboardAction{
          std::nullopt, SourceControl::Step, "Replay single-frame step requested"};
      default:
        return KeyboardAction{
          std::nullopt, std::nullopt, "Replay motion editing is disabled"};
    }
  }

  TeleopIntent intent;
  switch (key) {
    case 'x':
      return KeyboardAction{std::nullopt, SourceControl::Stop, "Exiting"};
    case ' ':
      return KeyboardAction{std::nullopt, SourceControl::TogglePause, "Publishing pause toggled"};
    case 'm':
      entering_step_ = true;
      step_buffer_.clear();
      return KeyboardAction{std::nullopt, std::nullopt, "Enter translation step"};
    case 'r':
      intent.kind = TeleopIntentKind::ResetTarget;
      break;
    case 'n':
      intent.kind = TeleopIntentKind::CycleRotationAxis;
      break;
    case 'i':
    case 'u':
      intent.kind = TeleopIntentKind::Rotate;
      intent.clockwise = key == 'i';
      break;
    case 'w':
    case 's':
    case 'a':
    case 'd':
    case 'q':
    case 'e':
      intent.kind = TeleopIntentKind::Translate;
      if (key == 'w') intent.translation.x() = 1.0;
      if (key == 's') intent.translation.x() = -1.0;
      if (key == 'a') intent.translation.y() = 1.0;
      if (key == 'd') intent.translation.y() = -1.0;
      if (key == 'q') intent.translation.z() = 1.0;
      if (key == 'e') intent.translation.z() = -1.0;
      break;
    default:
      return {};
  }
  return KeyboardAction{intent, std::nullopt, {}};
}

}  // namespace motion_control_lab
