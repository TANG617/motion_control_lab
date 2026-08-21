#include "components/teleop/keyboard/keyboard_target_source.hpp"

#include <utility>

namespace motion_control_lab
{

KeyboardTargetSource::KeyboardTargetSource(
  TerminalFrontend & terminal,
  KeyboardSourceMode mode,
  CartesianTeleopOptions options,
  std::vector<ArmTarget> initial_targets,
  bool allow_side_switching)
: terminal_(terminal),
  mode_(mode),
  keyboard_(mode),
  cartesian_(std::move(options), std::move(initial_targets), allow_side_switching)
{
  input_status_.detail = cartesian_.status();
}

KeyboardTargetSourceUpdate KeyboardTargetSource::poll(double dt)
{
  KeyboardTargetSourceUpdate update;
  for (const auto & event : terminal_.poll()) {
    if (!keyboard_.capturingText() && router_.route(event) == KeyRoute::Navigation) {
      update.navigation.push_back(event);
      continue;
    }
    apply(keyboard_.handle(event), dt);
  }
  return update;
}

void KeyboardTargetSource::apply(const KeyboardAction & action, double dt)
{
  if (action.source_control.has_value()) {
    if (mode_ == KeyboardSourceMode::Teleop && !motion_input_enabled_ &&
        *action.source_control != SourceControl::Stop) {
      input_status_.detail = motion_input_disabled_status_;
      return;
    }
    if (mode_ == KeyboardSourceMode::Replay) {
      source_controls_.push_back(*action.source_control);
    }
    switch (*action.source_control) {
      case SourceControl::Pause:
        paused_ = true;
        input_status_.state = InputState::Paused;
        break;
      case SourceControl::Resume:
        paused_ = false;
        input_status_.state = InputState::Running;
        break;
      case SourceControl::TogglePause:
        paused_ = !paused_;
        input_status_.state = paused_ ? InputState::Paused : InputState::Running;
        input_status_.detail = paused_
          ? "Replay timeline paused"
          : "Replay timeline resumed";
        break;
      case SourceControl::Step:
        paused_ = true;
        input_status_.state = InputState::Paused;
        input_status_.detail = "Replay single-frame step requested";
        break;
      case SourceControl::Stop:
        stop_requested_ = true;
        input_status_.state = InputState::Stopped;
        break;
    }
  }
  if (action.teleop.has_value()) {
    const bool selection_only = action.teleop->kind == TeleopIntentKind::SelectArm;
    if (!motion_input_enabled_ && !selection_only) {
      input_status_.detail = motion_input_disabled_status_;
      return;
    }
    if (const auto reset = cartesian_.apply(*action.teleop, dt)) {
      reset_request_ = reset;
    }
    input_status_.detail = cartesian_.status();
  }
  if (!action.status.empty() && !action.source_control.has_value()) {
    input_status_.detail = action.status;
  }
}

const MotionTargetFrame & KeyboardTargetSource::targetFrame() const noexcept
{
  return cartesian_.frame();
}

const std::vector<ArmTarget> & KeyboardTargetSource::targets() const noexcept
{
  return cartesian_.frame().targets;
}

ArmSide KeyboardTargetSource::selectedSide() const noexcept { return cartesian_.selectedSide(); }

bool KeyboardTargetSource::paused() const noexcept
{
  return paused_;
}

bool KeyboardTargetSource::stopRequested() const noexcept { return stop_requested_; }

const std::string & KeyboardTargetSource::status() const noexcept { return input_status_.detail; }

void KeyboardTargetSource::setStatus(std::string status)
{
  input_status_.detail = std::move(status);
}

void KeyboardTargetSource::setPaused(bool paused, std::string status)
{
  paused_ = paused;
  input_status_.state = paused ? InputState::Paused : InputState::Running;
  input_status_.detail = std::move(status);
}

void KeyboardTargetSource::setMotionInputEnabled(bool enabled, std::string status)
{
  motion_input_enabled_ = enabled;
  motion_input_disabled_status_ = status;
  input_status_.detail = std::move(status);
}

void KeyboardTargetSource::setTargetPose(ArmSide side, const Pose & pose, std::string status)
{
  cartesian_.setTargetPose(side, pose);
  cartesian_.setStatus(status);
  input_status_.detail = std::move(status);
}

std::optional<ArmSide> KeyboardTargetSource::consumeResetRequest()
{
  const auto result = reset_request_;
  reset_request_.reset();
  return result;
}

std::vector<SourceControl> KeyboardTargetSource::consumeSourceControls()
{
  auto result = std::move(source_controls_);
  source_controls_.clear();
  return result;
}

}  // namespace motion_control_lab
