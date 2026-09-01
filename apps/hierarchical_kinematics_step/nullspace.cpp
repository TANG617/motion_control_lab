#include "nullspace.hpp"

#include "components/tui/standard_ik_tui.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>
#include <utility>

namespace motion_control_lab::hierarchical_kinematics_step {
namespace {

std::string fixed(double value, int precision = 5) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(precision) << value;
  return output.str();
}

std::string vectorText(const Eigen::Vector3d &value) {
  return fixed(value.x()) + ", " + fixed(value.y()) + ", " + fixed(value.z());
}

std::string yesNo(bool value) { return value ? "yes" : "no"; }

TuiTableColumn textColumn(std::string title, int cell_width = 0) {
  if (cell_width > static_cast<int>(title.size())) {
    title.append(static_cast<std::size_t>(cell_width) - title.size(), ' ');
  }
  return TuiTableColumn{std::move(title), TuiTableAlignment::Left};
}

TuiTableColumn numberColumn(std::string title, int cell_width = 8) {
  if (cell_width > static_cast<int>(title.size())) {
    title.insert(0U, static_cast<std::size_t>(cell_width) - title.size(), ' ');
  }
  return TuiTableColumn{std::move(title), TuiTableAlignment::Right};
}

TuiSection tableSection(std::string title, std::vector<TuiTableColumn> columns,
                        std::vector<std::vector<std::string>> rows,
                        std::size_t column = 0U) {
  TuiSection result;
  result.title = std::move(title);
  result.column = column;
  result.tables.push_back(
      TuiTable{std::move(columns), std::move(rows), TuiTableStyle::Compact});
  return result;
}

std::vector<TuiSection> nullspacePanels(const NullspaceTuiDebug &debug) {
  const std::string held = debug.held_link4_side.has_value()
                               ? armSideName(*debug.held_link4_side)
                               : "none";
  std::vector<TuiSection> sections;
  sections.push_back(tableSection(
      "Control", {textColumn("Metric", 22), textColumn("Value", 20)},
      {{"Selected arm", armSideName(debug.selected_side)},
       {"Edit focus", controlPointName(debug.control_point)},
       {"Held link4", held},
       {"Primary pass",
        debug.primary_attempted ? "attempted" : "not attempted"},
       {"Secondary pass",
        debug.secondary_attempted
            ? (debug.secondary_succeeded ? "succeeded" : "failed")
            : "not attempted"},
       {"Tertiary pass",
        debug.tertiary_attempted
            ? (debug.tertiary_succeeded ? "succeeded" : "failed")
            : "not attempted"},
       {"Highest priority", debug.highest_completed_priority},
       {"Fallback", debug.fallback_priority},
       {"Left primary scale", fixed(debug.left_task_scale)},
       {"Right primary scale", fixed(debug.right_task_scale)}},
      0U));
  sections.push_back(tableSection(
      "TCP hierarchy status",
      {textColumn("Arm"), numberColumn("Position error [m]"),
       numberColumn("Orientation error [rad]")},
      {{"left", fixed(debug.left_tcp_position_error_m),
        fixed(debug.left_tcp_orientation_error_rad)},
       {"right", fixed(debug.right_tcp_position_error_m),
        fixed(debug.right_tcp_orientation_error_rad)},
       {"maximum preservation drift",
        fixed(debug.primary_maximum_position_preservation_drift_mps),
        fixed(debug.primary_maximum_orientation_preservation_drift_radps)},
       {"preservation tolerance",
        fixed(debug.primary_position_preservation_tolerance_mps),
        fixed(debug.primary_orientation_preservation_tolerance_radps)}},
      0U));
  sections.push_back(tableSection(
      "Secondary objectives",
      {textColumn("Objective"), textColumn("Enabled"), numberColumn("Weight"),
       numberColumn("Target error")},
      {{"left link4", yesNo(debug.link4_target.left_enabled),
        fixed(debug.link4_weight, 2),
        debug.link4_target.left_enabled ? fixed(debug.link4_task_error_m)
                                        : "-"},
       {"right link4", yesNo(debug.link4_target.right_enabled),
        fixed(debug.link4_weight, 2),
        debug.link4_target.right_enabled ? fixed(debug.link4_task_error_m)
                                         : "-"},
       {"Yellow posture coupling", "yes", fixed(debug.yellow_weight, 2),
        fixed(debug.yellow_posture_error_rad)},
       {"Terminal", debug.terminal_attempted ? "attempted" : "not attempted",
        "-", debug.terminal_status}},
      1U));
  sections.push_back(tableSection(
      "Link4 target to raw HKS to executed OTG",
      {textColumn("Arm", 5), textColumn("Target xyz [m]", 28),
       textColumn("Raw xyz [m]", 28), textColumn("Executed xyz [m]", 28),
       numberColumn("Raw error [m]"), numberColumn("Executed error [m]")},
      {{"left", vectorText(debug.link4_target.left),
        vectorText(debug.raw_left_link4), vectorText(debug.executed_left_link4),
        fixed(debug.left_link4_raw_error_m),
        fixed(debug.left_link4_executed_error_m)},
       {"right", vectorText(debug.link4_target.right),
        vectorText(debug.raw_right_link4),
        vectorText(debug.executed_right_link4),
        fixed(debug.right_link4_raw_error_m),
        fixed(debug.right_link4_executed_error_m)}},
      1U));
  return sections;
}

} // namespace

const char *controlPointName(ControlPoint control_point) {
  return control_point == ControlPoint::Tcp ? "TCP" : "link4";
}

const char *elbowTeleopEventName(ElbowTeleopEventKind kind) {
  switch (kind) {
  case ElbowTeleopEventKind::Capture:
    return "capture";
  case ElbowTeleopEventKind::Move:
    return "move";
  case ElbowTeleopEventKind::SwitchSide:
    return "switch-side";
  case ElbowTeleopEventKind::Clear:
    return "clear";
  }
  return "unknown";
}

bool link4Enabled(const Link4TargetSnapshot &snapshot, ArmSide side) {
  return side == ArmSide::Left ? snapshot.left_enabled : snapshot.right_enabled;
}

const Eigen::Vector3d &link4Target(const Link4TargetSnapshot &snapshot,
                                  ArmSide side) {
  return side == ArmSide::Left ? snapshot.left : snapshot.right;
}

NullspaceTargetSource::NullspaceTargetSource(
    TerminalFrontend &terminal, KeyboardSourceMode mode,
    CartesianTeleopOptions options, std::vector<ArmTarget> initial_targets,
    const Eigen::Vector3d &initial_left_link4,
    const Eigen::Vector3d &initial_right_link4, bool allow_side_switching,
    bool replay_elbow_teleop_enabled)
    : terminal_(terminal), mode_(mode), keyboard_(mode),
      cartesian_(std::move(options), std::move(initial_targets),
                 allow_side_switching),
      executed_left_link4_(initial_left_link4),
      executed_right_link4_(initial_right_link4),
      replay_elbow_teleop_enabled_(replay_elbow_teleop_enabled) {
  link4_targets_.left = initial_left_link4;
  link4_targets_.right = initial_right_link4;
  input_status_.detail = cartesian_.status();
}

NullspaceTargetSourceUpdate NullspaceTargetSource::poll(double dt) {
  NullspaceTargetSourceUpdate update;
  for (const auto &event : terminal_.poll()) {
    if (!capturingText() && router_.route(event) == KeyRoute::Navigation) {
      update.navigation.push_back(event);
      continue;
    }
    handleSourceEvent(event, dt);
  }
  return update;
}

void NullspaceTargetSource::handleSourceEvent(const KeyEvent &event,
                                              double dt) {
  if (mode_ == KeyboardSourceMode::Replay && replay_elbow_teleop_enabled_ &&
      handleReplayElbowEvent(event, dt)) {
    return;
  }

  if (mode_ == KeyboardSourceMode::Teleop && !capturingText() &&
      event.code == KeyCode::Character) {
    const char key = static_cast<char>(
        std::tolower(static_cast<unsigned char>(event.character)));
    if (key == 'c') {
      if (!motion_input_enabled_) {
        input_status_.detail = motion_input_disabled_status_;
        return;
      }
      control_point_ = control_point_ == ControlPoint::Tcp ? ControlPoint::Link4
                           : ControlPoint::Tcp;
      if (control_point_ == ControlPoint::Link4 &&
          !link4Enabled(link4_targets_, selectedSide())) {
        captureLink4(selectedSide(), ElbowTeleopEventKind::Capture);
      }
      input_status_.detail = std::string{"Editing "} +
                             armSideName(selectedSide()) + " " +
                             controlPointName(control_point_);
      return;
    }
    if (key == 'x' &&
        (link4_targets_.left_enabled || link4_targets_.right_enabled)) {
      clearLink4();
      return;
    }
  }
  apply(keyboard_.handle(event), dt);
}

bool NullspaceTargetSource::capturingText() const noexcept {
  return keyboard_.capturingText() ||
         (mode_ == KeyboardSourceMode::Replay && replay_elbow_teleop_enabled_ &&
          elbow_keyboard_.capturingText());
}

bool NullspaceTargetSource::handleReplayElbowEvent(const KeyEvent &event,
                                                   double dt) {
  if (elbow_keyboard_.capturingText()) {
    apply(elbow_keyboard_.handle(event), dt);
    return true;
  }

  if (event.code == KeyCode::Character) {
    const char key = static_cast<char>(
        std::tolower(static_cast<unsigned char>(event.character)));
    if (key == ' ' || key == '.') {
      apply(keyboard_.handle(event), dt);
      return true;
    }
    if (key == 'c') {
      if (paused_) {
        input_status_.detail =
            "Replay paused; elbow target unchanged until replay resumes";
      } else if (!motion_input_enabled_) {
        input_status_.detail = motion_input_disabled_status_;
      } else if (link4_targets_.left_enabled || link4_targets_.right_enabled) {
        clearLink4();
        control_point_ = ControlPoint::Tcp;
        input_status_.detail = "Disabled replay elbow target";
      } else {
        control_point_ = ControlPoint::Link4;
        captureLink4(selectedSide(), ElbowTeleopEventKind::Capture);
        input_status_.detail = std::string{"Enabled live replay "} +
                               armSideName(selectedSide()) + " elbow target";
      }
      return true;
    }
    if (key == 'x') {
      if (link4_targets_.left_enabled || link4_targets_.right_enabled) {
        if (paused_) {
          input_status_.detail =
              "Replay paused; elbow target unchanged until replay resumes";
        } else if (!motion_input_enabled_) {
          input_status_.detail = motion_input_disabled_status_;
        } else {
          clearLink4();
        }
      } else {
        apply(keyboard_.handle(event), dt);
      }
      return true;
    }
    const bool elbow_character = key == 'w' || key == 's' || key == 'a' ||
                                 key == 'd' || key == 'q' || key == 'e' ||
                                 key == 'r' || key == 'm';
    if (elbow_character) {
      if (control_point_ != ControlPoint::Link4 && key != 'm') {
        input_status_.detail = "Replay owns TCP goals; press c to edit elbow";
      } else {
        apply(elbow_keyboard_.handle(event), dt);
      }
      return true;
    }
  }

  const bool elbow_navigation =
      event.code == KeyCode::ArrowLeft || event.code == KeyCode::ArrowRight ||
      event.code == KeyCode::ArrowUp || event.code == KeyCode::ArrowDown;
  if (elbow_navigation) {
    apply(elbow_keyboard_.handle(event), dt);
    return true;
  }
  if (event.code == KeyCode::Escape) {
    apply(keyboard_.handle(event), dt);
    return true;
  }
  return false;
}

void NullspaceTargetSource::apply(const KeyboardAction &action, double dt) {
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
      input_status_.detail =
          paused_ ? "Replay timeline paused" : "Replay timeline resumed";
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
  if (!action.teleop.has_value()) {
    if (!action.status.empty() && !action.source_control.has_value()) {
      input_status_.detail = action.status;
    }
    return;
  }

  const auto &intent = *action.teleop;
  const bool modifies_link4 = control_point_ == ControlPoint::Link4 &&
      (intent.kind == TeleopIntentKind::SelectArm ||
       intent.kind == TeleopIntentKind::Translate ||
       intent.kind == TeleopIntentKind::ResetTarget);
  const bool disabled_teleop_motion =
      mode_ == KeyboardSourceMode::Teleop &&
      intent.kind != TeleopIntentKind::SelectArm;
  if (!motion_input_enabled_ && (disabled_teleop_motion || modifies_link4)) {
    input_status_.detail = motion_input_disabled_status_;
    return;
  }
  if (mode_ == KeyboardSourceMode::Replay && paused_ && modifies_link4) {
    input_status_.detail =
        "Replay paused; elbow target unchanged until replay resumes";
    return;
  }

  if (control_point_ == ControlPoint::Link4) {
    if (intent.kind == TeleopIntentKind::SelectArm) {
      const ArmSide previous_side = selectedSide();
      cartesian_.apply(intent, dt);
      if (selectedSide() != previous_side ||
          !link4Enabled(link4_targets_, selectedSide())) {
        captureLink4(selectedSide(), ElbowTeleopEventKind::SwitchSide);
      }
    } else if (intent.kind == TeleopIntentKind::Translate) {
      if (!link4Enabled(link4_targets_, selectedSide())) {
        captureLink4(selectedSide(), ElbowTeleopEventKind::Capture);
      }
      const double scale = intent.discrete ? cartesian_.stepMetres() : dt;
      mutableLink4Target(selectedSide()) += intent.translation * scale;
      ++link4_targets_.revision;
      recordElbowEvent(ElbowTeleopEventKind::Move, selectedSide(),
                       link4Target(link4_targets_, selectedSide()));
      input_status_.detail =
          std::string{"Moved "} + armSideName(selectedSide()) + " link4";
    } else if (intent.kind == TeleopIntentKind::ResetTarget) {
      captureLink4(selectedSide(), ElbowTeleopEventKind::Capture);
      input_status_.detail = std::string{"Captured current "} +
                             armSideName(selectedSide()) + " link4";
    } else if (intent.kind == TeleopIntentKind::Rotate ||
               intent.kind == TeleopIntentKind::CycleRotationAxis) {
      input_status_.detail = "Rotation keys apply only to TCP";
    } else {
      cartesian_.apply(intent, dt);
      input_status_.detail = cartesian_.status();
    }
  } else {
    if (mode_ == KeyboardSourceMode::Replay &&
        intent.kind != TeleopIntentKind::SelectArm &&
        intent.kind != TeleopIntentKind::IncreaseStep &&
        intent.kind != TeleopIntentKind::DecreaseStep &&
        intent.kind != TeleopIntentKind::SetStep) {
      input_status_.detail = "Replay owns TCP goals; press c to edit elbow";
      return;
    }
    if (const auto reset = cartesian_.apply(intent, dt)) {
      reset_request_ = reset;
    }
    input_status_.detail = cartesian_.status();
  }
}

void NullspaceTargetSource::captureLink4(ArmSide side,
                                         ElbowTeleopEventKind kind) {
  link4_targets_.left_enabled = side == ArmSide::Left;
  link4_targets_.right_enabled = side == ArmSide::Right;
  mutableLink4Target(side) =
      side == ArmSide::Left ? executed_left_link4_ : executed_right_link4_;
  ++link4_targets_.revision;
  recordElbowEvent(kind, side, link4Target(link4_targets_, side));
}

void NullspaceTargetSource::clearLink4() {
  const auto held_side = heldLink4Side();
  const Eigen::Vector3d held_target =
      held_side.has_value() ? link4Target(link4_targets_, *held_side)
                            : Eigen::Vector3d::Zero();
  link4_targets_.left_enabled = false;
  link4_targets_.right_enabled = false;
  ++link4_targets_.revision;
  recordElbowEvent(ElbowTeleopEventKind::Clear, held_side, held_target);
  input_status_.detail = "Cleared held link4 target; press x again to exit";
}

void NullspaceTargetSource::recordElbowEvent(ElbowTeleopEventKind kind,
                                             std::optional<ArmSide> side,
    const Eigen::Vector3d &target) {
  elbow_events_.push_back({kind, link4_targets_.revision, side, target});
  ++elbow_edit_count_;
}

Eigen::Vector3d &NullspaceTargetSource::mutableLink4Target(ArmSide side) {
  return side == ArmSide::Left ? link4_targets_.left : link4_targets_.right;
}

const MotionTargetFrame &NullspaceTargetSource::targetFrame() const noexcept {
  return cartesian_.frame();
}

const std::vector<ArmTarget> &NullspaceTargetSource::targets() const noexcept {
  return cartesian_.frame().targets;
}

const Link4TargetSnapshot &
NullspaceTargetSource::link4Targets() const noexcept {
  return link4_targets_;
}

ArmSide NullspaceTargetSource::selectedSide() const noexcept {
  return cartesian_.selectedSide();
}

ControlPoint NullspaceTargetSource::controlPoint() const noexcept {
  return control_point_;
}

std::optional<ArmSide> NullspaceTargetSource::heldLink4Side() const noexcept {
  if (link4_targets_.left_enabled) {
    return ArmSide::Left;
  }
  if (link4_targets_.right_enabled) {
    return ArmSide::Right;
  }
  return std::nullopt;
}

double NullspaceTargetSource::stepMetres() const noexcept {
  return cartesian_.stepMetres();
}

bool NullspaceTargetSource::replayElbowTeleopEnabled() const noexcept {
  return replay_elbow_teleop_enabled_;
}

std::size_t NullspaceTargetSource::elbowEditCount() const noexcept {
  return elbow_edit_count_;
}

bool NullspaceTargetSource::paused() const noexcept { return paused_; }

bool NullspaceTargetSource::stopRequested() const noexcept {
  return stop_requested_;
}

const std::string &NullspaceTargetSource::status() const noexcept {
  return input_status_.detail;
}

std::string NullspaceTargetSource::headerContext() const {
  const auto held = heldLink4Side();
  const std::string input_mode =
      mode_ == KeyboardSourceMode::Replay
          ? (replay_elbow_teleop_enabled_ ? "replay+elbow-teleop" : "replay")
          : "keyboard-teleop";
  return "input " + input_mode + " · focus " +
         controlPointName(control_point_) + " · held " +
         (held.has_value() ? armSideName(*held) : "-") + " · step " +
         fixed(stepMetres(), 4) + " m";
}

std::string NullspaceTargetSource::footerHints() const {
  if (mode_ == KeyboardSourceMode::Replay) {
    if (replay_elbow_teleop_enabled_) {
      if (paused_) {
        return "Space resume · . step · elbow edits paused · ? help · Esc exit";
      }
      if (control_point_ == ControlPoint::Link4) {
        return "c disable elbow · ←/→ arm · wasd/qe move · x clear · ? help";
      }
      return "Space pause · . step · c enable elbow · 1–6 pages · ? help";
    }
    return "Space pause · . step · 1–6 pages · ? help · x exit";
  }
  if (control_point_ == ControlPoint::Link4) {
    return "c TCP/link4 · wasd/qe move · r capture · x clear · Esc exit · ? "
           "help";
  }
  return "c TCP/link4 · wasd/qe move · n/i/u rotate · Esc exit · ? help";
}

std::vector<std::string> NullspaceTargetSource::helpLines() const {
  if (mode_ == KeyboardSourceMode::Replay) {
    if (!replay_elbow_teleop_enabled_) {
      return {
          "Space: pause/resume; .: single replay frame; x or Esc: exit",
              "Replay elbow editing is disabled; enable --replay-elbow-teleop on"};
    }
    return {"Replay: Space pauses/resumes; . advances one frame; Esc exits",
            "Elbow: c enables/disables one link4 target; Left/Right selects "
            "its arm",
        "Elbow: w/s +/-x; a/d +/-y; q/e +/-z in base_link",
            "Elbow: Up/Down or m changes step; r captures executed link4; x "
            "clears/exits",
            "Paused replay accepts navigation and replay controls, not elbow "
            "edits"};
  }
  return {"Arrow Left/Right: select arm; c: switch TCP/link4 edit focus",
      "w/s: +/-x; a/d: +/-y; q/e: +/-z; Arrow Up/Down or m: step size",
      "TCP: n selects rotation axis, i/u rotate, r resets from executed FK",
      "link4: r captures executed link4; x clears held link4; Esc exits",
      "A held link4 target stays active after returning to TCP focus"};
}

void NullspaceTargetSource::setExecutedLink4Positions(
    const Eigen::Vector3d &left, const Eigen::Vector3d &right) {
  executed_left_link4_ = left;
  executed_right_link4_ = right;
}

void NullspaceTargetSource::setStatus(std::string status) {
  input_status_.detail = std::move(status);
}

void NullspaceTargetSource::setPaused(bool paused, std::string status) {
  paused_ = paused;
  input_status_.state = paused ? InputState::Paused : InputState::Running;
  input_status_.detail = std::move(status);
}

void NullspaceTargetSource::setMotionInputEnabled(bool enabled,
                                                  std::string status) {
  motion_input_enabled_ = enabled;
  motion_input_disabled_status_ = status;
  input_status_.detail = std::move(status);
}

void NullspaceTargetSource::setTargetPose(ArmSide side, const Pose &pose,
                                          std::string status) {
  cartesian_.setTargetPose(side, pose);
  cartesian_.setStatus(status);
  input_status_.detail = std::move(status);
}

std::optional<ArmSide> NullspaceTargetSource::consumeResetRequest() {
  const auto result = reset_request_;
  reset_request_.reset();
  return result;
}

std::vector<SourceControl> NullspaceTargetSource::consumeSourceControls() {
  auto result = std::move(source_controls_);
  source_controls_.clear();
  return result;
}

std::vector<ElbowTeleopEvent>
NullspaceTargetSource::consumeElbowTeleopEvents() {
  auto result = std::move(elbow_events_);
  elbow_events_.clear();
  return result;
}

TuiPage makeNullspaceTuiPage(const NullspaceTuiDebug &debug) {
  return makeStandardCapabilityPage("Null-space", nullspacePanels(debug));
}

} // namespace motion_control_lab::hierarchical_kinematics_step
