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
       {"Solution quality", debug.solution_quality},
       {"Selected priority", debug.selected_priority},
       {"Highest priority", debug.highest_completed_priority},
       {"Fallback", debug.fallback_priority},
       {"Left correction scale", fixed(debug.left_task_scale)},
       {"Right correction scale", fixed(debug.right_task_scale)},
       {"HQP layout", debug.pose_primary ? "pose-primary" : "position-first"},
       {"Left elbow source", debug.link4_target.left_enabled ? debug.elbow_source : "Yellow posture"},
       {"Left predicted angle [deg]", debug.elbow_source == "manual" ? "-" : fixed(debug.predicted_angles_rad[0] * 180.0 / std::acos(-1.0))},
       {"Right predicted angle [deg]", debug.elbow_source == "manual" ? "-" : fixed(debug.predicted_angles_rad[1] * 180.0 / std::acos(-1.0))},
       {"Reference age [ms]", debug.elbow_source == "manual" ? "-" : fixed(debug.reference_age_ms)},
       {"Source inference [ms]", debug.elbow_source == "manual" ? "-" : fixed(debug.inference_ms)},
       {"Mirror TCP input", debug.mirror_tcp_input ? "ON (left master)" : "OFF"},
       {"Right elbow source", debug.link4_target.right_enabled ? debug.elbow_source : "Yellow posture"}},
      0U));
  sections.push_back(tableSection(
      "TCP hierarchy status",
      {textColumn("Metric", 28), textColumn("Left", 26),
       textColumn("Right", 26)},
      {{"Position error [m]", fixed(debug.left_tcp_position_error_m),
        fixed(debug.right_tcp_position_error_m)},
       {"Orientation error [rad]", fixed(debug.left_tcp_orientation_error_rad),
        fixed(debug.right_tcp_orientation_error_rad)},
       {"Position preservation [m/s]",
        fixed(debug.primary_maximum_position_preservation_drift_mps),
        "maximum across arms"},
       {"Preservation tolerance [m/s]",
        fixed(debug.primary_position_preservation_tolerance_mps),
        debug.pose_primary ? "orientation scaled in Primary" : "orientation soft"},
       {"Orientation preservation [rad/s]",
        fixed(debug.primary_maximum_orientation_preservation_drift_radps),
        fixed(debug.primary_orientation_preservation_tolerance_radps)},
       {"Baseline xyz [m/s]", vectorText(debug.left_baseline_velocity),
        vectorText(debug.right_baseline_velocity)},
       {"Baseline projection max",
        fixed(debug.scale_reference_projection_max_change),
        "joint velocity units"},
       {"Correction s=0 / s=1", "baseline velocity", "target velocity"}},
      0U));
  sections.push_back(tableSection(
      debug.pose_primary ? "Task objectives" : "Secondary objectives",
      {textColumn("Objective"), textColumn("Enabled"), numberColumn("Weight"),
       numberColumn("Target error")},
      {{"left orientation", debug.pose_primary ? "Primary scaled" : "Secondary soft", debug.pose_primary ? "-" : fixed(debug.orientation_weight, 2),
        fixed(debug.left_tcp_orientation_error_rad)},
       {"right orientation", debug.pose_primary ? "Primary scaled" : "Secondary soft", debug.pose_primary ? "-" : fixed(debug.orientation_weight, 2),
        fixed(debug.right_tcp_orientation_error_rad)},
       {"left link4", yesNo(debug.link4_target.left_enabled),
        fixed(debug.link4_weight, 2),
        debug.link4_target.left_enabled ? fixed(debug.left_link4_raw_error_m)
                                        : "-"},
       {"right link4", yesNo(debug.link4_target.right_enabled),
        fixed(debug.link4_weight, 2),
        debug.link4_target.right_enabled ? fixed(debug.right_link4_raw_error_m)
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
  mirror_available_ = allow_side_switching && mode_ == KeyboardSourceMode::Teleop;
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
  if (mirror_available_ && !capturingText() && event.code == KeyCode::Character &&
      (event.character == 'b' || event.character == 'B')) {
    if (!motion_input_enabled_) { input_status_.detail = motion_input_disabled_status_; return; }
    if (!paused_) { input_status_.detail = "Pause target publishing with Space before toggling mirror"; return; }
    setMirrorTcpInput(!mirror_tcp_input_);
    return;
  }
  if (mirror_tcp_input_ && event.code == KeyCode::Character && (event.character == 'c' || event.character == 'C') && !capturingText()) {
    input_status_.detail = "Mirror controls TCP only; elbow ownership follows the selected profile";
    return;
  }
  if ((elbow_owned_[selectedSide() == ArmSide::Left ? 0 : 1] || elbow_reference_source_ != "manual") &&
      event.code == KeyCode::Character && (event.character == 'c' ||
      control_point_ == ControlPoint::Link4)) {
    input_status_.detail = "Elbow editing is disabled: the reference profile owns model/Yellow task selection";
    return;
  }
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
      input_status_.detail = mode_ == KeyboardSourceMode::Replay
          ? (paused_ ? "Replay timeline paused" : "Replay timeline resumed")
          : (paused_ ? "Target publishing paused" : "Target publishing resumed");
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
  if (mirror_tcp_input_ && intent.kind == TeleopIntentKind::SelectArm) {
    input_status_.detail = "Mirror input: left TCP drives right TCP";
    return;
  }

  const bool modifies_link4 = control_point_ == ControlPoint::Link4 &&
      (intent.kind == TeleopIntentKind::SelectArm ||
       intent.kind == TeleopIntentKind::Translate ||
       intent.kind == TeleopIntentKind::ResetTarget);
  const bool disabled_teleop_motion =
      mode_ == KeyboardSourceMode::Teleop &&
      intent.kind != TeleopIntentKind::SelectArm;
  if ((elbow_owned_[selectedSide() == ArmSide::Left ? 0 : 1] || elbow_reference_source_ != "manual") && modifies_link4) {
    input_status_.detail = "Elbow editing is disabled: the reference profile owns model/Yellow task selection";
    return;
  }
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
    if (mirror_tcp_input_ && (intent.kind == TeleopIntentKind::Translate ||
                             intent.kind == TeleopIntentKind::Rotate))
      updateMirroredRightTarget();
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
         fixed(stepMetres(), 4) + " m" +
         (mirror_available_ ? std::string{" · 左右臂镜像 "} + (mirror_tcp_input_ ? "ON" : "OFF") +
          (mirror_tcp_input_ ? (" · L " + (elbow_owned_[0] ? (elbow_reference_source_ == "harp" ? std::string("HARP") : elbow_reference_source_) : std::string("Yellow")) + " / R " + (elbow_owned_[1] ? (elbow_reference_source_ == "harp" ? std::string("HARP") : elbow_reference_source_) : std::string("Yellow"))) : "") : "");
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
  return std::string{mirror_available_ ? "Space pause · b mirror · " : ""} +
      "c TCP/link4 · wasd/qe move · n/i/u rotate · Esc exit · ? help";
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
      "A held link4 target stays active after returning to TCP focus",
      "Nullspace keyboard: Space pauses target publishing; b toggles mirror while paused",
      "Mirror: left TCP drives both goals; r resets both from FK and reanchors"};
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

void NullspaceTargetSource::configureTcpMirror(const Pose &left_tcp_offset,
                                              const Pose &right_tcp_offset,
                                              bool enabled) {
  left_tcp_offset_ = left_tcp_offset;
  right_tcp_offset_ = right_tcp_offset;
  if (enabled) setMirrorTcpInput(true);
}

void NullspaceTargetSource::captureMirrorAnchors() {
  left_tcp_anchor_ = targets().at(0).target_pose * left_tcp_offset_;
  right_tcp_anchor_ = targets().at(1).target_pose * right_tcp_offset_;
}

void NullspaceTargetSource::setMirrorTcpInput(bool enabled) {
  mirror_tcp_input_ = enabled;
  if (enabled) {
    TeleopIntent select;
    select.kind = TeleopIntentKind::SelectArm;
    select.side = ArmSide::Left;
    cartesian_.apply(select, 0.0);
    control_point_ = ControlPoint::Tcp;
    captureMirrorAnchors();
    if (link4_targets_.right_enabled) {
      link4_targets_.right_enabled = false;
      ++link4_targets_.revision;
    }
  }
  input_status_.detail = enabled ? "Mirror ON: left TCP drives right; anchors captured"
                                : "Mirror OFF: both TCP goals retained";
}

void NullspaceTargetSource::updateMirroredRightTarget() {
  const Pose left_tcp = targets().at(0).target_pose * left_tcp_offset_;
  const Eigen::Matrix3d reflection = Eigen::Vector3d(1, -1, 1).asDiagonal();
  Pose right_tcp = Pose::Identity();
  right_tcp.translation() = right_tcp_anchor_.translation() + reflection *
      (left_tcp.translation() - left_tcp_anchor_.translation());
  right_tcp.linear() = reflection * left_tcp.linear() *
      left_tcp_anchor_.linear().transpose() * reflection * right_tcp_anchor_.linear();
  cartesian_.setTargetPose(ArmSide::Right, right_tcp * right_tcp_offset_.inverse());
}

void NullspaceTargetSource::resetMirroredTargets(const Pose &left, const Pose &right) {
  cartesian_.setTargetPose(ArmSide::Left, left);
  cartesian_.setTargetPose(ArmSide::Right, right);
  captureMirrorAnchors();
  input_status_.detail = "Mirror: both TCP targets reset from FK; anchors captured";
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
