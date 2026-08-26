#include "nullspace.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>
#include <utility>

namespace motion_control_lab::planned_hierarchical_step_otg_nullspace {
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

TuiTableColumn textColumn(std::string title) {
  return TuiTableColumn{std::move(title), TuiTableAlignment::Left};
}

TuiTableColumn numberColumn(std::string title) {
  return TuiTableColumn{std::move(title), TuiTableAlignment::Right};
}

TuiSection tableSection(std::string title,
                        std::vector<TuiTableColumn> columns,
                        std::vector<std::vector<std::string>> rows,
                        std::size_t column = 0U) {
  TuiSection result;
  result.title = std::move(title);
  result.column = column;
  result.tables.push_back(
      TuiTable{std::move(columns), std::move(rows), TuiTableStyle::Compact});
  return result;
}

std::vector<TuiSection> wideSections(const NullspaceTuiDebug &debug) {
  const std::string held = debug.held_link4_side.has_value()
                               ? armSideName(*debug.held_link4_side)
                               : "none";
  std::vector<TuiSection> sections;
  sections.push_back(tableSection(
      "Control",
      {textColumn("Metric"), textColumn("Value")},
      {{"Selected arm", armSideName(debug.selected_side)},
       {"Edit focus", controlPointName(debug.control_point)},
       {"Held link4", held},
       {"Primary pass", debug.primary_attempted ? "attempted" : "not attempted"},
       {"Secondary pass",
        debug.secondary_attempted
            ? (debug.secondary_succeeded ? "succeeded" : "failed")
            : "not attempted"},
       {"Highest priority", debug.highest_completed_priority},
       {"Fallback", debug.fallback_priority},
       {"Left primary scale", fixed(debug.left_task_scale)},
       {"Right primary scale", fixed(debug.right_task_scale)}},
      0U));
  sections.push_back(tableSection(
      "Primary TCP preservation",
      {textColumn("Arm"), numberColumn("Position error [m]"),
       numberColumn("Orientation error [rad]")},
      {{"left", fixed(debug.left_tcp_position_error_m),
        fixed(debug.left_tcp_orientation_error_rad)},
       {"right", fixed(debug.right_tcp_position_error_m),
        fixed(debug.right_tcp_orientation_error_rad)},
       {"maximum drift", fixed(debug.primary_maximum_preservation_drift),
        "tolerance " + fixed(debug.primary_preservation_tolerance)}},
      0U));
  sections.push_back(tableSection(
      "Secondary objectives",
      {textColumn("Objective"), textColumn("Enabled"),
       numberColumn("Weight"), numberColumn("Target error")},
      {{"left link4", yesNo(debug.link4_target.left_enabled),
        fixed(debug.link4_weight, 2),
        debug.link4_target.left_enabled ? fixed(debug.link4_task_error_m) : "-"},
       {"right link4", yesNo(debug.link4_target.right_enabled),
        fixed(debug.link4_weight, 2),
        debug.link4_target.right_enabled ? fixed(debug.link4_task_error_m) : "-"},
       {"Yellow qY posture", "yes", fixed(debug.yellow_weight, 2),
        fixed(debug.yellow_posture_error_rad)},
       {"Tertiary", debug.tertiary_attempted ? "attempted" : "not attempted", "-", "-"},
       {"Terminal", debug.terminal_attempted ? "attempted" : "not attempted",
        "-", debug.terminal_status}},
      1U));
  sections.push_back(tableSection(
      "Link4 target to raw HKS to executed OTG",
      {textColumn("Arm"), textColumn("Target xyz [m]"),
       textColumn("Raw xyz [m]"), textColumn("Executed xyz [m]"),
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

std::vector<TuiSection> standardSections(const NullspaceTuiDebug &debug) {
  auto sections = wideSections(debug);
  for (auto &section : sections) {
    section.column = 0U;
  }
  return sections;
}

std::vector<TuiSection> narrowSections(const NullspaceTuiDebug &debug) {
  const bool left = debug.selected_side == ArmSide::Left;
  const bool enabled = left ? debug.link4_target.left_enabled
                            : debug.link4_target.right_enabled;
  const double link4_error = left ? debug.left_link4_executed_error_m
                                  : debug.right_link4_executed_error_m;
  const double tcp_position_error =
      left ? debug.left_tcp_position_error_m
           : debug.right_tcp_position_error_m;
  const double tcp_orientation_error =
      left ? debug.left_tcp_orientation_error_rad
           : debug.right_tcp_orientation_error_rad;
  const double task_scale =
      left ? debug.left_task_scale : debug.right_task_scale;
  return {tableSection(
      "Selected-side null-space evidence",
      {textColumn("Metric"), textColumn("Value")},
      {{"Arm", armSideName(debug.selected_side)},
       {"Focus", controlPointName(debug.control_point)},
       {"Link4 enabled", yesNo(enabled)},
       {"Link4 executed error [m]", fixed(link4_error)},
       {"TCP position error [m]", fixed(tcp_position_error)},
       {"TCP orientation error [rad]", fixed(tcp_orientation_error)},
       {"Primary scale", fixed(task_scale)},
       {"Primary drift", fixed(debug.primary_maximum_preservation_drift)},
       {"Secondary", debug.secondary_attempted
                         ? (debug.secondary_succeeded ? "succeeded" : "failed")
                         : "not attempted"},
       {"Tertiary", debug.tertiary_attempted ? "attempted" : "not attempted"},
       {"Terminal", debug.terminal_attempted ? "attempted" : "not attempted"}})};
}

} // namespace

const char *controlPointName(ControlPoint control_point) {
  return control_point == ControlPoint::Tcp ? "TCP" : "link4";
}

bool link4Enabled(const Link4TargetSnapshot &snapshot, ArmSide side) {
  return side == ArmSide::Left ? snapshot.left_enabled
                               : snapshot.right_enabled;
}

const Eigen::Vector3d &link4Target(const Link4TargetSnapshot &snapshot,
                                  ArmSide side) {
  return side == ArmSide::Left ? snapshot.left : snapshot.right;
}

NullspaceTargetSource::NullspaceTargetSource(
    TerminalFrontend &terminal, KeyboardSourceMode mode,
    CartesianTeleopOptions options, std::vector<ArmTarget> initial_targets,
    const Eigen::Vector3d &initial_left_link4,
    const Eigen::Vector3d &initial_right_link4, bool allow_side_switching)
    : terminal_(terminal), mode_(mode), keyboard_(mode),
      cartesian_(std::move(options), std::move(initial_targets),
                 allow_side_switching),
      executed_left_link4_(initial_left_link4),
      executed_right_link4_(initial_right_link4) {
  link4_targets_.left = initial_left_link4;
  link4_targets_.right = initial_right_link4;
  input_status_.detail = cartesian_.status();
}

NullspaceTargetSourceUpdate NullspaceTargetSource::poll(double dt) {
  NullspaceTargetSourceUpdate update;
  for (const auto &event : terminal_.poll()) {
    if (!keyboard_.capturingText() &&
        router_.route(event) == KeyRoute::Navigation) {
      update.navigation.push_back(event);
      continue;
    }
    handleSourceEvent(event, dt);
  }
  return update;
}

void NullspaceTargetSource::handleSourceEvent(const KeyEvent &event,
                                              double dt) {
  if (mode_ == KeyboardSourceMode::Teleop && !keyboard_.capturingText() &&
      event.code == KeyCode::Character) {
    const char key = static_cast<char>(
        std::tolower(static_cast<unsigned char>(event.character)));
    if (key == 'c') {
      if (!motion_input_enabled_) {
        input_status_.detail = motion_input_disabled_status_;
        return;
      }
      control_point_ = control_point_ == ControlPoint::Tcp
                           ? ControlPoint::Link4
                           : ControlPoint::Tcp;
      if (control_point_ == ControlPoint::Link4 &&
          !link4Enabled(link4_targets_, selectedSide())) {
        captureLink4(selectedSide());
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
      input_status_.detail = paused_ ? "Replay timeline paused"
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
  if (!action.teleop.has_value()) {
    if (!action.status.empty() && !action.source_control.has_value()) {
      input_status_.detail = action.status;
    }
    return;
  }

  const auto &intent = *action.teleop;
  const bool selection_only = intent.kind == TeleopIntentKind::SelectArm;
  if (!motion_input_enabled_ && !selection_only) {
    input_status_.detail = motion_input_disabled_status_;
    return;
  }

  if (control_point_ == ControlPoint::Link4) {
    if (intent.kind == TeleopIntentKind::SelectArm) {
      const ArmSide previous_side = selectedSide();
      cartesian_.apply(intent, dt);
      if (selectedSide() != previous_side ||
          !link4Enabled(link4_targets_, selectedSide())) {
        captureLink4(selectedSide());
      }
    } else if (intent.kind == TeleopIntentKind::Translate) {
      if (!link4Enabled(link4_targets_, selectedSide())) {
        captureLink4(selectedSide());
      }
      const double scale = intent.discrete ? cartesian_.stepMetres() : dt;
      mutableLink4Target(selectedSide()) += intent.translation * scale;
      ++link4_targets_.revision;
      input_status_.detail = std::string{"Moved "} +
                             armSideName(selectedSide()) + " link4";
    } else if (intent.kind == TeleopIntentKind::ResetTarget) {
      captureLink4(selectedSide());
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
    if (const auto reset = cartesian_.apply(intent, dt)) {
      reset_request_ = reset;
    }
    input_status_.detail = cartesian_.status();
  }
}

void NullspaceTargetSource::captureLink4(ArmSide side) {
  link4_targets_.left_enabled = side == ArmSide::Left;
  link4_targets_.right_enabled = side == ArmSide::Right;
  mutableLink4Target(side) = side == ArmSide::Left ? executed_left_link4_
                                                   : executed_right_link4_;
  ++link4_targets_.revision;
}

void NullspaceTargetSource::clearLink4() {
  link4_targets_.left_enabled = false;
  link4_targets_.right_enabled = false;
  ++link4_targets_.revision;
  input_status_.detail = "Cleared held link4 target; press x again to exit";
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

bool NullspaceTargetSource::paused() const noexcept { return paused_; }

bool NullspaceTargetSource::stopRequested() const noexcept {
  return stop_requested_;
}

const std::string &NullspaceTargetSource::status() const noexcept {
  return input_status_.detail;
}

std::string NullspaceTargetSource::headerContext() const {
  const auto held = heldLink4Side();
  return std::string{"control="} + controlPointName(control_point_) +
         "  held-link4=" + (held.has_value() ? armSideName(*held) : "none");
}

std::string NullspaceTargetSource::footerHints() const {
  if (mode_ == KeyboardSourceMode::Replay) {
    return "Space pause · . step · 1–8 pages · ? help · x exit";
  }
  if (control_point_ == ControlPoint::Link4) {
    return "c TCP/link4 · wasd/qe move · r capture · x clear · Esc exit · ? help";
  }
  return "c TCP/link4 · wasd/qe move · n/i/u rotate · Esc exit · ? help";
}

std::vector<std::string> NullspaceTargetSource::helpLines() const {
  return {
      "Arrow Left/Right: select arm; c: switch TCP/link4 edit focus",
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

TuiPage makeNullspaceTuiPage(const NullspaceTuiDebug &debug) {
  TuiPage page;
  page.title = "Null-space";
  page.responsive_layouts = {
      {121, 0, wideSections(debug), {1, 1}, {}},
      {80, 120, standardSections(debug), {1}, {}},
      {0, 79, narrowSections(debug), {1}, {}}};
  return page;
}

} // namespace motion_control_lab::planned_hierarchical_step_otg_nullspace
