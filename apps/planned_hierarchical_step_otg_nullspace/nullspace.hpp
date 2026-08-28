#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "components/teleop/cartesian/cartesian_teleop.hpp"
#include "components/teleop/keyboard/keyboard_teleop.hpp"
#include "components/terminal_frontend/key_router.hpp"
#include "components/terminal_frontend/terminal_frontend.hpp"
#include "contracts/presentation/tui_document.hpp"

namespace motion_control_lab::planned_hierarchical_step_otg_nullspace {

enum class ControlPoint {
  Tcp,
  Link4,
};

const char *controlPointName(ControlPoint control_point);

struct Link4TargetSnapshot {
  std::uint64_t revision{0};
  Eigen::Vector3d left{Eigen::Vector3d::Zero()};
  Eigen::Vector3d right{Eigen::Vector3d::Zero()};
  bool left_enabled{false};
  bool right_enabled{false};
};

bool link4Enabled(const Link4TargetSnapshot &snapshot, ArmSide side);
const Eigen::Vector3d &link4Target(const Link4TargetSnapshot &snapshot,
                                  ArmSide side);

struct NullspaceTargetSourceUpdate {
  std::vector<KeyEvent> navigation;
};

enum class ElbowTeleopEventKind {
  Capture,
  Move,
  SwitchSide,
  Clear,
};

const char *elbowTeleopEventName(ElbowTeleopEventKind kind);

struct ElbowTeleopEvent {
  ElbowTeleopEventKind kind{ElbowTeleopEventKind::Capture};
  std::uint64_t revision{0};
  std::optional<ArmSide> side;
  Eigen::Vector3d target{Eigen::Vector3d::Zero()};
};

class NullspaceTargetSource {
public:
  NullspaceTargetSource(TerminalFrontend &terminal, KeyboardSourceMode mode,
                        CartesianTeleopOptions options,
                        std::vector<ArmTarget> initial_targets,
                        const Eigen::Vector3d &initial_left_link4,
                        const Eigen::Vector3d &initial_right_link4,
                        bool allow_side_switching,
                        bool replay_elbow_teleop_enabled = false);

  NullspaceTargetSourceUpdate poll(double dt);
  void handleSourceEvent(const KeyEvent &event, double dt);
  const MotionTargetFrame &targetFrame() const noexcept;
  const std::vector<ArmTarget> &targets() const noexcept;
  const Link4TargetSnapshot &link4Targets() const noexcept;
  ArmSide selectedSide() const noexcept;
  ControlPoint controlPoint() const noexcept;
  std::optional<ArmSide> heldLink4Side() const noexcept;
  double stepMetres() const noexcept;
  bool replayElbowTeleopEnabled() const noexcept;
  std::size_t elbowEditCount() const noexcept;
  bool paused() const noexcept;
  bool stopRequested() const noexcept;
  const std::string &status() const noexcept;
  std::string headerContext() const;
  std::string footerHints() const;
  std::vector<std::string> helpLines() const;
  void setExecutedLink4Positions(const Eigen::Vector3d &left,
                                 const Eigen::Vector3d &right);
  void setStatus(std::string status);
  void setPaused(bool paused, std::string status);
  void setMotionInputEnabled(bool enabled, std::string status);
  void setTargetPose(ArmSide side, const Pose &pose, std::string status);
  std::optional<ArmSide> consumeResetRequest();
  std::vector<SourceControl> consumeSourceControls();
  std::vector<ElbowTeleopEvent> consumeElbowTeleopEvents();

private:
  bool capturingText() const noexcept;
  bool handleReplayElbowEvent(const KeyEvent &event, double dt);
  void apply(const KeyboardAction &action, double dt);
  void captureLink4(ArmSide side, ElbowTeleopEventKind kind);
  void clearLink4();
  void recordElbowEvent(ElbowTeleopEventKind kind,
                        std::optional<ArmSide> side,
                        const Eigen::Vector3d &target);
  Eigen::Vector3d &mutableLink4Target(ArmSide side);

  TerminalFrontend &terminal_;
  KeyboardSourceMode mode_;
  KeyRouter router_;
  KeyboardTeleop keyboard_;
  KeyboardTeleop elbow_keyboard_{KeyboardSourceMode::Teleop};
  CartesianTeleop cartesian_;
  InputStatus input_status_;
  Link4TargetSnapshot link4_targets_;
  Eigen::Vector3d executed_left_link4_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d executed_right_link4_{Eigen::Vector3d::Zero()};
  ControlPoint control_point_{ControlPoint::Tcp};
  bool paused_{false};
  bool stop_requested_{false};
  bool motion_input_enabled_{true};
  bool replay_elbow_teleop_enabled_{false};
  std::size_t elbow_edit_count_{0U};
  std::string motion_input_disabled_status_{"Motion controls disabled"};
  std::optional<ArmSide> reset_request_;
  std::vector<SourceControl> source_controls_;
  std::vector<ElbowTeleopEvent> elbow_events_;
};

struct NullspaceTuiDebug {
  ArmSide selected_side{ArmSide::Left};
  ControlPoint control_point{ControlPoint::Tcp};
  std::optional<ArmSide> held_link4_side;
  Link4TargetSnapshot link4_target;
  Eigen::Vector3d raw_left_link4{Eigen::Vector3d::Zero()};
  Eigen::Vector3d raw_right_link4{Eigen::Vector3d::Zero()};
  Eigen::Vector3d executed_left_link4{Eigen::Vector3d::Zero()};
  Eigen::Vector3d executed_right_link4{Eigen::Vector3d::Zero()};
  double left_link4_raw_error_m{0.0};
  double right_link4_raw_error_m{0.0};
  double left_link4_executed_error_m{0.0};
  double right_link4_executed_error_m{0.0};
  double left_tcp_position_error_m{0.0};
  double right_tcp_position_error_m{0.0};
  double left_tcp_orientation_error_rad{0.0};
  double right_tcp_orientation_error_rad{0.0};
  double primary_maximum_preservation_drift{0.0};
  double primary_preservation_tolerance{0.0};
  double link4_task_error_m{0.0};
  double yellow_posture_error_rad{0.0};
  double link4_weight{0.0};
  double yellow_weight{0.0};
  double left_task_scale{1.0};
  double right_task_scale{1.0};
  std::string highest_completed_priority{"none"};
  std::string fallback_priority{"none"};
  bool primary_attempted{false};
  bool secondary_attempted{false};
  bool secondary_succeeded{false};
  bool tertiary_attempted{false};
  bool tertiary_succeeded{false};
  bool terminal_attempted{false};
  std::string terminal_status{"not-run"};
};

TuiPage makeNullspaceTuiPage(const NullspaceTuiDebug &debug);

} // namespace motion_control_lab::planned_hierarchical_step_otg_nullspace
