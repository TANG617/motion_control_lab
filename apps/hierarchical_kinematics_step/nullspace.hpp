#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <array>
#include <optional>
#include <string>
#include <vector>

#include "components/teleop/cartesian/cartesian_teleop.hpp"
#include "components/teleop/keyboard/keyboard_teleop.hpp"
#include "components/terminal_frontend/key_router.hpp"
#include "components/terminal_frontend/terminal_frontend.hpp"
#include "contracts/presentation/tui_document.hpp"

namespace motion_control_lab::hierarchical_kinematics_step {

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
  void setElbowReferenceSource(std::string source) { elbow_reference_source_ = std::move(source); }
  void setElbowOwned(std::array<bool,2> owned) noexcept { elbow_owned_ = owned; }
  void configureTcpMirror(const Pose &left_tcp_offset, const Pose &right_tcp_offset,
                          bool enabled);
  bool mirrorTcpInput() const noexcept { return mirror_tcp_input_; }
  void resetMirroredTargets(const Pose &left, const Pose &right);
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
  void setMirrorTcpInput(bool enabled);
  void captureMirrorAnchors();
  void updateMirroredRightTarget();
  void recordElbowEvent(ElbowTeleopEventKind kind, std::optional<ArmSide> side,
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
  std::array<bool,2> elbow_owned_{false,false};
  std::string elbow_reference_source_{"manual"};
  bool mirror_available_{false};
  bool mirror_tcp_input_{false};
  Pose left_tcp_offset_{Pose::Identity()}, right_tcp_offset_{Pose::Identity()};
  Pose left_tcp_anchor_{Pose::Identity()}, right_tcp_anchor_{Pose::Identity()};
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
  bool mirror_tcp_input{false};
  bool pose_primary{false};
  std::string elbow_source{"manual"};
  double reference_age_ms{0}, inference_ms{0};
  std::array<double,2> predicted_angles_rad{};
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
  double primary_maximum_position_preservation_drift_mps{0.0};
  double primary_maximum_orientation_preservation_drift_radps{0.0};
  double primary_position_preservation_tolerance_mps{0.0};
  double primary_orientation_preservation_tolerance_radps{0.0};
  double link4_task_error_m{0.0};
  double yellow_posture_error_rad{0.0};
  double orientation_weight{0.0};
  double link4_weight{0.0};
  double yellow_weight{0.0};
  Eigen::VectorXd scale_reference;
  double scale_reference_projection_max_change{0.0};
  Eigen::Vector3d left_baseline_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d right_baseline_velocity{Eigen::Vector3d::Zero()};
  double left_task_scale{1.0};
  double right_task_scale{1.0};
  std::string solution_quality{"not-accepted"};
  std::string selected_priority{"none"};
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

} // namespace motion_control_lab::hierarchical_kinematics_step
