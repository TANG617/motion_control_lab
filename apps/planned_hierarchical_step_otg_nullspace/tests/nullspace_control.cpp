#include "../nullspace.hpp"

#include <cstdlib>
#include <stdexcept>

namespace app = motion_control_lab::planned_hierarchical_step_otg_nullspace;
namespace mcl = motion_control_lab;

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

mcl::KeyEvent character(char value) {
  return mcl::KeyEvent::characterKey(value);
}

} // namespace

int main() {
  mcl::TerminalFrontend terminal({false, false});
  mcl::CartesianTeleopOptions options{"left", 0.005, 0.001, 0.5, 5.0};
  app::NullspaceTargetSource source(
      terminal, mcl::KeyboardSourceMode::Teleop, options,
      {{mcl::ArmSide::Left, mcl::Pose::Identity()},
       {mcl::ArmSide::Right, mcl::Pose::Identity()}},
      Eigen::Vector3d{1.0, 2.0, 3.0}, Eigen::Vector3d{-1.0, -2.0, -3.0},
      true);

  const auto initial_tcp_revision = source.targetFrame().revision;
  source.handleSourceEvent(character('c'), 0.01);
  require(source.controlPoint() == app::ControlPoint::Link4,
          "c did not select link4 focus");
  require(source.link4Targets().left_enabled &&
              !source.link4Targets().right_enabled,
          "entering link4 did not hold the selected side");

  source.handleSourceEvent(character('w'), 0.01);
  require(source.link4Targets().left.x() == 1.005,
          "link4 translation did not use the configured step");
  require(source.targetFrame().revision == initial_tcp_revision,
          "link4 editing changed the Cartesian target revision");

  source.handleSourceEvent(character('i'), 0.01);
  require(source.link4Targets().left.x() == 1.005,
          "TCP rotation key changed a link4 target");

  source.handleSourceEvent(character('c'), 0.01);
  require(source.controlPoint() == app::ControlPoint::Tcp &&
              source.link4Targets().left_enabled,
          "returning to TCP did not preserve the held link4 target");
  source.handleSourceEvent({mcl::KeyCode::ArrowRight, '\0'}, 0.01);
  require(source.selectedSide() == mcl::ArmSide::Right &&
              source.link4Targets().left_enabled,
          "TCP arm selection changed the held link4 side");

  source.setExecutedLink4Positions(Eigen::Vector3d{4.0, 5.0, 6.0},
                                   Eigen::Vector3d{-4.0, -5.0, -6.0});
  source.handleSourceEvent(character('c'), 0.01);
  require(!source.link4Targets().left_enabled &&
              source.link4Targets().right_enabled &&
              source.link4Targets().right.isApprox(
                  Eigen::Vector3d{-4.0, -5.0, -6.0}),
          "switching link4 side did not transfer and capture the target");

  source.setExecutedLink4Positions(Eigen::Vector3d{7.0, 8.0, 9.0},
                                   Eigen::Vector3d{-7.0, -8.0, -9.0});
  source.handleSourceEvent(character('r'), 0.01);
  require(source.link4Targets().right.isApprox(
              Eigen::Vector3d{-7.0, -8.0, -9.0}),
          "link4 reset did not recapture executed FK");

  source.handleSourceEvent(character('w'), 0.01);
  const Eigen::Vector3d moved_right = source.link4Targets().right;
  source.handleSourceEvent({mcl::KeyCode::ArrowRight, '\0'}, 0.01);
  require(source.link4Targets().right.isApprox(moved_right),
          "reselecting the held side recaptured and lost the edited target");

  source.handleSourceEvent(character('x'), 0.01);
  require(!source.link4Targets().left_enabled &&
              !source.link4Targets().right_enabled &&
              !source.stopRequested(),
          "first x did not clear link4 without exiting");
  source.handleSourceEvent(character('x'), 0.01);
  require(source.stopRequested(), "second x did not exit after link4 clear");

  mcl::TerminalFrontend disabled_teleop_terminal({false, false});
  app::NullspaceTargetSource disabled_teleop(
      disabled_teleop_terminal, mcl::KeyboardSourceMode::Teleop, options,
      {{mcl::ArmSide::Left, mcl::Pose::Identity()},
       {mcl::ArmSide::Right, mcl::Pose::Identity()}},
      Eigen::Vector3d{1.0, 2.0, 3.0}, Eigen::Vector3d{-1.0, -2.0, -3.0},
      true);
  const auto disabled_tcp_revision = disabled_teleop.targetFrame().revision;
  disabled_teleop.setMotionInputEnabled(false, "fault hold");
  disabled_teleop.handleSourceEvent(character('w'), 0.01);
  require(disabled_teleop.targetFrame().revision == disabled_tcp_revision,
          "disabled teleop motion changed the TCP target");

  mcl::TerminalFrontend replay_disabled_terminal({false, false});
  app::NullspaceTargetSource replay_disabled(
      replay_disabled_terminal, mcl::KeyboardSourceMode::Replay, options,
      {{mcl::ArmSide::Left, mcl::Pose::Identity()},
       {mcl::ArmSide::Right, mcl::Pose::Identity()}},
      Eigen::Vector3d{1.0, 2.0, 3.0}, Eigen::Vector3d{-1.0, -2.0, -3.0},
      true, false);
  replay_disabled.setMotionInputEnabled(false,
                                        "Replay motion editing is disabled");
  replay_disabled.handleSourceEvent(character('c'), 0.01);
  replay_disabled.handleSourceEvent(character('w'), 0.01);
  require(replay_disabled.controlPoint() == app::ControlPoint::Tcp &&
              replay_disabled.link4Targets().revision == 0U &&
              replay_disabled.elbowEditCount() == 0U,
          "feature-off replay accepted elbow edits");
  replay_disabled.handleSourceEvent(character(' '), 0.01);
  const auto disabled_controls = replay_disabled.consumeSourceControls();
  require(disabled_controls.size() == 1U &&
              disabled_controls.front() == mcl::SourceControl::TogglePause &&
              replay_disabled.paused(),
          "feature-off replay pause control regressed");

  mcl::TerminalFrontend replay_terminal({false, false});
  app::NullspaceTargetSource replay(
      replay_terminal, mcl::KeyboardSourceMode::Replay, options,
      {{mcl::ArmSide::Left, mcl::Pose::Identity()},
       {mcl::ArmSide::Right, mcl::Pose::Identity()}},
      Eigen::Vector3d{1.0, 2.0, 3.0}, Eigen::Vector3d{-1.0, -2.0, -3.0},
      true, true);
  const auto replay_tcp_revision = replay.targetFrame().revision;
  replay.handleSourceEvent(character('c'), 0.01);
  replay.handleSourceEvent(character('w'), 0.01);
  require(replay.controlPoint() == app::ControlPoint::Link4 &&
              replay.link4Targets().left_enabled &&
              !replay.link4Targets().right_enabled &&
              replay.link4Targets().left.x() == 1.005 &&
              replay.targetFrame().revision == replay_tcp_revision,
          "live replay elbow edit changed the wrong target state");
  auto replay_events = replay.consumeElbowTeleopEvents();
  require(replay_events.size() == 2U &&
              replay_events.at(0).kind == app::ElbowTeleopEventKind::Capture &&
              replay_events.at(1).kind == app::ElbowTeleopEventKind::Move,
          "live replay elbow events were not recorded");

  replay.setExecutedLink4Positions(Eigen::Vector3d{4.0, 5.0, 6.0},
                                   Eigen::Vector3d{-4.0, -5.0, -6.0});
  replay.handleSourceEvent({mcl::KeyCode::ArrowRight, '\0'}, 0.01);
  require(!replay.link4Targets().left_enabled &&
              replay.link4Targets().right_enabled &&
              replay.link4Targets().right.isApprox(
                  Eigen::Vector3d{-4.0, -5.0, -6.0}),
          "live replay arm switch did not preserve single-side activation");
  replay_events = replay.consumeElbowTeleopEvents();
  require(replay_events.size() == 1U &&
              replay_events.front().kind ==
                  app::ElbowTeleopEventKind::SwitchSide,
          "live replay side-switch event was not recorded");

  replay.handleSourceEvent(character('m'), 0.01);
  replay.handleSourceEvent(character('0'), 0.01);
  replay.handleSourceEvent(character('.'), 0.01);
  replay.handleSourceEvent(character('0'), 0.01);
  replay.handleSourceEvent(character('1'), 0.01);
  replay.handleSourceEvent({mcl::KeyCode::Enter, '\0'}, 0.01);
  require(replay.stepMetres() == 0.01 &&
              replay.consumeSourceControls().empty(),
          "replay elbow numeric step entry collided with single-step");

  replay.handleSourceEvent(character(' '), 0.01);
  auto replay_controls = replay.consumeSourceControls();
  require(replay_controls.size() == 1U &&
              replay_controls.front() == mcl::SourceControl::TogglePause &&
              replay.paused(),
          "hybrid replay pause control regressed");
  const auto paused_revision = replay.link4Targets().revision;
  const auto paused_target = replay.link4Targets().right;
  replay.handleSourceEvent(character('w'), 0.01);
  replay.handleSourceEvent(character('r'), 0.01);
  replay.handleSourceEvent(character('c'), 0.01);
  require(replay.link4Targets().revision == paused_revision &&
              replay.link4Targets().right.isApprox(paused_target) &&
              replay.controlPoint() == app::ControlPoint::Link4 &&
              replay.consumeElbowTeleopEvents().empty(),
          "paused replay mutated the elbow target");
  replay.handleSourceEvent(character('.'), 0.01);
  replay_controls = replay.consumeSourceControls();
  require(replay_controls.size() == 1U &&
              replay_controls.front() == mcl::SourceControl::Step,
          "hybrid replay single-frame control regressed");
  replay.handleSourceEvent(character(' '), 0.01);
  (void)replay.consumeSourceControls();
  replay.handleSourceEvent(character('w'), 0.01);
  require(replay.link4Targets().right.x() == paused_target.x() + 0.01,
          "resumed replay did not apply the configured elbow step");

  require(replay.headerContext().find("replay+elbow-teleop") !=
                  std::string::npos &&
              replay.headerContext().find("step=0.0100m") !=
                  std::string::npos &&
              replay.footerHints().find("c disable elbow") !=
                  std::string::npos &&
              replay.helpLines().at(1).find("enables/disables") !=
                  std::string::npos &&
              replay.helpLines().back().find("Paused replay") !=
                  std::string::npos,
          "hybrid replay TUI context is not discoverable");
  replay.handleSourceEvent(character('c'), 0.01);
  require(replay.controlPoint() == app::ControlPoint::Tcp &&
              !replay.link4Targets().left_enabled &&
              !replay.link4Targets().right_enabled &&
              !replay.stopRequested(),
          "hybrid replay c did not disable the elbow target");
  replay_events = replay.consumeElbowTeleopEvents();
  require(replay_events.size() == 2U &&
              replay_events.front().kind ==
                  app::ElbowTeleopEventKind::Move &&
              replay_events.back().kind == app::ElbowTeleopEventKind::Clear,
          "hybrid replay c disable event was not recorded");
  replay.handleSourceEvent(character('c'), 0.01);
  require(replay.controlPoint() == app::ControlPoint::Link4 &&
              !replay.link4Targets().left_enabled &&
              replay.link4Targets().right_enabled,
          "hybrid replay c did not recapture and enable the elbow target");
  replay_events = replay.consumeElbowTeleopEvents();
  require(replay_events.size() == 1U &&
              replay_events.front().kind ==
                  app::ElbowTeleopEventKind::Capture,
          "hybrid replay c enable event was not recorded");
  replay.handleSourceEvent(character('x'), 0.01);
  require(!replay.link4Targets().left_enabled &&
              !replay.link4Targets().right_enabled &&
              !replay.stopRequested(),
          "hybrid replay x did not clear elbow before exit");
  replay.handleSourceEvent(character('x'), 0.01);
  require(replay.stopRequested(), "hybrid replay x did not exit after clear");
  return EXIT_SUCCESS;
}
