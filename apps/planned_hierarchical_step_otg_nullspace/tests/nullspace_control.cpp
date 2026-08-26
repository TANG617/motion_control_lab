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
  return EXIT_SUCCESS;
}
