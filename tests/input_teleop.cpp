#include "components/teleop/cartesian/cartesian_teleop.hpp"
#include "components/teleop/keyboard/keyboard_teleop.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace mcl = motion_control_lab;

namespace
{

void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

}  // namespace

int main()
{
  mcl::KeyboardTeleop teleop_keyboard(mcl::KeyboardSourceMode::Teleop);
  const auto move = teleop_keyboard.handle(mcl::KeyEvent::characterKey('w'));
  require(move.teleop.has_value(), "w did not produce teleop intent");
  require(move.teleop->kind == mcl::TeleopIntentKind::Translate, "w intent is not translation");
  require(move.teleop->translation.isApprox(Eigen::Vector3d::UnitX()), "w axis changed");

  const auto pause = teleop_keyboard.handle(mcl::KeyEvent::characterKey(' '));
  require(pause.source_control == mcl::SourceControl::TogglePause, "space did not toggle pause");

  mcl::KeyboardTeleop replay_keyboard(mcl::KeyboardSourceMode::Replay);
  const auto step = replay_keyboard.handle(mcl::KeyEvent::characterKey('.'));
  require(step.source_control == mcl::SourceControl::Step, "replay step key changed");
  require(!step.teleop.has_value(), "replay key leaked a teleop intent");

  const mcl::CartesianTeleopOptions options{"left", 0.01, 0.001, 0.1, 90.0};
  mcl::CartesianTeleop cartesian(
    options, {{mcl::ArmSide::Left, Eigen::Isometry3d::Identity()}}, false);
  cartesian.apply(*move.teleop, 0.25);
  require(
    std::abs(cartesian.frame().targets.at(0).target_pose.translation().x() - 0.01) < 1.0e-12,
    "discrete Cartesian integration changed");

  mcl::TeleopIntent continuous;
  continuous.kind = mcl::TeleopIntentKind::Translate;
  continuous.translation = Eigen::Vector3d::UnitY() * 2.0;
  continuous.discrete = false;
  cartesian.apply(continuous, 0.25);
  require(
    std::abs(cartesian.frame().targets.at(0).target_pose.translation().y() - 0.5) < 1.0e-12,
    "continuous Cartesian integration did not use dt");
  require(cartesian.frame().revision == 2U, "target revision did not track integration");
  return EXIT_SUCCESS;
}
