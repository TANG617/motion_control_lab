#include "../nullspace.hpp"

#include <iostream>
#include <stdexcept>

namespace app = motion_control_lab::hierarchical_kinematics_step;
namespace mcl = motion_control_lab;

void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}

mcl::Pose pose(const Eigen::Vector3d &position, double angle,
               const Eigen::Vector3d &axis) {
  auto p = mcl::Pose::Identity();
  p.translation() = position;
  p.linear() = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  return p;
}

int main() {
  try {
    mcl::TerminalFrontend terminal({false, false});
    const auto left = pose({.3, .2, .5}, .4, {1, 2, 3});
    const auto right = pose({.2, -.3, .4}, -.6, {3, 1, 2});
    const auto lo = pose({.02, .01, .08}, .3, {1, 0, 1});
    const auto ro = pose({-.01, .03, .05}, -.2, {0, 1, 1});
    app::NullspaceTargetSource source(
        terminal, mcl::KeyboardSourceMode::Teleop,
        {"right", .005, .001, .5, 5.0},
        {{mcl::ArmSide::Left, left}, {mcl::ArmSide::Right, right}}, {1, 2, 3},
        {-1, -2, -3}, true);
    auto key = [&](char c) {
      source.handleSourceEvent(mcl::KeyEvent::characterKey(c), .01);
    };
    auto goals = [&] {
      return std::pair<mcl::Pose, mcl::Pose>{source.targets()[0].target_pose,
                                             source.targets()[1].target_pose};
    };
    auto unchanged = [&](const auto &before) {
      const auto after = goals();
      require(after.first.isApprox(before.first, 1e-12) &&
                  after.second.isApprox(before.second, 1e-12),
              "mode transition changed goals");
    };
    require(!source.mirrorTcpInput(), "mirror must default off");
    key('c');
    require(source.link4Targets().right_enabled, "right manual target setup");
    source.setLeftElbowOwned(true);
    const auto before = goals();
    source.configureTcpMirror(lo, ro, true);
    for (const std::string name : {"manual", "harp", "recorded"}) {
      source.setElbowReferenceSource(name);
      const auto label = name == "harp" ? std::string("HARP") : name;
      require(source.headerContext().find("L " + label + " / R Yellow") != std::string::npos, "actual configured reference label");
    }
    source.setElbowReferenceSource("harp");

    unchanged(before);
    require(source.mirrorTcpInput() && !source.link4Targets().right_enabled &&
                source.selectedSide() == mcl::ArmSide::Left &&
                source.controlPoint() == app::ControlPoint::Tcp,
            "enable ownership");
    key('b');
    require(source.mirrorTcpInput(), "running toggle must be rejected");
    key('C');
    require(source.controlPoint() == app::ControlPoint::Tcp,
            "mirror cannot edit link4");
    source.handleSourceEvent({mcl::KeyCode::ArrowRight, '\0'}, .01);
    require(source.selectedSide() == mcl::ArmSide::Left,
            "left must remain master");

    Eigen::Matrix3d s = Eigen::Vector3d(1, -1, 1).asDiagonal();
    auto check = [&](const auto &anchors) {
      const mcl::Pose l0 = anchors.first * lo, r0 = anchors.second * ro;
      const auto current = goals();
      const mcl::Pose l = current.first * lo, r = current.second * ro;
      require(((r.translation() - r0.translation()) -
               s * (l.translation() - l0.translation()))
                      .norm() < 1e-12,
              "TCP translation mirror");
      require(r.linear().isApprox(s * l.linear() * l0.linear().transpose() * s *
                                      r0.linear(),
                                  1e-12),
              "TCP orientation mirror with unequal offsets");
      require(std::abs(r.linear().determinant() - 1) < 1e-12,
              "proper right rotation");
    };
    for (char c : std::string("waqininiu")) {
      key(c);
      check(before);
    }
    const auto moved = goals();
    key(' ');
    require(source.status() == "Target publishing paused", "keyboard pause label");
    key('b');
    require(!source.mirrorTcpInput() && source.paused(), "paused toggle off");
    unchanged(moved);
    key('w');
    require(goals().second.isApprox(moved.second, 1e-12),
            "disabled mirror changed right goal");
    const auto reanchors = goals();
    key('b');
    unchanged(reanchors);
    key(' ');
    for (char c : std::string("dqinu")) {
      key(c);
      check(reanchors);
    }
    key('r');
    require(source.consumeResetRequest() == mcl::ArmSide::Left,
            "reset must reach UI FK handler");
    const auto fk_left = pose({.25, .18, .43}, .2, {0, 1, 0});
    const auto fk_right = pose({.26, -.19, .44}, -.3, {1, 0, 0});
    source.resetMirroredTargets(fk_left, fk_right);
    const auto reset = goals();
    require(reset.first.isApprox(fk_left) && reset.second.isApprox(fk_right),
            "both FK reset");
    key('a');
    key('i');
    check(reset);
    key(' ');
    source.setMotionInputEnabled(false, "test fault");
    key('b');
    require(source.mirrorTcpInput(), "fault must reject toggle");
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
