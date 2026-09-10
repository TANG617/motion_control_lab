#include "loop.hpp"
#include "tui_projection.hpp"
#include <atomic>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace app = motion_control_lab::psibot_teleop;
namespace mcl = motion_control_lab;
using namespace std::chrono_literals;
void require(bool ok, const char * message) { if (!ok) throw std::runtime_error(message); }
template<class F> void until(F check) {
  const auto deadline = app::Clock::now() + 4s;
  while (!check()) {
    if (app::Clock::now() >= deadline) throw std::runtime_error("test wait timed out");
    std::this_thread::sleep_for(2ms);
  }
}
class Fake final : public app::Client {
public:
  std::atomic<int> fail_send{0}, attempts{0}, wbc_starts{0}, wbc_stops{0}, disconnects{0};
  std::mutex mutex;
  std::vector<app::Targets> sent;
  std::vector<app::JointTarget> joint_sent;
  std::atomic<int> joint_part{0}, fail_profile{0};
  std::vector<std::string> operations;
  int connect() override { return 0; }
  bool connected() override { return true; }
  void disconnect() override { ++disconnects; }
  int readPoses(app::Poses & out) override {
    out = {mcl::Pose::Identity(), mcl::Pose::Identity()};
    out[0].translation() = Eigen::Vector3d(.4, .2, .5);
    out[1].translation() = Eigen::Vector3d(.4, -.2, .5);
    return 0;
  }
  int readJoints(app::Feedback & out) override {
    out.names = {"head_yaw_joint", "head_pitch_joint", "torso_yaw_joint", "torso_pitch_joint", "knee_pitch_joint", "ankle_pitch_joint"};
    for (const auto * side : {"left", "right"}) for (int i = 1; i <= 7; ++i)
      out.names.push_back(std::string(side) + "_arm_joint" + std::to_string(i));
    out.positions.assign(20, .1); out.velocities.assign(20, 0);
    out.accelerations.assign(20, 0); out.torques.assign(20, 0); return 0;
  }
  int readLimits(app::sdk::RobotLimits & out) override {
    app::Feedback feedback; readJoints(feedback);
    for (const auto & name : feedback.names) {
      decltype(out.joints)::value_type limit;
      limit.name = name; limit.position_lower = -1; limit.position_upper = 1;
      out.joints.push_back(limit);
    }
    return 0;
  }
  int readEnabled(int, bool & out) override { out = true; return 0; }
  int readProfile(int part, app::sdk::ArmProfile & out) override { out = part == joint_part ? app::sdk::ArmProfile::JointPosition : app::sdk::ArmProfile::CartesianPosition; return 0; }
  int setEnabled(int part, bool enable) override {
    std::lock_guard lock(mutex); operations.push_back("enable:" + std::to_string(part) + ":" + std::to_string(enable)); return 0;
  }
  int setProfile(int part, app::sdk::ArmProfile profile) override {
    if (fail_profile != 0) return fail_profile;
    joint_part = profile == app::sdk::ArmProfile::JointPosition ? part : 0;
    std::lock_guard lock(mutex); operations.push_back("profile:" + std::to_string(part) + ":" + std::to_string(static_cast<int>(profile))); return 0;
  }
  int startWbc() override { ++wbc_starts; std::lock_guard lock(mutex); operations.push_back("startWbc"); return 0; }
  int stopWbc() override { ++wbc_stops; std::lock_guard lock(mutex); operations.push_back("stopWbc"); return 0; }
  int send(app::Mode, const app::Targets & targets) override {
    ++attempts;
    if (fail_send != 0) return fail_send;
    std::lock_guard lock(mutex); sent.push_back(targets); return 0;
  }
  std::vector<app::Targets> sends() { std::lock_guard lock(mutex); return sent; }
  int sendJoints(const app::JointTarget & target) override {
    ++attempts;
    if (fail_send != 0) return fail_send;
    std::lock_guard lock(mutex); joint_sent.push_back(target); return 0;
  }
  std::vector<app::JointTarget> jointSends() { std::lock_guard lock(mutex); return joint_sent; }
};
void controlAndProjection() {
  app::Options options; options.send_hz = 5;
  Fake fake; app::Worker worker(options, fake); app::InputController input(options);
  worker.start(); until([&] { return worker.snapshot().ready; });
  auto sync = [&] { input.synchronize(worker.snapshot()); };
  auto key = [&](char c) { sync(); input.handle(mcl::KeyEvent::characterKey(c), worker); };
  sync();
  key('w');
  require(fake.attempts == 0 && !input.sending(), "paused key caused motion");
  key(' '); until([&] { sync(); return input.sending(); });
  require(fake.operations.empty(), "startup performed a power/profile operation");
  key('w'); until([&] { return fake.sends().size() == 1; });
  const auto first = fake.sends().front();
  require(first[0] && !first[1], "single arm submitted the wrong sides");
  require(std::abs(first[0]->translation().x() - .405) < 1e-10, "translation not initialized from feedback");
  for (int i = 0; i < 20; ++i) key('w');
  until([&] { return fake.sends().size() == 2; });
  require(std::abs(fake.sends().back()[0]->translation().x() - .505) < 1e-10, "pending targets were not coalesced");
  key('w'); key('s');
  std::this_thread::sleep_for(250ms);
  require(fake.sends().size() == 2, "target reverted to last submitted value was repeated");
  key(' ');
  until([&] { return !worker.snapshot().publishing; });
  key('w'); require(fake.sends().size() == 2, "paused edits accumulated");
  key(' '); until([&] { sync(); return input.sending(); });
  require(std::abs(input.teleop().frame().targets[0].target_pose.translation().x() - .4) < 1e-10,
          "resume did not reinitialize actual pose");
  input.handle({mcl::KeyCode::ArrowRight}, worker);
  require(!input.sending(), "single-arm switch failed to pause");
  until([&] { sync(); return !input.applying(); });
  until([&] { return !worker.snapshot().publishing; });
  key(' '); until([&] { sync(); return input.sending(); });
  key('n'); key('u');
  until([&] { return fake.sends().size() == 3; });
  require(fake.sends().back()[1] && !fake.sends().back()[0], "right arm routing failed");
  const auto right = *fake.sends().back()[1];
  const auto expected = Eigen::AngleAxisd(5 * 3.141592653589793 / 180, Eigen::Vector3d::UnitY()).toRotationMatrix();
  require(right.rotation().isApprox(expected), "rotation should be around selected local TCP axis");
  require(app::fromSdkPose(app::toSdkPose(right)).isApprox(right), "xyzw pose conversion changed pose");
  auto request = app::Request{app::Action::ChangeMode}; request.mode = app::Mode::Wbc;
  auto serial = worker.enqueue(request);
  until([&] { return worker.snapshot().completed_request == serial; });
  serial = worker.enqueue({app::Action::Begin});
  until([&] { return worker.snapshot().completed_request == serial; });
  require(fake.wbc_starts == 1, "WBC not started exactly once");
  app::Targets both;
  both[0] = right; both[1] = right;
  worker.targets(both);
  until([&] { return fake.sends().size() == 4; });
  require(fake.sends().back()[0] && fake.sends().back()[1], "WBC lost dirty sides");
  const auto doc = app::makeDocument(options, worker.snapshot(), input, "/tmp/test-log");
  require(doc.pages.size() == 5 && doc.minimum_width == 48 && doc.minimum_height == 12, "TUI page contract");
  require(doc.pages[2].sections[0].lines[1].find("placeholders") != std::string::npos, "torque placeholders presented as real feedback");
  request = {app::Action::Profile}; request.component = 2; request.profile = app::sdk::ArmProfile::CartesianPosition;
  serial = worker.enqueue(request);
  until([&] { return worker.snapshot().completed_request == serial; });
  require(!worker.snapshot().publishing, "profile operation did not pause publishing");
  request = {app::Action::Enable}; request.component = 2; request.enable = true;
  serial = worker.enqueue(request);
  until([&] { return worker.snapshot().completed_request == serial; });
  require(fake.operations == std::vector<std::string>{"profile:2:3", "startWbc", "profile:2:3", "enable:2:1"}, "operation order changed");
  worker.requestStop(); worker.join();
  require(fake.wbc_stops == 1 && fake.disconnects == 1, "normal shutdown did not stop WBC and disconnect once");
}
void rejectionStopsWorker() {
  app::Options options; options.mode = app::Mode::Wbc;
  Fake fake; app::Worker worker(options, fake);
  worker.start(); until([&] { return worker.snapshot().ready; });
  const auto serial = worker.enqueue({app::Action::Begin});
  until([&] { return worker.snapshot().completed_request == serial; });
  fake.fail_send = -17;
  app::Targets targets; targets[0] = mcl::Pose::Identity();
  worker.targets(targets);
  until([&] { return worker.finished(); });
  worker.targets(targets);
  bool failed = false;
  try { worker.join(); } catch (const std::runtime_error & error) {
    failed = std::string(error.what()).find("moveWbcPose returned SDK error -17") != std::string::npos;
  }
  require(failed, "original SDK operation/code was lost");
  require(fake.attempts == 1 && fake.disconnects == 1 && fake.wbc_stops == 1, "failure retried motion or failed cleanup");
}
void jointJog() {
  app::Options options; options.send_hz = 5;
  Fake fake; fake.joint_part = 1;
  app::Worker worker(options, fake); app::InputController input(options);
  worker.start(); until([&] { return worker.snapshot().ready; });
  auto sync = [&] { input.synchronize(worker.snapshot()); };
  auto key = [&](char c) { sync(); input.handle(mcl::KeyEvent::characterKey(c), worker); };
  key('w'); require(fake.attempts == 0, "paused joint jog moved");
  key(' '); until([&] { sync(); return input.sending(); });
  require(input.jointControl(), "JointPosition did not activate joint keys");
  input.handle({mcl::KeyCode::ArrowRight}, worker);
  key('w'); until([&] { return fake.jointSends().size() == 1; });
  auto target = fake.jointSends().back();
  require(target.component == 1 && target.positions.size() == 7, "moveJ component/DOF mismatch");
  require(std::abs(target.positions[1] - (.1 + 3.141592653589793 / 180)) < 1e-12, "w did not add one degree to selected joint");
  require(target.positions[0] == .1 && fake.sends().empty(), "joint key modified another joint or sent Cartesian target");
  for (int i = 0; i < 10; ++i) key('w');
  until([&] { return fake.jointSends().size() == 2; });
  require(std::abs(fake.jointSends().back().positions[1] - (.1 + 11 * 3.141592653589793 / 180)) < 1e-12, "joint targets not coalesced");
  key('q'); require(fake.sends().empty(), "Cartesian key leaked into joint mode");
  key('r'); until([&] { sync(); return input.jointTargets().at(7) == .1; });
  require(input.sending(), "joint reset incorrectly paused publishing");
  for (int i = 0; i < 100; ++i) key('w');
  require(input.jointTargets().at(7) <= 1 && input.status().find("rejected") != std::string::npos, "joint limit rejection failed");
  key('c'); until([&] { return !worker.snapshot().publishing; });
  const auto paused_count = fake.jointSends().size();
  key('c'); key('w'); std::this_thread::sleep_for(250ms);
  require(fake.jointSends().size() == paused_count, "pending joint target survived pause");
  key(' '); until([&] { sync(); return input.sending(); });
  require(input.jointTargets().at(7) == .1, "resume did not re-read actual joints");
  fake.fail_send = -17; key('s'); until([&] { return worker.finished(); });
  bool failed = false;
  try { worker.join(); } catch (const std::runtime_error & e) { failed = std::string(e.what()).find("moveJ returned SDK error -17") != std::string::npos; }
  require(failed && fake.disconnects == 1, "joint rejection did not stop with original SDK code");
}
void directModes(bool reject_profile = false) {
  app::Options options; options.mode = app::Mode::Wbc;
  Fake fake; app::Worker worker(options, fake); app::InputController input(options);
  worker.start(); until([&] { return worker.snapshot().ready; });
  auto key = [&](mcl::KeyEvent event) { input.synchronize(worker.snapshot()); input.handle(event, worker); };
  auto ch = [&](char c) {
    const auto previous = worker.snapshot().completed_request;
    key(mcl::KeyEvent::characterKey(c));
    if (c == 'c') until([&] { return worker.snapshot().completed_request > previous; });
  };
  auto sync = [&] { input.synchronize(worker.snapshot()); };
  ch(' '); until([&] { sync(); return input.sending(); });
  ch('c'); until([&] { return !worker.snapshot().publishing; });
  require(input.menuRow() == 2, "menu should open on current WBC mode");
  key({mcl::KeyCode::ArrowDown}); // Joint control.
  auto doc = app::makeDocument(options, worker.snapshot(), input, "logs");
  require(doc.modal_sections[0].lines[0] == "> Joint control", "mode focus missing");
  require(doc.modal_sections[0].lines[2] == "  Dual-arm WBC", "three-mode menu missing");
  require(!doc.dim_secondary_text && doc.header_right.empty(), "compact readable header missing");
  fake.fail_profile = reject_profile ? -42 : 0;
  key({mcl::KeyCode::Enter});
  ch('w'); // Switching must never leak motion.
  if (reject_profile) {
    until([&] { return worker.finished(); });
    bool failed = false;
    try { worker.join(); } catch (const std::runtime_error & e) {
      failed = std::string(e.what()).find("setProfile(1,1) returned SDK error -42") != std::string::npos;
    }
    require(failed && fake.attempts == 0 && fake.wbc_stops == 1, "failed switch retried, moved or lost error");
    return;
  }
  until([&] { sync(); return !input.applying(); });
  require(fake.operations == std::vector<std::string>{"startWbc", "stopWbc", "profile:1:1"},
          "must stop WBC before setting single-part profile");
  require(input.jointControl() && !input.sending() && fake.attempts == 0, "mode selection must stay paused");
  ch(' '); until([&] { sync(); return input.sending(); });
  ch('w'); until([&] { return fake.jointSends().size() == 1; });
  ch('c'); until([&] { return !worker.snapshot().publishing; });
  key({mcl::KeyCode::ArrowLeft}); // Wrap left arm to head.
  require(input.component() == 4, "joint menu must support head");
  key({mcl::KeyCode::ArrowDown}); // Cartesian restricts target to arms.
  require(input.component() == 1, "Cartesian must not target head/waist");
  key({mcl::KeyCode::Enter});
  until([&] { sync(); return !input.applying(); });
  require(!input.jointControl() && !input.sending(), "Cartesian selection not applied while paused");
  ch('c'); until([&] { return !worker.snapshot().publishing; });
  key({mcl::KeyCode::ArrowDown});
  key({mcl::KeyCode::Enter});
  until([&] { sync(); return !input.applying(); });
  require(fake.wbc_starts == 1, "WBC started before Space");
  ch(' '); until([&] { sync(); return input.sending(); });
  require(fake.wbc_starts == 2, "WBC did not restart on Space");
  worker.requestStop(); worker.join();
  require(fake.wbc_stops == 2, "exit must end restarted WBC once");
}
int main() {
  controlAndProjection();
  rejectionStopsWorker();
  jointJog();
  directModes();
  directModes(true);
  std::cout << "psibot_teleop state, target, SDK error and projection tests passed\n";
}
