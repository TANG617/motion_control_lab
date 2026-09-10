#include "client.hpp"

namespace motion_control_lab::psibot_teleop {
sdk::PoseArray toSdkPose(const Pose & pose) {
  const Eigen::Quaterniond q(pose.rotation());
  return {pose.translation().x(), pose.translation().y(), pose.translation().z(), q.x(), q.y(), q.z(), q.w()};
}
Pose fromSdkPose(const sdk::PoseArray & values) {
  Pose pose = Pose::Identity();
  pose.translation() = Eigen::Vector3d(values[0], values[1], values[2]);
  pose.linear() = Eigen::Quaterniond(values[6], values[3], values[4], values[5]).toRotationMatrix();
  return pose;
}
namespace {
sdk::RobotComponent component(int value) { return static_cast<sdk::RobotComponent>(value); }
class SdkClient final : public Client {
public:
  explicit SdkClient(const Options & options) : robot_(options.address, options.auth) {
    move_.speed = {sdk::SpeedMode::Relative, options.velocity, options.acceleration, options.jerk};
    move_.pose_frame = sdk::PoseFrame::Base;
  }
  int connect() override { return robot_.setControlBackend(2); }
  bool connected() override { return robot_.isConnected(); }
  void disconnect() override { robot_.disconnect(); }
  int readPoses(Poses & poses) override {
    sdk::PoseArray left{}, right{};
    const auto rc = robot_.getEndPose(left, right, sdk::PoseFrame::Base);
    if (rc == 0) poses = {fromSdkPose(left), fromSdkPose(right)};
    return rc;
  }
  int readJoints(Feedback & out) override {
    return robot_.getJointStates(out.names, out.positions, out.velocities, out.accelerations, out.torques);
  }
  int readLimits(sdk::RobotLimits & out) override { return robot_.getRobotLimits(out); }
  int readEnabled(int part, bool & out) override { return robot_.getEnabled(component(part), out); }
  int readProfile(int part, sdk::ArmProfile & out) override { return robot_.getProfile(component(part), out); }
  int setEnabled(int part, bool value) override { return robot_.setEnable(component(part), value); }
  int setProfile(int part, sdk::ArmProfile value) override { return robot_.setProfile(component(part), value); }
  int startWbc() override { return robot_.startWbc(0); }
  int stopWbc() override { return robot_.stopWbc(); }
  int sendJoints(const JointTarget & target) override {
    sdk::MoveJointOptions options;
    options.speed = move_.speed;
    return robot_.moveJ(component(target.component), target.positions, false, options);
  }
  int send(Mode mode, const Targets & targets) override {
    if (mode == Mode::Single) {
      const auto index = targets[0] ? 0 : 1;
      return robot_.moveEndPose(component(index + 1), toSdkPose(*targets[index]), move_);
    }
    std::optional<sdk::PoseArray> left, right;
    if (targets[0]) left = toSdkPose(*targets[0]);
    if (targets[1]) right = toSdkPose(*targets[1]);
    return robot_.moveWbcPose(left, right, move_);
  }
private:
  sdk::Robot robot_;
  sdk::MoveCartesianOptions move_;
};
}
std::unique_ptr<Client> makeSdkClient(const Options & options) { return std::make_unique<SdkClient>(options); }
}  // namespace motion_control_lab::psibot_teleop
