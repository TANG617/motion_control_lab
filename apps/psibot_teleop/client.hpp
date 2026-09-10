#pragma once

#include "options.hpp"
#include "contracts/input/input_contract.hpp"
#include <robot.h>
#include <array>
#include <memory>
#include <optional>
#include <vector>

namespace motion_control_lab::psibot_teleop {
namespace sdk = psibot_robot_sdk;
using Poses = std::array<Pose, 2>;
using Targets = std::array<std::optional<Pose>, 2>;
struct JointTarget {
  int component{1};
  std::vector<double> positions;
  bool operator==(const JointTarget & other) const {
    return component == other.component && positions == other.positions;
  }
};
// SDK public R1 order: head, waist, left arm, right arm.
inline std::size_t jointOffset(int part) { return std::array<std::size_t, 5>{0, 6, 13, 2, 0}.at(part); }
inline std::size_t jointCount(int part) { return std::array<std::size_t, 5>{20, 7, 7, 4, 2}.at(part); }
struct Feedback {
  Poses poses{Pose::Identity(), Pose::Identity()};
  std::vector<std::string> names;
  std::vector<double> positions, velocities, accelerations, torques;
};
// App-local SDK seam: tests implement it; no solver or pipeline lives here.
class Client {
public:
  virtual ~Client() = default;
  virtual int connect() = 0;
  virtual bool connected() = 0;
  virtual void disconnect() = 0;
  virtual int readPoses(Poses &) = 0;
  virtual int readJoints(Feedback &) = 0;
  virtual int readLimits(sdk::RobotLimits &) = 0;
  virtual int readEnabled(int component, bool &) = 0;
  virtual int readProfile(int component, sdk::ArmProfile &) = 0;
  virtual int setEnabled(int component, bool) = 0;
  virtual int setProfile(int component, sdk::ArmProfile) = 0;
  virtual int startWbc() = 0;
  virtual int stopWbc() = 0;
  virtual int send(Mode, const Targets &) = 0;
  virtual int sendJoints(const JointTarget &) = 0;
};
sdk::PoseArray toSdkPose(const Pose &);
Pose fromSdkPose(const sdk::PoseArray &);
std::unique_ptr<Client> makeSdkClient(const Options &);
}  // namespace motion_control_lab::psibot_teleop
