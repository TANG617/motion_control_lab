#pragma once

#include "client.hpp"
#include "components/teleop/keyboard/keyboard_teleop.hpp"
#include "components/teleop/cartesian/cartesian_teleop.hpp"
#include <chrono>
#include <condition_variable>
#include <deque>
#include <future>
#include <mutex>

namespace motion_control_lab::psibot_teleop {
using Clock = std::chrono::steady_clock;
enum class Action { Pause, Begin, ChangeMode, Enable, Profile, Reset, ResetJoints, EndWbc };
struct Request {
  Action action{Action::Pause};
  Mode mode{Mode::Single};
  int component{1};
  bool enable{false};
  sdk::ArmProfile profile{sdk::ArmProfile::JointPosition};
  ArmSide side{ArmSide::Left};
  std::uint64_t serial{0};
};
struct Snapshot {
  bool ready{false}, connected{false}, publishing{false}, owns_wbc{false};
  Mode mode{Mode::Single};
  Feedback feedback;
  sdk::RobotLimits limits;
  std::array<bool, 4> enabled{};
  std::array<sdk::ArmProfile, 4> profiles{};
  Targets submitted;
  std::optional<JointTarget> submitted_joints;
  std::vector<double> joint_seeds;
  Poses seeds{Pose::Identity(), Pose::Identity()};
  unsigned seed_mask{3};
  std::uint64_t seed_revision{0}, completed_request{0};
  std::uint64_t requests{0}, sends{0}, reads{0};
  double rpc_ms{0}, maximum_rpc_ms{0}, elapsed_s{0};
  int last_code{0};
  std::string activity{"Connecting"}, last_rpc;
  Clock::time_point pose_read_at{}, joint_read_at{};
  std::deque<std::string> events;
};

class Worker {
public:
  Worker(const Options &, Client &);
  ~Worker();
  void start();
  std::uint64_t enqueue(Request);
  void targets(const Targets &);
  void joints(const JointTarget &);
  Snapshot snapshot() const;
  void requestStop();
  bool finished() const;
  void join();  // Propagates the worker's original exception.
private:
  void run();
  void event(const std::string &);
  template<class F> void rpc(const std::string &, F &&);
  void readPoses();
  void readFeedback();
  void readMetadata();
  void seed(unsigned mask);
  void handle(const Request &);
  Options options_;
  Client & client_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  Snapshot snapshot_;
  std::deque<Request> commands_;
  Targets pending_targets_;
  std::optional<JointTarget> pending_joints_;
  bool stopping_{false};
  std::uint64_t next_serial_{0};
  std::future<void> task_;
};

// UI state and key interpretation remain independent from the terminal and SDK.
class InputController {
public:
  explicit InputController(const Options &);
  void synchronize(const Snapshot &);
  bool handle(const KeyEvent &, Worker &);  // false means renderer navigation.
  const CartesianTeleop & teleop() const { return cartesian_; }
  bool sending() const { return sending_; }
  bool stopping() const { return stopping_; }
  bool menu() const { return menu_; }
  bool applying() const { return applying_; }
  bool starting() const { return starting_; }
  int menuRow() const { return menu_row_; }
  int component() const { return component_; }
  const std::string & status() const { return status_; }
  bool jointControl() const;
  int controlPart() const { return control_part_; }
  std::size_t selectedJoint() const { return joint_index_; }
  double jointStepDeg() const { return joint_step_deg_; }
  const std::vector<double> & jointTargets() const { return joint_targets_; }
private:
  void pause(Worker &);
  KeyboardTeleop keyboard_{KeyboardSourceMode::Teleop};
  CartesianTeleop cartesian_;
  Snapshot latest_;
  std::uint64_t pending_{0}, seed_revision_{0};
  bool sending_{false}, starting_{false}, stopping_{false}, menu_{false}, applying_{false};
  int menu_row_{0}, component_{1};
  int control_part_{1};
  std::size_t joint_index_{0};
  double joint_step_deg_{1};
  std::vector<double> joint_targets_;
  std::string status_{"Observation only; Space starts sending"};
};
int runLoop(const Options &);
}  // namespace motion_control_lab::psibot_teleop
