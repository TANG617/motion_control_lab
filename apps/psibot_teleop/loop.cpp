#include "loop.hpp"
#include "tui_projection.hpp"
#include "components/scheduler/single_rate_scheduler.hpp"
#include "components/terminal_frontend/key_router.hpp"
#include "components/terminal_frontend/terminal_frontend.hpp"
#include "components/tui/tui_renderer.hpp"
#include <json/json.h>
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace motion_control_lab::psibot_teleop {
namespace {
std::size_t index(ArmSide side) { return side == ArmSide::Left ? 0 : 1; }
Clock::duration period(double hz) {
  return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / hz));
}
// The SDK writes to process stdout. Keep a separate descriptor for the UI,
// redirect SDK output for the entire session, then restore descriptors once.
class SessionLog {
public:
  explicit SessionLog(const Options & options) {
    const auto stamp = std::chrono::system_clock::now().time_since_epoch().count();
    directory = options.log_dir.empty() ?
      "runs/psibot_teleop/" + std::to_string(stamp) + "-" + std::to_string(::getpid()) : options.log_dir;
    if (!std::filesystem::create_directories(directory))
      throw std::runtime_error("Log directory already exists: " + directory);
    Json::Value config;
    config["app"] = "mcl_psibot_teleop";
    config["address"] = options.address;
    config["mode"] = modeName(options.mode);
    config["side"] = options.teleop.side;
    config["ui"] = options.tui ? "tui" : "none";
    config["step_m"] = options.teleop.step_m;
    config["rotation_step_deg"] = options.teleop.rotation_step_deg;
    config["joint_step_deg"] = options.joint_step_deg;
    config["ui_hz"] = options.ui_hz;
    config["feedback_hz"] = options.feedback_hz;
    config["send_hz"] = options.send_hz;
    config["velocity"] = options.velocity;
    config["acceleration"] = options.acceleration;
    config["jerk"] = options.jerk;
    config["sdk_root"] = MCL_PSIBOT_SDK_ORIGIN;
    config["auth"] = "<redacted>";
    std::ofstream file(directory + "/session.json");
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file << config << '\n';
    stdout_ = ::dup(STDOUT_FILENO);
    stderr_ = ::dup(STDERR_FILENO);
    log_ = ::open((directory + "/sdk.log").c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
    if (stdout_ < 0 || stderr_ < 0 || log_ < 0) {
      const auto message = std::string(std::strerror(errno));
      closeDescriptors();
      throw std::runtime_error(message);
    }
    std::fflush(nullptr);
    if (::dup2(log_, STDOUT_FILENO) < 0 || ::dup2(log_, STDERR_FILENO) < 0) {
      const auto message = std::string(std::strerror(errno));
      restore();
      throw std::runtime_error(message);
    }
  }
  ~SessionLog() { restore(); }
  int output() const { return stdout_; }
  std::string directory;
private:
  void closeDescriptors() {
    for (int * fd : {&stdout_, &stderr_, &log_}) {
      if (*fd >= 0) ::close(*fd);
      *fd = -1;
    }
  }
  void restore() {
    std::fflush(nullptr);
    if (stdout_ >= 0) (void)::dup2(stdout_, STDOUT_FILENO);
    if (stderr_ >= 0) (void)::dup2(stderr_, STDERR_FILENO);
    closeDescriptors();
  }
  int stdout_{-1}, stderr_{-1}, log_{-1};
};
}

Worker::Worker(const Options & options, Client & client) : options_(options), client_(client) {
  snapshot_.mode = options.mode;
}
Worker::~Worker() { requestStop(); if (task_.valid()) task_.wait(); }
void Worker::start() { task_ = std::async(std::launch::async, [this] { run(); }); }
bool Worker::finished() const {
  return task_.valid() && task_.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}
void Worker::join() { if (task_.valid()) task_.get(); }
void Worker::requestStop() {
  { std::lock_guard lock(mutex_); stopping_ = true; pending_targets_ = {}; pending_joints_.reset(); commands_.clear(); }
  wake_.notify_one();
}
std::uint64_t Worker::enqueue(Request request) {
  std::lock_guard lock(mutex_);
  request.serial = ++next_serial_;
  if (request.action != Action::Reset) { pending_targets_ = {}; pending_joints_.reset(); }
  commands_.push_back(request);
  wake_.notify_one();
  return request.serial;
}
void Worker::targets(const Targets & targets) {
  std::lock_guard lock(mutex_);
  if (stopping_ || !snapshot_.publishing || !commands_.empty()) return;
  for (std::size_t i = 0; i < 2; ++i) if (targets[i]) pending_targets_[i] = targets[i];
  wake_.notify_one();
}
Snapshot Worker::snapshot() const { std::lock_guard lock(mutex_); return snapshot_; }
void Worker::joints(const JointTarget & target) {
  std::lock_guard lock(mutex_);
  if (stopping_ || !snapshot_.publishing || !commands_.empty()) return;
  pending_joints_ = target;
  wake_.notify_one();
}
void Worker::event(const std::string & text) {
  std::fprintf(stderr, "teleop: %s\n", text.c_str());
  std::lock_guard lock(mutex_);
  snapshot_.events.push_back(text);
  if (snapshot_.events.size() > 128) snapshot_.events.pop_front();
}
template<class F> void Worker::rpc(const std::string & name, F && operation) {
  { std::lock_guard lock(mutex_); snapshot_.activity = "Waiting: " + name; }
  const auto start = Clock::now();
  const int code = operation();
  const double ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
  {
    std::lock_guard lock(mutex_);
    ++snapshot_.requests;
    snapshot_.last_rpc = name;
    snapshot_.last_code = code;
    snapshot_.rpc_ms = ms;
    snapshot_.maximum_rpc_ms = std::max(snapshot_.maximum_rpc_ms, ms);
    snapshot_.activity = code == 0 ? "Ready" : "Failed: " + name + " code=" + std::to_string(code) + "; exiting";
  }
  std::fprintf(stderr, "rpc %s code=%d elapsed_ms=%.3f\n", name.c_str(), code, ms);
  if (code != 0) throw std::runtime_error(name + " returned SDK error " + std::to_string(code));
}
void Worker::readPoses() {
  Poses poses;
  rpc("getEndPose", [&] { return client_.readPoses(poses); });
  std::lock_guard lock(mutex_);
  snapshot_.feedback.poses = poses;
  snapshot_.pose_read_at = Clock::now();
}
void Worker::readFeedback() {
  Feedback feedback;
  rpc("getJointStates", [&] { return client_.readJoints(feedback); });
  {
    std::lock_guard lock(mutex_);
    feedback.poses = snapshot_.feedback.poses;
    snapshot_.feedback = std::move(feedback);
    snapshot_.joint_read_at = Clock::now();
  }
  readPoses();
  std::lock_guard lock(mutex_);
  ++snapshot_.reads;
}
void Worker::readMetadata() {
  for (int part = 1; part <= 4; ++part) {
    bool enabled = false;
    sdk::ArmProfile profile{};
    rpc("getEnabled(" + std::to_string(part) + ")", [&] { return client_.readEnabled(part, enabled); });
    rpc("getProfile(" + std::to_string(part) + ")", [&] { return client_.readProfile(part, profile); });
    std::lock_guard lock(mutex_);
    snapshot_.enabled[part - 1] = enabled;
    snapshot_.profiles[part - 1] = profile;
  }
}
void Worker::seed(unsigned mask) {
  readFeedback();
  std::lock_guard lock(mutex_);
  snapshot_.seeds = snapshot_.feedback.poses;
  snapshot_.seed_mask = mask;
  snapshot_.joint_seeds = snapshot_.feedback.positions;
  ++snapshot_.seed_revision;
}
void Worker::handle(const Request & request) {
  if (request.action != Action::Reset && request.action != Action::ResetJoints) {
    std::lock_guard lock(mutex_);
    snapshot_.publishing = false;
    pending_targets_ = {};
    pending_joints_.reset();
  }
  switch (request.action) {
    case Action::Pause: event("Sending paused; previously accepted motion may continue"); break;
    case Action::Begin: {
      if (snapshot().mode == Mode::Wbc && !snapshot().owns_wbc) {
        rpc("startWbc", [&] { return client_.startWbc(); });
        std::lock_guard lock(mutex_);
        snapshot_.owns_wbc = true;
      }
      seed(3);
      { std::lock_guard lock(mutex_); snapshot_.publishing = true; snapshot_.submitted = {}; snapshot_.submitted_joints.reset(); }
      event("Sending enabled; targets initialized from newly read actual poses");
      break;
    }
    case Action::ChangeMode:
    case Action::EndWbc: {
      if (snapshot().owns_wbc) {
        // Consume cleanup ownership before calling, so an error is not retried.
        { std::lock_guard lock(mutex_); snapshot_.owns_wbc = false; }
        rpc("stopWbc", [&] { return client_.stopWbc(); });
      }
      if (request.action == Action::ChangeMode) {
        if (request.mode == Mode::Single) {
          rpc("setProfile(" + std::to_string(request.component) + "," +
              std::to_string(static_cast<int>(request.profile)) + ")",
              [&] { return client_.setProfile(request.component, request.profile); });
          readMetadata();
        }
        seed(3);
        std::lock_guard lock(mutex_);
        snapshot_.mode = request.mode;
        snapshot_.submitted = {};
        snapshot_.submitted_joints.reset();
      }
      event("Session selection completed; Space is required to start sending");
      break;
    }
    case Action::Enable:
      rpc("setEnable(" + std::to_string(request.component) + "," + (request.enable ? "true" : "false") + ")",
          [&] { return client_.setEnabled(request.component, request.enable); });
      readMetadata();
      event("Power operation accepted for component " + std::to_string(request.component));
      break;
    case Action::Profile:
      rpc("setProfile(" + std::to_string(request.component) + "," + std::to_string(static_cast<int>(request.profile)) + ")",
          [&] { return client_.setProfile(request.component, request.profile); });
      readMetadata();
      event("Position profile accepted for component " + std::to_string(request.component));
      break;
    case Action::ResetJoints:
      seed(3);
      break;
    case Action::Reset: {
      const auto side = index(request.side);
      seed(1U << side);
      std::lock_guard lock(mutex_);
      if (snapshot_.publishing) pending_targets_[side] = snapshot_.seeds[side];
      break;
    }
  }
  std::lock_guard lock(mutex_);
  snapshot_.completed_request = request.serial;
}
void Worker::run() {
  // Normal and exceptional exit both close the SDK before the process tears down.
  struct Cleanup {
    Worker & owner;
    ~Cleanup() {
      {
        std::lock_guard lock(owner.mutex_);
        owner.snapshot_.publishing = false;
        owner.stopping_ = true;
        owner.pending_targets_ = {};
        owner.pending_joints_.reset();
        owner.commands_.clear();
      }
      if (owner.snapshot().owns_wbc) {
        const int code = owner.client_.stopWbc();
        std::fprintf(stderr, "cleanup stopWbc code=%d\n", code);
      }
      owner.client_.disconnect();
    }
  } cleanup{*this};
  rpc("setControlBackend(Psi)", [&] { return client_.connect(); });
  const auto start = Clock::now();
  while (!client_.connected()) {
    std::unique_lock lock(mutex_);
    if (stopping_) return;
    if (std::chrono::duration<double>(Clock::now() - start).count() >= options_.connect_timeout_s)
      throw std::runtime_error("Psi initial connection timeout: " + options_.address);
    wake_.wait_for(lock, std::chrono::milliseconds(20));
  }
  sdk::RobotLimits limits;
  rpc("getRobotLimits", [&] { return client_.readLimits(limits); });
  readFeedback();
  readMetadata();
  seed(3);
  {
    std::lock_guard lock(mutex_);
    snapshot_.limits = std::move(limits);
    snapshot_.connected = true;
    snapshot_.ready = true;
  }
  event("Connected to Psi; observation only");
  auto next_read = Clock::now() + period(options_.feedback_hz);
  auto next_send = Clock::now();
  auto next_metadata = Clock::now() + std::chrono::seconds(1);
  for (;;) {
    Request command;
    bool has_command = false;
    Targets send;
    std::optional<JointTarget> joint_send;
    Mode mode;
    {
      std::unique_lock lock(mutex_);
      if (stopping_) break;
      snapshot_.elapsed_s = std::chrono::duration<double>(Clock::now() - start).count();
      mode = snapshot_.mode;
      if (!commands_.empty()) {
        command = commands_.front(); commands_.pop_front(); has_command = true;
      } else if (snapshot_.publishing && Clock::now() >= next_send) {
        send.swap(pending_targets_);
        joint_send.swap(pending_joints_);
        if (joint_send && snapshot_.submitted_joints && *joint_send == *snapshot_.submitted_joints)
          joint_send.reset();
        for (std::size_t i = 0; i < 2; ++i) {
          if (send[i] && snapshot_.submitted[i] && send[i]->isApprox(*snapshot_.submitted[i], 1e-12))
            send[i].reset();
        }
      }
    }
    if (!client_.connected()) throw std::runtime_error("Psi disconnected; motion will not be resumed");
    if (has_command) { handle(command); continue; }
    if (joint_send) {
      rpc("moveJ", [&] { return client_.sendJoints(*joint_send); });
      {
        std::lock_guard lock(mutex_);
        snapshot_.submitted_joints = joint_send;
        ++snapshot_.sends;
      }
      event("moveJ accepted: component " + std::to_string(joint_send->component));
      next_send = Clock::now() + period(options_.send_hz);
    }
    if (send[0] || send[1]) {
      rpc(mode == Mode::Single ? "moveEndPose" : "moveWbcPose", [&] { return client_.send(mode, send); });
      {
        std::lock_guard lock(mutex_);
        for (std::size_t i = 0; i < 2; ++i) if (send[i]) snapshot_.submitted[i] = send[i];
        ++snapshot_.sends;
      }
      event(std::string(mode == Mode::Single ? "moveEndPose" : "moveWbcPose") +
            " accepted: " + (send[0] ? "left " : "") + (send[1] ? "right" : ""));
      next_send = Clock::now() + period(options_.send_hz);
    }
    if (Clock::now() >= next_read) { readFeedback(); next_read = Clock::now() + period(options_.feedback_hz); }
    if (Clock::now() >= next_metadata) { readMetadata(); next_metadata = Clock::now() + std::chrono::seconds(1); }
    std::unique_lock lock(mutex_);
    wake_.wait_for(lock, std::chrono::milliseconds(2));
  }
  { std::lock_guard lock(mutex_); snapshot_.publishing = false; }
  if (snapshot().owns_wbc) {
    { std::lock_guard lock(mutex_); snapshot_.owns_wbc = false; }
    rpc("stopWbc", [&] { return client_.stopWbc(); });
  }
  event("Stopped sending; disconnecting (not a braking acknowledgement)");
}

InputController::InputController(const Options & options)
  : cartesian_(options.teleop, {{ArmSide::Left, Pose::Identity()}, {ArmSide::Right, Pose::Identity()}}, true),
    control_part_(options.teleop.side == "right" ? 2 : 1), joint_step_deg_(options.joint_step_deg) {
  if (!std::isfinite(joint_step_deg_) || joint_step_deg_ <= 0)
    throw std::invalid_argument("--joint-step-deg must be finite and positive");
  component_ = control_part_;
}
bool InputController::jointControl() const {
  return latest_.ready && latest_.mode == Mode::Single &&
    latest_.profiles.at(control_part_ - 1) == sdk::ArmProfile::JointPosition;
}
void InputController::synchronize(const Snapshot & snapshot) {
  latest_ = snapshot;
  if (!snapshot.publishing && !starting_) sending_ = false;
  if (snapshot.seed_revision != seed_revision_) {
    for (std::size_t i = 0; i < 2; ++i)
      if (snapshot.seed_mask & (1U << i)) cartesian_.setTargetPose(i == 0 ? ArmSide::Left : ArmSide::Right, snapshot.seeds[i]);
    seed_revision_ = snapshot.seed_revision;
    joint_targets_ = snapshot.joint_seeds;
  }
  if (starting_ && snapshot.completed_request >= pending_) {
    sending_ = snapshot.publishing;
    starting_ = false;
    status_ = "Sending enabled; edit a target to submit";
  }
  if (applying_ && snapshot.completed_request >= pending_) {
    applying_ = false;
    status_ = "Control selected; Space to start";
  }
}
void InputController::pause(Worker & worker) {
  sending_ = starting_ = false;
  pending_ = worker.enqueue({Action::Pause});
  status_ = "Sending paused; accepted motion may continue";
}
bool InputController::handle(const KeyEvent & key, Worker & worker) {
  if (key.code == KeyCode::Character && key.character == 'x') { stopping_ = true; worker.requestStop(); return true; }
  if (menu_) {
    if (key.code == KeyCode::Escape || (key.code == KeyCode::Character && key.character == 'c')) { menu_ = false; return true; }
    if (key.code == KeyCode::ArrowUp) menu_row_ = (menu_row_ + 2) % 3;
    if (key.code == KeyCode::ArrowDown) menu_row_ = (menu_row_ + 1) % 3;
    if (menu_row_ == 1 && component_ > 2) component_ = 1;
    if (menu_row_ != 2) {
      const int count = menu_row_ == 0 ? 4 : 2;
      if (key.code == KeyCode::ArrowLeft) component_ = (component_ + count - 2) % count + 1;
      if (key.code == KeyCode::ArrowRight) component_ = component_ % count + 1;
    }
    if (key.code == KeyCode::Enter && latest_.ready && latest_.completed_request >= pending_) {
      Request request{Action::ChangeMode};
      request.component = component_;
      request.mode = menu_row_ == 2 ? Mode::Wbc : Mode::Single;
      request.profile = menu_row_ == 0 ? sdk::ArmProfile::JointPosition : sdk::ArmProfile::CartesianPosition;
      pending_ = worker.enqueue(request);
      applying_ = true;
      status_ = "Selecting control; please wait";
      if (menu_row_ != 2) {
        control_part_ = component_;
        joint_index_ = 0;
        if (component_ <= 2) {
          TeleopIntent select;
          select.kind = TeleopIntentKind::SelectArm;
          select.side = component_ == 1 ? ArmSide::Left : ArmSide::Right;
          cartesian_.apply(select, 0);
        }
      }
      menu_ = false;
    }
    return true;
  }
  if (key.code == KeyCode::Escape && !keyboard_.capturingText()) { stopping_ = true; worker.requestStop(); return true; }
  if (!keyboard_.capturingText() && key.code == KeyCode::Character && key.character == 'c') {
    if (applying_) return true;
    pause(worker);
    menu_row_ = latest_.mode == Mode::Wbc ? 2 : jointControl() ? 0 : 1;
    component_ = control_part_;
    menu_ = true;
    return true;
  }
  KeyRouter router;
  if (!keyboard_.capturingText() && router.route(key) == KeyRoute::Navigation) return false;
  if (jointControl() && !keyboard_.capturingText()) {
    const auto count = jointCount(control_part_);
    if (key.code == KeyCode::ArrowLeft || key.code == KeyCode::ArrowRight) {
      joint_index_ = (joint_index_ + (key.code == KeyCode::ArrowRight ? 1 : count - 1)) % count;
      status_ = "Selected " + latest_.feedback.names.at(jointOffset(control_part_) + joint_index_);
      return true;
    }
    if (key.code == KeyCode::ArrowUp || key.code == KeyCode::ArrowDown) {
      joint_step_deg_ *= key.code == KeyCode::ArrowUp ? 2.0 : 0.5;
      status_ = "Joint step " + std::to_string(joint_step_deg_) + " deg";
      return true;
    }
    if (key.code == KeyCode::Character && (key.character == 'w' || key.character == 's' || key.character == 'r')) {
      if (!sending_ || latest_.completed_request < pending_) {
        status_ = "Joint editing paused; Space starts sending";
        return true;
      }
      if (key.character == 'r') {
        pending_ = worker.enqueue({Action::ResetJoints});
        status_ = "Reading actual joint positions to reset the edited targets";
        return true;
      }
      const auto offset = jointOffset(control_part_);
      const auto selected = offset + joint_index_;
      const auto & name = latest_.feedback.names.at(selected);
      const double candidate = joint_targets_.at(selected) +
        (key.character == 'w' ? 1.0 : -1.0) * joint_step_deg_ * 3.141592653589793 / 180.0;
      const auto limit = std::find_if(latest_.limits.joints.begin(), latest_.limits.joints.end(),
        [&](const auto & item) { return item.name == name; });
      if (limit == latest_.limits.joints.end() || !std::isfinite(candidate) ||
          candidate < limit->position_lower || candidate > limit->position_upper) {
        status_ = "Joint target rejected by SDK limits: " + name;
        return true;
      }
      joint_targets_[selected] = candidate;
      JointTarget target;
      target.component = control_part_;
      for (std::size_t i = 0; i < count; ++i) target.positions.push_back(joint_targets_.at(offset + i));
      worker.joints(target);
      status_ = name + " edited target " + std::to_string(candidate) + " rad";
      return true;
    }
    if (key.code == KeyCode::Character && std::string("adqenium").find(key.character) != std::string::npos) {
      status_ = "Joint: Left/Right joint; w/s angle; Up/Down step";
      return true;
    }
  }
  auto action = keyboard_.handle(key);
  if (action.source_control == SourceControl::TogglePause) {
    if (sending_ || starting_) pause(worker);
    else if (latest_.ready && latest_.completed_request >= pending_) {
      pending_ = worker.enqueue({Action::Begin}); starting_ = true; status_ = "Starting; reading current actual poses";
    }
    return true;
  }
  if (!action.status.empty()) status_ = action.status;
  if (!action.teleop) return true;
  const auto & intent = *action.teleop;
  if (intent.kind == TeleopIntentKind::SelectArm) {
    if (applying_) return true;
    if (latest_.mode == Mode::Single && intent.side != cartesian_.selectedSide()) {
      pause(worker);
      Request request{Action::ChangeMode};
      request.component = intent.side == ArmSide::Left ? 1 : 2;
      request.profile = sdk::ArmProfile::CartesianPosition;
      pending_ = worker.enqueue(request);
      applying_ = true;
      status_ = "Selecting arm; Space to start when ready";
    }
    cartesian_.apply(intent, 0);
    control_part_ = intent.side == ArmSide::Left ? 1 : 2;
    component_ = control_part_;
    return true;
  }
  const bool motion = intent.kind == TeleopIntentKind::Translate || intent.kind == TeleopIntentKind::Rotate;
  if ((motion || intent.kind == TeleopIntentKind::ResetTarget) &&
      (!sending_ || latest_.completed_request < pending_)) {
    status_ = "Motion editing disabled while sending is paused or an operation is pending";
    return true;
  }
  if (intent.kind == TeleopIntentKind::ResetTarget) {
    Request request{Action::Reset}; request.side = cartesian_.selectedSide();
    pending_ = worker.enqueue(request);
    return true;
  }
  cartesian_.apply(intent, 0);
  status_ = cartesian_.status();
  if (motion) {
    const auto side = index(cartesian_.selectedSide());
    Targets targets;
    targets[side] = cartesian_.frame().targets[side].target_pose;
    worker.targets(targets);
  }
  return true;
}

int runLoop(const Options & options) {
  SessionLog logs(options);
  writeTerminalOutput(logs.output(), "Logs: " + logs.directory + "\n");
  TerminalFrontend terminal({options.tui, options.tui, logs.output()});
  TuiRenderer renderer(options.tui, logs.output());
  auto client = makeSdkClient(options);
  Worker worker(options, *client);
  InputController input(options);
  installRuntimeSignalHandlers();
  SingleRateScheduler scheduler({options.ui_hz, options.duration_s});
  worker.start();
  auto next_plain = Clock::now();
  while (const auto tick = scheduler.next()) {
    if (worker.finished()) break;
    if (tick->update_due) {
      const auto state = worker.snapshot();
      input.synchronize(state);
      for (const auto & key : terminal.poll())
        if (!input.handle(key, worker)) renderer.handleNavigation(key);
      if (input.stopping()) break;
      if (options.tui) renderer.render(makeDocument(options, state, input, logs.directory));
      else if (Clock::now() >= next_plain) {
        writeTerminalOutput(logs.output(), plainStatus(state) + "\n");
        next_plain = Clock::now() + std::chrono::seconds(1);
      }
    }
    scheduler.sleep();
  }
  worker.requestStop();
  while (!worker.finished()) {
    if (options.tui) {
      auto state = worker.snapshot();
      state.activity = "Exiting; " + state.activity;
      renderer.render(makeDocument(options, state, input, logs.directory));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  worker.join();
  return 0;
}
}  // namespace motion_control_lab::psibot_teleop
