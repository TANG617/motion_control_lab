#include "loop.hpp"
#include "components/teleop/cartesian/cartesian_teleop.hpp"
#include "components/teleop/keyboard/keyboard_teleop.hpp"
#include "components/terminal_frontend/key_router.hpp"
#include "components/terminal_frontend/terminal_frontend.hpp"
#include "components/tui/tui_renderer.hpp"
#include "tui_projection.hpp"
#include "visualization.hpp"
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
#include <unistd.h>
namespace motion_control_lab::joint_path_planning {
namespace {
using Clock = std::chrono::steady_clock;
std::uint64_t nowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}
std::array<double, 3> point(const Pose &p) {
  return {p.translation().x(), p.translation().y(), p.translation().z()};
}
void appendCurve(std::vector<std::array<double, 3>> &out,
                 const Eigen::VectorXd &a, const Eigen::VectorXd &b,
                 Solver &solver, ArmSide side) {
  // Diagnostic FK also depicts a rejected endpoint; it never authorizes
  // execution.
  for (int i = 0; i <= 80; ++i) {
    mcc::RobotState state;
    state.joint_positions = a + (b - a) * (i / 80.0);
    out.push_back(point(solver.tcp(state, side, false)));
  }
}
volatile std::sig_atomic_t interrupted = 0;
void stopSignal(int) { interrupted = 1; }
class Signals {
  using Handler = void (*)(int);
  Handler old_int_, old_term_;

public:
  Signals()
      : old_int_(std::signal(SIGINT, stopSignal)),
        old_term_(std::signal(SIGTERM, stopSignal)) {
    interrupted = 0;
  }
  ~Signals() {
    std::signal(SIGINT, old_int_);
    std::signal(SIGTERM, old_term_);
  }
};
// Keep OMPL/native library messages out of the terminal's alternate screen.
class SessionLog {
  int out_{-1}, err_{-1}, log_{-1};

public:
  explicit SessionLog(const Options &o) {
    out_ = ::dup(1);
    err_ = ::dup(2);
    log_ = ::open((o.output / "native.log").c_str(),
                  O_WRONLY | O_CREAT | O_EXCL, 0600);
    if (out_ < 0 || err_ < 0 || log_ < 0)
      throw std::runtime_error("cannot open session log");
    std::fflush(nullptr);
    if (::dup2(log_, 1) < 0 || ::dup2(log_, 2) < 0)
      throw std::runtime_error("cannot redirect native log");
  }
  ~SessionLog() {
    std::fflush(nullptr);
    ::dup2(out_, 1);
    ::dup2(err_, 2);
    ::close(out_);
    ::close(err_);
    ::close(log_);
  }
  int terminal() const { return out_; }
};
class Observer {
  struct Item {
    Snapshot snapshot;
    std::uint64_t stamp;
  };
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Item> queue_;
  bool stop_{false};
  std::future<void> task_;

public:
  Observer(const Options &o, const mcc::RobotModel &m,
           motion_control::viz::RenderSink &sink) {
    task_ = std::async(std::launch::async, [&, this] {
      sink.open();
      Visualization visualization;
      while (true) {
        Item item;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          cv_.wait(lock, [&] { return stop_ || !queue_.empty(); });
          if (queue_.empty())
            break;
          item = std::move(queue_.front());
          queue_.pop_front();
        }
        sink.write(visualization.render(item.snapshot, o, m, item.stamp));
      }
      sink.flush();
      sink.close();
    });
  }
  ~Observer() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    cv_.notify_one();
    if (task_.valid())
      task_.wait();
  }
  void check() {
    if (task_.valid() &&
        task_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
      task_.get();
  }
  void publish(const Snapshot &s) {
    check();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (queue_.size() >= 128)
        throw std::runtime_error("visualization queue overflow");
      queue_.push_back({s, nowNs()});
    }
    cv_.notify_one();
  }
  void finish() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    cv_.notify_one();
    if (task_.valid())
      task_.get();
  }
};
Json::Value array(const std::vector<double> &v) {
  Json::Value a(Json::arrayValue);
  for (double x : v)
    a.append(x);
  return a;
}
void persist(const Options &o, const Snapshot &s) {
  const auto prefix = o.output / ("attempt-" + std::to_string(s.attempt->id));
  writeJson(prefix.string() + ".json", diagnosticsJson(s));
  if (!s.attempt->motion.accepted)
    return;
  const auto &m = s.attempt->motion;
  std::ofstream out;
  out.exceptions(std::ios::badbit | std::ios::failbit);
  out.open(prefix.string() + "-trajectory.csv");
  out << std::setprecision(17) << "time_s";
  for (const auto &name : m.timed.trajectory.joint_names)
    for (const auto *field : {"q", "v", "a", "jerk"})
      out << ',' << name << '_' << field;
  out << '\n';
  for (const auto &x : m.timed.trajectory.samples) {
    out << x.time_from_start;
    for (std::size_t j = 0; j < x.positions.size(); ++j)
      out << ',' << x.positions[j] << ',' << x.velocities[j] << ','
          << x.accelerations[j] << ',' << x.jerks[j];
    out << '\n';
  }
  std::ofstream path;
  path.exceptions(std::ios::badbit | std::ios::failbit);
  path.open(prefix.string() + "-path.csv");
  path << std::setprecision(17);
  for (std::size_t j = 0; j < m.path.path.joint_names.size(); ++j)
    path << (j ? "," : "") << m.path.path.joint_names[j];
  path << '\n';
  for (const auto &q : m.path.path.waypoints) {
    for (Eigen::Index j = 0; j < q.size(); ++j)
      path << (j ? "," : "") << q[j];
    path << '\n';
  }
}
} // namespace
Attempt generateAttempt(unsigned id, ArmSide side, const Pose &target,
                        const mcc::RobotState &state, Solver &solver,
                        Planning &planning, std::atomic<bool> &cancel,
                        std::atomic<Stage> &stage) {
  const auto request_started = Clock::now();
  Attempt a;
  const auto finish = [&]() {
    a.request_time_ms = std::chrono::duration<double, std::milli>(
                            Clock::now() - request_started)
                            .count();
    return std::move(a);
  };
  a.id = id;
  a.side = side;
  a.target = target;
  stage = Stage::Ik;
  a.ik = solver.solveGoal(state, side, target);
  if (!a.ik.accepted) {
    a.motion.reason = "IK endpoint not converged within pose tolerances";
    return finish();
  }
  if (cancel) {
    a.motion.reason = "Cancelled";
    return finish();
  }
  a.motion = planning.generate(state, a.ik.names, a.ik.positions,
                               a.ik.pose_constraint, cancel, stage);
  a.ik_tcp = {solver.tcp(a.motion.goal, ArmSide::Left, false),
              solver.tcp(a.motion.goal, ArmSide::Right, false)};
  stage = Stage::Preview;
  if (cancel) {
    a.motion.accepted = false;
    a.motion.reason = "Cancelled";
    return finish();
  }
  appendCurve(a.direct_curve, state.joint_positions,
              a.motion.goal.joint_positions, solver, side);
  if (a.motion.accepted)
    for (std::size_t i = 1; i < a.motion.path.path.waypoints.size(); ++i) {
      if (cancel) {
        a.motion.accepted = false;
        a.motion.reason = "Cancelled";
        break;
      }
      appendCurve(a.path_curve, a.motion.path.path.waypoints[i - 1],
                  a.motion.path.path.waypoints[i], solver, side);
    }
  if (a.motion.accepted && a.motion.curve) {
    for (const auto time : a.motion.smoothing.stop_times) {
      mcc::JointTrajectorySample sample;
      require(a.motion.curve->evaluate(time, sample));
      mcc::RobotState q;
      q.joint_positions = Eigen::Map<const Eigen::VectorXd>(
          sample.positions.data(), sample.positions.size());
      a.smooth_stops.push_back(point(solver.tcp(q, side, false)));
    }
    for (const auto &sample : a.motion.timed.trajectory.samples) {
      if (cancel) {
        a.motion.accepted = false;
        a.motion.reason = "Cancelled";
        break;
      }
      mcc::RobotState q;
      q.joint_positions = Eigen::Map<const Eigen::VectorXd>(
          sample.positions.data(), sample.positions.size());
      a.smooth_curve.push_back(point(solver.tcp(q, side, false)));
    }
  }
  if (a.motion.accepted && a.path_curve.empty())
    a.path_curve.push_back(point(a.ik_tcp[side == ArmSide::Left ? 0 : 1]));
  return finish();
}
Json::Value diagnosticsJson(const Snapshot &s) {
  Json::Value j;
  j["state"] = stageName(s.stage);
  j["timing_mode"] = s.timing_mode;
  j["smooth_validation"] = s.smooth_validation;
  j["planning_mode"] = s.whole_body ? "whole-body" : "single-arm";
  j["active_dof"] = s.whole_body ? 16 : 7;
  j["execution_complete"] = s.execution_complete;
  j["side"] = armSideName(s.side);
  j["message"] = s.message;
  j["trajectory_time_s"] = s.progress_s;
  for (int k = 0; k < 3; ++k) {
    j["tcp_xyz"].append(s.actual.translation()[k]);
    j["draft_xyz"].append(s.draft.translation()[k]);
  }
  j["remaining_position_error_m"] =
      (s.actual.translation() - s.draft.translation()).norm();
  for (Eigen::Index i = 0; i < s.state.joint_positions.size(); ++i)
    j["positions"].append(s.state.joint_positions[i]);
  j["planned_velocity"] = array(s.sample.velocities);
  j["planned_acceleration"] = array(s.sample.accelerations);
  j["planned_jerk"] = array(s.sample.jerks);
  if (s.attempt) {
    const auto &a = *s.attempt;
    const auto &m = a.motion;
    j["request_id"] = a.id;
    j["accepted"] = m.accepted;
    j["reason"] = m.reason;
    for (const auto &name : a.ik.names)
      j["active_joint_names"].append(name);
    if (a.ik.pose_constraint) {
      const auto &c = *a.ik.pose_constraint;
      const auto &actual = s.execution_tcp[a.side == ArmSide::Left ? 1 : 0];
      j["held_frame"] = c.frame;
      j["held_position_error_m"] =
          (actual.translation() - c.model_root_T_target.translation()).norm();
      j["held_orientation_error_rad"] =
          Eigen::AngleAxisd(c.model_root_T_target.linear().transpose() *
                            actual.linear())
              .angle();
      j["hold_position_tolerance_m"] = c.position_tolerance_m;
      j["hold_orientation_tolerance_rad"] = c.orientation_tolerance_rad;
      for (int k = 0; k < 3; ++k)
        j["held_target_xyz"].append(c.model_root_T_target.translation()[k]);
      const Eigen::Quaterniond rotation(c.model_root_T_target.linear());
      for (int k = 0; k < 4; ++k)
        j["held_target_quaternion_xyzw"].append(rotation.coeffs()[k]);
    }
    j["ik_accepted"] = a.ik.accepted;
    j["ik_ms"] = a.ik.diagnostics.solve_time_ms;
    j["ik_iterations"] = a.ik.diagnostics.iterations;
    for (const auto &scale : a.ik.diagnostics.optimization.task_scales) {
      Json::Value value;
      value["name"] = scale.name;
      value["active"] = scale.active;
      value["scale"] = scale.scale;
      j["ik_task_scales"].append(value);
    }
    for (const auto &error : a.ik.diagnostics.posture_errors)
      j["ik_posture_error_rad"] = error.maximum_absolute_error_rad;
    if (!a.ik.diagnostics.position_errors.empty())
      j["ik_position_error_m"] =
          a.ik.diagnostics.position_errors.front().norm_m;
    if (!a.ik.diagnostics.orientation_errors.empty())
      j["ik_orientation_error_rad"] =
          a.ik.diagnostics.orientation_errors.front().norm_rad;
    j["scene_id"] = m.planning.scene_id;
    j["scene_revision"] = Json::UInt64(m.planning.scene_revision);
    j["seed"] = m.planning.random_seed;
    j["termination"] = static_cast<int>(m.planning.termination);
    j["backend_status"] = m.planning.backend_status;
    j["termination_reason"] = terminationName(m.planning.termination);
    j["direct_valid"] = m.direct.valid;
    j["waypoints"] = Json::UInt64(m.path.path.waypoints.size());
    j["checked_states"] = Json::UInt64(m.planning.checked_states);
    j["planning_ms"] = m.planning.total_time_ms;
    j["search_ms"] = m.planning.search_time_ms;
    j["simplification_ms"] = m.planning.simplification_time_ms;
    j["simplification_termination"] =
        simplificationTerminationName(m.planning.simplification_termination);
    j["simplification_attempts"] =
        Json::UInt64(m.planning.simplification_attempts);
    j["accepted_shortcuts"] = Json::UInt64(m.planning.accepted_shortcuts);
    j["removed_vertices"] = Json::UInt64(m.planning.removed_vertices);
    j["linear_segments_before_reduction"] =
        Json::UInt64(m.planning.linear_segments_before_reduction);
    j["linear_segments_after_reduction"] = Json::UInt64(m.planning.linear_segments_after_reduction);
    j["vertex_reduction_ms"] = m.planning.vertex_reduction_time_ms;
    j["trajectory_validation_status"] = m.trajectory_validation_status;
    j["trajectory_verification_ms"] = m.trajectory_validation_ms;
    const auto &tv = m.trajectory_validation;
    if (tv.failed_time_s)
      j["trajectory_failed_time_s"] = *tv.failed_time_s;
    if (tv.geometry.violating_pair)
      j["trajectory_violating_pair"] = tv.geometry.violating_pair->first.name +
                                       " / " +
                                       tv.geometry.violating_pair->second.name;
    if (tv.geometry.violating_pair_distance_m)
      j["trajectory_violating_distance_m"] =
          *tv.geometry.violating_pair_distance_m;
    for (Eigen::Index k = 0; k < tv.violating_positions.size(); ++k)
      j["trajectory_violating_positions"].append(tv.violating_positions[k]);
    j["trajectory_interval_count"] =
        Json::UInt64(m.trajectory_validation.geometry.certified_interval_count);
    j["trajectory_collision_queries"] =
        Json::UInt64(m.trajectory_validation.geometry.collision_queries);
    j["smoothing_segments"] = Json::UInt64(m.smoothing.segment_count);
    for (const auto t : m.smoothing.stop_times)
      j["smooth_stop_times"].append(t);
    j["smoothing_scaling_attempts"] =
        Json::UInt64(m.smoothing.scaling_attempts);
    if (m.verified.clearance_lower_bound_m)
      j["path_clearance_lower_bound_m"] = *m.verified.clearance_lower_bound_m;
    if (m.trajectory_validation.geometry.clearance_lower_bound_m)
      j["trajectory_clearance_lower_bound_m"] =
          *m.trajectory_validation.geometry.clearance_lower_bound_m;
    for (Eigen::Index k = 0;
         k < m.trajectory_validation.maximum_deviation_bound.size(); ++k)
      j["maximum_deviation_bound"].append(
          m.trajectory_validation.maximum_deviation_bound[k]);
    if (m.planning.path_length_before_simplification &&
        m.planning.path_length_after_simplification) {
      const double before = *m.planning.path_length_before_simplification;
      const double after = *m.planning.path_length_after_simplification;
      j["path_length_before_simplification"] = before;
      j["path_length_after_simplification"] = after;
      j["path_shortening_percent"] =
          before > 0 ? 100.0 * (before - after) / before : 0.0;
    }
    j["verification_ms"] = m.planning.validation_time_ms;
    j["duration_s"] = m.timing.duration;
    j["request_ms"] = a.request_time_ms;
    j["timing_ms"] = m.timing.calculation_time_ms;
    j["app_verification_ms"] = 0.;
    j["app_path_checks"] = 0;
    j["app_timed_state_checks"] = 0;
    j["through_waypoint_count"] = Json::UInt64(m.timing.through_waypoint_count);
    j["stop_waypoint_count"] =
        Json::UInt64(m.curve ? m.smoothing.stop_times.size()
                             : m.timing.stop_waypoint_indices.size());
    j["stop_waypoint_indices"] = Json::arrayValue;
    for (const auto i : m.timing.stop_waypoint_indices)
      j["stop_waypoint_indices"].append(Json::UInt64(i));
    j["collision_queries"] = Json::UInt64(m.planning.collision_queries);
    j["narrowphase_calls"] = Json::UInt64(m.planning.narrowphase_calls);
    j["broadphase_skips"] = Json::UInt64(m.planning.broadphase_skips);
    j["pose_checks"] = Json::UInt64(m.planning.pose_checks);
    j["velocity_limit_ratio"] = m.timing.maximum_velocity_ratio;
    j["acceleration_limit_ratio"] = m.timing.maximum_acceleration_ratio;
    j["jerk_limit_ratio"] = m.timing.maximum_jerk_ratio;
    if (m.verified.minimum_distance_m)
      j["final_path_sampled_distance_m"] = *m.verified.minimum_distance_m;

    const auto &v = m.accepted ? m.verified : m.planning.last_validation;
    j["validation_reason"] = static_cast<int>(v.reason);
    j["validation_detail"] = validationName(v.reason);
    j["held_path_position_error_bound_m"] = v.position_error_bound_m;
    j["held_path_orientation_error_bound_rad"] = v.orientation_error_bound_rad;
    j["held_path_max_position_error_m"] = v.max_position_error_m;
    j["held_path_max_orientation_error_rad"] = v.max_orientation_error_rad;
    if (v.violating_pair) {
      j["violating_pair"].append(v.violating_pair->first.name);
      j["violating_pair"].append(v.violating_pair->second.name);
    }
    if (v.nearest_pair) {
      j["nearest_pair"].append(v.nearest_pair->first.name);
      j["nearest_pair"].append(v.nearest_pair->second.name);
    }
  }
  for (const auto &e : s.events)
    j["events"].append(e);
  return j;
}
int run(const Options &o, std::shared_ptr<const mcc::RobotModel> model,
        Solver &solver, Planning &planning,
        motion_control::viz::RenderSink &sink) {
  Snapshot s;
  s.whole_body = o.planning_mode == "whole-body";
  s.timing_mode = o.timing_mode;
  s.smooth_validation = o.smooth_validation;
  s.state = initialState(*model, o);
  s.side = parseArmSide(o.side);
  const auto initial = planning.check(s.state);
  Json::Value geometry;
  geometry["geometry_count"] = Json::UInt64(planning.geometry.geometry_count);
  geometry["initial_valid"] = initial.valid;
  geometry["initial_checked_pairs"] = Json::UInt64(initial.checked_pairs);
  for (const auto &name : planning.geometry.links_without_geometry)
    geometry["unmodelled_links"].append(name);
  if (initial.nearest_pair) {
    geometry["nearest_pair"].append(initial.nearest_pair->first.name);
    geometry["nearest_pair"].append(initial.nearest_pair->second.name);
  }
  writeJson(o.output / "geometry.json", geometry);
  if (!initial.valid)
    throw std::runtime_error("Initial scene invalid; see geometry.json (no "
                             "collision exemptions added at runtime)");
  Solver observation(model, s.state, s.whole_body);
  s.actual = observation.tcp(s.state, s.side);
  s.draft = s.actual;
  CartesianTeleopOptions teleop_options;
  teleop_options.side = o.side;
  teleop_options.step_m = .01;
  CartesianTeleop targets(
      teleop_options,
      {{ArmSide::Left, observation.tcp(s.state, ArmSide::Left)},
       {ArmSide::Right, observation.tcp(s.state, ArmSide::Right)}},
      true);
  targets.setTargetPose(s.side, configuredGoal(o.goal, s.actual));
  KeyboardTeleop keyboard(KeyboardSourceMode::Teleop);
  KeyRouter router;
  Signals signals;
  SessionLog log(o);
  TerminalFrontend terminal({!o.headless, !o.headless, log.terminal()});
  TuiRenderer tui(!o.headless, log.terminal());
  Observer observer(o, *model, sink);
  std::atomic<bool> cancel{false};
  std::atomic<Stage> stage{Stage::Idle};
  std::future<Attempt> task;
  struct Join {
    std::atomic<bool> &c;
    std::future<Attempt> &f;
    ~Join() {
      c = true;
      if (f.valid())
        f.wait();
    }
  } join{cancel, task};
  unsigned id = 0;
  bool submitted = false, finished = false, stop = false;
  std::size_t sample_index = 0;
  auto last = Clock::now(), next_ui = last, next_viz = last;
  const auto updateDraft = [&] {
    s.side = targets.selectedSide();
    s.step_m = targets.stepMetres();
    for (const auto &t : targets.frame().targets) {
      s.draft_tcp[t.side == ArmSide::Left ? 0 : 1] = t.target_pose;
      if (t.side == s.side)
        s.draft = t.target_pose;
    }
  };
  const auto submit = [&] {
    finished = false;
    s.execution_complete = false;
    cancel = false;
    stage = Stage::Ik;
    s.attempt.reset();
    s.trail.clear();
    s.progress_s = 0;
    s.sample = {};
    s.message = "Planning submitted";
    s.events.push_back("Request " + std::to_string(++id) + " submitted");
    if (s.events.size() > 20)
      s.events.erase(s.events.begin());
    const auto state = s.state;
    const auto target = s.draft;
    const auto side = s.side;
    const auto request = id;
    task = std::async(std::launch::async, [&, state, target, side, request] {
      return generateAttempt(request, side, target, state, solver, planning,
                             cancel, stage);
    });
    submitted = true;
  };
  while (!stop && !interrupted) {
    const auto now = Clock::now();
    const double dt = std::chrono::duration<double>(now - last).count();
    last = now;
    observer.check();
    updateDraft();
    if (o.headless && !submitted)
      submit();
    for (const auto &key : terminal.poll()) {
      if (!keyboard.capturingText() &&
          router.route(key) == KeyRoute::Navigation) {
        tui.handleNavigation(key);
        continue;
      }
      if (!keyboard.capturingText() &&
          (key.code == KeyCode::Escape ||
           (key.code == KeyCode::Character && key.character == 'x'))) {
        stop = true;
        cancel = true;
        break;
      }
      if (!keyboard.capturingText() && key.code == KeyCode::Character &&
          key.character == 'c' && task.valid()) {
        cancel = true;
        s.message = "Cancellation requested";
        continue;
      }
      if (!keyboard.capturingText() && key.code == KeyCode::Character &&
          key.character == ' ') {
        if (stage == Stage::Executing) {
          stage = Stage::Paused;
          s.events.push_back("Execution clock paused");
        } else if (stage == Stage::Paused) {
          stage = Stage::Executing;
          s.events.push_back("Execution clock resumed");
        }
        continue;
      }
      if (task.valid() || stage == Stage::Executing || stage == Stage::Paused) {
        s.message = "Busy: pause/resume or wait for completion";
        continue;
      }
      if (key.code == KeyCode::Enter && !keyboard.capturingText()) {
        updateDraft();
        submit();
        continue;
      }
      if (key.code == KeyCode::Character && key.character == 'p' &&
          !keyboard.capturingText()) {
        targets.setTargetPose(
            s.side, configuredGoal(o.scene["goals"][armSideName(s.side)],
                                   observation.tcp(s.state, s.side)));
        s.message = "Configured demo target loaded";
        continue;
      }
      auto action = keyboard.handle(key);
      if (action.teleop) {
        const auto reset = targets.apply(*action.teleop, dt);
        if (reset)
          targets.setTargetPose(*reset, observation.tcp(s.state, *reset));
        s.message = targets.status();
      }
      if (!action.status.empty())
        s.message = action.status;
    }
    if (task.valid() &&
        task.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto completed = task.get();
      if (cancel) {
        completed.motion.accepted = false;
        completed.motion.reason = "Cancelled";
      }
      s.attempt = std::make_shared<Attempt>(std::move(completed));
      s.message = s.attempt->motion.reason;
      s.events.push_back(s.message);
      sample_index = 0;
      s.progress_s = 0;
      stage = cancel ? Stage::Cancelled
                     : (s.attempt->motion.accepted ? Stage::Executing
                                                   : Stage::Rejected);
      s.stage = stage.load();
      persist(o, s);
      if (stage != Stage::Executing)
        finished = true;
    }
    if (stage == Stage::Executing) {
      const auto &samples = s.attempt->motion.timed.trajectory.samples;
      if (o.headless && !o.realtime)
        s.progress_s = samples[sample_index].time_from_start;
      else
        s.progress_s += dt;
      while (sample_index < samples.size() &&
             samples[sample_index].time_from_start <= s.progress_s) {
        s.sample = samples[sample_index++];
        s.state.joint_positions = Eigen::Map<const Eigen::VectorXd>(
            s.sample.positions.data(), s.sample.positions.size());
      }
      if (s.attempt->motion.curve) {
        require(s.attempt->motion.curve->evaluate(
            std::min(s.progress_s, s.attempt->motion.curve->duration()),
            s.sample));
        s.state.joint_positions = Eigen::Map<const Eigen::VectorXd>(
            s.sample.positions.data(), s.sample.positions.size());
      }
      if (sample_index == samples.size()) {
        s.progress_s = samples.back().time_from_start;
        stage = Stage::Idle;
        finished = true;
        s.execution_complete = true;
        s.message = "Reached goal";
        s.events.push_back(s.message);
      }
    }
    s.stage = stage.load();
    updateDraft();
    s.execution_tcp = {observation.tcp(s.state, ArmSide::Left),
                       observation.tcp(s.state, ArmSide::Right)};
    s.actual = s.execution_tcp[s.side == ArmSide::Left ? 0 : 1];
    if (s.stage == Stage::Executing || finished) {
      const auto p = point(s.actual);
      if (s.trail.empty() || s.trail.back() != p)
        s.trail.push_back(p);
    }
    if (now >= next_viz || (o.headless && finished)) {
      observer.publish(s);
      next_viz = now + std::chrono::milliseconds(33);
    }
    if (now >= next_ui) {
      tui.render(makeDocument(s, *model, planning));
      next_ui = now + std::chrono::milliseconds(100);
    }
    if (o.headless && finished)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  cancel = true;
  if (task.valid()) {
    auto completed = task.get();
    completed.motion.accepted = false;
    completed.motion.reason = "Cancelled";
    s.attempt = std::make_shared<Attempt>(std::move(completed));
    s.stage = Stage::Cancelled;
    s.message = "Cancelled";
    persist(o, s);
  } else if (s.stage == Stage::Executing || s.stage == Stage::Paused) {
    s.stage = Stage::Stopped;
    s.message = "Execution stopped before goal";
  }
  observer.publish(s);
  observer.finish();
  writeJson(o.output / "status.json", diagnosticsJson(s));
  return o.headless && !s.execution_complete ? 2 : 0;
}
} // namespace motion_control_lab::joint_path_planning
