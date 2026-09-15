#include "execution.hpp"
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sched.h>
#include <thread>
namespace motion_control_lab::hierarchical_kinematics_step {
namespace {
using Clock = std::chrono::steady_clock;
std::int64_t cpuNs() {
  timespec t{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t))
    throw std::runtime_error("thread CPU clock failed");
  return std::int64_t(t.tv_sec) * 1000000000 + t.tv_nsec;
}
Json::Value vec(const Eigen::VectorXd &v) {
  Json::Value a(Json::arrayValue);
  for (auto x : v)
    a.append(x);
  return a;
}
Eigen::VectorXd eigen(const Json::Value &a) {
  Eigen::VectorXd v(a.size());
  for (Json::ArrayIndex i = 0; i < a.size(); ++i)
    v[i] = a[i].asDouble();
  return v;
}
void pin(int cpu) {
  if (cpu < 0)
    return;
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set))
    throw std::runtime_error("cannot apply declared CPU affinity");
}
Json::Value
taskEvidence(const std::vector<mcc::HierarchicalTaskDiagnostics> &tasks) {
  Json::Value rows(Json::arrayValue);
  for (const auto &t : tasks) {
    Json::Value x;
    x["name"] = t.name;
    x["kind"] = static_cast<int>(t.kind);
    x["priority"] = static_cast<int>(t.priority);
    x["enabled"] = t.enabled;
    x["state"] = static_cast<int>(t.state);
    x["enforcement"] = static_cast<int>(t.enforcement);
    x["residual_norm"] = t.residual_norm;
    x["baseline_velocity"] = vec(t.baseline_velocity);
    x["maximum_violation"] = t.maximum_violation;
    x["objective_cost"] =
        t.objective_cost ? Json::Value(*t.objective_cost) : Json::Value();
    x["target_error_norm"] = t.target_error_norm;
    x["residual_optimum"] = vec(t.residual_optimum);
    x["actual_preservation_drift"] = vec(t.actual_preservation_drift);
    rows.append(x);
  }
  return rows;
}
Json::Value scaleEvidence(
    const std::vector<mcc::HierarchicalTaskScaleDiagnostics> &scales) {
  Json::Value rows(Json::arrayValue);
  for (const auto &t : scales) {
    Json::Value x;
    x["name"] = t.name;
    x["priority"] = static_cast<int>(t.priority);
    x["active"] = t.active;
    x["evaluated"] = t.evaluated;
    x["scale"] = t.weighted_progress_scale;
    x["objective_cost"] = t.objective_cost;
    x["preservation_drift"] = t.actual_preservation_drift;
    x["degraded"] = t.degraded;
    x["stuck"] = t.stuck;
    rows.append(x);
  }
  return rows;
}
Json::Value constraintEvidence(
    const std::vector<mcc::HierarchicalConstraintDiagnostics> &constraints) {
  Json::Value rows(Json::arrayValue);
  for (const auto &t : constraints) {
    Json::Value x;
    x["name"] = t.name;
    x["component_name"] = t.component_name;
    x["unit"] = t.unit;
    x["kind"] = static_cast<int>(t.kind);
    x["bound_source"] = static_cast<int>(t.bound_source);
    x["bound_side"] = static_cast<int>(t.bound_side);
    x["value"] = t.value;
    x["lower"] = t.lower;
    x["upper"] = t.upper;
    x["state"] = static_cast<int>(t.state);
    x["maximum_violation"] = t.maximum_violation;
    x["minimum_slack"] = t.minimum_slack;
    rows.append(x);
  }
  return rows;
}
Json::Value evidence(const mcc::Status &status, const SolverSolution &solution,
                     const SolverDiagnostics &d) {
  Json::Value j;
  j["cpp_new_count"] = d.allocation_observation
                           ? Json::Value(Json::UInt64(d.cpp_new_count))
                           : Json::Value();
  j["cpp_new_bytes"] = d.allocation_observation
                           ? Json::Value(Json::UInt64(d.cpp_new_bytes))
                           : Json::Value();
  j["native_disposition"] = static_cast<int>(d.native_candidate.disposition);
  j["native_status"] = static_cast<int>(status.code);
  j["native_message"] = status.message;
  j["accepted"] = status.ok();
  j["committed"] = status.ok();
  j["candidate_q"] = vec(d.native_candidate.joint_positions);
  j["candidate_v"] = vec(d.native_candidate.joint_velocities);
  j["q"] = vec(solution.kinematics_solution.joint_positions);
  j["v"] = vec(solution.kinematics_solution.joint_velocities);
  j["candidate_available"] = d.native_candidate.joint_positions.size() > 0;
  j["solve_time_ms"] = d.solve_time_ms;
  j["qp_solve_time_ms"] = d.qp_solve_time_ms;
  j["maximum_hard_violation"] = d.maximum_hard_violation;
  j["coupling_state"] = static_cast<int>(d.coupling_state);
  j["proposal_revision"] = Json::UInt64(d.consumed_source_value_revision);
  j["proposal_state_sequence"] = Json::UInt64(d.consumed_source_state_sequence);
  j["proposal_state_time_ns"] =
      Json::Int64(d.consumed_source_state_time_nanoseconds);
  j["state_sequence"] = Json::UInt64(d.captured_state_sequence);
  j["state_time_ns"] = Json::Int64(d.captured_state_time_nanoseconds);
  j["selected_pass"] =
      d.hierarchy.selected_priority
          ? Json::Value(static_cast<int>(*d.hierarchy.selected_priority))
          : Json::Value();
  j["solution_quality"] = static_cast<int>(d.hierarchy.solution_quality);
  j["task_scale_reference"] = vec(d.hierarchy.task_scale_reference);
  j["tasks"] = taskEvidence(d.hierarchy.tasks);
  j["scales"] = scaleEvidence(d.hierarchy.task_scales);
  j["constraints"] = constraintEvidence(d.hierarchy.constraints);
  for (const auto &p : d.hierarchy.passes) {
    Json::Value x;
    x["pass"] = static_cast<int>(p.pass);
    x["attempted"] = p.attempted;
    x["succeeded"] = p.succeeded;
    x["backend_status"] = static_cast<int>(p.backend_status);
    x["native_status"] = p.native_status;
    x["iterations"] = p.iterations;
    x["solve_time_ms"] = p.solve_time_ms;
    x["objective_value"] = p.objective_value;
    x["primal_residual"] = p.primal_residual;
    x["dual_residual"] = p.dual_residual;
    x["last_iterate_available"] = p.last_iterate_available;
    x["last_iterate_tasks"] = taskEvidence(p.last_iterate_tasks);
    x["last_iterate_scales"] = scaleEvidence(p.last_iterate_task_scales);
    x["last_iterate_constraints"] =
        constraintEvidence(p.last_iterate_constraints);
    j["passes"].append(x);
  }
  return j;
}
} // namespace
void enableNativeJournal(SolverRuntime &runtime,
                         const std::filesystem::path &path,
                         const std::string &mode) {
  runtime.setAllocationObservation(mode == "cpp-new");
  if (path.empty())
    return;
  struct Journal {
    std::mutex mutex;
    std::ofstream stream;
    explicit Journal(const std::filesystem::path &p) {
      if (std::filesystem::exists(p))
        throw std::runtime_error("native journal already exists");
      stream.exceptions(std::ios::badbit | std::ios::failbit);
      stream.open(p);
    }
  };
  auto journal = std::make_shared<Journal>(path);
  runtime.setObserver([journal, mode](const SolverRequest &request,
                                      const SolverDiagnostics &d,
                                      const mcc::Status *status) {
    SolverSolution solution;
    solution.kinematics_solution = d.native_candidate;
    auto row = status ? evidence(*status, solution, d)
                      : Json::Value(Json::objectValue);
    if (mode == "minimal") {
      for (auto key :
           {"passes", "tasks", "scales", "constraints", "task_scale_reference"})
        row.removeMember(key);
    }
    row["observation_mode"] = mode;
    row["kind"] = status ? "native_solver_result" : "native_solver_begin";
    row["worker"] = d.group == WorkerGroup::Red ? "red" : "yellow";
    row["input_q"] = vec(request.captured_state.state.joint_positions);
    row["input_v"] = vec(request.captured_state.state.joint_velocities);
    for (const auto &t : request.position_targets) {
      Json::Value x;
      x["handle"] = t.handle.value;
      x["position"] = vec(t.position);
      x["enabled"] = t.enabled;
      x["feed_forward_velocity"] = t.feed_forward_velocity
                                       ? vec(*t.feed_forward_velocity)
                                       : Json::Value();
      row["position_targets"].append(x);
    }
    for (const auto &t : request.orientation_targets) {
      Json::Value x;
      x["handle"] = t.handle.value;
      x["enabled"] = t.enabled;
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          x["rotation"].append(t.orientation(i, j));
      x["feed_forward_angular_velocity"] =
          t.feed_forward_angular_velocity
              ? vec(*t.feed_forward_angular_velocity)
              : Json::Value();
      row["orientation_targets"].append(x);
    }

    row["committed"] = false;
    row["commit_scope"] = "solver boundary; actual pipeline commit recorded "
                          "separately in native replay trace";
    Json::StreamWriterBuilder w;
    w["indentation"] = "";
    std::lock_guard lock(journal->mutex);
    journal->stream << Json::writeString(w, row) << '\n';
    journal->stream.flush();
  });
}
void writeModelMapping(const Options &options,
                       const std::shared_ptr<const mcc::RobotModel> &model,
                       const std::filesystem::path &output) {
  const auto &robot = options.interactive.robot;
  Json::Value mapping;
  mapping["schema_version"] = "model_mapping.v1";
  mapping["urdf_path"] = options.interactive.urdf_path;
  mapping["urdf_sha256"] =
      motion_control_lab::sha256_file(options.interactive.urdf_path);
  mapping["hard_feasibility_tolerance"] =
      options.interactive.solver.maximum_accepted_hard_violation;
  mapping["hard_feasibility_unit"] = "rad/s";
  mapping["input_target_space"] = options.target_space;
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col) {
      mapping["left_tcp_offset"].append(
          robot.left_tcp_offset.matrix()(row, col));
      mapping["right_tcp_offset"].append(
          robot.right_tcp_offset.matrix()(row, col));
    }
  mapping["joint_position_margin_rad"] =
      options.interactive.solver.joint_position_margin_rad;
  mapping["base_frame"] = robot.base_frame;
  mapping["left_frame"] = robot.left_end_effector_frame;
  mapping["right_frame"] = robot.right_end_effector_frame;
  for (const auto &n : robot.joint_names)
    mapping["joint_names"].append(n);
  for (auto i : activeJointFullIndices(robot, robot))
    mapping["active_indices"].append(Json::UInt64(i));
  for (const auto &limit : model->jointLimits()) {
    Json::Value l;
    l["minimum_position"] = limit.lower;
    l["maximum_position"] = limit.upper;
    l["velocity"] = limit.velocity;
    l["acceleration"] =
        limit.acceleration ? Json::Value(*limit.acceleration) : Json::Value();
    mapping["joint_limits"].append(l);
  }
  execution::writeJson(output / "model_mapping.json", mapping);
}
Json::Value replayReleasePlan(const Options &options,
                              std::size_t source_count) {
  Json::Value plan;
  plan["schema_version"] = "hks_replay_release_plan.v1";
  plan["source_frame_count"] = Json::UInt64(source_count);
  plan["source_count_origin"] = "loaded replay timeline before worker startup";
  plan["input_sha256"] = sha256_file(options.replay->input_path);
  plan["ui_duration_limit_s"] = options.interactive.duration_s;
  plan["duration_limit_semantics"] =
      "UI wall-time stop condition; not a predetermined worker release budget";
  plan["planned_worker_releases"] = Json::nullValue;
  plan["worker_not_run_releases"] = Json::nullValue;
  plan["worker_plan_availability"] = "unavailable";
  plan["worker_plan_reason"] =
      "original periodic workers run until data-dependent settling, "
      "replay/user stop, fault, or UI duration; source frames are not worker "
      "releases";
  plan["red_rate_hz"] = options.interactive.red_rate_hz;
  plan["yellow_rate_hz"] = options.interactive.yellow_rate_hz;
  plan["clock"] =
      "real steady_clock periodic workers, including batch input driving";
  plan["worker_skip_counter_semantics"] =
      "native skipped_release_count counts release increments after a missed "
      "deadline, including the increment to the next future slot; retained "
      "verbatim, not an exact count of bypassed release slots";
  return plan;
}
Json::Value
replayReleaseCounts(const Options &options,
                    const motion_control_lab::PeriodicWorkerStatistics &red,
                    const motion_control_lab::PeriodicWorkerStatistics &yellow,
                    std::size_t source_count, std::size_t selected,
                    std::size_t dropped, std::size_t source_index,
                    bool completed, bool faulted, bool recorded) {
  Json::Value counts;
  counts["schema_version"] = "hks_replay_release_counts.v1";
  counts["captured_after_worker_join"] = true;
  counts["worker_plan_availability"] = "unavailable";
  counts["planned_worker_releases"] = Json::nullValue;
  counts["worker_not_run_releases"] = Json::nullValue;
  counts["worker_plan_reason"] =
      replayReleasePlan(options, source_count)["worker_plan_reason"];
  counts["stop_state"] = faulted     ? "fault"
                         : completed ? "replay_completed"
                                     : "stopped_before_replay_completion";
  auto worker = [](const motion_control_lab::PeriodicWorkerStatistics &stats) {
    Json::Value j;
    j["callback_iteration_count"] = Json::UInt64(stats.iteration_count);
    j["callback_scope"] =
        "all native scheduler invocations including idle, rejected and "
        "exception iterations; excludes pre-worker solver warmups";
    j["deadline_miss_count"] = Json::UInt64(stats.deadline_miss_count);
    j["raw_skipped_release_count"] = Json::UInt64(stats.skipped_release_count);
    j["recoverable_rejection_count"] =
        Json::UInt64(stats.recoverable_rejection_count);
    j["maximum_release_lateness_ms"] = stats.maximum_release_lateness_ms;
    j["exact_bypassed_release_count"] = Json::nullValue;
    j["exact_all_release_denominator"] = Json::nullValue;
    j["unavailable_reason"] =
        "no fixed worker release horizon; native skip counter includes "
        "future-slot advancement, so callback count plus raw skip count is not "
        "an exact release denominator";
    return j;
  };
  counts["workers"]["red"] = worker(red);
  counts["workers"]["yellow"] = worker(yellow);
  auto &source = counts["source"];
  source["planned_frame_count"] = Json::UInt64(source_count);
  source["native_selected_frame_count"] = Json::UInt64(selected);
  source["native_dropped_frame_count"] = Json::UInt64(dropped);
  source["native_current_source_index"] = Json::UInt64(source_index);
  source["selected_semantics"] = "ReplaySource selection includes initial "
                                 "frame, not solver success or worker delivery";
  source["actual_reference_driver"] =
      recorded ? "recorded_elbow_reference" : "ReplaySource";
  if (recorded) {
    source["not_selected_suffix_count"] = Json::nullValue;
    source["coverage_reason"] =
        "recorded reference controls actual source identity; ReplaySource "
        "counters alone do not certify reference coverage";
  } else {
    source["not_selected_suffix_count"] =
        Json::UInt64(source_count - source_index - 1);
    source["first_not_selected_index"] =
        source_index + 1 < source_count
            ? Json::Value(Json::UInt64(source_index + 1))
            : Json::Value();
    source["partition_consistent"] =
        (selected + dropped + source_count - source_index - 1 == source_count);
  }
  return counts;
}
void writeReplaySummary(const std::filesystem::path &output, int result) {
  Json::Value summary;
  summary["schema_version"] = "app_execution_summary.v1";
  summary["status"] = result == 0 ? "completed" : "failed";
  summary["formal_rt"] = false;
  summary["execution_structure"] = "replay";
  for (const auto *name : {"trace.csv", "manifest.json", "status.json",
                           "release_plan.json", "release_counts.json"}) {
    const auto path = output / "replay" / name;
    if (std::filesystem::exists(path)) {
      Json::Value item;
      item["path"] = path.string();
      item["sha256"] = motion_control_lab::sha256_file(path);
      summary["native_evidence"].append(item);
    }
  }
  if (std::filesystem::exists(output / "replay" / "release_counts.json"))
    summary["release_accounting"] =
        execution::readJson(output / "replay" / "release_counts.json");
  else {
    summary["release_accounting"]["availability"] = "unavailable";
    summary["release_accounting"]["reason"] =
        "native loop did not produce a final joined-worker accounting snapshot";
  }
  if (std::filesystem::exists(output / "replay" / "status.json")) {
    const auto state =
        execution::readJson(output / "replay" / "status.json")["state"]
            .asString();
    summary["native_replay_state"] = state;
    if (state == "stopped")
      summary["status"] = "stopped";
  }
  motion_control_lab::execution::writeJson(output / "summary.json", summary);
}
Json::Value executionCapabilities() {
  Json::Value j;
  j["schema_version"] = "app_capabilities.v1";
  j["app_id"] = "mcl_hierarchical_kinematics_step";
  j["version"] = "execution-1";
  for (auto v : {"snapshot", "trajectory", "replay"})
    j["execution_structures"].append(v);
  for (auto v :
       {"hierarchical", "planned", "planned-otg", "planned-otg-nullspace",
        "planned-otg-nullspace-admittance-kinematic-sim"})
    j["profiles"].append(v);
  for (auto v : {"position-first", "pose-primary",
                 "position-orientation-posture", "primary-only"})
    j["task_layouts"].append(v);
  j["backends"].append("proxqp");
  j["solver_mode"] = "ServoStep";
  for (auto v : {"sync", "divided", "async_same_core", "async_dual_core"})
    j["schedules"].append(v);
  for (auto v : {"virtual", "real"})
    j["timing_modes"].append(v);
  for (auto v : {"native_status", "candidate", "commit", "selected_pass",
                 "all_calls", "releases", "not_run_suffix"})
    j["observation"].append(v);
  for (auto mode : {"minimal", "full", "cpp-new"})
    j["observation_modes"].append(mode);
  j["target_spaces"].append("frame");
  j["target_spaces"].append("tcp");
  j["batch_profile"] = "hierarchical";
  j["replay_planning"] =
      "existing runLoop CartesianPlanner/JointPlanner profiles";
  j["native_reference_sources"].append("manual");
  j["native_reference_sources"].append("recorded");
  j["native_reference_sources"].append("harp");
  return j;
}
Options requestOptions(const execution::Request &r) {
  const auto &c = r.config();
  auto keys = [](const Json::Value &value,
                 std::initializer_list<const char *> allowed) {
    for (const auto &key : value.getMemberNames()) {
      bool found = false;
      for (const auto *candidate : allowed)
        found = found || key == candidate;
      if (!found)
        throw std::runtime_error("unsupported request field: " + key);
    }
  };
  const auto &e = r.document["execution"],
             &observation = r.document["observation"];
  keys(e, {"schedule", "timing_mode", "yellow_divisor", "red_cpu", "yellow_cpu",
           "planned_releases", "warmup_calls", "sample_selection",
           "target_period_ms"});
  keys(observation, {"mode", "capacity"});
  const auto mode = observation.get("mode", "full").asString();
  if (mode != "full" && mode != "minimal" && mode != "cpp-new")
    throw std::runtime_error("unsupported observation mode: " + mode);
  const auto schedule = e.get("schedule", "sync").asString(),
             timing = e.get("timing_mode", "virtual").asString(),
             selection = e.get("sample_selection", "cycle").asString();
  if (schedule != "sync" && schedule != "divided" &&
      schedule != "async_same_core" && schedule != "async_dual_core")
    throw std::runtime_error("unsupported schedule");
  if (timing != "virtual" && timing != "real")
    throw std::runtime_error("unsupported timing_mode");
  if (selection != "cycle" && selection != "source_time")
    throw std::runtime_error("unsupported sample_selection");
  for (auto key : {"planned_releases", "warmup_calls"})
    if (e.isMember(key) &&
        (!e[key].isUInt() ||
         (std::string(key) == "planned_releases" && e[key].asUInt() == 0)))
      throw std::runtime_error("invalid call count");
  if (e.isMember("yellow_divisor") &&
      (!e["yellow_divisor"].isUInt() || e["yellow_divisor"].asUInt() == 0))
    throw std::runtime_error("invalid yellow divisor");
  if (observation.isMember("capacity") && !observation["capacity"].isUInt())
    throw std::runtime_error("invalid observation capacity");

  for (const auto &key : c.getMemberNames())
    if (key != "profile" && key != "options" && key != "target_space")
      throw std::runtime_error("unknown app_config field: " + key);
  const auto structure = r.document["execution_structure"].asString();
  if (structure != "snapshot" && structure != "trajectory" &&
      structure != "replay")
    throw std::runtime_error("unknown execution structure: " + structure);
  const bool replay =
      r.document["execution_structure"] == "replay" ||
      c.get("profile", "hierarchical").asString() != "hierarchical";
  const bool canonical = replay && r.document["input"]["format"] == "json";
  if (replay) {
    keys(e, {"target_period_ms"});
    keys(observation, {"mode"});
  }
  if (!replay && e.isMember("target_period_ms"))
    throw std::runtime_error("target_period_ms is a replay input setting");

  std::vector<std::string> args{"mcl_hierarchical_kinematics_step", "--profile",
                                c.get("profile", "hierarchical").asString(),
                                replay ? "replay" : "teleop"};
  if (replay) {
    args.insert(args.end(), {"--input",
                             canonical ? (r.output / "canonical.csv").string()
                                       : r.document["input"]["path"].asString(),
                             "--output-dir", (r.output / "replay").string()});
    if (canonical)
      args.insert(
          args.end(),
          {"--input-format", "csv", "--left-stream", "left", "--right-stream",
           "right", "--timestamp-source", "csv_timestamp", "--target-period-ms",
           std::to_string(
               r.document["execution"].get("target_period_ms", 10).asDouble()),
           "--initial-state", r.document["input"]["path"].asString(), "--ui",
           "none", "--no-terminal-input"});
  }
  args.insert(args.end(),
              {"--target-space", c.get("target_space", "frame").asString()});
  const auto &o = c["options"];
  for (const auto &key : o.getMemberNames()) {
    if (key == "input" || key == "dump-resolved-options" ||
        key == "batch-input" || key == "batch-output" ||
        key == "target-space" || key == "raw-journal" ||
        key == "observation-mode" || key == "output-dir" ||
        key == "initial-state")
      throw std::runtime_error(
          "request option is controlled by execution envelope: " + key);
    if (o[key].isBool()) {
      args.push_back(std::string(o[key].asBool() ? "--" : "--no-") + key);
    } else {
      args.push_back("--" + key);
      args.push_back(o[key].asString());
    }
  }
  std::vector<char *> pointers;
  for (auto &s : args)
    pointers.push_back(s.data());
  auto result = parseOptions(pointers.size(), pointers.data());

  result.observation_mode = mode;
  if (r.input.isMember("model")) {
    const auto &model = r.input["model"];
    if (model["locator"].asString() != result.interactive.urdf_path)
      throw std::runtime_error(
          "canonical model locator differs from configured URDF");
    if (model["sha256"].asString() != sha256_file(result.interactive.urdf_path))
      throw std::runtime_error(
          "canonical model sha256 differs from configured URDF");
  }
  result.execution_output_dir = r.output.string();
  result.raw_journal_path = (r.output / "native_calls.jsonl").string();
  return result;
}
void prepareRequestReplay(const execution::Request &r, const Options &options) {
  if (!options.replay || r.document["input"]["format"] != "json")
    return;
  std::ofstream csv(r.output / "canonical.csv");
  csv.exceptions(std::ios::badbit | std::ios::failbit);
  csv << std::setprecision(17);
  csv << "timestamp_ns,left_frame_id,right_frame_id,left_x,left_y,left_z,left_"
         "qx,left_qy,left_qz,left_qw,right_x,right_y,right_z,right_qx,right_qy,"
         "right_qz,right_qw\n";
  const double period =
      r.document["execution"].get("target_period_ms", 10).asDouble() / 1000;
  unsigned index = 0;
  for (const auto &sample : r.input["samples"]) {
    csv << static_cast<long long>(
               sample.get("source_time_s", index * period).asDouble() * 1e9)
        << ',' << options.interactive.robot.base_frame << ','
        << options.interactive.robot.base_frame;
    for (auto side : {"left", "right"}) {
      const auto &t = sample["targets"][side];
      if (t["position"].size() != 3 || t["rotation"].empty())
        throw std::runtime_error("canonical replay requires complete bilateral "
                                 "positions and rotations");
      Eigen::Matrix3d rotation;
      const auto &a = t["rotation"];
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          rotation(i, j) =
              a[i].isArray() ? a[i][j].asDouble() : a[i * 3 + j].asDouble();
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.linear() = rotation;
      pose.translation() = eigen(t["position"]);
      if (options.target_space == "frame")
        pose = pose * (std::string(side) == "left"
                           ? options.interactive.robot.left_tcp_offset
                           : options.interactive.robot.right_tcp_offset);
      for (int axis = 0; axis < 3; ++axis)
        csv << ',' << pose.translation()[axis];
      const Eigen::Quaterniond q(pose.linear());
      csv << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w();
    }
    csv << '\n';
    ++index;
  }
  csv.close();
  Json::Value conversion;
  conversion["schema_version"] = "canonical_replay_conversion.v1";
  conversion["source_sha256"] = r.document["input"]["sha256"];
  conversion["pose_conversion"] =
      options.target_space == "frame"
          ? "frame target times app TCP offset; replay applies inverse offset"
          : "physical TCP target unchanged; replay applies inverse app TCP "
            "offset";
  conversion["csv_sha256"] = sha256_file(r.output / "canonical.csv");
  conversion["initial_state"] =
      "explicit --initial-state via existing runLoop initial state staging";
  execution::writeJson(r.output / "canonical_conversion.json", conversion);
}
int runBatch(const Options &options, const Json::Value &input,
             const Json::Value &settings, const std::filesystem::path &output,
             SolverRuntime &runtime, const SolverHandles &handles) {
  const auto &robot = options.interactive.robot;
  const auto indices = activeJointFullIndices(robot, robot);
  const std::string structure =
      settings.get("structure", "snapshot").asString();
  const auto schedule = settings.get("schedule", "sync").asString();
  const auto timing = settings.get("timing_mode", "virtual").asString();
  const bool async =
      schedule == "async_same_core" || schedule == "async_dual_core";
  if (schedule != "sync" && schedule != "divided" && !async)
    throw std::runtime_error("unknown schedule");
  if (timing != "virtual" && timing != "real")
    throw std::runtime_error("unknown timing mode");
  const int red_cpu = settings.get("red_cpu", -1).asInt(),
            yellow_cpu = settings.get("yellow_cpu", red_cpu).asInt();
  if (async && timing == "real" &&
      (red_cpu < 0 || yellow_cpu < 0 ||
       (schedule == "async_same_core" ? red_cpu != yellow_cpu
                                      : red_cpu == yellow_cpu)))
    throw std::runtime_error(
        "async core topology requires explicit matching/distinct CPU IDs");
  const unsigned divisor =
      settings.get("yellow_divisor", schedule == "sync" ? 1 : 10).asUInt();
  if (!divisor)
    throw std::runtime_error("yellow_divisor must be positive");
  if (schedule == "sync" && divisor != 1)
    throw std::runtime_error("sync schedule requires yellow_divisor=1");
  const double period = 1 / options.interactive.red_rate_hz;
  mcc::RobotState state;
  state.joint_positions = Eigen::Map<const Eigen::VectorXd>(
      robot.default_positions.data(), robot.default_positions.size());
  state.joint_velocities = Eigen::VectorXd::Zero(state.joint_positions.size());
  auto applyState = [&](const Json::Value &s) {
    if (s.isMember("q"))
      state.joint_positions = eigen(s["q"]);
    if (s.isMember("v"))
      state.joint_velocities = eigen(s["v"]);
  };
  applyState(input["initial_state"]);
  const auto &samples = input["samples"];
  if (!samples.isArray() || samples.empty())
    throw std::runtime_error("batch input requires samples");
  const unsigned releases =
      settings.get("planned_releases", samples.size()).asUInt();
  const bool cyclic =
      settings.get("sample_selection", "cycle").asString() == "cycle";
  if (input.isMember("joint_names")) {
    Json::Value names(Json::arrayValue);
    for (auto &n : robot.joint_names)
      names.append(n);
    if (input["joint_names"] != names)
      throw std::runtime_error("batch joint_names must match app robot order");
  }
  std::ofstream raw(output / "raw.jsonl");
  raw.exceptions(std::ios::badbit | std::ios::failbit);
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  auto emit = [&](const Json::Value &row) {
    raw << Json::writeString(writer, row) << '\n';
    raw.flush();
  };
  unsigned ledger_cursor = 0;
  struct ExceptionSuffix {
    unsigned &cursor;
    unsigned count;
    decltype(emit) &emit_row;
    ~ExceptionSuffix() {
      if (std::uncaught_exceptions()) {
        for (; cursor < count; ++cursor) {
          Json::Value row;
          row["kind"] = "release";
          row["release_index"] = cursor;
          row["status"] = "not-run";
          row["reason"] = "original application exception interrupted "
                          "execution; see process stderr";
          emit_row(row);
        }
      }
    }
  } exception_suffix{ledger_cursor, releases, emit};
  runtime.beginRun(1);
  Json::Value start;
  start["kind"] = "begin";
  start["schedule"] = schedule;
  start["timing_mode"] = timing;
  start["formal_rt"] = false;
  start["red_cpu"] = red_cpu;
  start["yellow_cpu"] = yellow_cpu;
  start["period_s"] = period;
  start["planned_releases"] = releases;
  start["input"] = input;
  emit(start);
  std::mutex mutex;
  std::condition_variable wake;
  bool stop = false, pending = false;
  SolverRequest queued;
  std::exception_ptr worker_error;
  std::vector<Json::Value> yellow_records;
  auto yellowCall = [&](const SolverRequest &request) {
    SolverSolution s;
    SolverDiagnostics d;
    const auto status = runtime.solveYellow(request, s, d);
    auto row = evidence(status, s, d);
    row["kind"] = "yellow_call";
    row["actual_cpu"] = sched_getcpu();
    return row;
  };
  std::thread worker;
  if (async && timing == "real")
    worker = std::thread([&] {
      try {
        pin(yellow_cpu);
        while (true) {
          SolverRequest request;
          {
            std::unique_lock lock(mutex);
            wake.wait(lock, [&] { return stop || pending; });
            if (stop && !pending)
              break;
            request = queued;
            pending = false;
          }
          auto row = yellowCall(request);
          {
            std::lock_guard lock(mutex);
            yellow_records.push_back(std::move(row));
          }
        }
      } catch (...) {
        std::lock_guard lock(mutex);
        worker_error = std::current_exception();
      }
    });
  struct Join {
    std::thread &worker;
    std::mutex &mutex;
    std::condition_variable &wake;
    bool &stop;
    ~Join() {
      {
        std::lock_guard lock(mutex);
        stop = true;
      }
      wake.notify_one();
      if (worker.joinable())
        worker.join();
    }
  } join{worker, mutex, wake, stop};
  if (timing == "real")
    pin(red_cpu);
  const auto origin = Clock::now();
  bool failed = false;
  unsigned completed = 0, skipped = 0, notrun = 0, overflow = 0;
  const auto capacity =
      settings["observation"].get("capacity", releases).asUInt();
  std::vector<Json::Value> observation_buffer;
  observation_buffer.reserve(std::min(capacity, releases));
  const unsigned warmup = settings.get("warmup_calls", 0).asUInt();
  for (Json::ArrayIndex i = 0; i < releases; ++i) {
    Json::ArrayIndex sample_index = i % samples.size();
    if (!cyclic) {
      sample_index = 0;
      while (sample_index + 1 < samples.size() &&
             samples[sample_index + 1].get("source_time_s", 0).asDouble() <=
                 i * period)
        ++sample_index;
    }
    const auto &sample = samples[sample_index];
    Json::Value row;
    row["kind"] = "release";
    row["release_index"] = i;
    row["source_index"] = sample_index;
    row["planned_time_s"] = i * period;
    if (failed) {
      row["status"] = "not-run";
      ++notrun;
      emit(row);
      ledger_cursor = i + 1;
      continue;
    }
    if (timing == "real") {
      auto due = origin + std::chrono::duration_cast<Clock::duration>(
                              std::chrono::duration<double>(i * period));
      std::this_thread::sleep_until(due);
      if (i > 0 && Clock::now() > due + std::chrono::duration<double>(period)) {
        row["status"] = "skipped";
        ++skipped;
        emit(row);
        ledger_cursor = i + 1;
        continue;
      }
    }
    if (structure == "snapshot")
      applyState(sample);
    SolverRequest request;
    request.captured_state = {state, i,
                              static_cast<std::int64_t>(i * period * 1e9)};
    request.reference_frame_name = robot.base_frame;
    mcc::ForwardKinematicsRequest fk;
    fk.state = state;
    fk.reference_frame_name = robot.base_frame;
    fk.frame_names = {robot.left_end_effector_frame,
                      robot.right_end_effector_frame, robot.left_link4_frame,
                      robot.right_link4_frame};
    mcc::ForwardKinematicsSolution poses;
    mcc::ForwardKinematicsDiagnostics fd;
    requireOk(runtime.computeForwardKinematics(fk, poses, fd), "batch FK");
    for (unsigned arm = 0; arm < 2; ++arm) {
      const auto &t = sample["targets"][arm ? "right" : "left"];
      auto p = poses.poses.at(arm).pose;
      if (t.isMember("position"))
        p.translation() = eigen(t["position"]);
      if (t.isMember("rotation")) {
        const auto &a = t["rotation"];
        for (int r = 0; r < 3; ++r)
          for (int c = 0; c < 3; ++c)
            p.linear()(r, c) =
                a[r].isArray() ? a[r][c].asDouble() : a[r * 3 + c].asDouble();
      }
      if (options.target_space == "tcp" && t.isMember("position"))
        p = p *
            (arm ? robot.right_tcp_offset : robot.left_tcp_offset).inverse();
      const bool enabled = t.get("enabled", true).asBool();
      request.position_targets.emplace_back(arm ? handles.red.right_position
                                                : handles.red.left_position,
                                            p.translation(), enabled);
      request.orientation_targets.emplace_back(
          arm ? handles.red.right_orientation : handles.red.left_orientation,
          p.linear(), enabled);
      const auto &elbow = sample["elbow_targets"][arm ? "right" : "left"];
      request.position_targets.emplace_back(
          arm ? handles.red_right_link4 : handles.red_left_link4,
          elbow.isArray() ? Eigen::Vector3d(eigen(elbow))
                          : poses.poses.at(arm + 2).pose.translation(),
          elbow.isArray());
    }
    SolverRequest yellow;
    yellow.captured_state = request.captured_state;
    yellow.reference_frame_name = robot.base_frame;
    if (i % divisor == 0) {
      if (worker.joinable()) {
        std::lock_guard lock(mutex);
        if (worker_error)
          std::rethrow_exception(worker_error);
        if (pending) {
          Json::Value x;
          x["kind"] = "yellow_release";
          x["status"] = "skipped";
          x["state_sequence"] = Json::UInt64(queued.captured_state.sequence);
          emit(x);
        }
        queued = yellow;
        pending = true;
        wake.notify_one();
      } else
        emit(yellowCall(yellow));
    }
    SolverSolution solution;
    SolverDiagnostics diagnostics;
    const auto cpu_begin = cpuNs();
    auto begin = Clock::now();
    const auto status = runtime.solveRed(request, solution, diagnostics);
    auto end = Clock::now();
    const auto cpu_end = cpuNs();
    auto call = evidence(status, solution, diagnostics);
    call["kind"] = "red_call";
    call["release_index"] = i;
    call["actual_cpu"] = sched_getcpu();
    call["input_q"] = vec(state.joint_positions);
    call["input_v"] = vec(state.joint_velocities);
    call["targets"] = sample["targets"];
    call["target_space"] = options.target_space;
    for (const auto &t : request.position_targets) {
      Json::Value x;
      x["position"] = vec(t.position);
      x["enabled"] = t.enabled;
      call["actual_position_targets"].append(x);
    }
    for (const auto &t : request.orientation_targets) {
      Json::Value x;
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          x["rotation"].append(t.orientation(i, j));
      x["enabled"] = t.enabled;
      call["actual_orientation_targets"].append(x);
    }

    call["phase"] = completed == 0        ? "cold"
                    : completed <= warmup ? "warm-up"
                                          : "steady";
    call["call_elapsed_ns"] = Json::Int64(
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin)
            .count());
    call["sample_index"] = sample_index;
    call["thread_cpu_time_available"] = true;
    call["thread_cpu_ns"] = Json::Int64(cpu_end - cpu_begin);
    call["state_age_s"] =
        timing == "real"
            ? std::chrono::duration<double>(begin - origin).count() - i * period
            : 0.0;
    call["proposal_age_s"] =
        diagnostics.consumed_source_value_revision
            ? Json::Value(
                  (timing == "real"
                       ? std::chrono::duration<double>(begin - origin).count()
                       : i * period) -
                  diagnostics.consumed_source_state_time_nanoseconds * 1e-9)
            : Json::Value();
    emit(call);
    if (observation_buffer.size() < capacity)
      observation_buffer.push_back(call);
    else {
      ++overflow;
      failed = true;
      Json::Value fault;
      fault["kind"] = "observation_overflow";
      fault["capacity"] = capacity;
      fault["release_index"] = i;
      emit(fault);
    }
    row["status"] = "executed";
    row["deadline_miss"] =
        timing == "real" &&
        end > origin + std::chrono::duration<double>((i + 1) * period);
    emit(row);
    ledger_cursor = i + 1;
    ++completed;
    if (status.ok() && structure == "trajectory") {
      for (unsigned j = 0; j < indices.size(); ++j) {
        state.joint_positions[indices[j]] =
            solution.kinematics_solution.joint_positions[j];
        state.joint_velocities[indices[j]] =
            solution.kinematics_solution.joint_velocities[j];
      }
    }
    if (!status.ok())
      failed = true;
  }
  {
    std::lock_guard lock(mutex);
    stop = true;
  }
  wake.notify_one();
  if (worker.joinable())
    worker.join();
  if (worker_error)
    std::rethrow_exception(worker_error);
  for (const auto &row : yellow_records)
    emit(row);
  Json::Value summary;
  summary["schema_version"] = "app_execution_summary.v1";
  summary["completed_calls"] = completed;
  summary["skipped_releases"] = skipped;
  summary["not_run_releases"] = notrun;
  summary["planned_releases"] = releases;
  summary["observation_overflow_count"] = overflow;
  summary["observation_capacity"] = capacity;
  summary["tail_quantiles_available"] = overflow == 0;
  summary["status"] = failed ? "failed" : "completed";
  summary["formal_rt"] = false;
  execution::writeJson(output / "summary.json", summary);
  return failed ? 1 : 0;
}
} // namespace motion_control_lab::hierarchical_kinematics_step
