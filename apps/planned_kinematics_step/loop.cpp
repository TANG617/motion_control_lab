#include "loop.hpp"
#include <chrono>
#include <cmath>
#include <fstream>
namespace motion_control_lab::planned_kinematics_step {
int loop(const Options &o, Solver &solver, Planning &planning) {
  using Clock = std::chrono::steady_clock;
  auto ns = []() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               Clock::now().time_since_epoch())
        .count();
  };
  std::ofstream raw(o.output / "raw.jsonl");
  raw.exceptions(std::ios::badbit | std::ios::failbit);
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  auto emit = [&](const Json::Value &row) {
    raw << Json::writeString(writer, row) << '\n';
    raw.flush();
  };
  auto q = numbers(o.input["initial_state"]["q"]),
       v = numbers(o.input["initial_state"]["v"]);
  std::vector<double> a(q.size(), 0);
  const double dt = o.config["dt_s"].asDouble();
  const int total = o.planned_releases;
  const auto &samples = o.input["samples"];
  if (samples.empty())
    throw std::runtime_error("empty motion input");
  int source = 0, previous = -1, attempts = 0, committed = -1;
  bool failed = false;
  Json::Value schedule;
  schedule["planned_releases"] = total;
  schedule["period_s"] = dt;
  schedule["period_ns"] = Json::Int64(o.period_ns);
  schedule["duration_ns"] = Json::Int64(o.duration_ns);
  schedule["release_clock"] =
      "nearest-nanosecond grid; half-open duration window";
  schedule["runtime_mode"] = "virtual";
  schedule["skipped"] = 0;
  write(o.output / "planned_schedule.json", schedule);
  for (int tick = 0; tick < total; ++tick) {
    const double t = static_cast<double>(tick * o.period_ns) * 1e-9;
    while (source + 1 < static_cast<int>(samples.size()) &&
           samples[source + 1]["source_time_s"].asDouble() <= t + 1e-12)
      ++source;
    Json::Value row;
    row["record_type"] = "attempt_begin";
    row["attempt_sequence"] = tick;
    row["input_sequence"] = source;
    row["logical_time_s"] = t;
    row["source_time_s"] = t;
    row["input_source_time_s"] = samples[source]["source_time_s"];
    row["runtime_mode"] = "virtual";
    row["release_ns"] = Json::nullValue;
    row["release_ns_reason"] =
        "virtual schedule; host durations are non-RT observations";
    row["state_sequence"] = tick;
    row["feedback_age_s"] = 0.;
    row["feedback_q"] = array(q);
    row["feedback_v"] = array(v);
    row["feedback_a"] = array(a);
    row["joint_names"] = o.input["joint_names"];
    row["goal"] = samples[source]["targets"];
    row["task_revision"] = source;
    row["start_ns"] = Json::Int64(ns());
    emit(row);
    ++attempts;
    const auto reference =
        planning.reference(row["goal"], source != previous, row);
    previous = source;
    row["reference"] = reference;
    Result result;
    if (!reference.isNull()) {
      const auto start = ns();
      result = solver.solve(q, v, reference);
      row["ik_solve_call_ns"] = Json::Int64(ns() - start);
      for (const auto &key : result.native.getMemberNames())
        row[key] = result.native[key];
    } else {
      row["ik_solve_call_ns"] = Json::nullValue;
      row["quality_category"] = "not-run";
      row["ik_missing_reason"] = "CartesianTrajectoryGenerator failure";
    }
    row["candidate_available"] = !result.q.empty();
    row["raw_ik_q"] = result.q.empty() ? Json::Value() : array(result.q);
    row["raw_ik_v"] = result.v.empty() ? Json::Value() : array(result.v);
    row["ik_accepted"] = result.accepted;
    row["ik_disposition"] = reference.isNull()
                                ? "not-run"
                                : (result.accepted ? "accepted" : "rejected");
    bool accepted = result.accepted;
    if (accepted) {
      const auto sample = planning.execute(q, v, a, result.q, row);
      accepted = row["joint_plan_ok"].asBool() && row["joint_step_ok"].asBool();
      if (accepted) {
        q = sample.positions;
        v = sample.velocities;
        a = sample.accelerations;
        committed = tick;
      }
    }
    row["committed"] = accepted;
    row["committed_sequence"] = committed;
    row["q"] = array(q);
    row["v"] = array(v);
    row["a"] = array(a);
    row["execution_state"] = accepted ? "committed" : "terminated";
    row["disposition"] = accepted ? "accepted" : "rejected";
    row["acceleration_source"] = "JointPtpTrajectoryGenerator-native";
    row["finish_ns"] = Json::Int64(ns());
    row["record_type"] = "attempt";
    emit(row);
    if (!accepted) {
      failed = true;
      break;
    }
  }
  for (int tick = attempts; tick < total; ++tick) {
    Json::Value row;
    row["record_type"] = "release";
    row["attempt_sequence"] = tick;
    row["logical_time_s"] = static_cast<double>(tick * o.period_ns) * 1e-9;
    row["execution_state"] = "not-run";
    row["reason"] = "terminated after native failure";
    emit(row);
  }
  Json::Value status;
  status["status"] = failed ? "failed" : "completed";
  status["attempts"] = attempts;
  status["committed_count"] = committed + 1;
  status["planned_releases"] = total;
  status["not_run"] = total - attempts;
  status["skipped"] = 0;
  status["plant_model"] =
      "ideal kinematic committed JointPtpTrajectoryGenerator output; no hardware or dynamics";
  status["failure_policy"] =
      "stop without modifying rejected candidate or state";
  write(o.output / "native_status.json", status);
  return failed ? 2 : 0;
}
} // namespace motion_control_lab::planned_kinematics_step
