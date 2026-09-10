#include "loop.hpp"
#include <chrono>
#include <cmath>
#include <fstream>
namespace study_e11 {
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
  write(o.output / "resolved_config.json", o.config);
  auto q = numbers(o.input["initial_state"]["q"]),
       v = numbers(o.input["initial_state"]["v"]);
  std::vector<double> a(q.size(), 0);
  std::vector<std::vector<double>> historyQ{q}, historyV{v};
  double dt = o.config["dt_s"].asDouble();
  int delay = std::llround(o.config["feedback_delay_s"].asDouble() / dt);
  auto pipeline = o.config["pipeline"].asString();
  bool cp = pipeline == "cartesian" || pipeline == "cartesian-joint",
       jp = pipeline == "joint" || pipeline == "cartesian-joint";
  auto &samples = o.input["samples"];
  int total = std::ceil(o.config["duration_s"].asDouble() / dt);
  int source = 0, previous = -1, committed = -1;
  bool anyFailure = false;
  auto epoch = ns();
  for (int tick = 0; tick < total; tick++) {
    double t = tick * dt;
    while (source + 1 < (int)samples.size() &&
           samples[source + 1]["source_time_s"].asDouble() <= t + 1e-12)
      source++;
    auto goal = samples[source]["targets"];
    Json::Value row;
    row["record_type"] = "attempt";
    row["attempt_sequence"] = tick;
    row["input_sequence"] = source;
    row["state_sequence"] = std::max(0, tick - delay);
    row["task_revision"] = source;
    row["joint_names"] = o.input["joint_names"];
    row["goal"] = goal;
    row["release_ns"] = Json::Value();
    row["release_ns_reason"] =
        "virtual schedule; wall start/finish measure complete pipeline "
        "including adaptation and evidence assembly";
    row["runtime_mode"] = "virtual";
    row["logical_time_s"] = t;
    row["input_source_time_s"] = samples[source]["source_time_s"];
    row["start_ns"] = Json::Int64(ns());
    row["source_time_s"] = t;
    row["window_id"] = samples[source].get("event", "motion");
    Json::Value beginning = row;
    beginning["record_type"] = "attempt_begin";
    raw << Json::writeString(writer, beginning) << '\n';
    raw.flush();
    auto reference =
        cp ? planning.reference(goal, source != previous, row) : goal;
    previous = source;
    row["reference"] = reference;
    row["targets"] = goal;
    int capture = std::max(0, tick - delay);
    row["feedback_q"] = array(historyQ[capture]);
    row["feedback_v"] = array(historyV[capture]);
    row["feedback_age_s"] = std::min(tick, delay) * dt;
    row["raw_target_derivative_policy"] =
        "stationary-joint-endpoint; Cartesian native continuous PVA";
    Result result;
    bool accepted = !reference.isNull();
    if (accepted) {
      result = solver.solve(historyQ[capture], historyV[capture], reference);
      for (auto &key : result.native.getMemberNames())
        row[key] = result.native[key];
      accepted = result.accepted;
    }
    row["ik_committed"] = result.accepted;
    row["ik_disposition"] = reference.isNull()
                                ? "not-run"
                                : (result.accepted ? "accepted" : "rejected");
    if (reference.isNull())
      row["quality_category"] = "not-run";
    if (reference.isNull())
      row["ik_missing_reason"] = "Cartesian planner returned failure before IK";
    row["raw_ik_q"] = result.q.empty() ? Json::Value() : array(result.q);
    row["raw_ik_v"] = result.v.empty() ? Json::Value() : array(result.v);
    auto projected = result.q;
    if (result.accepted && o.input["fixture"].asBool() &&
        o.config["fixture_downstream_outside_limit"].asBool()) {
      projected.back() =
          o.input["limits"]["upper"][(int)projected.size() - 1].asDouble() +
          0.1;
      row["fixture_target_transform"] =
          "explicit downstream position fault injection; original IK unchanged";
    }
    row["raw_downstream_target_q"] =
        projected.empty() ? Json::Value() : array(projected);
    row["projection_events"] = Json::Value(Json::arrayValue);
    if (accepted && o.config["projection_policy"].asString() ==
                        "explicit-position-bounds") {
      for (size_t i = 0; i < projected.size(); i++) {
        double changed = std::clamp(
            projected[i], o.input["limits"]["lower"][(int)i].asDouble(),
            o.input["limits"]["upper"][(int)i].asDouble());
        if (changed != projected[i]) {
          Json::Value e;
          e["joint_index"] = (int)i;
          e["before"] = projected[i];
          e["after"] = changed;
          row["projection_events"].append(e);
          projected[i] = changed;
        }
      }
    }
    row["projected_target_q"] =
        projected.empty() ? Json::Value() : array(projected);
    auto oldv = v;
    if (accepted && jp) {
      auto s = planning.execute(q, v, a, projected, row);
      accepted = !s.positions.empty();
      if (accepted) {
        q = s.positions;
        v = s.velocities;
        a = s.accelerations;
        row["acceleration_source"] = "JointPlanner-native";
      }
    } else if (accepted) {
      // Explicit ideal velocity integration from current committed state.
      // Delayed feedback remains the IK capture state only.
      v = result.v;
      for (size_t i = 0; i < q.size(); ++i)
        q[i] += dt * v[i];
      for (size_t i = 0; i < a.size(); i++)
        a[i] = (v[i] - oldv[i]) / dt;
      row["acceleration_source"] = "backward-difference-native-velocity; first "
                                   "boundary initialized-zero";
    }
    if (accepted)
      committed = tick;
    else {
      anyFailure = true;
      std::fill(v.begin(), v.end(), 0);
      std::fill(a.begin(), a.end(), 0);
      row["hold_reason"] =
          "native rejection; explicit zero-order position hold";
    }
    row["committed"] = accepted;
    row["committed_sequence"] = committed;
    row["q"] = array(q);
    row["v"] = array(v);
    row["a"] = array(a);
    row["disposition"] = accepted ? "accepted" : "rejected";
    row["execution_state"] = accepted ? "committed" : "HOLD";
    row["finish_ns"] = Json::Int64(ns());
    row["proposal_revision"] = tick;
    row["proposal_created_ns"] = row["finish_ns"];
    row["proposal_state_capture_ns"] = Json::Value();
    row["proposal_state_capture_logical_s"] = capture * dt;
    row["accepted_position_tolerance"] = Json::Value();
    row["accepted_position_tolerance_reason"] =
        "ServoStep acceptance is hard-feasibility, no pose convergence "
        "acceptance";
    historyQ.push_back(q);
    historyV.push_back(v);
    raw << Json::writeString(writer, row) << '\n';
    raw.flush();
  }
  Json::Value status;
  status["status"] = anyFailure ? "partial" : "completed";
  status["attempts"] = total;
  status["plant_model"] =
      "ideal-kinematic-committed-state; no dynamics or hardware";
  write(o.output / "native_status.json", status);
  return anyFailure ? 2 : 0;
}
} // namespace study_e11
