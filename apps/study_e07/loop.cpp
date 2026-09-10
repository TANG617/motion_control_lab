#include "loop.hpp"
#include <fstream>
namespace study {
int loop(const Options &o, Solver &s) {
  write(o.output + "/resolved_config.json", o.config);
  Json::Value mapping;
  for (auto name : {"model", "joint_names", "active_joint_names", "frames",
                    "root_frame", "tcp_offsets", "limits", "units"})
    mapping[name] = o.input[name];
  write(o.output + "/model_mapping.json", mapping);
  Json::Value sem = o.config;
  sem["method_id"] = o.method;
  sem["tcp_equation"] = "frame_goal=p_tcp-R_goal*tcp_offset; frame-origin "
                        "residual; actual TCP checked independently";
  sem["paired_causal_admission"] = "requires specific E06 parity artifact; "
                                   "input correctness alone insufficient";
  sem["decision_variable"] =
      o.method.find("placo") != std::string::npos
          ? "floating base plus all joint increments with masks"
          : "active joint velocity for ServoStep; active increment for "
            "TargetSolve";
  write(o.output + "/method_semantics.json", sem);
  std::ofstream raw(o.output + "/raw.jsonl");
  raw.exceptions(std::ios::badbit | std::ios::failbit);
  Json::StreamWriterBuilder b;
  b["indentation"] = "";
  Json::Value state = o.input["initial_state"];
  int committed = 0;
  int count = o.config["steps"].asInt();
  for (int i = 0; i < count; ++i) {
    const auto &samples = o.input["samples"];
    int ix = o.config["state_mode"].asString() == "snapshot"
                 ? i % samples.size()
                 : std::min<int>(i, samples.size() - 1);
    if (o.config["state_mode"].asString() == "evolving") {
      ix = 0;
      double time = i * o.config["period_s"].asDouble();
      for (unsigned k = 0; k < samples.size(); ++k)
        if (samples[k]["source_time_s"].asDouble() <= time)
          ix = k;
    }
    auto sample = samples[ix];
    if (o.config["state_mode"].asString() == "snapshot") {
      state["q"] = sample["q"];
      state["v"] = sample["v"];
    }
    Json::Value begin;
    begin["record_type"] = "attempt_started";
    begin["attempt_sequence"] = i;
    begin["start_ns"] = Json::Int64(now());
    raw << Json::writeString(b, begin) << std::endl;
    auto wrapper_start = now();
    auto row = s.solve(state, sample, i);
    auto wrapper_finish = now();
    row["call_release_to_extract_start_ns"] = Json::Int64(wrapper_start);
    row["call_release_to_extract_finish_ns"] = Json::Int64(wrapper_finish);
    row["attempt_sequence"] = i;
    row["input_sequence"] = sample["sequence"];
    row["state_sequence"] = o.config["state_mode"].asString() == "snapshot"
                                ? sample["sequence"]
                                : Json::Value(committed);
    row["task_revision"] = sample.get("task_revision", 0);
    row["release_ns"] = Json::nullValue;
    row["release_ns_reason"] =
        "unpaced numerical development loop; no realtime release";
    for (auto key : {"proposal_revision", "proposal_created_ns",
                     "proposal_state_capture_ns"})
      row[key] = Json::nullValue;
    if (!row.isMember("selected_priority"))
      row["selected_priority"] = Json::nullValue;
    if (!row.isMember("highest_completed_priority"))
      row["highest_completed_priority"] = Json::nullValue;
    if (row["committed"].asBool()) {
      ++committed;
      if (o.config["state_mode"].asString() == "evolving") {
        state["q"] = row["q"];
        state["v"] = row["v"];
      }
    }
    row["committed_sequence"] = committed;
    row["execution_state"] = row["committed"].asBool() ? "committed" : "HOLD";
    row["input_source_time_s"] = sample["source_time_s"];
    row["source_time_s"] = o.config["state_mode"].asString() == "evolving"
                               ? i * o.config["period_s"].asDouble()
                               : sample["source_time_s"].asDouble();
    row["secondary_enabled"] = sample["secondary_enabled"];
    row["elbow_targets"] = sample["elbows"];
    row["posture_target"] = sample["posture"];
    raw << Json::writeString(b, row) << std::endl;
  }
  Json::Value status;
  status["status"] = "completed";
  status["attempt_count"] = count;
  status["committed_count"] = committed;
  write(o.output + "/native_status.json", status);
  return 0;
}
} // namespace study
