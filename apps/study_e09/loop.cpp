#include "loop.hpp"
#include "allocation.hpp"
#include <fstream>
#include <map>
#include <pthread.h>
#include <sched.h>
#include <stdexcept>
namespace study_e09 {
int run(const Options &o, Solver &solver) {
  if (!o.config["cpu_ids"].empty()) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(o.config["cpu_ids"][0].asInt(), &set);
    int code = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    if (code)
      throw std::runtime_error("pthread_setaffinity_np failed: " +
                               std::to_string(code));
  }
  writeJson(o.output + "/resolved_config.json", o.config);
  Json::Value boundary;
  boundary["schema_version"] = "timing_boundaries.v1";
  boundary["included"] =
      "Canonical request adaptation, state copy/FK required by native API, "
      "tasks, complete native IK call, consumable q/v extraction";
  boundary["excluded"] =
      "Input decoding/preload, model/topology construction, JSON "
      "serialization, independent FK/oracle, artifact IO";
  boundary["observation"] = o.config["observation"];
  boundary["native_diagnostics"] =
      "Current Core API always computes mandatory native diagnostics. Full arm "
      "adds retained task diagnostics and serialization outside IK call; no "
      "claim of Core diagnostics-disabled overhead.";
  boundary["snapshot"] = "Each call resets to frozen input q/v; workspace "
                         "retained; no hidden dummy solve";
  writeJson(o.output + "/timing_boundaries.json", boundary);
  std::ofstream raw(o.output + "/raw.jsonl"),
      times(o.output + "/call_timing.csv"),
      work(o.output + "/workload_inventory.csv");
  raw.exceptions(std::ios::badbit | std::ios::failbit);
  times.exceptions(std::ios::badbit | std::ios::failbit);
  times << "attempt_sequence,input_sequence,window_id,start_ns,finish_ns,ik_"
           "call_time_ms,cpu_time_s,solution_quality\n";
  work << "method,mode,backend,layers,workload,active_dof,observation,warm_"
          "start\n"
       << o.config["method"].asString() << ',' << o.config["mode"].asString()
       << ',' << o.config["backend"].asString() << ','
       << o.config["layers"].asInt() << ',' << o.config["workload"].asInt()
       << ','
       << (o.config.isMember("active_joint_names")
               ? o.config["active_joint_names"].size()
               : o.input["active_joint_names"].size())
       << ',' << o.config["observation"].asString() << ','
       << o.config["warm_start"].asBool() << '\n';
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  int warm = o.config["warmup_calls"].asInt(),
      steady = o.config["measured_calls"].asInt(), total = 1 + warm + steady;
  std::map<std::string, int> counts;
  int rejected = 0;
  for (int i = 0; i < total; ++i) {
    const auto &sample = o.input["samples"][i % o.input["samples"].size()];
    Json::Value beginning;
    beginning["record_type"] = "attempt_begin";
    beginning["attempt_sequence"] = i;
    beginning["input_sequence"] = sample["sequence"];
    beginning["source_time_s"] = sample["source_time_s"];
    raw << Json::writeString(writer, beginning) << '\n';
    raw.flush();
    bool allocation_observation =
        o.config["observation"].asString() == "cpp-new";
    if (allocation_observation)
      beginAllocationObservation();
    double cpu = threadCpuSeconds();
    long long start = nowNs();
    auto result = solver.solve(sample);
    long long finish = nowNs();
    double spent = threadCpuSeconds() - cpu;
    auto allocations = endAllocationObservation();
    auto row = solver.evidence(result);
    if (allocation_observation) {
      row["allocation_count"] = Json::UInt64(allocations.count);
      row["allocation_bytes"] = Json::UInt64(allocations.bytes);
      row["allocation_reason"] = Json::nullValue;
    }
    row["allocation_scope"] =
        "Current thread replaceable C++ new/new[]/aligned new between adapter "
        "entry and result extraction; excludes direct malloc/calloc/realloc, "
        "Eigen malloc and other threads";
    row["record_type"] = "attempt";
    row["attempt_sequence"] = i;
    row["input_sequence"] = sample["sequence"];
    row["source_time_s"] = sample["source_time_s"];
    row["state_sequence"] = sample["sequence"];
    row["task_revision"] = sample["sequence"];
    row["targets"] = sample["targets"];
    row["release_ns"] = Json::Int64(start);
    row["start_ns"] = Json::Int64(start);
    row["finish_ns"] = Json::Int64(finish);
    row["ik_call_time_ms"] = (finish - start) * 1e-6;
    row["cpu_time_s"] = spent;
    row["window_id"] = i == 0 ? "cold" : i <= warm ? "warmup" : "steady";
    row["execution_state"] = "snapshot";
    row["committed_sequence"] =
        result.accepted ? Json::Value(i) : Json::Value();
    row["proposal_revision"] = Json::nullValue;
    row["proposal_created_ns"] = Json::nullValue;
    row["proposal_state_capture_ns"] = Json::nullValue;
    row["proposal_reason"] = "Snapshot benchmark has no proposal";
    raw << Json::writeString(writer, row) << '\n';
    raw.flush();
    times << i << ',' << sample["sequence"].asInt() << ','
          << row["window_id"].asString() << ',' << start << ',' << finish << ','
          << row["ik_call_time_ms"].asDouble() << ',' << spent << ','
          << row["solution_quality"].asString() << '\n';
    times.flush();
    ++counts[row["solution_quality"].asString()];
    if (!result.accepted)
      ++rejected;
  }
  std::ofstream quality(o.output + "/quality_counts.csv");
  quality << "quality,count\n";
  for (auto &item : counts)
    quality << item.first << ',' << item.second << '\n';
  Json::Value status;
  status["operation"] = "completed";
  status["attempts"] = total;
  status["rejected"] = rejected;
  status["dropped_timing_samples"] = 0;
  status["retention"] =
      "Unbounded sequential file stream flushed per attempt; write failure "
      "terminates when stream exceptions enabled";
  writeJson(o.output + "/native_status.json", status);
  return rejected ? 2 : 0;
}
} // namespace study_e09
