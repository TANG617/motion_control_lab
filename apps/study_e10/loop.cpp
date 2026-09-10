#include "loop.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <mutex>
#include <pthread.h>
#include <sched.h>
#include <sstream>
#include <stdexcept>
#include <thread>
namespace study_e10 {
namespace {
struct State {
  Json::Value sample;
  long long captured = 0, sequence = 0, committed_sequence = 0;
};
struct Proposal {
  Json::Value q;
  long long revision = 0, created = 0, captured = 0, state_sequence = 0,
            delivery = 0;
  bool accepted = false;
};
struct Recorder {
  std::ofstream raw;
  std::mutex mutex;
  std::vector<Json::Value> events;
  size_t capacity;
  long long overflow = 0;
  Json::StreamWriterBuilder writer;
  Recorder(const Options &o)
      : raw(o.output + "/raw.jsonl"),
        capacity(o.config["buffer_capacity"].asUInt64()) {
    writer["indentation"] = "";
    raw.exceptions(std::ios::badbit | std::ios::failbit);
    events.reserve(capacity);
  }
  void attempt(Json::Value row) {
    std::lock_guard<std::mutex> l(mutex);
    raw << Json::writeString(writer, row) << '\n';
    raw.flush();
  }
  void event(Json::Value row) {
    std::lock_guard<std::mutex> l(mutex);
    if (events.size() < capacity)
      events.push_back(std::move(row));
    else
      ++overflow;
  }
};
void pin(int cpu) {
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  int code = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
  if (code)
    throw std::runtime_error("pthread_setaffinity_np failed: " +
                             std::to_string(code));
}
std::vector<int> allowedCpus() {
  cpu_set_t set;
  CPU_ZERO(&set);
  if (sched_getaffinity(0, sizeof(set), &set))
    throw std::runtime_error("sched_getaffinity failed");
  std::vector<int> cpus;
  for (int i = 0; i < CPU_SETSIZE; ++i)
    if (CPU_ISSET(i, &set))
      cpus.push_back(i);
  return cpus;
}
Json::Value event(const char *worker, long long seq, long long release,
                  long long start, long long finish, long long period,
                  bool skip = false) {
  Json::Value e;
  e["worker"] = worker;
  e["sequence"] = Json::Int64(seq);
  e["release_ns"] = Json::Int64(release);
  e["start_ns"] = skip ? Json::Value() : Json::Value(Json::Int64(start));
  e["finish_ns"] = skip ? Json::Value() : Json::Value(Json::Int64(finish));
  e["deadline_ns"] = Json::Int64(release + period);
  e["skipped"] = skip;
  e["deadline_miss"] = skip || finish > release + period;
  return e;
}
} // namespace
int run(const Options &o, Solver &primary) {
  const auto &c = o.config;
  bool virtual_time = c["schedule_mode"].asString() == "virtual";
  std::string scheduler = c["scheduler"].asString();
  bool async = scheduler.find("asynchronous") == 0;
  long long duration = llround(c["duration_s"].asDouble() * 1e9),
            primary_period = llround(1e9 / c["solver_hz"].asDouble()),
            secondary_period = llround(1e9 / c["secondary_hz"].asDouble()),
            target_period = llround(1e9 / c["target_hz"].asDouble()),
            feedback_period = llround(1e9 / c["feedback_hz"].asDouble()),
            output_period = llround(1e9 / c["output_hz"].asDouble());
  long long proposal_delay = llround(c["proposal_delay_ms"].asDouble() * 1e6),
            state_delay = llround(c["state_delay_ms"].asDouble() * 1e6),
            pause_start = llround(c["pause_start_s"].asDouble() * 1e9),
            pause_end =
                pause_start + llround(c["pause_duration_s"].asDouble() * 1e9);
  auto cpus = allowedCpus();
  int primary_cpu = c["cpu_ids"].empty() ? cpus.at(0) : c["cpu_ids"][0].asInt();
  int second_cpu =
      scheduler == "asynchronous-two-core"
          ? (c["cpu_ids"].size() > 1 ? c["cpu_ids"][1].asInt() : cpus.at(1))
          : primary_cpu;
  Json::Value resources;
  resources["requested_cpu_ids"] = c["cpu_ids"];
  resources["effective_primary_cpu"] =
      virtual_time ? Json::Value() : Json::Value(primary_cpu);
  resources["effective_secondary_cpu"] =
      virtual_time ? Json::Value() : Json::Value(second_cpu);
  resources["resource_selection"] =
      c["cpu_ids"].empty() ? "development allowed cpuset selection; formal "
                             "requires explicitly frozen physical core IDs"
                           : "explicit";
  resources["schedule_mode"] = c["schedule_mode"];
  resources["thread_count"] =
      virtual_time ? 1
                   : (async ? 2 : 1) + (c["load"].asString() == "none" ? 0 : 1);
  resources["model"] =
      "ideal post-command kinematic state, no physical dynamics";
  resources["observation"] = "flush each raw attempt; fixed-capacity event "
                             "trace; diagnostic development timing";
  writeJson(o.output + "/resolved_config.json", c);
  writeJson(o.output + "/injection_schedule.json",
            o.input["injection_schedule"]);
  Options secondary_options = o;
  secondary_options.config["layers"] = 1;
  secondary_options.config["workload"] = 2;
  secondary_options.config["posture_only"] = true;
  secondary_options.config["dt_s"] = 1. / c["secondary_hz"].asDouble();
  Solver secondary(secondary_options);
  // Freeze every planned denominator before workers or native numerical calls
  // start.
  std::ofstream planned(o.output + "/planned_releases.csv");
  planned << "worker,sequence,release_ns,deadline_ns\n";
  for (const auto &clock : std::vector<std::pair<std::string, long long>>{
           {"primary", primary_period},
           {"secondary", secondary_period},
           {"target", target_period},
           {"feedback-capture", feedback_period},
           {"output", output_period}}) {
    for (long long i = 0; i * clock.second < duration; ++i)
      planned << clock.first << ',' << i << ',' << i * clock.second << ','
              << (i + 1) * clock.second << '\n';
  }
  planned.close();
  Json::Value injection = o.input["injection_schedule"];
  injection["resolved_injection"] = c["injection"];
  injection["resolved_start_s"] = c["pause_start_s"];
  injection["resolved_duration_s"] = c["pause_duration_s"];
  injection["resolved_proposal_delay_ms"] = c["proposal_delay_ms"];
  injection["resolved_state_delay_ms"] = c["state_delay_ms"];
  writeJson(o.output + "/injection_schedule.json", injection);
  Recorder recorder(o);
  std::mutex state_mutex, proposal_mutex;
  State truth{o.input["samples"][0], 0, 0}, measured = truth;
  Proposal published;
  std::atomic<bool> stop{false};
  std::atomic<long long> secondary_calls{0};
  double secondary_cpu = 0, load_cpu = 0;
  long long epoch = nowNs();
  auto time = [&]() { return nowNs() - epoch; };
  auto publish = [&](const Proposal &p, long long published_at) {
    std::lock_guard<std::mutex> lock(proposal_mutex);
    published = p;
    Json::Value e;
    e["worker"] = "proposal-publication";
    e["revision"] = Json::Int64(p.revision);
    e["created_ns"] = Json::Int64(p.created);
    e["state_capture_ns"] = Json::Int64(p.captured);
    e["delivery_ns"] = Json::Int64(p.delivery);
    e["publication_ns"] = Json::Int64(published_at);
    e["state_sequence"] = Json::Int64(p.state_sequence);
    e["accepted"] = p.accepted;
    recorder.event(e);
  };
  auto secondary_step = [&](long long index, long long release, long long start,
                            const State &captured) {
    ++secondary_calls;
    auto sample = captured.sample;
    Json::Value begin;
    begin["record_type"] = "attempt_begin";
    begin["worker"] = "secondary";
    begin["release_ns"] = Json::Int64(release);
    begin["start_ns"] = Json::Int64(start);
    begin["attempt_sequence"] = Json::Int64(index);
    recorder.attempt(begin);
    auto result = secondary.solve(sample);
    long long finish = virtual_time
                           ? start + c["virtual_secondary_cost_ns"].asInt64()
                           : time();
    auto row = secondary.evidence(result);
    row["record_type"] = "attempt";
    row["worker"] = "secondary";
    row["attempt_sequence"] = Json::Int64(index);
    row["input_sequence"] = sample["sequence"];
    row["source_time_s"] = sample["source_time_s"];
    row["state_sequence"] = Json::Int64(captured.sequence);
    row["task_revision"] = sample["sequence"];
    row["targets"] = sample["targets"];
    row["release_ns"] = Json::Int64(release);
    row["start_ns"] = Json::Int64(start);
    row["finish_ns"] = Json::Int64(finish);
    row["proposal_revision"] = Json::Int64(index + 1);
    row["proposal_created_ns"] = Json::Int64(finish);
    row["proposal_state_capture_ns"] = Json::Int64(captured.captured);
    row["committed"] = false;
    row["committed_sequence"] = Json::nullValue;
    row["execution_state"] = "proposal-only";
    row["clock_domain"] =
        virtual_time ? "deterministic-virtual" : "run-monotonic";
    recorder.attempt(row);
    recorder.event(
        event("secondary", index, release, start, finish, secondary_period));
    Proposal p;
    p.q = row["q"];
    p.revision = index + 1;
    p.created = finish;
    p.captured = captured.captured;
    p.state_sequence = captured.sequence;
    p.delivery = finish + proposal_delay;
    p.accepted = result.accepted;
    if (c["injection"].asString() == "publication-failure" &&
        release >= pause_start && release < pause_end)
      p.accepted = false;
    if (c["injection"].asString() == "old-version" && index > 0 &&
        release >= pause_start && release < pause_end)
      p.revision = index;
    return p;
  };
  std::thread secondary_thread, load_thread;
  if (async && !virtual_time)
    secondary_thread = std::thread([&]() {
      pin(second_cpu);
      double cpu_start = threadCpuSeconds();
      std::deque<Proposal> pending;
      long long index = 0;
      while (!stop) {
        long long now = time();
        while (!pending.empty() && pending.front().delivery <= now) {
          publish(pending.front(), now);
          pending.pop_front();
        }
        long long release = index * secondary_period;
        if (release >= duration)
          break;
        if (now < release) {
          std::this_thread::sleep_until(
              std::chrono::steady_clock::time_point(std::chrono::nanoseconds(
                  epoch + std::min(release, pending.empty()
                                                ? release
                                                : pending.front().delivery))));
          continue;
        }
        bool pause = c["injection"].asString() == "pause" &&
                     release >= pause_start && release < pause_end;
        if (pause || now >= release + secondary_period) {
          recorder.event(
              event("secondary", index, release, 0, 0, secondary_period, true));
          ++index;
          continue;
        }
        State state;
        {
          std::lock_guard<std::mutex> lock(state_mutex);
          state = measured;
        }
        auto p = secondary_step(index, release, now, state);
        pending.push_back(std::move(p));
        ++index;
      }
      secondary_cpu = threadCpuSeconds() - cpu_start;
    });
  if (c["load"].asString() != "none" && !virtual_time)
    load_thread = std::thread([&]() {
      int cpu = c["load"].asString() == "same-core"
                    ? primary_cpu
                    : (c["cpu_ids"].size() > 1 ? c["cpu_ids"][1].asInt()
                                               : cpus.at(1));
      pin(cpu);
      double begin = threadCpuSeconds();
      volatile double value = 1.001;
      long long iteration = 0;
      while (!stop) {
        long long release = iteration * 1000000;
        if (release >= duration)
          break;
        std::this_thread::sleep_until(std::chrono::steady_clock::time_point(
            std::chrono::nanoseconds(epoch + release)));
        long long start = time();
        double budget =
            threadCpuSeconds() + c["load_cpu_budget_us"].asDouble() * 1e-6;
        while (threadCpuSeconds() < budget)
          value = value * 1.0000001 + .0000001;
        recorder.event(
            event("cpu-load", iteration, release, start, time(), 1000000));
        ++iteration;
      }
      load_cpu = threadCpuSeconds() - begin;
    });
  if (!virtual_time)
    pin(primary_cpu);
  double primary_cpu_start = threadCpuSeconds();
  long long target_index = 0, feedback_index = 0, output_index = 0,
            secondary_index = 0, committed_sequence = 0, previous_revision = 0;
  State delivered = measured;
  Proposal local_proposal;
  Json::Value command = truth.sample["q"], velocity = truth.sample["v"];
  bool command_valid = false;
  long long command_attempt_sequence = -1;
  std::deque<State> feedback_pending;
  std::deque<Proposal> proposal_pending;
  int rejected = 0;
  long long main_attempts = 0, virtual_now = 0;
  for (long long index = 0; index * primary_period < duration; ++index) {
    long long release = index * primary_period;
    if (!virtual_time)
      std::this_thread::sleep_until(std::chrono::steady_clock::time_point(
          std::chrono::nanoseconds(epoch + release)));
    long long now = virtual_time ? std::max(virtual_now, release) : time();
    while (target_index * target_period <= now &&
           target_index * target_period < duration) {
      auto sample =
          o.input["samples"]
                 [Json::ArrayIndex(target_index % o.input["samples"].size())];
      truth.sample["targets"] = sample["targets"];
      truth.sample["sequence"] = sample["sequence"];
      truth.sample["canonical_source_time_s"] = sample["source_time_s"];
      truth.sample["source_time_s"] = target_index * target_period * 1e-9;
      recorder.event(event("target", target_index, target_index * target_period,
                           now, now, target_period));
      ++target_index;
    }
    while (feedback_index * feedback_period <= now &&
           feedback_index * feedback_period < duration) {
      State capture = truth;
      capture.captured = now;
      capture.sequence = feedback_index;
      capture.committed_sequence = truth.sequence;
      feedback_pending.push_back(capture);
      auto capture_event =
          event("feedback-capture", feedback_index,
                feedback_index * feedback_period, now, now, feedback_period);
      capture_event["q"] = capture.sample["q"];
      capture_event["v"] = capture.sample["v"];
      capture_event["captured_committed_sequence"] =
          Json::Int64(capture.committed_sequence);
      capture_event["capture_ns"] = Json::Int64(capture.captured);
      recorder.event(capture_event);
      ++feedback_index;
    }
    while (!feedback_pending.empty() &&
           feedback_pending.front().captured + state_delay <= now) {
      delivered = feedback_pending.front();
      feedback_pending.pop_front();
      std::lock_guard<std::mutex> lock(state_mutex);
      measured = delivered;
    }
    if (!async || virtual_time) {
      while (secondary_index * secondary_period <= now &&
             secondary_index * secondary_period < duration) {
        long long sr = secondary_index * secondary_period;
        bool pause = c["injection"].asString() == "pause" &&
                     sr >= pause_start && sr < pause_end;
        if (pause || now >= sr + secondary_period) {
          recorder.event(event("secondary", secondary_index, sr, 0, 0,
                               secondary_period, true));
          ++secondary_index;
          continue;
        }
        auto p = secondary_step(secondary_index, sr, now, delivered);
        if (!async)
          now = p.created;
        proposal_pending.push_back(std::move(p));
        ++secondary_index;
      }
      while (!proposal_pending.empty() &&
             proposal_pending.front().delivery <= now) {
        publish(proposal_pending.front(), now);
        proposal_pending.pop_front();
      }
    }
    {
      std::unique_lock<std::mutex> lock(proposal_mutex, std::try_to_lock);
      if (lock.owns_lock())
        local_proposal = published;
      else {
        Json::Value e;
        e["worker"] = "proposal-consumption-contention";
        e["release_ns"] = Json::Int64(release);
        recorder.event(e);
      }
    }
    if (now >= release + primary_period) {
      recorder.event(
          event("primary", index, release, 0, 0, primary_period, true));
    } else {
      auto sample = delivered.sample;
      sample["targets"] = truth.sample["targets"];
      sample["sequence"] = truth.sample["sequence"];
      sample["source_time_s"] = truth.sample["source_time_s"];
      bool coupling = c["coupling_enabled"].asBool() && local_proposal.accepted;
      sample["disable_posture"] = !coupling;
      Json::Value begin;
      begin["record_type"] = "attempt_begin";
      begin["worker"] = "primary";
      begin["release_ns"] = Json::Int64(release);
      begin["start_ns"] = Json::Int64(virtual_time ? now : time());
      begin["attempt_sequence"] = Json::Int64(index);
      recorder.attempt(begin);
      long long start = virtual_time ? now : time();
      auto result =
          primary.solve(sample, coupling ? local_proposal.q : Json::Value());
      long long finish = virtual_time
                             ? start + c["virtual_primary_cost_ns"].asInt64()
                             : time();
      auto row = primary.evidence(result);
      row["record_type"] = "attempt";
      row["worker"] = "primary";
      row["attempt_sequence"] = Json::Int64(index);
      row["input_sequence"] = sample["sequence"];
      row["source_time_s"] = sample["source_time_s"];
      row["state_sequence"] = Json::Int64(delivered.sequence);
      row["task_revision"] = Json::Int64(target_index - 1);
      row["targets"] = sample["targets"];
      row["release_ns"] = Json::Int64(release);
      row["start_ns"] = Json::Int64(start);
      row["finish_ns"] = Json::Int64(finish);
      row["proposal_revision"] = Json::Int64(local_proposal.revision);
      row["proposal_created_ns"] =
          local_proposal.revision
              ? Json::Value(Json::Int64(local_proposal.created))
              : Json::Value();
      row["proposal_state_capture_ns"] =
          local_proposal.revision
              ? Json::Value(Json::Int64(local_proposal.captured))
              : Json::Value();
      row["proposal_age_ns"] =
          local_proposal.revision
              ? Json::Value(Json::Int64(start - local_proposal.created))
              : Json::Value();
      row["proposal_state_age_ns"] =
          local_proposal.revision
              ? Json::Value(Json::Int64(start - local_proposal.captured))
              : Json::Value();
      row["measured_state_capture_ns"] = Json::Int64(delivered.captured);
      row["measured_state_age_ns"] = Json::Int64(start - delivered.captured);
      row["coupling_enabled"] = coupling;
      row["proposal_repeated"] = local_proposal.revision != 0 &&
                                 local_proposal.revision == previous_revision;
      row["proposal_stale_version"] =
          local_proposal.revision < previous_revision;
      row["truth_q"] = truth.sample["q"];
      row["measured_q"] = delivered.sample["q"];
      row["measured_state_committed_sequence"] =
          Json::Int64(delivered.committed_sequence);
      row["clock_domain"] =
          virtual_time ? "deterministic-virtual" : "run-monotonic";
      previous_revision = local_proposal.revision;
      // Preserve the native acceptance gate. A rejected attempt never supplies
      // a command.
      if (result.accepted) {
        command = row["q"];
        velocity = row["v"];
        command_valid = true;
        command_attempt_sequence = index;
      } else {
        ++rejected;
        command_valid = false;
      }
      row["committed"] = false;
      row["committed_sequence"] = Json::Int64(committed_sequence);
      row["execution_state"] =
          result.accepted ? "command-pending-output" : "HOLD";
      recorder.attempt(row);
      recorder.event(
          event("primary", index, release, start, finish, primary_period));
      ++main_attempts;
      now = finish;
    }
    while (output_index * output_period <= now &&
           output_index * output_period < duration) {
      bool did_commit = command_valid;
      if (command_valid) {
        truth.sample["q"] = command;
        truth.sample["v"] = velocity;
        ++committed_sequence;
        truth.sequence = committed_sequence;
        truth.captured = now;
        command_valid = false;
      }
      auto e = event("output", output_index, output_index * output_period, now,
                     now, output_period);
      e["q"] = truth.sample["q"];
      e["v"] = truth.sample["v"];
      e["targets"] = truth.sample["targets"];
      e["input_sequence"] = truth.sample["sequence"];
      e["committed_sequence"] = Json::Int64(committed_sequence);
      e["committed"] = did_commit;
      e["execution_state"] = did_commit ? "committed" : "HOLD";
      e["command_attempt_sequence"] =
          did_commit ? Json::Value(Json::Int64(command_attempt_sequence))
                     : Json::Value();
      recorder.event(e);
      ++output_index;
    }
    virtual_now = now;
  }
  stop = true;
  if (secondary_thread.joinable())
    secondary_thread.join();
  if (load_thread.joinable())
    load_thread.join();
  double primary_cpu_seconds = threadCpuSeconds() - primary_cpu_start;
  // Retain any planned clock suffix explicitly, including worker termination
  // suffixes.
  auto suffix = [&](const char *name, long long index, long long period) {
    for (; index * period < duration; ++index)
      recorder.event(event(name, index, index * period, 0, 0, period, true));
  };
  suffix("target", target_index, target_period);
  suffix("feedback-capture", feedback_index, feedback_period);
  suffix("output", output_index, output_period);
  std::ofstream timeline(o.output + "/worker_timeline.jsonl"),
      csv(o.output + "/worker_timeline.csv"),
      coupling(o.output + "/coupling_trace.csv");
  csv << "worker,sequence,release_ns,start_ns,finish_ns,deadline_ns,skipped,"
         "deadline_miss\n";
  for (auto &e : recorder.events) {
    timeline << Json::writeString(recorder.writer, e) << '\n';
    if (e.isMember("sequence"))
      csv << e["worker"].asString() << ',' << e["sequence"].asInt64() << ','
          << e["release_ns"].asInt64() << ','
          << (e["start_ns"].isNull() ? "" : e["start_ns"].asString()) << ','
          << (e["finish_ns"].isNull() ? "" : e["finish_ns"].asString()) << ','
          << e["deadline_ns"].asInt64() << ',' << e["skipped"].asBool() << ','
          << e["deadline_miss"].asBool() << '\n';
  }
  coupling << "attempt_sequence,input_sequence,state_sequence,proposal_"
              "revision,proposal_created_ns,proposal_state_capture_ns,start_ns,"
              "proposal_age_ns,measured_state_age_ns,coupling_enabled,proposal_"
              "repeated,proposal_stale_version\n";
  {
    std::ifstream source(o.output + "/raw.jsonl");
    std::string line;
    while (std::getline(source, line)) {
      Json::Value row;
      Json::CharReaderBuilder reader;
      std::string errors;
      std::istringstream record(line);
      Json::parseFromStream(reader, record, &row, &errors);
      if (row["record_type"] != "attempt" || row["worker"] != "primary")
        continue;
      bool first = true;
      for (auto field :
           {"attempt_sequence", "input_sequence", "state_sequence",
            "proposal_revision", "proposal_created_ns",
            "proposal_state_capture_ns", "start_ns", "proposal_age_ns",
            "measured_state_age_ns", "coupling_enabled", "proposal_repeated",
            "proposal_stale_version"}) {
        if (!first)
          coupling << ',';
        first = false;
        if (!row[field].isNull())
          coupling << row[field].asString();
      }
      coupling << '\n';
    }
  }
  resources["worker_cpu_time_s"] = primary_cpu_seconds + secondary_cpu;
  resources["primary_thread_cpu_time_s"] = primary_cpu_seconds;
  resources["secondary_thread_cpu_time_s"] =
      async && !virtual_time ? Json::Value(secondary_cpu) : Json::Value();
  resources["secondary_cpu_reason"] =
      async && !virtual_time ? "separate thread clock"
                             : "included in single primary thread CPU";
  resources["load_thread_cpu_time_s"] = load_cpu;
  resources["wall_duration_s"] = (nowNs() - epoch) * 1e-9;
  writeJson(o.output + "/resource_summary.json", resources);
  std::ofstream resource_csv(o.output + "/resource_summary.csv");
  resource_csv << "configured_primary_cpu,configured_secondary_cpu,primary_"
                  "thread_cpu_s,secondary_"
                  "thread_cpu_s,load_thread_cpu_s,wall_duration_s\n"
               << primary_cpu << ',' << second_cpu << ',' << primary_cpu_seconds
               << ',' << secondary_cpu << ',' << load_cpu << ','
               << resources["wall_duration_s"].asDouble() << '\n';
  Json::Value status;
  status["operation"] = "completed";
  status["main_attempts"] = Json::Int64(main_attempts);
  status["secondary_calls"] = Json::Int64(secondary_calls.load());
  status["rejected"] = rejected;
  status["timing_buffer_overflow"] = Json::Int64(recorder.overflow);
  status["timing_completeness"] =
      recorder.overflow ? "missing-samples" : "complete";
  status["formal_evidence"] = false;
  writeJson(o.output + "/native_status.json", status);
  return rejected || recorder.overflow ? 2 : 0;
}
} // namespace study_e10
