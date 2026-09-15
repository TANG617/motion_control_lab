#include "execution.hpp"
#include "adapters/execution/include/motion_control_lab/execution_request.hpp"
#include "solver.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <set>
namespace motion_control_lab::step {
namespace ex = motion_control_lab::execution;
namespace {
void keys(const Json::Value &v, std::initializer_list<const char *> allowed) {
  std::set<std::string> names;
  for (auto k : allowed)
    names.insert(k);
  for (const auto &k : v.getMemberNames())
    if (!names.count(k))
      throw std::runtime_error("unsupported option: " + k);
}
std::vector<double> vector(const Json::Value &v) {
  std::vector<double> x;
  for (auto &a : v)
    x.push_back(a.asDouble());
  return x;
}
Json::Value json(const std::vector<double> &v) {
  Json::Value x(Json::arrayValue);
  for (auto a : v)
    x.append(a);
  return x;
}
AppOptions options(const Json::Value &c) {
  keys(c, {"urdf_path", "solver", "backend", "rate_hz", "task_layout",
           "algorithm", "target_space"});
  AppOptions o;
  o.interactive.urdf_path = c["urdf_path"].asString();
  auto solver = c.get("solver", "mcc").asString();
  if (solver != "mcc" && solver != "placo")
    throw std::runtime_error("unsupported solver");
  o.solver = solver == "mcc" ? SolverKind::Mcc : SolverKind::Placo;
  auto backend =
      c.get("backend", solver == "placo" ? "eiquadprog" : "proxqp").asString();
  if (backend != "proxqp" && backend != "eiquadprog")
    throw std::runtime_error("unsupported backend");
  if (solver == "placo" && backend != "eiquadprog")
    throw std::runtime_error("PlaCo only supports eiquadprog");
  o.backend = backend == "proxqp" ? MccBackend::Proxqp : MccBackend::Eiquadprog;
  o.interactive.rate_hz = c.get("rate_hz", 100.).asDouble();
  if (o.interactive.rate_hz <= 0)
    throw std::runtime_error("rate_hz must be positive");
  if (c.get("task_layout", "dual-arm-hard-pose") != "dual-arm-hard-pose")
    throw std::runtime_error("unsupported task layout");
  const auto &a = c["algorithm"];
  keys(a, {"regularization", "position_tolerance_m",
           "orientation_tolerance_rad", "joint_position_margin_rad"});
  if (a.isMember("regularization"))
    o.algorithm.regularization = a["regularization"].asDouble();
  if (a.isMember("position_tolerance_m"))
    o.algorithm.position_tolerance_m = a["position_tolerance_m"].asDouble();
  if (a.isMember("orientation_tolerance_rad"))
    o.algorithm.orientation_tolerance_rad =
        a["orientation_tolerance_rad"].asDouble();
  if (a.isMember("joint_position_margin_rad"))
    o.algorithm.joint_position_margin_rad =
        a["joint_position_margin_rad"].asDouble();
  if (c.get("target_space", "frame") != "frame" && c["target_space"] != "tcp")
    throw std::runtime_error("unsupported target_space");
  return o;
}
Json::Value resolve(const AppOptions &o) {
  Json::Value r;
  r["app_id"] = "mcl_step";
  r["app_version"] = "2";
  r["urdf_path"] = o.interactive.urdf_path;
  r["solver"] = o.solver == SolverKind::Mcc ? "mcc" : "placo";
  r["backend"] =
      o.solver == SolverKind::Placo
          ? "eiquadprog"
          : (o.backend == MccBackend::Proxqp ? "proxqp" : "eiquadprog");
  r["rate_hz"] = o.interactive.rate_hz;
  r["task_layout"] = "dual-arm-hard-pose";
  r["algorithm"]["regularization"] = o.algorithm.regularization;
  r["algorithm"]["position_tolerance_m"] = o.algorithm.position_tolerance_m;
  r["algorithm"]["orientation_tolerance_rad"] =
      o.algorithm.orientation_tolerance_rad;
  r["algorithm"]["joint_position_margin_rad"] =
      o.algorithm.joint_position_margin_rad;
  return r;
}
template <class Solver>
std::vector<ArmTarget> targets(const Json::Value &sample, Solver &solver,
                               bool tcp) {
  std::vector<ArmTarget> result;
  for (int k = 0; k < 2; ++k) {
    auto side = k == 0 ? ArmSide::Left : ArmSide::Right;
    Pose pose = solver.currentPose(side);
    const auto &t = sample["targets"][k == 0 ? "left" : "right"];
    if (!t.isNull()) {
      for (int j = 0; j < 3; ++j) {
        pose.translation()[j] = t["position"][j].asDouble();
        for (int l = 0; l < 3; ++l)
          pose.linear()(j, l) = t["rotation"][j][l].asDouble();
      }
    }
    if (tcp && !t.isNull())
      pose = pose * (k == 0 ? r1RobotConfig().left_tcp_offset
                            : r1RobotConfig().right_tcp_offset)
                        .inverse();
    ArmTarget target;
    target.side = side;
    target.target_pose = pose;
    result.push_back(target);
  }
  return result;
}
template <class Solver>
int batch(ex::Request &r, const AppOptions &o, Solver &solver, double init_ms) {
  const auto &e = r.document["execution"];
  keys(e, {"cold_calls", "warmup_calls", "measured_calls", "state_mode"});
  keys(r.document["observation"], {"mode"});
  auto observation = r.document["observation"].get("mode", "full").asString();
  if (observation != "full" && observation != "minimal" &&
      observation != "cpp-new")
    throw std::runtime_error("unsupported observation mode");
  const auto &samples = r.input["samples"];
  int cold = e.get("cold_calls", 1).asInt(),
      warm = e.get("warmup_calls", 0).asInt();
  int measured = e.get("measured_calls", int(samples.size()) - cold).asInt();
  if (cold < 0 || warm < 0 || measured < 0 || samples.empty())
    throw std::runtime_error(
        "nonempty samples and nonnegative call windows required");
  auto state_mode = e.get("state_mode", "evolving").asString();
  if (state_mode != "evolving" && state_mode != "snapshot")
    throw std::runtime_error("unsupported state_mode");
  auto q0 = r.input.isMember("initial_state")
                ? vector(r.input["initial_state"]["q"])
                : r1RobotConfig().default_positions;
  auto v0 = r.input.isMember("initial_state")
                ? vector(r.input["initial_state"]["v"])
                : std::vector<double>(q0.size(), 0.);
  solver.setState(q0, v0);
  auto mapping = solver.modelMapping();
  mapping["input_target_space"] = r.config().get("target_space", "frame");
  mapping["raw_target_space"] = "frame";
  mapping["position_margin_rad"] = o.algorithm.joint_position_margin_rad;
  mapping["velocity_limits_enforced"] = true;
  mapping["bounds_include_margin"] = o.solver == SolverKind::Placo;
  if (o.solver == SolverKind::Mcc)
    for (unsigned j = 0; j < mapping["position_lower"].size(); ++j) {
      mapping["position_lower"][j] = mapping["position_lower"][j].asDouble() +
                                     o.algorithm.joint_position_margin_rad;
      mapping["position_upper"][j] = mapping["position_upper"][j].asDouble() -
                                     o.algorithm.joint_position_margin_rad;
    }
  mapping["bounds_include_margin"] = true;
  ex::writeJson(r.output / "model_mapping.json", mapping);
  Json::Value plan;
  plan["planned_calls"] = cold + warm + measured;
  plan["cold_calls"] = cold;
  plan["warmup_calls"] = warm;
  plan["measured_calls"] = measured;
  plan["unrecorded_suffix_status"] = "not-run";
  plan["initialization_ms"] = init_ms;
  ex::writeJson(r.output / "call_plan.json", plan);
  std::ofstream raw(r.output / "raw.jsonl");
  raw.exceptions(std::ios::badbit | std::ios::failbit);
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  int index = 0;
  std::string phase;
  auto emit = [&](Json::Value row) {
    row["call_index"] = index;
    row["phase"] = phase;
    raw << Json::writeString(writer, row) << '\n';
    raw.flush();
  };
  solver.observe([&](const Json::Value &row) { emit(row); },
                 observation != "minimal", observation == "cpp-new");
  int rejected = 0;
  for (index = 0; index < cold + warm + measured; ++index) {
    phase = index < cold ? "cold" : index < cold + warm ? "warmup" : "steady";
    const auto &sample = samples[index % samples.size()];
    if (state_mode == "snapshot")
      solver.setState(sample.isMember("q") ? vector(sample["q"]) : q0,
                      sample.isMember("v") ? vector(sample["v"]) : v0);
    auto desired = targets(sample, solver,
                           r.config().get("target_space", "frame") == "tcp");
    Json::Value begin;
    begin["record_type"] = "attempt_begin";
    begin["input_state"]["q"] = json(solver.positions());
    begin["input_state"]["v"] = json(solver.velocities());
    begin["sample_index"] = index % samples.size();
    begin["source_time_s"] = sample["source_time_s"];
    for (int arm = 0; arm < 2; ++arm) {
      auto &t = begin["targets"][arm == 0 ? "left" : "right"];
      for (int j = 0; j < 3; ++j) {
        t["position"].append(desired[arm].target_pose.translation()[j]);
        Json::Value row(Json::arrayValue);
        for (int k = 0; k < 3; ++k)
          row.append(desired[arm].target_pose.linear()(j, k));
        t["rotation"].append(row);
      }
    }
    emit(begin);
    const auto started = std::chrono::steady_clock::now();
    const auto result = solver.solve(desired);
    const double elapsed = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - started)
                               .count();
    Json::Value row;
    row["record_type"] = "result";
    row["committed"] = true;
    row["q"] = json(solver.positions());
    row["v"] = json(solver.velocities());
    row["converged"] = result.converged;
    row["iterations"] = result.iterations;
    row["disposition"] = result.solver_debug.disposition;
    row["qp_status"] = result.solver_debug.qp_status;
    row["termination_reason"] = result.solver_debug.termination_reason;
    row["app_solve_with_observer_ms"] = elapsed;
    row["solver_reported_ms"] = result.solve_time_ms;
    for (const auto &error : result.target_errors) {
      Json::Value a;
      a["position_m"] = error.position_m;
      a["orientation_rad"] = error.orientation_rad;
      row["target_errors"].append(a);
    }
    emit(row);
    if (!row["committed"].asBool())
      ++rejected;
  }
  solver.observe({});
  Json::Value summary;
  summary["completed_calls"] = index;
  summary["rejected_calls"] = rejected;
  summary["raw_sha256"] = sha256_file(r.output / "raw.jsonl");
  summary["scientific_admission"] =
      "requires-new-equation-and-acceptance-audit";
  ex::writeJson(r.output / "summary.json", summary);
  return rejected ? 1 : 0;
}
} // namespace
Json::Value resolvedOptions(const AppOptions &o) { return resolve(o); }
Json::Value capabilities() {
  Json::Value c;
  c["schema_version"] = "app_capabilities.v1";
  c["app_id"] = "mcl_step";
  c["app_version"] = "2";
  c["execution_structures"].append("servo_step");
  c["task_layouts"].append("dual-arm-hard-pose");
  c["solvers"]["mcc"]["backends"].append("proxqp");
  c["solvers"]["mcc"]["backends"].append("eiquadprog");
  c["solvers"]["placo"]["backends"].append("eiquadprog");
  c["input_formats"].append("json");
  c["observation_modes"].append("full");
  c["observation_modes"].append("minimal");
  c["observation_modes"].append("cpp-new");
  c["state_modes"].append("snapshot");
  c["state_modes"].append("evolving");
  c["acceptance_contract"] = "existing-app-native-v1";
  c["native_pose_enforcement"] = "hard";
  return c;
}
bool publicExecution(int argc, char **argv) {
  return argc > 1 && (std::string(argv[1]) == "--describe-capabilities" ||
                      std::string(argv[1]) == "--request" ||
                      std::string(argv[1]) == "batch");
}
int executePublic(int argc, char **argv) {
  if (std::string(argv[1]) == "--describe-capabilities") {
    if (argc != 2)
      throw std::runtime_error("capabilities accepts no overrides");
    std::cout << capabilities() << '\n';
    return 0;
  }
  ex::Request r;
  bool dump = false;
  if (ex::isRequest(argc, argv)) {
    dump = ex::requestDumpOnly(argc, argv);
    r = ex::readRequest(argv[2], "mcl_step");
  } else {
    std::filesystem::path input, config, output, execution;
    for (int i = 2; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--dump-resolved-options") {
        dump = true;
        continue;
      }
      if (i + 1 >= argc)
        throw std::runtime_error("missing batch option value");
      if (arg == "--input")
        input = argv[++i];
      else if (arg == "--config")
        config = argv[++i];
      else if (arg == "--output")
        output = argv[++i];
      else if (arg == "--execution")
        execution = argv[++i];
      else
        throw std::runtime_error("unsupported batch option: " + arg);
    }
    r.input = ex::readJson(input);
    r.output = std::filesystem::absolute(output);
    r.document["schema_version"] = "execution_request.v1";
    r.document["app_id"] = "mcl_step";
    r.document["execution_structure"] = "servo_step";
    r.document["input"]["path"] = std::filesystem::absolute(input).string();
    r.document["input"]["sha256"] = sha256_file(input);
    r.document["input"]["format"] = "json";
    r.document["app_config"] = ex::readJson(config);
    r.document["execution"] = execution.empty() ? Json::Value(Json::objectValue)
                                                : ex::readJson(execution);
    r.document["observation"] = Json::Value(Json::objectValue);
    r.document["tracking"] = Json::Value(Json::objectValue);
    r.document["output_dir"] = r.output.string();
    r.sha256 = "normal-batch-no-request-file";
  }
  if (r.document["execution_structure"] != "servo_step" ||
      r.document["input"]["format"] != "json")
    throw std::runtime_error("unsupported execution structure/input format");
  const auto o = options(r.config());
  auto resolved = resolve(o);
  resolved["execution"] = r.document["execution"];
  resolved["observation"] = r.document["observation"];
  resolved["target_space"] = r.config().get("target_space", "frame");
  keys(r.document["execution"],
       {"cold_calls", "warmup_calls", "measured_calls", "state_mode"});
  keys(r.document["observation"], {"mode"});
  auto mode = r.document["observation"].get("mode", "full").asString();
  if (mode != "full" && mode != "minimal" && mode != "cpp-new")
    throw std::runtime_error("unsupported observation mode");
  auto state = r.document["execution"].get("state_mode", "evolving").asString();
  if (state != "snapshot" && state != "evolving")
    throw std::runtime_error("unsupported state mode");
  const auto &samples = r.input["samples"];
  if (samples.empty())
    throw std::runtime_error("nonempty samples required");
  for (const char *key : {"cold_calls", "warmup_calls", "measured_calls"})
    if (r.document["execution"].isMember(key) &&
        r.document["execution"][key].asInt() < 0)
      throw std::runtime_error("negative call count");
  resolved["execution"]["cold_calls"] =
      r.document["execution"].get("cold_calls", 1);
  resolved["execution"]["warmup_calls"] =
      r.document["execution"].get("warmup_calls", 0);
  resolved["execution"]["measured_calls"] = r.document["execution"].get(
      "measured_calls",
      int(samples.size()) - resolved["execution"]["cold_calls"].asInt());
  resolved["execution"]["state_mode"] = state;
  resolved["observation"]["mode"] = mode;
  if (resolved["execution"]["measured_calls"].asInt() < 0)
    throw std::runtime_error(
        "call windows require explicit nonnegative measured_calls");
  for (const auto &key : resolved["algorithm"].getMemberNames())
    if (resolved["algorithm"][key].asDouble() < 0)
      throw std::runtime_error("negative algorithm option: " + key);
  const auto &robot_config = r1RobotConfig();
  if (r.input["joint_names"].size() != robot_config.joint_names.size())
    throw std::runtime_error("joint_names dimensions differ from app");
  for (unsigned j = 0; j < robot_config.joint_names.size(); ++j)
    if (r.input["joint_names"][j].asString() != robot_config.joint_names[j])
      throw std::runtime_error("joint_names order differs from app");
  auto checkState = [&](const Json::Value &s) {
    if (s["q"].size() != robot_config.joint_names.size() ||
        s["v"].size() != robot_config.joint_names.size())
      throw std::runtime_error("state dimensions differ from app");
  };
  if (r.input.isMember("initial_state"))
    checkState(r.input["initial_state"]);
  for (const auto &sample : samples)
    if (sample.isMember("q") || sample.isMember("v"))
      checkState(sample);
  if (r.input.isMember("root_frame") &&
      r.input["root_frame"] != robot_config.base_frame)
    throw std::runtime_error("input root frame differs from app");
  if (r.input.isMember("frames") &&
      (r.input["frames"]["left"] != robot_config.left_end_effector_frame ||
       r.input["frames"]["right"] != robot_config.right_end_effector_frame))
    throw std::runtime_error("input task frames differ from app");
  if (resolved["target_space"] == "tcp" && r.input.isMember("tcp_offsets"))
    for (int j = 0; j < 3; ++j)
      if (r.input["tcp_offsets"]["left"][j].asDouble() !=
              robot_config.left_tcp_offset.translation()[j] ||
          r.input["tcp_offsets"]["right"][j].asDouble() !=
              robot_config.right_tcp_offset.translation()[j])
        throw std::runtime_error("input TCP offsets differ from app");
  if (!r.input["model"].isObject() ||
      r.input["model"]["sha256"].asString() !=
          sha256_file(o.interactive.urdf_path) ||
      !std::filesystem::equivalent(r.input["model"]["locator"].asString(),
                                   o.interactive.urdf_path))
    throw std::runtime_error(
        "input model binding differs from configured URDF");
  if (dump) {
    std::cout << resolved << '\n';
    return 0;
  }
  ex::beginOutput(r, resolved, capabilities());
  Json::Value model;
  model["urdf_path"] = o.interactive.urdf_path;
  model["sha256"] = sha256_file(o.interactive.urdf_path);
  ex::writeJson(r.output / "model_binding.json", model);
  const auto started = std::chrono::steady_clock::now();
  const auto &robot = r1RobotConfig();
  if (o.solver == SolverKind::Mcc) {
    MccServoSolver solver(o.interactive.urdf_path, o.interactive.rate_hz, robot,
                          o.backend, o.algorithm);
    return batch(r, o, solver,
                 std::chrono::duration<double, std::milli>(
                     std::chrono::steady_clock::now() - started)
                     .count());
  }
  PlacoServoSolver solver(o.interactive.urdf_path, o.interactive.rate_hz, robot,
                          o.algorithm);
  return batch(r, o, solver,
               std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - started)
                   .count());
}
} // namespace motion_control_lab::step
