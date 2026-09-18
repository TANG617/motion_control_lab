#include "options.hpp"
#include "adapters/replay/replay_support.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
namespace motion_control_lab::planned_kinematics_step {
Json::Value capabilities() {
  Json::Value c;
  c["schema_version"] = "app_capabilities.v1";
  c["app_id"] = "mcl_planned_kinematics_step";
  c["app_version"] = "1";
  c["execution_structures"].append("cartesian-weighted-ik-joint-otg");
  c["task_layouts"].append("dual-arm-pose-posture");
  c["solvers"].append("mcc");
  c["solvers"].append("placo");
  c["backends"].append("eiquadprog");
  for (auto f : {"json", "csv", "mcap"})
    c["input_formats"].append(f);
  c["runtime_modes"].append("virtual");
  for (auto f :
       {"full-raw", "solve-call-timing", "planned-releases", "native-failures"})
    c["observation"].append(f);
  c["failure_policy"] =
      "record native rejection and stop; remaining releases not-run";
  c["acceptance_contract"] =
      "native weighted acceptance then native Cartesian/JointPtpTrajectoryGenerator success; "
      "no candidate projection";
  return c;
}
Options parse(int argc, char **argv) {
  Options o;
  if (execution::isRequest(argc, argv)) {
    o.dump_only = execution::requestDumpOnly(argc, argv);
    o.request = execution::readRequest(argv[2], "mcl_planned_kinematics_step");
  } else {
    Json::Value d;
    d["schema_version"] = "execution_request.v1";
    d["app_id"] = "mcl_planned_kinematics_step";
    d["execution_structure"] = "cartesian-weighted-ik-joint-otg";
    d["execution"]["runtime_mode"] = "virtual";
    d["observation"] = Json::Value(Json::objectValue);
    d["tracking"] = Json::Value(Json::objectValue);
    for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      if (key == "--dump-resolved-options") {
        o.dump_only = true;
        continue;
      }
      if (i + 1 == argc)
        throw std::runtime_error("missing value: " + key);
      const std::string value = argv[++i];
      if (key == "--input")
        d["input"]["path"] = std::filesystem::absolute(value).string();
      else if (key == "--input-format")
        d["input"]["format"] = value;
      else if (key == "--config")
        d["app_config"] = execution::readJson(value);
      else if (key == "--output-dir")
        d["output_dir"] = std::filesystem::absolute(value).string();
      else
        throw std::runtime_error("unknown option: " + key);
    }
    if (!d["input"].isMember("path") || !d.isMember("app_config") ||
        !d.isMember("output_dir"))
      throw std::runtime_error("expected --input FILE --input-format "
                               "json|csv|mcap --config FILE --output-dir DIR");
    if (!d["input"].isMember("format"))
      d["input"]["format"] = "json";
    d["input"]["sha256"] = sha256_file(d["input"]["path"].asString());
    o.request.document = d;
    o.request.output = d["output_dir"].asString();
    o.request.sha256 = ""; // standalone invocation has no request file; full
                           // document is recorded.
    if (d["input"]["format"] == "json")
      o.request.input = execution::readJson(d["input"]["path"].asString());
  }
  const auto &d = o.request.document;
  for (const auto &key : d["execution"].getMemberNames())
    if (key != "runtime_mode")
      throw std::runtime_error("unsupported execution setting: " + key);
  for (const auto &key : d["observation"].getMemberNames())
    if (key != "mode" || d["observation"][key] != "full-raw")
      throw std::runtime_error("only full-raw observation is supported");
  if (d["execution_structure"] != "cartesian-weighted-ik-joint-otg")
    throw std::runtime_error("unsupported execution_structure");
  if (d["execution"].get("runtime_mode", "virtual") != "virtual")
    throw std::runtime_error(
        "only virtual replay is supported; no RT scheduling claim");
  const std::set<std::string> keys = {"solver",
                                      "backend",
                                      "dt_s",
                                      "duration_s",
                                      "servo_gain_per_s",
                                      "posture_weight",
                                      "regularization",
                                      "cartesian_velocity",
                                      "cartesian_acceleration",
                                      "cartesian_jerk",
                                      "joint_acceleration",
                                      "joint_jerk",
                                      "robot_input",
                                      "replay"};
  for (const auto &k : d["app_config"].getMemberNames())
    if (!keys.count(k))
      throw std::runtime_error("unsupported app_config key: " + k);
  o.config = d["app_config"];
  auto def = [&](const char *key, Json::Value value) {
    if (!o.config.isMember(key))
      o.config[key] = value;
  };
  def("solver", "mcc");
  def("backend", "eiquadprog");
  def("dt_s", 0.01);
  def("duration_s", 1.0);
  def("servo_gain_per_s", 1.0);
  def("posture_weight", 0.001);
  def("regularization", 1e-6);
  def("cartesian_velocity", 0.1);
  def("cartesian_acceleration", 0.5);
  def("cartesian_jerk", 5.0);
  def("joint_acceleration", 2.0);
  def("joint_jerk", 20.0);
  if (o.config["solver"] != "mcc" && o.config["solver"] != "placo")
    throw std::runtime_error("unsupported solver: weighted mcc|placo only");
  if (o.config["backend"] != "eiquadprog")
    throw std::runtime_error("unsupported backend");
  for (auto k :
       {"dt_s", "duration_s", "cartesian_velocity", "cartesian_acceleration",
        "cartesian_jerk", "joint_acceleration", "joint_jerk"})
    if (!std::isfinite(o.config[k].asDouble()) || o.config[k].asDouble() <= 0)
      throw std::runtime_error(std::string("expected positive ") + k);
  // The release clock is explicitly a nanosecond grid, avoiding
  // ceil(0.07/0.01)=8.
  auto nanoseconds = [&](const char *key) {
    const long double value =
        static_cast<long double>(o.config[key].asDouble()) * 1e9L;
    if (value < 0.5L || value > static_cast<long double>(
                                    std::numeric_limits<std::int64_t>::max()) -
                                    1)
      throw std::runtime_error(
          std::string("time is outside nanosecond clock range: ") + key);
    return static_cast<std::int64_t>(std::llround(value));
  };
  o.period_ns = nanoseconds("dt_s");
  o.duration_ns = nanoseconds("duration_s");
  const auto releases =
      o.duration_ns / o.period_ns + (o.duration_ns % o.period_ns != 0);
  if (releases > std::numeric_limits<int>::max())
    throw std::runtime_error(
        "planned release count exceeds supported index range");
  o.planned_releases = static_cast<int>(releases);
  const auto format = d["input"]["format"].asString();
  if (format != "json" && format != "csv" && format != "mcap")
    throw std::runtime_error("unsupported input format");
  auto reference = [](const Json::Value &ref, const char *label) {
    for (const auto &key : ref.getMemberNames())
      if (key != "path" && key != "sha256")
        throw std::runtime_error(std::string("unsupported ") + label +
                                 " key: " + key);
    const auto path = ref["path"].asString();
    if (!std::filesystem::path(path).is_absolute() ||
        ref["sha256"].asString() != sha256_file(path))
      throw std::runtime_error(std::string(label) + " path or sha256 mismatch");
    return path;
  };
  o.output = o.request.output;
  o.input = o.request.input;
  if (format == "json") {
    if (o.config.isMember("replay") || o.config.isMember("robot_input"))
      throw std::runtime_error(
          "replay/robot_input apply only to CSV or MCAP input");
  } else {
    o.input =
        execution::readJson(reference(o.config["robot_input"], "robot_input"));
    const auto &r = o.config["replay"];
    for (const auto &key : r.getMemberNames())
      if (key != "left_stream" && key != "right_stream" &&
          key != "timestamp_source" && key != "csv_mapping")
        throw std::runtime_error("unsupported replay key: " + key);
    data::parseTimestampSource(
        r.get("timestamp_source",
              format == "csv" ? "configured_column" : "header_stamp")
            .asString());
    if (r.isMember("csv_mapping")) {
      if (format != "csv")
        throw std::runtime_error("csv_mapping applies only to CSV input");
      reference(r["csv_mapping"], "csv_mapping");
    }
  }
  const auto model_path = o.input["model"]["locator"].asString();
  if (!std::filesystem::path(model_path).is_absolute() ||
      o.input["model"]["sha256"].asString() != sha256_file(model_path))
    throw std::runtime_error("model path or sha256 mismatch");
  Json::Value argv_json(Json::arrayValue);
  for (int i = 0; i < argc; ++i)
    argv_json.append(argv[i]);
  o.original_argv = argv_json;
  return o;
}
void prepareInput(Options &o) {
  const auto format = o.request.document["input"]["format"].asString();
  if (format == "json")
    return;
  if (format != "csv" && format != "mcap")
    throw std::runtime_error("unsupported input format");
  const auto robot_path = o.config["robot_input"]["path"].asString();
  if (o.config["robot_input"]["sha256"].asString() != sha256_file(robot_path))
    throw std::runtime_error("robot_input sha256 mismatch");
  o.input = execution::readJson(robot_path);
  replay::ReplayOptions r;
  r.input_path = o.request.document["input"]["path"].asString();
  r.input_format =
      format == "csv" ? replay::InputFormat::Csv : replay::InputFormat::Mcap;
  const auto &c = o.config["replay"];
  r.left_stream = c.get("left_stream", "left").asString();
  r.right_stream = c.get("right_stream", "right").asString();
  r.timestamp_source = data::parseTimestampSource(
      c.get("timestamp_source",
            format == "csv" ? "configured_column" : "header_stamp")
          .asString());
  r.target_period_ns = std::llround(o.config["dt_s"].asDouble() * 1e9);
  r.timestamp_projection.period_ns = r.target_period_ns;
  if (c.isMember("csv_mapping")) {
    const auto mapping = c["csv_mapping"]["path"].asString();
    if (c["csv_mapping"]["sha256"].asString() != sha256_file(mapping))
      throw std::runtime_error("csv_mapping sha256 mismatch");
    r.csv_mapping_path = mapping;
  }
  const auto loaded = replay::loadReplay(r);
  o.input["samples"] = Json::Value(Json::arrayValue);
  for (const auto &frame : loaded.timeline.timeline) {
    Json::Value sample;
    sample["source_time_s"] = frame.projected_time_ns * 1e-9;
    if (frame.value.left.frame_id != o.input["root_frame"].asString() ||
        frame.value.right.frame_id != o.input["root_frame"].asString())
      throw std::runtime_error("replay frame differs from configured robot "
                               "reference frame; no implicit transform");
    sample["targets"]["left"] = jsonPose(frame.value.left.pose);
    sample["targets"]["right"] = jsonPose(frame.value.right.pose);
    sample["original_logical_time_ns"] =
        Json::Int64(frame.original_logical_time_ns);
    sample["source_time_from_start_ns"] =
        Json::Int64(frame.source_time_from_start_ns);
    o.input["samples"].append(sample);
  }
  if (o.input["samples"].empty())
    throw std::runtime_error("empty replay");
}
std::vector<double> numbers(const Json::Value &v) {
  std::vector<double> r;
  for (auto &x : v)
    r.push_back(x.asDouble());
  return r;
}
std::vector<std::string> strings(const Json::Value &v) {
  std::vector<std::string> r;
  for (auto &x : v)
    r.push_back(x.asString());
  return r;
}
Json::Value array(const std::vector<double> &v) {
  Json::Value r(Json::arrayValue);
  for (double x : v)
    r.append(x);
  return r;
}
mcc::Pose pose(const Json::Value &v) {
  auto p = mcc::Pose::Identity();
  for (int i = 0; i < 3; i++) {
    p.translation()[i] = v["position"][i].asDouble();
    for (int j = 0; j < 3; j++)
      p.linear()(i, j) = v["rotation"][i][j].asDouble();
  }
  return p;
}
Json::Value jsonPose(const mcc::Pose &p) {
  Json::Value v;
  for (int i = 0; i < 3; i++) {
    v["position"].append(p.translation()[i]);
    for (int j = 0; j < 3; j++)
      v["rotation"][i].append(p.linear()(i, j));
  }
  return v;
}
void write(const std::filesystem::path &p, const Json::Value &v) {
  std::ofstream f(p);
  f.exceptions(std::ios::badbit | std::ios::failbit);
  f << v << '\n';
}
void require(const mcc::Status &s) {
  if (!s.ok())
    throw std::runtime_error(s.message);
}
} // namespace motion_control_lab::planned_kinematics_step
