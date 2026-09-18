#include "options.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
namespace motion_control_lab::joint_path_planning {
Json::Value readJson(const std::filesystem::path &path) {
  std::ifstream in(path);
  in.exceptions(std::ios::badbit);
  Json::Value value;
  in >> value;
  return value;
}
void writeJson(const std::filesystem::path &path, const Json::Value &value) {
  std::ofstream out;
  out.exceptions(std::ios::badbit | std::ios::failbit);
  out.open(path);
  out << value << '\n';
}
void require(const mcc::Status &status) {
  if (!status.ok())
    throw std::runtime_error(status.message);
}
Options parseOptions(int argc, char **argv) {
  Options o;
  const auto share = std::filesystem::path(MCL_JOINT_PATH_SHARE);
  o.config = share / "apps/joint_path_planning/configs/cube.json";
  o.urdf = "/workspace/models/Psi_R1_visual_collision.urdf";
  o.output = "build/joint_path_planning/run";
  for (int i = 0; i < argc; ++i)
    o.argv.append(argv[i]);
  Json::Value request;
  for (int i = 1; i < argc; ++i)
    if (std::string(argv[i]) == "--request") {
      if (++i >= argc)
        throw std::runtime_error("missing value for --request");
      request = readJson(argv[i]);
      if (request["schema_version"] != "joint_path_planning.request.v1")
        throw std::runtime_error("unknown request schema");
      o.config = request["config"].asString();
      o.urdf = request["urdf"].asString();
      o.output = request["output"].asString();
      o.side = request.get("side", "left").asString();
      o.planning_mode = request.get("planning_mode", "whole-body").asString();
      o.timing_mode = request.get("timing_mode", o.timing_mode).asString();
      o.smooth_validation = request.get("smooth_validation", o.smooth_validation).asString();
      o.smoothing_budget =
          request.get("smoothing_budget", o.smoothing_budget).asDouble();
      o.smoothing_revolute_deviation = request
                                           .get("smoothing_revolute_deviation",
                                                o.smoothing_revolute_deviation)
                                           .asDouble();
      o.headless = true;
      o.viz = request.get("viz", false).asBool();
      o.record = request.get("record", true).asBool();
      o.goal = request["goal"];
      o.realtime = request.get("realtime", false).asBool();
      o.port = request.get("port", 8765).asInt();
      o.host = request.get("host", o.host).asString();
      o.budget = request.get("budget_s", o.budget).asDouble();
      o.simplification_budget =
          request.get("simplification_budget_s", o.simplification_budget)
              .asDouble();
      o.seed = request.get("seed", o.seed).asUInt();
    }
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--request") {
      ++i;
      continue;
    }
    if (a == "--dump-resolved-options") {
      o.dump = true;
      continue;
    }
    if (!request.isNull())
      throw std::runtime_error(
          "request mode accepts only --dump-resolved-options");
    const auto value = [&]() -> std::string {
      if (++i >= argc)
        throw std::runtime_error("missing value for " + a);
      return argv[i];
    };
    if (a == "--help")
      o.help = true;
    else if (a == "--describe-capabilities")
      o.describe = true;
    else if (a == "--config")
      o.config = value();
    else if (a == "--urdf")
      o.urdf = value();
    else if (a == "--output")
      o.output = value();
    else if (a == "--planning-mode")
      o.planning_mode = value();
    else if (a == "--timing-mode")
      o.timing_mode = value();
    else if (a == "--smooth-validation")
      o.smooth_validation = value();
    else if (a == "--side")
      o.side = value();
    else if (a == "--port")
      o.port = std::stoi(value());
    else if (a == "--host")
      o.host = value();
    else if (a == "--budget")
      o.budget = std::stod(value());
    else if (a == "--smoothing-budget")
      o.smoothing_budget = std::stod(value());
    else if (a == "--smoothing-revolute-deviation")
      o.smoothing_revolute_deviation = std::stod(value());
    else if (a == "--simplification-budget")
      o.simplification_budget = std::stod(value());
    else if (a == "--seed")
      o.seed = std::stoul(value());
    else if (a == "--goal") {
      Json::CharReaderBuilder b;
      std::istringstream in(value());
      std::string e;
      if (!Json::parseFromStream(b, in, &o.goal, &e))
        throw std::runtime_error(e);
    } else if (a == "--headless")
      o.headless = true;
    else if (a == "--realtime")
      o.realtime = true;
    else if (a == "--no-viz")
      o.viz = false;
    else if (a == "--no-record")
      o.record = false;
    else if (a == "--launcher")
      o.launcher = value();
    else
      throw std::runtime_error("unknown option: " + a);
  }
  if (o.help || o.describe)
    return o;
  if (o.planning_mode != "whole-body" && o.planning_mode != "single-arm")
    throw std::runtime_error("planning mode must be whole-body or single-arm");
  if (!std::isfinite(o.simplification_budget) || o.simplification_budget < 0)
    throw std::runtime_error(
        "simplification budget must be finite and non-negative");
  if (!std::isfinite(o.smoothing_budget) || o.smoothing_budget <= 0 ||
      !std::isfinite(o.smoothing_revolute_deviation) ||
      o.smoothing_revolute_deviation <= 0)
    throw std::runtime_error(
        "smoothing budget and deviation must be finite and positive");
  if (o.timing_mode != "straight-through" &&
      o.timing_mode != "stop-at-waypoints" && o.timing_mode != "smooth")
    throw std::runtime_error(
        "timing mode must be straight-through, stop-at-waypoints or smooth");
  if (o.smooth_validation != "full" && o.smooth_validation != "none")
    throw std::runtime_error("smooth validation must be full or none");
  o.scene = readJson(o.config);
  if (o.goal.isNull())
    o.goal = o.scene["goals"][o.side];
  return o;
}
Json::Value resolved(const Options &o) {
  Json::Value r;
  r["app"] = "mcl_joint_path_planning";
  r["planning_mode"] = o.planning_mode;
  r["timing_mode"] = o.timing_mode;
  r["smooth_validation"] = o.smooth_validation;
  r["path_policy"] = "adaptive-interval-shortcut-prune-v1";
  r["trajectory_policy"] = o.timing_mode == "smooth"
                               ? (o.smooth_validation == "full"
                                      ? "totg-ruckig-validated-curve-v1"
                                      : "totg-ruckig-generated-curve-v1")
                               : "exact-polyline-scalar-ruckig-v1";
  r["smoothing_budget"] = o.smoothing_budget;
  r["smoothing_revolute_deviation"] = o.smoothing_revolute_deviation;
  r["method_id"] =
      o.planning_mode == "whole-body" ? "whole-body-held-tcp-v1" : kIkPolicy;
  r["ik_policy"] = r["method_id"];
  r["active_dof"] = o.planning_mode == "whole-body" ? 16 : 7;
  if (o.planning_mode == "whole-body") {
    r["hold_position_tolerance_m"] = 0.001;
    r["hold_orientation_tolerance_rad"] = 0.01;
    r["hold_reference"] = "base_link/request-start";
  }
  r["ik_progress_weight"] = kIkProgressWeight;
  r["ik_posture_weight"] = kIkPostureWeight;
  r["ik_posture_reference"] = "fixed-initial-state";
  r["config"] = o.config.string();
  r["urdf"] = o.urdf.string();
  r["output"] = o.output.string();
  r["side"] = o.side;
  r["headless"] = o.headless;
  r["realtime"] = o.realtime;
  r["viz"] = o.viz;
  r["record"] = o.record;
  r["host"] = o.host;
  r["port"] = o.port;
  r["budget_s"] = o.budget;
  r["simplification_budget_s"] = o.simplification_budget;
  r["seed"] = o.seed;
  r["period_s"] = o.period;
  r["acceleration"] = o.acceleration;
  r["jerk"] = o.jerk;
  r["scene"] = o.scene;
  r["goal"] = o.goal;
  r["argv"] = o.argv;
  r["launcher"] = o.launcher;
  return r;
}
} // namespace motion_control_lab::joint_path_planning
