#include "options.hpp"
#include <set>
#include <stdexcept>
namespace motion_control_lab::optimization_problem {
Json::Value resolveOptions(const Json::Value &config) {
  const std::set<std::string> keys{"solver", "mode", "backend",
                                   "regularization", "preservation_tolerance"};
  for (const auto &key : config.getMemberNames())
    if (!keys.count(key))
      throw std::runtime_error("unsupported option: " + key);
  Json::Value o = config;
  o["solver"] = config.get("solver", "mcc");
  o["mode"] = config.get("mode", "hqp");
  o["backend"] = config.get("backend", "eiquadprog");
  o["regularization"] = config.get("regularization", 1e-8);
  o["preservation_tolerance"] = config.get("preservation_tolerance", 1e-7);
  if (o["solver"] != "mcc" && o["solver"] != "placo")
    throw std::runtime_error("unsupported solver");
  if (o["mode"] != "weighted" && o["mode"] != "hqp")
    throw std::runtime_error("unsupported mode");
  if (o["backend"] != "eiquadprog" && o["backend"] != "proxqp")
    throw std::runtime_error("unsupported backend");
  if (o["solver"] == "placo" &&
      (o["mode"] != "weighted" || o["backend"] != "eiquadprog"))
    throw std::runtime_error(
        "PlaCo explicit problem supports weighted eiquadprog only");
  return o;
}
Json::Value capabilities() {
  Json::Value c;
  c["schema_version"] = "app_capabilities.v1";
  c["app_id"] = "mcl_optimization_problem";
  c["app_version"] = "1";
  c["execution_structures"].append("explicit_matrix");
  c["input_formats"].append("json");
  c["modes"].append("weighted");
  c["modes"].append("hqp");
  c["solvers"]["mcc"]["backends"].append("eiquadprog");
  c["solvers"]["mcc"]["backends"].append("proxqp");
  c["solvers"]["placo"]["backends"].append("eiquadprog");
  c["solvers"]["placo"]["modes"].append("weighted");
  return c;
}
} // namespace motion_control_lab::optimization_problem
