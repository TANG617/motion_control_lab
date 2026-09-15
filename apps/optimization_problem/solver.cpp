#include "solver.hpp"
#include <chrono>
#include <map>
#include <motion_control_core/optimization/hierarchical_problem.hpp>
#include <motion_control_core/optimization/problem.hpp>
#include <placo/problem/problem.h>
#include <stdexcept>
namespace motion_control_lab::optimization_problem {
namespace mcc = motion_control::core;
namespace {
void ok(const mcc::Status &s) {
  if (!s.ok())
    throw std::runtime_error(s.message);
}
Eigen::VectorXd vector(const Json::Value &v) {
  Eigen::VectorXd r(v.size());
  for (unsigned i = 0; i < v.size(); ++i)
    r[i] = v[i].asDouble();
  return r;
}
Eigen::MatrixXd matrix(const Json::Value &v) {
  Eigen::MatrixXd r(v.size(), v[0].size());
  for (unsigned i = 0; i < v.size(); ++i)
    for (unsigned j = 0; j < v[i].size(); ++j)
      r(i, j) = v[i][j].asDouble();
  return r;
}
Json::Value json(const Eigen::VectorXd &v) {
  Json::Value r(Json::arrayValue);
  for (auto x : v)
    r.append(x);
  return r;
}
} // namespace
Json::Value solve(const Json::Value &input, const Json::Value &o) {
  const auto &a = input.isMember("analytic") ? input["analytic"] : input;
  const int size = a["lower"].size();
  Json::Value out;
  out["problem"] = a;
  out["record_type"] = "result";
  if (o["solver"] == "placo") {
    placo::problem::Problem p;
    p.regularization = o["regularization"].asDouble();
    p.use_sparsity = false;
    p.rewrite_equalities = true;
    auto &x = p.add_variable(size);
    placo::problem::Expression e;
    e.A = Eigen::MatrixXd::Identity(size, size);
    e.b = -vector(a["lower"]);
    p.add_constraint(e >= 0);
    e.A = -Eigen::MatrixXd::Identity(size, size);
    e.b = vector(a["upper"]);
    p.add_constraint(e >= 0);
    for (const auto &t : a["tasks"]) {
      if (!t.get("enabled", true).asBool())
        continue;
      e.A = matrix(t["A"]);
      e.b = -vector(t["b"]);
      p.add_constraint(e == 0).configure(t.get("hard", false).asBool() ? "hard"
                                                                       : "soft",
                                         t.get("weight", 1.).asDouble());
    }
    const auto start = std::chrono::steady_clock::now();
    p.solve();
    out["native_call_ms"] = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - start)
                                .count();
    out["candidate"] = json(x.value);
    out["accepted"] = true;
    out["native_status"] = "PlaCo solve returned";
    out["native_status_code"] = Json::nullValue;
    out["native_status_reason"] = "throw-on-failure API";
    out["candidate_available"] = true;
    out["committed"] = false;
    out["commit_reason"] = "pure optimization has no robot state";
    return out;
  }
  mcc::QpSolverConfig c;
  c.regularization = o["regularization"].asDouble();
  c.backend = o["backend"] == "proxqp" ? mcc::QpBackend::ProxQp
                                       : mcc::QpBackend::Eiquadprog;
  bool hierarchy = o["mode"] == "hqp";
  mcc::OptimizationProblem w;
  mcc::HierarchicalOptimizationProblem h;
  mcc::VariableBlockHandle x;
  if (hierarchy) {
    ok(h.configure(c));
    ok(h.addVariableBlock("x", size, x));
  } else {
    ok(w.configure(c));
    ok(w.addVariableBlock("x", size, x));
  }
  std::map<int, mcc::HierarchyLevelHandle> levels;
  if (hierarchy) {
    int index = 0;
    for (const auto &t : a["tasks"]) {
      int priority = t.get("priority", index++).asInt();
      levels.emplace(priority, mcc::HierarchyLevelHandle{});
    }
    for (auto &[priority, level] : levels)
      ok(h.addLevel({"priority-" + std::to_string(priority)}, level));
  }
  mcc::VariableBoundsRequirement bounds;
  bounds.name = "box";
  bounds.block = x;
  bounds.bounds.lower = vector(a["lower"]);
  bounds.bounds.upper = vector(a["upper"]);
  mcc::RequirementHandle bh;
  if (hierarchy)
    ok(h.addSharedVariableBounds(bounds, bh));
  else
    ok(w.addVariableBounds(bounds, bh));
  int index = 0;
  for (const auto &t : a["tasks"]) {
    mcc::LinearRequirement r;
    r.name = t.get("name", "task-" + std::to_string(index)).asString();
    r.unit = t.get("unit", "unit").asString();
    r.A = matrix(t["A"]);
    r.offset = Eigen::VectorXd::Zero(r.A.rows());
    r.bounds.lower = vector(t["b"]);
    r.bounds.upper = r.bounds.lower;
    r.enabled = t.get("enabled", true).asBool();
    if (t.get("hard", false).asBool())
      r.enforcement = mcc::HardEnforcement{};
    else
      r.enforcement = mcc::SoftEnforcement{mcc::QuadraticPenalty{
          t.get("weight", 1.).asDouble(), Eigen::VectorXd::Ones(r.A.rows())}};
    mcc::RequirementHandle rh;
    if (hierarchy)
      ok(h.addLevelRequirement(
          levels.at(t.get("priority", index).asInt()), r,
          {Eigen::VectorXd::Constant(r.A.rows(),
                                     o["preservation_tolerance"].asDouble())},
          rh));
    else
      ok(w.addRequirement(r, rh));
    ++index;
  }
  if (hierarchy) {
    ok(h.finalize());
    mcc::HierarchicalOptimizationSolution s;
    mcc::HierarchicalOptimizationDiagnostics d;
    const auto start = std::chrono::steady_clock::now();
    auto st = h.solve(s, d);
    out["native_call_ms"] = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - start)
                                .count();
    out["native_status"] = st.message;
    out["native_status_code"] = int(st.code);
    out["accepted"] = st.ok();
    out["candidate"] = json(s.values);
    out["selected_pass"] = Json::nullValue;
    int i = 0;
    for (const auto &l : d.levels) {
      Json::Value pass;
      pass["name"] = l.name;
      pass["native_status"] = l.optimization.native_status;
      pass["iterate"] = json(l.last_iterate);
      pass["succeeded"] = l.succeeded;
      out["passes"].append(pass);
      if (st.ok() && l.succeeded)
        out["selected_pass"] = i;
      ++i;
    }
  } else {
    ok(w.finalize());
    mcc::OptimizationSolution s;
    mcc::OptimizationDiagnostics d;
    const auto start = std::chrono::steady_clock::now();
    auto st = w.solve(s, d);
    out["native_call_ms"] = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - start)
                                .count();
    out["native_status"] = st.message;
    out["native_status_code"] = int(st.code);
    out["accepted"] = st.ok();
    out["candidate"] = json(s.values);
    out["native_qp_status"] = d.native_status;
  }
  out["candidate_available"] = !out["candidate"].empty();
  out["committed"] = false;
  out["commit_reason"] = "pure optimization has no robot state";
  return out;
}
} // namespace motion_control_lab::optimization_problem
