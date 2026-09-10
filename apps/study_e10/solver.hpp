#pragma once
#include "options.hpp"
#include <memory>
#include <motion_control_core/motion_control_core.hpp>
#include <vector>
namespace study_e10 {
namespace mcc = motion_control::core;
struct Result {
  std::vector<double> q, v;
  mcc::Status status;
  mcc::InverseKinematicsSolution solution;
  mcc::InverseKinematicsDiagnostics weighted;
  mcc::HierarchicalInverseKinematicsDiagnostics hierarchy;
  bool accepted = false, placo = false, hierarchical = false;
  int iterations = 0;
  std::string termination_reason;
  bool converged = false;
};
class Solver {
public:
  explicit Solver(const Options &options);
  ~Solver();
  Result solve(const Json::Value &sample,
               const Json::Value &proposal = Json::Value());
  Json::Value evidence(const Result &) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl;
};
} // namespace study_e10
