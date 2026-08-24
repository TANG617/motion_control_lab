#pragma once

#include <memory>
#include <string>
#include <vector>

#include "components/robot/r1/r1_robot_config.hpp"
#include "contracts/presentation/ik_app_snapshot.hpp"
#include "options.hpp"

namespace motion_control_lab::target {

struct TargetSolveResult {
  bool accepted{false};
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<ArmForwardKinematics> forward_kinematics;
  std::vector<ArmTargetError> target_errors;
  SolverDebug solver_debug;
  std::string ik_status;
  std::string status;
  int iterations{0};
  bool converged{false};
  double solve_time_ms{0.0};
};

const char *mccSolverTitle(MccBackend backend);

class MccTargetSolver {
public:
  MccTargetSolver(const std::string &urdf_path, const R1RobotConfig &robot,
                  MccBackend backend, const AlgorithmOptions &algorithm);
  ~MccTargetSolver();

  const std::vector<double> &positions() const;
  const std::vector<double> &velocities() const;
  Pose currentPose(ArmSide side);
  TargetSolveResult solve(const std::vector<ArmTarget> &targets);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

class PlacoTargetSolver {
public:
  PlacoTargetSolver(const std::string &urdf_path, const R1RobotConfig &robot,
                    const AlgorithmOptions &algorithm);
  ~PlacoTargetSolver();

  const std::vector<double> &positions() const;
  const std::vector<double> &velocities() const;
  Pose currentPose(ArmSide side);
  TargetSolveResult solve(const std::vector<ArmTarget> &targets);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace motion_control_lab::target
