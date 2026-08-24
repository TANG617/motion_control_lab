#pragma once

#include <memory>
#include <string>
#include <vector>

#include "components/robot/r1/r1_robot_config.hpp"
#include "contracts/presentation/ik_app_snapshot.hpp"
#include "options.hpp"

namespace motion_control_lab::step {

struct ServoSolveResult {
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<ArmForwardKinematics> forward_kinematics;
  std::vector<ArmTargetError> target_errors;
  SolverDebug solver_debug;
  int iterations{0};
  bool converged{false};
  double solve_time_ms{0.0};
};

const char *mccSolverTitle(MccBackend backend);

class MccServoSolver {
public:
  MccServoSolver(const std::string &urdf_path, double rate_hz,
                 const R1RobotConfig &robot, MccBackend backend,
                 const AlgorithmOptions &algorithm,
                 const std::vector<double> &initial_positions = {},
                 const std::vector<double> &initial_velocities = {});
  ~MccServoSolver();

  const std::vector<double> &positions() const;
  const std::vector<double> &velocities() const;
  Pose currentPose(ArmSide side);
  ServoSolveResult solve(const std::vector<ArmTarget> &targets);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

class PlacoServoSolver {
public:
  PlacoServoSolver(const std::string &urdf_path, double rate_hz,
                   const R1RobotConfig &robot,
                   const AlgorithmOptions &algorithm,
                   const std::vector<double> &initial_positions = {},
                   const std::vector<double> &initial_velocities = {});
  ~PlacoServoSolver();

  const std::vector<double> &positions() const;
  const std::vector<double> &velocities() const;
  Pose currentPose(ArmSide side);
  ServoSolveResult solve(const std::vector<ArmTarget> &targets);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace motion_control_lab::step
