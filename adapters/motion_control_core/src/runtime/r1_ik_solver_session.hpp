#pragma once

#include "runtime/r1_robot_setup.hpp"

#include <string>

namespace motion_control_lab
{

class R1IkSolverSession final : public IkSolverBackend
{
public:
  explicit R1IkSolverSession(const std::string & urdf_path);

  std::string_view backendId() const override;

  const JointNames & jointNames() const override;

  const std::vector<double> & positions() const override;

  const std::vector<double> & velocities() const override;

  void reset(const JointState & state) override;

  Pose currentTargetPose(ArmSide side) const override;

  std::vector<ArmTarget> currentTargetPoses(
    const std::vector<ArmSide> & sides) const override;

  IkSolveResult solveTargets(
    const std::vector<ArmTarget> & targets,
    double dt) override;

private:
  JointNames joint_names_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  mcc::KinematicsSolver solver_;
};

}  // namespace motion_control_lab
