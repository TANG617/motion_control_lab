#include <motion_control_lab/ik_solver_backend.hpp>

#include <cstdlib>
#include <string_view>
#include <vector>

namespace
{

class FakeIkSolverBackend final : public motion_control_lab::IkSolverBackend
{
public:
  std::string_view backendId() const override
  {
    return "contract-test";
  }

  const motion_control_lab::JointNames & jointNames() const override
  {
    return names_;
  }

  const std::vector<double> & positions() const override
  {
    return positions_;
  }

  const std::vector<double> & velocities() const override
  {
    return velocities_;
  }

  void reset(const motion_control_lab::JointState & state) override
  {
    names_ = state.names;
    positions_ = state.positions;
    velocities_ = state.velocities;
  }

  motion_control_lab::Pose currentTargetPose(motion_control_lab::ArmSide) const override
  {
    return motion_control_lab::Pose::Identity();
  }

  std::vector<motion_control_lab::ArmTarget> currentTargetPoses(
    const std::vector<motion_control_lab::ArmSide> & sides) const override
  {
    std::vector<motion_control_lab::ArmTarget> targets;
    targets.reserve(sides.size());
    for (const auto side : sides) {
      targets.push_back({side, currentTargetPose(side)});
    }
    return targets;
  }

  motion_control_lab::IkSolveResult solveTargets(
    const std::vector<motion_control_lab::ArmTarget> & targets,
    double) override
  {
    motion_control_lab::IkSolveResult result;
    result.joint_state = {names_, positions_, velocities_};
    result.solved_targets = targets;
    result.diagnostics.converged = true;
    return result;
  }

private:
  motion_control_lab::JointNames names_{"joint"};
  std::vector<double> positions_{0.0};
  std::vector<double> velocities_{0.0};
};

}  // namespace

int main()
{
  FakeIkSolverBackend backend;
  backend.reset({{"joint"}, {0.25}, {0.0}});
  const auto targets = backend.currentTargetPoses({motion_control_lab::ArmSide::Left});
  const auto result = backend.solveTargets(targets, 0.01);
  if (backend.backendId() != "contract-test" ||
      !result.status.ok() ||
      !result.diagnostics.converged ||
      result.joint_state.positions != std::vector<double>{0.25} ||
      result.solved_targets.size() != 1) {
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
