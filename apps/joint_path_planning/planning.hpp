#pragma once
#include "motion_control_core/planning/joint/path_planner.hpp"
#include "motion_control_core/planning/joint/time_parameterization.hpp"
#include "options.hpp"
#include <atomic>
namespace motion_control_lab::joint_path_planning {
enum class Stage {
  Idle,
  Ik,
  Search,
  Check,
  Simplify,
  Timing,
  Verification,
  Preview,
  Executing,
  Paused,
  Rejected,
  Cancelled,
  Stopped
};
const char *stageName(Stage);
const char *terminationName(mcc::JointPathPlanningTermination);
const char *
    simplificationTerminationName(mcc::JointPathSimplificationTermination);
const char *validationName(mcc::JointPathValidationReason);
struct PlannedMotion {
  bool accepted{false};
  std::string reason;
  mcc::RobotState start, goal;
  mcc::JointPathPlanningResult path;
  mcc::JointPathPlanningDiagnostics planning;
  mcc::JointPathTimingResult timed;
  mcc::JointPathTimingDiagnostics timing;
  mcc::JointPathValidationResult direct, verified;
};
class Planning {
public:
  Planning(std::shared_ptr<const mcc::RobotModel>, const Options &);
  PlannedMotion generate(const mcc::RobotState &, const mcc::JointNames &,
                         const Eigen::VectorXd &,
                         const std::optional<mcc::JointPathPoseConstraint> &,
                         std::atomic<bool> &, std::atomic<Stage> &);
  mcc::JointPathValidationResult check(const mcc::RobotState &);
  std::shared_ptr<const mcc::PlanningScene> scene;
  mcc::CollisionModelDiagnostics geometry;

private:
  Options options_;
  mcc::JointPathPlanner planner_;
  mcc::JointPathValidator validator_;
};
mcc::RobotState initialState(const mcc::RobotModel &, const Options &);
} // namespace motion_control_lab::joint_path_planning
