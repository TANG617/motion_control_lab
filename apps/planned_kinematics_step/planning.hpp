#pragma once
#include "options.hpp"
namespace motion_control_lab::planned_kinematics_step {
class Planning {
  Options o;
  mcc::CartesianPlanner cp;
  mcc::JointPlanner jp;
  mcc::CartesianTrajectorySample last;

public:
  explicit Planning(const Options &);
  Json::Value reference(const Json::Value &goal, bool update,
                        Json::Value &native);
  mcc::JointTrajectorySample execute(const std::vector<double> &q,
                                     const std::vector<double> &v,
                                     const std::vector<double> &a,
                                     const std::vector<double> &target,
                                     Json::Value &native);
};
} // namespace motion_control_lab::planned_kinematics_step
