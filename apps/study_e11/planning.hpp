#pragma once
#include "options.hpp"
namespace study_e11 {
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
} // namespace study_e11
