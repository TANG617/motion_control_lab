#pragma once
#include "options.hpp"
#include "placo/kinematics/kinematics_solver.h"
#include "placo/model/robot_wrapper.h"
namespace motion_control_lab::planned_kinematics_step {
struct Result {
  std::vector<double> q, v;
  Json::Value native;
  bool accepted = false;
};
class Solver {
  Options o;
  bool placo;
  std::vector<std::string> names, active;
  std::shared_ptr<const mcc::RobotModel> model;
  mcc::KinematicsSolver weighted;
  mcc::PositionTaskHandle ph[2];
  mcc::OrientationTaskHandle oh[2];
  mcc::PostureTaskHandle posture;
  std::unique_ptr<placo::model::RobotWrapper> robot;
  std::unique_ptr<placo::kinematics::KinematicsSolver> ps;
  placo::kinematics::PositionTask *pp[2];
  placo::kinematics::OrientationTask *po[2];
  placo::kinematics::JointsTask *pposture;

public:
  explicit Solver(const Options &);
  Result solve(const std::vector<double> &q, const std::vector<double> &v,
               const Json::Value &targets);
};
} // namespace motion_control_lab::planned_kinematics_step
