#pragma once
#include "planning.hpp"
#include "solver.hpp"
#include <future>
#include <motion_control_viz/render_sink.hpp>
namespace motion_control_lab::joint_path_planning {
struct Attempt {
  unsigned id{0};
  double request_time_ms{0};
  ArmSide side{ArmSide::Left};
  Pose target{Pose::Identity()};
  NamedGoal ik;
  std::array<Pose, 2> ik_tcp{Pose::Identity(), Pose::Identity()};
  PlannedMotion motion;
  std::vector<std::array<double, 3>> direct_curve, path_curve, smooth_curve,
      smooth_stops;
};
struct Snapshot {
  Stage stage{Stage::Idle};
  bool execution_complete{false};
  bool whole_body{true};
  std::string timing_mode{"straight-through"};
  std::string smooth_validation{"full"};
  ArmSide side{ArmSide::Left};
  Pose draft{Pose::Identity()}, actual{Pose::Identity()};
  std::array<Pose, 2> draft_tcp{Pose::Identity(), Pose::Identity()};
  std::array<Pose, 2> execution_tcp{Pose::Identity(), Pose::Identity()};
  mcc::RobotState state;
  mcc::JointTrajectorySample sample;
  std::shared_ptr<const Attempt> attempt;
  std::vector<std::array<double, 3>> trail;
  std::vector<std::string> events;
  std::string message{"Edit target, Enter plans and executes"};
  double progress_s{0}, step_m{.01};
};
Attempt generateAttempt(unsigned, ArmSide, const Pose &,
                        const mcc::RobotState &, Solver &, Planning &,
                        std::atomic<bool> &, std::atomic<Stage> &);
Json::Value diagnosticsJson(const Snapshot &);
int run(const Options &, std::shared_ptr<const mcc::RobotModel>, Solver &,
        Planning &, motion_control::viz::RenderSink &);
} // namespace motion_control_lab::joint_path_planning
