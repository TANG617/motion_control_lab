#include "planning.hpp"
namespace study_e12 {
Planning::Planning(const Options &x) : o(x) {
  mcc::JointPlannerConfig c;
  c.synchronization = mcc::TrajectorySynchronization::Time;
  require(jp.configure(c));
  for (auto s : {"left", "right"}) {
    mcc::CartesianFrameSample f;
    f.frame_name = o.input["frames"][s].asString();
    f.reference_frame_name = o.input["root_frame"].asString();
    f.pose = pose(o.input["samples"][0]["targets"][s]);
    last.frames.push_back(f);
  }
}
Json::Value Planning::reference(const Json::Value &goal, bool update,
                                Json::Value &native) {
  mcc::PlanningDiagnostics d;
  if (update) {
    mcc::CartesianRetargetRequest r;
    r.reference_frame_name = o.input["root_frame"].asString();
    r.sample_period = o.config["dt_s"].asDouble();
    r.synchronization = mcc::TrajectorySynchronization::Time;
    auto &l = r.limits;
    l.max_linear_velocity =
        Eigen::Vector3d::Constant(o.config["cartesian_velocity"].asDouble());
    l.max_linear_acceleration = Eigen::Vector3d::Constant(
        o.config["cartesian_acceleration"].asDouble());
    l.max_linear_jerk =
        Eigen::Vector3d::Constant(o.config["cartesian_jerk"].asDouble());
    l.max_rotation_vector_velocity = Eigen::Vector3d::Constant(1.);
    l.max_rotation_vector_acceleration = Eigen::Vector3d::Constant(5.);
    l.max_rotation_vector_jerk = Eigen::Vector3d::Constant(50.);
    int i = 0;
    for (auto s : {"left", "right"}) {
      mcc::CartesianRetargetSegment seg;
      seg.frame_name = last.frames[i].frame_name;
      seg.current_pose = last.frames[i].pose;
      seg.current_twist = last.frames[i].twist;
      seg.current_acceleration = last.frames[i].acceleration;
      seg.target_pose = pose(goal[s]);
      r.segments.push_back(seg);
      i++;
    }
    auto status = cp.replan(r, d);
    native["cartesian_replan_status"] = status.message;
    native["cartesian_replan_ok"] = status.ok();
    if (!status.ok())
      return Json::Value();
  }
  auto status = cp.step(last, d);
  native["cartesian_step_status"] = status.message;
  native["cartesian_step_ok"] = status.ok();
  if (!status.ok())
    return Json::Value();
  Json::Value result;
  int i = 0;
  for (auto s : {"left", "right"}) {
    result[s] = jsonPose(last.frames[i].pose);
    result[s]["twist"] =
        array({last.frames[i].twist.data(), last.frames[i].twist.data() + 6});
    result[s]["acceleration"] = array({last.frames[i].acceleration.data(),
                                       last.frames[i].acceleration.data() + 6});
    i++;
  }
  return result;
}
mcc::JointTrajectorySample Planning::execute(const std::vector<double> &q,
                                             const std::vector<double> &v,
                                             const std::vector<double> &a,
                                             const std::vector<double> &target,
                                             Json::Value &native) {
  mcc::JointTrajectoryRequest r;
  r.joint_names = strings(o.input["joint_names"]);
  r.sample_period = o.config["dt_s"].asDouble();
  r.current = {q, v, a};
  r.target.positions = target;
  r.target.velocities = std::vector<double>(q.size(), 0);
  r.target.accelerations = std::vector<double>(q.size(), 0);
  r.limits.position_lower = numbers(o.input["limits"]["lower"]);
  r.limits.position_upper = numbers(o.input["limits"]["upper"]);
  r.limits.max_velocity = numbers(o.input["limits"]["velocity"]);
  r.limits.max_acceleration =
      std::vector<double>(q.size(), o.config["joint_acceleration"].asDouble());
  r.limits.max_jerk =
      std::vector<double>(q.size(), o.config["joint_jerk"].asDouble());
  mcc::PlanningDiagnostics d;
  auto status = jp.plan(r, d);
  native["joint_plan_status"] = status.message;
  native["joint_plan_ok"] = status.ok();
  mcc::JointTrajectorySample s;
  if (!status.ok())
    return s;
  status = jp.step(s, d);
  native["joint_step_status"] = status.message;
  native["joint_step_ok"] = status.ok();
  return s;
}
} // namespace study_e12
