#include "planning.hpp"
#include <algorithm>
namespace motion_control_lab::joint_path_planning {
const char *stageName(Stage s) {
  switch (s) {
  case Stage::Idle:
    return "READY";
  case Stage::Ik:
    return "IK";
  case Stage::Check:
    return "CHECK";
  case Stage::Simplify:
    return "SIMPLIFY";
  case Stage::Search:
    return "SEARCH";
  case Stage::Timing:
    return "TIMING";
  case Stage::Verification:
    return "VERIFY";
  case Stage::Preview:
    return "PREVIEW";
  case Stage::Executing:
    return "EXECUTING";
  case Stage::Paused:
    return "PAUSED";
  case Stage::Rejected:
    return "REJECTED";
  case Stage::Cancelled:
    return "CANCELLED";
  case Stage::Stopped:
    return "STOPPED";
  }
  return "UNKNOWN";
}
const char *terminationName(mcc::JointPathPlanningTermination t) {
  using T = mcc::JointPathPlanningTermination;
  switch (t) {
  case T::NotStarted:
    return "Not started";
  case T::ExactSolution:
    return "Exact solution";
  case T::InvalidStart:
    return "Invalid start configuration";
  case T::InvalidGoal:
    return "Invalid goal configuration";
  case T::BudgetExhausted:
    return "Planning budget exhausted";
  case T::Interrupted:
    return "Interrupted";
  case T::NoExactSolution:
    return "No exact solution found";
  case T::ValidationFailed:
    return "Final path validation failed";
  case T::BackendError:
    return "Backend error";
  }
  return "Unknown termination";
}
const char *
simplificationTerminationName(mcc::JointPathSimplificationTermination value) {
  using T = mcc::JointPathSimplificationTermination;
  switch (value) {
  case T::NotRun:
    return "not_run";
  case T::Disabled:
    return "disabled";
  case T::Completed:
    return "completed";
  case T::Stalled:
    return "stalled";
  case T::AttemptLimitReached:
    return "attempt_limit";
  case T::BudgetExhausted:
    return "budget_exhausted";
  case T::Interrupted:
    return "cancelled";
  case T::BackendError:
    return "backend_error";
  }
  return "unknown";
}
const char *validationName(mcc::JointPathValidationReason reason) {
  using R = mcc::JointPathValidationReason;
  switch (reason) {
  case R::NotChecked:
    return "not checked";
  case R::Valid:
    return "valid";
  case R::JointLimit:
    return "joint limit";
  case R::Collision:
    return "collision";
  case R::Clearance:
    return "clearance below 5 mm";
  case R::Interrupted:
    return "budget or cancellation";
  case R::PoseConstraint:
    return "held TCP tolerance exceeded";
  case R::ConstraintUncertified:
    return "held TCP segment bound not certified";
  }
  return "unknown";
}
mcc::RobotState initialState(const mcc::RobotModel &model, const Options &o) {
  mcc::RobotState state;
  const auto &r = r1RobotConfig();
  state.joint_positions.resize(model.jointNames().size());
  state.joint_velocities = Eigen::VectorXd::Zero(model.jointNames().size());
  for (std::size_t i = 0; i < model.jointNames().size(); ++i) {
    const auto &name = model.jointNames()[i];
    const auto it = std::find(r.joint_names.begin(), r.joint_names.end(), name);
    if (o.scene["initial_positions"].isMember(name))
      state.joint_positions[i] = o.scene["initial_positions"][name].asDouble();
    else if (it != r.joint_names.end())
      state.joint_positions[i] =
          r.default_positions[it - r.joint_names.begin()];
    else
      throw std::runtime_error("explicit initial position required: " + name);
  }
  return state;
}
Planning::Planning(std::shared_ptr<const mcc::RobotModel> model,
                   const Options &o)
    : options_(o) {
  mcc::PlanningSceneDescription d;
  d.robot_model = model;
  d.id = o.scene["id"].asString();
  d.revision = 1;
  mcc::RobotCollisionModelDescription collision_description;
  collision_description.mesh_search_paths = {
      std::filesystem::absolute(o.urdf).parent_path().string()};
  require(mcc::RobotCollisionModel::load(model, collision_description,
                                         d.collision_model, geometry));
  std::vector<mcc::CollisionObjectDescription> objects;
  for (const auto &v : o.scene["obstacles"]) {
    mcc::CollisionObjectDescription x;
    x.id = v["id"].asString();
    Eigen::Vector3d size;
    for (int j = 0; j < 3; ++j) {
      size[j] = v["size"][j].asDouble();
      x.world_pose.translation()[j] = v["xyz"][j].asDouble();
    }
    x.shape = mcc::CollisionBox{size};
    objects.push_back(x);
  }
  require(mcc::CollisionWorld::create(objects, d.world));
  for (const auto &v : o.scene["allowed_collisions"])
    d.allowed_collisions.push_back(
        {{mcc::CollisionBodyKind::RobotLink, v["links"][0].asString()},
         {mcc::CollisionBodyKind::RobotLink, v["links"][1].asString()}});
  require(mcc::PlanningScene::create(d, scene));
  require(planner_.configure(scene));
  require(validator_.configure(scene));
}
mcc::JointPathValidationResult Planning::check(const mcc::RobotState &state) {
  mcc::JointPathValidationResult r;
  require(validator_.checkState(state, {}, r));
  return r;
}
PlannedMotion Planning::generate(
    const mcc::RobotState &state, const mcc::JointNames &names,
    const Eigen::VectorXd &values,
    const std::optional<mcc::JointPathPoseConstraint> &constraint,
    std::atomic<bool> &cancel, std::atomic<Stage> &stage) {
  PlannedMotion m;
  m.start = state;
  m.goal = state;
  const auto &model = *scene->description().robot_model;
  for (std::size_t i = 0; i < names.size(); ++i)
    m.goal.joint_positions[std::find(model.jointNames().begin(),
                                     model.jointNames().end(), names[i]) -
                           model.jointNames().begin()] = values[i];
  mcc::JointPathValidationOptions validation;
  validation.should_stop = [&] { return cancel.load(); };
  mcc::JointPathPlanningRequest request;
  request.start_state = state;
  request.pose_constraint = constraint;
  request.active_joint_names = names;
  request.goal_positions = values;
  request.options.soft_time_budget = options_.budget;
  request.options.simplification_time_budget = options_.simplification_budget;
  request.options.random_seed = options_.seed;
  request.options.validation = validation;
  request.options.stage_changed = [&](mcc::JointPathPlanningStage value) {
    using S = mcc::JointPathPlanningStage;
    switch (value) {
    case S::CheckingEndpoints:
    case S::CheckingDirectMotion:
      stage = Stage::Check;
      break;
    case S::Searching:
      stage = Stage::Search;
      break;
    case S::Simplifying:
      stage = Stage::Simplify;
      break;
    case S::Validating:
      stage = Stage::Verification;
      break;
    }
  };
  require(planner_.plan(request, m.path, m.planning));
  m.direct = m.planning.direct_validation;
  m.verified = m.planning.last_validation;
  if (!mcc::isAccepted(m.path.disposition)) {
    m.reason = std::string("Path rejected: ") +
               terminationName(m.planning.termination) + " (" +
               validationName(m.planning.last_validation.reason) + ")";
    const auto &witness = m.planning.last_validation.violating_pair
                              ? m.planning.last_validation.violating_pair
                              : m.planning.last_validation.nearest_pair;
    if (witness) {
      const auto &pair = *witness;
      m.reason += ": " + pair.first.name + " / " + pair.second.name;
    }
    return m;
  }
  if (cancel) {
    m.reason = "Cancelled";
    return m;
  }
  stage = Stage::Timing;
  mcc::JointPathTimingOptions timing;
  timing.sample_period = options_.period;
  timing.waypoint_policy =
      options_.timing_mode == "straight-through"
          ? mcc::JointPathWaypointPolicy::ContinueAlongStraightSegments
          : mcc::JointPathWaypointPolicy::StopAtEveryWaypoint;
  timing.should_stop = [&] { return cancel.load(); };
  for (const auto &l : model.jointLimits()) {
    timing.limits.position_lower.push_back(l.lower);
    timing.limits.position_upper.push_back(l.upper);
    timing.limits.max_velocity.push_back(l.velocity);
    timing.limits.max_acceleration.push_back(options_.acceleration);
    timing.limits.max_jerk.push_back(options_.jerk);
  }
  require(mcc::JointPathTimeParameterizer{}.generate(m.path.path, timing,
                                                     m.timed, m.timing));
  if (!mcc::isAccepted(m.timed.disposition)) {
    m.reason = "Timing rejected";
    return m;
  }
  // Same immutable scene and accepted polyline; timing does not reshape it.
  // Reuse Core's denser final validation instead of repeating it on time
  // samples.
  m.accepted = m.verified.valid && !cancel;
  m.reason = m.accepted ? "Exact path accepted"
                        : (cancel ? "Cancelled" : "Final validation rejected");
  return m;
}
} // namespace motion_control_lab::joint_path_planning
