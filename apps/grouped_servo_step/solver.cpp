#include "solver.hpp"

#include <algorithm>
#include <array>
#include <filesystem>
#include <stdexcept>
#include <string_view>

#include "components/app_helpers/app_helpers.hpp"

namespace motion_control_lab::grouped_servo_step {
namespace {

constexpr double kDefaultPostureJointWeightMultiplier = 1.0e-3;
constexpr double kArmJoint4PostureWeightMultiplier = 1.0e-1;
constexpr std::array<std::string_view, 4> kWaistJointNames{
    "torso_yaw_joint", "torso_pitch_joint", "knee_pitch_joint",
    "ankle_pitch_joint"};

bool isWaistJoint(const std::string &joint_name) {
  return std::find(kWaistJointNames.begin(), kWaistJointNames.end(),
                   joint_name) != kWaistJointNames.end();
}

std::filesystem::path
collisionMeshSearchRoot(const std::filesystem::path &urdf_path) {
  const auto canonical_urdf = std::filesystem::weakly_canonical(urdf_path);
  return canonical_urdf.parent_path().parent_path().parent_path();
}

mcc::SelfCollisionModelDescription
collisionModelDescription(const std::filesystem::path &urdf_path) {
  mcc::SelfCollisionModelDescription description;
  description.link_pairs = {{"left_arm_link4", "body_link4"},
                            {"right_arm_link4", "body_link4"},
                            {"left_arm_link7", "right_arm_link4"},
                            {"right_arm_link7", "left_arm_link4"}};
  description.mesh_search_paths = {collisionMeshSearchRoot(urdf_path).string()};
  return description;
}

CartesianHandles addCartesianTasks(mcc::KinematicsSolverBuilder &builder,
                                   mcc::SolverGroup group,
                                   const std::string &prefix,
                                   const R1RobotConfig &robot,
                                   const SolverOptions &options) {
  CartesianHandles handles;

  mcc::TaskScaleGroupConfig scale;
  scale.progress_weight = options.cartesian_progress_weight;
  scale.name = prefix + "-left-cartesian-progress";
  requireOk(builder.addTaskScaleGroup(group, scale, handles.left_scale),
            "register " + scale.name);
  scale.name = prefix + "-right-cartesian-progress";
  requireOk(builder.addTaskScaleGroup(group, scale, handles.right_scale),
            "register " + scale.name);

  mcc::GroupedScaledTaskConfig position;
  position.enforcement.feasibility_tolerance =
      options.maximum_accepted_hard_violation;
  position.scale_group = handles.left_scale;
  position.name = prefix + "-left-position";
  requireOk(builder.addScaledPositionTask(group, robot.left_end_effector_frame,
                                          position, handles.left_position),
            "register " + position.name);
  position.scale_group = handles.right_scale;
  position.name = prefix + "-right-position";
  requireOk(builder.addScaledPositionTask(group, robot.right_end_effector_frame,
                                          position, handles.right_position),
            "register " + position.name);

  mcc::GroupedScaledTaskConfig orientation;
  orientation.enforcement.feasibility_tolerance =
      options.maximum_accepted_hard_violation;
  orientation.scale_group = handles.left_scale;
  orientation.name = prefix + "-left-orientation";
  requireOk(
      builder.addScaledOrientationTask(group, robot.left_end_effector_frame,
                                       orientation, handles.left_orientation),
      "register " + orientation.name);
  orientation.scale_group = handles.right_scale;
  orientation.name = prefix + "-right-orientation";
  requireOk(
      builder.addScaledOrientationTask(group, robot.right_end_effector_frame,
                                       orientation, handles.right_orientation),
      "register " + orientation.name);
  return handles;
}

void addPositionLimits(mcc::KinematicsSolverBuilder &builder,
                       mcc::SolverGroup group, const SolverOptions &options) {
  mcc::JointPositionLimitConfig position;
  position.margin = options.joint_position_margin_rad;
  position.enforcement =
      mcc::HardEnforcement{options.maximum_accepted_hard_violation};
  mcc::GroupedJointPositionLimitHandle handle;
  requireOk(builder.addJointPositionLimits(group, position, handle),
            "register position limits");
}

void addVelocityLimits(mcc::KinematicsSolverBuilder &builder,
                       mcc::SolverGroup group, const SolverOptions &options) {
  mcc::JointVelocityLimitConfig velocity;
  velocity.enforcement =
      mcc::HardEnforcement{options.maximum_accepted_hard_violation};
  mcc::GroupedJointVelocityLimitHandle handle;
  requireOk(builder.addJointVelocityLimits(group, velocity, handle),
            "register velocity limits");
}

} // namespace

void requireOk(const mcc::Status &status, const std::string &) {
  if (!status.ok()) {
    throw std::runtime_error(status.message);
  }
}

mcc::JointNames activeJointNames(const R1RobotConfig &robot) {
  mcc::JointNames result;
  result.reserve(robot.joint_names.size() - kWaistJointNames.size());
  for (const auto &joint_name : robot.joint_names) {
    if (!isWaistJoint(joint_name)) {
      result.push_back(joint_name);
    }
  }
  return result;
}

std::vector<Eigen::Index> activeJointFullIndices(const R1RobotConfig &robot) {
  std::vector<Eigen::Index> result;
  result.reserve(robot.joint_names.size() - kWaistJointNames.size());
  for (std::size_t index = 0; index < robot.joint_names.size(); ++index) {
    if (!isWaistJoint(robot.joint_names[index])) {
      result.push_back(static_cast<Eigen::Index>(index));
    }
  }
  return result;
}

std::shared_ptr<const mcc::RobotModel>
loadRobotModel(const R1RobotConfig &robot, const GroupedOptions &options) {
  mcc::RobotModelDescription description;
  description.urdf_path = options.urdf_path;
  description.kinematics_reference_frame = robot.base_frame;
  description.joint_names = robot.joint_names;
  std::shared_ptr<const mcc::RobotModel> model;
  requireOk(mcc::RobotModel::load(description, model), "load robot model");
  return model;
}

std::shared_ptr<const mcc::SelfCollisionModel>
loadCollisionModel(const std::shared_ptr<const mcc::RobotModel> &model,
                   const GroupedOptions &options) {
  std::shared_ptr<const mcc::SelfCollisionModel> collision_model;
  requireOk(
      mcc::SelfCollisionModel::load(
          model, collisionModelDescription(options.urdf_path), collision_model),
      "load PSI R1 self-collision model");
  return collision_model;
}

mcc::GroupedKinematicsSolverConfig
makeSolverConfig(const GroupedOptions &options) {
  const auto &app = options;
  mcc::GroupedKinematicsSolverConfig config;
  config.profile = mcc::GroupedSolverProfile::RedYellow;
  config.red.mode = mcc::IkSolveMode::ServoStep;
  config.red.servo_period = 1.0 / app.red_rate_hz;
  config.red.maximum_iterations = 1;
  config.red.soft_solve_time_budget_ms = 1000.0 / app.red_rate_hz;
  config.yellow.mode = mcc::IkSolveMode::ServoStep;
  config.yellow.servo_period = 1.0 / app.yellow_rate_hz;
  config.yellow.maximum_iterations = 1;
  config.yellow.soft_solve_time_budget_ms = 1000.0 / app.yellow_rate_hz;
  config.yellow_to_red.enforcement =
      mcc::squaredL2Penalty(app.solver.yellow_to_red_coupling_weight, 1);
  for (auto *group : {&config.red, &config.yellow}) {
    group->joint_limit_policy =
        mcc::KinematicsJointLimitPolicy::ExplicitRequirements;
    group->qp.backend = mcc::QpBackend::ProxQp;
    group->qp.regularization = app.solver.regularization;
    group->position_tolerance_m = app.solver.position_tolerance_m;
    group->orientation_tolerance_rad = app.solver.orientation_tolerance_rad;
    group->minimum_position_improvement_m = 1.0e-8;
    group->minimum_orientation_improvement_rad = 1.0e-8;
    group->maximum_accepted_hard_violation =
        app.solver.maximum_accepted_hard_violation;
  }
  config.red.qp.proxqp.absolute_tolerance =
      app.solver.red_proxqp_absolute_tolerance;
  config.red.qp.proxqp.warm_start_enabled = false;
  return config;
}

void configureSolver(
    mcc::KinematicsSolverBuilder &builder, SolverHandles &handles,
    const std::shared_ptr<const mcc::SelfCollisionModel> &collision_model,
    const R1RobotConfig &robot, const GroupedOptions &options) {
  const auto &solver = options.solver;
  handles.red =
      addCartesianTasks(builder, mcc::SolverGroup::Red, "red", robot, solver);

  // The posture task remains intentionally disabled, matching the pre-refactor
  // topology.
  mcc::PostureTaskConfig yellow_posture;
  yellow_posture.name = "yellow-initial-posture";
  yellow_posture.enforcement =
      mcc::squaredL2Penalty(solver.yellow_posture_weight, 1);
  yellow_posture.reference_positions = toEigen(robot.default_positions);
  yellow_posture.role = mcc::PostureTaskRole::Convergence;
  yellow_posture.joint_weight_multipliers = Eigen::VectorXd::Constant(
      static_cast<Eigen::Index>(robot.default_positions.size()),
      kDefaultPostureJointWeightMultiplier);
  yellow_posture.joint_weight_multipliers(static_cast<Eigen::Index>(
      robot.left_arm_joint_indices[3])) = kArmJoint4PostureWeightMultiplier;
  yellow_posture.joint_weight_multipliers(static_cast<Eigen::Index>(
      robot.right_arm_joint_indices[3])) = kArmJoint4PostureWeightMultiplier;

  mcc::SelfCollisionAvoidanceConfig collision;
  collision.minimum_distance_m = solver.minimum_collision_distance_m;
  collision.influence_distance_m = solver.collision_influence_distance_m;
  collision.damping_gain_per_s = solver.collision_damping_gain_per_s;
  collision.weight = solver.collision_weight;
  requireOk(builder.addSelfCollisionAvoidance(mcc::SolverGroup::Yellow,
                                              collision_model, collision,
                                              handles.yellow_collision),
            "register Yellow self-collision avoidance");
  addPositionLimits(builder, mcc::SolverGroup::Red, solver);
  addVelocityLimits(builder, mcc::SolverGroup::Red, solver);
  addPositionLimits(builder, mcc::SolverGroup::Yellow, solver);
}

void beginWarmup(mcc::GroupedKinematicsSolver &solver) {
  requireOk(solver.beginRun(1), "begin warm-up run");
}

void beginTimedRun(mcc::GroupedKinematicsSolver &solver) {
  requireOk(solver.beginRun(2), "begin timed grouped run");
}

} // namespace motion_control_lab::grouped_servo_step
