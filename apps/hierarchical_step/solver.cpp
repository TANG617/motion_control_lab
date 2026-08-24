#include "solver.hpp"

#include <algorithm>
#include <array>
#include <filesystem>
#include <stdexcept>
#include <string_view>

namespace motion_control_lab::hierarchical_step {
namespace {

constexpr double kCartesianPreservationTolerance = 5.0e-4;
constexpr double kScalePreservationTolerance = 1.0e-4;
constexpr double kPosturePreservationTolerance = 1.0e-5;
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

mcc::KinematicsSolverConfig makeYellowConfig(const HierarchicalOptions &options) {
  mcc::KinematicsSolverConfig config;
  config.mode = mcc::IkSolveMode::ServoStep;
  config.servo_period = 1.0 / options.yellow_rate_hz;
  config.maximum_iterations = 1;
  config.soft_solve_time_budget_ms = 1000.0 / options.yellow_rate_hz;
  config.joint_limit_policy =
      mcc::KinematicsJointLimitPolicy::ExplicitRequirements;
  config.qp.backend = mcc::QpBackend::ProxQp;
  config.qp.regularization = options.solver.regularization;
  config.position_tolerance_m = options.solver.position_tolerance_m;
  config.orientation_tolerance_rad = options.solver.orientation_tolerance_rad;
  config.minimum_position_improvement_m = 1.0e-8;
  config.minimum_orientation_improvement_rad = 1.0e-8;
  config.maximum_accepted_hard_violation =
      options.solver.maximum_accepted_hard_violation;
  return config;
}

mcc::HierarchicalKinematicsSolverConfig
makeRedConfig(const HierarchicalOptions &options) {
  mcc::HierarchicalKinematicsSolverConfig config;
  config.mode = mcc::IkSolveMode::ServoStep;
  config.backend = mcc::QpBackend::ProxQp;
  config.servo_period = 1.0 / options.red_rate_hz;
  config.joint_limit_policy =
      mcc::KinematicsJointLimitPolicy::ExplicitRequirements;
  config.proxqp.absolute_tolerance =
      options.solver.red_proxqp_absolute_tolerance;
  config.proxqp.warm_start_enabled = false;
  config.maximum_accepted_hard_violation =
      options.solver.maximum_accepted_hard_violation;
  return config;
}

CartesianHandles
addCartesianTasks(mcc::HierarchicalKinematicsSolverBuilder &builder,
                  const R1RobotConfig &robot, const SolverOptions &options) {
  CartesianHandles handles;

  mcc::TaskScaleGroupConfig scale;
  scale.progress_weight = options.cartesian_progress_weight;
  scale.name = "red-left-cartesian-progress";
  requireOk(builder.addTaskScaleGroup(mcc::PriorityLevel::Primary, scale,
                                      kScalePreservationTolerance,
                                      handles.left_scale),
            "register " + scale.name);
  scale.name = "red-right-cartesian-progress";
  requireOk(builder.addTaskScaleGroup(mcc::PriorityLevel::Primary, scale,
                                      kScalePreservationTolerance,
                                      handles.right_scale),
            "register " + scale.name);

  mcc::PositionTaskConfig position;
  position.name = "red-left-position";
  position.enforcement = mcc::ScaledEnforcement{
      handles.left_scale, options.maximum_accepted_hard_violation};
  requireOk(builder.addPositionTask(
                mcc::PriorityLevel::Primary, robot.left_end_effector_frame,
                position,
                Eigen::Vector3d::Constant(kCartesianPreservationTolerance),
                handles.left_position),
            "register " + position.name);
  position.name = "red-right-position";
  position.enforcement = mcc::ScaledEnforcement{
      handles.right_scale, options.maximum_accepted_hard_violation};
  requireOk(builder.addPositionTask(
                mcc::PriorityLevel::Primary, robot.right_end_effector_frame,
                position,
                Eigen::Vector3d::Constant(kCartesianPreservationTolerance),
                handles.right_position),
            "register " + position.name);

  mcc::OrientationTaskConfig orientation;
  orientation.name = "red-left-orientation";
  orientation.enforcement = mcc::ScaledEnforcement{
      handles.left_scale, options.maximum_accepted_hard_violation};
  requireOk(builder.addOrientationTask(
                mcc::PriorityLevel::Primary, robot.left_end_effector_frame,
                orientation,
                Eigen::Vector3d::Constant(kCartesianPreservationTolerance),
                handles.left_orientation),
            "register " + orientation.name);
  orientation.name = "red-right-orientation";
  orientation.enforcement = mcc::ScaledEnforcement{
      handles.right_scale, options.maximum_accepted_hard_violation};
  requireOk(builder.addOrientationTask(
                mcc::PriorityLevel::Primary, robot.right_end_effector_frame,
                orientation,
                Eigen::Vector3d::Constant(kCartesianPreservationTolerance),
                handles.right_orientation),
            "register " + orientation.name);
  return handles;
}

void addRedLimits(mcc::HierarchicalKinematicsSolverBuilder &builder,
                  const SolverOptions &options) {
  mcc::JointPositionLimitConfig position;
  position.margin = options.joint_position_margin_rad;
  position.enforcement =
      mcc::HardEnforcement{options.maximum_accepted_hard_violation};
  mcc::JointPositionLimitHandle position_handle;
  requireOk(builder.addJointPositionLimits(position, position_handle),
            "register Red position limits");
  mcc::JointVelocityLimitConfig velocity;
  velocity.enforcement =
      mcc::HardEnforcement{options.maximum_accepted_hard_violation};
  mcc::JointVelocityLimitHandle velocity_handle;
  requireOk(builder.addJointVelocityLimits(velocity, velocity_handle),
            "register Red velocity limits");
}

void addYellowLimits(mcc::KinematicsSolverBuilder &builder,
                     const SolverOptions &options) {
  mcc::JointPositionLimitConfig position;
  position.margin = options.joint_position_margin_rad;
  position.enforcement =
      mcc::HardEnforcement{options.maximum_accepted_hard_violation};
  mcc::JointPositionLimitHandle handle;
  requireOk(builder.addJointPositionLimits(position, handle),
            "register Yellow position limits");
}

double hierarchicalSolveTime(
    const mcc::HierarchicalInverseKinematicsDiagnostics &diagnostics) {
  double result = 0.0;
  for (const auto &pass : diagnostics.passes) {
    result += pass.solve_time_ms;
  }
  return result;
}

int hierarchicalIterations(
    const mcc::HierarchicalInverseKinematicsDiagnostics &diagnostics) {
  int result = 0;
  for (const auto &pass : diagnostics.passes) {
    result += pass.iterations;
  }
  return result;
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
loadRobotModel(const R1RobotConfig &robot, const HierarchicalOptions &options) {
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
                   const HierarchicalOptions &options) {
  std::shared_ptr<const mcc::SelfCollisionModel> collision_model;
  requireOk(
      mcc::SelfCollisionModel::load(
          model, collisionModelDescription(options.urdf_path), collision_model),
      "load PSI R1 self-collision model");
  return collision_model;
}

void configureSolver(
    SolverRuntime &runtime, SolverHandles &handles,
    const std::shared_ptr<const mcc::RobotModel> &model,
    const mcc::JointNames &active_joint_names,
    const std::shared_ptr<const mcc::SelfCollisionModel> &collision_model,
    const R1RobotConfig &robot, const HierarchicalOptions &options) {
  mcc::HierarchicalKinematicsSolverBuilder red_builder;
  requireOk(
      red_builder.configure(model, active_joint_names, makeRedConfig(options)),
      "configure Red HKS");
  handles.red = addCartesianTasks(red_builder, robot, options.solver);

  mcc::PostureTaskConfig coupling;
  coupling.name = "yellow-posture-preference";
  coupling.enforcement =
      mcc::squaredL2Penalty(options.solver.yellow_to_red_coupling_weight, 1);
  coupling.reference_positions = Eigen::VectorXd::Zero(
      static_cast<Eigen::Index>(active_joint_names.size()));
  coupling.role = mcc::PostureTaskRole::Regularization;
  requireOk(red_builder.addPostureTask(
                mcc::PriorityLevel::Secondary, coupling,
                Eigen::VectorXd::Constant(
                    static_cast<Eigen::Index>(active_joint_names.size()),
                    kPosturePreservationTolerance),
                handles.red_yellow_posture),
            "register Secondary Yellow posture preference");
  addRedLimits(red_builder, options.solver);
  requireOk(red_builder.finalize(runtime.redSolver()), "finalize Red HKS");

  mcc::KinematicsSolverBuilder yellow_builder;
  requireOk(yellow_builder.configure(model, active_joint_names,
                                     makeYellowConfig(options)),
            "configure Yellow solver");
  mcc::SelfCollisionAvoidanceConfig collision;
  collision.minimum_distance_m = options.solver.minimum_collision_distance_m;
  collision.influence_distance_m =
      options.solver.collision_influence_distance_m;
  collision.damping_gain_per_s = options.solver.collision_damping_gain_per_s;
  collision.weight = options.solver.collision_weight;
  requireOk(yellow_builder.addSelfCollisionAvoidance(collision_model, collision,
                                                     handles.yellow_collision),
            "register Yellow self-collision avoidance");
  addYellowLimits(yellow_builder, options.solver);
  requireOk(yellow_builder.finalize(runtime.yellowSolver()),
            "finalize Yellow solver");

  mcc::KinematicsSolverBuilder fk_builder;
  mcc::KinematicsSolverConfig fk_config;
  fk_config.mode = mcc::IkSolveMode::TargetSolve;
  fk_config.joint_limit_policy = mcc::KinematicsJointLimitPolicy::Unconstrained;
  fk_config.qp.backend = mcc::QpBackend::ProxQp;
  fk_config.qp.regularization = options.solver.regularization;
  requireOk(fk_builder.configure(model, active_joint_names, fk_config),
            "configure FK solver");
  requireOk(fk_builder.finalize(runtime.fkSolver()), "finalize FK solver");

  runtime.initialize(handles,
                     static_cast<Eigen::Index>(active_joint_names.size()));
}

void SolverRuntime::initialize(const SolverHandles &handles,
                               Eigen::Index active_joint_count) {
  handles_ = handles;
  YellowEnvelope prototype;
  prototype.accepted_positions.setZero(active_joint_count);
  yellow_to_red_.initialize(prototype);
  yellow_publish_ = prototype;
  yellow_read_ = prototype;
  disabled_coupling_positions_.setZero(active_joint_count);
}

void SolverRuntime::beginRun(std::uint64_t generation) {
  generation_ = generation;
  yellow_state_ = GroupState{};
  red_state_ = GroupState{};
  yellow_to_red_.reset();
}

mcc::Status SolverRuntime::computeForwardKinematics(
    const mcc::ForwardKinematicsRequest &request,
    mcc::ForwardKinematicsSolution &solution,
    mcc::ForwardKinematicsDiagnostics &diagnostics) {
  return fk_solver_.computeForwardKinematics(request, solution, diagnostics);
}

void SolverRuntime::initializeDiagnostics(
    WorkerGroup group, const SolverRequest &request, const GroupState &state,
    SolverDiagnostics &diagnostics) const {
  diagnostics = SolverDiagnostics{};
  diagnostics.group = group;
  diagnostics.run_generation = generation_;
  diagnostics.attempt_revision = state.attempt_revision;
  diagnostics.value_revision = state.value_revision;
  diagnostics.captured_state_sequence = request.captured_state.sequence;
  diagnostics.captured_state_time_nanoseconds =
      request.captured_state.monotonic_time_nanoseconds;
}

mcc::Status SolverRuntime::solveYellow(const SolverRequest &request,
                                       SolverSolution &solution,
                                       SolverDiagnostics &diagnostics) {
  ++yellow_state_.attempt_revision;
  initializeDiagnostics(WorkerGroup::Yellow, request, yellow_state_,
                        diagnostics);

  mcc::InverseKinematicsRequest local;
  local.state = request.captured_state.state;
  local.reference_frame_name = request.reference_frame_name;
  local.position_targets = request.position_targets;
  local.orientation_targets = request.orientation_targets;
  mcc::InverseKinematicsSolution candidate;
  const mcc::Status status = yellow_solver_.solveInverseKinematics(
      local, candidate, diagnostics.kinematics);
  diagnostics.solve_time_ms = diagnostics.kinematics.solve_time_ms;
  diagnostics.iterations = diagnostics.kinematics.iterations;
  diagnostics.converged = diagnostics.kinematics.converged;
  diagnostics.maximum_hard_violation =
      diagnostics.kinematics.optimization.maximum_hard_violation;
  diagnostics.coupling_state = CouplingState::Unavailable;
  if (!status.ok()) {
    diagnostics.rejection_reason = status.code == mcc::StatusCode::InvalidTarget
                                       ? SolverRejectionReason::InvalidTarget
                                       : SolverRejectionReason::SolverRejected;
    yellow_publish_.attempt_accepted = false;
    yellow_publish_.value_revision = yellow_state_.value_revision;
    yellow_to_red_.publish(yellow_publish_);
    return status;
  }
  ++yellow_state_.value_revision;
  yellow_publish_.accepted_positions = candidate.joint_positions;
  yellow_publish_.value_revision = yellow_state_.value_revision;
  yellow_publish_.attempt_accepted = true;
  yellow_to_red_.publish(yellow_publish_);
  diagnostics.rejection_reason = SolverRejectionReason::None;
  diagnostics.value_revision = yellow_state_.value_revision;
  solution.kinematics_solution = candidate;
  return status;
}

mcc::Status SolverRuntime::solveRed(const SolverRequest &request,
                                    SolverSolution &solution,
                                    SolverDiagnostics &diagnostics) {
  ++red_state_.attempt_revision;
  initializeDiagnostics(WorkerGroup::Red, request, red_state_, diagnostics);
  diagnostics.hierarchical = true;

  mcc::InverseKinematicsRequest local;
  local.state = request.captured_state.state;
  local.reference_frame_name = request.reference_frame_name;
  local.position_targets = request.position_targets;
  local.orientation_targets = request.orientation_targets;
  const bool has_yellow_attempt = yellow_to_red_.readLatest(yellow_read_);
  const bool coupling_active =
      has_yellow_attempt && yellow_read_.attempt_accepted;
  diagnostics.coupling_state =
      !has_yellow_attempt ? CouplingState::WaitingForValue
                          : (coupling_active ? CouplingState::Active
                                             : CouplingState::RejectedSource);
  diagnostics.consumed_source_value_revision =
      coupling_active ? yellow_read_.value_revision : 0;
  local.posture_targets.emplace_back(handles_.red_yellow_posture,
                                     coupling_active
                                         ? yellow_read_.accepted_positions
                                         : disabled_coupling_positions_,
                                     coupling_active);

  mcc::InverseKinematicsSolution candidate;
  const mcc::Status status = red_solver_.solveInverseKinematics(
      local, candidate, diagnostics.hierarchy);
  diagnostics.solve_time_ms = hierarchicalSolveTime(diagnostics.hierarchy);
  diagnostics.iterations = hierarchicalIterations(diagnostics.hierarchy);
  diagnostics.converged = false;
  diagnostics.maximum_hard_violation =
      diagnostics.hierarchy.maximum_shared_hard_violation;
  if (!status.ok()) {
    diagnostics.rejection_reason = status.code == mcc::StatusCode::InvalidTarget
                                       ? SolverRejectionReason::InvalidTarget
                                       : SolverRejectionReason::SolverRejected;
    return status;
  }
  ++red_state_.value_revision;
  diagnostics.rejection_reason = SolverRejectionReason::None;
  diagnostics.value_revision = red_state_.value_revision;
  solution.kinematics_solution = candidate;
  return status;
}

mcc::Status SolverRuntime::getSelfCollisionDiagnostics(
    mcc::SelfCollisionAvoidanceHandle handle,
    mcc::SelfCollisionDiagnostics &diagnostics) {
  return yellow_solver_.getSelfCollisionDiagnostics(handle, diagnostics);
}

} // namespace motion_control_lab::hierarchical_step
