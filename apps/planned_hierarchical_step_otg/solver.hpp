#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <motion_control_core/motion_control_core.hpp>

#include "components/robot/r1/r1_robot_config.hpp"
#include "options.hpp"

namespace motion_control_lab::planned_hierarchical_step_otg {

namespace mcc = motion_control::core;

struct CartesianHandles {
  mcc::TaskScaleGroupHandle left_scale;
  mcc::TaskScaleGroupHandle right_scale;
  mcc::PositionTaskHandle left_position;
  mcc::OrientationTaskHandle left_orientation;
  mcc::PositionTaskHandle right_position;
  mcc::OrientationTaskHandle right_orientation;
};

struct SolverHandles {
  CartesianHandles red;
  mcc::PostureTaskHandle red_yellow_posture;
  mcc::SelfCollisionAvoidanceHandle yellow_collision;
};

enum class WorkerGroup { Red, Yellow };
enum class SolverRejectionReason {
  None,
  InvalidTarget,
  SolverRejected,
};
enum class CouplingState {
  Unavailable,
  WaitingForValue,
  Active,
  RejectedSource,
};

struct CapturedRobotState {
  mcc::RobotState state;
  std::uint64_t sequence{0};
  std::int64_t monotonic_time_nanoseconds{0};
};

struct SolverRequest {
  CapturedRobotState captured_state;
  mcc::FrameName reference_frame_name;
  std::vector<mcc::PositionTaskTarget> position_targets;
  std::vector<mcc::OrientationTaskTarget> orientation_targets;
};

struct SolverSolution {
  mcc::InverseKinematicsSolution kinematics_solution;
};

struct SolverDiagnostics {
  WorkerGroup group{WorkerGroup::Red};
  SolverRejectionReason rejection_reason{SolverRejectionReason::None};
  std::uint64_t run_generation{0};
  std::uint64_t attempt_revision{0};
  std::uint64_t value_revision{0};
  CouplingState coupling_state{CouplingState::Unavailable};
  std::uint64_t consumed_source_value_revision{0};
  std::uint64_t captured_state_sequence{0};
  std::int64_t captured_state_time_nanoseconds{0};
  bool hierarchical{false};
  mcc::InverseKinematicsDiagnostics kinematics;
  mcc::HierarchicalInverseKinematicsDiagnostics hierarchy;
  double solve_time_ms{0.0};
  int iterations{0};
  bool converged{false};
  double maximum_hard_violation{0.0};
};

class SolverRuntime {
public:
  void initialize(const SolverHandles &handles,
                  Eigen::Index active_joint_count);
  mcc::KinematicsSolver &yellowSolver() { return yellow_solver_; }
  mcc::HierarchicalKinematicsSolver &redSolver() { return red_solver_; }
  mcc::KinematicsSolver &fkSolver() { return fk_solver_; }
  void beginRun(std::uint64_t generation);
  mcc::Status
  computeForwardKinematics(const mcc::ForwardKinematicsRequest &request,
                           mcc::ForwardKinematicsSolution &solution,
                           mcc::ForwardKinematicsDiagnostics &diagnostics);
  mcc::Status solveYellow(const SolverRequest &request,
                          SolverSolution &solution,
                          SolverDiagnostics &diagnostics);
  mcc::Status solveRed(const SolverRequest &request, SolverSolution &solution,
                       SolverDiagnostics &diagnostics);
  mcc::Status
  getSelfCollisionDiagnostics(mcc::SelfCollisionAvoidanceHandle handle,
                              mcc::SelfCollisionDiagnostics &diagnostics);

private:
  struct YellowEnvelope {
    Eigen::VectorXd accepted_positions;
    std::uint64_t value_revision{0};
    bool attempt_accepted{false};
  };
  struct GroupState {
    std::uint64_t attempt_revision{0};
    std::uint64_t value_revision{0};
  };
  void initializeDiagnostics(WorkerGroup group, const SolverRequest &request,
                             const GroupState &state,
                             SolverDiagnostics &diagnostics) const;
  mcc::KinematicsSolver yellow_solver_;
  mcc::HierarchicalKinematicsSolver red_solver_;
  mcc::KinematicsSolver fk_solver_;
  SolverHandles handles_;
  mcc::SnapshotBuffer<YellowEnvelope> yellow_to_red_;
  YellowEnvelope yellow_publish_;
  YellowEnvelope yellow_read_;
  Eigen::VectorXd disabled_coupling_positions_;
  std::uint64_t generation_{0};
  GroupState yellow_state_;
  GroupState red_state_;
};

void requireOk(const mcc::Status &status, const std::string &context);
mcc::JointNames activeJointNames(const R1RobotConfig &robot);
std::vector<std::size_t> activeJointFullIndices(const R1RobotConfig &robot);
std::shared_ptr<const mcc::RobotModel>
loadRobotModel(const R1RobotConfig &robot, const Options &options);
std::shared_ptr<const mcc::SelfCollisionModel>
loadCollisionModel(const std::shared_ptr<const mcc::RobotModel> &model,
                   const Options &options);
void configureSolver(
    SolverRuntime &runtime, SolverHandles &handles,
    const std::shared_ptr<const mcc::RobotModel> &model,
    const mcc::JointNames &active_joint_names,
    const std::shared_ptr<const mcc::SelfCollisionModel> &collision_model,
    const R1RobotConfig &robot, const Options &options);

} // namespace motion_control_lab::planned_hierarchical_step_otg
