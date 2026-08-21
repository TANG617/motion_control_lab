#pragma once

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>

#include <motion_control_core/motion_control_core.hpp>

#include "components/robot/r1/r1_robot_config.hpp"
#include "options.hpp"

namespace motion_control_lab::planned_grouped_step_otg {

namespace mcc = motion_control::core;

struct CartesianHandles {
  mcc::GroupedTaskScaleGroupHandle left_scale;
  mcc::GroupedTaskScaleGroupHandle right_scale;
  mcc::GroupedPositionTaskHandle left_position;
  mcc::GroupedOrientationTaskHandle left_orientation;
  mcc::GroupedPositionTaskHandle right_position;
  mcc::GroupedOrientationTaskHandle right_orientation;
};

struct SolverHandles {
  CartesianHandles red;
  mcc::GroupedPostureTaskHandle yellow_posture;
  mcc::GroupedSelfCollisionAvoidanceHandle yellow_collision;
};

void requireOk(const mcc::Status &status, const std::string &context);

mcc::JointNames activeJointNames(const R1RobotConfig &robot);

std::vector<std::size_t> activeJointFullIndices(const R1RobotConfig &robot);

std::shared_ptr<const mcc::RobotModel>
loadRobotModel(const R1RobotConfig &robot, const Options &options);

std::shared_ptr<const mcc::SelfCollisionModel>
loadCollisionModel(const std::shared_ptr<const mcc::RobotModel> &model,
                   const Options &options);

mcc::GroupedKinematicsSolverConfig makeSolverConfig(const Options &options);

void configureSolver(
    mcc::KinematicsSolverBuilder &builder, SolverHandles &handles,
    const std::shared_ptr<const mcc::SelfCollisionModel> &collision_model,
    const R1RobotConfig &robot, const Options &options);

void beginWarmup(mcc::GroupedKinematicsSolver &solver);

void beginTimedRun(mcc::GroupedKinematicsSolver &solver);

} // namespace motion_control_lab::planned_grouped_step_otg
