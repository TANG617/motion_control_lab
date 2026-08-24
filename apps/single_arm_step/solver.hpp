#pragma once

#include <memory>

#include <motion_control_core/motion_control_core.hpp>

#include "components/robot/r1/r1_robot_config.hpp"
#include "contracts/presentation/ik_app_snapshot.hpp"
#include "options.hpp"

namespace motion_control_lab::single_arm_step {

namespace mcc = motion_control::core;

struct SolverHandles {
  mcc::PositionTaskHandle position;
  mcc::OrientationTaskHandle orientation;
};

void requireOk(const mcc::Status &status);

std::shared_ptr<const mcc::RobotModel>
loadRobotModel(const R1RobotConfig &robot, const AppOptions &options);

mcc::KinematicsSolverConfig makeSolverConfig(const AppOptions &options);

void configureSolver(mcc::KinematicsSolverBuilder &builder,
                     SolverHandles &handles, const R1RobotConfig &robot,
                     ArmSide controlled_side, const AppOptions &options);

} // namespace motion_control_lab::single_arm_step
