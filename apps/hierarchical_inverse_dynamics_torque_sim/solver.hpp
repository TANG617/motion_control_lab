#pragma once

#include <memory>

#include <motion_control_core/motion_control_core.hpp>

#include "components/robot/r1/r1_robot_config.hpp"
#include "options.hpp"

namespace motion_control_lab::hierarchical_inverse_dynamics_torque_sim {

namespace mcc = motion_control::core;

struct Handles {
  mcc::PositionAccelerationTaskHandle left_position;
  mcc::PositionAccelerationTaskHandle right_position;
  mcc::OrientationAccelerationTaskHandle left_orientation;
  mcc::OrientationAccelerationTaskHandle right_orientation;
  mcc::PostureAccelerationTaskHandle posture;
};

struct Runtime {
  std::shared_ptr<const mcc::RobotModel> robot_model;
  std::shared_ptr<const mcc::ActuationModel> actuation_model;
  mcc::HierarchicalInverseDynamicsSolver solver;
};

Runtime configureSolver(const R1RobotConfig &robot, const Options &options,
                        Handles &handles);

} // namespace motion_control_lab::hierarchical_inverse_dynamics_torque_sim
