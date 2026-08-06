#include "ik_app_utils.hpp"

#include <stdexcept>

namespace motion_control_lab
{

Eigen::VectorXd toEigen(const std::vector<double> & values)
{
  Eigen::VectorXd result(static_cast<Eigen::Index>(values.size()));
  for (std::size_t index = 0; index < values.size(); ++index) {
    result(static_cast<Eigen::Index>(index)) = values[index];
  }
  return result;
}

std::vector<double> toStdVector(const Eigen::VectorXd & values)
{
  return std::vector<double>(values.data(), values.data() + values.size());
}

void requireOk(
  const motion_control::core::Status & status,
  const std::string & operation)
{
  if (!status.ok()) {
    throw std::runtime_error(operation + ": " + status.message);
  }
}

motion_control::core::RobotState makeRobotState(
  const std::vector<double> & positions,
  const std::vector<double> & velocities)
{
  motion_control::core::RobotState state;
  state.joint_positions = toEigen(positions);
  state.joint_velocities = toEigen(velocities);
  return state;
}

const motion_control::core::FramePose & requirePose(
  const std::vector<motion_control::core::FramePose> & poses,
  const std::string & frame_name)
{
  for (const auto & pose : poses) {
    if (pose.frame_name == frame_name) {
      return pose;
    }
  }
  throw std::runtime_error("kinematics result did not contain frame: " + frame_name);
}

const ArmTarget & requireTarget(
  const std::vector<ArmTarget> & targets,
  ArmSide side)
{
  for (const auto & target : targets) {
    if (target.side == side) {
      return target;
    }
  }
  throw std::runtime_error(
          std::string{"TUI did not provide a target for "} + armSideName(side));
}

}  // namespace motion_control_lab
