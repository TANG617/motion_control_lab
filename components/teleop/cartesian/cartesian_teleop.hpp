#pragma once

#include "contracts/input/cartesian_teleop_options.hpp"
#include "contracts/input/input_contract.hpp"

#include <optional>
#include <string>
#include <vector>

namespace motion_control_lab
{

class CartesianTeleop
{
public:
  CartesianTeleop(
    CartesianTeleopOptions options,
    std::vector<ArmTarget> initial_targets,
    bool allow_side_switching);

  const MotionTargetFrame & frame() const noexcept;
  ArmSide selectedSide() const noexcept;
  double stepMetres() const noexcept;
  std::optional<ArmSide> apply(const TeleopIntent & intent, double dt);
  void setTargetPose(ArmSide side, const Pose & pose);
  const std::string & status() const noexcept;
  void setStatus(std::string status);

private:
  ArmTarget * selectedTarget();
  ArmTarget * targetFor(ArmSide side);

  CartesianTeleopOptions options_;
  MotionTargetFrame frame_;
  ArmSide selected_side_{ArmSide::Left};
  double step_m_{0.0};
  std::size_t rotation_axis_index_{0};
  bool allow_side_switching_{false};
  std::string status_{"Ready"};
};

}  // namespace motion_control_lab
