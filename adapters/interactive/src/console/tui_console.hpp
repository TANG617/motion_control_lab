#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "config/interactive_ik_options.hpp"
#include "controls/tui_source_controls.hpp"
#include "runtime/interactive_types.hpp"

namespace motion_control_lab
{

class TuiConsole
{
public:
  TuiConsole(
    const TuiTeleopOptions & options, double rate_hz, std::string title,
    InteractiveIkPresentation presentation, std::vector<ArmTarget> initial_targets,
    bool allow_side_switching, bool console_enabled = true,
    TuiControlMode control_mode = TuiControlMode::Teleop);
  ~TuiConsole();

  void poll();

  const TargetCommand & command() const;

  std::optional<ArmSide> consumeResetRequest();

  bool consumeSingleStepRequest();

  void setTargetPose(ArmSide side, const Pose & target_pose, const std::string & status);

  void setStatus(const std::string & status);

  void setMotionInputEnabled(bool enabled, const std::string & status);

  void render(
    const IkDebugFrame & frame, std::size_t publish_count, const std::string & sink_status);

private:
  class Impl;

  TuiTeleopOptions options_;
  double rate_hz_{20.0};
  std::string title_;
  InteractiveIkPresentation presentation_;
  TargetCommand command_;
  double step_m_{0.01};
  std::size_t rotation_axis_index_{0};
  double rotation_step_rad_{0.0};
  bool show_help_{false};
  bool allow_side_switching_{false};
  bool console_enabled_{true};
  TuiSourceControls source_controls_;
  bool single_step_requested_{false};
  bool motion_input_enabled_{true};
  std::string motion_input_disabled_status_{"Motion controls disabled"};
  std::optional<ArmSide> reset_requested_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace motion_control_lab
