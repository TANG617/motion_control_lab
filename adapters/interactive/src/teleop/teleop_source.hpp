#pragma once

#include "runtime/interactive_types.hpp"

#include <cstddef>
#include <optional>
#include <string>

namespace motion_control_lab
{

class TeleopSource
{
public:
  virtual ~TeleopSource() = default;

  virtual void poll() = 0;

  virtual const TargetCommand & command() const = 0;

  virtual std::optional<ArmSide> consumeResetRequest() = 0;

  virtual void setTargetPose(
    ArmSide side,
    const Pose & target_pose,
    const std::string & status) = 0;

  virtual void setStatus(const std::string & status) = 0;

  virtual void render(
    const IkDebugFrame & frame,
    std::size_t publish_count,
    const std::string & sink_status) = 0;
};

}  // namespace motion_control_lab
