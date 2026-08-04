#pragma once

#include "config/r1_ik_options.hpp"
#include "teleop/teleop_source.hpp"

#include <motion_control_lab/ik_solver_backend.hpp>
#include "motion_control_viz/frame_sink.hpp"

#include <cstdint>
#include <functional>

namespace motion_control_lab
{

struct InteractiveRunnerOptions
{
  double rate_hz{20.0};
  double duration_s{0.0};
  std::function<bool()> stop_requested;
};

class InteractiveRunner
{
public:
  InteractiveRunner(
    InteractiveRunnerOptions options,
    IkSolverBackend & solver_backend,
    TeleopSource & teleop_source,
    motion_control::viz::FrameSink & visualization_sink);

  void run();

private:
  IkDebugFrame makeFrame(
    const TargetCommand & command,
    const IkSolveResult * result) const;

  motion_control::viz::VisualizationFrame makeVisualizationFrame(
    const IkDebugFrame & frame,
    std::uint64_t sequence,
    std::int64_t sample_time_ns,
    std::uint64_t emit_time_ns) const;

  void resetTargetFromCurrentFk(ArmSide side);

  InteractiveRunnerOptions options_;
  IkSolverBackend & solver_backend_;
  TeleopSource & teleop_source_;
  motion_control::viz::FrameSink & visualization_sink_;
};

}  // namespace motion_control_lab
