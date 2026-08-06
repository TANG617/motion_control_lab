#pragma once

#include "runtime/interactive_types.hpp"

#include "motion_control_viz/frame.hpp"

#include <cstdint>

namespace motion_control_lab
{

motion_control::viz::VisualizationFrame makeIkVisualizationFrame(
  const IkDebugFrame & frame,
  const InteractiveIkPresentation & presentation,
  std::uint64_t sequence,
  std::int64_t sample_time_ns,
  std::uint64_t emit_time_ns);

}  // namespace motion_control_lab
