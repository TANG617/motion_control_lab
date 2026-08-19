#pragma once

#include "runtime/interactive_types.hpp"

#include "motion_control_viz/render_batch.hpp"

#include <cstdint>

namespace motion_control_lab
{

motion_control::viz::RenderBatch makeIkRenderBatch(
  const IkDebugFrame & frame,
  const InteractiveIkPresentation & presentation,
  std::uint64_t timestamp_ns);

}  // namespace motion_control_lab
