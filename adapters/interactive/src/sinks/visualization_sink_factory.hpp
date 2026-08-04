#pragma once

#include "config/r1_ik_options.hpp"

#include "motion_control_viz/frame_sink.hpp"

#include <memory>

namespace motion_control_lab
{

std::unique_ptr<motion_control::viz::FrameSink> createVisualizationSink(
  const R1IkOptions & options);

}  // namespace motion_control_lab
