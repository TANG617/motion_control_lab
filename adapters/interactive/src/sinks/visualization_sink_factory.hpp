#pragma once

#include "sinks/visualization_sink_options.hpp"

#include "motion_control_viz/frame_sink.hpp"

#include <memory>
#include <string>

namespace motion_control_lab
{

std::unique_ptr<motion_control::viz::FrameSink> createVisualizationSink(
  const VisualizationSinkOptions & options,
  const std::string & server_name);

}  // namespace motion_control_lab
