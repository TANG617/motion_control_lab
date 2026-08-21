#pragma once

#include "components/visualization/preview_sink_options.hpp"

#include <motion_control_viz/render_sink.hpp>

#include <memory>
#include <string>

namespace motion_control_lab
{

std::unique_ptr<motion_control::viz::RenderSink> createPreviewSink(
  const PreviewSinkOptions & options,
  const std::string & server_name);

}  // namespace motion_control_lab
