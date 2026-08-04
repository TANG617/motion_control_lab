#include "sinks/visualization_sink_factory.hpp"

#include "motion_control_viz/null_frame_sink.hpp"

#if MCL_WITH_FOXGLOVE_SINK
#include "motion_control_viz/foxglove_frame_sink.hpp"
#endif

#include <memory>
#include <utility>

namespace motion_control_lab
{

std::unique_ptr<motion_control::viz::FrameSink> createVisualizationSink(
  const R1IkOptions & options)
{
#if MCL_WITH_FOXGLOVE_SINK
  motion_control::viz::FoxgloveFrameSinkOptions sink_options;
  sink_options.server_name = "r1_ik_tui_teleop";
  sink_options.host = options.host;
  sink_options.port = options.port;
  sink_options.mcap_path = options.mcap_path;
  return std::make_unique<motion_control::viz::FoxgloveFrameSink>(
    std::move(sink_options));
#else
  (void)options;
  return std::make_unique<motion_control::viz::NullFrameSink>();
#endif
}

}  // namespace motion_control_lab
