#include "components/visualization/preview_transport.hpp"

#include <motion_control_viz/null_render_sink.hpp>

#if MCL_WITH_FOXGLOVE_TRANSPORT
#include <motion_control_viz/fanout_render_sink.hpp>
#include <motion_control_viz/foxglove_mcap_sink.hpp>
#include <motion_control_viz/foxglove_websocket_sink.hpp>
#endif

#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace motion_control_lab
{

std::unique_ptr<motion_control::viz::RenderSink> createPreviewSink(
  const PreviewSinkOptions & options,
  const std::string & server_name)
{
  if (!options.enabled) {
    return std::make_unique<motion_control::viz::NullRenderSink>();
  }
#if MCL_WITH_FOXGLOVE_TRANSPORT
  motion_control::viz::FoxgloveWebSocketSinkOptions websocket_options;
  websocket_options.server_name = server_name;
  websocket_options.host = options.host;
  websocket_options.port = options.port;
  auto websocket = std::make_unique<motion_control::viz::FoxgloveWebSocketSink>(
    std::move(websocket_options));
  if (!options.mcap_path.has_value()) return websocket;
  std::vector<std::unique_ptr<motion_control::viz::RenderSink>> sinks;
  sinks.push_back(std::move(websocket));
  sinks.push_back(std::make_unique<motion_control::viz::FoxgloveMcapSink>(
    motion_control::viz::FoxgloveMcapSinkOptions{*options.mcap_path}));
  return std::make_unique<motion_control::viz::FanoutRenderSink>(std::move(sinks));
#else
  static_cast<void>(options);
  static_cast<void>(server_name);
  throw std::runtime_error(
          "Foxglove visualization was requested but the transport was not built; use --viz none");
#endif
}

}  // namespace motion_control_lab
