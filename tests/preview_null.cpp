#include "components/visualization/preview_transport.hpp"

#include <motion_control_viz/null_render_sink.hpp>
#include <motion_control_viz/render_batch.hpp>

#include <cstdlib>

int main()
{
  motion_control_lab::PreviewSinkOptions options;
  options.enabled = false;
  auto sink = motion_control_lab::createPreviewSink(options, "null-test");
  if (dynamic_cast<motion_control::viz::NullRenderSink *>(sink.get()) == nullptr) {
    return EXIT_FAILURE;
  }
  sink->open();
  sink->write({});
  sink->flush();
  sink->close();
  return EXIT_SUCCESS;
}
