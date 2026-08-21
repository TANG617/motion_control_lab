#include "components/replay/replay_source.hpp"

#include <Eigen/Geometry>

#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <thread>
#include <vector>

namespace data = motion_control_lab::data;
namespace replay = motion_control_lab::replay;

namespace
{

void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

replay::LoadedReplay makeReplay()
{
  replay::LoadedReplay loaded;
  std::vector<data::TimelineFrame<data::DualArmFrame>> frames;
  for (std::uint64_t index = 0; index < 3; ++index) {
    data::DualArmFrame value;
    value.left.pose.translation().x() = static_cast<double>(index);
    value.right.pose.translation().y() = static_cast<double>(index) + 10.0;
    frames.push_back({index, static_cast<std::int64_t>(index * 10),
      static_cast<std::int64_t>(index * 10), static_cast<std::int64_t>(index * 10),
      data::TimestampPolicy::FixedPeriod, value});
  }
  loaded.timeline.timeline = data::Timeline<data::DualArmFrame>(std::move(frames));
  return loaded;
}

}  // namespace

int main()
{
  const auto loaded = makeReplay();
  replay::ReplaySource batch(loaded, data::ExecutionMode::Batch, 1.0, true);
  require(batch.paused() && batch.sourceIndex() == 0U, "start-paused state changed");
  require(!batch.advance(10).frame_changed, "paused replay advanced");
  batch.applyControl(motion_control_lab::SourceControl::Step);
  const auto one = batch.advance(10);
  require(one.frame_changed && batch.sourceIndex() == 1U && batch.paused(), "single step failed");
  batch.markFrameProcessed();
  require(!batch.endOfStream(), "EOS reported before final frame");
  batch.applyControl(motion_control_lab::SourceControl::Resume);
  require(batch.advance(10).frame_changed && batch.sourceIndex() == 2U, "batch resume failed");
  batch.markFrameProcessed();
  require(batch.endOfStream(), "final frame did not report EOS");
  require(batch.consumedFrameCount() == 3U && batch.droppedFrameCount() == 0U,
    "batch accounting changed");

  replay::ReplaySource stopped(loaded, data::ExecutionMode::Batch, 1.0);
  stopped.applyControl(motion_control_lab::SourceControl::Stop);
  require(stopped.stopped() && !stopped.advance(10).frame_changed, "stop did not hold source");

  replay::ReplaySource realtime(loaded, data::ExecutionMode::Realtime, 10.0);
  const auto realtime_advance = realtime.advance(10);
  require(realtime_advance.frame_changed && realtime.sourceIndex() == 2U,
    "realtime catch-up did not advance to due frame");
  require(realtime_advance.dropped_frames == 1U && realtime.droppedFrameCount() == 1U,
    "realtime dropped-frame accounting changed");
  return EXIT_SUCCESS;
}
