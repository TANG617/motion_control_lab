#include "components/replay/replay_source.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace motion_control_lab::replay
{

ReplaySource::ReplaySource(
  const LoadedReplay & loaded,
  data::ExecutionMode mode,
  double playback_rate,
  bool start_paused)
: loaded_(loaded),
  clock_(mode, playback_rate),
  clock_origin_(clock_.now())
{
  if (loaded_.timeline.timeline.empty()) {
    throw std::runtime_error("replay timeline is empty");
  }
  status_.state = start_paused ? InputState::Paused : InputState::Running;
  status_.detail = start_paused
    ? "Replay timeline paused; press space to start"
    : "Replay timeline running";
  timeline_time_ns_ = loaded_.timeline.timeline.at(0).projected_time_ns;
  updateFrame();
  rebaseClock();
}

const MotionTargetFrame & ReplaySource::frame() const noexcept { return frame_; }

const data::TimelineFrame<data::DualArmFrame> & ReplaySource::sourceFrame() const
{
  return loaded_.timeline.timeline.at(source_index_);
}

const InputStatus & ReplaySource::status() const noexcept { return status_; }

std::size_t ReplaySource::sourceIndex() const noexcept { return source_index_; }
std::size_t ReplaySource::consumedFrameCount() const noexcept { return consumed_frame_count_; }
std::size_t ReplaySource::droppedFrameCount() const noexcept { return dropped_frame_count_; }
std::size_t ReplaySource::deadlineMissCount() const noexcept { return deadline_miss_count_; }
bool ReplaySource::paused() const noexcept { return status_.state == InputState::Paused; }
bool ReplaySource::stopped() const noexcept { return stopped_; }
bool ReplaySource::endOfStream() const noexcept { return end_of_stream_; }

void ReplaySource::applyControl(SourceControl control)
{
  switch (control) {
    case SourceControl::Pause:
      status_.state = InputState::Paused;
      status_.detail = "Replay timeline paused";
      break;
    case SourceControl::Resume:
      status_.state = InputState::Running;
      status_.detail = "Replay timeline resumed";
      rebaseClock();
      break;
    case SourceControl::TogglePause:
      if (paused()) {
        applyControl(SourceControl::Resume);
      } else {
        applyControl(SourceControl::Pause);
      }
      break;
    case SourceControl::Step:
      single_step_requested_ = true;
      status_.state = InputState::Paused;
      status_.detail = "Replay single-frame step requested";
      break;
    case SourceControl::Stop:
      stopped_ = true;
      status_.state = InputState::Stopped;
      status_.detail = "Replay stopped";
      break;
  }
}

ReplayAdvance ReplaySource::advance(std::int64_t tick_period_ns)
{
  if (tick_period_ns <= 0) {
    throw std::runtime_error("replay tick period must be positive");
  }
  ReplayAdvance result;
  if (stopped_ || end_of_stream_) {
    result.end_of_stream = end_of_stream_;
    return result;
  }
  if (paused() && !single_step_requested_) {
    return result;
  }

  if (single_step_requested_ || clock_.mode() == data::ExecutionMode::Batch) {
    single_step_requested_ = false;
    if (source_index_ + 1U < loaded_.timeline.timeline.size()) {
      timeline_time_ns_ = loaded_.timeline.timeline.at(source_index_ + 1U).projected_time_ns;
    }
  } else {
    timeline_time_ns_ += static_cast<std::int64_t>(std::llround(
      static_cast<double>(tick_period_ns) * clock_.playbackRate()));
  }

  std::size_t next_index = source_index_;
  while (next_index + 1U < loaded_.timeline.timeline.size() &&
         loaded_.timeline.timeline.at(next_index + 1U).projected_time_ns <= timeline_time_ns_) {
    ++next_index;
  }
  if (next_index > source_index_) {
    result.dropped_frames = clock_.mode() == data::ExecutionMode::Realtime
      ? next_index - source_index_ - 1U
      : 0U;
    dropped_frame_count_ += result.dropped_frames;
    source_index_ = next_index;
    ++consumed_frame_count_;
    result.frame_changed = true;
    updateFrame();
  }
  return result;
}

ReplayAdvance ReplaySource::advanceSequential()
{
  ReplayAdvance result;
  if (stopped_ || end_of_stream_) {
    result.end_of_stream = end_of_stream_;
    return result;
  }
  if (source_index_ + 1U >= loaded_.timeline.timeline.size()) {
    return result;
  }
  ++source_index_;
  ++consumed_frame_count_;
  timeline_time_ns_ = sourceFrame().projected_time_ns;
  updateFrame();
  result.frame_changed = true;
  return result;
}

data::ReplayClock::TimePoint ReplaySource::scheduledTime() const
{
  return clock_.deadline(clock_origin_, timeline_time_ns_);
}

void ReplaySource::waitForCurrentFrame()
{
  if (clock_.mode() != data::ExecutionMode::Realtime || paused()) {
    return;
  }
  const auto deadline = scheduledTime();
  clock_.waitUntil(deadline);
  if (clock_.now() > deadline) {
    ++deadline_miss_count_;
  }
}

void ReplaySource::markFrameProcessed()
{
  if (source_index_ + 1U == loaded_.timeline.timeline.size()) {
    end_of_stream_ = true;
    status_.state = InputState::EndOfStream;
    status_.detail = "Replay end of stream";
  }
}

void ReplaySource::updateFrame()
{
  const auto & source = sourceFrame();
  frame_.targets = {
    {ArmSide::Left, source.value.left.pose},
    {ArmSide::Right, source.value.right.pose},
  };
  frame_.logical_time_ns = source.original_logical_time_ns;
  frame_.revision = source.sequence;
  frame_.source = "replay";
}

void ReplaySource::rebaseClock()
{
  const long double scaled =
    static_cast<long double>(timeline_time_ns_) / clock_.playbackRate();
  clock_origin_ = clock_.now() - std::chrono::nanoseconds{
    static_cast<std::int64_t>(std::llround(scaled))};
}

}  // namespace motion_control_lab::replay
