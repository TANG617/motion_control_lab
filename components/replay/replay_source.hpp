#pragma once

#include "adapters/replay/replay_support.hpp"
#include "contracts/input/input_contract.hpp"

#include <cstddef>
#include <cstdint>

namespace motion_control_lab::replay
{

struct ReplayAdvance
{
  bool frame_changed{false};
  std::size_t dropped_frames{0};
  bool end_of_stream{false};
};

class ReplaySource
{
public:
  ReplaySource(
    const LoadedReplay & loaded,
    data::ExecutionMode mode,
    double playback_rate,
    bool start_paused = false);

  const MotionTargetFrame & frame() const noexcept;
  const data::TimelineFrame<data::DualArmFrame> & sourceFrame() const;
  const InputStatus & status() const noexcept;
  std::size_t sourceIndex() const noexcept;
  std::size_t consumedFrameCount() const noexcept;
  std::size_t droppedFrameCount() const noexcept;
  std::size_t deadlineMissCount() const noexcept;
  bool paused() const noexcept;
  bool stopped() const noexcept;
  bool endOfStream() const noexcept;

  void applyControl(SourceControl control);
  ReplayAdvance advance(std::int64_t tick_period_ns);
  ReplayAdvance advanceSequential();
  data::ReplayClock::TimePoint scheduledTime() const;
  void waitForCurrentFrame();
  void markFrameProcessed();

private:
  void updateFrame();
  void rebaseClock();

  const LoadedReplay & loaded_;
  data::ReplayClock clock_;
  data::ReplayClock::TimePoint clock_origin_;
  MotionTargetFrame frame_;
  InputStatus status_;
  std::size_t source_index_{0};
  std::size_t consumed_frame_count_{1};
  std::size_t dropped_frame_count_{0};
  std::size_t deadline_miss_count_{0};
  std::int64_t timeline_time_ns_{0};
  bool single_step_requested_{false};
  bool stopped_{false};
  bool end_of_stream_{false};
};

}  // namespace motion_control_lab::replay
