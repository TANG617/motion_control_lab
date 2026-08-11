#pragma once

#include "contracts/data/data_error.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

namespace motion_control_lab::data
{

enum class ExecutionMode
{
  Batch,
  Realtime
};

ExecutionMode parseExecutionMode(const std::string & value);
std::string toString(ExecutionMode mode);

class ReplayClock
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  using NowFunction = std::function<TimePoint()>;
  using SleepUntilFunction = std::function<void(TimePoint)>;

  explicit ReplayClock(
    ExecutionMode mode,
    double playback_rate,
    NowFunction now = [] {return Clock::now();},
    SleepUntilFunction sleep_until = [](TimePoint deadline) {
        std::this_thread::sleep_until(deadline);
      });

  TimePoint now() const;
  TimePoint deadline(TimePoint run_start, std::int64_t projected_time_ns) const;
  void waitUntil(TimePoint deadline) const;
  ExecutionMode mode() const noexcept { return mode_; }
  double playbackRate() const noexcept { return playback_rate_; }

private:
  ExecutionMode mode_;
  double playback_rate_;
  NowFunction now_;
  SleepUntilFunction sleep_until_;
};

}  // namespace motion_control_lab::data
