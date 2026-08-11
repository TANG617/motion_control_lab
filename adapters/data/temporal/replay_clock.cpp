#include "adapters/data/temporal/replay_clock.hpp"

#include <cmath>
#include <limits>
#include <thread>

namespace motion_control_lab::data
{

ExecutionMode parseExecutionMode(const std::string & value)
{
  if (value == "batch") {
    return ExecutionMode::Batch;
  }
  if (value == "realtime") {
    return ExecutionMode::Realtime;
  }
  throw DataError(DataErrorCode::InvalidArgument, "unknown execution mode: " + value);
}

std::string toString(ExecutionMode mode)
{
  return mode == ExecutionMode::Batch ? "batch" : "realtime";
}

ReplayClock::ReplayClock(
  ExecutionMode mode,
  double playback_rate,
  NowFunction now,
  SleepUntilFunction sleep_until)
  : mode_(mode),
    playback_rate_(playback_rate),
    now_(std::move(now)),
    sleep_until_(std::move(sleep_until))
{
  if (!std::isfinite(playback_rate_) || playback_rate_ <= 0.0 || !now_ || !sleep_until_) {
    throw DataError(DataErrorCode::InvalidArgument, "ReplayClock requires playback_rate > 0");
  }
}

ReplayClock::TimePoint ReplayClock::now() const
{
  return now_();
}

ReplayClock::TimePoint ReplayClock::deadline(
  TimePoint run_start,
  std::int64_t projected_time_ns) const
{
  if (projected_time_ns < 0) {
    throw DataError(DataErrorCode::InvalidTimestamp, "projected timestamp must be non-negative");
  }
  const long double scaled =
    static_cast<long double>(projected_time_ns) / playback_rate_;
  if (scaled > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
    throw DataError(DataErrorCode::InvalidTimestamp, "replay deadline overflows nanoseconds");
  }
  return run_start + std::chrono::nanoseconds{
    static_cast<std::int64_t>(std::llround(scaled))};
}

void ReplayClock::waitUntil(TimePoint deadline_value) const
{
  if (mode_ == ExecutionMode::Realtime && now_() < deadline_value) {
    sleep_until_(deadline_value);
  }
}

}  // namespace motion_control_lab::data
