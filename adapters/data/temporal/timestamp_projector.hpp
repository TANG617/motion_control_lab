#pragma once

#include "adapters/data/temporal/timeline.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace motion_control_lab::data
{

template<typename Frame>
Timeline<Frame> projectTimestamps(
  std::vector<UnprojectedFrame<Frame>> frames,
  const TimestampProjectionConfig & config)
{
  if (config.policy == TimestampPolicy::FixedPeriod && config.period_ns <= 0) {
    throw DataError(DataErrorCode::InvalidArgument, "fixed-period projection requires period_ns > 0");
  }
  typename Timeline<Frame>::Container result;
  result.reserve(frames.size());
  if (frames.empty()) {
    return Timeline<Frame>{std::move(result)};
  }
  const auto first = frames.front().original_logical_time_ns;
  std::int64_t previous = -1;
  for (std::size_t index = 0; index < frames.size(); ++index) {
    const auto original = frames[index].original_logical_time_ns;
    if (original < 0 || (index > 0 && original <= frames[index - 1].original_logical_time_ns)) {
      throw DataError(
              DataErrorCode::NonMonotonicTimestamp,
              "semantic frame timestamps must be strictly increasing before projection");
    }
    const auto source_from_start = original - first;
    std::int64_t projected = source_from_start;
    if (config.policy == TimestampPolicy::FixedPeriod) {
      if (index > static_cast<std::size_t>(
          std::numeric_limits<std::int64_t>::max() / config.period_ns)) {
        throw DataError(DataErrorCode::InvalidTimestamp, "fixed-period projection overflows int64");
      }
      projected = static_cast<std::int64_t>(index) * config.period_ns;
    }
    if (previous >= 0 && projected <= previous) {
      throw DataError(DataErrorCode::InvalidTimestamp, "projected timeline is not strictly increasing");
    }
    previous = projected;
    result.push_back(
      {static_cast<std::uint64_t>(index), original, source_from_start, projected,
        config.policy, std::move(frames[index].value)});
  }
  return Timeline<Frame>{std::move(result)};
}

}  // namespace motion_control_lab::data
