#pragma once

#include "adapters/data/temporal/timestamp_policy.hpp"

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace motion_control_lab::data
{

template<typename Frame>
struct TimelineFrame
{
  std::uint64_t sequence{};
  std::int64_t original_logical_time_ns{};
  std::int64_t source_time_from_start_ns{};
  std::int64_t projected_time_ns{};
  TimestampPolicy projection_policy{TimestampPolicy::Preserve};
  Frame value;
};

template<typename Frame>
class Timeline
{
public:
  using Container = std::vector<TimelineFrame<Frame>>;
  using const_iterator = typename Container::const_iterator;

  Timeline() = default;

  explicit Timeline(Container frames)
    : frames_(makeValidated(std::move(frames)))
  {
  }

  std::size_t size() const noexcept { return frames_.size(); }
  bool empty() const noexcept { return frames_.empty(); }
  const TimelineFrame<Frame> & at(std::size_t index) const { return frames_.at(index); }
  const_iterator begin() const noexcept { return frames_.begin(); }
  const_iterator end() const noexcept { return frames_.end(); }

private:
  static Container makeValidated(Container frames)
  {
    for (std::size_t index = 0; index < frames.size(); ++index) {
      const auto & frame = frames[index];
      if (frame.sequence != index || frame.original_logical_time_ns < 0 ||
          frame.source_time_from_start_ns < 0 || frame.projected_time_ns < 0) {
        throw DataError(
                DataErrorCode::InvalidTimestamp,
                "Timeline frame sequence or timestamp is invalid");
      }
      if (index > 0 &&
          (frame.original_logical_time_ns <= frames[index - 1].original_logical_time_ns ||
          frame.source_time_from_start_ns <= frames[index - 1].source_time_from_start_ns ||
          frame.projected_time_ns <= frames[index - 1].projected_time_ns)) {
        throw DataError(
                DataErrorCode::NonMonotonicTimestamp,
                "Timeline frames must be strictly sorted by source and projected time");
      }
    }
    return frames;
  }

  Container frames_;
};

template<typename Frame>
struct UnprojectedFrame
{
  std::int64_t original_logical_time_ns{};
  Frame value;
};

}  // namespace motion_control_lab::data
