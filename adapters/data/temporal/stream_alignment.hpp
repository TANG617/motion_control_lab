#pragma once

#include "contracts/data/sample_time.hpp"
#include "contracts/data/typed_stream.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace motion_control_lab::data
{

template<typename T>
struct LogicalSample
{
  std::int64_t logical_time_ns{};
  T sample;
};

template<typename T>
std::vector<LogicalSample<T>> validateAndOrderStream(
  const TypedStream<T> & stream,
  TimestampSource timestamp_source)
{
  std::vector<LogicalSample<T>> result;
  result.reserve(stream.samples.size());
  for (std::size_t index = 0; index < stream.samples.size(); ++index) {
    std::int64_t timestamp = 0;
    try {
      timestamp = selectTimestamp(stream.samples[index].time, timestamp_source);
    } catch (const DataError & error) {
      throw DataError(
              error.code(),
              "stream " + stream.logical_name + " sample " + std::to_string(index) +
              ": " + error.what());
    }
    if (!result.empty()) {
      if (timestamp == result.back().logical_time_ns) {
        throw DataError(
                DataErrorCode::DuplicateTimestamp,
                "stream " + stream.logical_name + " has duplicate logical timestamp " +
                std::to_string(timestamp));
      }
      if (timestamp < result.back().logical_time_ns) {
        throw DataError(
                DataErrorCode::NonMonotonicTimestamp,
                "stream " + stream.logical_name + " is non-monotonic at sample " +
                std::to_string(index));
      }
    }
    result.push_back({timestamp, stream.samples[index]});
  }
  return result;
}

}  // namespace motion_control_lab::data
