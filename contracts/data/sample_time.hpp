#pragma once

#include "contracts/data/data_error.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace motion_control_lab::data
{

struct SampleTime
{
  std::optional<std::int64_t> header_stamp_ns;
  std::optional<std::int64_t> log_time_ns;
  std::optional<std::int64_t> publish_time_ns;
  std::optional<std::int64_t> configured_time_ns;
};

enum class TimestampSource
{
  HeaderStamp,
  LogTime,
  PublishTime,
  ConfiguredColumn
};

TimestampSource parseTimestampSource(const std::string & value);
std::string toString(TimestampSource source);
std::int64_t selectTimestamp(const SampleTime & time, TimestampSource source);

}  // namespace motion_control_lab::data
