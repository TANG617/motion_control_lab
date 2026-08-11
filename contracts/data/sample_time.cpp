#include "contracts/data/sample_time.hpp"

#include <limits>

namespace motion_control_lab::data
{

TimestampSource parseTimestampSource(const std::string & value)
{
  if (value == "header_stamp") {
    return TimestampSource::HeaderStamp;
  }
  if (value == "log_time") {
    return TimestampSource::LogTime;
  }
  if (value == "publish_time") {
    return TimestampSource::PublishTime;
  }
  if (value == "csv_timestamp" || value == "configured_column") {
    return TimestampSource::ConfiguredColumn;
  }
  throw DataError(
          DataErrorCode::InvalidArgument,
          "unknown timestamp source: " + value);
}

std::string toString(TimestampSource source)
{
  switch (source) {
    case TimestampSource::HeaderStamp:
      return "header_stamp";
    case TimestampSource::LogTime:
      return "log_time";
    case TimestampSource::PublishTime:
      return "publish_time";
    case TimestampSource::ConfiguredColumn:
      return "csv_timestamp";
  }
  throw DataError(DataErrorCode::InvalidArgument, "invalid timestamp source enum");
}

std::int64_t selectTimestamp(const SampleTime & time, TimestampSource source)
{
  const std::optional<std::int64_t> * selected = nullptr;
  switch (source) {
    case TimestampSource::HeaderStamp:
      selected = &time.header_stamp_ns;
      break;
    case TimestampSource::LogTime:
      selected = &time.log_time_ns;
      break;
    case TimestampSource::PublishTime:
      selected = &time.publish_time_ns;
      break;
    case TimestampSource::ConfiguredColumn:
      selected = &time.configured_time_ns;
      break;
  }
  if (selected == nullptr || !selected->has_value()) {
    throw DataError(
            DataErrorCode::MissingTimestamp,
            "selected timestamp is absent: " + toString(source));
  }
  if (selected->value() < 0) {
    throw DataError(
            DataErrorCode::InvalidTimestamp,
            "selected timestamp is outside the supported non-negative nanosecond range");
  }
  return selected->value();
}

}  // namespace motion_control_lab::data
