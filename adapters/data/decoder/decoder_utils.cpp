#include "adapters/data/decoder/decoder_utils.hpp"

#include <cmath>
#include <limits>

namespace motion_control_lab::data
{

std::int64_t checkedTimestamp(std::uint64_t value, const std::string & field)
{
  if (value > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
    throw DataError(DataErrorCode::InvalidTimestamp, field + " exceeds int64 nanoseconds");
  }
  return static_cast<std::int64_t>(value);
}

std::int64_t combineRosTime(std::int32_t seconds, std::uint32_t nanoseconds)
{
  if (seconds < 0 || nanoseconds >= 1'000'000'000U) {
    throw DataError(DataErrorCode::InvalidTimestamp, "ROS header stamp is invalid");
  }
  constexpr std::int64_t billion = 1'000'000'000LL;
  return static_cast<std::int64_t>(seconds) * billion + nanoseconds;
}

SampleTime binarySampleTime(const BinaryRecord & record, std::int64_t header_stamp_ns)
{
  SampleTime result;
  result.header_stamp_ns = header_stamp_ns;
  result.log_time_ns = checkedTimestamp(record.log_time_ns, "MCAP log_time");
  result.publish_time_ns = checkedTimestamp(record.publish_time_ns, "MCAP publish_time");
  return result;
}

void setConfiguredTimestamp(
  SampleTime & time,
  TimestampSource target,
  std::int64_t timestamp_ns)
{
  if (timestamp_ns < 0) {
    throw DataError(DataErrorCode::InvalidTimestamp, "CSV timestamp must be non-negative");
  }
  switch (target) {
    case TimestampSource::HeaderStamp:
      time.header_stamp_ns = timestamp_ns;
      break;
    case TimestampSource::LogTime:
      time.log_time_ns = timestamp_ns;
      break;
    case TimestampSource::PublishTime:
      time.publish_time_ns = timestamp_ns;
      break;
    case TimestampSource::ConfiguredColumn:
      time.configured_time_ns = timestamp_ns;
      break;
  }
}

std::int64_t parseInt64(const std::string & value, const std::string & field)
{
  if (value.empty()) {
    throw DataError(DataErrorCode::DecodeFailure, "CSV field is empty: " + field);
  }
  std::size_t parsed = 0;
  std::int64_t result = 0;
  try {
    result = std::stoll(value, &parsed, 10);
  } catch (const std::exception &) {
    throw DataError(DataErrorCode::DecodeFailure, "invalid int64 CSV field: " + field);
  }
  if (parsed != value.size()) {
    throw DataError(DataErrorCode::DecodeFailure, "invalid int64 CSV field: " + field);
  }
  return result;
}

double parseFiniteDouble(const std::string & value, const std::string & field)
{
  if (value.empty()) {
    throw DataError(DataErrorCode::DecodeFailure, "CSV field is empty: " + field);
  }
  std::size_t parsed = 0;
  double result = 0.0;
  try {
    result = std::stod(value, &parsed);
  } catch (const std::exception &) {
    throw DataError(DataErrorCode::DecodeFailure, "invalid floating CSV field: " + field);
  }
  if (parsed != value.size() || !std::isfinite(result)) {
    throw DataError(DataErrorCode::DecodeFailure, "non-finite or invalid CSV field: " + field);
  }
  return result;
}

std::vector<std::string> splitNonEmpty(const std::string & value, char delimiter)
{
  std::vector<std::string> result;
  if (value.empty()) {
    return result;
  }
  std::size_t begin = 0;
  while (begin <= value.size()) {
    const auto end = value.find(delimiter, begin);
    const auto item = value.substr(begin, end == std::string::npos ? end : end - begin);
    if (item.empty()) {
      throw DataError(DataErrorCode::DecodeFailure, "CSV vector contains an empty item");
    }
    result.push_back(item);
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1;
  }
  return result;
}

}  // namespace motion_control_lab::data
