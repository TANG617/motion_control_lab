#pragma once

#include "adapters/data/source/data_source.hpp"
#include "contracts/data/sample_time.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace motion_control_lab::data
{

std::int64_t checkedTimestamp(std::uint64_t value, const std::string & field);
std::int64_t combineRosTime(std::int32_t seconds, std::uint32_t nanoseconds);
SampleTime binarySampleTime(const BinaryRecord & record, std::int64_t header_stamp_ns);
void setConfiguredTimestamp(
  SampleTime & time,
  TimestampSource target,
  std::int64_t timestamp_ns);
std::int64_t parseInt64(const std::string & value, const std::string & field);
double parseFiniteDouble(const std::string & value, const std::string & field);
std::vector<std::string> splitNonEmpty(const std::string & value, char delimiter);

}  // namespace motion_control_lab::data
