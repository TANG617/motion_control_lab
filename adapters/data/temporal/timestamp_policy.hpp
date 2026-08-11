#pragma once

#include "contracts/data/data_error.hpp"

#include <cstdint>
#include <string>

namespace motion_control_lab::data
{

enum class TimestampPolicy
{
  Preserve,
  FixedPeriod
};

struct TimestampProjectionConfig
{
  TimestampPolicy policy{TimestampPolicy::Preserve};
  std::int64_t period_ns{10'000'000};
};

TimestampPolicy parseTimestampPolicy(const std::string & value);
std::string toString(TimestampPolicy policy);

}  // namespace motion_control_lab::data
