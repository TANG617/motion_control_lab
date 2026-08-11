#include "adapters/data/temporal/timestamp_policy.hpp"

namespace motion_control_lab::data
{

TimestampPolicy parseTimestampPolicy(const std::string & value)
{
  if (value == "preserve") {
    return TimestampPolicy::Preserve;
  }
  if (value == "fixed-period" || value == "fixed_period") {
    return TimestampPolicy::FixedPeriod;
  }
  throw DataError(DataErrorCode::InvalidArgument, "unknown timestamp policy: " + value);
}

std::string toString(TimestampPolicy policy)
{
  switch (policy) {
    case TimestampPolicy::Preserve:
      return "preserve";
    case TimestampPolicy::FixedPeriod:
      return "fixed_period";
  }
  throw DataError(DataErrorCode::InvalidArgument, "invalid timestamp policy enum");
}

}  // namespace motion_control_lab::data
