#include "adapters/data/source/data_source.hpp"

namespace motion_control_lab::data
{

const std::string & RowRecord::at(const std::string & column) const
{
  for (const auto & descriptor : stream.columns) {
    if (descriptor.name == column) {
      if (descriptor.index >= values.size()) {
        throw DataError(
                DataErrorCode::InvalidFormat,
                "CSV row is missing column value: " + column);
      }
      return values[descriptor.index];
    }
  }
  throw DataError(DataErrorCode::InvalidArgument, "CSV column does not exist: " + column);
}

bool RowRecord::has(const std::string & column) const
{
  for (const auto & descriptor : stream.columns) {
    if (descriptor.name == column) {
      return descriptor.index < values.size();
    }
  }
  return false;
}

}  // namespace motion_control_lab::data
