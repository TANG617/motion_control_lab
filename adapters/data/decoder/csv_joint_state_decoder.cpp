#include "adapters/data/decoder/csv_joint_state_decoder.hpp"

#include "adapters/data/decoder/decoder_utils.hpp"

#include <set>

namespace motion_control_lab::data
{
namespace
{
bool contains(const StreamDescriptor & stream, const std::string & name)
{
  for (const auto & column : stream.columns) {
    if (column.name == name) {
      return true;
    }
  }
  return false;
}

std::vector<double> parseVector(
  const std::string & value,
  char delimiter,
  const std::string & field)
{
  std::vector<double> result;
  for (const auto & item : splitNonEmpty(value, delimiter)) {
    result.push_back(parseFiniteDouble(item, field));
  }
  return result;
}
}  // namespace

CsvJointStateDecoder::CsvJointStateDecoder(CsvJointStateMapping mapping)
  : mapping_(std::move(mapping))
{
  if (mapping_.decoder_id.empty() || mapping_.timestamp_column.empty() ||
      mapping_.names_column.empty() || mapping_.positions_column.empty() ||
      mapping_.vector_delimiter == '\0') {
    throw DataError(DataErrorCode::InvalidArgument, "CSV joint-state mapping is incomplete");
  }
}

const std::string & CsvJointStateDecoder::id() const
{
  return mapping_.decoder_id;
}

std::type_index CsvJointStateDecoder::outputType() const
{
  return std::type_index(typeid(StampedJointState));
}

bool CsvJointStateDecoder::supports(const StreamDescriptor & stream) const
{
  return stream.format == PhysicalFormat::Csv &&
         contains(stream, mapping_.timestamp_column) &&
         contains(stream, mapping_.names_column) &&
         contains(stream, mapping_.positions_column) &&
         (!mapping_.header_stamp_column.has_value() ||
         contains(stream, *mapping_.header_stamp_column)) &&
         (!mapping_.log_time_column.has_value() ||
         contains(stream, *mapping_.log_time_column)) &&
         (!mapping_.publish_time_column.has_value() ||
         contains(stream, *mapping_.publish_time_column)) &&
         (!mapping_.velocities_column.has_value() ||
         contains(stream, *mapping_.velocities_column));
}

AnyDecodedSample CsvJointStateDecoder::decode(const EncodedRecord & record) const
{
  const auto * row = std::get_if<RowRecord>(&record);
  if (row == nullptr || !supports(row->stream)) {
    throw DataError(DataErrorCode::SchemaMismatch, "CSV row does not satisfy joint-state mapping");
  }
  StampedJointState result;
  setConfiguredTimestamp(
    result.time,
    mapping_.timestamp_target,
    parseInt64(row->at(mapping_.timestamp_column), mapping_.timestamp_column));
  if (mapping_.header_stamp_column.has_value()) {
    result.time.header_stamp_ns = parseInt64(
      row->at(*mapping_.header_stamp_column), *mapping_.header_stamp_column);
  }
  if (mapping_.log_time_column.has_value()) {
    result.time.log_time_ns = parseInt64(
      row->at(*mapping_.log_time_column), *mapping_.log_time_column);
  }
  if (mapping_.publish_time_column.has_value()) {
    result.time.publish_time_ns = parseInt64(
      row->at(*mapping_.publish_time_column), *mapping_.publish_time_column);
  }
  result.names = splitNonEmpty(row->at(mapping_.names_column), mapping_.vector_delimiter);
  result.positions = parseVector(
    row->at(mapping_.positions_column), mapping_.vector_delimiter, mapping_.positions_column);
  if (mapping_.velocities_column.has_value()) {
    result.velocities = parseVector(
      row->at(*mapping_.velocities_column),
      mapping_.vector_delimiter,
      *mapping_.velocities_column);
  }
  if (result.names.size() != result.positions.size()) {
    throw DataError(
            DataErrorCode::DecodeFailure,
            "CSV JointState names and positions have different lengths");
  }
  if (!result.velocities.empty() && result.velocities.size() != result.names.size()) {
    throw DataError(
            DataErrorCode::DecodeFailure,
            "CSV JointState velocities must be empty or match names length");
  }
  return AnyDecodedSample{std::move(result), {}};
}

}  // namespace motion_control_lab::data
