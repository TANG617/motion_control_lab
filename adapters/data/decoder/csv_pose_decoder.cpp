#include "adapters/data/decoder/csv_pose_decoder.hpp"

#include "adapters/data/decoder/decoder_utils.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <set>

namespace motion_control_lab::data
{
namespace
{
bool hasColumns(const StreamDescriptor & stream, const std::set<std::string> & required)
{
  std::set<std::string> available;
  for (const auto & column : stream.columns) {
    available.insert(column.name);
  }
  for (const auto & column : required) {
    if (available.count(column) == 0) {
      return false;
    }
  }
  return true;
}
}  // namespace

CsvPoseDecoder::CsvPoseDecoder(CsvPoseMapping mapping)
  : mapping_(std::move(mapping))
{
  if (mapping_.decoder_id.empty() || mapping_.timestamp_column.empty()) {
    throw DataError(DataErrorCode::InvalidArgument, "CSV pose mapping is incomplete");
  }
  if ((!mapping_.frame_id_column.has_value() || mapping_.frame_id_column->empty()) &&
      mapping_.fixed_frame_id.empty()) {
    throw DataError(
            DataErrorCode::InvalidArgument,
            "CSV pose mapping requires frame_id_column or fixed_frame_id");
  }
}

const std::string & CsvPoseDecoder::id() const
{
  return mapping_.decoder_id;
}

bool CsvPoseDecoder::supports(const StreamDescriptor & stream) const
{
  std::set<std::string> required{
    mapping_.timestamp_column,
    mapping_.x_column,
    mapping_.y_column,
    mapping_.z_column,
    mapping_.qx_column,
    mapping_.qy_column,
    mapping_.qz_column,
    mapping_.qw_column};
  if (mapping_.frame_id_column.has_value()) {
    required.insert(*mapping_.frame_id_column);
  }
  if (mapping_.header_stamp_column.has_value()) {
    required.insert(*mapping_.header_stamp_column);
  }
  if (mapping_.log_time_column.has_value()) {
    required.insert(*mapping_.log_time_column);
  }
  if (mapping_.publish_time_column.has_value()) {
    required.insert(*mapping_.publish_time_column);
  }
  return stream.format == PhysicalFormat::Csv && hasColumns(stream, required);
}

DecodedSample<CsvPoseDecoder::Sample> CsvPoseDecoder::decode(const EncodedRecord & record) const
{
  const auto * row = std::get_if<RowRecord>(&record);
  if (row == nullptr || !supports(row->stream)) {
    throw DataError(DataErrorCode::SchemaMismatch, "CSV row does not satisfy pose mapping");
  }

  StampedPose result;
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
  result.frame_id = mapping_.frame_id_column.has_value()
    ? row->at(*mapping_.frame_id_column)
    : mapping_.fixed_frame_id;
  const double x = parseFiniteDouble(row->at(mapping_.x_column), mapping_.x_column);
  const double y = parseFiniteDouble(row->at(mapping_.y_column), mapping_.y_column);
  const double z = parseFiniteDouble(row->at(mapping_.z_column), mapping_.z_column);
  const double qx = parseFiniteDouble(row->at(mapping_.qx_column), mapping_.qx_column);
  const double qy = parseFiniteDouble(row->at(mapping_.qy_column), mapping_.qy_column);
  const double qz = parseFiniteDouble(row->at(mapping_.qz_column), mapping_.qz_column);
  const double qw = parseFiniteDouble(row->at(mapping_.qw_column), mapping_.qw_column);

  Eigen::Quaterniond quaternion(qw, qx, qy, qz);
  const double norm = quaternion.norm();
  if (!std::isfinite(norm) || norm <= 1.0e-12) {
    throw DataError(DataErrorCode::DecodeFailure, "CSV pose quaternion is zero or invalid");
  }
  DecodedSample<Sample> decoded;
  if (std::abs(norm - 1.0) > 1.0e-12) {
    quaternion.normalize();
    decoded.diagnostics.push_back(
      {DiagnosticSeverity::Warning, "quaternion_normalized",
        "CSV pose quaternion norm was " + std::to_string(norm) +
        " and was explicitly normalized", row->row_number});
  }
  result.pose = Eigen::Isometry3d::Identity();
  result.pose.translation() = Eigen::Vector3d{x, y, z};
  result.pose.linear() = quaternion.toRotationMatrix();
  decoded.sample = std::move(result);
  return decoded;
}

}  // namespace motion_control_lab::data
