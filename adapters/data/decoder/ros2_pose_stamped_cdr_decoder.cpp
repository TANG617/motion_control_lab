#include "adapters/data/decoder/ros2_pose_stamped_cdr_decoder.hpp"

#include "adapters/data/decoder/cdr_reader.hpp"
#include "adapters/data/decoder/decoder_utils.hpp"

#include <Eigen/Geometry>

#include <cmath>

namespace motion_control_lab::data
{
namespace
{
const std::string kDecoderId = "ros2_cdr.geometry_msgs.msg.PoseStamped";
constexpr const char * kSchemaName = "geometry_msgs/msg/PoseStamped";

const BinaryRecord & requireRecord(const EncodedRecord & record)
{
  const auto * binary = std::get_if<BinaryRecord>(&record);
  if (binary == nullptr) {
    throw DataError(DataErrorCode::UnsupportedEncoding, "PoseStamped CDR decoder requires binary input");
  }
  if (binary->stream.schema_name != kSchemaName ||
      binary->stream.schema_encoding != "ros2msg" ||
      binary->stream.message_encoding != "cdr") {
    throw DataError(
            DataErrorCode::SchemaMismatch,
            "PoseStamped decoder requires geometry_msgs/msg/PoseStamped, ros2msg, cdr");
  }
  if (binary->stream.schema_data.empty()) {
    throw DataError(DataErrorCode::SchemaMismatch, "PoseStamped embedded schema is empty");
  }
  return *binary;
}
}  // namespace

const std::string & Ros2PoseStampedCdrDecoder::id() const
{
  return kDecoderId;
}

bool Ros2PoseStampedCdrDecoder::supports(const StreamDescriptor & stream) const
{
  return stream.format == PhysicalFormat::Mcap &&
         stream.schema_name == kSchemaName &&
         stream.schema_encoding == "ros2msg" &&
         stream.message_encoding == "cdr";
}

DecodedSample<Ros2PoseStampedCdrDecoder::Sample> Ros2PoseStampedCdrDecoder::decode(
  const EncodedRecord & record) const
{
  const auto & binary = requireRecord(record);
  CdrReader reader(binary.payload);
  const auto seconds = reader.readInt32();
  const auto nanoseconds = reader.readUint32();

  StampedPose result;
  result.time = binarySampleTime(binary, combineRosTime(seconds, nanoseconds));
  result.frame_id = reader.readString();
  const double x = reader.readDouble();
  const double y = reader.readDouble();
  const double z = reader.readDouble();
  const double qx = reader.readDouble();
  const double qy = reader.readDouble();
  const double qz = reader.readDouble();
  const double qw = reader.readDouble();
  reader.requireFinished();

  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      !std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz) ||
      !std::isfinite(qw)) {
    throw DataError(DataErrorCode::DecodeFailure, "PoseStamped contains a non-finite value");
  }
  Eigen::Quaterniond quaternion(qw, qx, qy, qz);
  const double norm = quaternion.norm();
  if (!std::isfinite(norm) || norm <= 1.0e-12) {
    throw DataError(DataErrorCode::DecodeFailure, "PoseStamped quaternion is zero or invalid");
  }

  DecodedSample<Sample> decoded;
  if (std::abs(norm - 1.0) > 1.0e-12) {
    quaternion.normalize();
    decoded.diagnostics.push_back(
      {DiagnosticSeverity::Warning, "quaternion_normalized",
        "PoseStamped quaternion norm was " + std::to_string(norm) +
        " and was explicitly normalized", std::nullopt});
  }
  result.pose = Eigen::Isometry3d::Identity();
  result.pose.translation() = Eigen::Vector3d{x, y, z};
  result.pose.linear() = quaternion.toRotationMatrix();
  decoded.sample = std::move(result);
  return decoded;
}

}  // namespace motion_control_lab::data
