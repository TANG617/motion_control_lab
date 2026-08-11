#include "adapters/data/decoder/ros2_joint_state_cdr_decoder.hpp"

#include "adapters/data/decoder/cdr_reader.hpp"
#include "adapters/data/decoder/decoder_utils.hpp"

#include <cmath>

namespace motion_control_lab::data
{
namespace
{
const std::string kDecoderId = "ros2_cdr.sensor_msgs.msg.JointState";
constexpr const char * kSchemaName = "sensor_msgs/msg/JointState";

void requireFinite(const std::vector<double> & values, const std::string & field)
{
  for (const double value : values) {
    if (!std::isfinite(value)) {
      throw DataError(DataErrorCode::DecodeFailure, "JointState " + field + " contains non-finite data");
    }
  }
}
}  // namespace

const std::string & Ros2JointStateCdrDecoder::id() const
{
  return kDecoderId;
}

std::type_index Ros2JointStateCdrDecoder::outputType() const
{
  return std::type_index(typeid(StampedJointState));
}

bool Ros2JointStateCdrDecoder::supports(const StreamDescriptor & stream) const
{
  return stream.format == PhysicalFormat::Mcap &&
         stream.schema_name == kSchemaName &&
         stream.schema_encoding == "ros2msg" &&
         stream.message_encoding == "cdr";
}

AnyDecodedSample Ros2JointStateCdrDecoder::decode(const EncodedRecord & record) const
{
  const auto * binary = std::get_if<BinaryRecord>(&record);
  if (binary == nullptr) {
    throw DataError(DataErrorCode::UnsupportedEncoding, "JointState CDR decoder requires binary input");
  }
  if (!supports(binary->stream)) {
    throw DataError(
            DataErrorCode::SchemaMismatch,
            "JointState decoder requires sensor_msgs/msg/JointState, ros2msg, cdr");
  }
  if (binary->stream.schema_data.empty()) {
    throw DataError(DataErrorCode::SchemaMismatch, "JointState embedded schema is empty");
  }

  CdrReader reader(binary->payload);
  const auto seconds = reader.readInt32();
  const auto nanoseconds = reader.readUint32();
  (void)reader.readString();  // Header frame_id is not part of the JointState sample contract.

  StampedJointState result;
  result.time = binarySampleTime(*binary, combineRosTime(seconds, nanoseconds));
  result.names = reader.readStringSequence();
  result.positions = reader.readDoubleSequence();
  result.velocities = reader.readDoubleSequence();
  const auto efforts = reader.readDoubleSequence();
  reader.requireFinished();

  requireFinite(result.positions, "positions");
  requireFinite(result.velocities, "velocities");
  requireFinite(efforts, "efforts");
  return AnyDecodedSample{std::move(result), {}};
}

}  // namespace motion_control_lab::data
