#include "adapters/data/decoder/csv_joint_state_decoder.hpp"
#include "adapters/data/decoder/csv_pose_decoder.hpp"
#include "adapters/data/decoder/decoder_registry.hpp"
#include "adapters/data/decoder/ros2_joint_state_cdr_decoder.hpp"
#include "adapters/data/decoder/ros2_pose_stamped_cdr_decoder.hpp"
#include "adapters/data/projection/dual_arm_timeline.hpp"
#include "adapters/data/source/csv_source.hpp"
#include "adapters/data/source/mcap_source.hpp"
#include "adapters/data/temporal/replay_clock.hpp"

#include <mcap/writer.hpp>

#include <Eigen/Geometry>

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace data = motion_control_lab::data;

namespace
{

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    path_ = std::filesystem::temp_directory_path() /
      ("mcl-data-test-" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(path_);
  }

  ~TemporaryDirectory()
  {
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
  }

  const std::filesystem::path & path() const { return path_; }

private:
  std::filesystem::path path_;
};

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

template<typename Function>
void requireError(data::DataErrorCode code, Function function, const std::string & message)
{
  try {
    function();
  } catch (const data::DataError & error) {
    require(error.code() == code, message + " (wrong DataErrorCode)");
    return;
  }
  throw std::runtime_error(message + " (did not throw)");
}

class CdrWriter
{
public:
  explicit CdrWriter(bool little_endian)
    : little_endian_(little_endian)
  {
    bytes_ = {std::byte{0}, little_endian ? std::byte{1} : std::byte{0},
      std::byte{0}, std::byte{0}};
  }

  void int32(std::int32_t value) { uint32(static_cast<std::uint32_t>(value)); }

  void uint32(std::uint32_t value)
  {
    align(4);
    appendInteger(value, 4);
  }

  void real64(double value)
  {
    align(8);
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    appendInteger(bits, 8);
  }

  void string(const std::string & value)
  {
    uint32(static_cast<std::uint32_t>(value.size() + 1));
    for (const char character : value) {
      bytes_.push_back(static_cast<std::byte>(character));
    }
    bytes_.push_back(std::byte{0});
  }

  void strings(const std::vector<std::string> & values)
  {
    uint32(static_cast<std::uint32_t>(values.size()));
    for (const auto & value : values) {
      string(value);
    }
  }

  void doubles(const std::vector<double> & values)
  {
    uint32(static_cast<std::uint32_t>(values.size()));
    for (const double value : values) {
      real64(value);
    }
  }

  std::vector<std::byte> finish() { return std::move(bytes_); }

private:
  void align(std::size_t alignment)
  {
    const auto relative = bytes_.size() - 4;
    const auto padding = (alignment - relative % alignment) % alignment;
    bytes_.insert(bytes_.end(), padding, std::byte{0});
  }

  void appendInteger(std::uint64_t value, std::size_t width)
  {
    for (std::size_t index = 0; index < width; ++index) {
      const auto shift = little_endian_ ? index * 8U : (width - index - 1U) * 8U;
      bytes_.push_back(static_cast<std::byte>((value >> shift) & 0xffU));
    }
  }

  bool little_endian_;
  std::vector<std::byte> bytes_;
};

std::vector<std::byte> poseCdr(
  bool little_endian,
  std::int64_t stamp_ns,
  const std::string & frame_id,
  const std::array<double, 7> & values)
{
  CdrWriter writer(little_endian);
  writer.int32(static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL));
  writer.uint32(static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL));
  writer.string(frame_id);
  for (const double value : values) {
    writer.real64(value);
  }
  return writer.finish();
}

std::vector<std::byte> jointStateCdr(
  bool little_endian,
  const std::vector<std::string> & names,
  const std::vector<double> & positions,
  const std::vector<double> & velocities)
{
  CdrWriter writer(little_endian);
  writer.int32(1);
  writer.uint32(23);
  writer.string("base");
  writer.strings(names);
  writer.doubles(positions);
  writer.doubles(velocities);
  writer.doubles({});
  return writer.finish();
}

data::StreamDescriptor poseDescriptor(const std::string & topic = "/pose")
{
  data::StreamDescriptor descriptor;
  descriptor.format = data::PhysicalFormat::Mcap;
  descriptor.logical_name = topic;
  descriptor.topic = topic;
  descriptor.schema_name = "geometry_msgs/msg/PoseStamped";
  descriptor.schema_encoding = "ros2msg";
  descriptor.message_encoding = "cdr";
  descriptor.schema_data = {std::byte{0x78}};
  return descriptor;
}

data::StreamDescriptor jointDescriptor()
{
  auto descriptor = poseDescriptor("/joints");
  descriptor.schema_name = "sensor_msgs/msg/JointState";
  return descriptor;
}

data::StampedPose decodePoseRecord(data::BinaryRecord record)
{
  data::Ros2PoseStampedCdrDecoder decoder;
  return std::any_cast<data::StampedPose>(decoder.decode(data::EncodedRecord{std::move(record)}).sample);
}

data::StampedJointState decodeJointRecord(data::BinaryRecord record)
{
  data::Ros2JointStateCdrDecoder decoder;
  return std::any_cast<data::StampedJointState>(decoder.decode(data::EncodedRecord{std::move(record)}).sample);
}

void writeSyntheticMcap(const std::filesystem::path & path)
{
  mcap::McapWriter writer;
  mcap::McapWriterOptions options("ros2");
  options.compression = mcap::Compression::Zstd;
  options.forceCompression = true;
  options.chunkSize = 128;
  const auto open_status = writer.open(path.string(), options);
  require(open_status.ok(), "failed to create synthetic MCAP");

  mcap::Schema pose_schema(
    "geometry_msgs/msg/PoseStamped", "ros2msg",
    "std_msgs/Header header\ngeometry_msgs/Pose pose\n");
  writer.addSchema(pose_schema);
  mcap::Schema joint_schema(
    "sensor_msgs/msg/JointState", "ros2msg",
    "std_msgs/Header header\nstring[] name\nfloat64[] position\n");
  writer.addSchema(joint_schema);
  mcap::Channel pose_channel("/target", "cdr", pose_schema.id);
  writer.addChannel(pose_channel);
  mcap::Channel unrelated_channel("/unrelated", "cdr", joint_schema.id);
  writer.addChannel(unrelated_channel);

  const std::array<std::int64_t, 3> stamps{0, 9'500'000, 20'300'000};
  for (std::size_t index = 0; index < stamps.size(); ++index) {
    const auto payload = poseCdr(
      true, stamps[index], "base_link",
      {double(index), 2.0, 3.0, 0.0, 0.0, 0.0, 1.0});
    mcap::Message message;
    message.channelId = pose_channel.id;
    message.sequence = static_cast<std::uint32_t>(index + 1);
    message.logTime = static_cast<std::uint64_t>(stamps[index] + 100);
    message.publishTime = static_cast<std::uint64_t>(stamps[index] + 50);
    message.data = payload.data();
    message.dataSize = payload.size();
    require(writer.write(message).ok(), "failed to write pose MCAP message");
  }
  const std::array<std::byte, 1> unrelated{std::byte{0xff}};
  mcap::Message message;
  message.channelId = unrelated_channel.id;
  message.sequence = 1;
  message.logTime = 777;
  message.publishTime = 777;
  message.data = unrelated.data();
  message.dataSize = unrelated.size();
  require(writer.write(message).ok(), "failed to write unrelated MCAP message");
  writer.close();
}

std::vector<data::StampedPose> decodeMcapPoses(const std::filesystem::path & path)
{
  data::McapSource source(path);
  require(source.catalog().streams.size() == 2, "MCAP catalog did not inspect both channels");
  require(source.catalog().metadata.at("chunk_count") != "0", "synthetic MCAP was not chunked");
  auto cursor = source.select({"/target", std::nullopt, std::nullopt});
  data::DecoderRegistry registry;
  registry.registerDecoder(std::make_shared<data::Ros2PoseStampedCdrDecoder>());
  auto poses = registry.decode<data::StampedPose>(*cursor, "/target");
  require(poses.samples.size() == 3, "MCAP topic filter returned unrelated messages");
  return poses.samples;
}

void testMcapCsvAndDecoders(const TemporaryDirectory & temporary)
{
  const auto mcap_path = temporary.path() / "synthetic.mcap";
  writeSyntheticMcap(mcap_path);
  const auto mcap_poses = decodeMcapPoses(mcap_path);

  const auto big = decodePoseRecord({
    poseDescriptor(), 1, 101, 100,
    poseCdr(false, 9'500'000, "odd", {1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0})});
  require(big.frame_id == "odd", "big-endian CDR string alignment failed");
  require(big.time.header_stamp_ns == 9'500'000, "big-endian CDR stamp failed");
  require(big.pose.translation().isApprox(Eigen::Vector3d{1.0, 2.0, 3.0}), "big-endian pose failed");

  const auto joint = decodeJointRecord({
    jointDescriptor(), 1, 200, 150,
    jointStateCdr(false, {"joint_a", "b", "long_joint_name"},
      {1.0, 2.0, 3.0}, {0.1, 0.2, 0.3})});
  require(joint.names.size() == 3 && joint.names[2] == "long_joint_name", "JointState names failed");
  require(joint.positions.size() == 3 && joint.velocities.size() == 3, "JointState vectors failed");

  const auto pose_csv = temporary.path() / "poses.csv";
  {
    std::ofstream output(pose_csv);
    output << "header_ns,log_ns,publish_ns,frame,x,y,z,qx,qy,qz,qw\n";
    for (std::size_t index = 0; index < mcap_poses.size(); ++index) {
      const auto & pose = mcap_poses[index];
      output << *pose.time.header_stamp_ns << ',' << *pose.time.log_time_ns << ','
             << *pose.time.publish_time_ns << ",base_link," << index
             << ",2,3,0,0,0,1\n";
    }
  }
  data::CsvSource csv_source(pose_csv, {"pose"});
  data::CsvPoseMapping mapping;
  mapping.decoder_id = "csv_pose_equivalence";
  mapping.timestamp_column = "header_ns";
  mapping.timestamp_target = data::TimestampSource::HeaderStamp;
  mapping.log_time_column = "log_ns";
  mapping.publish_time_column = "publish_ns";
  mapping.frame_id_column = "frame";
  data::DecoderRegistry csv_registry;
  csv_registry.registerDecoder(std::make_shared<data::CsvPoseDecoder>(mapping));
  auto csv_cursor = csv_source.select({"pose", std::nullopt, std::nullopt});
  const auto csv_poses = csv_registry.decode<data::StampedPose>(
    *csv_cursor, "pose", mapping.decoder_id);
  require(csv_poses.samples.size() == mcap_poses.size(), "MCAP/CSV pose count differs");
  for (std::size_t index = 0; index < mcap_poses.size(); ++index) {
    require(csv_poses.samples[index].time.header_stamp_ns == mcap_poses[index].time.header_stamp_ns,
      "MCAP/CSV header provenance differs");
    require(csv_poses.samples[index].time.log_time_ns == mcap_poses[index].time.log_time_ns,
      "MCAP/CSV log provenance differs");
    require(csv_poses.samples[index].time.publish_time_ns == mcap_poses[index].time.publish_time_ns,
      "MCAP/CSV publish provenance differs");
    require(csv_poses.samples[index].frame_id == mcap_poses[index].frame_id,
      "MCAP/CSV frame differs");
    require(csv_poses.samples[index].pose.matrix().isApprox(mcap_poses[index].pose.matrix()),
      "MCAP/CSV pose differs");
  }

  const auto joint_csv = temporary.path() / "joints.csv";
  {
    std::ofstream output(joint_csv);
    output << "timestamp_ns,names,positions,velocities\n"
           << "5,a;b;c,1;2;3,0.1;0.2;0.3\n";
  }
  data::CsvSource joint_source(joint_csv, {"joints"});
  data::CsvJointStateMapping joint_mapping;
  data::DecoderRegistry joint_registry;
  joint_registry.registerDecoder(std::make_shared<data::CsvJointStateDecoder>(joint_mapping));
  auto joint_cursor = joint_source.select({"joints", std::nullopt, std::nullopt});
  const auto joint_samples = joint_registry.decode<data::StampedJointState>(*joint_cursor, "joints");
  require(joint_samples.samples.front().names.size() == 3, "CSV JointState names failed");
  require(joint_samples.samples.front().velocities[2] == 0.3, "CSV JointState velocities failed");
}

data::StampedPose sample(std::optional<std::int64_t> stamp, const std::string & frame = "base")
{
  data::StampedPose result;
  result.time.header_stamp_ns = stamp;
  result.frame_id = frame;
  return result;
}

data::TypedStream<data::StampedPose> stream(
  const std::string & name,
  std::initializer_list<std::optional<std::int64_t>> stamps,
  const std::string & frame = "base")
{
  data::TypedStream<data::StampedPose> result;
  result.logical_name = name;
  for (const auto stamp : stamps) {
    result.samples.push_back(sample(stamp, frame));
  }
  return result;
}

void testTemporalProjection()
{
  const auto left = stream("left", {0, 9'500'000, 20'300'000});
  const auto right = stream("right", {0, 9'500'000, 20'300'000});
  data::DualArmProjectionConfig config;
  config.timestamp_source = data::TimestampSource::HeaderStamp;
  config.pairing_policy = data::PairingPolicy::Exact;
  config.timestamp_projection.policy = data::TimestampPolicy::Preserve;
  auto preserve = data::makeDualArmTimeline(left, right, config);
  require(preserve.timeline.size() == 3, "exact pairing count failed");
  require(preserve.timeline.at(0).projected_time_ns == 0 &&
          preserve.timeline.at(1).projected_time_ns == 9'500'000 &&
          preserve.timeline.at(2).projected_time_ns == 20'300'000,
    "preserve projection failed");

  config.timestamp_projection.policy = data::TimestampPolicy::FixedPeriod;
  config.timestamp_projection.period_ns = 10'000'000;
  auto fixed = data::makeDualArmTimeline(left, right, config);
  require(fixed.timeline.at(0).projected_time_ns == 0 &&
          fixed.timeline.at(1).projected_time_ns == 10'000'000 &&
          fixed.timeline.at(2).projected_time_ns == 20'000'000,
    "fixed-period projection failed");
  require(fixed.timeline.at(1).value.left.time.header_stamp_ns == 9'500'000 &&
          fixed.timeline.at(2).value.right.time.header_stamp_ns == 20'300'000,
    "fixed-period mutated source timestamp");

  config.pairing_policy = data::PairingPolicy::Nearest;
  config.nearest_tolerance_ns = 600'000;
  auto nearest_right = stream("right", {300'000, 9'900'000, 20'000'000});
  auto nearest = data::makeDualArmTimeline(left, nearest_right, config);
  require(nearest.timeline.size() == 3 && nearest.pairing.maximum_pair_delta_ns == 400'000,
    "nearest pairing failed");

  config.pairing_policy = data::PairingPolicy::Exact;
  config.nearest_tolerance_ns = 0;
  auto missing_right = stream("right", {0, 20'300'000});
  requireError(data::DataErrorCode::UnmatchedSample, [&] {
      (void)data::makeDualArmTimeline(left, missing_right, config);
    }, "missing right frame was hidden by fixed-period projection");
  config.unmatched_policy = data::UnmatchedPolicy::DropWithDiagnostics;
  const auto dropped = data::makeDualArmTimeline(left, missing_right, config);
  require(dropped.timeline.size() == 2 && dropped.pairing.unmatched_left_count == 1,
    "drop_with_diagnostics did not preserve missing-frame evidence");
  require(dropped.timeline.at(1).projected_time_ns == 10'000'000,
    "fixed retime should be one-to-one over matched semantic frames");

  config.unmatched_policy = data::UnmatchedPolicy::Error;
  requireError(data::DataErrorCode::NonMonotonicTimestamp, [&] {
      (void)data::makeDualArmTimeline(stream("left", {0, 2, 1}), stream("right", {0, 2, 3}), config);
    }, "non-monotonic timestamps were accepted");
  requireError(data::DataErrorCode::DuplicateTimestamp, [&] {
      (void)data::makeDualArmTimeline(stream("left", {0, 1, 1}), stream("right", {0, 1, 2}), config);
    }, "duplicate timestamps were accepted");
  requireError(data::DataErrorCode::MissingTimestamp, [&] {
      (void)data::makeDualArmTimeline(stream("left", {0, std::nullopt}), stream("right", {0, 1}), config);
    }, "missing timestamps were accepted");
  requireError(data::DataErrorCode::FrameMismatch, [&] {
      (void)data::makeDualArmTimeline(stream("left", {0}, "a"), stream("right", {0}, "b"), config);
    }, "frame mismatch was accepted");
}

void testInvalidPose()
{
  auto descriptor = poseDescriptor();
  data::Ros2PoseStampedCdrDecoder decoder;
  auto normalized = decoder.decode(data::EncodedRecord{data::BinaryRecord{
      descriptor, 1, 1, 1,
      poseCdr(true, 0, "base", {0, 0, 0, 0, 0, 0, 2})}});
  require(normalized.diagnostics.size() == 1 &&
          normalized.diagnostics.front().code == "quaternion_normalized",
    "quaternion normalization lacked diagnostics");
  requireError(data::DataErrorCode::DecodeFailure, [&] {
      (void)decodePoseRecord({descriptor, 1, 1, 1,
        poseCdr(true, 0, "base", {0, 0, 0, 0, 0, 0, 0})});
    }, "zero quaternion was accepted");
  requireError(data::DataErrorCode::DecodeFailure, [&] {
      (void)decodePoseRecord({descriptor, 1, 1, 1,
        poseCdr(true, 0, "base", {std::numeric_limits<double>::quiet_NaN(), 0, 0, 0, 0, 0, 1})});
    }, "NaN pose was accepted");
}

void testReplayClock()
{
  using Clock = data::ReplayClock::Clock;
  const auto start = Clock::time_point{std::chrono::nanoseconds{1'000}};
  auto current = start;
  int sleeps = 0;
  data::ReplayClock batch(
    data::ExecutionMode::Batch, 2.0,
    [&] {return current;},
    [&](Clock::time_point) {++sleeps;});
  const auto deadline = batch.deadline(start, 20'000'000);
  require(deadline - start == std::chrono::nanoseconds{10'000'000},
    "deadline did not use projected timestamp/playback rate");
  batch.waitUntil(deadline);
  require(sleeps == 0, "batch execution slept");

  data::ReplayClock realtime(
    data::ExecutionMode::Realtime, 1.0,
    [&] {return current;},
    [&](Clock::time_point requested) {++sleeps; current = requested;});
  realtime.waitUntil(realtime.deadline(start, 5'000'000));
  require(sleeps == 1 && current - start == std::chrono::nanoseconds{5'000'000},
    "realtime clock did not wait for absolute projected deadline");
}

}  // namespace

int main()
{
  try {
    TemporaryDirectory temporary;
    testMcapCsvAndDecoders(temporary);
    testTemporalProjection();
    testInvalidPose();
    testReplayClock();
    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "data_pipeline test failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
