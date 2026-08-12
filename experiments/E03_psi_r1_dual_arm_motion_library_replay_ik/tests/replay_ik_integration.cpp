#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <mcap/writer.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "apps/dual_arm_replay_ik/replay_ik_engine.hpp"

namespace replay = motion_control_lab::replay;
namespace data = motion_control_lab::data;

namespace
{

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    path_ = std::filesystem::temp_directory_path() /
            ("mcl-e03-ik-test-" +
             std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directory(path_);
  }

  ~TemporaryDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path & path() const { return path_; }

private:
  std::filesystem::path path_;
};

class CdrWriter
{
public:
  CdrWriter() { bytes_ = {std::byte{0}, std::byte{1}, std::byte{0}, std::byte{0}}; }

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
      bytes_.push_back(static_cast<std::byte>((value >> (index * 8U)) & 0xffU));
    }
  }

  std::vector<std::byte> bytes_;
};

std::vector<std::byte> poseCdr(std::int64_t stamp_ns, double x)
{
  CdrWriter writer;
  writer.int32(static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL));
  writer.uint32(static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL));
  writer.string("base_link");
  for (const double value : std::array<double, 7>{x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}) {
    writer.real64(value);
  }
  return writer.finish();
}

std::vector<std::byte> jointStateCdr()
{
  const auto & robot = replay::r1ReplayIkContract();
  CdrWriter writer;
  writer.int32(0);
  writer.uint32(0);
  writer.string("base_link");
  writer.strings(robot.joint_names);
  writer.doubles(robot.fallback_initial_positions);
  writer.doubles(std::vector<double>(robot.joint_names.size(), 9.0));
  writer.doubles({});
  return writer.finish();
}

void writeMessage(
  mcap::McapWriter & writer, mcap::ChannelId channel_id, std::uint32_t sequence,
  std::uint64_t timestamp, const std::vector<std::byte> & payload)
{
  mcap::Message message;
  message.channelId = channel_id;
  message.sequence = sequence;
  message.logTime = timestamp;
  message.publishTime = timestamp;
  message.data = payload.data();
  message.dataSize = payload.size();
  require(writer.write(message).ok(), "failed to write synthetic MCAP message");
}

void writeSyntheticAction(const std::filesystem::path & path)
{
  mcap::McapWriter writer;
  mcap::McapWriterOptions options("ros2");
  options.compression = mcap::Compression::Zstd;
  const auto open_status = writer.open(path.string(), options);
  require(open_status.ok(), "failed to create synthetic MCAP");

  mcap::Schema pose_schema(
    "geometry_msgs/msg/PoseStamped", "ros2msg",
    "std_msgs/Header header\ngeometry_msgs/Pose pose\n");
  writer.addSchema(pose_schema);
  mcap::Schema joint_schema(
    "sensor_msgs/msg/JointState", "ros2msg",
    "std_msgs/Header header\nstring[] name\nfloat64[] position\n"
    "float64[] velocity\nfloat64[] effort\n");
  writer.addSchema(joint_schema);
  mcap::Channel left_channel("/mc/ik/target/left_pose", "cdr", pose_schema.id);
  writer.addChannel(left_channel);
  mcap::Channel right_channel("/mc/ik/target/right_pose", "cdr", pose_schema.id);
  writer.addChannel(right_channel);
  mcap::Channel joint_channel("/mc/ik/joint_states", "cdr", joint_schema.id);
  writer.addChannel(joint_channel);

  const auto joint_payload = jointStateCdr();
  writeMessage(writer, joint_channel.id, 1, 0, joint_payload);
  for (std::size_t index = 0; index < 2; ++index) {
    const std::int64_t timestamp = static_cast<std::int64_t>(index) * 10'000'000;
    const auto left_payload = poseCdr(timestamp, 100.0);
    const auto right_payload = poseCdr(timestamp, -100.0);
    writeMessage(
      writer, left_channel.id, static_cast<std::uint32_t>(index + 1),
      static_cast<std::uint64_t>(timestamp), left_payload);
    writeMessage(
      writer, right_channel.id, static_cast<std::uint32_t>(index + 1),
      static_cast<std::uint64_t>(timestamp), right_payload);
  }
  writer.close();
}

}  // namespace

int main(int argc, char ** argv)
{
  require(argc == 2, "expected URDF path argument");
  TemporaryDirectory temporary;
  const auto input = temporary.path() / "unreachable.mcap";
  writeSyntheticAction(input);

  replay::ReplayOptions options;
  options.urdf_path = argv[1];
  options.input_path = input;
  options.left_stream = "/mc/ik/target/left_pose";
  options.right_stream = "/mc/ik/target/right_pose";
  options.initial_joint_state_stream = "/mc/ik/joint_states";
  options.timestamp_source = data::TimestampSource::HeaderStamp;
  options.pairing_policy = data::PairingPolicy::Nearest;
  options.nearest_tolerance_ns = 5'000'000;
  options.unmatched_policy = data::UnmatchedPolicy::Error;
  options.execution_mode = data::ExecutionMode::Batch;
  options.state_policy = replay::StatePolicy::PreviousSolution;
  options.servo_period_ns = 10'000'000;

  replay::ReplayIkExecutionConfig execution;
  execution.stop_on_first_error = true;
  std::vector<std::uint64_t> visualization_sequences;
  std::vector<double> first_visualized_velocities;
  execution.visualization_callback = [&](const replay::ReplayIkVisualizationSample & sample) {
    visualization_sequences.push_back(sample.sequence);
    if (sample.sequence == 0) {
      first_visualized_velocities = sample.velocities;
    }
  };
  const auto result = replay::executeReplayIkCase(options, execution);
  require(result.frames_planned == 2, "synthetic action must plan both paired frames");
  require(result.frames_attempted == 1, "action must stop on its first rejected frame");
  require(result.accepted_solves == 0, "unreachable target must not be accepted");
  require(result.rejected_solves == 1, "first rejected frame must be counted");
  require(result.first_failure_frame == 0, "first failure frame must be recorded");
  require(
    result.initial_positions == replay::r1ReplayIkContract().fallback_initial_positions,
    "first JointState positions must initialize by joint name");
  require(
    first_visualized_velocities.size() == replay::r1ReplayIkContract().joint_names.size() &&
      std::all_of(
        first_visualized_velocities.begin(), first_visualized_velocities.end(),
        [](double velocity) { return velocity == 0.0; }),
    "recorded JointState velocity must not replace the zero initial velocity contract");
  require(
    static_cast<std::size_t>(std::count(result.trace_csv.begin(), result.trace_csv.end(), '\n')) ==
      2,
    "prefix trace must contain its header and first rejected frame");
  execution.first_visualization_sequence = result.next_visualization_sequence;
  const auto second_result = replay::executeReplayIkCase(options, execution);
  require(
    visualization_sequences == std::vector<std::uint64_t>{0, 1, 2, 3},
    "action boundaries must publish a new initial frame with global monotonic sequence");
  require(
    second_result.next_visualization_sequence == 4,
    "visualization sequence cursor must advance across actions");
  return 0;
}
