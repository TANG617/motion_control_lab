#include "telemetry.hpp"

#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/timestamp.pb.h>

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace motion_control_lab::planned_hierarchical_step_otg
{
namespace
{

std::uint64_t systemClockNanoseconds()
{
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
}

void setTimestamp(google::protobuf::Timestamp & timestamp, std::uint64_t nanoseconds)
{
  timestamp.set_seconds(static_cast<std::int64_t>(nanoseconds / 1000000000ULL));
  timestamp.set_nanos(static_cast<std::int32_t>(nanoseconds % 1000000000ULL));
}

void addFileDescriptor(
  const google::protobuf::FileDescriptor * descriptor,
  std::unordered_set<std::string> & included,
  google::protobuf::FileDescriptorSet & descriptor_set)
{
  if (!included.emplace(descriptor->name()).second) {
    return;
  }
  for (int index = 0; index < descriptor->dependency_count(); ++index) {
    addFileDescriptor(descriptor->dependency(index), included, descriptor_set);
  }
  descriptor->CopyTo(descriptor_set.add_file());
}

std::shared_ptr<const std::vector<std::byte>> makeSchemaData()
{
  google::protobuf::FileDescriptorSet descriptor_set;
  std::unordered_set<std::string> included;
  addFileDescriptor(telemetry_proto::RunInfo::descriptor()->file(), included, descriptor_set);
  const auto size = descriptor_set.ByteSizeLong();
  auto data = std::make_shared<std::vector<std::byte>>(size);
  if (!descriptor_set.SerializeToArray(data->data(), static_cast<int>(size))) {
    throw std::runtime_error("failed to serialize MCL telemetry FileDescriptorSet");
  }
  return data;
}

void setEmitTime(google::protobuf::Message & message, std::uint64_t emit_time_ns)
{
  const auto * descriptor = message.GetDescriptor();
  const auto * context_field = descriptor->FindFieldByName("context");
  if (context_field == nullptr ||
    context_field->cpp_type() != google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE)
  {
    return;
  }
  auto * context = message.GetReflection()->MutableMessage(&message, context_field);
  const auto * emit_field = context->GetDescriptor()->FindFieldByName("emit_time");
  if (emit_field == nullptr ||
    emit_field->cpp_type() != google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE)
  {
    throw std::runtime_error("telemetry context is missing emit_time");
  }
  auto * timestamp = context->GetReflection()->MutableMessage(context, emit_field);
  auto * typed_timestamp = dynamic_cast<google::protobuf::Timestamp *>(timestamp);
  if (typed_timestamp == nullptr) {
    throw std::runtime_error("telemetry emit_time is not google.protobuf.Timestamp");
  }
  setTimestamp(*typed_timestamp, emit_time_ns);
}

}  // namespace

RunClock::RunClock()
: steady_origin_(std::chrono::steady_clock::now()),
  wall_origin_ns_(systemClockNanoseconds())
{
}

EventStamp RunClock::sample()
{
  const auto run_time_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - steady_origin_).count());
  return EventStamp{
    wall_origin_ns_ + run_time_ns,
    run_time_ns,
    next_sequence_.fetch_add(1U, std::memory_order_relaxed)};
}

telemetry_proto::SampleContext makeSampleContext(
  const EventStamp & stamp,
  telemetry_proto::AttemptOutcome outcome,
  bool committed,
  std::uint64_t run_generation,
  std::uint64_t target_revision,
  std::uint64_t attempt_revision,
  std::uint64_t value_revision)
{
  telemetry_proto::SampleContext context;
  setTimestamp(*context.mutable_timestamp(), stamp.timestamp_ns);
  context.set_run_time_ns(stamp.run_time_ns);
  context.set_sequence(stamp.sequence);
  context.set_run_generation(run_generation);
  context.set_target_revision(target_revision);
  context.set_attempt_revision(attempt_revision);
  context.set_value_revision(value_revision);
  context.set_outcome(outcome);
  context.set_committed(committed);
  return context;
}

TelemetryRecord TelemetryRecord::encoded(
  EventStamp stamp,
  std::string channel,
  std::unique_ptr<google::protobuf::Message> message)
{
  if (!message) {
    throw std::invalid_argument("telemetry message must not be null");
  }
  TelemetryRecord record;
  record.stamp = stamp;
  record.channel = std::move(channel);
  record.message = std::move(message);
  return record;
}

TelemetryRecord TelemetryRecord::log(
  EventStamp stamp,
  std::string channel,
  motion_control::viz::LogLevel level,
  std::string name,
  std::string message)
{
  TelemetryRecord record;
  record.stamp = stamp;
  record.channel = std::move(channel);
  record.event = TelemetryEvent{level, std::move(message), std::move(name)};
  return record;
}

void drainTelemetryQueue(
  WorkerTelemetryQueue & queue,
  std::vector<TelemetryRecord> & records)
{
  TelemetryRecord record;
  while (queue.tryPop(record)) {
    records.push_back(std::move(record));
  }
}

void sortTelemetryRecords(std::vector<TelemetryRecord> & records)
{
  std::stable_sort(
    records.begin(), records.end(),
    [](const TelemetryRecord & left, const TelemetryRecord & right) {
      if (left.stamp.timestamp_ns != right.stamp.timestamp_ns) {
        return left.stamp.timestamp_ns < right.stamp.timestamp_ns;
      }
      return left.stamp.sequence < right.stamp.sequence;
    });
}

TelemetryEncoder::TelemetryEncoder()
: schema_data_(makeSchemaData())
{
}

EncodeStatistics TelemetryEncoder::append(
  std::vector<TelemetryRecord> records,
  const EventStamp & emit_stamp,
  motion_control::viz::RenderBatch & batch) const
{
  sortTelemetryRecords(records);
  EncodeStatistics statistics;
  for (auto & record : records) {
    if (record.message) {
      appendMessage(
        std::move(record.channel), *record.message, record.stamp, emit_stamp, batch, statistics);
    } else if (record.event.has_value()) {
      batch.logs.push_back(motion_control::viz::LogSample{
        std::move(record.channel),
        record.event->level,
        std::move(record.event->message),
        std::move(record.event->name),
        {},
        0U,
        record.stamp.timestamp_ns});
    }
  }
  return statistics;
}

void TelemetryEncoder::appendMessage(
  std::string channel,
  google::protobuf::Message & message,
  const EventStamp & stamp,
  const EventStamp & emit_stamp,
  motion_control::viz::RenderBatch & batch,
  EncodeStatistics & statistics) const
{
  const auto started = std::chrono::steady_clock::now();
  setEmitTime(message, emit_stamp.timestamp_ns);
  const auto size = message.ByteSizeLong();
  std::vector<std::byte> data(size);
  if (!message.SerializeToArray(data.data(), static_cast<int>(size))) {
    throw std::runtime_error("failed to serialize " + message.GetDescriptor()->full_name());
  }
  statistics.serialized_bytes += size;
  statistics.encode_time_ms += std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  batch.encoded_messages.push_back(motion_control::viz::EncodedMessageSample{
    std::move(channel),
    "protobuf",
    message.GetDescriptor()->full_name(),
    "protobuf",
    schema_data_,
    std::move(data),
    stamp.timestamp_ns});
}

}  // namespace motion_control_lab::planned_hierarchical_step_otg
