#pragma once

#include <motion_control_viz/render_batch.hpp>

#include <google/protobuf/message.h>

#include "mcl_telemetry_v1.pb.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace motion_control_lab::hierarchical_kinematics_step
{

namespace telemetry_proto = ::mcl::telemetry::v1;

struct EventStamp
{
  std::uint64_t timestamp_ns{0};
  std::uint64_t run_time_ns{0};
  std::uint64_t sequence{0};
};

class RunClock
{
public:
  RunClock();
  EventStamp sample();

private:
  std::chrono::steady_clock::time_point steady_origin_;
  std::uint64_t wall_origin_ns_{0};
  std::atomic<std::uint64_t> next_sequence_{1};
};

telemetry_proto::SampleContext makeSampleContext(
  const EventStamp & stamp,
  telemetry_proto::AttemptOutcome outcome,
  bool committed,
  std::uint64_t run_generation,
  std::uint64_t target_revision,
  std::uint64_t attempt_revision,
  std::uint64_t value_revision);

struct TelemetryEvent
{
  motion_control::viz::LogLevel level{motion_control::viz::LogLevel::Info};
  std::string message;
  std::string name;
};

struct TelemetryRecord
{
  EventStamp stamp;
  std::string channel;
  std::unique_ptr<google::protobuf::Message> message;
  std::optional<TelemetryEvent> event;

  static TelemetryRecord encoded(
    EventStamp stamp,
    std::string channel,
    std::unique_ptr<google::protobuf::Message> message);
  static TelemetryRecord log(
    EventStamp stamp,
    std::string channel,
    motion_control::viz::LogLevel level,
    std::string name,
    std::string message);
};

template<std::size_t Capacity>
class TelemetryQueue
{
public:
  static_assert(Capacity > 1U);

  bool tryPush(TelemetryRecord record)
  {
    const auto head = head_.load(std::memory_order_relaxed);
    const auto tail = tail_.load(std::memory_order_acquire);
    if (head - tail >= Capacity) {
      dropped_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }
    slots_[head % Capacity].emplace(std::move(record));
    head_.store(head + 1U, std::memory_order_release);
    return true;
  }

  bool tryPop(TelemetryRecord & record)
  {
    const auto tail = tail_.load(std::memory_order_relaxed);
    const auto head = head_.load(std::memory_order_acquire);
    if (tail == head) {
      return false;
    }
    auto & slot = slots_[tail % Capacity];
    record = std::move(*slot);
    slot.reset();
    tail_.store(tail + 1U, std::memory_order_release);
    return true;
  }

  std::size_t depth() const
  {
    return static_cast<std::size_t>(
      head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire));
  }

  constexpr std::size_t capacity() const { return Capacity; }
  std::uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

private:
  std::array<std::optional<TelemetryRecord>, Capacity> slots_{};
  alignas(64) std::atomic<std::uint64_t> head_{0};
  alignas(64) std::atomic<std::uint64_t> tail_{0};
  std::atomic<std::uint64_t> dropped_{0};
};

inline constexpr std::size_t kTelemetryQueueCapacity = 512U;
using WorkerTelemetryQueue = TelemetryQueue<kTelemetryQueueCapacity>;

void drainTelemetryQueue(
  WorkerTelemetryQueue & queue,
  std::vector<TelemetryRecord> & records);
void sortTelemetryRecords(std::vector<TelemetryRecord> & records);

struct EncodeStatistics
{
  double encode_time_ms{0.0};
  std::uint64_t serialized_bytes{0};
};

class TelemetryEncoder
{
public:
  TelemetryEncoder();
  EncodeStatistics append(
    std::vector<TelemetryRecord> records,
    const EventStamp & emit_stamp,
    motion_control::viz::RenderBatch & batch) const;
  void appendMessage(
    std::string channel,
    google::protobuf::Message & message,
    const EventStamp & stamp,
    const EventStamp & emit_stamp,
    motion_control::viz::RenderBatch & batch,
    EncodeStatistics & statistics) const;

private:
  std::shared_ptr<const std::vector<std::byte>> schema_data_;
};

}  // namespace motion_control_lab::hierarchical_kinematics_step
