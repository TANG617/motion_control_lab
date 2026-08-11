#pragma once

#include "adapters/data/temporal/timeline.hpp"
#include "contracts/data/pose_sample.hpp"
#include "contracts/data/typed_stream.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace motion_control_lab::data
{

struct DualArmFrame
{
  StampedPose left;
  StampedPose right;
};

enum class PairingPolicy
{
  Exact,
  Nearest
};

enum class UnmatchedPolicy
{
  Error,
  DropWithDiagnostics
};

struct DualArmProjectionConfig
{
  TimestampSource timestamp_source{TimestampSource::HeaderStamp};
  PairingPolicy pairing_policy{PairingPolicy::Exact};
  std::int64_t nearest_tolerance_ns{};
  UnmatchedPolicy unmatched_policy{UnmatchedPolicy::Error};
  TimestampProjectionConfig timestamp_projection;
};

struct PairingDiagnostics
{
  std::size_t left_input_count{};
  std::size_t right_input_count{};
  std::size_t matched_count{};
  std::size_t unmatched_left_count{};
  std::size_t unmatched_right_count{};
  std::int64_t maximum_pair_delta_ns{};
};

struct DualArmTimelineResult
{
  Timeline<DualArmFrame> timeline;
  PairingDiagnostics pairing;
  std::vector<Diagnostic> diagnostics;
};

PairingPolicy parsePairingPolicy(const std::string & value);
UnmatchedPolicy parseUnmatchedPolicy(const std::string & value);
std::string toString(PairingPolicy policy);
std::string toString(UnmatchedPolicy policy);

DualArmTimelineResult makeDualArmTimeline(
  const TypedStream<StampedPose> & left,
  const TypedStream<StampedPose> & right,
  const DualArmProjectionConfig & config);

}  // namespace motion_control_lab::data
