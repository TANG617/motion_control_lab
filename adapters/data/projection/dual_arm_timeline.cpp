#include "adapters/data/projection/dual_arm_timeline.hpp"

#include "adapters/data/temporal/stream_alignment.hpp"
#include "adapters/data/temporal/timestamp_projector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <utility>

namespace motion_control_lab::data
{
namespace
{

void requireFrameMatch(const StampedPose & left, const StampedPose & right)
{
  if (left.frame_id != right.frame_id) {
    throw DataError(
            DataErrorCode::FrameMismatch,
            "dual-arm frame_id mismatch: left='" + left.frame_id +
            "', right='" + right.frame_id + "'");
  }
}

std::int64_t difference(std::int64_t left, std::int64_t right)
{
  if ((right > 0 && left < std::numeric_limits<std::int64_t>::min() + right) ||
      (right < 0 && left > std::numeric_limits<std::int64_t>::max() + right)) {
    throw DataError(DataErrorCode::InvalidTimestamp, "timestamp difference overflows int64");
  }
  return left - right;
}

void recordUnmatched(
  PairingDiagnostics & diagnostics,
  std::vector<Diagnostic> & messages,
  bool left,
  std::int64_t timestamp)
{
  if (left) {
    ++diagnostics.unmatched_left_count;
  } else {
    ++diagnostics.unmatched_right_count;
  }
  messages.push_back(
    {DiagnosticSeverity::Warning,
      left ? "unmatched_left" : "unmatched_right",
      std::string{"dropped unmatched "} + (left ? "left" : "right") +
      " sample at logical timestamp " + std::to_string(timestamp),
      std::nullopt});
}

}  // namespace

PairingPolicy parsePairingPolicy(const std::string & value)
{
  if (value == "exact") {
    return PairingPolicy::Exact;
  }
  if (value == "nearest") {
    return PairingPolicy::Nearest;
  }
  throw DataError(DataErrorCode::InvalidArgument, "unknown pairing policy: " + value);
}

UnmatchedPolicy parseUnmatchedPolicy(const std::string & value)
{
  if (value == "error") {
    return UnmatchedPolicy::Error;
  }
  if (value == "drop_with_diagnostics" || value == "drop-with-diagnostics") {
    return UnmatchedPolicy::DropWithDiagnostics;
  }
  throw DataError(DataErrorCode::InvalidArgument, "unknown unmatched policy: " + value);
}

std::string toString(PairingPolicy policy)
{
  return policy == PairingPolicy::Exact ? "exact" : "nearest";
}

std::string toString(UnmatchedPolicy policy)
{
  return policy == UnmatchedPolicy::Error ? "error" : "drop_with_diagnostics";
}

DualArmTimelineResult makeDualArmTimeline(
  const TypedStream<StampedPose> & left,
  const TypedStream<StampedPose> & right,
  const DualArmProjectionConfig & config)
{
  if (config.nearest_tolerance_ns < 0) {
    throw DataError(DataErrorCode::InvalidArgument, "nearest pairing tolerance must be non-negative");
  }
  const auto left_samples = validateAndOrderStream(left, config.timestamp_source);
  const auto right_samples = validateAndOrderStream(right, config.timestamp_source);

  DualArmTimelineResult result;
  result.pairing.left_input_count = left_samples.size();
  result.pairing.right_input_count = right_samples.size();
  std::vector<UnprojectedFrame<DualArmFrame>> frames;
  frames.reserve(std::min(left_samples.size(), right_samples.size()));

  auto add_pair = [&](std::size_t left_index, std::size_t right_index) {
      const auto & left_sample = left_samples[left_index];
      const auto & right_sample = right_samples[right_index];
      requireFrameMatch(left_sample.sample, right_sample.sample);
      const auto delta = difference(left_sample.logical_time_ns, right_sample.logical_time_ns);
      const auto absolute_delta = delta < 0 ? -delta : delta;
      result.pairing.maximum_pair_delta_ns =
        std::max(result.pairing.maximum_pair_delta_ns, absolute_delta);
      ++result.pairing.matched_count;
      // Nearest pairing uses the left stream as the semantic frame clock. The right
      // sample retains its independent provenance in DualArmFrame.
      frames.push_back(
        {left_sample.logical_time_ns,
          {left_sample.sample, right_sample.sample}});
    };

  std::size_t left_index = 0;
  std::size_t right_index = 0;
  if (config.pairing_policy == PairingPolicy::Exact) {
    while (left_index < left_samples.size() && right_index < right_samples.size()) {
      const auto left_time = left_samples[left_index].logical_time_ns;
      const auto right_time = right_samples[right_index].logical_time_ns;
      if (left_time == right_time) {
        add_pair(left_index++, right_index++);
      } else if (left_time < right_time) {
        recordUnmatched(result.pairing, result.diagnostics, true, left_time);
        ++left_index;
      } else {
        recordUnmatched(result.pairing, result.diagnostics, false, right_time);
        ++right_index;
      }
    }
  } else {
    const auto tolerance = config.nearest_tolerance_ns;
    while (left_index < left_samples.size() && right_index < right_samples.size()) {
      const auto left_time = left_samples[left_index].logical_time_ns;
      while (right_index < right_samples.size() &&
             difference(left_time, right_samples[right_index].logical_time_ns) > tolerance) {
        recordUnmatched(
          result.pairing, result.diagnostics, false,
          right_samples[right_index].logical_time_ns);
        ++right_index;
      }
      if (right_index >= right_samples.size()) {
        break;
      }
      const auto current_delta =
        std::llabs(difference(left_time, right_samples[right_index].logical_time_ns));
      if (current_delta > tolerance) {
        recordUnmatched(result.pairing, result.diagnostics, true, left_time);
        ++left_index;
        continue;
      }

      std::size_t best = right_index;
      if (right_index + 1 < right_samples.size()) {
        const auto next_delta =
          std::llabs(difference(left_time, right_samples[right_index + 1].logical_time_ns));
        if (next_delta <= tolerance && next_delta < current_delta) {
          recordUnmatched(
            result.pairing, result.diagnostics, false,
            right_samples[right_index].logical_time_ns);
          best = right_index + 1;
        }
      }
      add_pair(left_index, best);
      ++left_index;
      right_index = best + 1;
    }
  }

  while (left_index < left_samples.size()) {
    recordUnmatched(
      result.pairing, result.diagnostics, true,
      left_samples[left_index++].logical_time_ns);
  }
  while (right_index < right_samples.size()) {
    recordUnmatched(
      result.pairing, result.diagnostics, false,
      right_samples[right_index++].logical_time_ns);
  }

  if (config.unmatched_policy == UnmatchedPolicy::Error &&
      (result.pairing.unmatched_left_count > 0 ||
      result.pairing.unmatched_right_count > 0)) {
    throw DataError(
            DataErrorCode::UnmatchedSample,
            "dual-arm pairing left " + std::to_string(result.pairing.unmatched_left_count) +
            " and right " + std::to_string(result.pairing.unmatched_right_count) +
            " sample(s) unmatched");
  }
  result.timeline = projectTimestamps(std::move(frames), config.timestamp_projection);
  return result;
}

}  // namespace motion_control_lab::data
