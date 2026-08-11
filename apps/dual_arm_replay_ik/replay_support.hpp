#pragma once

#include "adapters/data/projection/dual_arm_timeline.hpp"
#include "adapters/data/source/data_source.hpp"
#include "adapters/data/temporal/replay_clock.hpp"

#include <filesystem>
#include <optional>
#include <string>

#include <json/json.h>

namespace motion_control_lab::replay
{

enum class InputFormat
{
  Mcap,
  Csv
};

enum class StatePolicy
{
  PreviousSolution,
  FixedInitialState
};

struct ReplayOptions
{
  bool help{};
  std::filesystem::path urdf_path;
  std::filesystem::path input_path;
  InputFormat input_format{InputFormat::Mcap};
  std::string left_stream;
  std::string right_stream;
  std::optional<std::filesystem::path> csv_mapping_path;
  data::TimestampSource timestamp_source{data::TimestampSource::HeaderStamp};
  data::TimestampProjectionConfig timestamp_projection;
  data::PairingPolicy pairing_policy{data::PairingPolicy::Exact};
  std::int64_t nearest_tolerance_ns{};
  data::UnmatchedPolicy unmatched_policy{data::UnmatchedPolicy::Error};
  data::ExecutionMode execution_mode{data::ExecutionMode::Batch};
  double playback_rate{1.0};
  StatePolicy state_policy{StatePolicy::PreviousSolution};
  std::int64_t servo_period_ns{10'000'000};
  std::filesystem::path output_dir{"dual_arm_replay_output"};
};

struct LoadedReplay
{
  data::SourceCatalog catalog;
  data::DualArmTimelineResult timeline;
  std::string left_decoder;
  std::string right_decoder;
  std::size_t decoder_diagnostic_count{};
};

ReplayOptions parseReplayOptions(int argc, char ** argv, bool require_urdf);
std::string replayHelp(const std::string & program, bool include_urdf);
std::string toString(InputFormat format);
std::string toString(StatePolicy policy);

LoadedReplay loadReplay(const ReplayOptions & options);

void createOutputDirectory(const std::filesystem::path & output_dir);
void writeTextFile(const std::filesystem::path & path, const std::string & contents);
std::string csvEscape(const std::string & value);
std::string optionalTimestamp(const std::optional<std::int64_t> & value);
Json::Value makeReplayManifest(
  const ReplayOptions & options,
  const LoadedReplay & loaded,
  std::size_t deadline_miss_count,
  std::size_t accepted_count,
  const std::string & trace_sha256);
void writeReplayPlanArtifacts(
  const ReplayOptions & options,
  const LoadedReplay & loaded);

}  // namespace motion_control_lab::replay
