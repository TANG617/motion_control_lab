#include "apps/dual_arm_replay_ik/replay_support.hpp"

#include "adapters/data/decoder/csv_pose_decoder.hpp"
#include "adapters/data/decoder/decoder_registry.hpp"
#include "adapters/data/decoder/ros2_pose_stamped_cdr_decoder.hpp"
#include "adapters/data/source/csv_source.hpp"
#include "adapters/data/source/mcap_source.hpp"
#include "motion_control_lab/sha256.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <utility>

namespace motion_control_lab::replay
{
namespace
{

InputFormat parseInputFormat(const std::string & value)
{
  if (value == "mcap") {
    return InputFormat::Mcap;
  }
  if (value == "csv") {
    return InputFormat::Csv;
  }
  throw data::DataError(data::DataErrorCode::InvalidArgument, "unknown input format: " + value);
}

StatePolicy parseStatePolicy(const std::string & value)
{
  if (value == "previous_solution") {
    return StatePolicy::PreviousSolution;
  }
  if (value == "fixed_initial_state") {
    return StatePolicy::FixedInitialState;
  }
  throw data::DataError(data::DataErrorCode::InvalidArgument, "unknown state policy: " + value);
}

double parsePositiveDouble(const std::string & value, const std::string & option)
{
  std::size_t parsed = 0;
  double result = 0.0;
  try {
    result = std::stod(value, &parsed);
  } catch (const std::exception &) {
    throw data::DataError(data::DataErrorCode::InvalidArgument, option + " requires a number");
  }
  if (parsed != value.size() || !std::isfinite(result) || result <= 0.0) {
    throw data::DataError(data::DataErrorCode::InvalidArgument, option + " must be finite and > 0");
  }
  return result;
}

std::int64_t millisecondsToNanoseconds(const std::string & value, const std::string & option)
{
  const long double milliseconds = parsePositiveDouble(value, option);
  const long double nanoseconds = milliseconds * 1'000'000.0L;
  if (nanoseconds > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
    throw data::DataError(data::DataErrorCode::InvalidArgument, option + " is too large");
  }
  return static_cast<std::int64_t>(std::llround(nanoseconds));
}

std::int64_t nonnegativeMillisecondsToNanoseconds(
  const std::string & value,
  const std::string & option)
{
  std::size_t parsed = 0;
  long double milliseconds = 0.0L;
  try {
    milliseconds = std::stold(value, &parsed);
  } catch (const std::exception &) {
    throw data::DataError(data::DataErrorCode::InvalidArgument, option + " requires a number");
  }
  if (parsed != value.size() || !std::isfinite(milliseconds) || milliseconds < 0.0L) {
    throw data::DataError(data::DataErrorCode::InvalidArgument, option + " must be finite and >= 0");
  }
  const long double nanoseconds = milliseconds * 1'000'000.0L;
  if (nanoseconds > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
    throw data::DataError(data::DataErrorCode::InvalidArgument, option + " is too large");
  }
  return static_cast<std::int64_t>(std::llround(nanoseconds));
}

data::TimestampSource parseCsvTimestampTarget(
  const Json::Value & object,
  data::TimestampSource default_value)
{
  if (!object.isMember("timestamp_target")) {
    return default_value;
  }
  if (!object["timestamp_target"].isString()) {
    throw data::DataError(
            data::DataErrorCode::InvalidFormat,
            "CSV mapping timestamp_target must be a string");
  }
  return data::parseTimestampSource(object["timestamp_target"].asString());
}

std::string optionalString(
  const Json::Value & object,
  const std::string & key,
  const std::string & default_value)
{
  if (!object.isMember(key)) {
    return default_value;
  }
  if (!object[key].isString() || object[key].asString().empty()) {
    throw data::DataError(
            data::DataErrorCode::InvalidFormat,
            "CSV mapping field must be a non-empty string: " + key);
  }
  return object[key].asString();
}

data::CsvPoseMapping defaultMapping(
  const std::string & decoder_id,
  const std::string & prefix,
  data::TimestampSource timestamp_target)
{
  data::CsvPoseMapping mapping;
  mapping.decoder_id = decoder_id;
  mapping.timestamp_target = timestamp_target;
  mapping.frame_id_column = prefix + "frame_id";
  mapping.x_column = prefix + "x";
  mapping.y_column = prefix + "y";
  mapping.z_column = prefix + "z";
  mapping.qx_column = prefix + "qx";
  mapping.qy_column = prefix + "qy";
  mapping.qz_column = prefix + "qz";
  mapping.qw_column = prefix + "qw";
  return mapping;
}

data::CsvPoseMapping parseMapping(
  const Json::Value & root,
  const std::string & stream_name,
  const std::string & decoder_id,
  data::TimestampSource timestamp_target)
{
  if (!root.isMember("streams") || !root["streams"].isObject() ||
      !root["streams"].isMember(stream_name) ||
      !root["streams"][stream_name].isObject()) {
    throw data::DataError(
            data::DataErrorCode::InvalidFormat,
            "CSV mapping lacks streams['" + stream_name + "']");
  }
  const auto & object = root["streams"][stream_name];
  data::CsvPoseMapping mapping;
  mapping.decoder_id = decoder_id;
  mapping.timestamp_column = optionalString(object, "timestamp_column", "timestamp_ns");
  mapping.timestamp_target = parseCsvTimestampTarget(object, timestamp_target);
  if (object.isMember("header_stamp_column")) {
    mapping.header_stamp_column = optionalString(object, "header_stamp_column", "");
  }
  if (object.isMember("log_time_column")) {
    mapping.log_time_column = optionalString(object, "log_time_column", "");
  }
  if (object.isMember("publish_time_column")) {
    mapping.publish_time_column = optionalString(object, "publish_time_column", "");
  }
  if (object.isMember("fixed_frame_id")) {
    mapping.fixed_frame_id = optionalString(object, "fixed_frame_id", "");
    mapping.frame_id_column.reset();
  } else {
    mapping.frame_id_column = optionalString(object, "frame_id_column", "frame_id");
  }
  if (!object.isMember("columns") || !object["columns"].isObject()) {
    throw data::DataError(
            data::DataErrorCode::InvalidFormat,
            "CSV pose mapping requires a columns object");
  }
  const auto & columns = object["columns"];
  mapping.x_column = optionalString(columns, "x", "x");
  mapping.y_column = optionalString(columns, "y", "y");
  mapping.z_column = optionalString(columns, "z", "z");
  mapping.qx_column = optionalString(columns, "qx", "qx");
  mapping.qy_column = optionalString(columns, "qy", "qy");
  mapping.qz_column = optionalString(columns, "qz", "qz");
  mapping.qw_column = optionalString(columns, "qw", "qw");
  return mapping;
}

Json::Value loadJson(const std::filesystem::path & path)
{
  std::ifstream input(path);
  if (!input) {
    throw data::DataError(data::DataErrorCode::Io, "failed to open JSON: " + path.string());
  }
  Json::CharReaderBuilder builder;
  Json::Value result;
  std::string errors;
  if (!Json::parseFromStream(builder, input, &result, &errors)) {
    throw data::DataError(data::DataErrorCode::InvalidFormat, "invalid JSON: " + errors);
  }
  return result;
}

std::string jsonString(const Json::Value & value)
{
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "  ";
  return Json::writeString(builder, value) + "\n";
}

std::string pathString(const std::filesystem::path & path)
{
  return std::filesystem::absolute(path).lexically_normal().string();
}

Json::Int64 asJsonInt64(std::int64_t value)
{
  return static_cast<Json::Int64>(value);
}

}  // namespace

ReplayOptions parseReplayOptions(int argc, char ** argv, bool require_urdf)
{
  ReplayOptions result;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto requireValue = [&]() -> std::string {
        if (index + 1 >= argc) {
          throw data::DataError(
                  data::DataErrorCode::InvalidArgument,
                  argument + " requires a value");
        }
        return argv[++index];
      };
    if (argument == "--help" || argument == "-h") {
      result.help = true;
    } else if (argument == "--urdf") {
      result.urdf_path = requireValue();
    } else if (argument == "--input") {
      result.input_path = requireValue();
    } else if (argument == "--input-format") {
      result.input_format = parseInputFormat(requireValue());
    } else if (argument == "--left-stream") {
      result.left_stream = requireValue();
    } else if (argument == "--right-stream") {
      result.right_stream = requireValue();
    } else if (argument == "--csv-mapping") {
      result.csv_mapping_path = std::filesystem::path{requireValue()};
    } else if (argument == "--timestamp-source") {
      result.timestamp_source = data::parseTimestampSource(requireValue());
    } else if (argument == "--timestamp-policy") {
      result.timestamp_projection.policy = data::parseTimestampPolicy(requireValue());
    } else if (argument == "--period-ms") {
      result.timestamp_projection.period_ns = millisecondsToNanoseconds(requireValue(), argument);
    } else if (argument == "--pairing-policy") {
      result.pairing_policy = data::parsePairingPolicy(requireValue());
    } else if (argument == "--nearest-tolerance-ms") {
      result.nearest_tolerance_ns = nonnegativeMillisecondsToNanoseconds(requireValue(), argument);
    } else if (argument == "--unmatched-policy") {
      result.unmatched_policy = data::parseUnmatchedPolicy(requireValue());
    } else if (argument == "--execution-mode") {
      result.execution_mode = data::parseExecutionMode(requireValue());
    } else if (argument == "--playback-rate") {
      result.playback_rate = parsePositiveDouble(requireValue(), argument);
    } else if (argument == "--state-policy") {
      result.state_policy = parseStatePolicy(requireValue());
    } else if (argument == "--servo-period-ms") {
      result.servo_period_ns = millisecondsToNanoseconds(requireValue(), argument);
    } else if (argument == "--output-dir") {
      result.output_dir = requireValue();
    } else {
      throw data::DataError(data::DataErrorCode::InvalidArgument, "unknown option: " + argument);
    }
  }
  if (result.help) {
    return result;
  }
  if (result.input_path.empty() || result.left_stream.empty() || result.right_stream.empty()) {
    throw data::DataError(
            data::DataErrorCode::InvalidArgument,
            "--input, --left-stream, and --right-stream are required");
  }
  if (result.left_stream == result.right_stream) {
    throw data::DataError(data::DataErrorCode::InvalidArgument, "left and right streams must differ");
  }
  if (!std::filesystem::is_regular_file(result.input_path)) {
    throw data::DataError(
            data::DataErrorCode::Io,
            "input is not a regular file: " + result.input_path.string());
  }
  if (result.input_format == InputFormat::Mcap && result.csv_mapping_path.has_value()) {
    throw data::DataError(
            data::DataErrorCode::InvalidArgument,
            "--csv-mapping is only valid with --input-format csv");
  }
  if (result.csv_mapping_path.has_value() &&
      !std::filesystem::is_regular_file(*result.csv_mapping_path)) {
    throw data::DataError(data::DataErrorCode::Io, "CSV mapping file does not exist");
  }
  if (result.pairing_policy == data::PairingPolicy::Exact && result.nearest_tolerance_ns != 0) {
    throw data::DataError(
            data::DataErrorCode::InvalidArgument,
            "--nearest-tolerance-ms requires --pairing-policy nearest");
  }
  if (require_urdf && !std::filesystem::is_regular_file(result.urdf_path)) {
    throw data::DataError(data::DataErrorCode::Io, "--urdf must name an existing file");
  }
  if (result.output_dir.empty()) {
    throw data::DataError(data::DataErrorCode::InvalidArgument, "--output-dir must be non-empty");
  }
  return result;
}

std::string replayHelp(const std::string & program, bool include_urdf)
{
  std::ostringstream output;
  output << "Usage: " << program << " [options]\n";
  if (include_urdf) {
    output << "  --urdf <path>                    R1 URDF (required)\n";
  }
  output
    << "  --input <path>                   MCAP or CSV input (required)\n"
    << "  --input-format mcap|csv          Physical source backend\n"
    << "  --left-stream <name>             Left logical stream/topic\n"
    << "  --right-stream <name>            Right logical stream/topic\n"
    << "  --csv-mapping <json>             Optional CSV pose column mapping\n"
    << "  --timestamp-source header_stamp|log_time|publish_time|csv_timestamp\n"
    << "  --timestamp-policy preserve|fixed-period\n"
    << "  --period-ms <ms>                 Fixed retime period (default 10)\n"
    << "  --pairing-policy exact|nearest   Original-time pairing (default exact)\n"
    << "  --nearest-tolerance-ms <ms>      Nearest pairing tolerance\n"
    << "  --unmatched-policy error|drop_with_diagnostics\n"
    << "  --execution-mode batch|realtime\n"
    << "  --playback-rate <rate>           Positive replay rate (default 1)\n"
    << "  --state-policy previous_solution|fixed_initial_state\n"
    << "  --servo-period-ms <ms>           Core control horizon (default 10)\n"
    << "  --output-dir <path>              New artifact directory\n"
    << "  --help                           Show this help\n";
  return output.str();
}

std::string toString(InputFormat format)
{
  return format == InputFormat::Mcap ? "mcap" : "csv";
}

std::string toString(StatePolicy policy)
{
  return policy == StatePolicy::PreviousSolution
    ? "previous_solution" : "fixed_initial_state";
}

LoadedReplay loadReplay(const ReplayOptions & options)
{
  data::DecoderRegistry registry;
  std::unique_ptr<data::DataSource> source;
  std::string left_decoder;
  std::string right_decoder;

  if (options.input_format == InputFormat::Mcap) {
    source = std::make_unique<data::McapSource>(options.input_path);
    auto decoder = std::make_shared<data::Ros2PoseStampedCdrDecoder>();
    left_decoder = decoder->id();
    right_decoder = decoder->id();
    registry.registerDecoder(std::move(decoder));
  } else {
    source = std::make_unique<data::CsvSource>(
      options.input_path,
      std::vector<std::string>{options.left_stream, options.right_stream});
    data::CsvPoseMapping left_mapping;
    data::CsvPoseMapping right_mapping;
    if (options.csv_mapping_path.has_value()) {
      const auto mapping = loadJson(*options.csv_mapping_path);
      if (mapping.isMember("schema_version") &&
          (!mapping["schema_version"].isString() ||
          mapping["schema_version"].asString() != "mcl.csv_mapping.v1")) {
        throw data::DataError(
                data::DataErrorCode::InvalidFormat,
                "CSV mapping schema_version must be mcl.csv_mapping.v1");
      }
      left_mapping = parseMapping(
        mapping, options.left_stream, "csv_pose:left", options.timestamp_source);
      right_mapping = parseMapping(
        mapping, options.right_stream, "csv_pose:right", options.timestamp_source);
    } else {
      left_mapping = defaultMapping("csv_pose:left", "left_", options.timestamp_source);
      right_mapping = defaultMapping("csv_pose:right", "right_", options.timestamp_source);
    }
    left_decoder = left_mapping.decoder_id;
    right_decoder = right_mapping.decoder_id;
    registry.registerDecoder(std::make_shared<data::CsvPoseDecoder>(std::move(left_mapping)));
    registry.registerDecoder(std::make_shared<data::CsvPoseDecoder>(std::move(right_mapping)));
  }

  auto left_cursor = source->select({options.left_stream, std::nullopt, std::nullopt});
  auto right_cursor = source->select({options.right_stream, std::nullopt, std::nullopt});
  auto left = registry.decode<data::StampedPose>(
    *left_cursor, options.left_stream, left_decoder);
  auto right = registry.decode<data::StampedPose>(
    *right_cursor, options.right_stream, right_decoder);

  data::DualArmProjectionConfig projection;
  projection.timestamp_source = options.timestamp_source;
  projection.pairing_policy = options.pairing_policy;
  projection.nearest_tolerance_ns = options.nearest_tolerance_ns;
  projection.unmatched_policy = options.unmatched_policy;
  projection.timestamp_projection = options.timestamp_projection;

  LoadedReplay loaded;
  loaded.catalog = source->catalog();
  loaded.left_decoder = left.decoder_id;
  loaded.right_decoder = right.decoder_id;
  loaded.decoder_diagnostic_count = left.diagnostics.size() + right.diagnostics.size();
  loaded.timeline = data::makeDualArmTimeline(left, right, projection);
  return loaded;
}

void createOutputDirectory(const std::filesystem::path & output_dir)
{
  if (std::filesystem::exists(output_dir)) {
    throw data::DataError(
            data::DataErrorCode::Io,
            "refusing to overwrite output directory: " + output_dir.string());
  }
  std::error_code error;
  if (!std::filesystem::create_directories(output_dir, error) || error) {
    throw data::DataError(
            data::DataErrorCode::Io,
            "failed to create output directory: " + error.message());
  }
}

void writeTextFile(const std::filesystem::path & path, const std::string & contents)
{
  std::ofstream output(path, std::ios::binary);
  if (!output) {
    throw data::DataError(data::DataErrorCode::Io, "failed to create artifact: " + path.string());
  }
  output << contents;
  if (!output) {
    throw data::DataError(data::DataErrorCode::Io, "failed to write artifact: " + path.string());
  }
}

std::string csvEscape(const std::string & value)
{
  if (value.find_first_of(",\"\r\n") == std::string::npos) {
    return value;
  }
  std::string result{"\""};
  for (const char character : value) {
    if (character == '"') {
      result += "\"\"";
    } else {
      result += character;
    }
  }
  result += '"';
  return result;
}

std::string optionalTimestamp(const std::optional<std::int64_t> & value)
{
  return value.has_value() ? std::to_string(*value) : std::string{};
}

Json::Value makeReplayManifest(
  const ReplayOptions & options,
  const LoadedReplay & loaded,
  std::size_t deadline_miss_count,
  std::size_t accepted_count,
  const std::string & trace_sha256)
{
  Json::Value manifest;
  manifest["schema_version"] = "dual_arm_replay_manifest.v1";
  manifest["dependencies"]["mcap"] =
    "foxglove/mcap releases/cpp/v2.1.3@1420296ffcfdcde4b6894c0c1aba0ad083f93dde";
  manifest["input"]["path"] = pathString(options.input_path);
  manifest["input"]["sha256"] = sha256_file(options.input_path);
  manifest["input"]["format"] = toString(options.input_format);
  manifest["streams"]["left"] = options.left_stream;
  manifest["streams"]["right"] = options.right_stream;
  manifest["decoders"]["left"] = loaded.left_decoder;
  manifest["decoders"]["right"] = loaded.right_decoder;
  manifest["timestamp"]["source"] = data::toString(options.timestamp_source);
  manifest["timestamp"]["pairing_policy"] = data::toString(options.pairing_policy);
  manifest["timestamp"]["nearest_tolerance_ns"] = asJsonInt64(options.nearest_tolerance_ns);
  manifest["timestamp"]["unmatched_policy"] = data::toString(options.unmatched_policy);
  manifest["timestamp"]["projection_policy"] = data::toString(options.timestamp_projection.policy);
  manifest["timestamp"]["period_ns"] = asJsonInt64(options.timestamp_projection.period_ns);
  manifest["execution"]["mode"] = data::toString(options.execution_mode);
  manifest["execution"]["playback_rate"] = options.playback_rate;
  manifest["execution"]["state_policy"] = toString(options.state_policy);
  manifest["execution"]["servo_period_ns"] = asJsonInt64(options.servo_period_ns);
  manifest["execution"]["hard_realtime"] = false;
  manifest["statistics"]["frames"] = Json::UInt64(loaded.timeline.timeline.size());
  manifest["statistics"]["left_input"] = Json::UInt64(loaded.timeline.pairing.left_input_count);
  manifest["statistics"]["right_input"] = Json::UInt64(loaded.timeline.pairing.right_input_count);
  manifest["statistics"]["matched"] = Json::UInt64(loaded.timeline.pairing.matched_count);
  manifest["statistics"]["unmatched_left"] = Json::UInt64(loaded.timeline.pairing.unmatched_left_count);
  manifest["statistics"]["unmatched_right"] = Json::UInt64(loaded.timeline.pairing.unmatched_right_count);
  manifest["statistics"]["maximum_pair_delta_ns"] =
    asJsonInt64(loaded.timeline.pairing.maximum_pair_delta_ns);
  manifest["statistics"]["decoder_diagnostics"] = Json::UInt64(loaded.decoder_diagnostic_count);
  manifest["statistics"]["deadline_misses"] = Json::UInt64(deadline_miss_count);
  manifest["statistics"]["accepted_solves"] = Json::UInt64(accepted_count);
  manifest["artifacts"]["trace.csv"]["sha256"] = trace_sha256;
  return manifest;
}

void writeReplayPlanArtifacts(
  const ReplayOptions & options,
  const LoadedReplay & loaded)
{
  createOutputDirectory(options.output_dir);
  std::ostringstream trace;
  trace << "sequence,original_logical_timestamp_ns,source_time_from_start_ns,"
           "projected_timestamp_ns,scheduled_time_from_start_ns,"
           "left_header_stamp_ns,left_log_time_ns,left_publish_time_ns,"
           "right_header_stamp_ns,right_log_time_ns,right_publish_time_ns\n";
  for (const auto & frame : loaded.timeline.timeline) {
    const auto scheduled = static_cast<std::int64_t>(std::llround(
      static_cast<long double>(frame.projected_time_ns) / options.playback_rate));
    trace
      << frame.sequence << ','
      << frame.original_logical_time_ns << ','
      << frame.source_time_from_start_ns << ','
      << frame.projected_time_ns << ','
      << scheduled << ','
      << optionalTimestamp(frame.value.left.time.header_stamp_ns) << ','
      << optionalTimestamp(frame.value.left.time.log_time_ns) << ','
      << optionalTimestamp(frame.value.left.time.publish_time_ns) << ','
      << optionalTimestamp(frame.value.right.time.header_stamp_ns) << ','
      << optionalTimestamp(frame.value.right.time.log_time_ns) << ','
      << optionalTimestamp(frame.value.right.time.publish_time_ns) << '\n';
  }
  const auto trace_path = options.output_dir / "trace.csv";
  writeTextFile(trace_path, trace.str());
  const auto manifest = makeReplayManifest(
    options, loaded, 0, 0, sha256_file(trace_path));
  writeTextFile(options.output_dir / "manifest.json", jsonString(manifest));
}

}  // namespace motion_control_lab::replay
