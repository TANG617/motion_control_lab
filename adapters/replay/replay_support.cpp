#include "adapters/replay/replay_support.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <utility>

#include "adapters/data/decoder/csv_pose_decoder.hpp"
#include "adapters/data/decoder/decode_stream.hpp"
#include "adapters/data/decoder/ros2_joint_state_cdr_decoder.hpp"
#include "adapters/data/decoder/ros2_pose_stamped_cdr_decoder.hpp"
#include "adapters/data/source/csv_source.hpp"
#include "adapters/data/source/mcap_source.hpp"
#include "motion_control_lab/sha256.hpp"

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

double parseDouble(const std::string & value) { return std::stod(value); }

std::uint16_t parsePort(const std::string & value)
{
  return static_cast<std::uint16_t>(std::stoul(value));
}

void validateRunId(const std::string & value)
{
  if (value.empty() || !std::all_of(value.begin(), value.end(), [](unsigned char character) {
        return std::isalnum(character) || character == '-' || character == '_' || character == '.';
      })) {
    throw data::DataError(
      data::DataErrorCode::InvalidArgument,
      "--run-id must contain only letters, digits, dot, "
      "underscore, or hyphen");
  }
}

std::int64_t millisecondsToNanoseconds(const std::string & value)
{
  return static_cast<std::int64_t>(std::llround(std::stold(value) * 1'000'000.0L));
}

std::int64_t nonnegativeMillisecondsToNanoseconds(const std::string & value)
{
  return millisecondsToNanoseconds(value);
}

data::TimestampSource parseCsvTimestampTarget(
  const Json::Value & object, data::TimestampSource default_value)
{
  if (!object.isMember("timestamp_target")) {
    return default_value;
  }
  return data::parseTimestampSource(object["timestamp_target"].asString());
}

std::string optionalString(
  const Json::Value & object, const std::string & key, const std::string & default_value)
{
  if (!object.isMember(key)) {
    return default_value;
  }
  return object[key].asString();
}

data::CsvPoseMapping defaultMapping(
  const std::string & decoder_id, const std::string & prefix,
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
  const Json::Value & root, const std::string & stream_name, const std::string & decoder_id,
  data::TimestampSource timestamp_target)
{
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
  std::ifstream input;
  input.exceptions(std::ios::failbit | std::ios::badbit);
  input.open(path);
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

Json::Int64 asJsonInt64(std::int64_t value) { return static_cast<Json::Int64>(value); }

}  // namespace

ReplayOptions parseReplayOptions(int argc, char ** argv, bool require_urdf)
{
  ReplayOptions result;
  for (int index = 0; index < argc; ++index) {
    result.original_argv.emplace_back(argv[index]);
  }
  bool initial_joint_state_stream_set = false;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto requireValue = [&]() -> std::string {
      if (index + 1 >= argc) {
        throw data::DataError(data::DataErrorCode::InvalidArgument, argument + " requires a value");
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
    } else if (argument == "--initial-joint-state-stream") {
      result.initial_joint_state_stream = requireValue();
      initial_joint_state_stream_set = true;
    } else if (argument == "--csv-mapping") {
      result.csv_mapping_path = std::filesystem::path{requireValue()};
    } else if (argument == "--timestamp-source") {
      result.timestamp_source = data::parseTimestampSource(requireValue());
    } else if (argument == "--target-period-ms") {
      result.target_period_ns = millisecondsToNanoseconds(requireValue());
    } else if (argument == "--pairing-policy") {
      result.pairing_policy = data::parsePairingPolicy(requireValue());
    } else if (argument == "--nearest-tolerance-ms") {
      result.nearest_tolerance_ns = nonnegativeMillisecondsToNanoseconds(requireValue());
    } else if (argument == "--unmatched-policy") {
      result.unmatched_policy = data::parseUnmatchedPolicy(requireValue());
    } else if (argument == "--execution-mode") {
      result.execution_mode = data::parseExecutionMode(requireValue());
    } else if (argument == "--playback-rate") {
      result.playback_rate = parseDouble(requireValue());
    } else if (argument == "--output-dir") {
      result.output_dir = requireValue();
      result.output_dir_explicit = true;
    } else if (argument == "--output-root") {
      result.output_root = std::filesystem::path{requireValue()};
    } else if (argument == "--run-id") {
      result.run_id = requireValue();
      validateRunId(*result.run_id);
    } else if (argument == "--launcher") {
      result.launcher = requireValue();
    } else if (argument == "--ui") {
      result.ui_mode = requireValue();
    } else if (argument == "--terminal-input") {
      const auto value = requireValue();
      if (value == "on") {
        result.terminal_input_enabled = true;
      } else if (value == "off") {
        result.terminal_input_enabled = false;
      } else {
        throw data::DataError(
          data::DataErrorCode::InvalidArgument, "--terminal-input must be 'on' or 'off'");
      }
    } else if (argument == "--viz") {
      const auto value = requireValue();
      if (value == "foxglove") {
        result.visualization_enabled = true;
      } else if (value == "none") {
        result.visualization_enabled = false;
      } else {
        throw data::DataError(
          data::DataErrorCode::InvalidArgument, "--viz must be 'foxglove' or 'none'");
      }
    } else if (argument == "--host" || argument == "--viz-host") {
      result.visualization_host = requireValue();
    } else if (argument == "--port" || argument == "--viz-port") {
      result.visualization_port = parsePort(requireValue());
    } else if (argument == "--mcap") {
      result.visualization_mcap_path = std::filesystem::path{requireValue()};
    } else if (argument == "--no-mcap") {
      result.visualization_mcap_path.reset();
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
    throw data::DataError(
      data::DataErrorCode::InvalidArgument, "left and right streams must differ");
  }
  if (result.target_period_ns <= 0) {
    throw data::DataError(
      data::DataErrorCode::InvalidArgument, "--target-period-ms is required and must be positive");
  }
  result.timestamp_projection.policy = data::TimestampPolicy::FixedPeriod;
  result.timestamp_projection.period_ns = result.target_period_ns;
  if (!std::isfinite(result.playback_rate) || result.playback_rate <= 0.0) {
    throw data::DataError(
      data::DataErrorCode::InvalidArgument, "--playback-rate must be finite and positive");
  }
  if (require_urdf && result.urdf_path.empty()) {
    throw data::DataError(data::DataErrorCode::InvalidArgument, "--urdf is required");
  }
  if (result.ui_mode != "tui" && result.ui_mode != "none") {
    throw data::DataError(
      data::DataErrorCode::InvalidArgument, "--ui must be either 'tui' or 'none'");
  }
  if (result.input_format == InputFormat::Mcap && result.csv_mapping_path.has_value()) {
    throw data::DataError(
      data::DataErrorCode::InvalidArgument, "--csv-mapping is only valid with --input-format csv");
  }
  if (result.input_format == InputFormat::Csv && initial_joint_state_stream_set) {
    throw data::DataError(
      data::DataErrorCode::InvalidArgument,
      "--initial-joint-state-stream is only valid with --input-format mcap");
  }
  if (require_urdf && result.input_format == InputFormat::Mcap && !initial_joint_state_stream_set) {
    result.initial_joint_state_stream = "/mc/ik/joint_states";
  }
  if (result.pairing_policy == data::PairingPolicy::Exact && result.nearest_tolerance_ns != 0) {
    throw data::DataError(
      data::DataErrorCode::InvalidArgument,
      "--nearest-tolerance-ms requires --pairing-policy nearest");
  }
  if (result.output_dir.empty()) {
    throw data::DataError(data::DataErrorCode::InvalidArgument, "--output-dir must be non-empty");
  }
  if (result.output_root.has_value() && result.output_root->empty()) {
    throw data::DataError(data::DataErrorCode::InvalidArgument, "--output-root must be non-empty");
  }
  if (result.output_dir_explicit && result.output_root.has_value()) {
    throw data::DataError(
      data::DataErrorCode::InvalidArgument,
      "--output-dir and --output-root are mutually exclusive");
  }
  if (result.output_dir_explicit && result.run_id.has_value()) {
    throw data::DataError(
      data::DataErrorCode::InvalidArgument, "--run-id cannot be combined with --output-dir");
  }
  if (
    !require_urdf && (result.output_root.has_value() || result.run_id.has_value() ||
                      result.visualization_mcap_path.has_value())) {
    throw data::DataError(
      data::DataErrorCode::InvalidArgument,
      "experiment output and visualization options are "
      "only valid for the IK runner");
  }
  return result;
}

std::string replayHelp(const std::string & program, bool include_urdf)
{
  std::ostringstream output;
  output << "Usage: " << program << " [options]\n";
  if (include_urdf) {
    output << "  --urdf <path>                    Robot URDF (required)\n"
           << "  --initial-joint-state-stream <name>\n"
           << "                                   First MCAP JointState "
              "initializes the robot\n"
           << "                                   (default /mc/ik/joint_states)\n"
           << "  --output-root <path>             Parent of an auto-named "
              "append-only E02 run\n"
           << "  --run-id <id>                    Override the auto-generated E02 "
              "run ID\n"
           << "  --launcher <path-or-id>          Launcher identity recorded in artifacts\n"
           << "  --ui tui|none                    User interface mode (default "
              "tui)\n"
           << "  --terminal-input on|off          Replay keyboard controls (default off)\n"
           << "  --viz foxglove|none              Visualization transport (default none)\n"
           << "  --host <address>                 Foxglove bind address (default "
              "127.0.0.1)\n"
           << "  --port <port>                    Foxglove port (default 8765)\n"
           << "  --mcap <path>                    Record visualization MCAP\n"
           << "  --no-mcap                        Disable visualization MCAP "
              "(default)\n";
  }
  output << "  --input <path>                   MCAP or CSV input (required)\n"
         << "  --input-format mcap|csv          Physical source backend\n"
         << "  --left-stream <name>             Left logical stream/topic\n"
         << "  --right-stream <name>            Right logical stream/topic\n"
         << "  --csv-mapping <json>             Optional CSV pose column mapping\n"
         << "  --timestamp-source "
            "header_stamp|log_time|publish_time|csv_timestamp\n"
         << "  --target-period-ms <ms>          Required fixed projected frame "
            "period\n"
         << "  --pairing-policy exact|nearest   Original-time pairing (default "
            "exact)\n"
         << "  --nearest-tolerance-ms <ms>      Nearest pairing tolerance\n"
         << "  --unmatched-policy error|drop_with_diagnostics\n"
         << "  --execution-mode batch|realtime\n"
         << "  --playback-rate <rate>           Positive replay rate (default 1)\n"
         << "  --output-dir <path>              Exact new artifact directory "
            "(legacy/manual)\n"
         << "  --help                           Show this help\n";
  return output.str();
}

std::string toString(InputFormat format) { return format == InputFormat::Mcap ? "mcap" : "csv"; }

LoadedReplay loadReplay(const ReplayOptions & options)
{
  std::unique_ptr<data::DataSource> source;
  data::TypedStream<data::StampedPose> left;
  data::TypedStream<data::StampedPose> right;
  LoadedReplay loaded;

  if (options.input_format == InputFormat::Mcap) {
    source = std::make_unique<data::McapSource>(options.input_path);
    const data::Ros2PoseStampedCdrDecoder pose_decoder;
    auto left_cursor = source->select({options.left_stream, std::nullopt, std::nullopt});
    auto right_cursor = source->select({options.right_stream, std::nullopt, std::nullopt});
    left = data::decodeStream(*left_cursor, options.left_stream, pose_decoder);
    right = data::decodeStream(*right_cursor, options.right_stream, pose_decoder);

    if (options.initial_joint_state_stream.has_value()) {
      const data::Ros2JointStateCdrDecoder joint_state_decoder;
      auto initial_cursor =
        source->select({*options.initial_joint_state_stream, std::nullopt, std::nullopt});
      auto initial_states = data::decodeStream(
        *initial_cursor, *options.initial_joint_state_stream, joint_state_decoder);
      loaded.initial_joint_state = std::move(initial_states.samples.at(0));
      loaded.initial_joint_state_decoder = initial_states.decoder_id;
      loaded.decoder_diagnostic_count += initial_states.diagnostics.size();
    }
  } else {
    source = std::make_unique<data::CsvSource>(
      options.input_path, std::vector<std::string>{options.left_stream, options.right_stream});
    data::CsvPoseMapping left_mapping;
    data::CsvPoseMapping right_mapping;
    if (options.csv_mapping_path.has_value()) {
      const auto mapping = loadJson(*options.csv_mapping_path);
      left_mapping =
        parseMapping(mapping, options.left_stream, "csv_pose:left", options.timestamp_source);
      right_mapping =
        parseMapping(mapping, options.right_stream, "csv_pose:right", options.timestamp_source);
    } else {
      left_mapping = defaultMapping("csv_pose:left", "left_", options.timestamp_source);
      right_mapping = defaultMapping("csv_pose:right", "right_", options.timestamp_source);
    }
    const data::CsvPoseDecoder left_decoder(std::move(left_mapping));
    const data::CsvPoseDecoder right_decoder(std::move(right_mapping));
    auto left_cursor = source->select({options.left_stream, std::nullopt, std::nullopt});
    auto right_cursor = source->select({options.right_stream, std::nullopt, std::nullopt});
    left = data::decodeStream(*left_cursor, options.left_stream, left_decoder);
    right = data::decodeStream(*right_cursor, options.right_stream, right_decoder);
  }

  data::DualArmProjectionConfig projection;
  projection.timestamp_source = options.timestamp_source;
  projection.pairing_policy = options.pairing_policy;
  projection.nearest_tolerance_ns = options.nearest_tolerance_ns;
  projection.unmatched_policy = options.unmatched_policy;
  projection.timestamp_projection = options.timestamp_projection;

  loaded.catalog = source->catalog();
  loaded.left_decoder = left.decoder_id;
  loaded.right_decoder = right.decoder_id;
  loaded.decoder_diagnostic_count += left.diagnostics.size() + right.diagnostics.size();
  loaded.timeline = data::makeDualArmTimeline(left, right, projection);
  return loaded;
}

void createOutputDirectory(const std::filesystem::path & output_dir)
{
  if (std::filesystem::exists(output_dir)) {
    throw data::DataError(
      data::DataErrorCode::Io, "refusing to overwrite output directory: " + output_dir.string());
  }
  std::filesystem::create_directories(output_dir);
}

void writeTextFile(const std::filesystem::path & path, const std::string & contents)
{
  std::ofstream output;
  output.exceptions(std::ios::failbit | std::ios::badbit);
  output.open(path, std::ios::binary);
  output << contents;
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
  const ReplayOptions & options, const LoadedReplay & loaded,
  const ReplayExecutionMetadata & execution, const std::string & trace_sha256)
{
  Json::Value manifest;
  manifest["schema_version"] = "ik_replay_manifest.v2";
  manifest["dependencies"]["mcap"] =
    "foxglove/mcap "
    "releases/cpp/v2.1.3@1420296ffcfdcde4b6894c0c1aba0ad083f93dde";
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
  manifest["execution"]["app"] = execution.app;
  manifest["execution"]["topology"] = execution.topology;
  manifest["execution"]["solver"] = execution.solver;
  manifest["execution"]["backend"] = execution.backend;
  manifest["execution"]["ui"] = options.ui_mode;
  manifest["execution"]["terminal_input"] = options.terminal_input_enabled;
  manifest["execution"]["visualization"] =
    options.visualization_enabled ? "foxglove" : "none";
  manifest["execution"]["rate_hz"] = execution.rate_hz;
  manifest["execution"]["red_rate_hz"] = execution.red_rate_hz;
  manifest["execution"]["yellow_rate_hz"] = execution.yellow_rate_hz;
  manifest["execution"]["hard_realtime"] = false;
  manifest["resolved_config"]["input_format"] = toString(options.input_format);
  manifest["resolved_config"]["timestamp_source"] = data::toString(options.timestamp_source);
  manifest["resolved_config"]["pairing_policy"] = data::toString(options.pairing_policy);
  manifest["resolved_config"]["execution_mode"] = data::toString(options.execution_mode);
  manifest["resolved_config"]["playback_rate"] = options.playback_rate;
  manifest["resolved_config"]["target_period_ns"] = asJsonInt64(options.target_period_ns);
  manifest["resolved_config"]["ui"] = options.ui_mode;
  manifest["resolved_config"]["terminal_input"] = options.terminal_input_enabled;
  manifest["resolved_config"]["visualization"] =
    options.visualization_enabled ? "foxglove" : "none";
  manifest["resolved_config"]["solver"] = execution.solver;
  manifest["resolved_config"]["backend"] = execution.backend;
  manifest["resolved_config"]["rate_hz"] = execution.rate_hz;
  manifest["resolved_config"]["red_rate_hz"] = execution.red_rate_hz;
  manifest["resolved_config"]["yellow_rate_hz"] = execution.yellow_rate_hz;
  manifest["statistics"]["frames"] = Json::UInt64(loaded.timeline.timeline.size());
  manifest["statistics"]["left_input"] = Json::UInt64(loaded.timeline.pairing.left_input_count);
  manifest["statistics"]["right_input"] = Json::UInt64(loaded.timeline.pairing.right_input_count);
  manifest["statistics"]["matched"] = Json::UInt64(loaded.timeline.pairing.matched_count);
  manifest["statistics"]["unmatched_left"] =
    Json::UInt64(loaded.timeline.pairing.unmatched_left_count);
  manifest["statistics"]["unmatched_right"] =
    Json::UInt64(loaded.timeline.pairing.unmatched_right_count);
  manifest["statistics"]["maximum_pair_delta_ns"] =
    asJsonInt64(loaded.timeline.pairing.maximum_pair_delta_ns);
  manifest["statistics"]["decoder_diagnostics"] = Json::UInt64(loaded.decoder_diagnostic_count);
  manifest["statistics"]["source_frames"] = Json::UInt64(loaded.timeline.timeline.size());
  manifest["statistics"]["consumed_frames"] = Json::UInt64(execution.consumed_frame_count);
  manifest["statistics"]["dropped_frames"] = Json::UInt64(execution.dropped_frame_count);
  manifest["statistics"]["deadline_misses"] = Json::UInt64(execution.deadline_miss_count);
  manifest["statistics"]["accepted_solves"] = Json::UInt64(execution.accepted_count);
  manifest["statistics"]["rejected_solves"] = Json::UInt64(execution.rejected_count);
  manifest["invocation"]["launcher"] =
    execution.launcher.empty() ? options.launcher : execution.launcher;
  const auto & original_argv =
    execution.original_argv.empty() ? options.original_argv : execution.original_argv;
  for (const auto & argument : original_argv) {
    manifest["invocation"]["argv"].append(argument);
  }
  for (const auto & [key, value] : execution.resolved_config) {
    manifest["resolved_config"][key] = value;
  }
  manifest["artifacts"]["trace.csv"]["sha256"] = trace_sha256;
  return manifest;
}

Json::Value makeReplayStatus(
  const LoadedReplay & loaded, const ReplayExecutionMetadata & execution, const std::string & state,
  const std::string & error)
{
  Json::Value status;
  status["schema_version"] = "ik_replay_status.v2";
  status["state"] = state;
  status["app"] = execution.app;
  status["source_frame_count"] = Json::UInt64(loaded.timeline.timeline.size());
  status["consumed_frame_count"] = Json::UInt64(execution.consumed_frame_count);
  status["dropped_frame_count"] = Json::UInt64(execution.dropped_frame_count);
  status["accepted_count"] = Json::UInt64(execution.accepted_count);
  status["rejected_count"] = Json::UInt64(execution.rejected_count);
  status["deadline_miss_count"] = Json::UInt64(execution.deadline_miss_count);
  if (!error.empty()) {
    status["error"] = error;
  }
  return status;
}

void writeReplayPlanArtifacts(const ReplayOptions & options, const LoadedReplay & loaded)
{
  createOutputDirectory(options.output_dir);
  std::ostringstream trace;
  trace << "sequence,original_logical_timestamp_ns,source_time_from_start_ns,"
           "projected_timestamp_ns,scheduled_time_from_start_ns,"
           "left_header_stamp_ns,left_log_time_ns,left_publish_time_ns,"
           "right_header_stamp_ns,right_log_time_ns,right_publish_time_ns\n";
  for (const auto & frame : loaded.timeline.timeline) {
    const auto scheduled = static_cast<std::int64_t>(
      std::llround(static_cast<long double>(frame.projected_time_ns) / options.playback_rate));
    trace << frame.sequence << ',' << frame.original_logical_time_ns << ','
          << frame.source_time_from_start_ns << ',' << frame.projected_time_ns << ',' << scheduled
          << ',' << optionalTimestamp(frame.value.left.time.header_stamp_ns) << ','
          << optionalTimestamp(frame.value.left.time.log_time_ns) << ','
          << optionalTimestamp(frame.value.left.time.publish_time_ns) << ','
          << optionalTimestamp(frame.value.right.time.header_stamp_ns) << ','
          << optionalTimestamp(frame.value.right.time.log_time_ns) << ','
          << optionalTimestamp(frame.value.right.time.publish_time_ns) << '\n';
  }
  const auto trace_path = options.output_dir / "trace.csv";
  writeTextFile(trace_path, trace.str());
  ReplayExecutionMetadata execution;
  execution.app = "mcl_replay_plan";
  execution.topology = "plan-only";
  const auto manifest = makeReplayManifest(options, loaded, execution, sha256_file(trace_path));
  writeTextFile(options.output_dir / "manifest.json", jsonString(manifest));
}

}  // namespace motion_control_lab::replay
