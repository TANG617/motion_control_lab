#include "replay_plan.hpp"

#include "adapters/data/decoder/csv_pose_decoder.hpp"
#include "adapters/data/decoder/decode_stream.hpp"
#include "adapters/data/decoder/ros2_pose_stamped_cdr_decoder.hpp"
#include "adapters/data/source/csv_source.hpp"
#include "adapters/data/source/mcap_source.hpp"
#include "motion_control_lab/sha256.hpp"

#include <json/json.h>

#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace motion_control_lab::replay_plan
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
  throw std::invalid_argument("unknown input format: " + value);
}

StatePolicy parseStatePolicy(const std::string & value)
{
  if (value == "previous_solution") {
    return StatePolicy::PreviousSolution;
  }
  if (value == "fixed_initial_state") {
    return StatePolicy::FixedInitialState;
  }
  throw std::invalid_argument("unknown state policy: " + value);
}

std::int64_t millisecondsToNanoseconds(const std::string & value)
{
  return static_cast<std::int64_t>(std::llround(std::stod(value) * 1'000'000.0));
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
    throw std::runtime_error(errors);
  }
  return result;
}

std::string optionalString(
  const Json::Value & object,
  const std::string & key,
  const std::string & default_value)
{
  return object.isMember(key) ? object[key].asString() : default_value;
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
  const auto & object = root["streams"][stream_name];
  const auto & columns = object["columns"];
  data::CsvPoseMapping mapping;
  mapping.decoder_id = decoder_id;
  mapping.timestamp_column = optionalString(object, "timestamp_column", "timestamp_ns");
  mapping.timestamp_target = object.isMember("timestamp_target")
                               ? data::parseTimestampSource(object["timestamp_target"].asString())
                               : timestamp_target;
  if (object.isMember("header_stamp_column")) {
    mapping.header_stamp_column = object["header_stamp_column"].asString();
  }
  if (object.isMember("log_time_column")) {
    mapping.log_time_column = object["log_time_column"].asString();
  }
  if (object.isMember("publish_time_column")) {
    mapping.publish_time_column = object["publish_time_column"].asString();
  }
  if (object.isMember("fixed_frame_id")) {
    mapping.fixed_frame_id = object["fixed_frame_id"].asString();
    mapping.frame_id_column.reset();
  } else {
    mapping.frame_id_column = optionalString(object, "frame_id_column", "frame_id");
  }
  mapping.x_column = optionalString(columns, "x", "x");
  mapping.y_column = optionalString(columns, "y", "y");
  mapping.z_column = optionalString(columns, "z", "z");
  mapping.qx_column = optionalString(columns, "qx", "qx");
  mapping.qy_column = optionalString(columns, "qy", "qy");
  mapping.qz_column = optionalString(columns, "qz", "qz");
  mapping.qw_column = optionalString(columns, "qw", "qw");
  return mapping;
}

std::string inputFormatName(InputFormat format)
{
  return format == InputFormat::Mcap ? "mcap" : "csv";
}

std::string statePolicyName(StatePolicy policy)
{
  return policy == StatePolicy::PreviousSolution
           ? "previous_solution"
           : "fixed_initial_state";
}

std::string optionalTimestamp(const std::optional<std::int64_t> & value)
{
  return value.has_value() ? std::to_string(*value) : std::string{};
}

std::string jsonString(const Json::Value & value)
{
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "  ";
  return Json::writeString(builder, value) + "\n";
}

void writeTextFile(const std::filesystem::path & path, const std::string & contents)
{
  std::ofstream output;
  output.exceptions(std::ios::failbit | std::ios::badbit);
  output.open(path, std::ios::binary);
  output << contents;
}

}  // namespace

Options parseOptions(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto value = [&]() -> std::string {
        if (index + 1 >= argc) {
          throw std::invalid_argument(argument + " requires a value");
        }
        return argv[++index];
      };
    if (argument == "--help" || argument == "-h") {
      options.help = true;
    } else if (argument == "--input") {
      options.input_path = value();
    } else if (argument == "--input-format") {
      options.input_format = parseInputFormat(value());
    } else if (argument == "--left-stream") {
      options.left_stream = value();
    } else if (argument == "--right-stream") {
      options.right_stream = value();
    } else if (argument == "--csv-mapping") {
      options.csv_mapping_path = value();
    } else if (argument == "--timestamp-source") {
      options.timestamp_source = data::parseTimestampSource(value());
    } else if (argument == "--timestamp-policy") {
      options.timestamp_projection.policy = data::parseTimestampPolicy(value());
    } else if (argument == "--period-ms") {
      options.timestamp_projection.period_ns = millisecondsToNanoseconds(value());
    } else if (argument == "--pairing-policy") {
      options.pairing_policy = data::parsePairingPolicy(value());
    } else if (argument == "--nearest-tolerance-ms") {
      options.nearest_tolerance_ns = millisecondsToNanoseconds(value());
    } else if (argument == "--unmatched-policy") {
      options.unmatched_policy = data::parseUnmatchedPolicy(value());
    } else if (argument == "--execution-mode") {
      options.execution_mode = data::parseExecutionMode(value());
    } else if (argument == "--playback-rate") {
      options.playback_rate = std::stod(value());
    } else if (argument == "--state-policy") {
      options.state_policy = parseStatePolicy(value());
    } else if (argument == "--servo-period-ms") {
      options.servo_period_ns = millisecondsToNanoseconds(value());
    } else if (argument == "--output-dir") {
      options.output_dir = value();
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  return options;
}

std::string help(const std::string & program)
{
  std::ostringstream output;
  output
    << "Usage: " << program << " [options]\n"
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
    << "  --playback-rate <rate>           Replay rate (default 1)\n"
    << "  --state-policy previous_solution|fixed_initial_state\n"
    << "  --servo-period-ms <ms>           Core control horizon (default 10)\n"
    << "  --output-dir <path>              New artifact directory\n"
    << "  --help                           Show this help\n";
  return output.str();
}

LoadedReplay load(const Options & options)
{
  std::unique_ptr<data::DataSource> source;
  data::TypedStream<data::StampedPose> left;
  data::TypedStream<data::StampedPose> right;
  LoadedReplay loaded;

  if (options.input_format == InputFormat::Mcap) {
    source = std::make_unique<data::McapSource>(options.input_path);
    const data::Ros2PoseStampedCdrDecoder decoder;
    auto left_cursor = source->select({options.left_stream, std::nullopt, std::nullopt});
    auto right_cursor = source->select({options.right_stream, std::nullopt, std::nullopt});
    left = data::decodeStream(*left_cursor, options.left_stream, decoder);
    right = data::decodeStream(*right_cursor, options.right_stream, decoder);
  } else {
    source = std::make_unique<data::CsvSource>(
      options.input_path, std::vector<std::string>{options.left_stream, options.right_stream});
    data::CsvPoseMapping left_mapping;
    data::CsvPoseMapping right_mapping;
    if (options.csv_mapping_path.has_value()) {
      const auto mapping = loadJson(*options.csv_mapping_path);
      left_mapping = parseMapping(
        mapping, options.left_stream, "csv_pose:left", options.timestamp_source);
      right_mapping = parseMapping(
        mapping, options.right_stream, "csv_pose:right", options.timestamp_source);
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
  loaded.decoder_diagnostic_count = left.diagnostics.size() + right.diagnostics.size();
  loaded.timeline = data::makeDualArmTimeline(left, right, projection);
  return loaded;
}

void writeArtifacts(const Options & options, const LoadedReplay & loaded)
{
  if (std::filesystem::exists(options.output_dir)) {
    throw std::runtime_error("refusing to overwrite output directory: " + options.output_dir.string());
  }
  std::filesystem::create_directories(options.output_dir);

  std::ostringstream trace;
  trace << "sequence,original_logical_timestamp_ns,source_time_from_start_ns,"
           "projected_timestamp_ns,scheduled_time_from_start_ns,"
           "left_header_stamp_ns,left_log_time_ns,left_publish_time_ns,"
           "right_header_stamp_ns,right_log_time_ns,right_publish_time_ns\n";
  for (const auto & frame : loaded.timeline.timeline) {
    const auto scheduled = static_cast<std::int64_t>(std::llround(
      static_cast<long double>(frame.projected_time_ns) / options.playback_rate));
    trace << frame.sequence << ',' << frame.original_logical_time_ns << ','
          << frame.source_time_from_start_ns << ',' << frame.projected_time_ns << ','
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

  Json::Value manifest;
  manifest["schema_version"] = "dual_arm_replay_manifest.v1";
  manifest["input"]["path"] = std::filesystem::absolute(options.input_path).string();
  manifest["input"]["sha256"] = sha256_file(options.input_path);
  manifest["input"]["format"] = inputFormatName(options.input_format);
  manifest["streams"]["left"] = options.left_stream;
  manifest["streams"]["right"] = options.right_stream;
  manifest["decoders"]["left"] = loaded.left_decoder;
  manifest["decoders"]["right"] = loaded.right_decoder;
  manifest["timestamp"]["source"] = data::toString(options.timestamp_source);
  manifest["timestamp"]["pairing_policy"] = data::toString(options.pairing_policy);
  manifest["timestamp"]["nearest_tolerance_ns"] = Json::Int64(options.nearest_tolerance_ns);
  manifest["timestamp"]["unmatched_policy"] = data::toString(options.unmatched_policy);
  manifest["timestamp"]["projection_policy"] = data::toString(options.timestamp_projection.policy);
  manifest["timestamp"]["period_ns"] = Json::Int64(options.timestamp_projection.period_ns);
  manifest["execution"]["mode"] = data::toString(options.execution_mode);
  manifest["execution"]["playback_rate"] = options.playback_rate;
  manifest["execution"]["state_policy"] = statePolicyName(options.state_policy);
  manifest["execution"]["servo_period_ns"] = Json::Int64(options.servo_period_ns);
  manifest["statistics"]["frames"] = Json::UInt64(loaded.timeline.timeline.size());
  manifest["statistics"]["left_input"] = Json::UInt64(loaded.timeline.pairing.left_input_count);
  manifest["statistics"]["right_input"] = Json::UInt64(loaded.timeline.pairing.right_input_count);
  manifest["statistics"]["matched"] = Json::UInt64(loaded.timeline.pairing.matched_count);
  manifest["statistics"]["unmatched_left"] =
    Json::UInt64(loaded.timeline.pairing.unmatched_left_count);
  manifest["statistics"]["unmatched_right"] =
    Json::UInt64(loaded.timeline.pairing.unmatched_right_count);
  manifest["statistics"]["maximum_pair_delta_ns"] =
    Json::Int64(loaded.timeline.pairing.maximum_pair_delta_ns);
  manifest["statistics"]["decoder_diagnostics"] =
    Json::UInt64(loaded.decoder_diagnostic_count);
  manifest["artifacts"]["trace.csv"]["sha256"] = sha256_file(trace_path);
  writeTextFile(options.output_dir / "manifest.json", jsonString(manifest));
}

}  // namespace motion_control_lab::replay_plan
