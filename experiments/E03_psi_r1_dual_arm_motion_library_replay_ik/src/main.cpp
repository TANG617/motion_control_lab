#include "contracts/data/data_error.hpp"
#include "contracts/visualization/foxglove_ik_v1.hpp"
#include "e03_build_config.hpp"
#include "experiments/E03_psi_r1_dual_arm_motion_library_replay_ik/src/batch_replay.hpp"
#include "experiments/E03_psi_r1_dual_arm_motion_library_replay_ik/src/legacy_replay/replay_ik_engine.hpp"
#include "motion_control_lab/run_artifacts.hpp"
#include "motion_control_lab/sha256.hpp"

#if MCL_WITH_REPLAY_VISUALIZATION
#include "components/visualization/preview_transport.hpp"
#include <motion_control_viz/render_batch.hpp>
#include <motion_control_viz/render_sink.hpp>
#endif

#include <Eigen/Geometry>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace
{
namespace mcl = motion_control_lab;
namespace e03 = motion_control_lab::e03;
namespace replay = motion_control_lab::replay;
namespace visualization_contract = motion_control_lab::contracts::foxglove_ik_v1;
#if MCL_WITH_REPLAY_VISUALIZATION
namespace mcv = motion_control::viz;
#endif

struct BatchOptions
{
  bool help{};
  std::filesystem::path urdf_path;
  std::filesystem::path library_directory{
    std::string(mcl::e03::build_config::kDefaultLibraryDirectory)};
  std::filesystem::path output_root{std::string(mcl::e03::build_config::kDefaultOutputRoot)};
  std::optional<std::string> run_id;
  bool visualize{};
  std::string visualization_host{"127.0.0.1"};
  std::uint16_t visualization_port{8765};
  double playback_rate{1.0};
  std::string launcher;
  std::vector<std::string> original_argv;
};

std::string jsonString(const Json::Value & value)
{
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "  ";
  return Json::writeString(builder, value) + "\n";
}

std::string utcNow()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  gmtime_r(&now_time, &utc);
  std::ostringstream output;
  output << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return output.str();
}

bool isSafeIdentifier(const std::string & value)
{
  return !value.empty() && std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return std::isalnum(character) || character == '.' || character == '_' || character == '-';
  });
}

double parsePositiveDouble(const std::string & value, const std::string & option)
{
  std::size_t parsed = 0;
  double result = 0.0;
  try {
    result = std::stod(value, &parsed);
  } catch (const std::exception &) {
    throw std::runtime_error(option + " requires a number");
  }
  if (parsed != value.size() || !std::isfinite(result) || result <= 0.0) {
    throw std::runtime_error(option + " must be finite and > 0");
  }
  return result;
}

std::uint16_t parsePort(const std::string & value)
{
  std::size_t parsed = 0;
  unsigned long result = 0;
  try {
    result = std::stoul(value, &parsed);
  } catch (const std::exception &) {
    throw std::runtime_error("--viz-port requires an integer");
  }
  if (parsed != value.size() || result == 0 || result > 65535) {
    throw std::runtime_error("--viz-port must be in [1, 65535]");
  }
  return static_cast<std::uint16_t>(result);
}

BatchOptions parseOptions(int argc, char ** argv)
{
  BatchOptions result;
  result.original_argv.assign(argv, argv + argc);
  bool playback_rate_explicit = false;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    const auto requireValue = [&]() -> std::string {
      if (index + 1 >= argc) {
        throw std::runtime_error(argument + " requires a value");
      }
      return argv[++index];
    };
    if (argument == "--help" || argument == "-h") {
      result.help = true;
    } else if (argument == "--urdf") {
      result.urdf_path = requireValue();
    } else if (argument == "--library-dir") {
      result.library_directory = requireValue();
    } else if (argument == "--output-root") {
      result.output_root = requireValue();
    } else if (argument == "--run-id") {
      result.run_id = requireValue();
      if (!isSafeIdentifier(*result.run_id)) {
        throw std::runtime_error(
          "--run-id must contain only letters, digits, "
          "dot, underscore, or hyphen");
      }
    } else if (argument == "--visualize") {
      result.visualize = true;
    } else if (argument == "--viz-host") {
      result.visualization_host = requireValue();
    } else if (argument == "--viz-port") {
      result.visualization_port = parsePort(requireValue());
    } else if (argument == "--playback-rate") {
      result.playback_rate = parsePositiveDouble(requireValue(), argument);
      playback_rate_explicit = true;
    } else if (argument == "--launcher") {
      result.launcher = requireValue();
    } else {
      throw std::runtime_error("unknown option: " + argument);
    }
  }
  if (result.help) {
    return result;
  }
  if (!std::filesystem::is_regular_file(result.urdf_path)) {
    throw std::runtime_error("--urdf must name an existing regular file");
  }
  if (result.library_directory.empty()) {
    throw std::runtime_error("--library-dir must be non-empty");
  }
  if (result.output_root.empty()) {
    throw std::runtime_error("--output-root must be non-empty");
  }
  if (playback_rate_explicit && !result.visualize) {
    throw std::runtime_error("--playback-rate requires --visualize");
  }
  return result;
}

std::string help(const std::string & program)
{
  return "Usage: " + program +
         " [options]\n"
         "  --urdf <path>          PSI R1 URDF (required)\n"
         "  --library-dir <path>   Direct-child MCAP motion library\n"
         "  --output-root <path>   Parent of append-only E03 runs\n"
         "  --run-id <id>          Override generated run ID\n"
         "  --visualize            Continuously stream the batch to Foxglove\n"
         "  --viz-host <address>   Foxglove bind address (default 127.0.0.1)\n"
         "  --viz-port <port>      Foxglove port (default 8765)\n"
         "  --playback-rate <rate> Visualization replay rate (default 1)\n"
         "  --launcher <path-or-id> Launcher identity recorded in artifacts\n"
         "  --help                 Show this help\n";
}

void atomicWrite(const std::filesystem::path & path, const std::string & contents)
{
  const auto temporary = path.parent_path() / (path.filename().string() + ".tmp");
  {
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    if (!output) {
      throw std::runtime_error("failed to create temporary artifact: " + temporary.string());
    }
    output << contents;
    output.flush();
    if (!output) {
      throw std::runtime_error("failed to write temporary artifact: " + temporary.string());
    }
  }
  std::error_code error;
  std::filesystem::rename(temporary, path, error);
  if (error) {
    std::filesystem::remove(temporary);
    throw std::runtime_error(
      "failed to atomically update " + path.string() + ": " + error.message());
  }
}

Json::Value actionSnapshotJson(const e03::ActionSnapshot & action)
{
  Json::Value result;
  result["action_id"] = action.action_id;
  result["file_name"] = action.file_name;
  result["path"] = action.path.string();
  result["size_bytes"] = Json::UInt64(action.size_bytes);
  result["sha256"] = action.sha256;
  return result;
}

Json::Value actionStatusJson(const e03::ActionExecutionRecord & record)
{
  Json::Value result;
  result["schema_version"] = "e03_action_status.v1";
  result["action_id"] = record.action_id;
  result["status"] = record.succeeded ? "completed" : "failed";
  result["failure"]["stage"] = record.failure_stage;
  result["failure"]["code"] = record.failure_code;
  result["failure"]["message"] = record.failure_message;
  if (record.failure_frame.has_value()) {
    result["failure"]["frame"] = Json::UInt64(*record.failure_frame);
  }
  result["frames"]["planned"] = Json::UInt64(record.frames_planned);
  result["frames"]["attempted"] = Json::UInt64(record.frames_attempted);
  result["frames"]["accepted"] = Json::UInt64(record.frames_accepted);
  result["frames"]["rejected"] = Json::UInt64(record.frames_rejected);
  result["pairing"]["left_input"] = Json::UInt64(record.left_input_count);
  result["pairing"]["right_input"] = Json::UInt64(record.right_input_count);
  result["pairing"]["matched"] = Json::UInt64(record.matched_count);
  result["pairing"]["unmatched_left"] = Json::UInt64(record.unmatched_left_count);
  result["pairing"]["unmatched_right"] = Json::UInt64(record.unmatched_right_count);
  result["pairing"]["maximum_pair_delta_ns"] = Json::Int64(record.maximum_pair_delta_ns);
  return result;
}

Json::Value isometryJson(const Eigen::Isometry3d & value)
{
  Json::Value result;
  result["translation"]["x"] = value.translation().x();
  result["translation"]["y"] = value.translation().y();
  result["translation"]["z"] = value.translation().z();
  const Eigen::Quaterniond orientation(value.linear());
  result["orientation_xyzw"]["x"] = orientation.x();
  result["orientation_xyzw"]["y"] = orientation.y();
  result["orientation_xyzw"]["z"] = orientation.z();
  result["orientation_xyzw"]["w"] = orientation.w();
  return result;
}

Json::Value actionManifestJson(
  const e03::ActionSnapshot & snapshot, const e03::ActionExecutionRecord & record,
  const replay::LegacyReplayIkOptions & replay_options, const replay::ReplayIkCaseResult * result,
  const std::string & trace_sha256, const std::string & status_sha256)
{
  Json::Value manifest;
  manifest["schema_version"] = "e03_action_manifest.v1";
  manifest["experiment_id"] = "E03";
  manifest["arm_id"] = "mcc_red_only";
  manifest["action"] = actionSnapshotJson(snapshot);
  manifest["status"] = record.succeeded ? "completed" : "failed";
  manifest["failure"]["stage"] = record.failure_stage;
  manifest["failure"]["code"] = record.failure_code;
  manifest["failure"]["message"] = record.failure_message;
  if (record.failure_frame.has_value()) {
    manifest["failure"]["frame"] = Json::UInt64(*record.failure_frame);
  }
  manifest["streams"]["joint_states"] = replay_options.initial_joint_state_stream.value_or("");
  manifest["streams"]["left_pose"] = replay_options.left_stream;
  manifest["streams"]["right_pose"] = replay_options.right_stream;
  manifest["timestamp"]["source"] = mcl::data::toString(replay_options.timestamp_source);
  manifest["timestamp"]["pairing_policy"] = mcl::data::toString(replay_options.pairing_policy);
  manifest["timestamp"]["nearest_tolerance_ns"] = Json::Int64(replay_options.nearest_tolerance_ns);
  manifest["timestamp"]["unmatched_policy"] = mcl::data::toString(replay_options.unmatched_policy);
  manifest["execution"]["state_policy"] = replay::toString(replay_options.state_policy);
  manifest["execution"]["servo_period_ns"] = Json::Int64(replay_options.servo_period_ns);
  manifest["execution"]["action_failure_policy"] = "stop_on_first_error";
  manifest["robot_model"]["urdf_path"] =
    std::filesystem::absolute(replay_options.urdf_path).lexically_normal().string();
  manifest["robot_model"]["urdf_sha256"] = mcl::sha256_file(replay_options.urdf_path);
  manifest["solver"]["profile"] = "Ordinary";
  manifest["solver"]["servo_mode"] = "ServoStep";
  manifest["target_pose"]["input_semantics"] = "tcp";
  manifest["target_pose"]["solver_semantics"] = "end_effector";
  manifest["target_pose"]["conversion"] = "end_effector_target=tcp_target*tcp_offset.inverse()";
  const auto & robot = mcl::r1RobotConfig();
  manifest["target_pose"]["tcp_offsets"]["left"] = isometryJson(robot.left_tcp_offset);
  manifest["target_pose"]["tcp_offsets"]["right"] = isometryJson(robot.right_tcp_offset);
  manifest["statistics"] = actionStatusJson(record)["frames"];
  manifest["pairing"] = actionStatusJson(record)["pairing"];
  if (result != nullptr) {
    manifest["decoders"]["left"] = result->loaded.left_decoder;
    manifest["decoders"]["right"] = result->loaded.right_decoder;
    manifest["decoders"]["joint_states"] = result->loaded.initial_joint_state_decoder;
    manifest["initial_state"]["source"] = "mcap_first_joint_state";
    manifest["initial_state"]["stream"] = replay_options.initial_joint_state_stream.value_or("");
    manifest["initial_state"]["sample_index"] = Json::UInt64(0);
    manifest["initial_state"]["velocity_source"] = "zero";
    for (const auto & name : robot.joint_names) {
      manifest["initial_state"]["joint_names"].append(name);
    }
    for (const double position : result->initial_positions) {
      manifest["initial_state"]["joint_positions"].append(position);
    }
  }
  manifest["artifacts"]["trace.csv"]["sha256"] = trace_sha256;
  manifest["artifacts"]["status.json"]["sha256"] = status_sha256;
  return manifest;
}

std::string dataErrorCodeString(mcl::data::DataErrorCode code)
{
  using Code = mcl::data::DataErrorCode;
  switch (code) {
    case Code::Io:
      return "data_io";
    case Code::InvalidArgument:
      return "invalid_argument";
    case Code::InvalidFormat:
      return "invalid_format";
    case Code::UnsupportedEncoding:
      return "unsupported_encoding";
    case Code::SchemaMismatch:
      return "schema_mismatch";
    case Code::DecodeFailure:
      return "decode_failure";
    case Code::MissingTimestamp:
      return "missing_timestamp";
    case Code::InvalidTimestamp:
      return "invalid_timestamp";
    case Code::NonMonotonicTimestamp:
      return "non_monotonic_timestamp";
    case Code::DuplicateTimestamp:
      return "duplicate_timestamp";
    case Code::UnmatchedSample:
      return "unmatched_sample";
    case Code::FrameMismatch:
      return "frame_mismatch";
  }
  return "data_error";
}

std::string dataErrorStage(mcl::data::DataErrorCode code)
{
  using Code = mcl::data::DataErrorCode;
  switch (code) {
    case Code::MissingTimestamp:
    case Code::InvalidTimestamp:
    case Code::NonMonotonicTimestamp:
    case Code::DuplicateTimestamp:
    case Code::UnmatchedSample:
    case Code::FrameMismatch:
      return "pairing";
    default:
      return "loading";
  }
}

replay::LegacyReplayIkOptions makeReplayOptions(
  const BatchOptions & batch, const e03::ActionSnapshot & action,
  const std::filesystem::path & action_output)
{
  replay::LegacyReplayIkOptions result;
  result.urdf_path = batch.urdf_path;
  result.input_path = action.path;
  result.input_format = replay::InputFormat::Mcap;
  result.left_stream = "/mc/ik/target/left_pose";
  result.right_stream = "/mc/ik/target/right_pose";
  result.initial_joint_state_stream = "/mc/ik/joint_states";
  result.timestamp_source = mcl::data::TimestampSource::HeaderStamp;
  result.pairing_policy = mcl::data::PairingPolicy::Nearest;
  result.nearest_tolerance_ns = 5'000'000;
  result.unmatched_policy = mcl::data::UnmatchedPolicy::Error;
  result.execution_mode =
    batch.visualize ? mcl::data::ExecutionMode::Realtime : mcl::data::ExecutionMode::Batch;
  result.playback_rate = batch.playback_rate;
  result.state_policy = replay::LegacyStatePolicy::PreviousSolution;
  result.servo_period_ns = 10'000'000;
  result.output_dir = action_output;
  return result;
}

std::string actionSummaryCsv(
  const std::vector<e03::ActionSnapshot> & actions,
  const std::vector<e03::ActionExecutionRecord> & records)
{
  std::map<std::string, const e03::ActionSnapshot *> snapshots;
  for (const auto & action : actions) {
    snapshots.emplace(action.action_id, &action);
  }
  std::ostringstream output;
  output << "action_id,file_name,size_bytes,sha256,status,failure_stage,failure_"
            "code,"
            "failure_frame,frames_planned,frames_attempted,frames_accepted,frames_"
            "rejected,"
            "solve_acceptance_ratio,left_input,right_input,matched,unmatched_left,"
            "unmatched_right,maximum_pair_delta_ns\n";
  output << std::setprecision(17);
  for (const auto & record : records) {
    const auto snapshot = snapshots.at(record.action_id);
    const double acceptance = record.frames_attempted == 0
                                ? 0.0
                                : static_cast<double>(record.frames_accepted) /
                                    static_cast<double>(record.frames_attempted);
    output << replay::csvEscape(record.action_id) << ',' << replay::csvEscape(snapshot->file_name)
           << ',' << snapshot->size_bytes << ',' << snapshot->sha256 << ','
           << (record.succeeded ? "completed" : "failed") << ','
           << replay::csvEscape(record.failure_stage) << ','
           << replay::csvEscape(record.failure_code) << ',';
    if (record.failure_frame.has_value()) {
      output << *record.failure_frame;
    }
    output << ',' << record.frames_planned << ',' << record.frames_attempted << ','
           << record.frames_accepted << ',' << record.frames_rejected << ',' << acceptance << ','
           << record.left_input_count << ',' << record.right_input_count << ','
           << record.matched_count << ',' << record.unmatched_left_count << ','
           << record.unmatched_right_count << ',' << record.maximum_pair_delta_ns << '\n';
  }
  return output.str();
}

struct AggregateMetrics
{
  std::size_t actions_total{};
  std::size_t actions_succeeded{};
  std::size_t actions_failed{};
  std::size_t frames_planned{};
  std::size_t frames_attempted{};
  std::size_t frames_accepted{};
  std::size_t frames_rejected{};
  std::size_t matched{};
  std::size_t unmatched_left{};
  std::size_t unmatched_right{};
};

AggregateMetrics aggregateMetrics(const std::vector<e03::ActionExecutionRecord> & records)
{
  AggregateMetrics result;
  result.actions_total = records.size();
  for (const auto & record : records) {
    result.actions_succeeded += record.succeeded ? 1U : 0U;
    result.actions_failed += record.succeeded ? 0U : 1U;
    result.frames_planned += record.frames_planned;
    result.frames_attempted += record.frames_attempted;
    result.frames_accepted += record.frames_accepted;
    result.frames_rejected += record.frames_rejected;
    result.matched += record.matched_count;
    result.unmatched_left += record.unmatched_left_count;
    result.unmatched_right += record.unmatched_right_count;
  }
  return result;
}

std::string metricsCsv(const AggregateMetrics & metrics)
{
  const double action_success_ratio =
    metrics.actions_total == 0
      ? 0.0
      : static_cast<double>(metrics.actions_succeeded) / static_cast<double>(metrics.actions_total);
  const double solve_acceptance_ratio = metrics.frames_attempted == 0
                                          ? 0.0
                                          : static_cast<double>(metrics.frames_accepted) /
                                              static_cast<double>(metrics.frames_attempted);
  std::ostringstream output;
  output << std::setprecision(17);
  output << "metric,value,unit\n";
  output << "action_count," << metrics.actions_total << ",actions\n";
  output << "successful_action_count," << metrics.actions_succeeded << ",actions\n";
  output << "failed_action_count," << metrics.actions_failed << ",actions\n";
  output << "action_success_ratio," << action_success_ratio << ",ratio\n";
  output << "frames_planned," << metrics.frames_planned << ",frames\n";
  output << "frames_attempted," << metrics.frames_attempted << ",frames\n";
  output << "accepted_solve_count," << metrics.frames_accepted << ",frames\n";
  output << "rejected_solve_count," << metrics.frames_rejected << ",frames\n";
  output << "aggregate_accepted_solve_ratio," << solve_acceptance_ratio << ",ratio\n";
  output << "paired_frame_count," << metrics.matched << ",frames\n";
  output << "unmatched_left_count," << metrics.unmatched_left << ",frames\n";
  output << "unmatched_right_count," << metrics.unmatched_right << ",frames\n";
  return output.str();
}

Json::Value failuresJson(
  const std::string & batch_failure_code, const std::string & batch_failure_message,
  const std::vector<e03::ActionExecutionRecord> & records)
{
  Json::Value result;
  result["schema_version"] = "e03_failures.v1";
  if (!batch_failure_code.empty()) {
    result["batch"]["code"] = batch_failure_code;
    result["batch"]["message"] = batch_failure_message;
  }
  result["actions"] = Json::arrayValue;
  for (const auto & record : records) {
    if (record.succeeded) {
      continue;
    }
    Json::Value failure;
    failure["action_id"] = record.action_id;
    failure["stage"] = record.failure_stage;
    failure["code"] = record.failure_code;
    failure["message"] = record.failure_message;
    if (record.failure_frame.has_value()) {
      failure["frame"] = Json::UInt64(*record.failure_frame);
    }
    result["actions"].append(std::move(failure));
  }
  return result;
}

std::string reportMarkdown(
  const std::string & run_id, const AggregateMetrics & metrics,
  const std::string & batch_failure_code, const std::vector<e03::ActionExecutionRecord> & records)
{
  const bool completed = batch_failure_code.empty() && metrics.actions_total > 0 &&
                         metrics.actions_failed == 0 && metrics.frames_rejected == 0;
  const double action_success_ratio =
    metrics.actions_total == 0
      ? 0.0
      : static_cast<double>(metrics.actions_succeeded) / static_cast<double>(metrics.actions_total);
  const double solve_acceptance_ratio = metrics.frames_attempted == 0
                                          ? 0.0
                                          : static_cast<double>(metrics.frames_accepted) /
                                              static_cast<double>(metrics.frames_attempted);
  std::ostringstream output;
  output << "# E03 batch replay IK report\n\n";
  output << "- Run: `" << run_id << "`\n";
  output << "- Status: **" << (completed ? "completed" : "failed") << "**\n";
  output << "- Actions: " << metrics.actions_succeeded << '/' << metrics.actions_total
         << " succeeded (" << std::setprecision(6) << action_success_ratio << ")\n";
  output << "- Solves: " << metrics.frames_accepted << '/' << metrics.frames_attempted
         << " accepted (" << solve_acceptance_ratio << ")\n";
  if (!batch_failure_code.empty()) {
    output << "- Batch failure: `" << batch_failure_code << "`\n";
  }
  output << "\n## Action results\n\n";
  output << "| Action | Status | Accepted / Attempted | First failure |\n";
  output << "|---|---:|---:|---|\n";
  for (const auto & record : records) {
    output << "| `" << record.action_id << "` | " << (record.succeeded ? "completed" : "failed")
           << " | " << record.frames_accepted << " / " << record.frames_attempted << " | ";
    if (record.succeeded) {
      output << "—";
    } else {
      output << '`' << record.failure_stage << ':' << record.failure_code << '`';
    }
    output << " |\n";
  }
  return output.str();
}

Json::Value outputInventory(const std::filesystem::path & run_directory)
{
  std::map<std::string, std::filesystem::path> paths;
  for (const auto & entry : std::filesystem::recursive_directory_iterator(run_directory)) {
    if (
      !entry.is_regular_file() || entry.path() == run_directory / "manifest.json" ||
      entry.path().extension() == ".tmp") {
      continue;
    }
    const auto relative = std::filesystem::relative(entry.path(), run_directory).generic_string();
    paths.emplace(relative, entry.path());
  }
  Json::Value result(Json::objectValue);
  for (const auto & [relative, path] : paths) {
    result[relative]["sha256"] = mcl::sha256_file(path);
    result[relative]["size_bytes"] = Json::UInt64(std::filesystem::file_size(path));
  }
  return result;
}

Json::Value baseRunManifest(
  const std::string & run_id, const Json::Value & resolved_definition,
  const std::string & definition_sha256, const std::string & started_at,
  const BatchOptions & options, const std::vector<e03::ActionSnapshot> & actions)
{
  Json::Value manifest;
  manifest["schema_version"] = "run_manifest.v1";
  manifest["run_id"] = run_id;
  manifest["run_kind"] = "experiment";
  manifest["experiment_id"] = "E03";
  manifest["definition"]["locator"] = std::string(mcl::e03::build_config::kDefinitionPath);
  manifest["definition"]["sha256"] = definition_sha256;
  manifest["definition"]["resolved"] = resolved_definition;
  manifest["source_control"]["revision"] = std::string(mcl::e03::build_config::kSourceRevision);
  manifest["source_control"]["dirty"] = mcl::e03::build_config::kSourceDirty;
  manifest["environment"]["runtime"] = std::string(mcl::e03::build_config::kRuntime);
  manifest["environment"]["visualization_enabled"] = options.visualize;
  manifest["environment"]["visualization_playback_rate"] = options.playback_rate;
  manifest["invocation"]["launcher"] = options.launcher;
  manifest["invocation"]["argv"] = Json::arrayValue;
  for (const auto & argument : options.original_argv) {
    manifest["invocation"]["argv"].append(argument);
  }
  manifest["resolved_config"]["urdf_path"] =
    std::filesystem::absolute(options.urdf_path).lexically_normal().string();
  manifest["resolved_config"]["library_directory"] =
    std::filesystem::absolute(options.library_directory).lexically_normal().string();
  manifest["resolved_config"]["output_root"] =
    std::filesystem::absolute(options.output_root).lexically_normal().string();
  manifest["resolved_config"]["visualization_enabled"] = options.visualize;
  manifest["resolved_config"]["visualization_host"] = options.visualization_host;
  manifest["resolved_config"]["visualization_port"] = options.visualization_port;
  manifest["resolved_config"]["playback_rate"] = options.playback_rate;
  manifest["started_at_utc"] = started_at;
  manifest["inputs"] = Json::arrayValue;
  Json::Value contract_input;
  contract_input["id"] = "psi_r1_dual_arm_motion_library_contract";
  contract_input["locator"] = std::string(mcl::e03::build_config::kInputContractPath);
  contract_input["sha256"] = mcl::sha256_file(
    std::filesystem::path{std::string(mcl::e03::build_config::kInputContractPath)});
  manifest["inputs"].append(std::move(contract_input));
  for (const auto & action : actions) {
    Json::Value input;
    input["id"] = "action:" + action.action_id;
    input["locator"] = action.path.string();
    input["sha256"] = action.sha256;
    input["size_bytes"] = Json::UInt64(action.size_bytes);
    manifest["inputs"].append(std::move(input));
  }
  Json::Value batch_unit;
  batch_unit["id"] = "batch/psi_r1_dual_arm_motion_library";
  batch_unit["arm_id"] = "mcc_red_only";
  batch_unit["input_id"] = "psi_r1_dual_arm_motion_library_contract";
  batch_unit["required"] = true;
  batch_unit["status"] = "running";
  manifest["units"].append(std::move(batch_unit));
  manifest["failures"]["total"] = 0;
  manifest["failures"]["required"] = 0;
  manifest["failures"]["messages"] = Json::arrayValue;
  manifest["outputs"] = Json::objectValue;
  return manifest;
}

void appendUnits(
  Json::Value & manifest, const std::vector<e03::ActionExecutionRecord> & records,
  bool batch_completed)
{
  manifest["units"] = Json::arrayValue;
  Json::Value batch_unit;
  batch_unit["id"] = "batch/psi_r1_dual_arm_motion_library";
  batch_unit["arm_id"] = "mcc_red_only";
  batch_unit["input_id"] = "psi_r1_dual_arm_motion_library_contract";
  batch_unit["required"] = true;
  batch_unit["status"] = batch_completed ? "completed" : "failed";
  manifest["units"].append(std::move(batch_unit));
  for (const auto & record : records) {
    Json::Value unit;
    unit["id"] = "mcc_red_only/" + record.action_id;
    unit["arm_id"] = "mcc_red_only";
    unit["input_id"] = "action:" + record.action_id;
    unit["required"] = true;
    unit["status"] = record.succeeded ? "completed" : "failed";
    unit["metrics"]["frames_planned"] = Json::UInt64(record.frames_planned);
    unit["metrics"]["frames_attempted"] = Json::UInt64(record.frames_attempted);
    unit["metrics"]["accepted_solves"] = Json::UInt64(record.frames_accepted);
    unit["metrics"]["rejected_solves"] = Json::UInt64(record.frames_rejected);
    if (!record.succeeded) {
      unit["failure"]["stage"] = record.failure_stage;
      unit["failure"]["code"] = record.failure_code;
      unit["failure"]["message"] = record.failure_message;
    }
    manifest["units"].append(std::move(unit));
  }
}

#if MCL_WITH_REPLAY_VISUALIZATION
std::uint64_t wallClockNanoseconds()
{
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::system_clock::now().time_since_epoch())
                                      .count());
}

mcv::Pose3d visualizationPose(const Eigen::Isometry3d & pose)
{
  Eigen::Quaterniond orientation(pose.linear());
  orientation.normalize();
  return {
    {pose.translation().x(), pose.translation().y(), pose.translation().z()},
    {orientation.x(), orientation.y(), orientation.z(), orientation.w()}};
}

mcv::RenderBatch visualizationBatch(const replay::ReplayIkVisualizationSample & sample)
{
  mcv::RenderBatch result;
  result.timestamp_ns = wallClockNanoseconds();
  result.poses = {
    {visualization_contract::kLeftInputTargetTopic, sample.left_target_frame_id,
     visualizationPose(sample.left_input_target)},
    {visualization_contract::kRightInputTargetTopic, sample.right_target_frame_id,
     visualizationPose(sample.right_input_target)},
    {visualization_contract::kLeftFkOutputTopic,
     sample.forward_kinematics_frame_id, visualizationPose(sample.left_end_effector_fk)},
    {visualization_contract::kRightFkOutputTopic,
     sample.forward_kinematics_frame_id, visualizationPose(sample.right_end_effector_fk)}};
  result.joint_states.push_back(mcv::JointStateSample{
    visualization_contract::kIkOutputJointStateTopic,
    sample.joint_names,
    sample.positions,
    sample.velocities});
  return result;
}
#endif

int execute(const BatchOptions & options)
{
  const std::filesystem::path definition_path{std::string(mcl::e03::build_config::kDefinitionPath)};
  const std::string definition_sha256 = mcl::sha256_file(definition_path);
  if (definition_sha256 != std::string(mcl::e03::build_config::kDefinitionSha256)) {
    throw std::runtime_error(
      "E03 definition changed after configuration; "
      "rerun CMake before replay");
  }
  const Json::Value experiment_definition = mcl::load_json_file(definition_path);
  const std::string run_id = options.run_id.value_or(mcl::make_run_id(definition_sha256));
  const auto run_directory = options.output_root / run_id;
  replay::createOutputDirectory(run_directory);
  std::filesystem::create_directories(run_directory / "definition");
  std::filesystem::create_directories(run_directory / "inputs");
  std::filesystem::create_directories(run_directory / "arms" / "mcc_red_only");
  std::filesystem::create_directories(run_directory / "evaluation");

  Json::Value resolved_definition;
  resolved_definition["schema_version"] = "e03_resolved_definition.v1";
  resolved_definition["experiment_definition"] = experiment_definition;
  resolved_definition["runtime"]["library_directory"] =
    std::filesystem::absolute(options.library_directory).lexically_normal().string();
  resolved_definition["runtime"]["output_root"] =
    std::filesystem::absolute(options.output_root).lexically_normal().string();
  resolved_definition["runtime"]["visualization_enabled"] = options.visualize;
  resolved_definition["runtime"]["visualization_playback_rate"] = options.playback_rate;
  resolved_definition["runtime"]["urdf_path"] =
    std::filesystem::absolute(options.urdf_path).lexically_normal().string();

  const std::string started_at = utcNow();
  Json::Value manifest =
    baseRunManifest(run_id, resolved_definition, definition_sha256, started_at, options, {});
  manifest["status"] = "running";
  atomicWrite(run_directory / "manifest.json", jsonString(manifest));
  replay::writeTextFile(
    run_directory / "definition" / "resolved.json", jsonString(resolved_definition));

  std::vector<e03::ActionSnapshot> actions;
  std::string batch_failure_code;
  std::string batch_failure_message;
  try {
    actions = e03::scanMotionLibrary(options.library_directory);
    if (actions.empty()) {
      batch_failure_code = "empty_motion_library";
      batch_failure_message = "motion library contains no direct-child *.mcap actions";
    }
  } catch (const std::exception & error) {
    batch_failure_code = "motion_library_scan_failed";
    batch_failure_message = error.what();
  }

  Json::Value inventory;
  inventory["schema_version"] = "e03_input_inventory.v1";
  inventory["library_directory"] =
    std::filesystem::absolute(options.library_directory).lexically_normal().string();
  inventory["snapshotted_at_utc"] = utcNow();
  inventory["selection"]["recursive"] = false;
  inventory["selection"]["pattern"] = "*.mcap";
  inventory["selection"]["accept_symlinks"] = false;
  inventory["actions"] = Json::arrayValue;
  for (const auto & action : actions) {
    inventory["actions"].append(actionSnapshotJson(action));
  }
  if (!batch_failure_code.empty()) {
    inventory["failure"]["code"] = batch_failure_code;
    inventory["failure"]["message"] = batch_failure_message;
  }
  replay::writeTextFile(run_directory / "inputs" / "inventory.json", jsonString(inventory));

  manifest =
    baseRunManifest(run_id, resolved_definition, definition_sha256, started_at, options, actions);
  manifest["status"] = "running";
  atomicWrite(run_directory / "manifest.json", jsonString(manifest));

#if !MCL_WITH_REPLAY_VISUALIZATION
  if (options.visualize && batch_failure_code.empty()) {
    batch_failure_code = "visualization_unavailable";
    batch_failure_message =
      "--visualize is unavailable in this build; configure with "
      "-DMCL_BUILD_E03_REPLAY_VISUALIZATION=ON";
  }
#endif

#if MCL_WITH_REPLAY_VISUALIZATION
  std::unique_ptr<mcv::RenderSink> visualization_sink;
  if (options.visualize && batch_failure_code.empty()) {
    try {
      mcl::PreviewSinkOptions sink_options;
      sink_options.enabled = true;
      sink_options.host = options.visualization_host;
      sink_options.port = options.visualization_port;
      visualization_sink = mcl::createPreviewSink(sink_options, "mcl_e03_batch_replay_ik");
      visualization_sink->open();
      std::cout << "Foxglove: " << visualization_sink->status() << '\n';
    } catch (const std::exception & error) {
      batch_failure_code = "visualization_setup_failed";
      batch_failure_message = error.what();
    }
  }
#endif

  std::uint64_t visualization_sequence = 0;
  const auto execute_action = [&](const e03::ActionSnapshot & action) {
    e03::ActionExecutionRecord record;
    record.action_id = action.action_id;
    const auto action_directory = run_directory / "arms" / "mcc_red_only" / action.action_id;
    std::filesystem::create_directories(action_directory);
    const auto replay_options = makeReplayOptions(options, action, action_directory);
    std::string trace = replay::replayIkTraceHeader();
    std::optional<replay::ReplayIkCaseResult> ik_result;

    if (!batch_failure_code.empty() && batch_failure_code != "empty_motion_library") {
      record.failure_stage = "batch_setup";
      record.failure_code = batch_failure_code;
      record.failure_message = batch_failure_message;
    } else {
      const auto before = e03::verifyActionInput(action);
      if (!before.unchanged) {
        record.failure_stage = "input_validation";
        record.failure_code = "input_changed";
        record.failure_message = before.message;
      } else {
        try {
          replay::ReplayIkExecutionConfig execution_config;
          execution_config.stop_on_first_error = true;
          execution_config.first_visualization_sequence = visualization_sequence;
#if MCL_WITH_REPLAY_VISUALIZATION
          if (visualization_sink) {
            execution_config.visualization_callback =
              [&](const replay::ReplayIkVisualizationSample & sample) {
                visualization_sink->write(visualizationBatch(sample));
                visualization_sequence = sample.sequence + 1;
              };
          }
#endif
          ik_result = replay::executeReplayIkCase(replay_options, execution_config);
          visualization_sequence =
            std::max(visualization_sequence, ik_result->next_visualization_sequence);
          trace = ik_result->trace_csv;
          record.frames_planned = ik_result->frames_planned;
          record.frames_attempted = ik_result->frames_attempted;
          record.frames_accepted = ik_result->accepted_solves;
          record.frames_rejected = ik_result->rejected_solves;
          record.left_input_count = ik_result->loaded.timeline.pairing.left_input_count;
          record.right_input_count = ik_result->loaded.timeline.pairing.right_input_count;
          record.matched_count = ik_result->loaded.timeline.pairing.matched_count;
          record.unmatched_left_count = ik_result->loaded.timeline.pairing.unmatched_left_count;
          record.unmatched_right_count = ik_result->loaded.timeline.pairing.unmatched_right_count;
          record.maximum_pair_delta_ns = ik_result->loaded.timeline.pairing.maximum_pair_delta_ns;
          if (!ik_result->completed()) {
            record.failure_stage = "solver";
            record.failure_code = ik_result->first_failure_code.empty()
                                    ? "incomplete_action"
                                    : ik_result->first_failure_code;
            record.failure_message = ik_result->first_failure_message.empty()
                                       ? "action did not attempt and accept every planned frame"
                                       : ik_result->first_failure_message;
            record.failure_frame = ik_result->first_failure_frame;
          }
        } catch (const mcl::data::DataError & error) {
          record.failure_stage = dataErrorStage(error.code());
          record.failure_code = dataErrorCodeString(error.code());
          record.failure_message = error.what();
        } catch (const std::exception & error) {
          record.failure_stage = "execution";
          record.failure_code = "replay_ik_error";
          record.failure_message = error.what();
        }

        const auto after = e03::verifyActionInput(action);
        if (!after.unchanged) {
          record.failure_stage = "input_validation";
          record.failure_code = "input_changed";
          record.failure_message = after.message;
          record.failure_frame.reset();
        }
      }
    }
    record.succeeded = record.failure_code.empty() &&
                       record.frames_attempted == record.frames_planned &&
                       record.frames_rejected == 0 && record.frames_planned > 0;

    const auto trace_path = action_directory / "trace.csv";
    replay::writeTextFile(trace_path, trace);
    const auto status_path = action_directory / "status.json";
    replay::writeTextFile(status_path, jsonString(actionStatusJson(record)));
    const auto action_manifest = actionManifestJson(
      action, record, replay_options, ik_result.has_value() ? &*ik_result : nullptr,
      mcl::sha256_file(trace_path), mcl::sha256_file(status_path));
    replay::writeTextFile(action_directory / "manifest.json", jsonString(action_manifest));
    return record;
  };

  std::vector<e03::ActionExecutionRecord> records;
  if (!actions.empty()) {
    records = e03::executeActionBatch(actions, execute_action);
  }

#if MCL_WITH_REPLAY_VISUALIZATION
  if (visualization_sink) {
    visualization_sink->flush();
    visualization_sink->close();
  }
#endif

  const auto metrics = aggregateMetrics(records);
  replay::writeTextFile(
    run_directory / "evaluation" / "action_summary.csv", actionSummaryCsv(actions, records));
  replay::writeTextFile(run_directory / "evaluation" / "metrics.csv", metricsCsv(metrics));
  replay::writeTextFile(
    run_directory / "evaluation" / "failures.json",
    jsonString(failuresJson(batch_failure_code, batch_failure_message, records)));
  replay::writeTextFile(
    run_directory / "evaluation" / "report.md",
    reportMarkdown(run_id, metrics, batch_failure_code, records));

  const bool completed = batch_failure_code.empty() && !records.empty() &&
                         metrics.actions_failed == 0 && metrics.frames_rejected == 0;
  appendUnits(manifest, records, completed);
  manifest["status"] = completed ? "completed" : "failed";
  manifest["ended_at_utc"] = utcNow();
  manifest["failures"]["total"] =
    Json::UInt64(metrics.actions_failed + (batch_failure_code.empty() ? 0U : 1U));
  manifest["failures"]["required"] = manifest["failures"]["total"];
  manifest["failures"]["messages"] = Json::arrayValue;
  if (!completed) {
    if (!batch_failure_code.empty()) {
      manifest["failure"]["code"] = batch_failure_code;
      manifest["failure"]["message"] = batch_failure_message;
      manifest["failures"]["messages"].append(batch_failure_message);
    } else {
      manifest["failure"]["code"] = "one_or_more_actions_failed";
      manifest["failure"]["message"] = "at least one action failed or rejected a solve";
    }
    for (const auto & record : records) {
      if (!record.succeeded) {
        manifest["failures"]["messages"].append(record.action_id + ": " + record.failure_message);
      }
    }
  }
  manifest["outputs"] = outputInventory(run_directory);
  atomicWrite(run_directory / "manifest.json", jsonString(manifest));

  std::cout << "actions=" << metrics.actions_total << " succeeded=" << metrics.actions_succeeded
            << " accepted=" << metrics.frames_accepted << '/' << metrics.frames_attempted
            << " output=" << run_directory.string() << '\n';
  return completed ? EXIT_SUCCESS : EXIT_FAILURE;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = parseOptions(argc, argv);
    if (options.help) {
      std::cout << help(argv[0]);
      return EXIT_SUCCESS;
    }
    return execute(options);
  } catch (const std::exception & error) {
    std::cerr << "mcl_e03_batch_replay_ik: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
