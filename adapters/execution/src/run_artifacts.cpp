#include "motion_control_lab/run_artifacts.hpp"

#include "motion_control_lab/sha256.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace motion_control_lab
{
namespace
{
void validate_identifier(const std::string& name, const std::string& value)
{
  if (value.empty() ||
      !std::all_of(value.begin(), value.end(), [](unsigned char character) {
        return std::isalnum(character) || character == '-' || character == '_' || character == '.';
      }))
  {
    throw std::invalid_argument(
      name + " must contain only letters, digits, dot, underscore, or hyphen");
  }
}

std::string utc_timestamp()
{
  const auto now = std::chrono::system_clock::now();
  const auto milliseconds =
    std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
  const std::time_t raw_time = std::chrono::system_clock::to_time_t(now);

  std::tm utc{};
#if defined(_WIN32)
  gmtime_s(&utc, &raw_time);
#else
  gmtime_r(&raw_time, &utc);
#endif

  std::ostringstream output;
  output << std::put_time(&utc, "%Y%m%dT%H%M%S") << std::setw(3) << std::setfill('0')
         << milliseconds.count() << "Z";
  return output.str();
}

void write_atomic(const std::filesystem::path& path, const std::string& contents)
{
  const auto temporary_path = path.string() + ".tmp";
  {
    std::ofstream stream(temporary_path, std::ios::binary | std::ios::trunc);
    if (!stream)
    {
      throw std::runtime_error("Unable to open temporary manifest: " + temporary_path);
    }
    stream << contents;
    stream.flush();
    if (!stream)
    {
      throw std::runtime_error("Unable to write temporary manifest: " + temporary_path);
    }
  }

  std::filesystem::rename(temporary_path, path);
}
}  // namespace

RunArtifacts::RunArtifacts(const std::filesystem::path& output_root, RunMetadata metadata)
  : metadata_(std::move(metadata)), run_directory_(output_root / metadata_.run_id)
{
  if (metadata_.run_id.empty() || metadata_.experiment_id.empty() || metadata_.arm_id.empty() ||
      metadata_.input_id.empty())
  {
    throw std::invalid_argument("Run metadata contains an empty required identifier");
  }
  validate_identifier("run_id", metadata_.run_id);
  validate_identifier("experiment_id", metadata_.experiment_id);
  validate_identifier("arm_id", metadata_.arm_id);
  validate_identifier("input_id", metadata_.input_id);

  std::filesystem::create_directories(output_root);
  if (!std::filesystem::create_directory(run_directory_))
  {
    throw std::runtime_error("Run directory already exists: " + run_directory_.string());
  }

  write_manifest("running", "running", "");
}

const std::filesystem::path& RunArtifacts::run_directory() const
{
  return run_directory_;
}

std::filesystem::path RunArtifacts::write_text(const std::filesystem::path& relative_path,
                                               const std::string& contents)
{
  if (finalized_)
  {
    throw std::logic_error("Cannot add artifacts to a finalized run");
  }
  validate_relative_path(relative_path);

  const auto destination = run_directory_ / relative_path;
  std::filesystem::create_directories(destination.parent_path());
  if (std::filesystem::exists(destination))
  {
    throw std::runtime_error("Refusing to overwrite run artifact: " + destination.string());
  }

  std::ofstream stream(destination, std::ios::binary);
  if (!stream)
  {
    throw std::runtime_error("Unable to create run artifact: " + destination.string());
  }
  stream << contents;
  stream.flush();
  if (!stream)
  {
    throw std::runtime_error("Unable to write run artifact: " + destination.string());
  }
  stream.close();

  outputs_[relative_path.generic_string()] = {
    sha256_file(destination),
    std::filesystem::file_size(destination),
  };
  return destination;
}

void RunArtifacts::finalize_completed()
{
  if (finalized_)
  {
    throw std::logic_error("Run has already been finalized");
  }
  write_manifest("completed", "completed", "");
  finalized_ = true;
}

void RunArtifacts::finalize_failed(const std::string& message)
{
  if (finalized_)
  {
    throw std::logic_error("Run has already been finalized");
  }
  write_manifest("failed", "failed", message);
  finalized_ = true;
}

void RunArtifacts::write_manifest(const std::string& status,
                                  const std::string& unit_status,
                                  const std::string& failure_message)
{
  Json::Value manifest(Json::objectValue);
  manifest["schema_version"] = "run_manifest.v1";
  manifest["run_id"] = metadata_.run_id;
  manifest["run_kind"] = "experiment";
  manifest["experiment_id"] = metadata_.experiment_id;
  manifest["status"] = status;

  manifest["definition"]["locator"] = metadata_.definition_locator;
  manifest["definition"]["sha256"] = metadata_.definition_sha256;
  manifest["definition"]["resolved"] = metadata_.resolved_definition;

  manifest["source_control"]["revision"] = metadata_.source_revision;
  manifest["source_control"]["dirty"] = metadata_.source_dirty;

  manifest["environment"]["runtime"] = metadata_.runtime;
  manifest["environment"]["placo_revision"] = metadata_.placo_revision;

  Json::Value input(Json::objectValue);
  input["id"] = metadata_.input_id;
  input["locator"] = metadata_.input_locator;
  input["sha256"] = metadata_.input_sha256;
  manifest["inputs"].append(input);

  Json::Value unit(Json::objectValue);
  unit["id"] = metadata_.arm_id + "/" + metadata_.input_id;
  unit["arm_id"] = metadata_.arm_id;
  unit["input_id"] = metadata_.input_id;
  unit["required"] = true;
  unit["status"] = unit_status;
  manifest["units"].append(unit);

  const bool failed = !failure_message.empty();
  manifest["failures"]["total"] = failed ? 1 : 0;
  manifest["failures"]["required"] = failed ? 1 : 0;
  if (failed)
  {
    manifest["failures"]["messages"].append(failure_message);
  }

  manifest["outputs"] = Json::Value(Json::objectValue);
  for (const auto& [relative_path, record] : outputs_)
  {
    manifest["outputs"][relative_path]["sha256"] = record.sha256;
    manifest["outputs"][relative_path]["size_bytes"] =
      static_cast<Json::UInt64>(record.size_bytes);
  }

  write_atomic(run_directory_ / "manifest.json", json_to_string(manifest));
}

void RunArtifacts::validate_relative_path(const std::filesystem::path& relative_path)
{
  if (relative_path.empty() || relative_path.is_absolute())
  {
    throw std::invalid_argument("Artifact path must be a non-empty relative path");
  }
  for (const auto& component : relative_path)
  {
    if (component == "..")
    {
      throw std::invalid_argument("Artifact path must remain inside the run directory");
    }
  }
}

std::string make_run_id(const std::string& definition_sha256)
{
  if (definition_sha256.size() < 12)
  {
    throw std::invalid_argument("Definition SHA-256 is too short");
  }
  return utc_timestamp() + "__" + definition_sha256.substr(0, 12);
}

std::string json_to_string(const Json::Value& value)
{
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "  ";
  return Json::writeString(builder, value) + "\n";
}

Json::Value load_json_file(const std::filesystem::path& path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream)
  {
    throw std::runtime_error("Unable to open JSON file: " + path.string());
  }

  Json::CharReaderBuilder builder;
  Json::Value root;
  std::string errors;
  if (!Json::parseFromStream(builder, stream, &root, &errors))
  {
    throw std::runtime_error("Invalid JSON in " + path.string() + ": " + errors);
  }
  return root;
}
}  // namespace motion_control_lab
