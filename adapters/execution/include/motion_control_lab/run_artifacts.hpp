#pragma once

#include <filesystem>
#include <map>
#include <string>

#include <json/json.h>

namespace motion_control_lab
{
struct RunMetadata
{
  std::string run_id;
  std::string experiment_id;
  std::string arm_id;
  std::string input_id;
  std::string definition_locator;
  std::string definition_sha256;
  Json::Value resolved_definition;
  std::string input_locator;
  std::string input_sha256;
  std::string source_revision;
  bool source_dirty = true;
  std::string placo_revision;
  std::string runtime;
};

class RunArtifacts
{
public:
  RunArtifacts(const std::filesystem::path& output_root, RunMetadata metadata);

  const std::filesystem::path& run_directory() const;

  std::filesystem::path write_text(const std::filesystem::path& relative_path,
                                   const std::string& contents);

  void finalize_completed();
  void finalize_failed(const std::string& message);

private:
  struct OutputRecord
  {
    std::string sha256;
    std::uintmax_t size_bytes = 0;
  };

  void write_manifest(const std::string& status,
                      const std::string& unit_status,
                      const std::string& failure_message);
  static void validate_relative_path(const std::filesystem::path& relative_path);

  RunMetadata metadata_;
  std::filesystem::path run_directory_;
  std::map<std::string, OutputRecord> outputs_;
  bool finalized_ = false;
};

std::string make_run_id(const std::string& definition_sha256);
std::string json_to_string(const Json::Value& value);
Json::Value load_json_file(const std::filesystem::path& path);
}  // namespace motion_control_lab

