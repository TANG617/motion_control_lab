#pragma once

#include <filesystem>
#include <fstream>
#include <json/json.h>
#include <stdexcept>
#include <string>
#include <set>
#include "motion_control_lab/sha256.hpp"

namespace motion_control_lab::execution {
// Data and provenance only. App configuration and execution remain app-owned.
struct Request {
  Json::Value document, input;
  std::filesystem::path path, output;
  std::string sha256;
  const Json::Value &config() const { return document["app_config"]; }
};
inline Json::Value readJson(const std::filesystem::path &path) {
  std::ifstream stream(path);
  stream.exceptions(std::ios::badbit);
  if (!stream) throw std::runtime_error("cannot open " + path.string());
  Json::CharReaderBuilder builder;
  builder["rejectDupKeys"] = true;
  Json::Value value;
  std::string errors;
  if (!Json::parseFromStream(builder, stream, &value, &errors))
    throw std::runtime_error(errors);
  return value;
}
inline void writeJson(const std::filesystem::path &path, const Json::Value &value) {
  std::ofstream stream(path);
  stream.exceptions(std::ios::badbit | std::ios::failbit);
  stream << value << '\n';
}
inline Request readRequest(const std::filesystem::path &path,
                           const std::string &app_id) {
  Request r;
  r.path = std::filesystem::absolute(path);
  r.document = readJson(path);
  r.sha256 = sha256_file(path);
  const auto &d = r.document;
  const std::set<std::string> keys{"schema_version", "app_id", "execution_structure", "input", "app_config", "execution", "observation", "output_dir", "tracking"};
  for (const auto &key : d.getMemberNames())
    if (!keys.count(key)) throw std::runtime_error("unknown execution request field: " + key);
  if (d["schema_version"] != "execution_request.v1" || d["app_id"] != app_id)
    throw std::runtime_error("execution request schema or app identity mismatch");
  for (const auto *key : {"app_config", "execution", "observation", "tracking", "input"})
    if (!d[key].isObject()) throw std::runtime_error(std::string("missing object: ") + key);
  if (!d["execution_structure"].isString() || !d["output_dir"].isString())
    throw std::runtime_error("missing execution_structure or output_dir");
  const std::filesystem::path input = d["input"]["path"].asString();
  const std::set<std::string> input_keys{"path", "sha256", "format"};
  for (const auto &key : d["input"].getMemberNames())
    if (!input_keys.count(key)) throw std::runtime_error("unknown input reference field: " + key);
  const auto format = d["input"]["format"].asString();
  if (format != "json" && format != "csv" && format != "mcap")
    throw std::runtime_error("input format must be json, csv or mcap");
  r.output = d["output_dir"].asString();
  if (!input.is_absolute() || !r.output.is_absolute())
    throw std::runtime_error("request input and output must be absolute paths");
  if (d["input"]["sha256"].asString() != sha256_file(input))
    throw std::runtime_error("input sha256 mismatch: " + input.string());
  if (format == "json") {
    r.input = readJson(input);
    if (r.input.isMember("model")) {
      const auto &model = r.input["model"];
      const std::filesystem::path locator = model["locator"].asString();
      if (!locator.is_absolute() || model["sha256"].asString() != sha256_file(locator))
        throw std::runtime_error("model sha256 mismatch or non-absolute locator");
    }
  }
  return r;
}
inline void beginOutput(const Request &r, const Json::Value &resolved,
                        const Json::Value &capabilities) {
  if (std::filesystem::exists(r.output))
    throw std::runtime_error("output already exists: " + r.output.string());
  std::filesystem::create_directories(r.output);
  writeJson(r.output / "execution_request.json", r.document);
  writeJson(r.output / "resolved_options.json", resolved);
  writeJson(r.output / "capabilities.json", capabilities);
  Json::Value binding;
  binding["schema_version"] = "execution_binding.v1";
  binding["request_sha256"] = r.sha256;
  binding["input"] = r.document["input"];
  binding["tracking"] = r.document["tracking"];
  writeJson(r.output / "binding.json", binding);
}
// Public request mode permits only a read-only resolved-options modifier.
inline bool isRequest(int argc, char **argv) {
  return argc > 1 && std::string(argv[1]) == "--request";
}
inline bool requestDumpOnly(int argc, char **argv) {
  if (argc == 3) return false;
  if (argc == 4 && std::string(argv[3]) == "--dump-resolved-options") return true;
  throw std::runtime_error("use --request FILE [--dump-resolved-options]; CLI overrides are forbidden");
}
} // namespace motion_control_lab::execution
