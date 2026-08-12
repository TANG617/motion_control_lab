#include "experiments/E03_psi_r1_dual_arm_motion_library_replay_ik/src/batch_replay.hpp"

#include <algorithm>
#include <cctype>
#include <set>
#include <stdexcept>
#include <system_error>
#include <utility>

#include "motion_control_lab/sha256.hpp"

namespace motion_control_lab::e03
{
namespace
{

bool isSafeActionId(const std::string & value)
{
  if (value.empty() || !std::isalnum(static_cast<unsigned char>(value.front()))) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return std::isalnum(character) || character == '.' || character == '_' || character == '-';
  });
}

}  // namespace

std::vector<ActionSnapshot> scanMotionLibrary(const std::filesystem::path & library_directory)
{
  std::error_code error;
  if (!std::filesystem::is_directory(library_directory, error) || error) {
    throw std::runtime_error("motion library is not a directory: " + library_directory.string());
  }

  std::vector<std::filesystem::directory_entry> entries;
  for (std::filesystem::directory_iterator iterator(library_directory, error), end;
       !error && iterator != end; iterator.increment(error)) {
    entries.push_back(*iterator);
  }
  if (error) {
    throw std::runtime_error("failed to scan motion library: " + error.message());
  }
  std::sort(entries.begin(), entries.end(), [](const auto & left, const auto & right) {
    return left.path().filename().generic_string() < right.path().filename().generic_string();
  });

  std::vector<ActionSnapshot> result;
  std::set<std::string> action_ids;
  for (const auto & entry : entries) {
    const auto status = entry.symlink_status(error);
    if (error) {
      throw std::runtime_error(
        "failed to inspect library entry " + entry.path().string() + ": " + error.message());
    }
    if (std::filesystem::is_symlink(status)) {
      throw std::runtime_error(
        "motion library must not contain symlinks: " + entry.path().filename().string());
    }
    if (!std::filesystem::is_regular_file(status) || entry.path().extension() != ".mcap") {
      continue;
    }

    const std::string action_id = entry.path().stem().string();
    if (!isSafeActionId(action_id)) {
      throw std::runtime_error(
        "unsafe action id from MCAP filename: " + entry.path().filename().string());
    }
    if (!action_ids.emplace(action_id).second) {
      throw std::runtime_error("duplicate action id: " + action_id);
    }
    const auto size_bytes = entry.file_size(error);
    if (error) {
      throw std::runtime_error(
        "failed to read MCAP size " + entry.path().string() + ": " + error.message());
    }
    result.push_back(ActionSnapshot{
      action_id, entry.path().filename().string(),
      std::filesystem::absolute(entry.path()).lexically_normal(), size_bytes,
      sha256_file(entry.path())});
  }
  return result;
}

InputVerification verifyActionInput(const ActionSnapshot & snapshot)
{
  try {
    std::error_code error;
    const auto status = std::filesystem::symlink_status(snapshot.path, error);
    if (error || !std::filesystem::is_regular_file(status) || std::filesystem::is_symlink(status)) {
      return {false, "input is no longer the snapshotted regular file"};
    }
    const auto size_bytes = std::filesystem::file_size(snapshot.path, error);
    if (error || size_bytes != snapshot.size_bytes) {
      return {false, "input size changed after inventory snapshot"};
    }
    if (sha256_file(snapshot.path) != snapshot.sha256) {
      return {false, "input SHA-256 changed after inventory snapshot"};
    }
    return {true, {}};
  } catch (const std::exception & error) {
    return {false, error.what()};
  }
}

std::vector<ActionExecutionRecord> executeActionBatch(
  const std::vector<ActionSnapshot> & actions, const ActionExecutor & executor)
{
  std::vector<ActionExecutionRecord> result;
  result.reserve(actions.size());
  for (const auto & action : actions) {
    try {
      auto record = executor(action);
      record.action_id = action.action_id;
      result.push_back(std::move(record));
    } catch (const std::exception & error) {
      ActionExecutionRecord record;
      record.action_id = action.action_id;
      record.failure_stage = "action_execution";
      record.failure_code = "unhandled_exception";
      record.failure_message = error.what();
      result.push_back(std::move(record));
    }
  }
  return result;
}

}  // namespace motion_control_lab::e03
