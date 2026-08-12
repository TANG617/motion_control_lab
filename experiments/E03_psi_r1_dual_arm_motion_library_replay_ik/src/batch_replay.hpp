#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace motion_control_lab::e03
{

struct ActionSnapshot
{
  std::string action_id;
  std::string file_name;
  std::filesystem::path path;
  std::uintmax_t size_bytes{};
  std::string sha256;
};

struct InputVerification
{
  bool unchanged{};
  std::string message;
};

std::vector<ActionSnapshot> scanMotionLibrary(const std::filesystem::path & library_directory);

InputVerification verifyActionInput(const ActionSnapshot & snapshot);

struct ActionExecutionRecord
{
  std::string action_id;
  bool succeeded{};
  std::string failure_stage;
  std::string failure_code;
  std::string failure_message;
  std::optional<std::uint64_t> failure_frame;
  std::size_t frames_planned{};
  std::size_t frames_attempted{};
  std::size_t frames_accepted{};
  std::size_t frames_rejected{};
  std::size_t left_input_count{};
  std::size_t right_input_count{};
  std::size_t matched_count{};
  std::size_t unmatched_left_count{};
  std::size_t unmatched_right_count{};
  std::int64_t maximum_pair_delta_ns{};
};

using ActionExecutor = std::function<ActionExecutionRecord(const ActionSnapshot &)>;

std::vector<ActionExecutionRecord> executeActionBatch(
  const std::vector<ActionSnapshot> & actions, const ActionExecutor & executor);

}  // namespace motion_control_lab::e03
