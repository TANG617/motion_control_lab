#include "../execution.hpp"
#include <stdexcept>
#include <unistd.h>
using namespace motion_control_lab::hierarchical_kinematics_step;
int main() {
  auto options = profileDefaults(Profile::PlannedOtg);
  options.replay.emplace();
  // Source hash is not used to manufacture a worker plan.
  options.replay->input_path = __FILE__;
  const auto plan = replayReleasePlan(options, 11);
  if (!plan["planned_worker_releases"].isNull() ||
      plan["source_frame_count"].asUInt64() != 11)
    throw std::runtime_error(
        "finite source frames must not become fixed worker releases");
  motion_control_lab::PeriodicWorkerStatistics red, yellow;
  red.iteration_count = 23;
  red.deadline_miss_count = 3;
  red.skipped_release_count = 7;
  yellow.iteration_count = 4;
  const auto failed = replayReleaseCounts(options, red, yellow, 11, 3, 1, 3,
                                          false, true, false);
  if (failed["stop_state"] != "fault" ||
      failed["source"]["not_selected_suffix_count"].asUInt64() != 7 ||
      !failed["source"]["partition_consistent"].asBool())
    throw std::runtime_error("actual source cursor and native selection counts "
                             "must retain failure suffix");
  if (failed["workers"]["red"]["callback_iteration_count"].asUInt64() != 23 ||
      failed["workers"]["red"]["raw_skipped_release_count"].asUInt64() != 7 ||
      !failed["workers"]["red"]["exact_all_release_denominator"].isNull())
    throw std::runtime_error("native skip counts cannot be relabeled as an "
                             "exact release denominator");
  const auto completed = replayReleaseCounts(options, red, yellow, 11, 11, 0,
                                             10, true, false, false);
  if (completed["source"]["not_selected_suffix_count"].asUInt64() != 0 ||
      completed["stop_state"] != "replay_completed" ||
      !completed["worker_not_run_releases"].isNull())
    throw std::runtime_error(
        "completion does not establish a predeclared control horizon");
  const auto output = std::filesystem::temp_directory_path() /
                      ("mcl-replay-accounting-" + std::to_string(getpid()));
  std::filesystem::create_directories(output / "replay");
  motion_control_lab::execution::writeJson(
      output / "replay/release_counts.json", completed);
  writeReplaySummary(output, 0);
  const auto summary =
      motion_control_lab::execution::readJson(output / "summary.json");
  const auto stored_counts = motion_control_lab::execution::readJson(
      output / "replay/release_counts.json");
  if (summary["release_accounting"] != stored_counts)
    throw std::runtime_error(
        "replay summary must include exact joined-worker accounting");
  std::filesystem::remove_all(output);
  const auto recorded =
      replayReleaseCounts(options, red, yellow, 11, 1, 0, 0, true, false, true);
  if (!recorded["source"]["not_selected_suffix_count"].isNull())
    throw std::runtime_error(
        "recorded reference coverage is separate from ReplaySource selection");
}
