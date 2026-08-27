#include "rejection_policy.hpp"

#include <sstream>

namespace motion_control_lab::planned_hierarchical_step_otg_nullspace {

namespace mcc = motion_control::core;

RedFailureDisposition
classifyRedIkFailure(SourceMode source_mode, const mcc::Status &status,
                     const SolverDiagnostics &diagnostics) {
  if (status.ok()) {
    return RedFailureDisposition::None;
  }

  if (diagnostics.hierarchical) {
    for (const auto &pass : diagnostics.hierarchy.passes) {
      if (pass.pass == mcc::HierarchicalSolvePass::Primary && pass.attempted &&
          !pass.succeeded &&
          pass.backend_status == mcc::QpSolveStatus::MaximumIterations) {
        return RedFailureDisposition::RecoverablePrimaryMaximumIterations;
      }
    }
  }

  if (source_mode == SourceMode::Teleop &&
      status.code == mcc::StatusCode::Infeasible) {
    return RedFailureDisposition::RecoverableTeleopInfeasible;
  }
  return RedFailureDisposition::Fatal;
}

bool isRecoverableRedFailure(RedFailureDisposition disposition) {
  return disposition == RedFailureDisposition::RecoverableTeleopInfeasible ||
         disposition ==
             RedFailureDisposition::RecoverablePrimaryMaximumIterations;
}

bool shouldSkipRejectedRevision(
    const std::optional<std::uint64_t> &rejected_revision,
    std::uint64_t current_revision) {
  return rejected_revision.has_value() &&
         current_revision == *rejected_revision;
}

bool isFinalReplayPrimaryMaximumIterationsRejection(
    const ReplayRejectionObservation &observation) {
  return observation.source_mode == SourceMode::Replay &&
         observation.disposition ==
             RedFailureDisposition::RecoverablePrimaryMaximumIterations &&
         observation.source_frame_count > 0U &&
         observation.rejected_source_index.has_value() &&
         observation.published_source_index.has_value() &&
         *observation.rejected_source_index ==
             *observation.published_source_index &&
         *observation.rejected_source_index + 1U ==
             observation.source_frame_count &&
         observation.rejected_revision == observation.published_revision;
}

ReplayRunDisposition
decideReplayRunDisposition(bool recorded_fault, bool replay_completed,
                           std::size_t primary_max_iter_rejection_count) {
  if (recorded_fault ||
      (replay_completed && primary_max_iter_rejection_count > 0U)) {
    return {ReplayRunState::Failed, true};
  }
  if (replay_completed) {
    return {ReplayRunState::Succeeded, false};
  }
  return {ReplayRunState::Stopped, false};
}

const char *replayRunStateName(ReplayRunState state) {
  switch (state) {
  case ReplayRunState::Succeeded:
    return "succeeded";
  case ReplayRunState::Failed:
    return "failed";
  case ReplayRunState::Stopped:
    return "stopped";
  }
  return "stopped";
}

std::string primaryMaximumIterationsReplayFailureDetail(
    std::size_t rejection_count, std::uint64_t last_rejected_revision,
    std::string_view last_rejection_detail) {
  std::ostringstream detail;
  detail << "replay completed with recoverable Primary MAX_ITER rejections="
         << rejection_count
         << " last_rejected_target_revision=" << last_rejected_revision;
  if (!last_rejection_detail.empty()) {
    detail << " last_rejection=" << last_rejection_detail;
  }
  return detail.str();
}

} // namespace motion_control_lab::planned_hierarchical_step_otg_nullspace
