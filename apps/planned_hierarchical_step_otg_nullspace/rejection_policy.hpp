#pragma once

#include "options.hpp"
#include "solver.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace motion_control_lab::planned_hierarchical_step_otg_nullspace {

enum class RedFailureDisposition {
  None,
  RecoverableTeleopInfeasible,
  RecoverablePrimaryMaximumIterations,
  Fatal,
};

RedFailureDisposition
classifyRedIkFailure(SourceMode source_mode,
                     const motion_control::core::Status &status,
                     const SolverDiagnostics &diagnostics);

bool isRecoverableRedFailure(RedFailureDisposition disposition);

bool shouldSkipRejectedRevision(
    const std::optional<std::uint64_t> &rejected_revision,
    std::uint64_t current_revision);

struct ReplayRejectionObservation {
  SourceMode source_mode{SourceMode::Teleop};
  RedFailureDisposition disposition{RedFailureDisposition::None};
  std::optional<std::size_t> rejected_source_index;
  std::uint64_t rejected_revision{0U};
  std::optional<std::size_t> published_source_index;
  std::uint64_t published_revision{0U};
  std::size_t source_frame_count{0U};
};

bool isFinalReplayPrimaryMaximumIterationsRejection(
    const ReplayRejectionObservation &observation);

enum class ReplayRunState {
  Succeeded,
  Failed,
  Stopped,
};

struct ReplayRunDisposition {
  ReplayRunState state{ReplayRunState::Stopped};
  bool fail_process{false};
};

ReplayRunDisposition
decideReplayRunDisposition(bool recorded_fault, bool replay_completed,
                           std::size_t primary_max_iter_rejection_count);

const char *replayRunStateName(ReplayRunState state);

std::string primaryMaximumIterationsReplayFailureDetail(
    std::size_t rejection_count, std::uint64_t last_rejected_revision,
    std::string_view last_rejection_detail);

} // namespace motion_control_lab::planned_hierarchical_step_otg_nullspace
