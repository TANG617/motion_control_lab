#pragma once

#include "options.hpp"
#include "solver.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace motion_control_lab::hierarchical_kinematics_step {

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

// Once CartesianAdmittance::step() has advanced its state, every downstream
// failure is fatal because there is no transactional rollback across the
// admittance, IK, JointPlanner, and executed-FK state machines.
RedFailureDisposition postAdmittanceFailureDisposition() noexcept;

// Synthetic replay hold frames represent a frozen calculation pipeline.  They
// must not advance the Cartesian planner, admittance, IK, or JointPlanner.
bool shouldFreezeControlPipeline(bool replay_joint_hold) noexcept;

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

} // namespace motion_control_lab::hierarchical_kinematics_step
