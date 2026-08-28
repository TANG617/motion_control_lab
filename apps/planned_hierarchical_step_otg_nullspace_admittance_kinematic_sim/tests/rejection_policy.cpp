#include "../rejection_policy.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace app = motion_control_lab::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim;
namespace mcc = motion_control::core;

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

app::SolverDiagnostics diagnosticsFor(mcc::HierarchicalSolvePass solve_pass,
                                      mcc::QpSolveStatus status,
                                      bool attempted = true,
                                      bool succeeded = false) {
  app::SolverDiagnostics diagnostics;
  diagnostics.hierarchical = true;
  const std::size_t index =
      solve_pass == mcc::HierarchicalSolvePass::Primary
          ? 0U
          : (solve_pass == mcc::HierarchicalSolvePass::Secondary
                 ? 1U
                 : (solve_pass == mcc::HierarchicalSolvePass::Tertiary ? 2U
                                                                       : 3U));
  auto &pass = diagnostics.hierarchy.passes.at(index);
  pass.pass = solve_pass;
  pass.attempted = attempted;
  pass.succeeded = succeeded;
  pass.backend_status = status;
  return diagnostics;
}

void testFailureClassification() {
  const mcc::Status solver_error{mcc::StatusCode::SolverError,
                                 "PROXQP_MAX_ITER_REACHED"};
  const mcc::Status infeasible{mcc::StatusCode::Infeasible,
                               "primal infeasible"};

  auto primary_max_iter = diagnosticsFor(mcc::HierarchicalSolvePass::Primary,
                                         mcc::QpSolveStatus::MaximumIterations);
  require(app::classifyRedIkFailure(app::SourceMode::Teleop, solver_error,
                                    primary_max_iter) ==
              app::RedFailureDisposition::RecoverablePrimaryMaximumIterations,
          "Teleop Primary MAX_ITER must be recoverable");
  require(app::classifyRedIkFailure(app::SourceMode::Replay, solver_error,
                                    primary_max_iter) ==
              app::RedFailureDisposition::RecoverablePrimaryMaximumIterations,
          "Replay Primary MAX_ITER must be recoverable");

  primary_max_iter.hierarchy.passes.at(0).last_iterate_available = true;
  primary_max_iter.maximum_hard_violation = 100.0;
  primary_max_iter.hierarchy.passes.at(0).constraint_violations.push_back({});
  require(app::classifyRedIkFailure(app::SourceMode::Replay, solver_error,
                                    primary_max_iter) ==
              app::RedFailureDisposition::RecoverablePrimaryMaximumIterations,
          "Primary MAX_ITER recovery must not depend on iterate quality");

  app::SolverDiagnostics plain_diagnostics;
  require(app::classifyRedIkFailure(app::SourceMode::Teleop, infeasible,
                                    plain_diagnostics) ==
              app::RedFailureDisposition::RecoverableTeleopInfeasible,
          "Teleop infeasible must remain recoverable");
  require(app::classifyRedIkFailure(app::SourceMode::Replay, infeasible,
                                    plain_diagnostics) ==
              app::RedFailureDisposition::Fatal,
          "Replay infeasible must remain fatal");

  for (const auto solve_pass : {mcc::HierarchicalSolvePass::Secondary,
                                mcc::HierarchicalSolvePass::Tertiary,
                                mcc::HierarchicalSolvePass::Terminal}) {
    const auto diagnostics =
        diagnosticsFor(solve_pass, mcc::QpSolveStatus::MaximumIterations);
    require(app::classifyRedIkFailure(app::SourceMode::Replay, solver_error,
                                      diagnostics) ==
                app::RedFailureDisposition::Fatal,
            "non-Primary MAX_ITER must remain fatal");
  }

  const auto numerical_failure =
      diagnosticsFor(mcc::HierarchicalSolvePass::Primary,
                     mcc::QpSolveStatus::NumericalFailure);
  require(app::classifyRedIkFailure(app::SourceMode::Replay, solver_error,
                                    numerical_failure) ==
              app::RedFailureDisposition::Fatal,
          "Primary numerical failure must remain fatal");
  for (const auto backend_status :
       {mcc::QpSolveStatus::PrimalInfeasible,
        mcc::QpSolveStatus::DualInfeasible, mcc::QpSolveStatus::NotRun,
        mcc::QpSolveStatus::Optimal}) {
    const auto diagnostics =
        diagnosticsFor(mcc::HierarchicalSolvePass::Primary, backend_status);
    require(app::classifyRedIkFailure(app::SourceMode::Replay, solver_error,
                                      diagnostics) ==
                app::RedFailureDisposition::Fatal,
            "non-MAX_ITER Primary failure must remain fatal");
  }
  const auto not_attempted =
      diagnosticsFor(mcc::HierarchicalSolvePass::Primary,
                     mcc::QpSolveStatus::MaximumIterations, false, false);
  require(app::classifyRedIkFailure(app::SourceMode::Replay, solver_error,
                                    not_attempted) ==
              app::RedFailureDisposition::Fatal,
          "unattempted Primary pass must remain fatal");
  require(app::classifyRedIkFailure(app::SourceMode::Replay, mcc::Status{},
                                    primary_max_iter) ==
              app::RedFailureDisposition::None,
          "successful status must not be classified as a rejection");
}

void testRejectedRevisionPolicy() {
  require(app::shouldSkipRejectedRevision(7U, 7U),
          "the same rejected revision must not be re-solved");
  require(!app::shouldSkipRejectedRevision(7U, 8U),
          "a newer revision must be allowed to solve");
  require(!app::shouldSkipRejectedRevision(std::nullopt, 7U),
          "an absent rejection must not block a solve");
}

void testAdmittanceTransactionPolicy() {
  require(app::postAdmittanceFailureDisposition() ==
              app::RedFailureDisposition::Fatal,
          "every failure after an admittance step must be fatal");
  require(app::shouldFreezeControlPipeline(true),
          "a replay hold must freeze the complete control pipeline");
  require(!app::shouldFreezeControlPipeline(false),
          "a normal target must advance the complete control pipeline");
}

void testReplayCompletionPolicy() {
  app::ReplayRejectionObservation final_rejection{
      app::SourceMode::Replay,
      app::RedFailureDisposition::RecoverablePrimaryMaximumIterations,
      2U,
      17U,
      2U,
      17U,
      3U};
  require(app::isFinalReplayPrimaryMaximumIterationsRejection(final_rejection),
          "matching final Replay MAX_ITER rejection must finish the source");

  auto changed = final_rejection;
  changed.rejected_source_index = 1U;
  require(!app::isFinalReplayPrimaryMaximumIterationsRejection(changed),
          "a middle rejected frame must not finish Replay");
  changed = final_rejection;
  changed.published_revision = 18U;
  require(!app::isFinalReplayPrimaryMaximumIterationsRejection(changed),
          "a stale rejected revision must not finish Replay");
  changed = final_rejection;
  changed.source_mode = app::SourceMode::Teleop;
  require(!app::isFinalReplayPrimaryMaximumIterationsRejection(changed),
          "Teleop rejection must not affect Replay completion");

  const auto succeeded = app::decideReplayRunDisposition(false, true, 0U);
  require(succeeded.state == app::ReplayRunState::Succeeded &&
              !succeeded.fail_process,
          "clean completed Replay must succeed");
  const auto rejected = app::decideReplayRunDisposition(false, true, 1U);
  require(rejected.state == app::ReplayRunState::Failed &&
              rejected.fail_process &&
              std::string{app::replayRunStateName(rejected.state)} == "failed",
          "completed Replay with Primary MAX_ITER must fail");
  const auto stopped = app::decideReplayRunDisposition(false, false, 1U);
  require(stopped.state == app::ReplayRunState::Stopped &&
              !stopped.fail_process,
          "manually stopped Replay must remain stopped");
  const auto faulted = app::decideReplayRunDisposition(true, false, 0U);
  require(faulted.state == app::ReplayRunState::Failed && faulted.fail_process,
          "fatal worker fault must fail Replay");

  const std::string detail = app::primaryMaximumIterationsReplayFailureDetail(
      2U, 17U, "PROXQP_MAX_ITER_REACHED");
  require(detail.find("rejections=2") != std::string::npos &&
              detail.find("revision=17") != std::string::npos &&
              detail.find("PROXQP_MAX_ITER_REACHED") != std::string::npos,
          "Replay failure detail must preserve count, revision, and evidence");
}

} // namespace

int main() {
  try {
    testFailureClassification();
    testRejectedRevisionPolicy();
    testAdmittanceTransactionPolicy();
    testReplayCompletionPolicy();
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
