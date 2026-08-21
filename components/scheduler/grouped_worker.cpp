#include "components/scheduler/grouped_worker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <utility>

namespace motion_control_lab
{

const char * workerGroupName(WorkerGroup group)
{
  switch (group) {
    case WorkerGroup::Red:
      return "Red";
    case WorkerGroup::Yellow:
      return "Yellow";
    case WorkerGroup::Green:
      return "Green";
  }
  return "Unknown";
}

const char * workerFailureName(WorkerFailureKind failure)
{
  switch (failure) {
    case WorkerFailureKind::RejectedAttempt:
      return "rejected attempt";
    case WorkerFailureKind::DeadlineMiss:
      return "deadline miss";
    case WorkerFailureKind::Exception:
      return "worker exception";
  }
  return "unknown failure";
}

bool GroupedFaultState::trigger(GroupedWorkerFault fault)
{
  int expected = 0;
  if (!state_.compare_exchange_strong(
      expected, 1, std::memory_order_acq_rel, std::memory_order_acquire))
  {
    return false;
  }
  fault_ = std::move(fault);
  state_.store(2, std::memory_order_release);
  return true;
}

bool GroupedFaultState::triggered() const
{
  return state_.load(std::memory_order_acquire) != 0;
}

std::optional<GroupedWorkerFault> GroupedFaultState::snapshot() const
{
  int state = state_.load(std::memory_order_acquire);
  while (state == 1) {
    std::this_thread::yield();
    state = state_.load(std::memory_order_acquire);
  }
  if (state != 2) {
    return std::nullopt;
  }
  return fault_;
}

void WorkerStopController::requestStop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_requested_.store(true, std::memory_order_release);
  }
  condition_.notify_all();
}

bool WorkerStopController::stopRequested() const
{
  return stop_requested_.load(std::memory_order_acquire);
}

bool WorkerStopController::waitUntil(std::chrono::steady_clock::time_point deadline)
{
  std::unique_lock<std::mutex> lock(mutex_);
  return condition_.wait_until(lock, deadline, [this]() {
      return stopRequested();
    });
}

namespace
{

void updateMaximum(std::atomic<double> & maximum, double value)
{
  double current = maximum.load(std::memory_order_relaxed);
  while (current < value &&
    !maximum.compare_exchange_weak(
      current, value, std::memory_order_relaxed, std::memory_order_relaxed))
  {
  }
}

}  // namespace

PeriodicWorkerStatistics PeriodicWorkerDiagnostics::snapshot() const
{
  return PeriodicWorkerStatistics{
    iteration_count_.load(std::memory_order_relaxed),
    deadline_miss_count_.load(std::memory_order_relaxed),
    consecutive_deadline_misses_.load(std::memory_order_relaxed),
    skipped_release_count_.load(std::memory_order_relaxed),
    recoverable_rejection_count_.load(std::memory_order_relaxed),
    maximum_release_lateness_ms_.load(std::memory_order_relaxed),
    maximum_execution_ms_.load(std::memory_order_relaxed),
    maximum_release_to_finish_ms_.load(std::memory_order_relaxed),
    maximum_overrun_ms_.load(std::memory_order_relaxed),
    maximum_solver_ms_.load(std::memory_order_relaxed),
    maximum_non_solver_execution_ms_.load(std::memory_order_relaxed),
    latest_release_lateness_ms_.load(std::memory_order_relaxed),
    latest_execution_ms_.load(std::memory_order_relaxed),
    latest_release_to_finish_ms_.load(std::memory_order_relaxed),
    latest_overrun_ms_.load(std::memory_order_relaxed),
    latest_solver_ms_.load(std::memory_order_relaxed),
    latest_non_solver_execution_ms_.load(std::memory_order_relaxed)};
}

void PeriodicWorkerDiagnostics::recordIteration(
  bool deadline_missed,
  double release_lateness_ms,
  double execution_ms,
  double release_to_finish_ms,
  double overrun_ms,
  double solver_ms)
{
  iteration_count_.fetch_add(1, std::memory_order_relaxed);
  if (deadline_missed) {
    deadline_miss_count_.fetch_add(1, std::memory_order_relaxed);
    consecutive_deadline_misses_.fetch_add(1, std::memory_order_relaxed);
  } else {
    consecutive_deadline_misses_.store(0, std::memory_order_relaxed);
  }
  updateMaximum(maximum_release_lateness_ms_, release_lateness_ms);
  updateMaximum(maximum_execution_ms_, execution_ms);
  updateMaximum(maximum_release_to_finish_ms_, release_to_finish_ms);
  updateMaximum(maximum_overrun_ms_, overrun_ms);
  updateMaximum(maximum_solver_ms_, solver_ms);
  const double non_solver_execution_ms = std::max(0.0, execution_ms - solver_ms);
  updateMaximum(maximum_non_solver_execution_ms_, non_solver_execution_ms);
  latest_release_lateness_ms_.store(release_lateness_ms, std::memory_order_relaxed);
  latest_execution_ms_.store(execution_ms, std::memory_order_relaxed);
  latest_release_to_finish_ms_.store(release_to_finish_ms, std::memory_order_relaxed);
  latest_overrun_ms_.store(overrun_ms, std::memory_order_relaxed);
  latest_solver_ms_.store(solver_ms, std::memory_order_relaxed);
  latest_non_solver_execution_ms_.store(non_solver_execution_ms, std::memory_order_relaxed);
}

void PeriodicWorkerDiagnostics::recordSkippedReleases(std::uint64_t count)
{
  skipped_release_count_.fetch_add(count, std::memory_order_relaxed);
}

void PeriodicWorkerDiagnostics::recordRecoverableRejection()
{
  recoverable_rejection_count_.fetch_add(1, std::memory_order_relaxed);
}

void runPeriodicWorker(
  PeriodicWorkerOptions options,
  WorkerStopController & stop_controller,
  GroupedFaultState & fault,
  PeriodicWorkerDiagnostics & diagnostics,
  const WorkerIteration & iteration)
{
  using Clock = std::chrono::steady_clock;
  if (options.rate_hz <= 0.0 || !std::isfinite(options.rate_hz)) {
    throw std::runtime_error("worker rate must be positive and finite");
  }
  const auto period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / options.rate_hz));
  if (period <= Clock::duration::zero()) {
    throw std::runtime_error("worker rate produces a zero clock period");
  }
  const double deadline_ms = 1000.0 / options.rate_hz;
  auto release = Clock::now();
  const auto epoch = release;

  while (!stop_controller.stopRequested()) {
    const auto deadline = release + period;
    const auto started = Clock::now();
    struct IterationTiming
    {
      double release_lateness_ms;
      double execution_ms;
      double release_to_finish_ms;
      double overrun_ms;
    };
    auto calculateTiming = [&](Clock::time_point finished) {
        return IterationTiming{
          std::max(
            0.0,
            std::chrono::duration<double, std::milli>(started - release).count()),
          std::chrono::duration<double, std::milli>(finished - started).count(),
          std::chrono::duration<double, std::milli>(finished - release).count(),
          std::max(
            0.0,
            std::chrono::duration<double, std::milli>(finished - deadline).count())};
      };
    auto makeFault = [&options, deadline_ms](
        WorkerFailureKind failure,
        std::uint64_t revision,
        const IterationTiming & timing,
        double solver_ms,
        std::string detail) {
        return GroupedWorkerFault{
          options.group,
          failure,
          revision,
          timing.release_lateness_ms,
          timing.execution_ms,
          timing.release_to_finish_ms,
          deadline_ms,
          timing.overrun_ms,
          solver_ms,
          std::move(detail)};
      };
    try {
      const auto sample_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        started - epoch).count();
      const WorkerIterationResult result = iteration(1.0 / options.rate_hz, sample_time_ns);
      const auto finished = Clock::now();
      const IterationTiming timing = calculateTiming(finished);
      const bool deadline_missed = finished > deadline;
      diagnostics.recordIteration(
        deadline_missed,
        timing.release_lateness_ms,
        timing.execution_ms,
        timing.release_to_finish_ms,
        timing.overrun_ms,
        result.solve_time_ms);
      if (result.outcome == WorkerIterationOutcome::RecoverableRejected) {
        diagnostics.recordRecoverableRejection();
      }
      if (result.outcome == WorkerIterationOutcome::FatalRejected) {
        fault.trigger(makeFault(
          WorkerFailureKind::RejectedAttempt,
          result.revision,
          timing,
          result.solve_time_ms,
          result.detail));
        stop_controller.requestStop();
        return;
      }
      if (deadline_missed && options.deadline_policy == DeadlinePolicy::Strict) {
        fault.trigger(makeFault(
          WorkerFailureKind::DeadlineMiss,
          result.revision,
          timing,
          result.solve_time_ms,
          result.detail));
        stop_controller.requestStop();
        return;
      }

      if (deadline_missed) {
        std::uint64_t skipped_releases = 0;
        do {
          release += period;
          ++skipped_releases;
        } while (release <= finished);
        diagnostics.recordSkippedReleases(skipped_releases);
      } else {
        release += period;
      }
    } catch (const std::exception & error) {
      const auto timing = calculateTiming(Clock::now());
      diagnostics.recordIteration(
        timing.overrun_ms > 0.0,
        timing.release_lateness_ms,
        timing.execution_ms,
        timing.release_to_finish_ms,
        timing.overrun_ms,
        0.0);
      fault.trigger(makeFault(
        WorkerFailureKind::Exception,
        0,
        timing,
        0.0,
        error.what()));
      stop_controller.requestStop();
      return;
    } catch (...) {
      const auto timing = calculateTiming(Clock::now());
      diagnostics.recordIteration(
        timing.overrun_ms > 0.0,
        timing.release_lateness_ms,
        timing.execution_ms,
        timing.release_to_finish_ms,
        timing.overrun_ms,
        0.0);
      fault.trigger(makeFault(
        WorkerFailureKind::Exception,
        0,
        timing,
        0.0,
        "unknown exception"));
      stop_controller.requestStop();
      return;
    }

    if (stop_controller.waitUntil(release)) {
      return;
    }
  }
}

}  // namespace motion_control_lab
