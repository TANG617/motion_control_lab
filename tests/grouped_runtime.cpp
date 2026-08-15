#include "runtime/grouped_worker.hpp"
#include "runtime/latest_value_mailbox.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace mcl = motion_control_lab;

namespace
{

struct Snapshot
{
  std::uint64_t revision{0};
  std::array<std::uint64_t, 32> values{};
};

bool testMailbox()
{
  Snapshot initial;
  mcl::LatestValueMailbox<Snapshot> mailbox(initial);
  constexpr std::uint64_t iterations = 200000;
  std::atomic_bool done{false};
  std::atomic_bool valid{true};

  std::thread producer([&]() {
    Snapshot value;
    for (std::uint64_t revision = 1; revision <= iterations; ++revision) {
      value.revision = revision;
      for (std::size_t index = 0; index < value.values.size(); ++index) {
        value.values[index] = revision * 101 + index;
      }
      mailbox.publish(value);
    }
    done.store(true, std::memory_order_release);
  });

  Snapshot observed;
  std::uint64_t last_revision = 0;
  while (!done.load(std::memory_order_acquire) || last_revision < iterations) {
    if (!mailbox.readLatest(observed)) {
      continue;
    }
    if (observed.revision < last_revision) {
      valid.store(false, std::memory_order_release);
      break;
    }
    last_revision = observed.revision;
    for (std::size_t index = 0; index < observed.values.size(); ++index) {
      if (observed.values[index] != observed.revision * 101 + index) {
        valid.store(false, std::memory_order_release);
        break;
      }
    }
  }
  producer.join();
  return valid.load(std::memory_order_acquire) && last_revision == iterations;
}

bool testFirstWriterFault()
{
  mcl::GroupedFaultState fault;
  std::atomic_int winner_count{0};
  std::vector<std::thread> writers;
  for (int index = 0; index < 8; ++index) {
    writers.emplace_back([&, index]() {
      mcl::GroupedWorkerFault candidate;
      candidate.group = mcl::WorkerGroup::Yellow;
      candidate.failure = mcl::WorkerFailureKind::Exception;
      candidate.revision = static_cast<std::uint64_t>(index + 1);
      candidate.deadline_ms = 10.0;
      candidate.detail = "test";
      if (fault.trigger(candidate)) {
        winner_count.fetch_add(1, std::memory_order_relaxed);
      }
    });
  }
  for (auto & writer : writers) {
    writer.join();
  }
  const auto snapshot = fault.snapshot();
  return winner_count.load() == 1 && snapshot.has_value() &&
         snapshot->revision >= 1 && snapshot->revision <= 8;
}

bool testRejectedAttemptFault()
{
  mcl::WorkerStopController stop;
  mcl::GroupedFaultState fault;
  mcl::PeriodicWorkerDiagnostics diagnostics;
  mcl::runPeriodicWorker(
    {mcl::WorkerGroup::Green, 1000.0, mcl::DeadlinePolicy::Monitor},
    stop,
    fault,
    diagnostics,
    [](double, std::int64_t) {
      return mcl::WorkerIterationResult{
        mcl::WorkerIterationOutcome::FatalRejected, 7, 0.1, "rejected"};
    });
  const auto snapshot = fault.snapshot();
  const auto statistics = diagnostics.snapshot();
  return stop.stopRequested() && snapshot.has_value() &&
         snapshot->failure == mcl::WorkerFailureKind::RejectedAttempt &&
         snapshot->revision == 7 && statistics.iteration_count == 1;
}

bool testRecoverableRejectionWaitsAndContinues()
{
  mcl::WorkerStopController stop;
  mcl::GroupedFaultState fault;
  mcl::PeriodicWorkerDiagnostics diagnostics;
  std::atomic_int iterations{0};
  mcl::runPeriodicWorker(
    {mcl::WorkerGroup::Red, 1000.0, mcl::DeadlinePolicy::Monitor},
    stop,
    fault,
    diagnostics,
    [&](double, std::int64_t) {
      const int iteration = iterations.fetch_add(1, std::memory_order_relaxed);
      if (iteration == 0) {
        return mcl::WorkerIterationResult{
          mcl::WorkerIterationOutcome::RecoverableRejected, 7, 0.1, "infeasible"};
      }
      if (iteration == 1) {
        return mcl::WorkerIterationResult{
          mcl::WorkerIterationOutcome::Idle, 7, 0.0, {}};
      }
      stop.requestStop();
      return mcl::WorkerIterationResult{
        mcl::WorkerIterationOutcome::Accepted, 8, 0.1, {}};
    });
  const auto statistics = diagnostics.snapshot();
  return iterations.load(std::memory_order_relaxed) == 3 && !fault.triggered() &&
         statistics.iteration_count == 3 && statistics.recoverable_rejection_count == 1;
}

bool testStrictDeadlineWinsOverRecoverableRejection()
{
  mcl::WorkerStopController stop;
  mcl::GroupedFaultState fault;
  mcl::PeriodicWorkerDiagnostics diagnostics;
  mcl::runPeriodicWorker(
    {mcl::WorkerGroup::Red, 100.0, mcl::DeadlinePolicy::Strict},
    stop,
    fault,
    diagnostics,
    [](double, std::int64_t) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      return mcl::WorkerIterationResult{
        mcl::WorkerIterationOutcome::RecoverableRejected, 9, 20.0, "infeasible"};
    });
  const auto snapshot = fault.snapshot();
  const auto statistics = diagnostics.snapshot();
  return stop.stopRequested() && snapshot.has_value() &&
         snapshot->failure == mcl::WorkerFailureKind::DeadlineMiss &&
         snapshot->revision == 9 && statistics.recoverable_rejection_count == 1;
}

bool testMonitorPolicyDoesNotSuppressExceptions()
{
  mcl::WorkerStopController stop;
  mcl::GroupedFaultState fault;
  mcl::PeriodicWorkerDiagnostics diagnostics;
  mcl::runPeriodicWorker(
    {mcl::WorkerGroup::Yellow, 100.0, mcl::DeadlinePolicy::Monitor},
    stop,
    fault,
    diagnostics,
    [](double, std::int64_t) -> mcl::WorkerIterationResult {
      throw std::runtime_error("worker failed");
    });
  const auto snapshot = fault.snapshot();
  return stop.stopRequested() && snapshot.has_value() &&
         snapshot->failure == mcl::WorkerFailureKind::Exception &&
         snapshot->detail == "worker failed";
}

bool testDeadlineAndCooperativeStop()
{
  mcl::WorkerStopController deadline_stop;
  mcl::GroupedFaultState deadline_fault;
  mcl::PeriodicWorkerDiagnostics deadline_diagnostics;
  mcl::runPeriodicWorker(
    {mcl::WorkerGroup::Red, 100.0},
    deadline_stop,
    deadline_fault,
    deadline_diagnostics,
    [](double, std::int64_t) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      return mcl::WorkerIterationResult{
        mcl::WorkerIterationOutcome::Accepted, 1, 20.0, {}};
    });
  const auto deadline = deadline_fault.snapshot();
  if (!deadline || deadline->failure != mcl::WorkerFailureKind::DeadlineMiss ||
      deadline->release_lateness_ms < 0.0 || deadline->execution_ms < 20.0 ||
      deadline->release_to_finish_ms < deadline->execution_ms ||
      deadline->deadline_ms != 10.0 || deadline->overrun_ms <= 0.0 ||
      deadline->solver_ms != 20.0)
  {
    return false;
  }

  mcl::WorkerStopController stop;
  std::atomic_int iterations{0};
  mcl::GroupedFaultState no_fault;
  mcl::PeriodicWorkerDiagnostics diagnostics;
  std::thread worker([&]() {
    mcl::runPeriodicWorker(
      {mcl::WorkerGroup::Yellow, 100.0},
      stop,
      no_fault,
      diagnostics,
      [&](double, std::int64_t) {
        if (iterations.fetch_add(1) >= 4) {
          stop.requestStop();
        }
        return mcl::WorkerIterationResult{
          mcl::WorkerIterationOutcome::Accepted, 1, 0.0, {}};
      });
  });
  worker.join();
  return iterations.load() >= 5 && !no_fault.triggered();
}

bool testMonitorPolicyContinuesAndSkipsExpiredReleases()
{
  mcl::WorkerStopController stop;
  mcl::GroupedFaultState fault;
  mcl::PeriodicWorkerDiagnostics diagnostics;
  std::atomic_int iterations{0};
  mcl::runPeriodicWorker(
    {mcl::WorkerGroup::Red, 100.0, mcl::DeadlinePolicy::Monitor},
    stop,
    fault,
    diagnostics,
    [&](double, std::int64_t) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      if (iterations.fetch_add(1, std::memory_order_relaxed) >= 2) {
        stop.requestStop();
      }
      return mcl::WorkerIterationResult{
        mcl::WorkerIterationOutcome::Accepted, 1, 20.0, {}};
    });

  const auto statistics = diagnostics.snapshot();
  return iterations.load(std::memory_order_relaxed) == 3 && !fault.triggered() &&
         statistics.iteration_count == 3 && statistics.deadline_miss_count == 3 &&
         statistics.consecutive_deadline_misses == 3 &&
         statistics.skipped_release_count >= 3 &&
         statistics.maximum_overrun_ms > 0.0 && statistics.maximum_solver_ms == 20.0;
}

bool testPeriodicWaitIsInterruptible()
{
  mcl::WorkerStopController stop;
  mcl::GroupedFaultState fault;
  mcl::PeriodicWorkerDiagnostics diagnostics;
  std::atomic_bool iteration_finished{false};
  std::thread worker([&]() {
    mcl::runPeriodicWorker(
      {mcl::WorkerGroup::Green, 0.5},
      stop,
      fault,
      diagnostics,
      [&](double, std::int64_t) {
        iteration_finished.store(true, std::memory_order_release);
        return mcl::WorkerIterationResult{
          mcl::WorkerIterationOutcome::Accepted, 1, 0.0, {}};
      });
  });
  while (!iteration_finished.load(std::memory_order_acquire)) {
    std::this_thread::yield();
  }
  const auto stop_started = std::chrono::steady_clock::now();
  stop.requestStop();
  worker.join();
  const auto stop_elapsed = std::chrono::steady_clock::now() - stop_started;
  return stop_elapsed < std::chrono::milliseconds(200) && !fault.triggered();
}

}  // namespace

int main()
{
  bool passed = true;
  auto check = [&](const char * name, bool result) {
      if (!result) {
        std::cerr << "failed: " << name << '\n';
        passed = false;
      }
    };
  check("mailbox", testMailbox());
  check("first writer fault", testFirstWriterFault());
  check("rejected attempt", testRejectedAttemptFault());
  check("recoverable rejection", testRecoverableRejectionWaitsAndContinues());
  check("recoverable rejection strict deadline", testStrictDeadlineWinsOverRecoverableRejection());
  check("monitor exception", testMonitorPolicyDoesNotSuppressExceptions());
  check("deadline and cooperative stop", testDeadlineAndCooperativeStop());
  check("monitor policy", testMonitorPolicyContinuesAndSkipsExpiredReleases());
  check("interruptible wait", testPeriodicWaitIsInterruptible());
  return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
