#include "runtime/grouped_worker.hpp"
#include "runtime/latest_value_mailbox.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
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
  std::atomic_bool stop{false};
  mcl::GroupedFaultState fault;
  mcl::runPeriodicWorker(
    {mcl::WorkerGroup::Green, 1000.0},
    stop,
    fault,
    [](double, std::int64_t) {
      return mcl::WorkerIterationResult{false, 7, 0.1, "rejected"};
    });
  const auto snapshot = fault.snapshot();
  return stop.load() && snapshot.has_value() &&
         snapshot->failure == mcl::WorkerFailureKind::RejectedAttempt &&
         snapshot->revision == 7;
}

bool testDeadlineAndCooperativeStop()
{
  std::atomic_bool deadline_stop{false};
  mcl::GroupedFaultState deadline_fault;
  mcl::runPeriodicWorker(
    {mcl::WorkerGroup::Red, 100.0},
    deadline_stop,
    deadline_fault,
    [](double, std::int64_t) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      return mcl::WorkerIterationResult{true, 1, 20.0, {}};
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

  std::atomic_bool stop{false};
  std::atomic_int iterations{0};
  mcl::GroupedFaultState no_fault;
  std::thread worker([&]() {
    mcl::runPeriodicWorker(
      {mcl::WorkerGroup::Yellow, 100.0},
      stop,
      no_fault,
      [&](double, std::int64_t) {
        if (iterations.fetch_add(1) >= 4) {
          stop.store(true, std::memory_order_release);
        }
        return mcl::WorkerIterationResult{true, 1, 0.0, {}};
      });
  });
  worker.join();
  return iterations.load() >= 5 && !no_fault.triggered();
}

}  // namespace

int main()
{
  return testMailbox() && testFirstWriterFault() && testRejectedAttemptFault() &&
         testDeadlineAndCooperativeStop() ? EXIT_SUCCESS : EXIT_FAILURE;
}
