#include "runtime/grouped_worker.hpp"

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

void runPeriodicWorker(
  PeriodicWorkerOptions options,
  std::atomic_bool & stop_requested,
  GroupedFaultState & fault,
  const WorkerIteration & iteration)
{
  using Clock = std::chrono::steady_clock;
  if (options.rate_hz <= 0.0 || !std::isfinite(options.rate_hz)) {
    throw std::runtime_error("worker rate must be positive and finite");
  }
  const auto period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / options.rate_hz));
  const double deadline_ms = 1000.0 / options.rate_hz;
  auto release = Clock::now();
  const auto epoch = release;

  while (!stop_requested.load(std::memory_order_acquire)) {
    const auto deadline = release + period;
    const auto started = Clock::now();
    auto makeFault = [&](
        WorkerFailureKind failure,
        std::uint64_t revision,
        Clock::time_point finished,
        double solver_ms,
        std::string detail) {
        const double release_lateness_ms = std::max(
          0.0,
          std::chrono::duration<double, std::milli>(started - release).count());
        const double execution_ms = std::chrono::duration<double, std::milli>(
          finished - started).count();
        const double release_to_finish_ms = std::chrono::duration<double, std::milli>(
          finished - release).count();
        const double overrun_ms = std::max(
          0.0,
          std::chrono::duration<double, std::milli>(finished - deadline).count());
        return GroupedWorkerFault{
          options.group,
          failure,
          revision,
          release_lateness_ms,
          execution_ms,
          release_to_finish_ms,
          deadline_ms,
          overrun_ms,
          solver_ms,
          std::move(detail)};
      };
    try {
      const auto sample_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        started - epoch).count();
      const WorkerIterationResult result = iteration(1.0 / options.rate_hz, sample_time_ns);
      const auto finished = Clock::now();
      if (!result.accepted) {
        fault.trigger(makeFault(
          WorkerFailureKind::RejectedAttempt,
          result.revision,
          finished,
          result.solve_time_ms,
          result.detail));
        stop_requested.store(true, std::memory_order_release);
        return;
      }
      if (finished > deadline) {
        fault.trigger(makeFault(
          WorkerFailureKind::DeadlineMiss,
          result.revision,
          finished,
          result.solve_time_ms,
          result.detail));
        stop_requested.store(true, std::memory_order_release);
        return;
      }
    } catch (const std::exception & error) {
      fault.trigger(makeFault(
        WorkerFailureKind::Exception,
        0,
        Clock::now(),
        0.0,
        error.what()));
      stop_requested.store(true, std::memory_order_release);
      return;
    } catch (...) {
      fault.trigger(makeFault(
        WorkerFailureKind::Exception,
        0,
        Clock::now(),
        0.0,
        "unknown exception"));
      stop_requested.store(true, std::memory_order_release);
      return;
    }

    release += period;
    while (!stop_requested.load(std::memory_order_acquire)) {
      const auto now = Clock::now();
      if (now >= release) {
        break;
      }
      std::this_thread::sleep_until(std::min(release, now + std::chrono::milliseconds(1)));
    }
  }
}

}  // namespace motion_control_lab
