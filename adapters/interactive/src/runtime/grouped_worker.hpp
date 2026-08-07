#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

namespace motion_control_lab
{

enum class WorkerGroup
{
  Red,
  Yellow,
  Green,
};

enum class WorkerFailureKind
{
  RejectedAttempt,
  DeadlineMiss,
  Exception,
};

enum class DeadlinePolicy
{
  Strict,
  Monitor,
};

const char * workerGroupName(WorkerGroup group);
const char * workerFailureName(WorkerFailureKind failure);

struct GroupedWorkerFault
{
  WorkerGroup group{WorkerGroup::Red};
  WorkerFailureKind failure{WorkerFailureKind::Exception};
  std::uint64_t revision{0};
  double release_lateness_ms{0.0};
  double execution_ms{0.0};
  double release_to_finish_ms{0.0};
  double deadline_ms{0.0};
  double overrun_ms{0.0};
  double solver_ms{0.0};
  std::string detail;
};

class GroupedFaultState
{
public:
  bool trigger(GroupedWorkerFault fault);
  bool triggered() const;
  std::optional<GroupedWorkerFault> snapshot() const;

private:
  // 0 = empty, 1 = first writer owns storage, 2 = published.
  std::atomic<int> state_{0};
  GroupedWorkerFault fault_;
};

struct PeriodicWorkerOptions
{
  WorkerGroup group{WorkerGroup::Red};
  double rate_hz{1.0};
  DeadlinePolicy deadline_policy{DeadlinePolicy::Strict};
};

struct WorkerIterationResult
{
  bool accepted{false};
  std::uint64_t revision{0};
  double solve_time_ms{0.0};
  std::string detail;
};

using WorkerIteration = std::function<WorkerIterationResult(double, std::int64_t)>;

class WorkerStopController
{
public:
  void requestStop();
  bool stopRequested() const;
  bool waitUntil(std::chrono::steady_clock::time_point deadline);

private:
  std::atomic_bool stop_requested_{false};
  mutable std::mutex mutex_;
  std::condition_variable condition_;
};

struct PeriodicWorkerStatistics
{
  std::uint64_t iteration_count{0};
  std::uint64_t deadline_miss_count{0};
  std::uint64_t consecutive_deadline_misses{0};
  std::uint64_t skipped_release_count{0};
  double maximum_release_lateness_ms{0.0};
  double maximum_execution_ms{0.0};
  double maximum_release_to_finish_ms{0.0};
  double maximum_overrun_ms{0.0};
  double maximum_solver_ms{0.0};
};

class PeriodicWorkerDiagnostics
{
public:
  PeriodicWorkerStatistics snapshot() const;

private:
  void recordIteration(
    bool deadline_missed,
    double release_lateness_ms,
    double execution_ms,
    double release_to_finish_ms,
    double overrun_ms,
    double solver_ms);
  void recordSkippedReleases(std::uint64_t count);

  std::atomic<std::uint64_t> iteration_count_{0};
  std::atomic<std::uint64_t> deadline_miss_count_{0};
  std::atomic<std::uint64_t> consecutive_deadline_misses_{0};
  std::atomic<std::uint64_t> skipped_release_count_{0};
  std::atomic<double> maximum_release_lateness_ms_{0.0};
  std::atomic<double> maximum_execution_ms_{0.0};
  std::atomic<double> maximum_release_to_finish_ms_{0.0};
  std::atomic<double> maximum_overrun_ms_{0.0};
  std::atomic<double> maximum_solver_ms_{0.0};

  friend void runPeriodicWorker(
    PeriodicWorkerOptions,
    WorkerStopController &,
    GroupedFaultState &,
    PeriodicWorkerDiagnostics &,
    const WorkerIteration &);
};

void runPeriodicWorker(
  PeriodicWorkerOptions options,
  WorkerStopController & stop_controller,
  GroupedFaultState & fault,
  PeriodicWorkerDiagnostics & diagnostics,
  const WorkerIteration & iteration);

}  // namespace motion_control_lab
