#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
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
};

struct WorkerIterationResult
{
  bool accepted{false};
  std::uint64_t revision{0};
  double solve_time_ms{0.0};
  std::string detail;
};

using WorkerIteration = std::function<WorkerIterationResult(double, std::int64_t)>;

void runPeriodicWorker(
  PeriodicWorkerOptions options,
  std::atomic_bool & stop_requested,
  GroupedFaultState & fault,
  const WorkerIteration & iteration);

}  // namespace motion_control_lab
