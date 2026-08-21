#include "components/scheduler/single_rate_scheduler.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <stdexcept>
#include <thread>

namespace motion_control_lab
{
namespace
{

std::atomic_bool stop_requested{false};

void handleSignal(int) { stop_requested.store(true); }

}  // namespace

SingleRateScheduler::SingleRateScheduler(SingleRateSchedulerOptions options)
: options_(options),
  start_(Clock::now()),
  next_update_(start_),
  next_draw_(start_),
  period_(Clock::duration::zero())
{
  if (options_.rate_hz <= 0.0 || !std::isfinite(options_.rate_hz)) {
    throw std::runtime_error("scheduler rate must be positive and finite");
  }
  if (options_.duration_s < 0.0 || !std::isfinite(options_.duration_s)) {
    throw std::runtime_error("scheduler duration must be finite and non-negative");
  }
  period_ = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / options_.rate_hz));
}

std::optional<SingleRateTick> SingleRateScheduler::next()
{
  if (stop_requested.load()) return std::nullopt;
  const auto now = Clock::now();
  const double elapsed_s = std::chrono::duration<double>(now - start_).count();
  if (options_.duration_s > 0.0 && elapsed_s >= options_.duration_s) return std::nullopt;

  SingleRateTick tick;
  tick.update_due = now >= next_update_;
  tick.draw_due = now >= next_draw_;
  tick.dt = 1.0 / options_.rate_hz;
  tick.sample_time_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now - start_).count();
  tick.emit_time_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
  if (tick.update_due) next_update_ = now + period_;
  if (tick.draw_due) next_draw_ = now + std::chrono::milliseconds(100);
  return tick;
}

void SingleRateScheduler::sleep() const
{
  std::this_thread::sleep_for(std::min(
    period_, std::chrono::duration_cast<Clock::duration>(std::chrono::milliseconds(10))));
}

void installRuntimeSignalHandlers()
{
  stop_requested.store(false);
  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);
}

}  // namespace motion_control_lab
