#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

namespace motion_control_lab
{

struct SingleRateSchedulerOptions
{
  double rate_hz{20.0};
  double duration_s{0.0};
};

struct SingleRateTick
{
  bool update_due{false};
  bool draw_due{false};
  double dt{0.0};
  std::int64_t sample_time_ns{0};
  std::uint64_t emit_time_ns{0};
};

class SingleRateScheduler
{
public:
  explicit SingleRateScheduler(SingleRateSchedulerOptions options);

  std::optional<SingleRateTick> next();
  void sleep() const;

private:
  using Clock = std::chrono::steady_clock;

  SingleRateSchedulerOptions options_;
  Clock::time_point start_;
  Clock::time_point next_update_;
  Clock::time_point next_draw_;
  Clock::duration period_;
};

void installRuntimeSignalHandlers();

}  // namespace motion_control_lab
