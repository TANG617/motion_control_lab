#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

namespace motion_control_lab
{

struct InteractiveSchedulerOptions
{
  double rate_hz{20.0};
  double duration_s{0.0};
};

struct InteractiveSchedule
{
  bool update_due{false};
  bool draw_due{false};
  double dt{0.0};
  std::int64_t sample_time_ns{0};
  std::uint64_t emit_time_ns{0};
};

class InteractiveScheduler
{
public:
  explicit InteractiveScheduler(InteractiveSchedulerOptions options);

  std::optional<InteractiveSchedule> next();

  void sleep() const;

private:
  using Clock = std::chrono::steady_clock;

  InteractiveSchedulerOptions options_;
  Clock::time_point start_;
  Clock::time_point next_update_;
  Clock::time_point next_draw_;
  Clock::duration period_;
};

void installInteractiveSignalHandlers();

}  // namespace motion_control_lab
