#pragma once

#include <cstddef>
#include <cstdint>

namespace motion_control_lab
{

struct RollingPercentilesSnapshot
{
  std::uint64_t total_sample_count{0};
  std::size_t window_sample_count{0};
  std::size_t window_capacity{0};
  double p90{0.0};
  double p95{0.0};
  double p99{0.0};
};

}  // namespace motion_control_lab
