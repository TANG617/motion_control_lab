#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

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

class RollingPercentiles
{
public:
  static constexpr std::size_t kWindowCapacity = 4096;

  RollingPercentiles();

  void record(double sample);
  RollingPercentilesSnapshot snapshot() const;

private:
  mutable std::mutex mutex_;
  std::vector<double> samples_;
  std::size_t next_index_{0};
  std::uint64_t total_sample_count_{0};
};

}  // namespace motion_control_lab
