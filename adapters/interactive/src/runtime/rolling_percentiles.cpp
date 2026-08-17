#include "runtime/rolling_percentiles.hpp"

#include <algorithm>
#include <cmath>

namespace motion_control_lab
{
namespace
{

double nearestRank(const std::vector<double> & sorted_samples, double percentile)
{
  const auto rank = static_cast<std::size_t>(
    std::ceil(percentile * static_cast<double>(sorted_samples.size())));
  return sorted_samples[std::max<std::size_t>(rank, 1U) - 1U];
}

}  // namespace

RollingPercentiles::RollingPercentiles()
{
  samples_.reserve(kWindowCapacity);
}

void RollingPercentiles::record(double sample)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (samples_.size() < kWindowCapacity) {
    samples_.push_back(sample);
  } else {
    samples_[next_index_] = sample;
    next_index_ = (next_index_ + 1U) % kWindowCapacity;
  }
  ++total_sample_count_;
}

RollingPercentilesSnapshot RollingPercentiles::snapshot() const
{
  std::vector<double> sorted_samples;
  std::uint64_t total_sample_count = 0;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    sorted_samples = samples_;
    total_sample_count = total_sample_count_;
  }

  std::sort(sorted_samples.begin(), sorted_samples.end());
  RollingPercentilesSnapshot result;
  result.total_sample_count = total_sample_count;
  result.window_sample_count = sorted_samples.size();
  result.window_capacity = kWindowCapacity;
  if (!sorted_samples.empty()) {
    result.p90 = nearestRank(sorted_samples, 0.90);
    result.p95 = nearestRank(sorted_samples, 0.95);
    result.p99 = nearestRank(sorted_samples, 0.99);
  }
  return result;
}

}  // namespace motion_control_lab
