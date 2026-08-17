#include "runtime/rolling_percentiles.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace mcl = motion_control_lab;

namespace
{

bool equal(double left, double right)
{
  return std::abs(left - right) < 1.0e-12;
}

bool testEmptySnapshot()
{
  mcl::RollingPercentiles percentiles;
  const auto snapshot = percentiles.snapshot();
  return snapshot.total_sample_count == 0 && snapshot.window_sample_count == 0 &&
         snapshot.window_capacity == mcl::RollingPercentiles::kWindowCapacity &&
         equal(snapshot.p90, 0.0) && equal(snapshot.p95, 0.0) && equal(snapshot.p99, 0.0);
}

bool testNearestRankPercentiles()
{
  mcl::RollingPercentiles percentiles;
  for (int sample = 1; sample <= 100; ++sample) {
    percentiles.record(static_cast<double>(sample));
  }
  const auto snapshot = percentiles.snapshot();
  return snapshot.total_sample_count == 100 && snapshot.window_sample_count == 100 &&
         equal(snapshot.p90, 90.0) && equal(snapshot.p95, 95.0) && equal(snapshot.p99, 99.0);
}

bool testRollingWindow()
{
  mcl::RollingPercentiles percentiles;
  for (std::size_t sample = 1;
    sample <= mcl::RollingPercentiles::kWindowCapacity + 1U; ++sample)
  {
    percentiles.record(static_cast<double>(sample));
  }
  const auto snapshot = percentiles.snapshot();
  return
    snapshot.total_sample_count == mcl::RollingPercentiles::kWindowCapacity + 1U &&
    snapshot.window_sample_count == mcl::RollingPercentiles::kWindowCapacity &&
    equal(snapshot.p90, 3688.0) && equal(snapshot.p95, 3893.0) &&
    equal(snapshot.p99, 4057.0);
}

}  // namespace

int main()
{
  const bool empty = testEmptySnapshot();
  const bool nearest_rank = testNearestRankPercentiles();
  const bool rolling_window = testRollingWindow();
  if (!empty || !nearest_rank || !rolling_window) {
    std::cerr << "rolling percentile tests failed: empty=" << empty
              << " nearest_rank=" << nearest_rank
              << " rolling_window=" << rolling_window << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
