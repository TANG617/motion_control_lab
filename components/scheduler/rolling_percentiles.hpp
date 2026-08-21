#pragma once

#include "contracts/runtime/runtime_contract.hpp"

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

namespace motion_control_lab
{

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
