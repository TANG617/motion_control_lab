#pragma once
#include <cstdint>
namespace motion_control_lab::target {
struct AllocationObservation {
  std::uint64_t count{}, bytes{};
};
void beginAllocationObservation();
AllocationObservation endAllocationObservation();
} // namespace motion_control_lab::target
