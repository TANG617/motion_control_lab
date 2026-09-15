#pragma once
#include <cstdint>
namespace motion_control_lab::step {
struct AllocationObservation {
  std::uint64_t count{}, bytes{};
};
void beginAllocationObservation();
AllocationObservation endAllocationObservation();
} // namespace motion_control_lab::step
