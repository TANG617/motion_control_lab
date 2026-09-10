#pragma once
#include <cstdint>
namespace study_e09 {
struct AllocationObservation {
  std::uint64_t count{}, bytes{};
};
void beginAllocationObservation();
AllocationObservation endAllocationObservation();
} // namespace study_e09
