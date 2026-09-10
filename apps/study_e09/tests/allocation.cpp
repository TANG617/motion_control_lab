#include "../allocation.hpp"
#include <new>
#include <stdexcept>
int main() {
  void *a = ::operator new(17);
  auto off = study_e09::endAllocationObservation();
  ::operator delete(a);
  if (off.count != 0)
    throw std::runtime_error("disabled observer collected allocations");
  study_e09::beginAllocationObservation();
  void *b = ::operator new(31);
  void *c = ::operator new[](67);
  void *d = ::operator new(128, std::align_val_t(64));
  auto on = study_e09::endAllocationObservation();
  ::operator delete(b);
  ::operator delete[](c);
  ::operator delete(d, std::align_val_t(64));
  if (on.count != 3 || on.bytes != 226)
    throw std::runtime_error("C++ allocation counter mismatch");
  study_e09::beginAllocationObservation();
  auto reset = study_e09::endAllocationObservation();
  if (reset.count || reset.bytes)
    throw std::runtime_error("allocation scope did not reset");
}
