#include "allocation.hpp"
#include <cstdlib>
#include <new>
namespace {
thread_local bool observing = false;
thread_local study_e09::AllocationObservation measured;
void count(std::size_t n) {
  if (observing) {
    ++measured.count;
    measured.bytes += n;
  }
}
void *allocate(std::size_t n) {
  void *p = std::malloc(n ? n : 1);
  if (!p)
    throw std::bad_alloc();
  count(n);
  return p;
}
void *aligned(std::size_t n, std::size_t a) {
  void *p = nullptr;
  if (posix_memalign(&p, a, n ? n : 1))
    throw std::bad_alloc();
  count(n);
  return p;
}
} // namespace
void *operator new(std::size_t n) { return allocate(n); }
void *operator new[](std::size_t n) { return allocate(n); }
void operator delete(void *p) noexcept { std::free(p); }
void operator delete[](void *p) noexcept { std::free(p); }
void operator delete(void *p, std::size_t) noexcept { std::free(p); }
void operator delete[](void *p, std::size_t) noexcept { std::free(p); }
void *operator new(std::size_t n, std::align_val_t a) {
  return aligned(n, static_cast<std::size_t>(a));
}
void *operator new[](std::size_t n, std::align_val_t a) {
  return aligned(n, static_cast<std::size_t>(a));
}
void operator delete(void *p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void *p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void *p, std::size_t, std::align_val_t) noexcept {
  std::free(p);
}
void operator delete[](void *p, std::size_t, std::align_val_t) noexcept {
  std::free(p);
}
namespace study_e09 {
void beginAllocationObservation() {
  measured = {};
  observing = true;
}
AllocationObservation endAllocationObservation() {
  observing = false;
  return measured;
}
} // namespace study_e09
