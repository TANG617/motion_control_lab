#pragma once

#include <array>
#include <atomic>
#include <cstddef>

namespace motion_control_lab
{

/**
 * Preallocated single-producer/single-consumer latest-value mailbox.
 * Intermediate publications may be skipped; a read always returns one whole
 * published value and never waits for the producer.
 */
template<typename T>
class LatestValueMailbox
{
public:
  explicit LatestValueMailbox(const T & initial_value)
  {
    for (auto & slot : slots_) {
      slot.value = initial_value;
    }
  }

  LatestValueMailbox(const LatestValueMailbox &) = delete;
  LatestValueMailbox & operator=(const LatestValueMailbox &) = delete;

  void publish(const T & value)
  {
    const int current = published_.load(std::memory_order_acquire);
    for (int index = 0; index < static_cast<int>(slots_.size()); ++index) {
      Slot & slot = slots_[static_cast<std::size_t>(index)];
      if (index != current && slot.readers.load(std::memory_order_acquire) == 0) {
        slot.value = value;
        published_.store(index, std::memory_order_release);
        return;
      }
    }
  }

  bool readLatest(T & value) const
  {
    for (;;) {
      const int selected = published_.load(std::memory_order_acquire);
      if (selected < 0) {
        return false;
      }
      Slot & slot = slots_[static_cast<std::size_t>(selected)];
      slot.readers.fetch_add(1, std::memory_order_acq_rel);
      if (published_.load(std::memory_order_acquire) == selected) {
        value = slot.value;
        slot.readers.fetch_sub(1, std::memory_order_release);
        return true;
      }
      slot.readers.fetch_sub(1, std::memory_order_release);
    }
  }

private:
  struct Slot
  {
    T value{};
    mutable std::atomic<unsigned int> readers{0};
  };

  mutable std::array<Slot, 3> slots_;
  std::atomic<int> published_{-1};
};

}  // namespace motion_control_lab
