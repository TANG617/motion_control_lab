#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

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
      slot = initial_value;
    }
  }

  LatestValueMailbox(const LatestValueMailbox &) = delete;
  LatestValueMailbox & operator=(const LatestValueMailbox &) = delete;

  void publish(const T & value)
  {
    slots_[back_index_] = value;
    const std::uint32_t previous = middle_.exchange(
      static_cast<std::uint32_t>(back_index_) | kDirty,
      std::memory_order_acq_rel);
    back_index_ = static_cast<std::size_t>(previous & kIndexMask);
  }

  bool readLatest(T & value) const
  {
    if ((middle_.load(std::memory_order_acquire) & kDirty) == 0) {
      return false;
    }
    const std::uint32_t previous = middle_.exchange(
      static_cast<std::uint32_t>(front_index_),
      std::memory_order_acq_rel);
    front_index_ = static_cast<std::size_t>(previous & kIndexMask);
    value = slots_[front_index_];
    return true;
  }

private:
  static constexpr std::uint32_t kDirty = 1U << 31U;
  static constexpr std::uint32_t kIndexMask = 0x3U;

  std::array<T, 3> slots_;
  std::size_t back_index_{0};
  mutable std::size_t front_index_{1};
  mutable std::atomic<std::uint32_t> middle_{2};
};

}  // namespace motion_control_lab
