#pragma once

#include <array>
#include <cstddef>
#include <limits>
#include <mutex>
#include <type_traits>

#include "lingtu/sim/runtime_contracts.hpp"

namespace lingtu::sim {

enum class SnapshotPushResult {
  published,
  published_replacing_latest,
  dropped_no_free_slot,
};

// Bounded mutex-protected latest-value transport.
//
// Storage is fully preallocated and snapshot copies do not allocate. All
// operations are linearizable, so readers see either the previous complete
// snapshot or its complete replacement. The mutex can block a producer or
// reader for the duration of a large snapshot copy; this is therefore not a
// lock-free or hard-real-time primitive and should stay outside a timing-
// critical physics section. push() waits for that lock instead of dropping a
// value; dropped_no_free_slot remains in the result enum for API compatibility.
//
// pop_latest() is a global consume operation for a single logical consumer.
// Use latest() for multiple independent readers; latest() is non-destructive.
// This is an in-process transport only. It is not a DDS/CDR or shared-memory
// wire ABI, and it intentionally does not expose a pointer to a slot.
template <std::size_t Capacity = 3>
class SnapshotBus final {
 public:
  static_assert(Capacity > 0, "SnapshotBus capacity must be positive");
  static_assert(std::is_nothrow_copy_assignable_v<TruthSnapshotEnvelope>,
                "snapshot copies must not allocate or throw");

  SnapshotBus() noexcept = default;
  SnapshotBus(const SnapshotBus &) = delete;
  SnapshotBus &operator=(const SnapshotBus &) = delete;

  SnapshotPushResult push(const TruthSnapshotEnvelope &snapshot) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool replacing = published_index_ != kNoSlot;
    slots_[producer_cursor_] = snapshot;
    published_index_ = producer_cursor_;
    producer_cursor_ = increment(producer_cursor_);
    return replacing ? SnapshotPushResult::published_replacing_latest
                     : SnapshotPushResult::published;
  }

  bool pop_latest(TruthSnapshotEnvelope &out) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (published_index_ == kNoSlot) {
      return false;
    }
    out = slots_[published_index_];
    published_index_ = kNoSlot;
    return true;
  }

  bool latest(TruthSnapshotEnvelope &out) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (published_index_ == kNoSlot) {
      return false;
    }
    out = slots_[published_index_];
    return true;
  }

  void clear() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    published_index_ = kNoSlot;
  }

  [[nodiscard]] std::size_t size() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return published_index_ == kNoSlot ? 0 : 1;
  }
  [[nodiscard]] constexpr std::size_t capacity() const noexcept { return Capacity; }

 private:
  static constexpr std::size_t kNoSlot = std::numeric_limits<std::size_t>::max();

  [[nodiscard]] static constexpr std::size_t increment(std::size_t index) noexcept {
    return index + 1 == Capacity ? 0 : index + 1;
  }

  mutable std::mutex mutex_;
  std::array<TruthSnapshotEnvelope, Capacity> slots_{};
  mutable std::size_t published_index_{kNoSlot};
  std::size_t producer_cursor_{0};
};

}  // namespace lingtu::sim
