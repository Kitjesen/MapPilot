#pragma once

#include <atomic>
#include <cstdint>
#include <limits>

namespace lingtu::drivers::lidar {

// The native stream currently owns one ScanAccumulator and one timestamp
// domain. Reject a second physical device instead of silently merging clouds.
class SingleLidarHandleGuard final {
 public:
  bool accept(std::uint32_t handle) noexcept {
    std::uint32_t expected = kUnset;
    if (active_.compare_exchange_strong(
            expected, handle, std::memory_order_relaxed)) {
      return true;
    }
    return expected == handle;
  }

 private:
  static constexpr std::uint32_t kUnset =
      std::numeric_limits<std::uint32_t>::max();
  std::atomic<std::uint32_t> active_{kUnset};
};

}  // namespace lingtu::drivers::lidar
