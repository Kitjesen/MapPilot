#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>

namespace lingtu::drivers::lidar {

template <
    typename RealtimeClock = std::chrono::system_clock,
    typename SteadyClock = std::chrono::steady_clock>
class FallbackTimestampClock final {
 public:
  static_assert(SteadyClock::is_steady, "fallback clock must use a steady clock");

  FallbackTimestampClock() noexcept
      : realtime_anchor_ns_(epoch_ns(RealtimeClock::now())),
        steady_anchor_(SteadyClock::now()),
        last_timestamp_ns_(realtime_anchor_ns_) {}

  std::uint64_t now_ns() const noexcept {
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
        SteadyClock::now() - steady_anchor_).count();
    std::uint64_t candidate = realtime_anchor_ns_;
    if (elapsed > 0) {
      const auto elapsed_ns = static_cast<std::uint64_t>(elapsed);
      candidate = elapsed_ns > std::numeric_limits<std::uint64_t>::max() - candidate
          ? std::numeric_limits<std::uint64_t>::max()
          : candidate + elapsed_ns;
    }
    return clamp_monotonic(candidate);
  }

 private:
  static std::uint64_t epoch_ns(typename RealtimeClock::time_point time) noexcept {
    const auto count = std::chrono::duration_cast<std::chrono::nanoseconds>(
        time.time_since_epoch()).count();
    return count > 0 ? static_cast<std::uint64_t>(count) : 0U;
  }

  std::uint64_t clamp_monotonic(std::uint64_t candidate) const noexcept {
    std::uint64_t previous = last_timestamp_ns_.load(std::memory_order_relaxed);
    while (candidate > previous) {
      if (last_timestamp_ns_.compare_exchange_weak(
              previous,
              candidate,
              std::memory_order_relaxed,
              std::memory_order_relaxed)) {
        return candidate;
      }
    }
    return previous;
  }

  const std::uint64_t realtime_anchor_ns_;
  const typename SteadyClock::time_point steady_anchor_;
  mutable std::atomic<std::uint64_t> last_timestamp_ns_;
};

}  // namespace lingtu::drivers::lidar
