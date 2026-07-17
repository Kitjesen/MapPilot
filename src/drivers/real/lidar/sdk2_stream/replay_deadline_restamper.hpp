#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

namespace lingtu::drivers::lidar {

template <
    typename RealtimeClock = std::chrono::system_clock,
    typename SteadyClock = std::chrono::steady_clock>
class ReplayDeadlineRestamper final {
 public:
  static_assert(SteadyClock::is_steady, "replay deadline clock must be steady");

  using SteadyTimePoint = typename SteadyClock::time_point;

  std::uint64_t stamp_ns(
      std::uint64_t source_timestamp_ns,
      SteadyTimePoint target_deadline) noexcept {
    const std::uint64_t realtime_now_ns = epoch_ns(RealtimeClock::now());

    // Preserve equality for records generated from the same observation. If
    // realtime stepped backwards between those records, the cached value would
    // be in the future and must be discarded instead.
    if (cached_source_ns_ == source_timestamp_ns &&
        cached_output_ns_.has_value() &&
        *cached_output_ns_ <= realtime_now_ns) {
      return *cached_output_ns_;
    }

    const auto steady_now = SteadyClock::now();
    std::uint64_t lateness_ns = 0U;
    if (steady_now > target_deadline) {
      const auto lateness = std::chrono::duration_cast<std::chrono::nanoseconds>(
          steady_now - target_deadline).count();
      if (lateness > 0) {
        lateness_ns = static_cast<std::uint64_t>(lateness);
      }
    }
    const std::uint64_t output_ns = lateness_ns < realtime_now_ns
        ? realtime_now_ns - lateness_ns
        : 0U;
    cached_source_ns_ = source_timestamp_ns;
    cached_output_ns_ = output_ns;
    return output_ns;
  }

 private:
  static std::uint64_t epoch_ns(typename RealtimeClock::time_point time) noexcept {
    const auto count = std::chrono::duration_cast<std::chrono::nanoseconds>(
        time.time_since_epoch()).count();
    return count > 0 ? static_cast<std::uint64_t>(count) : 0U;
  }

  std::optional<std::uint64_t> cached_source_ns_;
  std::optional<std::uint64_t> cached_output_ns_;
};

}  // namespace lingtu::drivers::lidar
