#include "fallback_timestamp_clock.hpp"

#include <cassert>
#include <chrono>
#include <cstdint>

namespace {

struct FakeRealtimeClock {
  using duration = std::chrono::nanoseconds;
  using rep = duration::rep;
  using period = duration::period;
  using time_point = std::chrono::time_point<FakeRealtimeClock>;
  static constexpr bool is_steady = false;

  static time_point now() noexcept { return current; }

  static inline time_point current{};
};

struct FakeSteadyClock {
  using duration = std::chrono::nanoseconds;
  using rep = duration::rep;
  using period = duration::period;
  using time_point = std::chrono::time_point<FakeSteadyClock>;
  static constexpr bool is_steady = true;

  static time_point now() noexcept { return current; }

  static inline time_point current{};
};

using TestClock = lingtu::drivers::lidar::FallbackTimestampClock<
    FakeRealtimeClock,
    FakeSteadyClock>;

constexpr std::uint64_t kNanosecondsPerSecond = 1000000000ULL;

}  // namespace

int main() {
  using namespace std::chrono;

  constexpr std::int64_t kEpochSeconds = 1720000000LL;
  FakeRealtimeClock::current = FakeRealtimeClock::time_point(seconds(kEpochSeconds));
  FakeSteadyClock::current = FakeSteadyClock::time_point(seconds(42));

  TestClock clock;
  const std::uint64_t initial = clock.now_ns();
  assert(initial == static_cast<std::uint64_t>(kEpochSeconds) * kNanosecondsPerSecond);
  assert(initial > 1577836800ULL * kNanosecondsPerSecond);  // 2020-01-01 UTC

  FakeRealtimeClock::current += hours(13);
  FakeSteadyClock::current += milliseconds(100);
  const std::uint64_t after_forward_jump = clock.now_ns();
  assert(after_forward_jump - initial == 100000000ULL);

  FakeRealtimeClock::current -= hours(26);
  FakeSteadyClock::current += milliseconds(100);
  const std::uint64_t after_backward_jump = clock.now_ns();
  assert(after_backward_jump - after_forward_jump == 100000000ULL);

  FakeSteadyClock::current -= seconds(1);
  assert(clock.now_ns() == after_backward_jump);

  return 0;
}
