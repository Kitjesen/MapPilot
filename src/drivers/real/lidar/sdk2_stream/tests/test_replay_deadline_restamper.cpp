#include "replay_deadline_restamper.hpp"

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

using Restamper = lingtu::drivers::lidar::ReplayDeadlineRestamper<
    FakeRealtimeClock,
    FakeSteadyClock>;

constexpr std::uint64_t kNanosecondsPerSecond = 1000000000ULL;

}  // namespace

int main() {
  using namespace std::chrono;

  FakeRealtimeClock::current = FakeRealtimeClock::time_point(seconds(1000));
  FakeSteadyClock::current = FakeSteadyClock::time_point(seconds(10));
  Restamper restamper;

  const auto first_deadline = FakeSteadyClock::current;
  const std::uint64_t first = restamper.stamp_ns(100U, first_deadline);
  assert(first == 1000ULL * kNanosecondsPerSecond);

  FakeRealtimeClock::current += milliseconds(10);
  FakeSteadyClock::current += milliseconds(10);
  const auto second_deadline = FakeSteadyClock::current;
  const std::uint64_t second = restamper.stamp_ns(110U, second_deadline);
  assert(second - first == 10000000ULL);

  // Sequential records from one observation retain an identical timestamp.
  FakeRealtimeClock::current += milliseconds(2);
  FakeSteadyClock::current += milliseconds(2);
  assert(restamper.stamp_ns(110U, second_deadline) == second);

  // A real processing delay is kept as age instead of being stamped fresh.
  const auto delayed_deadline = FakeSteadyClock::current - milliseconds(250);
  const std::uint64_t delayed_now = static_cast<std::uint64_t>(
      duration_cast<nanoseconds>(FakeRealtimeClock::current.time_since_epoch()).count());
  const std::uint64_t delayed = restamper.stamp_ns(120U, delayed_deadline);
  assert(delayed_now - delayed == 250000000ULL);

  // Realtime may step backwards under WSL clock synchronization. A cached
  // value that became future is discarded, even for the same source stamp.
  FakeRealtimeClock::current -= milliseconds(700);
  FakeSteadyClock::current += milliseconds(1);
  const std::uint64_t backward_now = static_cast<std::uint64_t>(
      duration_cast<nanoseconds>(FakeRealtimeClock::current.time_since_epoch()).count());
  const std::uint64_t after_backward_step =
      restamper.stamp_ns(120U, FakeSteadyClock::current);
  assert(after_backward_step == backward_now);
  assert(after_backward_step < delayed);

  // A forward step is also followed immediately rather than retaining a fixed
  // startup realtime/steady offset.
  FakeRealtimeClock::current += seconds(3);
  FakeSteadyClock::current += milliseconds(10);
  const std::uint64_t forward_now = static_cast<std::uint64_t>(
      duration_cast<nanoseconds>(FakeRealtimeClock::current.time_since_epoch()).count());
  assert(restamper.stamp_ns(130U, FakeSteadyClock::current) == forward_now);

  return 0;
}
