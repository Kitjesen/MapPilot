#include <chrono>
#include <cstdio>
#include <stdexcept>

#include "core.hpp"
#include "status/control_loop_health.hpp"

namespace {
void require(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
}  // namespace

int main() {
  using namespace std::chrono_literals;
  using namespace lingtu::nav::endpoint;
  try {
    ControlLoopHealthConfig config;
    config.period_ms = 10.0;
    ControlLoopHealth health(config);
    lingtu::driver::Core driver({1.0, 1.0, 200ms}, "host");
    auto now = lingtu::driver::Clock::now();
    for (std::uint64_t sequence = 1; sequence <= 200; ++sequence) {
      now += 20ms;
      const auto command = driver.accept(
          {"host", "nav", sequence, now.time_since_epoch(), now.time_since_epoch()},
          "body", 0.3, 0.0, 0.0, now);
      require(command.action && command.action->velocity.vx_mps == 0.3,
              "fresh commands below the target frequency must remain executable");
      require(!driver.poll(now), "live commands triggered the watchdog");
      health.observe({20.0, 0.0, 10.0});
    }
    const auto diagnostic = health.snapshot();
    require(diagnostic.ready && !diagnostic.healthy &&
                diagnostic.reason == "consecutive_deadline_misses" &&
                diagnostic.current_miss_streak == 200,
            "slow-cycle diagnostics were lost");
    require(!driver.poll(now + 199ms), "watchdog stopped fresh motion early");
    const auto stopped = driver.poll(now + 200ms);
    require(stopped && stopped->reason == lingtu::driver::ActionReason::Watchdog &&
                stopped->velocity.vx_mps == 0.0 && stopped->velocity.vy_mps == 0.0 &&
                stopped->velocity.yaw_rps == 0.0,
            "a real command gap must still stop all motion");
    const auto expired = driver.accept(
        {"host", "nav", 201, now.time_since_epoch(), (now + 201ms).time_since_epoch()},
        "body", 0.3, 0.0, 0.0, now + 201ms);
    require(!expired.action && !expired.freshness.accepted,
            "an old command restarted motion after the watchdog stop");
    std::puts("200 fresh 20ms commands accepted; performance warning retained; 200ms gap stopped");
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "test_control_cadence: %s\n", error.what());
    return 1;
  }
}
