#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <thread>

#include "platform/runtime.hpp"

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void setHostBootId(const char *value) {
#if defined(_WIN32)
  if (::_putenv_s("LINGTU_HOST_BOOT_ID", value) != 0) {
    throw std::runtime_error("failed to set test host boot id");
  }
#else
  if (::setenv("LINGTU_HOST_BOOT_ID", value, 1) != 0) {
    throw std::runtime_error("failed to set test host boot id");
  }
#endif
}

void testConfiguredIdentity() {
  setHostBootId("test-host-boot-id");
  require(lingtu::nav::platform::hostBootId() == "test-host-boot-id",
          "configured host boot id was not preserved");
  const auto start = lingtu::nav::platform::bootTimeNanoseconds();
  const auto producer =
      lingtu::nav::platform::producerBootId(lingtu::nav::platform::hostBootId(), start);
  require(producer.find("test-host-boot-id:") == 0U,
          "producer identity is not bound to the host boot id");
  require(producer.find(std::to_string(lingtu::nav::platform::processId())) != std::string::npos,
          "producer identity is not bound to the process id");
}

void testClockIsMonotonic() {
  const auto before = lingtu::nav::platform::bootTimeNanoseconds();
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  const auto after = lingtu::nav::platform::bootTimeNanoseconds();
  require(before > 0U && after >= before, "boot clock is not monotonic");
}

void testThreadCpuClockExcludesSleepAndCountsExecution() {
  using lingtu::nav::platform::threadCpuNanoseconds;
  const auto wall_start=std::chrono::steady_clock::now();
  const auto before=threadCpuNanoseconds();
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  const auto after=threadCpuNanoseconds();
  const auto wall_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now()-wall_start).count();
  require(after>=before,"thread CPU time must be monotonic");
  require(after-before<static_cast<std::uint64_t>(wall_ns)/2,
          "thread CPU time must not count sleeping as execution");
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  const auto busy_start=threadCpuNanoseconds();
  auto busy_end=busy_start;
  volatile std::uint64_t work=1;
  do {
    for (int i=0;i<100000;++i) work=work*1664525U+1013904223U;
    busy_end=threadCpuNanoseconds();
  } while(busy_end==busy_start && std::chrono::steady_clock::now()<deadline);
  require(busy_end>busy_start,"thread CPU clock must count actual execution");
}

void testInvalidConfiguredIdentityFailsClosed() {
  setHostBootId("invalid host boot id");
  bool rejected = false;
  try {
    (void)lingtu::nav::platform::hostBootId();
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  require(rejected, "invalid host boot id was accepted");
}

void testOversizedProducerIdentityFailsClosed() {
  bool rejected = false;
  try {
    (void)lingtu::nav::platform::producerBootId(std::string(128U, 'a'), 1U);
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  require(rejected, "oversized producer boot identity was accepted");
}

void testDeadlineSleeperUsesSteadyClockDeadline() {
  lingtu::nav::platform::DeadlineSleeper sleeper;
  const auto before = std::chrono::steady_clock::now();
  sleeper.sleepUntil(before - std::chrono::milliseconds(1));
  const auto expired_elapsed = std::chrono::steady_clock::now() - before;
  require(expired_elapsed < std::chrono::milliseconds(100),
          "expired deadline unexpectedly blocked");

  const auto future_start = std::chrono::steady_clock::now();
  sleeper.sleepUntil(future_start + std::chrono::milliseconds(5));
  const auto future_elapsed = std::chrono::steady_clock::now() - future_start;
  require(future_elapsed >= std::chrono::milliseconds(1),
          "future deadline returned without waiting");
  require(future_elapsed < std::chrono::seconds(1),
          "future deadline wait exceeded the bounded test interval");
}

}  // namespace

int main() {
  testConfiguredIdentity();
  testClockIsMonotonic();
  testThreadCpuClockExcludesSleepAndCountsExecution();
  testInvalidConfiguredIdentityFailsClosed();
  testOversizedProducerIdentityFailsClosed();
  testDeadlineSleeperUsesSteadyClockDeadline();
  return 0;
}
