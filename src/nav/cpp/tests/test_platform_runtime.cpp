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
  testInvalidConfiguredIdentityFailsClosed();
  testOversizedProducerIdentityFailsClosed();
  testDeadlineSleeperUsesSteadyClockDeadline();
  return 0;
}
