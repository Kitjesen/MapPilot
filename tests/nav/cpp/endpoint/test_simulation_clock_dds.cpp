#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <thread>

#include "dds/runtime.hpp"
#include "transport/dds/qos.hpp"
#include "message/generated/topics.hpp"

namespace {
void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

void testPhysicalClockRoundTrip() {
  constexpr int domain = 123;
  lingtu::nav::endpoint::Dds simulation(domain, nullptr, false, true);
  lingtu::nav::endpoint::Dds real(domain, nullptr, false);
  const auto participant = dds_create_participant(domain, nullptr, nullptr);
  require(participant > 0, "clock peer participant");
  const auto topic = dds_create_topic(participant, &lingtu_dds_Time_desc,
                                     lingtu::message::kSimClock.dds_topic.data(), nullptr, nullptr);
  auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(
      lingtu::message::kSimClock.dds_topic.data()));
  const auto writer = dds_create_writer(participant, topic, qos.get(), nullptr);
  require(writer > 0, "clock peer writer");
  const auto exchange = [&](lingtu_dds_Time value, double expected) {
    for (int attempt = 0; attempt < 100; ++attempt) {
      require(dds_write(writer, &value) == DDS_RETCODE_OK, "clock write");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      const auto batch = simulation.takeSensors(100.0);
      require(!real.takeSensors(100.0).simulation_time_s,
              "real runtime must not subscribe to simulation time");
      if (batch.simulation_time_s && std::abs(*batch.simulation_time_s - expected) < 1e-9)
        return;
    }
    throw std::runtime_error("physical clock did not arrive intact");
  };
  exchange({0, 0}, 0.0);
  exchange({2, 50'000'000U}, 2.05);
  exchange({2, 50'000'000U}, 2.05);
  // Drain valid data before proving malformed values cannot refresh the clock.
  (void)simulation.takeSensors(100.0);
  for (const auto bad : {lingtu_dds_Time{-1, 0}, lingtu_dds_Time{2, 1'000'000'000U}}) {
    require(dds_write(writer, &bad) == DDS_RETCODE_OK, "invalid clock write");
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    require(!simulation.takeSensors(100.0).simulation_time_s,
            "invalid physical time must not refresh the execution clock");
  }
  dds_delete(participant);
}
}  // namespace

int main() {
  try {
    testPhysicalClockRoundTrip();
    std::puts("test_simulation_clock_dds: PASS");
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "test_simulation_clock_dds: FAIL: %s\n", error.what());
    return 1;
  }
}
