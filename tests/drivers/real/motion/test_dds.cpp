#include <chrono>
#include <cstdio>
#include <ctime>
#include <stdexcept>
#include <string>
#include <thread>

#include "dds.hpp"
#include "dds/dds.h"
#include "messages.h"
#include "transport/dds/qos.hpp"

namespace {
void check(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
}  // namespace

int main() {
  using namespace std::chrono_literals;
  dds_entity_t participant = 0;
  try {
    auto velocity_qos = lingtu::dds::make_qos(lingtu::dds::QosProfile::FinalVelocityCommand);
    dds_reliability_kind_t reliability{};
    dds_duration_t blocking{};
    check(dds_qget_reliability(velocity_qos.get(), &reliability, &blocking) &&
              reliability == DDS_RELIABILITY_BEST_EFFORT && blocking == 0,
          "velocity publishing must not wait for reliable delivery");
    auto request_qos = lingtu::dds::make_qos(lingtu::dds::QosProfile::CommandRequest);
    check(dds_qget_reliability(request_qos.get(), &reliability, &blocking) &&
              reliability == DDS_RELIABILITY_RELIABLE,
          "task requests must retain reliable delivery");
    constexpr int domain = 94;
    participant = dds_create_participant(domain, nullptr, nullptr);
    check(participant > 0, "DDS test participant failed");
    const auto reader = [&](const dds_topic_descriptor_t &type, std::string_view name) {
      const auto topic = dds_create_topic(participant, &type, name.data(), nullptr, nullptr);
      check(topic > 0, "DDS test topic failed");
      auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(name));
      const auto value = dds_create_reader(participant, topic, qos.get(), nullptr);
      check(value > 0, "DDS test reader failed");
      return value;
    };
    const auto control = reader(lingtu_dds_DriverControlState_desc,
                                lingtu::message::kDriverControlState.dds_topic);
    const auto commands = reader(lingtu_dds_NavigationCommandRequest_desc,
                                 lingtu::message::kNavCommandRequest.dds_topic);
    lingtu::driver::DdsReader driver(domain);
    for (const bool ready : {false, true}) {
      lingtu::driver::ControlState state;
      state.connected = ready;
      state.ready = ready;
      state.motors_enabled = true;
      state.control_assured = ready;
      state.reason = ready ? "ready" : "rpc_timeout:code=3104";
      bool observed = false;
      for (int attempt = 0; attempt < 100 && !observed; ++attempt) {
        check(driver.writeControlState(state, false, 123.0), "status write failed");
        std::this_thread::sleep_for(10ms);
        void *raw = nullptr;
        dds_sample_info_t info{};
        const auto count = dds_take(control, &raw, &info, 1, 1);
        check(count >= 0, "status read failed");
        if (count > 0) {
          const auto &sample = *static_cast<lingtu_dds_DriverControlState *>(raw);
          observed = info.valid_data && sample.ready == ready && sample.reason &&
                     std::string(sample.reason) == state.reason;
          check(dds_return_loan(control, &raw, 1) >= 0, "status loan return failed");
        }
        check(dds_get_matched_publications(commands, nullptr, 0) == 0,
              "driver must report control loss without publishing task commands");
      }
      check(observed, "driver fault/recovery state did not arrive");
    }
    const auto topic = dds_create_topic(participant, &lingtu_dds_FinalVelocityCommand_desc,
        lingtu::message::kNavCmdVel.dds_topic.data(), nullptr, nullptr);
    check(topic > 0, "velocity test topic failed");
    const auto writer = dds_create_writer(participant, topic, velocity_qos.get(), nullptr);
    check(writer > 0, "velocity test writer failed");
    for (int attempt = 0; attempt < 100 && driver.matchedCommandWriters() == 0; ++attempt)
      std::this_thread::sleep_for(10ms);
    check(driver.matchedCommandWriters() == 1, "velocity endpoints did not match");
    // Publishing continues while the driver does not consume the stream.
    for (std::uint64_t sequence = 1; sequence <= 40; ++sequence) {
      timespec now{};
      check(clock_gettime(CLOCK_BOOTTIME, &now) == 0, "boottime read failed");
      lingtu_dds_FinalVelocityCommand value{};
      value.host_boot_id = const_cast<char *>(driver.hostBootId().c_str());
      value.producer_boot_id = const_cast<char *>("test-nav");
      value.output_seq = sequence;
      value.source_boottime_ns = static_cast<std::uint64_t>(now.tv_sec) * 1000000000ULL + now.tv_nsec;
      value.twist.linear.x = sequence == 40 ? 0.0 : 0.3;
      check(dds_write(writer, &value) >= 0, "velocity stream publication failed");
      std::this_thread::sleep_for(1ms);
    }
    std::this_thread::sleep_for(10ms);
    const auto latest = driver.takeLatest();
    check(latest.valid_samples == 1 && latest.latest &&
              latest.latest->vx == 0.0,
          "slow consumption replayed old motion instead of the latest zero");
    check(latest.latest->received_at != lingtu::driver::TimePoint{},
          "DDS reception must capture the local steady time for command expiry");
    check(!driver.takeLatest().latest, "old velocities remained queued behind the latest value");
    dds_delete(participant);
    std::puts("Driver status delivered; velocity stream retained only the latest zero");
    return 0;
  } catch (const std::exception &error) {
    if (participant > 0) dds_delete(participant);
    std::fprintf(stderr, "test_driver_dds: %s\n", error.what());
    return 1;
  }
}
