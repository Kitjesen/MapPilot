#include "dds.hpp"

#include "message/cpp/qos.hpp"
#include "message/cpp/topics.hpp"
#include "message/cpp/navigation_command.hpp"

#include "dds/dds.h"
#include "messages.h"

#include <cmath>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <stdexcept>
#include <string>

namespace lingtu::driver {
namespace {

dds_entity_t checked(dds_return_t value, const char* operation) {
  if (value < 0) {
    throw std::runtime_error(
        std::string(operation) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

void fillHeader(lingtu_dds_Header& header, double stamp_s, const char* frame_id) {
  if (!std::isfinite(stamp_s) || stamp_s <= 0.0) {
    stamp_s = 0.0;
  }
  header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  header.stamp.nanosec = static_cast<std::uint32_t>(
      (stamp_s - static_cast<double>(header.stamp.sec)) * 1e9);
  header.frame_id = const_cast<char*>(frame_id);
}

std::string readHostBootId() {
  std::ifstream input("/proc/sys/kernel/random/boot_id");
  std::string value;
  std::getline(input, value);
  if (value.empty()) {
    throw std::runtime_error("failed to read Linux host boot id");
  }
  return value;
}

std::uint64_t boottimeNanoseconds() {
  timespec value{};
  if (clock_gettime(CLOCK_BOOTTIME, &value) != 0) {
    throw std::runtime_error("clock_gettime(CLOCK_BOOTTIME) failed");
  }
  return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
      static_cast<std::uint64_t>(value.tv_nsec);
}

}  // namespace

struct DdsReader::Impl {
  explicit Impl(int domain_id) : host_boot_id(readHostBootId()) {
    participant = checked(
        dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
        "dds_create_participant");
    subscriber = checked(
        dds_create_subscriber(participant, nullptr, nullptr), "dds_create_subscriber");
    publisher = checked(
        dds_create_publisher(participant, nullptr, nullptr), "dds_create_publisher");
    topic = checked(
        dds_create_topic(
            participant,
            &lingtu_dds_FinalVelocityCommand_desc,
            lingtu::message::kNavCmdVel.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(cmd_vel)");
    auto qos = lingtu::dds::make_qos(
        lingtu::dds::qos_for_topic(lingtu::message::kNavCmdVel.dds_topic));
    reader = checked(
        dds_create_reader(subscriber, topic, qos.get(), nullptr),
        "dds_create_reader(cmd_vel)");

    control_state_topic = checked(
        dds_create_topic(
            participant,
            &lingtu_dds_DriverControlState_desc,
            lingtu::message::kDriverControlState.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(driver_control_state)");
    auto control_qos = lingtu::dds::make_qos(
        lingtu::dds::qos_for_topic(
            lingtu::message::kDriverControlState.dds_topic));
    control_state_writer = checked(
        dds_create_writer(
            publisher, control_state_topic, control_qos.get(), nullptr),
        "dds_create_writer(driver_control_state)");

    safety_stop_topic = checked(
        dds_create_topic(
            participant,
            &lingtu_dds_NavigationCommandRequest_desc,
            lingtu::message::kNavCommandRequest.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(nav_command_request)");
    auto safety_stop_qos = lingtu::dds::make_qos(
        lingtu::dds::qos_for_topic(
            lingtu::message::kNavCommandRequest.dds_topic));
    safety_stop_writer = checked(
        dds_create_writer(
            publisher, safety_stop_topic, safety_stop_qos.get(), nullptr),
        "dds_create_writer(nav_command_request)");
  }

  ~Impl() {
    if (participant > 0) {
      dds_delete(participant);
    }
  }

  dds_entity_t participant{0};
  dds_entity_t subscriber{0};
  dds_entity_t publisher{0};
  dds_entity_t topic{0};
  dds_entity_t reader{0};
  dds_entity_t control_state_topic{0};
  dds_entity_t control_state_writer{0};
  dds_entity_t safety_stop_topic{0};
  dds_entity_t safety_stop_writer{0};
  std::string host_boot_id;
};

DdsReader::DdsReader(int domain_id) : impl_(std::make_unique<Impl>(domain_id)) {}
DdsReader::~DdsReader() = default;

const std::string& DdsReader::hostBootId() const noexcept {
  return impl_->host_boot_id;
}

std::uint32_t DdsReader::matchedCommandWriters() const {
  const dds_return_t count =
      dds_get_matched_publications(impl_->reader, nullptr, 0);
  if (count < 0) {
    throw std::runtime_error(
        std::string("dds_get_matched_publications(cmd_vel): ") +
        dds_strretcode(-count));
  }
  return static_cast<std::uint32_t>(count);
}

ReadResult DdsReader::takeLatest() {
  constexpr std::size_t kMaxSamples = 8;
  void* samples[kMaxSamples]{};
  dds_sample_info_t infos[kMaxSamples];

  ReadResult result;
  const dds_return_t count =
      dds_take(impl_->reader, samples, infos, kMaxSamples, kMaxSamples);
  if (count < 0) {
    throw std::runtime_error(std::string("dds_take(cmd_vel): ") + dds_strretcode(-count));
  }

  for (dds_return_t i = 0; i < count; ++i) {
    if (!infos[i].valid_data) {
      continue;
    }
    const auto& msg =
        *static_cast<lingtu_dds_FinalVelocityCommand*>(samples[i]);
    TwistSample latest;
    latest.host_boot_id = msg.host_boot_id ? msg.host_boot_id : "";
    latest.producer_boot_id =
        msg.producer_boot_id ? msg.producer_boot_id : "";
    latest.output_seq = msg.output_seq;
    latest.source_boottime_ns = msg.source_boottime_ns;
    latest.source_wall_ns = msg.source_wall_ns;
    latest.receive_boottime_ns = boottimeNanoseconds();
    latest.frame = "body";
    latest.vx = msg.twist.linear.x;
    latest.vy = msg.twist.linear.y;
    latest.wz = msg.twist.angular.z;
    result.latest = std::move(latest);
    ++result.valid_samples;
  }
  if (count > 0) {
    const dds_return_t returned = dds_return_loan(impl_->reader, samples, count);
    if (returned < 0) {
      throw std::runtime_error(
          std::string("dds_return_loan(cmd_vel): ") + dds_strretcode(-returned));
    }
  }
  return result;
}

bool DdsReader::writeControlState(
    const ControlState& state,
    bool last_command_accepted,
    const std::string& accepted_producer_boot_id,
    std::uint64_t accepted_output_sequence,
    double stamp_s) {
  lingtu_dds_DriverControlState msg{};
  fillHeader(msg.header, stamp_s, "body");
  msg.connected = state.connected;
  msg.ready = state.ready;
  msg.motors_enabled = state.motors_enabled;
  msg.critical_fault = state.critical_fault;
  msg.control_assured = state.control_assured;
  msg.lease_valid = state.lease_valid;
  msg.lease_remaining_ms = state.lease_remaining_ms;
  msg.accepted_sequence = state.accepted_sequence;
  msg.accepted_producer_boot_id =
      const_cast<char*>(accepted_producer_boot_id.c_str());
  msg.accepted_output_sequence = accepted_output_sequence;
  msg.last_command_accepted = last_command_accepted;
  msg.fsm = const_cast<char*>(state.fsm.c_str());
  static char kDriverOwner[] = "driver";
  static char kNoOwner[] = "none";
  static char kEmpty[] = "";
  msg.owner = state.control_assured ? kDriverOwner : kNoOwner;
  msg.owner_id = state.control_assured ? const_cast<char*>(kDriverMotionPrincipal) : kEmpty;
  msg.reason = const_cast<char*>(state.reason.c_str());
  return dds_write(impl_->control_state_writer, &msg) >= 0;
}

bool DdsReader::writeNavigationStop(
    const std::string& reason,
    std::uint64_t sequence,
    double stamp_s) {
  lingtu_dds_NavigationCommandRequest msg{};
  fillHeader(msg.header, stamp_s, "body");
  const std::string request_id =
      "lingtu-driver-safety-" + std::to_string(sequence);
  msg.client_id = const_cast<char*>(kDriverMotionPrincipal);
  msg.request_id = const_cast<char*>(request_id.c_str());
  msg.kind = static_cast<std::int32_t>(
      lingtu::message::NavigationCommandKind::Stop);
  msg.reason = const_cast<char*>(reason.c_str());
  return dds_write(impl_->safety_stop_writer, &msg) >= 0;
}

}  // namespace lingtu::driver
