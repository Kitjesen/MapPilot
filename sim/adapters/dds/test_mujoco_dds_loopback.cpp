#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <thread>

#include <dds/dds.h>

#include "messages.h"
#include "message/cpp/qos.hpp"
#include "message/cpp/topics.hpp"
#include "test_dds_domain.hpp"

namespace {

dds_entity_t checked(dds_return_t value, const char* what) {
  if (value < 0) {
    throw std::runtime_error(std::string(what) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

int testDomainId() {
  return lingtu::sim::dds_adapter::test::domain_id_from_environment();
}

bool closeEnough(double left, double right) {
  return std::abs(left - right) < 1e-9;
}

}  // namespace

int main() {
  dds_entity_t reader_participant = 0;
  dds_entity_t writer_participant = 0;
  try {
    const int domain_id = testDomainId();
    reader_participant = checked(
        dds_create_participant(
            static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
        "dds_create_participant(reader)");
    writer_participant = checked(
        dds_create_participant(
            static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
        "dds_create_participant(writer)");

    const auto& contract = lingtu::message::kImuRaw;
    const dds_entity_t reader_topic = checked(
        dds_create_topic(
            reader_participant,
            &lingtu_dds_Imu_desc,
            contract.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(reader)");
    const dds_entity_t writer_topic = checked(
        dds_create_topic(
            writer_participant,
            &lingtu_dds_Imu_desc,
            contract.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(writer)");
    auto reader_qos =
        lingtu::dds::make_qos(lingtu::dds::qos_for_topic(contract.dds_topic));
    auto writer_qos =
        lingtu::dds::make_qos(lingtu::dds::qos_for_topic(contract.dds_topic));
    const dds_entity_t reader = checked(
        dds_create_reader(reader_participant, reader_topic, reader_qos.get(), nullptr),
        "dds_create_reader");
    const dds_entity_t writer = checked(
        dds_create_writer(writer_participant, writer_topic, writer_qos.get(), nullptr),
        "dds_create_writer");

    lingtu_dds_Imu sample{};
    sample.header.stamp.sec = 123;
    sample.header.stamp.nanosec = 456;
    sample.header.frame_id = const_cast<char*>("imu_link");
    sample.orientation.w = 1.0;
    sample.angular_velocity.x = 0.25;
    sample.angular_velocity.y = -0.5;
    sample.angular_velocity.z = 0.75;
    sample.linear_acceleration.x = 1.5;
    sample.linear_acceleration.y = 2.5;
    sample.linear_acceleration.z = 9.81;

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
      checked(dds_write(writer, &sample), "dds_write");
      void* samples[1]{};
      dds_sample_info_t infos[1]{};
      const dds_return_t count = dds_take(reader, samples, infos, 1, 1);
      if (count < 0) {
        throw std::runtime_error(
            std::string("dds_take: ") + dds_strretcode(-count));
      }
      if (count == 1 && infos[0].valid_data && samples[0] != nullptr) {
        const auto* received = static_cast<const lingtu_dds_Imu*>(samples[0]);
        const bool matches =
            received->header.stamp.sec == sample.header.stamp.sec &&
            received->header.stamp.nanosec == sample.header.stamp.nanosec &&
            received->header.frame_id != nullptr &&
            std::string(received->header.frame_id) == "imu_link" &&
            closeEnough(received->angular_velocity.x, 0.25) &&
            closeEnough(received->angular_velocity.y, -0.5) &&
            closeEnough(received->angular_velocity.z, 0.75) &&
            closeEnough(received->linear_acceleration.x, 1.5) &&
            closeEnough(received->linear_acceleration.y, 2.5) &&
            closeEnough(received->linear_acceleration.z, 9.81);
        checked(dds_return_loan(reader, samples, count), "dds_return_loan");
        if (!matches) {
          throw std::runtime_error("received IMU payload does not match");
        }
        dds_delete(reader_participant);
        dds_delete(writer_participant);
        std::printf(
            "native DDS loopback passed: domain=%d topic=%s\n",
            domain_id,
            contract.dds_topic.data());
        return 0;
      }
      if (count > 0) {
        checked(dds_return_loan(reader, samples, count), "dds_return_loan");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::runtime_error("timed out waiting for IMU loopback sample");
  } catch (const std::exception& exc) {
    if (reader_participant > 0) {
      dds_delete(reader_participant);
    }
    if (writer_participant > 0) {
      dds_delete(writer_participant);
    }
    std::fprintf(stderr, "test_mujoco_dds_loopback failed: %s\n", exc.what());
    return 1;
  }
}
