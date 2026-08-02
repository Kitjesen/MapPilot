#include <chrono>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>

#include "dds/dds.h"
#include "lingtu_slam.h"
#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/dds_topics.hpp"
#include "nav_dds_runtime.hpp"

namespace {

using namespace std::chrono_literals;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

dds_entity_t checked(dds_return_t value, const char *operation) {
  if (value < 0) {
    throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

dds_entity_t createWriter(dds_entity_t participant, const lingtu::message::TopicContract &contract,
                          const dds_topic_descriptor_t *descriptor) {
  const dds_entity_t topic = checked(
      dds_create_topic(participant, descriptor, contract.dds_topic.data(), nullptr, nullptr),
      "dds_create_topic(test_writer)");
  auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(contract.dds_topic));
  return checked(dds_create_writer(participant, topic, qos.get(), nullptr),
                 "dds_create_writer(test)");
}

dds_entity_t createReader(dds_entity_t participant, const lingtu::message::TopicContract &contract,
                          const dds_topic_descriptor_t *descriptor) {
  const dds_entity_t topic = checked(
      dds_create_topic(participant, descriptor, contract.dds_topic.data(), nullptr, nullptr),
      "dds_create_topic(test_reader)");
  auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(contract.dds_topic));
  return checked(dds_create_reader(participant, topic, qos.get(), nullptr),
                 "dds_create_reader(test)");
}

template <typename Message, typename Check>
bool takeAndCheck(dds_entity_t reader, Check &&check) {
  void *samples[1]{};
  dds_sample_info_t infos[1]{};
  const dds_return_t count = dds_take(reader, samples, infos, 1, 1);
  checked(count, "dds_take(test)");
  bool matched = false;
  if (count == 1 && infos[0].valid_data) {
    check(*static_cast<Message *>(samples[0]));
    matched = true;
  }
  if (count > 0) {
    checked(dds_return_loan(reader, samples, count), "dds_return_loan(test)");
  }
  return matched;
}

void testTypedOperatorMotionRoundTrip() {
  constexpr int kDomain = 122;
  lingtu::nav::endpoint::DdsRuntime runtime(kDomain);
  const dds_entity_t participant = checked(dds_create_participant(kDomain, nullptr, nullptr),
                                           "dds_create_participant(test_peer)");
  const dds_entity_t control_writer = createWriter(
      participant, lingtu::message::kOperatorMotionControl, &lingtu_dds_OperatorMotionControl_desc);
  const dds_entity_t sample_writer = createWriter(
      participant, lingtu::message::kOperatorMotionSample, &lingtu_dds_OperatorMotionSample_desc);
  const dds_entity_t ack_reader = createReader(participant, lingtu::message::kOperatorMotionAck,
                                               &lingtu_dds_OperatorMotionAck_desc);
  const dds_entity_t status_reader = createReader(
      participant, lingtu::message::kOperatorMotionStatus, &lingtu_dds_OperatorMotionStatus_desc);

  lingtu_dds_OperatorMotionControl control{};
  control.source_id = const_cast<char *>("ws:test");
  control.source_epoch = 17U;
  control.source_sequence = 3U;
  control.request_id = const_cast<char *>("claim-3");
  control.action = 1;
  control.lease_ttl_ms = 1000U;
  control.reason = const_cast<char *>("claim");

  bool control_seen = false;
  for (int attempt = 0; attempt < 100 && !control_seen; ++attempt) {
    checked(dds_write(control_writer, &control), "dds_write(test_control)");
    std::this_thread::sleep_for(10ms);
    runtime.drainOperatorMotionControls([&](const lingtu_dds_OperatorMotionControl &received) {
      require(std::string(received.source_id) == "ws:test", "control source mismatch");
      require(received.source_epoch == 17U, "control epoch mismatch");
      require(received.source_sequence == 3U, "control sequence mismatch");
      require(std::string(received.request_id) == "claim-3", "control request mismatch");
      control_seen = true;
    });
  }
  require(control_seen, "operator control reader received no sample");

  lingtu_dds_OperatorMotionSample sample{};
  sample.header.frame_id = const_cast<char *>("body");
  sample.source_id = const_cast<char *>("ws:test");
  sample.source_epoch = 17U;
  sample.source_sequence = 4U;
  sample.request_id = const_cast<char *>("sample-4");
  sample.deadman = true;
  sample.velocity.linear.x = 0.2;
  sample.velocity.angular.z = -0.1;
  sample.freshness_budget_ms = 350U;
  sample.source_stamp_ns = 123456789U;

  bool sample_seen = false;
  for (int attempt = 0; attempt < 100 && !sample_seen; ++attempt) {
    checked(dds_write(sample_writer, &sample), "dds_write(test_sample)");
    std::this_thread::sleep_for(10ms);
    runtime.drainOperatorMotionSamples([&](const lingtu_dds_OperatorMotionSample &received) {
      require(received.source_sequence == 4U, "sample sequence mismatch");
      require(received.deadman, "sample deadman mismatch");
      require(received.velocity.linear.x == 0.2, "sample velocity mismatch");
      sample_seen = true;
    });
  }
  require(sample_seen, "operator sample reader received no sample");

  lingtu::nav::endpoint::OperatorMotionAckSample ack;
  ack.source_id = "ws:test";
  ack.source_epoch = 17U;
  ack.sequence = 5U;
  ack.request_id = "hold-5";
  ack.action = 3;
  ack.accepted = true;
  ack.reason = "hold_zero_published";
  ack.accepted_sequence = 5U;
  ack.final_output_sequence = 9U;
  require(runtime.writeOperatorMotionAck(ack), "operator ACK write failed");
  bool ack_seen = false;
  for (int attempt = 0; attempt < 100 && !ack_seen; ++attempt) {
    std::this_thread::sleep_for(10ms);
    ack_seen = takeAndCheck<lingtu_dds_OperatorMotionAck>(ack_reader, [&](const auto &received) {
      require(std::string(received.request_id) == "hold-5", "ACK request mismatch");
      require(received.source_sequence == 5U, "ACK sequence mismatch");
      require(received.final_output_sequence == 9U, "ACK final output sequence mismatch");
    });
  }
  require(ack_seen, "operator ACK reader received no sample");

  lingtu::nav::endpoint::OperatorMotionStatusSample status;
  status.active_source_id = "ws:test";
  status.active_source_epoch = 17U;
  status.has_active_authority = true;
  status.holding = true;
  status.last_sample_sequence = 4U;
  status.admitted_sequence = 4U;
  status.final_output_sequence = 9U;
  status.authority_reason = "zero_barrier_complete";
  status.input_gate_reason = "ready";
  status.teleop_output = {0.2, 0.0, -0.1};
  require(runtime.writeOperatorMotionStatus(status), "operator status write failed");
  bool status_seen = false;
  for (int attempt = 0; attempt < 100 && !status_seen; ++attempt) {
    std::this_thread::sleep_for(10ms);
    status_seen =
        takeAndCheck<lingtu_dds_OperatorMotionStatus>(status_reader, [&](const auto &received) {
          require(std::string(received.active_source_id) == "ws:test", "status source mismatch");
          require(received.last_sample_sequence == 4U, "status sample sequence mismatch");
          require(received.admitted_sequence == 4U, "status admitted sequence mismatch");
          require(received.final_output_sequence == 9U, "status final output sequence mismatch");
        });
  }
  require(status_seen, "operator status reader received no sample");
  dds_delete(participant);
}

}  // namespace

int main() {
  try {
    testTypedOperatorMotionRoundTrip();
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_operator_motion_dds: FAIL: %s\n", exc.what());
    return 1;
  }
}
