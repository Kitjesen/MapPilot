#include "perception/inspection/native_bridge.h"

#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/dds_topics.hpp"

#include "dds/dds.h"
#include "lingtu_slam.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

using namespace std::chrono_literals;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

dds_entity_t checked(dds_return_t value, const char* operation) {
  if (value < 0) {
    throw std::runtime_error(
        std::string(operation) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

void fillHeader(lingtu_dds_Header& header, double stamp_s) {
  header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_s - static_cast<double>(header.stamp.sec)) * 1e9);
  header.frame_id = const_cast<char*>("map");
}

struct ResultSnapshot {
  int32_t stamp_sec{};
  std::string request_id;
  std::string evidence_id;
  bool persisted{};
  std::string reason;
  std::string analysis_verdict;
};

class BridgePeer {
 public:
  explicit BridgePeer(int domain_id) {
    participant_ = checked(
        dds_create_participant(
            static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
        "dds_create_participant(peer)");
    request_writer_ = makeWriter(
        lingtu::message::kNavInspectionEvidenceRequest.dds_topic.data(),
        &lingtu_dds_InspectionEvidenceRequest_desc);
    result_reader_ = makeReader(
        lingtu::message::kNavInspectionEvidenceResult.dds_topic.data(),
        &lingtu_dds_InspectionEvidenceResult_desc);
  }

  ~BridgePeer() {
    if (participant_ > 0) dds_delete(participant_);
  }

  void writeRequest(const std::string& request_id) {
    lingtu_dds_InspectionEvidenceRequest msg{};
    fillHeader(msg.header, 123.456);
    msg.request_id = const_cast<char*>(request_id.c_str());
    msg.run_id = const_cast<char*>("run-a");
    msg.route_id = const_cast<char*>("route-a");
    msg.revision = 9;
    msg.map_id = const_cast<char*>("map-a");
    msg.map_version = 4;
    msg.point_index = 2;
    msg.point_id = const_cast<char*>("point-a");
    msg.action = const_cast<char*>("capture:overview");
    msg.deadline_s = 130.0;
    checked(dds_write(request_writer_, &msg), "dds_write(request)");
  }

  bool takeResult(ResultSnapshot* output) {
    void* samples[1]{};
    dds_sample_info_t infos[1]{};
    const dds_return_t count = dds_take(result_reader_, samples, infos, 1, 1);
    checked(count, "dds_take(result)");
    if (count == 0) return false;
    const bool valid = infos[0].valid_data;
    if (valid) {
      const auto* wire =
          static_cast<const lingtu_dds_InspectionEvidenceResult*>(samples[0]);
      output->stamp_sec = wire->header.stamp.sec;
      output->request_id = wire->request_id == nullptr ? "" : wire->request_id;
      output->evidence_id = wire->evidence_id == nullptr ? "" : wire->evidence_id;
      output->persisted = wire->persisted;
      output->reason = wire->reason == nullptr ? "" : wire->reason;
      output->analysis_verdict =
          wire->analysis_verdict == nullptr ? "" : wire->analysis_verdict;
    }
    checked(dds_return_loan(result_reader_, samples, count), "dds_return_loan");
    return valid;
  }

 private:
  dds_entity_t makeReader(
      const char* topic_name,
      const dds_topic_descriptor_t* descriptor) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, descriptor, topic_name, nullptr, nullptr),
        "dds_create_topic(reader)");
    auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic_name));
    return checked(
        dds_create_reader(participant_, topic, qos.get(), nullptr),
        "dds_create_reader");
  }

  dds_entity_t makeWriter(
      const char* topic_name,
      const dds_topic_descriptor_t* descriptor) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, descriptor, topic_name, nullptr, nullptr),
        "dds_create_topic(writer)");
    auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic_name));
    return checked(
        dds_create_writer(participant_, topic, qos.get(), nullptr),
        "dds_create_writer");
  }

  dds_entity_t participant_{0};
  dds_entity_t request_writer_{0};
  dds_entity_t result_reader_{0};
};

template <typename Fn>
void waitUntil(Fn&& fn, const char* message) {
  for (int attempt = 0; attempt < 100; ++attempt) {
    if (fn()) return;
    std::this_thread::sleep_for(10ms);
  }
  throw std::runtime_error(message);
}

void testBridgeRoundTrip() {
  constexpr int kDomain = 98;
  BridgePeer peer(kDomain);
  void* bridge = lingtu_inspection_evidence_bridge_create(kDomain);
  require(bridge != nullptr, "bridge must be created");

  LingtuInspectionEvidenceRequest request{};
  require(
      lingtu_inspection_evidence_bridge_take_request(bridge, &request) == 0,
      "take_request must be non-blocking when no sample exists");

  peer.writeRequest("request-a");
  waitUntil(
      [&]() {
        return lingtu_inspection_evidence_bridge_take_request(bridge, &request) == 1;
      },
      "timed out waiting for request via bridge");

  require(request.requested_at_s > 123.455 && request.requested_at_s < 123.457,
          "request header time must be exposed");
  require(std::string(request.request_id) == "request-a", "request id");
  require(std::string(request.run_id) == "run-a", "run id");
  require(std::string(request.route_id) == "route-a", "route id");
  require(request.route_revision == 9, "route revision");
  require(std::string(request.map_id) == "map-a", "map id");
  require(request.map_version == 4, "map version");
  require(request.point_index == 2, "point index");
  require(std::string(request.point_id) == "point-a", "point id");
  require(std::string(request.action) == "capture:overview", "action");
  require(request.deadline_s == 130.0, "deadline");

  LingtuInspectionEvidenceResult result{};
  result.result_at_s = 124.5;
  std::strcpy(result.request_id, "request-a");
  std::strcpy(result.evidence_id, "evidence-a");
  result.persisted = 1;
  std::strcpy(result.reason, "ok");
  std::strcpy(result.analysis_verdict, "captured");
  require(
      lingtu_inspection_evidence_bridge_write_result(bridge, &result) == 0,
      "write_result must succeed");

  ResultSnapshot wire{};
  waitUntil([&]() { return peer.takeResult(&wire); }, "timed out waiting for result");
  require(wire.stamp_sec == 124, "result header seconds");
  require(wire.request_id == "request-a", "result request id");
  require(wire.evidence_id == "evidence-a", "result evidence id");
  require(wire.persisted, "result persisted");
  require(wire.reason == "ok", "result reason");
  require(wire.analysis_verdict == "captured", "result verdict");

  lingtu_inspection_evidence_bridge_destroy(bridge);
}

void testBridgeRejectsTruncatedResultFields() {
  constexpr int kDomain = 99;
  void* bridge = lingtu_inspection_evidence_bridge_create(kDomain);
  require(bridge != nullptr, "bridge must be created");

  LingtuInspectionEvidenceResult result{};
  result.result_at_s = 1.0;
  std::memset(result.request_id, 'x', sizeof(result.request_id));
  result.request_id[sizeof(result.request_id) - 1U] = 'x';
  require(
      lingtu_inspection_evidence_bridge_write_result(bridge, &result) == -1,
      "unterminated fixed buffer must be rejected");
  const char* error = lingtu_inspection_evidence_bridge_last_error(bridge);
  require(error != nullptr && std::string(error).find("request_id") != std::string::npos,
          "last error must identify rejected field");

  lingtu_inspection_evidence_bridge_destroy(bridge);
}

void testBridgeRejectsOverlongRequestFields() {
  constexpr int kDomain = 100;
  BridgePeer peer(kDomain);
  void* bridge = lingtu_inspection_evidence_bridge_create(kDomain);
  require(bridge != nullptr, "bridge must be created");

  peer.writeRequest(std::string(LINGTU_INSPECTION_EVIDENCE_ID_CAP, 'r'));
  LingtuInspectionEvidenceRequest request{};
  waitUntil(
      [&]() {
        return lingtu_inspection_evidence_bridge_take_request(bridge, &request) == -1;
      },
      "timed out waiting for overlong request rejection");
  const char* error = lingtu_inspection_evidence_bridge_last_error(bridge);
  require(error != nullptr && std::string(error).find("exceeds") != std::string::npos,
          "last error must report fixed buffer overflow");

  lingtu_inspection_evidence_bridge_destroy(bridge);
}

}  // namespace

int main() {
  try {
    testBridgeRoundTrip();
    testBridgeRejectsTruncatedResultFields();
    testBridgeRejectsOverlongRequestFields();
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "test_inspection_evidence_bridge: FAIL: %s\n", exc.what());
    return 1;
  }
}
