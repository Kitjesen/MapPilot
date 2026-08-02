#include <chrono>
#include <cmath>
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
namespace inspection = lingtu::nav::inspection;

void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

dds_entity_t checked(dds_return_t value, const char *operation) {
  if (value < 0) {
    throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

template <typename Wire>
bool takeOne(dds_entity_t reader, Wire *output, const dds_topic_descriptor_t &descriptor) {
  void *samples[1]{};
  dds_sample_info_t infos[1]{};
  samples[0] = dds_alloc(sizeof(Wire));
  std::memset(samples[0], 0, sizeof(Wire));
  const dds_return_t count = dds_take(reader, samples, infos, 1, 1);
  checked(count, "dds_take");
  if (count == 0) {
    dds_sample_free(samples[0], &descriptor, DDS_FREE_ALL);
    return false;
  }
  const bool valid = infos[0].valid_data;
  if (valid) {
    *output = *static_cast<Wire *>(samples[0]);
  }
  dds_free(samples[0]);
  return valid;
}

class EvidencePeer {
 public:
  explicit EvidencePeer(int domain_id) {
    participant_ =
        checked(dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
                "dds_create_participant(evidence_peer)");
    request_reader_ = makeReader(lingtu::message::kNavInspectionEvidenceRequest.dds_topic.data(),
                                 &lingtu_dds_InspectionEvidenceRequest_desc);
    status_reader_ = makeReader(lingtu::message::kNavInspectionStatus.dds_topic.data(),
                                &lingtu_dds_InspectionStatus_desc);
    result_writer_ = makeWriter(lingtu::message::kNavInspectionEvidenceResult.dds_topic.data(),
                                &lingtu_dds_InspectionEvidenceResult_desc);
  }

  ~EvidencePeer() {
    if (participant_ > 0)
      dds_delete(participant_);
  }

  bool takeRequest(lingtu_dds_InspectionEvidenceRequest *output) {
    return takeOne(request_reader_, output, lingtu_dds_InspectionEvidenceRequest_desc);
  }

  bool takeStatus(lingtu_dds_InspectionStatus *output) {
    return takeOne(status_reader_, output, lingtu_dds_InspectionStatus_desc);
  }

  void writeResult(const std::string &request_id, const std::string &evidence_id, bool persisted,
                   const std::string &reason, const std::string &analysis_verdict) {
    lingtu_dds_InspectionEvidenceResult msg{};
    msg.request_id = const_cast<char *>(request_id.c_str());
    msg.evidence_id = const_cast<char *>(evidence_id.c_str());
    msg.persisted = persisted;
    msg.reason = const_cast<char *>(reason.c_str());
    msg.analysis_verdict = const_cast<char *>(analysis_verdict.c_str());
    checked(dds_write(result_writer_, &msg), "dds_write(evidence_result)");
  }

 private:
  dds_entity_t makeReader(const char *topic_name, const dds_topic_descriptor_t *descriptor) {
    const dds_entity_t topic =
        checked(dds_create_topic(participant_, descriptor, topic_name, nullptr, nullptr),
                "dds_create_topic(evidence_reader)");
    auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic_name));
    return checked(dds_create_reader(participant_, topic, qos.get(), nullptr),
                   "dds_create_reader(evidence)");
  }

  dds_entity_t makeWriter(const char *topic_name, const dds_topic_descriptor_t *descriptor) {
    const dds_entity_t topic =
        checked(dds_create_topic(participant_, descriptor, topic_name, nullptr, nullptr),
                "dds_create_topic(evidence_writer)");
    auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic_name));
    return checked(dds_create_writer(participant_, topic, qos.get(), nullptr),
                   "dds_create_writer(evidence)");
  }

  dds_entity_t participant_{0};
  dds_entity_t request_reader_{0};
  dds_entity_t status_reader_{0};
  dds_entity_t result_writer_{0};
};

inspection::Executor makeActionPendingExecutor() {
  inspection::Route route;
  route.id = "route-1";
  route.name = "Bin route";
  route.map_id = "factory";
  route.map_version = 7;
  route.revision = 3;
  route.loop_count = 1;
  inspection::Point point;
  point.id = "bin-a";
  point.action = "capture:bin_full";
  point.dwell_s = 0.0;
  route.points.push_back(point);

  inspection::Executor executor;
  std::string error;
  require(executor.Start(route, "run-1", "factory", 7, 0.0, &error), "inspection route must start");
  require(executor.OnPlanningStarted(0.1), "planning start");
  require(executor.OnPlanReady(0.2), "plan ready");
  executor.OnGoalReached(1.0);
  require(executor.OnArrivalSample({1.10, 0.0, 0.0}, 1.10), "sample 1");
  require(executor.OnArrivalSample({1.35, 0.0, 0.0}, 1.35), "sample 2");
  require(executor.OnArrivalSample({1.61, 0.0, 0.0}, 1.61), "sample 3");
  executor.Tick(1.61);
  require(executor.status().state == inspection::RunState::kActionPending,
          "inspection action must be pending");
  return executor;
}

template <typename Take>
void waitForSample(Take &&take) {
  for (int attempt = 0; attempt < 100; ++attempt) {
    if (take())
      return;
    std::this_thread::sleep_for(10ms);
  }
  throw std::runtime_error("timed out waiting for DDS sample");
}

void testEvidenceQosIsReliableAndBounded() {
  for (const auto *topic : {
           lingtu::message::kNavInspectionEvidenceRequest.dds_topic.data(),
           lingtu::message::kNavInspectionEvidenceResult.dds_topic.data(),
       }) {
    require(lingtu::dds::qos_for_topic(topic) == lingtu::dds::QosProfile::InspectionEvidence,
            "evidence topic must use dedicated QoS");
    auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic));
    dds_reliability_kind_t reliability{};
    dds_duration_t max_blocking_time{};
    require(dds_qget_reliability(qos.get(), &reliability, &max_blocking_time),
            "evidence QoS must define reliability");
    require(reliability == DDS_RELIABILITY_RELIABLE, "evidence QoS must be reliable");
    dds_history_kind_t history{};
    int32_t depth = 0;
    require(dds_qget_history(qos.get(), &history, &depth), "evidence QoS must define history");
    require(history == DDS_HISTORY_KEEP_LAST && depth == 32,
            "evidence QoS must keep a bounded history");
    dds_duration_t deadline = 0;
    dds_duration_t lifespan = 0;
    require(dds_qget_deadline(qos.get(), &deadline) && deadline == DDS_SECS(5),
            "evidence QoS must have a 5 second deadline");
    require(dds_qget_lifespan(qos.get(), &lifespan) && lifespan == DDS_SECS(35),
            "evidence QoS must expire stale samples");
  }
}

void testRequestStatusAndMatchedResultHandshake() {
  constexpr int kDomain = 97;
  lingtu::nav::endpoint::DdsRuntime runtime(kDomain);
  require(!runtime.inspectionEvidenceWorkerMatched(),
          "evidence writer must report no worker before discovery");
  EvidencePeer peer(kDomain);
  waitForSample([&]() { return runtime.inspectionEvidenceWorkerMatched(); });
  auto executor = makeActionPendingExecutor();
  const auto request = executor.PendingAction();
  require(request.has_value(), "action request must be available");

  lingtu_dds_InspectionEvidenceRequest request_wire{};
  for (int attempt = 0; attempt < 100; ++attempt) {
    require(runtime.writeInspectionEvidenceRequest(*request, executor.route()->map_id,
                                                   executor.route()->map_version,
                                                   executor.status().deadline_s),
            "evidence request write must succeed");
    std::this_thread::sleep_for(10ms);
    if (peer.takeRequest(&request_wire))
      break;
  }
  require(request_wire.request_id != nullptr, "evidence request must arrive");
  require(std::string(request_wire.request_id) == request->request_id, "request id");
  require(std::string(request_wire.run_id) == "run-1", "run id");
  require(std::string(request_wire.route_id) == "route-1", "route id");
  require(request_wire.revision == 3, "route revision");
  require(std::string(request_wire.map_id) == "factory", "map id");
  require(request_wire.map_version == 7, "map version");
  require(request_wire.point_index == 0, "point index");
  require(std::string(request_wire.point_id) == "bin-a", "point id");
  require(std::string(request_wire.action) == "capture:bin_full", "action");
  require(request_wire.deadline_s > 1.61, "deadline");
  dds_free(request_wire.request_id);
  dds_free(request_wire.run_id);
  dds_free(request_wire.route_id);
  dds_free(request_wire.map_id);
  dds_free(request_wire.point_id);
  dds_free(request_wire.action);

  require(executor.OnActionStarted(request->request_id, 1.70),
          "successful request publication must start the action timeout");
  runtime.writeInspectionStatus(executor.status());
  lingtu_dds_InspectionStatus status_wire{};
  waitForSample([&]() { return peer.takeStatus(&status_wire); });
  require(std::string(status_wire.action) == "capture:bin_full", "status action");
  require(std::string(status_wire.action_request_id) == request->request_id,
          "status action request id");
  require(status_wire.evidence_id != nullptr, "status evidence id field");
  require(status_wire.phase_started_at == 1.70, "status phase start");
  require(status_wire.stable_since > 0.0, "status stable since");
  require(status_wire.deadline > status_wire.phase_started_at, "status deadline");
  dds_free(status_wire.run_id);
  dds_free(status_wire.route_id);
  dds_free(status_wire.point_id);
  dds_free(status_wire.action);
  dds_free(status_wire.action_request_id);
  dds_free(status_wire.evidence_id);
  dds_free(status_wire.reason);

  peer.writeResult("late-request", "evidence-late", true, "", "bin_full");
  std::this_thread::sleep_for(20ms);
  runtime.drainInspectionEvidenceResults([&](const lingtu_dds_InspectionEvidenceResult &result) {
    require(!lingtu::nav::endpoint::applyInspectionEvidenceResult(executor, result, 1.80),
            "late result must not advance inspection");
  });
  require(executor.status().state == inspection::RunState::kActionPending,
          "late result must leave action pending");

  peer.writeResult(request->request_id, "evidence-1", true, "", "bin_full");
  std::this_thread::sleep_for(20ms);
  bool applied = false;
  runtime.drainInspectionEvidenceResults([&](const lingtu_dds_InspectionEvidenceResult &result) {
    applied =
        lingtu::nav::endpoint::applyInspectionEvidenceResult(executor, result, 1.90) || applied;
  });
  require(applied, "matching persisted evidence result must advance inspection");
  require(executor.status().state == inspection::RunState::kSucceeded,
          "matching evidence must complete the single-point route");
  require(executor.status().evidence_id == "evidence-1", "evidence id retained");

  peer.writeResult(request->request_id, "evidence-duplicate", true, "", "bin_full");
  std::this_thread::sleep_for(20ms);
  runtime.drainInspectionEvidenceResults([&](const lingtu_dds_InspectionEvidenceResult &result) {
    require(!lingtu::nav::endpoint::applyInspectionEvidenceResult(executor, result, 2.0),
            "duplicate result must be ignored");
  });
  require(executor.status().evidence_id == "evidence-1", "duplicate must not replace evidence");
}

}  // namespace

int main() {
  try {
    testEvidenceQosIsReliableAndBounded();
    testRequestStatusAndMatchedResultHandshake();
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_inspection_evidence_dds: FAIL: %s\n", exc.what());
    return 1;
  }
}
