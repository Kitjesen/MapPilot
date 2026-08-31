#include "perception/inspection/native_bridge.h"

#include "message/cpp/qos.hpp"
#include "message/cpp/topics.hpp"

#include "dds/dds.h"
#include "messages.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

thread_local std::string g_last_error;

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

void fillHeader(lingtu_dds_Header& header, double stamp_s, const char* frame_id) {
  if (!std::isfinite(stamp_s) || stamp_s <= 0.0) {
    stamp_s = nowSeconds();
  }
  header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_s - static_cast<double>(header.stamp.sec)) * 1e9);
  header.frame_id = const_cast<char*>(frame_id);
}

double headerStampSeconds(const lingtu_dds_Header& header) {
  return static_cast<double>(header.stamp.sec) +
      static_cast<double>(header.stamp.nanosec) * 1e-9;
}

bool copyBounded(char* destination, std::size_t capacity, const char* value) {
  if (destination == nullptr || capacity == 0U) return false;
  const char* source = value == nullptr ? "" : value;
  const std::size_t length = std::strlen(source);
  if (length >= capacity) {
    return false;
  }
  std::memcpy(destination, source, length + 1U);
  return true;
}

std::string boundedString(const char* value, std::size_t capacity, const char* field) {
  if (value == nullptr) return "";
  const std::size_t length = strnlen(value, capacity);
  if (length == capacity) {
    throw std::runtime_error(std::string(field) + " exceeds fixed ABI buffer");
  }
  return std::string(value, length);
}

dds_entity_t checked(dds_return_t value, const char* operation) {
  if (value < 0) {
    throw std::runtime_error(
        std::string(operation) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

class EvidenceBridge {
 public:
  explicit EvidenceBridge(int32_t domain_id) {
    try {
      participant_ = checked(
          dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
          "dds_create_participant(inspection_evidence_bridge)");
      subscriber_ = checked(
          dds_create_subscriber(participant_, nullptr, nullptr),
          "dds_create_subscriber(inspection_evidence_bridge)");
      publisher_ = checked(
          dds_create_publisher(participant_, nullptr, nullptr),
          "dds_create_publisher(inspection_evidence_bridge)");
      request_reader_ = makeReader(
          lingtu::message::kNavInspectionEvidenceRequest.dds_topic.data(),
          &lingtu_dds_InspectionEvidenceRequest_desc,
          "inspection_evidence_request");
      result_writer_ = makeWriter(
          lingtu::message::kNavInspectionEvidenceResult.dds_topic.data(),
          &lingtu_dds_InspectionEvidenceResult_desc,
          "inspection_evidence_result");
    } catch (...) {
      if (participant_ > 0) {
        dds_delete(participant_);
        participant_ = 0;
      }
      throw;
    }
  }

  ~EvidenceBridge() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  int32_t takeRequest(LingtuInspectionEvidenceRequest* output) {
    if (output == nullptr) {
      setError("output request pointer is null");
      return -1;
    }
    void* samples[1]{};
    dds_sample_info_t infos[1]{};
    const dds_return_t count = dds_take(request_reader_, samples, infos, 1, 1);
    if (count < 0) {
      setError(std::string("dds_take(inspection_evidence_request): ") +
               dds_strretcode(-count));
      return -1;
    }
    if (count == 0) {
      return 0;
    }
    if (!infos[0].valid_data) {
      checked(dds_return_loan(request_reader_, samples, count), "dds_return_loan");
      return 0;
    }

    const auto* wire =
        static_cast<const lingtu_dds_InspectionEvidenceRequest*>(samples[0]);
    LingtuInspectionEvidenceRequest next{};
    const bool ok =
        copyBounded(next.request_id, sizeof(next.request_id), wire->request_id) &&
        copyBounded(next.run_id, sizeof(next.run_id), wire->run_id) &&
        copyBounded(next.route_id, sizeof(next.route_id), wire->route_id) &&
        copyBounded(next.map_id, sizeof(next.map_id), wire->map_id) &&
        copyBounded(next.point_id, sizeof(next.point_id), wire->point_id) &&
        copyBounded(next.action, sizeof(next.action), wire->action);
    next.requested_at_s = headerStampSeconds(wire->header);
    next.route_revision = wire->revision;
    next.map_content_epoch = wire->map_content_epoch;
    next.point_index = wire->point_index;
    next.deadline_s = wire->deadline_s;
    checked(dds_return_loan(request_reader_, samples, count), "dds_return_loan");
    if (!ok) {
      setError("inspection evidence request string exceeds fixed ABI buffer");
      return -1;
    }
    *output = next;
    clearError();
    return 1;
  }

  int32_t writeResult(const LingtuInspectionEvidenceResult* result) {
    if (result == nullptr) {
      setError("result pointer is null");
      return -1;
    }
    try {
      const std::string request_id =
          boundedString(result->request_id, sizeof(result->request_id), "request_id");
      const std::string evidence_id =
          boundedString(result->evidence_id, sizeof(result->evidence_id), "evidence_id");
      const std::string reason =
          boundedString(result->reason, sizeof(result->reason), "reason");
      const std::string verdict = boundedString(
          result->analysis_verdict,
          sizeof(result->analysis_verdict),
          "analysis_verdict");

      lingtu_dds_InspectionEvidenceResult wire{};
      fillHeader(wire.header, result->result_at_s, "map");
      wire.request_id = const_cast<char*>(request_id.c_str());
      wire.evidence_id = const_cast<char*>(evidence_id.c_str());
      wire.persisted = result->persisted != 0;
      wire.reason = const_cast<char*>(reason.c_str());
      wire.analysis_verdict = const_cast<char*>(verdict.c_str());
      const dds_return_t status = dds_write(result_writer_, &wire);
      if (status < 0) {
        setError(std::string("dds_write(inspection_evidence_result): ") +
                 dds_strretcode(-status));
        return -1;
      }
      clearError();
      return 0;
    } catch (const std::exception& exc) {
      setError(exc.what());
      return -1;
    }
  }

  const char* lastError() const noexcept { return last_error_.c_str(); }

 private:
  dds_entity_t makeReader(
      const char* topic_name,
      const dds_topic_descriptor_t* descriptor,
      const char* label) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, descriptor, topic_name, nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic_name));
    return checked(
        dds_create_reader(subscriber_, topic, qos.get(), nullptr),
        (std::string("dds_create_reader(") + label + ")").c_str());
  }

  dds_entity_t makeWriter(
      const char* topic_name,
      const dds_topic_descriptor_t* descriptor,
      const char* label) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, descriptor, topic_name, nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic_name));
    return checked(
        dds_create_writer(publisher_, topic, qos.get(), nullptr),
        (std::string("dds_create_writer(") + label + ")").c_str());
  }

  void clearError() { last_error_.clear(); }
  void setError(std::string value) {
    last_error_ = std::move(value);
    g_last_error = last_error_;
  }

  dds_entity_t participant_{0};
  dds_entity_t subscriber_{0};
  dds_entity_t publisher_{0};
  dds_entity_t request_reader_{0};
  dds_entity_t result_writer_{0};
  std::string last_error_;
};

EvidenceBridge* asBridge(void* handle) {
  return static_cast<EvidenceBridge*>(handle);
}

}  // namespace

extern "C" LINGTU_INSPECTION_EVIDENCE_BRIDGE_API void*
lingtu_inspection_evidence_bridge_create(int32_t domain_id) {
  try {
    g_last_error.clear();
    return new EvidenceBridge(domain_id);
  } catch (const std::exception& exc) {
    g_last_error = exc.what();
    return nullptr;
  }
}

extern "C" LINGTU_INSPECTION_EVIDENCE_BRIDGE_API void
lingtu_inspection_evidence_bridge_destroy(void* handle) {
  delete asBridge(handle);
}

extern "C" LINGTU_INSPECTION_EVIDENCE_BRIDGE_API int32_t
lingtu_inspection_evidence_bridge_take_request(
    void* handle,
    LingtuInspectionEvidenceRequest* output) {
  if (handle == nullptr) {
    g_last_error = "bridge handle is null";
    return -1;
  }
  try {
    return asBridge(handle)->takeRequest(output);
  } catch (const std::exception& exc) {
    g_last_error = exc.what();
    return -1;
  }
}

extern "C" LINGTU_INSPECTION_EVIDENCE_BRIDGE_API int32_t
lingtu_inspection_evidence_bridge_write_result(
    void* handle,
    const LingtuInspectionEvidenceResult* result) {
  if (handle == nullptr) {
    g_last_error = "bridge handle is null";
    return -1;
  }
  return asBridge(handle)->writeResult(result);
}

extern "C" LINGTU_INSPECTION_EVIDENCE_BRIDGE_API const char*
lingtu_inspection_evidence_bridge_last_error(void* handle) {
  if (handle == nullptr) {
    return g_last_error.c_str();
  }
  return asBridge(handle)->lastError();
}
