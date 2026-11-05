#include "lingtu/maps/mapd/save.hpp"

#include <algorithm>
#include <limits>
#include <utility>

#include "lingtu/maps/io/json.hpp"
#include "lingtu/maps/service.hpp"

namespace lingtu::maps::mapd {
namespace {

std::string Failure(const std::string &action, const std::string &reason,
                    const std::string &message) {
  return "{\"action\":\"" + JsonEscape(action) + "\",\"accepted\":false,\"reason_code\":\"" +
         JsonEscape(reason) + "\",\"message\":\"" + JsonEscape(message) + "\"}";
}

std::string PublicActionResponse(std::string json, const std::string &action) {
  const std::string prefix = "\"action\":\"";
  const auto begin = json.find(prefix);
  if (begin != std::string::npos) {
    const auto value_begin = begin + prefix.size();
    const auto value_end = json.find('"', value_begin);
    if (value_end != std::string::npos) {
      json.replace(value_begin, value_end - value_begin, JsonEscape(action));
    }
  }
  return json;
}

}  // namespace

SaveCoordinator::SaveCoordinator(MapsServiceCore &service, SlamSnapshotExchange &exchange,
                                 SaveCoordinatorConfig config)
    : service_(service), exchange_(exchange), config_(std::move(config)) {}

std::string SaveCoordinator::SaveMapJson(const std::string &request_id, const std::string &map_id,
                                         bool auto_process) {
  if (config_.product != "map" || config_.product_session_id.empty()) {
    return Failure("save_map", "map_product_required",
                   "SaveMap requires the committed map Product and its Product session");
  }
  SaveMapRequest request = config_.request_defaults;
  request.request_id = request_id;
  request.map_id = map_id;
  request.product_session_id = config_.product_session_id;
  request.auto_process = auto_process;
  return RequestSnapshot(service_.BeginSaveMapJson(request), "save_map");
}

std::string SaveCoordinator::RetrySaveMapJson(const std::string &job_id) {
  return RequestSnapshot(service_.RetrySaveMapJson(job_id), "retry_save_map");
}

std::string SaveCoordinator::CancelSaveMapJson(const std::string &job_id) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto current = std::find_if(pending_.begin(), pending_.end(),
        [&](const auto &entry) { return entry.second.job_id == job_id; });
    if (current != pending_.end()) pending_.erase(current);
  }
  return PublicActionResponse(service_.CancelSaveMapJson(job_id), "cancel_save_map");
}

std::string SaveCoordinator::RequestSnapshot(std::string response,
                                              const std::string &public_action) {
  if (JsonObjectBoolAtPath(response, {"accepted"}) != true) {
    return PublicActionResponse(std::move(response), public_action);
  }

  const auto job_id = JsonObjectStringAtPath(response, {"status", "job_id"});
  const auto map_id = JsonObjectStringAtPath(response, {"status", "map_id"});
  const auto capture_dir = JsonObjectStringAtPath(response, {"status", "capture_dir"});
  const auto state = JsonObjectStringAtPath(response, {"status", "state"});
  if (!job_id.has_value() || job_id->empty() || !capture_dir.has_value() || capture_dir->empty()) {
    if (job_id.has_value() && !job_id->empty()) {
      static_cast<void>(service_.RejectSaveMapSnapshotJson(
          *job_id, "save_job_invalid", "native SaveMap did not return its capture location"));
    }
    return Failure(public_action, "save_job_invalid",
                   "native SaveMap did not return its capture location");
  }
  if (state.has_value() && *state != "WAITING_SNAPSHOT") {
    return PublicActionResponse(std::move(response), public_action);
  }
  if (config_.product != "map" || config_.product_session_id.empty()) {
    return PublicActionResponse(service_.RejectSaveMapSnapshotJson(
        *job_id, "map_product_required",
        "Capturing a new snapshot requires the committed map Product and its Product session"),
        public_action);
  }
  if (JsonObjectStringAtPath(response, {"status", "product_session_id"}) !=
      config_.product_session_id) {
    return PublicActionResponse(service_.RejectSaveMapSnapshotJson(
        *job_id, "snapshot_session_expired",
        "The original mapping session has ended; save the map again"), public_action);
  }
  if (!map_id.has_value() || map_id->empty()) {
    static_cast<void>(service_.RejectSaveMapSnapshotJson(
        *job_id, "save_job_invalid", "native SaveMap did not return its target map"));
    return Failure(public_action, "save_job_invalid",
                   "native SaveMap did not return its target map");
  }

  SlamSnapshotRequest snapshot_request;
  snapshot_request.map_id = *map_id;
  snapshot_request.product_session_id = config_.product_session_id;
  snapshot_request.output_path = std::filesystem::path(*capture_dir) / "map.pcd";
  snapshot_request.save_patches = config_.save_patches;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (std::any_of(pending_.begin(), pending_.end(),
        [&](const auto &entry) { return entry.second.job_id == *job_id; })) {
      return PublicActionResponse(std::move(response), public_action);
    }
    snapshot_request.request_id = *job_id + "-capture-" + std::to_string(++next_capture_attempt_);
    if (!exchange_.Publish(snapshot_request)) {
      static_cast<void>(service_.RejectSaveMapSnapshotJson(
          *job_id, "slam_snapshot_request_failed", "failed to publish the SLAM snapshot request"));
      return Failure(public_action, "slam_snapshot_request_failed",
                     "failed to publish the SLAM snapshot request");
    }
    pending_.emplace(snapshot_request.request_id,
                     Pending{*job_id, *map_id, snapshot_request.output_path});
  }
  return PublicActionResponse(std::move(response), public_action);
}

void SaveCoordinator::Poll() {
  for (auto &ack : exchange_.TakeAcks()) {
    Pending pending;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto found = pending_.find(ack.request_id);
      if (found == pending_.end()) {
        continue;
      }
      pending = std::move(found->second);
      pending_.erase(found);
    }
    if (!ack.success) {
      const std::string &message = ack.message.empty() ? ack.health_message : ack.message;
      static_cast<void>(service_.RejectSaveMapSnapshotJson(
          pending.job_id, "slam_snapshot_failed",
          message.empty() ? "SLAM rejected the map snapshot request" : message));
      continue;
    }
    if (ack.map_id != pending.map_id || ack.output_path != pending.output_path ||
        ack.product_session_id != config_.product_session_id) {
      static_cast<void>(
          service_.RejectSaveMapSnapshotJson(pending.job_id, "snapshot_ack_identity_mismatch",
                                             "SLAM snapshot acknowledgement does not match the "
                                             "requested map, path, or Product session"));
      continue;
    }

    MapSnapshot snapshot;
    snapshot.snapshot_id = ack.request_id;
    snapshot.source_dir = pending.output_path.parent_path();
    snapshot.frame_id = ack.frame_id;
    if (ack.captured_at_ns <=
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
      snapshot.captured_at_ns = static_cast<std::int64_t>(ack.captured_at_ns);
    }
    snapshot.first_sequence = ack.observation_sequence;
    snapshot.last_sequence = ack.observation_sequence;
    snapshot.slam_boot_id = ack.runtime_instance_id;
    snapshot.product_session_id = ack.product_session_id;
    snapshot.reset_epoch = ack.reset_epoch;
    snapshot.observation_sequence = ack.observation_sequence;
    snapshot.source_point_count = ack.point_count;
    snapshot.slam_healthy = ack.healthy;
    snapshot.health_message = ack.health_message.empty() ? ack.message : ack.health_message;
    const auto provided = service_.ProvideSaveMapSnapshotJson(pending.job_id, snapshot);
    if (JsonObjectBoolAtPath(provided, {"accepted"}) != true &&
        JsonObjectStringAtPath(provided, {"status", "state"}) == "WAITING_SNAPSHOT") {
      const auto reason = JsonObjectStringAtPath(provided, {"reason_code"});
      const auto message = JsonObjectStringAtPath(provided, {"status", "message"});
      static_cast<void>(service_.RejectSaveMapSnapshotJson(
          pending.job_id, reason.value_or("snapshot_rejected"),
          message.value_or("SLAM snapshot was rejected")));
    }
  }
}

}  // namespace lingtu::maps::mapd
