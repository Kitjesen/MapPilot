#include "lingtu/maps/mapd/activation.hpp"

#include <algorithm>
#include <exception>
#include <optional>
#include <utility>

#include "lingtu/maps/lock.hpp"

namespace lingtu::maps::mapd {
namespace {

std::string ArtifactTypeName(ArtifactType type) {
  switch (type) {
    case ArtifactType::kPointCloud:
      return "pointcloud";
    case ArtifactType::kOccupancy2D:
      return "occupancy2d";
    case ArtifactType::kOctomap3D:
      return "octomap3d";
    case ArtifactType::kEsdf:
      return "esdf";
    case ArtifactType::kTraversability:
      return "traversability";
    case ArtifactType::kSemantic:
      return "semantic";
  }
  return "unknown";
}

std::string VersionId(const MapRecord &record) {
  const auto found = record.metadata.find("version_id");
  if (found != record.metadata.end() && !found->second.empty()) {
    return found->second;
  }
  return record.map_id + ":v" + std::to_string(record.version);
}

std::string ContentDir(const MapRecord &record) {
  const auto found = record.metadata.find("content_dir");
  return found == record.metadata.end() ? std::string{} : found->second;
}

bool GateReady(MapStore &store, const std::string &map_id, std::string *message) {
  ArtifactValidationOptions options;
  options.require_runtime_planning_artifact = true;
  options.expected_frame_id = "map";
  const auto gate = store.ValidateArtifacts(map_id, options);
  if (gate.ok) {
    return true;
  }
  *message = "artifact_gate_failed";
  for (const auto &blocker : gate.blockers) {
    *message += ":" + blocker;
  }
  return false;
}

}  // namespace

bool ArtifactIdentity::operator==(const ArtifactIdentity &other) const noexcept {
  return type == other.type && uri == other.uri && sha256 == other.sha256;
}

bool MapIdentity::operator==(const MapIdentity &other) const noexcept {
  return present == other.present && map_id == other.map_id && version_id == other.version_id &&
         frame_id == other.frame_id && map_dir == other.map_dir && artifacts == other.artifacts;
}

bool ActivationRequest::operator==(const ActivationRequest &other) const noexcept {
  return request_id == other.request_id && operation == other.operation && target == other.target &&
         previous == other.previous && caller == other.caller && reason == other.reason;
}

const char *ActivationOperationName(ActivationOperation operation) noexcept {
  switch (operation) {
    case ActivationOperation::kStage:
      return "STAGE";
    case ActivationOperation::kRestore:
      return "RESTORE";
    case ActivationOperation::kVerify:
      return "VERIFY";
  }
  return "UNKNOWN";
}

bool IsCanonicalIdentity(const MapIdentity &identity) noexcept {
  if (!identity.present) {
    return identity.map_id.empty() && identity.version_id.empty() && identity.frame_id.empty() &&
           identity.map_dir.empty() && identity.artifacts.empty();
  }
  if (identity.map_id.empty() || identity.version_id.empty() || identity.frame_id.empty() ||
      identity.map_dir.empty() || identity.artifacts.empty()) {
    return false;
  }
  return std::all_of(
      identity.artifacts.begin(), identity.artifacts.end(), [](const ArtifactIdentity &artifact) {
        return !artifact.type.empty() && !artifact.uri.empty() && artifact.sha256.size() == 64U;
      });
}

MapIdentity ActivationCoordinator::IdentityFor(const std::string &map_id) const {
  const auto record = store_.GetMapRecord(map_id);
  if (!record.has_value()) {
    return {};
  }
  MapIdentity identity;
  identity.present = true;
  identity.map_id = record->map_id;
  identity.version_id = VersionId(*record);
  identity.frame_id = record->scope.frame_id;
  identity.map_dir = ContentDir(*record);
  identity.artifacts.reserve(record->artifacts.size());
  for (const auto &artifact : record->artifacts) {
    identity.artifacts.push_back({ArtifactTypeName(artifact.type), artifact.uri, artifact.sha256});
  }
  std::sort(identity.artifacts.begin(), identity.artifacts.end(),
            [](const ArtifactIdentity &lhs, const ArtifactIdentity &rhs) {
              if (lhs.type != rhs.type) {
                return lhs.type < rhs.type;
              }
              if (lhs.uri != rhs.uri) {
                return lhs.uri < rhs.uri;
              }
              return lhs.sha256 < rhs.sha256;
            });
  return identity;
}

MapIdentity ActivationCoordinator::ActiveIdentity() const {
  const std::string map_id = store_.ActiveMapId();
  return map_id.empty() ? MapIdentity{} : IdentityFor(map_id);
}

ActivationResult ActivationCoordinator::Reject(const ActivationRequest &request,
                                               std::string message) const {
  return {request.request_id, request.operation, false, std::move(message), false, request.target,
          request.previous,   ActiveIdentity(),  {}};
}

ActivationResult ActivationCoordinator::Execute(const ActivationRequest &request) {
  if (request.request_id.empty()) {
    return Reject(request, "invalid_request_id");
  }
  if (!IsCanonicalIdentity(request.target)) {
    return Reject(request, "invalid_target_identity");
  }
  if (request.target.present && !MapStore::IsValidMapId(request.target.map_id)) {
    return Reject(request, "invalid_target_map_id");
  }
  if (!IsCanonicalIdentity(request.previous)) {
    return Reject(request, "invalid_previous_identity");
  }
  if (request.previous.present && !MapStore::IsValidMapId(request.previous.map_id)) {
    return Reject(request, "invalid_previous_map_id");
  }
  try {
    switch (request.operation) {
      case ActivationOperation::kStage:
        return Stage(request);
      case ActivationOperation::kRestore:
        return Restore(request);
      case ActivationOperation::kVerify:
        return Verify(request);
    }
  } catch (const std::exception &exception) {
    return Reject(request, std::string("activation_error:") + exception.what());
  }
  return Reject(request, "invalid_operation");
}

ActivationResult ActivationCoordinator::Stage(const ActivationRequest &request) {
  if (!request.target.present) {
    return Reject(request, "target_identity_required");
  }
  auto map_lock =
      MapLock::TryAcquire(store_.RootDir(), request.target.map_id, "map-activation-stage");
  if (!map_lock.has_value()) {
    return Reject(request, "target_map_write_in_progress");
  }
  const MapIdentity actual_target = IdentityFor(request.target.map_id);
  if (actual_target != request.target) {
    return Reject(request, "target_identity_mismatch");
  }
  const MapIdentity actual_previous = ActiveIdentity();
  if (actual_previous != request.previous) {
    return Reject(request, "previous_identity_mismatch");
  }
  if (actual_previous == actual_target) {
    return {request.request_id, request.operation, true, "already_staged", false, actual_target,
            actual_previous,    actual_target,     {}};
  }
  const auto changed = store_.SetActiveMapWhileLocked(request.target.map_id, true, *map_lock);
  if (!changed.ok) {
    return Reject(request, "stage_failed:" + changed.message);
  }
  const MapIdentity active = ActiveIdentity();
  if (active != actual_target) {
    return Reject(request, "stage_postcondition_failed");
  }
  return {request.request_id, request.operation, true,   "staged", true,
          actual_target,      actual_previous,   active, {}};
}

ActivationResult ActivationCoordinator::Restore(const ActivationRequest &request) {
  if (!request.target.present) {
    return Reject(request, "staged_target_identity_required");
  }
  const MapIdentity active_before = ActiveIdentity();
  if (active_before != request.target) {
    return Reject(request, "stale_rollback");
  }
  if (request.previous == request.target) {
    return {request.request_id, request.operation, true, "already_restored", false, request.target,
            request.previous,   active_before,     {}};
  }
  if (!request.previous.present) {
    store_.ClearActiveMap();
  } else {
    auto previous_lock =
        MapLock::TryAcquire(store_.RootDir(), request.previous.map_id, "map-activation-restore");
    if (!previous_lock.has_value()) {
      return Reject(request, "previous_map_write_in_progress");
    }
    const MapIdentity actual_previous = IdentityFor(request.previous.map_id);
    if (actual_previous != request.previous) {
      return Reject(request, "previous_identity_mismatch");
    }
    const auto restored =
        store_.SetActiveMapWhileLocked(request.previous.map_id, true, *previous_lock);
    if (!restored.ok) {
      return Reject(request, "restore_failed:" + restored.message);
    }
  }
  const MapIdentity active = ActiveIdentity();
  if (active != request.previous) {
    return Reject(request, "restore_postcondition_failed");
  }
  return {request.request_id, request.operation, true,   "restored", true,
          request.target,     request.previous,  active, {}};
}

ActivationResult ActivationCoordinator::Verify(const ActivationRequest &request) {
  if (!request.target.present) {
    return Reject(request, "target_identity_required");
  }
  const MapIdentity active = ActiveIdentity();
  if (active != request.target) {
    return Reject(request, "active_identity_mismatch");
  }
  std::string gate_message;
  if (!GateReady(store_, request.target.map_id, &gate_message)) {
    return Reject(request, std::move(gate_message));
  }
  const MapIdentity verified = ActiveIdentity();
  if (verified != request.target) {
    return Reject(request, "active_identity_changed_during_verify");
  }
  return {request.request_id, request.operation, true,     "verified", false,
          verified,           request.previous,  verified, {}};
}

}  // namespace lingtu::maps::mapd
