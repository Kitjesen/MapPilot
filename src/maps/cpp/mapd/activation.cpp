#include "lingtu/maps/mapd/activation.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

#include "lingtu/maps/lock.hpp"

namespace lingtu::maps::mapd {
namespace {

void AppendU32(std::string *output, std::uint32_t value) {
  for (int shift = 24; shift >= 0; shift -= 8) {
    output->push_back(static_cast<char>((value >> shift) & 0xffU));
  }
}

void AppendI64(std::string *output, std::int64_t value) {
  const auto encoded = static_cast<std::uint64_t>(value);
  for (int shift = 56; shift >= 0; shift -= 8) {
    output->push_back(static_cast<char>((encoded >> shift) & 0xffU));
  }
}

void AppendString(std::string *output, const std::string &value) {
  if (value.size() > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("activation token field is too large");
  }
  AppendU32(output, static_cast<std::uint32_t>(value.size()));
  output->append(value);
}

void AppendIdentity(std::string *output, const MapIdentity &identity) {
  output->push_back(identity.present ? '\x01' : '\x00');
  if (!identity.present) {
    return;
  }
  AppendString(output, identity.map_id);
  AppendI64(output, identity.content_epoch);
  AppendString(output, identity.frame_id);
  if (identity.artifacts.size() > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("activation token has too many artifacts");
  }
  AppendU32(output, static_cast<std::uint32_t>(identity.artifacts.size()));
  for (const auto &artifact : identity.artifacts) {
    AppendString(output, artifact.type);
    AppendString(output, artifact.uri);
  }
}

std::string Base64UrlEncode(const std::string &input) {
  constexpr char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
  std::string output;
  output.reserve((input.size() * 4U + 2U) / 3U);
  std::uint32_t buffer = 0U;
  int bits = 0;
  for (const unsigned char byte : input) {
    buffer = (buffer << 8U) | byte;
    bits += 8;
    while (bits >= 6) {
      bits -= 6;
      output.push_back(kAlphabet[(buffer >> bits) & 0x3fU]);
    }
  }
  if (bits > 0) {
    output.push_back(kAlphabet[(buffer << (6 - bits)) & 0x3fU]);
  }
  return output;
}

int Base64UrlValue(char value) {
  if (value >= 'A' && value <= 'Z')
    return value - 'A';
  if (value >= 'a' && value <= 'z')
    return value - 'a' + 26;
  if (value >= '0' && value <= '9')
    return value - '0' + 52;
  if (value == '-')
    return 62;
  if (value == '_')
    return 63;
  return -1;
}

std::string Base64UrlDecode(const std::string &input) {
  std::string output;
  output.reserve(input.size() * 3U / 4U);
  std::uint32_t buffer = 0U;
  int bits = 0;
  for (const char character : input) {
    const int value = Base64UrlValue(character);
    if (value < 0) {
      throw std::invalid_argument("activation token payload is not base64url");
    }
    buffer = (buffer << 6U) | static_cast<std::uint32_t>(value);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      output.push_back(static_cast<char>((buffer >> bits) & 0xffU));
    }
  }
  if (bits > 0 && (buffer & ((1U << bits) - 1U)) != 0U) {
    throw std::invalid_argument("activation token payload has non-zero padding");
  }
  return output;
}

std::uint32_t ReadU32(const std::string &input, std::size_t *offset) {
  if (*offset > input.size() || input.size() - *offset < 4U) {
    throw std::invalid_argument("activation token payload is truncated");
  }
  std::uint32_t value = 0U;
  for (int index = 0; index < 4; ++index) {
    value = (value << 8U) | static_cast<unsigned char>(input[(*offset)++]);
  }
  return value;
}

std::int64_t ReadI64(const std::string &input, std::size_t *offset) {
  if (*offset > input.size() || input.size() - *offset < 8U) {
    throw std::invalid_argument("activation token payload is truncated");
  }
  std::uint64_t value = 0U;
  for (int index = 0; index < 8; ++index) {
    value = (value << 8U) | static_cast<unsigned char>(input[(*offset)++]);
  }
  return static_cast<std::int64_t>(value);
}

std::string ReadString(const std::string &input, std::size_t *offset) {
  constexpr std::uint32_t kMaxFieldBytes = 64U * 1024U;
  const std::uint32_t length = ReadU32(input, offset);
  if (length > kMaxFieldBytes || *offset > input.size() || input.size() - *offset < length) {
    throw std::invalid_argument("activation token string is invalid");
  }
  std::string value = input.substr(*offset, length);
  *offset += length;
  return value;
}

MapIdentity ReadIdentity(const std::string &input, std::size_t *offset) {
  if (*offset >= input.size()) {
    throw std::invalid_argument("activation token payload is truncated");
  }
  MapIdentity identity;
  const unsigned char present = static_cast<unsigned char>(input[(*offset)++]);
  if (present > 1U) {
    throw std::invalid_argument("activation token presence flag is invalid");
  }
  identity.present = present == 1U;
  if (!identity.present) {
    return identity;
  }
  identity.map_id = ReadString(input, offset);
  identity.content_epoch = ReadI64(input, offset);
  identity.frame_id = ReadString(input, offset);
  const std::uint32_t count = ReadU32(input, offset);
  if (count > 32U) {
    throw std::invalid_argument("activation token has too many artifacts");
  }
  identity.artifacts.reserve(count);
  for (std::uint32_t index = 0U; index < count; ++index) {
    identity.artifacts.push_back({ReadString(input, offset), ReadString(input, offset)});
  }
  return identity;
}

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

const char *IdentityMismatchField(const MapIdentity &actual,
                                  const MapIdentity &requested) noexcept {
  if (actual.present != requested.present)
    return "present";
  if (actual.map_id != requested.map_id)
    return "map_id";
  if (actual.content_epoch != requested.content_epoch)
    return "content_epoch";
  if (actual.frame_id != requested.frame_id)
    return "frame_id";
  if (actual.artifacts != requested.artifacts)
    return "artifacts";
  return "unknown";
}

}  // namespace

ActivationCoordinator::ActivationCoordinator(MapStore &store) : store_(store) {
  const std::string active_map_id = store_.ActiveMapId();
  if (!active_map_id.empty()) {
    staged_identity_ = IdentityFor(active_map_id);
  }
}

bool ArtifactIdentity::operator==(const ArtifactIdentity &other) const noexcept {
  return type == other.type && uri == other.uri;
}

bool MapIdentity::operator==(const MapIdentity &other) const noexcept {
  return present == other.present && map_id == other.map_id && content_epoch == other.content_epoch &&
         frame_id == other.frame_id && artifacts == other.artifacts;
}

bool ActivationRequest::operator==(const ActivationRequest &other) const noexcept {
  return request_id == other.request_id && operation == other.operation && target == other.target &&
         previous == other.previous && caller == other.caller && reason == other.reason;
}

bool IsCanonicalIdentity(const MapIdentity &identity) noexcept {
  if (!identity.present) {
    return identity.map_id.empty() && identity.content_epoch == 0 && identity.frame_id.empty() &&
           identity.artifacts.empty();
  }
  if (identity.map_id.empty() || identity.content_epoch <= 0 || identity.frame_id.empty() ||
      identity.artifacts.empty()) {
    return false;
  }
  return std::all_of(
      identity.artifacts.begin(), identity.artifacts.end(), [](const ArtifactIdentity &artifact) {
        return !artifact.type.empty() && !artifact.uri.empty();
      });
}

std::string EncodeActivationToken(const MapIdentity &target, const MapIdentity &previous) {
  if (!target.present || !IsCanonicalIdentity(target)) {
    throw std::invalid_argument("target_identity_invalid");
  }
  if (!IsCanonicalIdentity(previous)) {
    throw std::invalid_argument("previous_identity_invalid");
  }
  std::string payload{"LTMAP3", 6U};
  AppendIdentity(&payload, target);
  AppendIdentity(&payload, previous);
  return "v3." + Base64UrlEncode(payload);
}

std::pair<MapIdentity, MapIdentity> DecodeActivationToken(const std::string &token) {
  if (token.rfind("v3.", 0U) != 0U) {
    throw std::invalid_argument("activation token version is unsupported");
  }
  const std::string payload = Base64UrlDecode(token.substr(3U));
  if (payload.size() < 6U || payload.compare(0U, 6U, "LTMAP3") != 0) {
    throw std::invalid_argument("activation token magic is invalid");
  }
  std::size_t offset = 6U;
  MapIdentity target = ReadIdentity(payload, &offset);
  MapIdentity previous = ReadIdentity(payload, &offset);
  if (offset != payload.size()) {
    throw std::invalid_argument("activation token payload has trailing data");
  }
  if (!target.present || !IsCanonicalIdentity(target)) {
    throw std::invalid_argument("target_identity_invalid");
  }
  if (!IsCanonicalIdentity(previous)) {
    throw std::invalid_argument("previous_identity_invalid");
  }
  return {std::move(target), std::move(previous)};
}

MapIdentity ActivationCoordinator::IdentityFor(const std::string &map_id) const {
  const auto record = store_.GetMapRecord(map_id);
  if (!record.has_value()) {
    return {};
  }
  MapIdentity identity;
  identity.present = true;
  identity.map_id = record->map_id;
  identity.content_epoch = record->content_epoch;
  identity.frame_id = record->scope.frame_id;
  identity.artifacts.reserve(record->artifacts.size());
  for (const auto &artifact : record->artifacts) {
    identity.artifacts.push_back({ArtifactTypeName(artifact.type), artifact.uri});
  }
  std::sort(identity.artifacts.begin(), identity.artifacts.end(),
            [](const ArtifactIdentity &lhs, const ArtifactIdentity &rhs) {
              if (lhs.type != rhs.type) {
                return lhs.type < rhs.type;
              }
              return lhs.uri < rhs.uri;
            });
  return identity;
}

MapIdentity ActivationCoordinator::ActiveIdentity() const {
  const std::string map_id = store_.ActiveMapId();
  if (map_id.empty()) {
    staged_identity_ = {};
    return {};
  }
  if (!staged_identity_.present || staged_identity_.map_id != map_id) {
    staged_identity_ = IdentityFor(map_id);
  }
  return staged_identity_;
}

ActivationRequest ActivationCoordinator::PrepareStage(const std::string &map_id) const {
  ActivationRequest request;
  request.operation = ActivationOperation::kStage;
  request.target = IdentityFor(map_id);
  std::string active_state_error;
  if (!store_.ValidateActiveState(&active_state_error)) {
    throw std::invalid_argument("previous_identity_invalid:" + active_state_error);
  }
  request.previous = ActiveIdentity();
  if (!request.target.present || !IsCanonicalIdentity(request.target)) {
    throw std::invalid_argument("target_identity_invalid");
  }
  if (!IsCanonicalIdentity(request.previous)) {
    throw std::invalid_argument("previous_identity_invalid");
  }
  return request;
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
    return Reject(request, std::string("target_identity_mismatch:") +
                               IdentityMismatchField(actual_target, request.target));
  }
  const MapIdentity actual_previous = ActiveIdentity();
  if (actual_previous != request.previous) {
    return Reject(request, std::string("previous_identity_mismatch:") +
                               IdentityMismatchField(actual_previous, request.previous));
  }
  if (actual_previous == actual_target) {
    return {request.request_id, request.operation, true, "already_staged", false, actual_target,
            actual_previous,    actual_target,     {}};
  }
  const auto activation_check =
      store_.CheckMapActivationWhileLocked(request.target.map_id, *map_lock);
  if (!activation_check.ok) {
    std::string message = "artifact_gate_failed";
    for (const auto &blocker : activation_check.blockers) {
      message += ":" + blocker;
    }
    return Reject(request, std::move(message));
  }
  const std::string expected_active =
      request.previous.present ? request.previous.map_id : std::string{};
  const auto changed = store_.SetActiveMapWhileLocked(
      request.target.map_id, false, *map_lock, expected_active);
  if (!changed.ok) {
    return Reject(request, "stage_failed:" + changed.message);
  }
  staged_identity_ = actual_target;
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
  if (active_before == request.previous) {
    return {request.request_id, request.operation, true, "already_restored", false, request.target,
            request.previous,   active_before,     {}};
  }
  if (active_before != request.target) {
    return Reject(request, "stale_rollback");
  }
  if (!request.previous.present) {
    const auto cleared = store_.ClearActiveMap(request.target.map_id);
    if (!cleared.ok) {
      return Reject(request, "restore_failed:" + cleared.message);
    }
    staged_identity_ = {};
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
        store_.SetActiveMapWhileLocked(
            request.previous.map_id, true, *previous_lock, request.target.map_id);
    if (!restored.ok) {
      return Reject(request, "restore_failed:" + restored.message);
    }
    staged_identity_ = request.previous;
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
  const auto activation_check = store_.CheckMapActivation(request.target.map_id);
  if (!activation_check.ok) {
    std::string message = "artifact_gate_failed";
    for (const auto &blocker : activation_check.blockers) {
      message += ":" + blocker;
    }
    return Reject(request, std::move(message));
  }
  const MapIdentity verified = ActiveIdentity();
  if (verified != request.target) {
    return Reject(request, "active_identity_changed_during_verify");
  }
  return {request.request_id, request.operation, true,     "verified", false,
          verified,           request.previous,  verified, {}};
}

}  // namespace lingtu::maps::mapd
