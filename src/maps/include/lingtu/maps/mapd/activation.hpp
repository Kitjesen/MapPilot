#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "lingtu/maps/store.hpp"

namespace lingtu::maps::mapd {

enum class ActivationOperation {
  kStage = 0,
  kRestore = 1,
  kVerify = 2,
};

struct ArtifactIdentity {
  std::string type;
  std::string uri;

  bool operator==(const ArtifactIdentity &other) const noexcept;
  bool operator!=(const ArtifactIdentity &other) const noexcept { return !(*this == other); }
};

struct MapIdentity {
  bool present{false};
  std::string map_id;
  std::int64_t content_epoch{0};
  std::string frame_id;
  std::vector<ArtifactIdentity> artifacts;

  bool operator==(const MapIdentity &other) const noexcept;
  bool operator!=(const MapIdentity &other) const noexcept { return !(*this == other); }
};

struct ActivationRequest {
  std::string request_id;
  ActivationOperation operation{ActivationOperation::kVerify};
  MapIdentity target;
  MapIdentity previous;
  std::string caller;
  std::string reason;

  bool operator==(const ActivationRequest &other) const noexcept;
  bool operator!=(const ActivationRequest &other) const noexcept { return !(*this == other); }
};

struct ActivationResult {
  std::string request_id;
  ActivationOperation operation{ActivationOperation::kVerify};
  bool accepted{false};
  std::string message;
  bool changed{false};
  MapIdentity target;
  MapIdentity previous;
  MapIdentity active;
  std::string producer_boot_id;
};

class ActivationCoordinator final {
 public:
  explicit ActivationCoordinator(MapStore &store);

  MapIdentity IdentityFor(const std::string &map_id) const;
  MapIdentity ActiveIdentity() const;
  ActivationRequest PrepareStage(const std::string &map_id) const;
  ActivationResult Execute(const ActivationRequest &request);

 private:
  ActivationResult Stage(const ActivationRequest &request);
  ActivationResult Restore(const ActivationRequest &request);
  ActivationResult Verify(const ActivationRequest &request);
  ActivationResult Reject(const ActivationRequest &request, std::string message) const;

  MapStore &store_;
  mutable MapIdentity staged_identity_;
};

bool IsCanonicalIdentity(const MapIdentity &identity) noexcept;
std::string EncodeActivationToken(const MapIdentity &target, const MapIdentity &previous);
std::pair<MapIdentity, MapIdentity> DecodeActivationToken(const std::string &token);

}  // namespace lingtu::maps::mapd
