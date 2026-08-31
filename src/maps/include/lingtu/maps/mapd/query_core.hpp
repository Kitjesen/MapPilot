#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include "lingtu/maps/model.hpp"
#include "lingtu/maps/service.hpp"

namespace lingtu::maps::mapd {
class SaveCoordinator;
}

namespace lingtu::maps::mapd::query {

struct OpenArtifactResult {
  bool ok{false};
  std::string json;
  int fd{-1};
};

struct ServiceResult {
  bool ok{false};
  std::string json;
};

class MapQueryCore {
 public:
  explicit MapQueryCore(MapsServiceCore &service, mapd::SaveCoordinator *save_coordinator = nullptr)
      : service_(service), save_coordinator_(save_coordinator) {}

  std::string PingJson() const;
  ServiceResult ServiceJson(const std::string &request_json);
  OpenArtifactResult OpenArtifact(const std::string &map_id, const std::string &capability) const;

 private:
  std::string FailureJson(const std::string &action, const std::string &message,
                          const std::string &reason_code) const;
  std::string BundleJsonFromIdentity(const DeclaredArtifactIdentity &identity,
                                     const std::string &capability, std::uint64_t size_bytes) const;
  std::optional<DeclaredArtifactIdentity> DeclaredIdentityFor(const std::string &map_id,
                                                              const std::string &capability,
                                                              std::string *error) const;

  MapsServiceCore &service_;
  mapd::SaveCoordinator *save_coordinator_{nullptr};
};

}  // namespace lingtu::maps::mapd::query
