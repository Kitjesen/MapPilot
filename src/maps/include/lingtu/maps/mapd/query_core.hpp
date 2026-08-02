#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include "lingtu/maps/model.hpp"
#include "lingtu/maps/store.hpp"

namespace lingtu::maps::mapd::query {

struct OpenArtifactResult {
  bool ok{false};
  std::string json;
  int fd{-1};
};

class MapQueryCore {
 public:
  explicit MapQueryCore(const MapStore& store) : store_(store) {}

  std::string PingJson() const;
  OpenArtifactResult OpenArtifact(const std::string& map_id, const std::string& capability) const;

 private:
  std::string FailureJson(
      const std::string& action,
      const std::string& message,
      const std::string& reason_code) const;
  std::string BundleJsonFromIdentity(
      const DeclaredArtifactIdentity& identity,
      const std::string& capability,
      std::uint64_t size_bytes) const;
  std::optional<DeclaredArtifactIdentity> DeclaredIdentityFor(
      const std::string& map_id,
      const std::string& capability,
      std::string* error) const;

  const MapStore& store_;
};

}  // namespace lingtu::maps::mapd::query
