#include "lingtu/maps/mapd/query_core.hpp"

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "lingtu/maps/lock.hpp"
#include "lingtu/maps/mapd/service_dispatch.hpp"

#if defined(__linux__)
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace lingtu::maps::mapd::query {
namespace {

std::string JsonEscape(const std::string &value) {
  std::ostringstream out;
  for (const unsigned char ch : value) {
    switch (ch) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (ch < 0x20U) {
          constexpr char kHex[] = "0123456789abcdef";
          out << "\\u00" << kHex[(ch >> 4U) & 0x0fU] << kHex[ch & 0x0fU];
        } else {
          out << static_cast<char>(ch);
        }
        break;
    }
  }
  return out.str();
}

std::string JsonString(const std::string &value) {
  return "\"" + JsonEscape(value) + "\"";
}

std::string ArtifactTypeName(ArtifactType type) {
  switch (type) {
    case ArtifactType::kPointCloud:
      return "POINTCLOUD";
    case ArtifactType::kOccupancy2D:
      return "OCCUPANCY_2D";
    case ArtifactType::kOctomap3D:
      return "OCTOMAP_3D";
    case ArtifactType::kEsdf:
      return "ESDF";
    case ArtifactType::kTraversability:
      return "TRAVERSABILITY";
    case ArtifactType::kSemantic:
      return "SEMANTIC";
  }
  return "UNKNOWN";
}

}  // namespace

std::string MapQueryCore::PingJson() const {
  return "{\"action\":\"ping\",\"success\":true,\"schema_version\":\"mapd.query.v2\"}";
}

ServiceResult MapQueryCore::ServiceJson(const std::string &request_json) {
  auto result = DispatchServiceJson(service_, save_coordinator_, request_json);
  return {result.ok, std::move(result.json)};
}

std::string MapQueryCore::FailureJson(const std::string &action, const std::string &message,
                                      const std::string &reason_code) const {
  return "{"
         "\"action\":" +
         JsonString(action) + "," + "\"success\":false," +
         "\"reason_code\":" + JsonString(reason_code) + "," + "\"message\":" + JsonString(message) +
         "}";
}

std::optional<DeclaredArtifactIdentity>
MapQueryCore::DeclaredIdentityFor(const std::string &map_id, const std::string &capability,
                                  std::string *error) const {
  if (map_id.empty()) {
    *error = "missing map name";
    return std::nullopt;
  }
  if (capability.empty()) {
    *error = "missing capability";
    return std::nullopt;
  }
  const auto type = ArtifactTypeForCapability(capability);
  if (!type.has_value()) {
    *error = "unknown capability";
    return std::nullopt;
  }
  const auto identity = service_.Store().ReadDeclaredArtifactIdentity(map_id, *type, "");
  if (!identity.ok()) {
    *error = identity.reason;
    return std::nullopt;
  }
  return identity.identity;
}

std::string MapQueryCore::BundleJsonFromIdentity(const DeclaredArtifactIdentity &identity,
                                                 const std::string &capability,
                                                 std::uint64_t size_bytes) const {
  return "{"
         "\"action\":\"open_artifact\","
         "\"success\":true,"
         "\"schema_version\":\"mapd.artifact.v1\","
         "\"map_id\":" +
         JsonString(identity.map_id) + "," +
         "\"content_epoch\":" + std::to_string(identity.content_epoch) + "," +
         "\"frame_id\":" + JsonString(identity.frame_id) + "," +
         "\"capability\":" + JsonString(capability) + "," +
         "\"artifact\":{\"type\":" + JsonString(ArtifactTypeName(identity.type)) +
         ",\"size_bytes\":" + std::to_string(size_bytes) + "}" + "}";
}

OpenArtifactResult MapQueryCore::OpenArtifact(const std::string &map_id,
                                              const std::string &capability) const {
#if defined(__linux__)
  std::string id;
  try {
    id = MapStore::NormalizeMapId(map_id);
  } catch (const std::exception &exc) {
    return {false, FailureJson("open_artifact", exc.what(), "invalid_map_id"), -1};
  }
  try {
    if (capability.empty() || !ArtifactTypeForCapability(capability).has_value()) {
      return {false,
              FailureJson("open_artifact", "capability unavailable: " + capability,
                          "missing_capability"),
              -1};
    }
    auto map_lock = MapLock::TryAcquire(service_.Store().RootDir(), id, "mapd-open-artifact");
    if (!map_lock.has_value()) {
      return {false,
              FailureJson("open_artifact", "map write in progress: " + id, "map_write_in_progress"),
              -1};
    }
    if (!std::filesystem::is_directory(service_.Store().MapPath(id))) {
      return {false, FailureJson("open_artifact", "map not found: " + id, "map_not_found"), -1};
    }
    std::string error;
    const auto identity = DeclaredIdentityFor(id, capability, &error);
    if (!identity.has_value()) {
      return {false, FailureJson("open_artifact", error, "artifact_not_found"), -1};
    }
    const int fd = ::open(identity->artifact_path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) {
      return {false,
              FailureJson("open_artifact", std::string("open failed: ") + std::strerror(errno),
                          "artifact_not_found"),
              -1};
    }
    struct stat statbuf{};
    if (::fstat(fd, &statbuf) != 0 || !S_ISREG(statbuf.st_mode)) {
      ::close(fd);
      return {false,
              FailureJson("open_artifact", "opened artifact is not a regular file",
                          "artifact_not_found"),
              -1};
    }
    return {
        true,
        BundleJsonFromIdentity(*identity, capability, static_cast<std::uint64_t>(statbuf.st_size)),
        fd};
  } catch (const std::exception &exc) {
    return {false, FailureJson("open_artifact", exc.what(), "internal_error"), -1};
  }
#else
  (void)map_id;
  (void)capability;
  return {false,
          FailureJson("open_artifact", "file descriptor passing requires Linux",
                      "unsupported_platform"),
          -1};
#endif
}

}  // namespace lingtu::maps::mapd::query
