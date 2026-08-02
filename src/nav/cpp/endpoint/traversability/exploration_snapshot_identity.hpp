#pragma once

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

namespace lingtu::nav::endpoint {
namespace detail {

[[nodiscard]] inline bool validMapId(const std::string &value) {
  if (value.empty() || value.front() == '.' || value.front() == '-' ||
      value.find("..") != std::string::npos) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char ch) {
    return std::isalnum(ch) != 0 || ch == '_' || ch == '-' || ch == '.';
  });
}

[[nodiscard]] inline bool validProductSessionId(const std::string &value) {
  return value.size() >= 16U && value.size() <= 128U &&
         std::all_of(value.begin(), value.end(), [](unsigned char ch) {
           return std::isalnum(ch) != 0 || ch == '_' || ch == '-' || ch == '.';
         });
}

[[nodiscard]] inline std::int64_t mapVersionNumber(const std::string &value) {
  const std::size_t marker = value.rfind(":v");
  if (marker == std::string::npos || marker == 0U || marker + 2U >= value.size() ||
      value[marker + 2U] < '1' || value[marker + 2U] > '9') {
    throw std::runtime_error("LINGTU_MAP_VERSION must end in :v<positive integer>");
  }
  const char *first = value.data() + marker + 2U;
  const char *last = value.data() + value.size();
  std::int64_t version = 0;
  const auto parsed = std::from_chars(first, last, version);
  if (parsed.ec != std::errc{} || parsed.ptr != last || version <= 0) {
    throw std::runtime_error("LINGTU_MAP_VERSION must end in :v<positive integer>");
  }
  return version;
}

[[nodiscard]] inline bool validSha256(const std::string &value) {
  return value.size() == 64U && std::all_of(value.begin(), value.end(), [](unsigned char ch) {
           return std::isxdigit(ch) != 0;
         });
}

}  // namespace detail

struct ExplorationSnapshotIdentityEnvironment {
  std::string route;
  std::string product_session_id;
  std::string map_id;
  std::string map_version_id;
  std::string artifact_hash;
};

struct ExplorationSnapshotIdentity {
  std::string session_id;
  std::string map_id;
  std::int64_t map_version{0};
  std::string artifact_hash;
  bool live{true};
};

[[nodiscard]] inline ExplorationSnapshotIdentity
resolveExplorationSnapshotIdentity(ExplorationSnapshotIdentityEnvironment environment) {
  if (environment.route != "live" && environment.route != "map") {
    throw std::runtime_error("LINGTU_EXPLORE_ROUTE must be map or live");
  }
  if (!detail::validProductSessionId(environment.product_session_id)) {
    throw std::runtime_error("LINGTU_PRODUCT_SESSION_ID is missing or invalid");
  }
  if (environment.route == "live") {
    return {std::move(environment.product_session_id), {}, 0, {}, true};
  }
  if (!detail::validMapId(environment.map_id)) {
    throw std::runtime_error("LINGTU_MAP_ID is missing or invalid");
  }

  const std::int64_t version = detail::mapVersionNumber(environment.map_version_id);
  if (!detail::validSha256(environment.artifact_hash)) {
    throw std::runtime_error("LINGTU_MAP_POINTCLOUD_SHA256 must be a 64-character hex digest");
  }
  return {
      std::move(environment.product_session_id),
      std::move(environment.map_id),
      version,
      std::move(environment.artifact_hash),
      false,
  };
}

}  // namespace lingtu::nav::endpoint
