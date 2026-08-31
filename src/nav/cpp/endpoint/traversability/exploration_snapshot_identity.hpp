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
  if (value.empty() || value.size() > 63U ||
      !((value.front() >= 'a' && value.front() <= 'z') ||
        (value.front() >= 'A' && value.front() <= 'Z') ||
        (value.front() >= '0' && value.front() <= '9'))) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char ch) {
    return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
           (ch >= '0' && ch <= '9') || ch == '_' || ch == '-' || ch == '.';
  });
}

[[nodiscard]] inline std::int64_t contentEpochNumber(const std::string &value) {
  if (value.empty() || value.front() < '1' || value.front() > '9') {
    throw std::runtime_error("LINGTU_MAP_CONTENT_EPOCH must be a positive integer");
  }
  const char *first = value.data();
  const char *last = value.data() + value.size();
  std::int64_t content_epoch = 0;
  const auto parsed = std::from_chars(first, last, content_epoch);
  if (parsed.ec != std::errc{} || parsed.ptr != last || content_epoch <= 0) {
    throw std::runtime_error("LINGTU_MAP_CONTENT_EPOCH must be a positive integer");
  }
  return content_epoch;
}

}  // namespace detail

struct ExplorationSnapshotIdentityEnvironment {
  std::string route;
  std::string product_session_id;
  std::string map_id;
  std::string map_content_epoch;
};

struct ExplorationSnapshotIdentity {
  std::string session_id;
  std::string map_id;
  std::int64_t map_content_epoch{0};
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
    return {std::move(environment.product_session_id), {}, 0, true};
  }
  if (!detail::validMapId(environment.map_id)) {
    throw std::runtime_error("LINGTU_MAP_ID is missing or invalid");
  }

  const std::int64_t content_epoch =
      detail::contentEpochNumber(environment.map_content_epoch);
  return {
      std::move(environment.product_session_id),
      std::move(environment.map_id),
      content_epoch,
      false,
  };
}

}  // namespace lingtu::nav::endpoint
