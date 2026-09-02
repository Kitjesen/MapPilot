#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

namespace lingtu::nav::endpoint {

enum class Route {
  Map,
  Live,
};

[[nodiscard]] constexpr bool usesLiveSegment(Route route) noexcept {
  return route == Route::Live;
}

[[nodiscard]] constexpr std::string_view routeName(Route route) noexcept {
  switch (route) {
    case Route::Map:
      return "map";
    case Route::Live:
      return "live";
  }
  return "unknown";
}

[[nodiscard]] constexpr std::optional<Route> parseRoute(std::string_view value) noexcept {
  if (value == "map") {
    return Route::Map;
  }
  if (value == "live") {
    return Route::Live;
  }
  return std::nullopt;
}

[[nodiscard]] constexpr bool isCanonicalExploreRouteMapIdentity(
    Route route, std::string_view map_id, std::int64_t map_content_epoch) noexcept {
  if (route == Route::Live) {
    return map_id.empty() && map_content_epoch == 0;
  }
  return !map_id.empty() && map_content_epoch > 0;
}

[[nodiscard]] constexpr bool isCanonicalExploreSnapshotBinding(
    Route route, std::string_view expected_product_session_id,
    std::string_view expected_map_id, std::int64_t expected_map_content_epoch,
    std::string_view snapshot_session_id, bool snapshot_live, std::string_view snapshot_map_id,
    std::int64_t snapshot_map_content_epoch) noexcept {
  return !expected_product_session_id.empty() &&
         snapshot_session_id == expected_product_session_id &&
         snapshot_live == usesLiveSegment(route) &&
         isCanonicalExploreRouteMapIdentity(route, expected_map_id, expected_map_content_epoch) &&
         isCanonicalExploreRouteMapIdentity(route, snapshot_map_id, snapshot_map_content_epoch) &&
         snapshot_map_id == expected_map_id &&
         snapshot_map_content_epoch == expected_map_content_epoch;
}

}  // namespace lingtu::nav::endpoint
