#pragma once

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

[[nodiscard]] constexpr bool allowsExplorationSegmentFallback(Route route,
                                                              bool snapshot_live) noexcept {
  return usesLiveSegment(route) && snapshot_live;
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

}  // namespace lingtu::nav::endpoint
