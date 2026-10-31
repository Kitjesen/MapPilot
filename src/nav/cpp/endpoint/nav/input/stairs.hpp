#pragma once

#include "lingtu/maps/storage/stairs.hpp"
#include "navigation/stairs.hpp"
#include <algorithm>
#include <filesystem>
#include <limits>

namespace lingtu::nav::endpoint {

inline std::vector<navigation::StairFlight> readStairFlights(
    const std::filesystem::path& map_artifact, std::int64_t content_epoch) {
  std::vector<navigation::StairFlight> flights;
  for (const auto& annotation : maps::ReadStairAnnotations(map_artifact.parent_path() / "pois.tsv")) {
    const auto& geometry = annotation.geometry;
    if (annotation.content_epoch != content_epoch || annotation.frame_id != "map" ||
        !geometry.recognized || geometry.surfaces.size() < 3U) continue;
    navigation::StairFlight flight;
    flight.id = annotation.id;
    flight.origin = {geometry.origin.x, geometry.origin.y, geometry.origin.z};
    flight.axis = {geometry.axis_x, geometry.axis_y, 0.0};
    flight.lateralMin = -std::numeric_limits<double>::infinity();
    flight.lateralMax = std::numeric_limits<double>::infinity();
    for (const auto& surface : geometry.surfaces) {
      flight.lateralMin = std::max(flight.lateralMin, surface.lateral_min);
      flight.lateralMax = std::min(flight.lateralMax, surface.lateral_max);
      flight.treads.push_back({surface.s_min, surface.s_max, surface.surface_z});
    }
    if (flight.lateralMin < flight.lateralMax) flights.push_back(std::move(flight));
  }
  return flights;
}

}  // namespace lingtu::nav::endpoint
