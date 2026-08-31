#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <limits>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include "dds/frame.hpp"
#include "lingtu/maps/layers/grid.hpp"
#include "traversability/observed_safety_grid.hpp"

namespace lingtu::nav::endpoint {

struct LocalRiskProjectionResult {
  std::optional<lingtu::maps::layers::Grid2D> grid;
  std::string_view reason{"none"};

  [[nodiscard]] bool ok() const { return grid.has_value(); }
};

// Terrain points contain both support samples and vertical obstacle returns.
// Surface slope/step must use the lowest support in each cell; cells with a
// vertical span or obstacle height are left to the obstacle-height layer.
inline std::optional<lingtu::maps::layers::ElevationMapResult>
supportElevation(const std::vector<float> &terrain_xyzi, double center_x, double center_y,
                 double resolution, double radius, double z_floor, double z_ceil,
                 float obstacle_height) {
  if (terrain_xyzi.size() % 4U != 0U || !std::isfinite(center_x) ||
      !std::isfinite(center_y) || !(resolution > 0.0) || !std::isfinite(resolution) ||
      !(radius > 0.0) || !std::isfinite(radius) || !std::isfinite(z_floor) ||
      !std::isfinite(z_ceil) || z_floor >= z_ceil || !(obstacle_height > 0.0F) ||
      !std::isfinite(obstacle_height)) {
    return std::nullopt;
  }

  std::vector<float> xyz;
  xyz.reserve((terrain_xyzi.size() / 4U) * 3U);
  for (std::size_t i = 0; i < terrain_xyzi.size(); i += 4U) {
    xyz.push_back(terrain_xyzi[i + 0U]);
    xyz.push_back(terrain_xyzi[i + 1U]);
    xyz.push_back(terrain_xyzi[i + 2U]);
  }

  try {
    auto elevation = lingtu::maps::layers::buildElevationMap(
        xyz, center_x, center_y, resolution, radius, z_floor, z_ceil);
    std::vector<float> max_height(elevation.valid.size(), 0.0F);
    for (std::size_t i = 0; i < terrain_xyzi.size(); i += 4U) {
      const float x = terrain_xyzi[i + 0U];
      const float y = terrain_xyzi[i + 1U];
      const float z = terrain_xyzi[i + 2U];
      const float height = terrain_xyzi[i + 3U];
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
          !std::isfinite(height) || !(z > z_floor && z < z_ceil)) {
        continue;
      }
      const int col = static_cast<int>(std::floor((x - elevation.minZ.originX) / resolution));
      const int row = static_cast<int>(std::floor((y - elevation.minZ.originY) / resolution));
      if (row < 0 || row >= elevation.minZ.rows || col < 0 || col >= elevation.minZ.cols) {
        continue;
      }
      const std::size_t cell = static_cast<std::size_t>(elevation.minZ.index(row, col));
      max_height[cell] = std::max(max_height[cell], height);
    }

    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (std::size_t i = 0; i < elevation.valid.size(); ++i) {
      if (elevation.valid[i] == 0U) {
        continue;
      }
      const float span = elevation.clearance.data[i];
      if (!std::isfinite(span) || span >= obstacle_height || max_height[i] >= obstacle_height) {
        elevation.valid[i] = 0U;
        elevation.minZ.data[i] = nan;
        elevation.maxZ.data[i] = nan;
        elevation.clearance.data[i] = nan;
        continue;
      }
      elevation.maxZ.data[i] = elevation.minZ.data[i];
      elevation.clearance.data[i] = 0.0F;
    }
    return elevation;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

inline bool mergeRiskGridMax(lingtu::maps::layers::Grid2D &destination,
                             const lingtu::maps::layers::Grid2D &source) {
  try {
    destination.validate("risk destination grid");
    source.validate("risk source grid");
  } catch (const std::exception &) {
    return false;
  }
  for (int row = 0; row < source.rows; ++row) {
    for (int col = 0; col < source.cols; ++col) {
      const float cost = source.data[static_cast<std::size_t>(source.index(row, col))];
      if (!std::isfinite(cost) || cost < 0.0F || cost > 100.0F) {
        return false;
      }
      if (cost <= 0.0F) {
        continue;
      }
      const double x = source.originX + (static_cast<double>(col) + 0.5) * source.resolution;
      const double y = source.originY + (static_cast<double>(row) + 0.5) * source.resolution;
      const int destination_col =
          static_cast<int>(std::floor((x - destination.originX) / destination.resolution));
      const int destination_row =
          static_cast<int>(std::floor((y - destination.originY) / destination.resolution));
      if (destination_row < 0 || destination_row >= destination.rows || destination_col < 0 ||
          destination_col >= destination.cols) {
        continue;
      }
      float &destination_cost = destination.data[static_cast<std::size_t>(
          destination.index(destination_row, destination_col))];
      destination_cost = std::max(destination_cost, cost);
    }
  }
  return true;
}

// Traversability is a terrain-risk layer. LiDAR occupancy remains owned by the
// obstacle cloud (CMU and direct-command safety) or the 3D collision map (SCAN). The
// geometry grid supplies only the current rolling window and never contributes
// occupancy or unknown-space costs.
inline std::optional<lingtu::maps::layers::Grid2D>
makeControlRiskGrid(const lingtu::maps::layers::Grid2D &geometry_grid,
                    const lingtu::maps::layers::Grid2D *terrain_risk) {
  try {
    geometry_grid.validate("control-risk geometry grid");
  } catch (const std::exception &) {
    return std::nullopt;
  }

  auto result = geometry_grid;
  std::fill(result.data.begin(), result.data.end(), 0.0F);
  if (terrain_risk != nullptr && !mergeRiskGridMax(result, *terrain_risk)) {
    return std::nullopt;
  }
  return result;
}

// Reprojects the current rolling map-frame risk window into an axis-aligned,
// robot-centred odom window. The source grid stays authoritative for global
// map/safety uses; this is a short-lived local-planning view. The caller makes
// the uncovered-space policy explicit instead of silently turning rotation
// corners into obstacles.
inline LocalRiskProjectionResult
projectRollingRiskGridToOdom(const lingtu::maps::layers::Grid2D &map_grid,
                             const RigidTransform &map_from_odom,
                             const nav_kernel::Vec3 &odom_center, float uncovered_cost) {
  LocalRiskProjectionResult out;
  try {
    map_grid.validate("map risk grid");
  } catch (const std::exception &) {
    out.reason = "map_grid_invalid";
    return out;
  }
  if (map_grid.empty()) {
    out.reason = "map_grid_empty";
    return out;
  }
  if (!map_from_odom.valid || !quaternionIsFiniteAndNonzero(map_from_odom.rotation) ||
      !std::isfinite(map_from_odom.translation.x) || !std::isfinite(map_from_odom.translation.y) ||
      !std::isfinite(map_from_odom.translation.z) || !std::isfinite(map_from_odom.stamp_s) ||
      map_from_odom.stamp_s <= 0.0) {
    out.reason = "map_odom_transform_invalid";
    return out;
  }
  if (!std::isfinite(odom_center.x) || !std::isfinite(odom_center.y)) {
    out.reason = "odom_center_invalid";
    return out;
  }
  if (!std::isfinite(uncovered_cost) || uncovered_cost < 0.0F || uncovered_cost > 100.0F) {
    out.reason = "uncovered_cost_invalid";
    return out;
  }

  const double half_width_m = static_cast<double>(map_grid.cols) * map_grid.resolution * 0.5;
  const double half_height_m = static_cast<double>(map_grid.rows) * map_grid.resolution * 0.5;
  const double origin_x = snappedSafetyGridOrigin(odom_center.x, half_width_m, map_grid.resolution);
  const double origin_y =
      snappedSafetyGridOrigin(odom_center.y, half_height_m, map_grid.resolution);
  auto odom_grid = makeUnknownSafetyGrid(map_grid.rows, map_grid.cols, map_grid.resolution,
                                         origin_x, origin_y, uncovered_cost);
  for (int row = 0; row < odom_grid.rows; ++row) {
    for (int col = 0; col < odom_grid.cols; ++col) {
      const nav_kernel::Vec3 map_point = transformPoint(
          map_from_odom,
          {odom_grid.originX + (static_cast<double>(col) + 0.5) * odom_grid.resolution,
           odom_grid.originY + (static_cast<double>(row) + 0.5) * odom_grid.resolution, 0.0});
      int source_row = -1;
      int source_col = -1;
      if (!safetyGridCell(map_grid, map_point.x, map_point.y, source_row, source_col))
        continue;
      const float source_cost =
          map_grid.data[static_cast<std::size_t>(map_grid.index(source_row, source_col))];
      odom_grid.data[static_cast<std::size_t>(odom_grid.index(row, col))] =
          std::isfinite(source_cost) ? std::clamp(source_cost, 0.0F, 100.0F) : 100.0F;
    }
  }
  out.grid = std::move(odom_grid);
  return out;
}

}  // namespace lingtu::nav::endpoint
