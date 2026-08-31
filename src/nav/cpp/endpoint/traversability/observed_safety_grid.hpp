#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "lingtu/maps/layers/grid.hpp"

namespace lingtu::nav::endpoint {

inline double snappedSafetyGridOrigin(double center, double radius, double resolution) {
  const double raw_origin = center - radius;
  if (!std::isfinite(raw_origin) || !std::isfinite(resolution) || resolution <= 0.0) {
    return raw_origin;
  }
  const double scaled =
      std::nextafter(raw_origin / resolution, std::numeric_limits<double>::infinity());
  return std::floor(scaled) * resolution;
}

inline double safetyGridAlignmentResidual(double origin, double resolution) {
  if (!std::isfinite(origin) || !std::isfinite(resolution) || resolution <= 0.0) {
    return -1.0;
  }
  return std::abs(origin - std::round(origin / resolution) * resolution);
}

inline lingtu::maps::layers::Grid2D makeUnknownSafetyGrid(int rows, int cols, double resolution,
                                                          double origin_x, double origin_y,
                                                          float unknown_cost = 100.0F) {
  return lingtu::maps::layers::makeGrid2D(rows, cols, resolution, origin_x, origin_y, unknown_cost);
}

inline std::int64_t safetyGridLatticeIndex(double coordinate, double origin, double resolution) {
  const double normalized = (coordinate - origin) / resolution;
  const double boundary = std::nearbyint(normalized);
  const double tolerance =
      4.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, std::fabs(normalized));
  return static_cast<std::int64_t>(
      std::floor(std::fabs(normalized - boundary) <= tolerance ? boundary : normalized));
}

inline bool safetyGridCell(const lingtu::maps::layers::Grid2D &grid, double x, double y, int &row,
                           int &col) {
  if (grid.empty() || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(grid.resolution) ||
      grid.resolution <= 0.0) {
    return false;
  }
  col = static_cast<int>(safetyGridLatticeIndex(x, grid.originX, grid.resolution));
  row = static_cast<int>(safetyGridLatticeIndex(y, grid.originY, grid.resolution));
  return row >= 0 && row < grid.rows && col >= 0 && col < grid.cols;
}

inline bool raiseSafetyCostAt(lingtu::maps::layers::Grid2D &grid, double x, double y, float cost) {
  int row = -1;
  int col = -1;
  if (!std::isfinite(cost) || !safetyGridCell(grid, x, y, row, col)) {
    return false;
  }
  float &value = grid.data[static_cast<std::size_t>(grid.index(row, col))];
  value = std::max(value, cost);
  return true;
}

inline bool markObservedRayFree(lingtu::maps::layers::Grid2D &grid, double origin_x,
                                double origin_y, double hit_x, double hit_y,
                                float free_cost = 0.0F) {
  int origin_row = -1;
  int origin_col = -1;
  if (!std::isfinite(hit_x) || !std::isfinite(hit_y) || !std::isfinite(free_cost) ||
      !safetyGridCell(grid, origin_x, origin_y, origin_row, origin_col)) {
    return false;
  }
  const double dx = hit_x - origin_x;
  const double dy = hit_y - origin_y;
  const double distance = std::hypot(dx, dy);
  if (!std::isfinite(distance)) {
    return false;
  }
  const double grid_diagonal = std::hypot(static_cast<double>(grid.cols) * grid.resolution,
                                          static_cast<double>(grid.rows) * grid.resolution);
  const double traversed_distance = std::min(distance, grid_diagonal);
  const double sample_step = std::max(1e-6, grid.resolution * 0.5);
  const int steps = std::max(1, static_cast<int>(std::ceil(traversed_distance / sample_step)));
  bool observed = false;
  for (int step = 0; step <= steps; ++step) {
    const double ray_distance = std::min(distance, static_cast<double>(step) * traversed_distance /
                                                       static_cast<double>(steps));
    const double ratio = distance > 1e-12 ? ray_distance / distance : 0.0;
    const double x = origin_x + ratio * dx;
    const double y = origin_y + ratio * dy;
    int row = -1;
    int col = -1;
    if (!safetyGridCell(grid, x, y, row, col)) {
      if (observed) {
        break;
      }
      continue;
    }
    grid.data[static_cast<std::size_t>(grid.index(row, col))] = free_cost;
    observed = true;
  }
  return observed;
}

}  // namespace lingtu::nav::endpoint
