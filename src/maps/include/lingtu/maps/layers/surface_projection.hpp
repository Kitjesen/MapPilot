#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <vector>

#include "lingtu/maps/layers/rolling_occupancy.hpp"
#include "lingtu/maps/layers/grid.hpp"
#include "lingtu/maps/layers/ground_surface.hpp"

namespace lingtu::maps::layers {

// Display-only support/obstacle projection. It does not authorize traversal.
// Ground candidates use observed lower surfaces connected to a nearby seed;
// obstacles use the current 3D occupancy relative to that support surface.
inline Grid2D ProjectSupportSurface(
    const GroundSurfaceResult& surface, const RollingOccupancySnapshot& occupancy,
    double sensor_x, double sensor_y, double sensor_z) {
  if (surface.height.empty()) return {};
  const auto& geometry = surface.height;
  const double resolution = geometry.resolution;
  const double origin_x = geometry.originX, origin_y = geometry.originY;
  const int rows = geometry.rows, cols = geometry.cols;
  auto result = makeGrid2D(rows, cols, resolution, origin_x, origin_y, -1.0F);
  const auto count = static_cast<std::size_t>(rows * cols);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const auto& ground = surface.height.data;
  std::vector<float> seed_heights;
  // MID-360 ground returns can start beyond a metre; search observed surfaces
  // farther out without filling the near-field blind region.
  constexpr double seed_radius_m = 2.0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const auto i = result.index(r, c);
      const double x = result.originX + (c + 0.5) * result.resolution;
      const double y = result.originY + (r + 0.5) * result.resolution;
      if (std::hypot(x - sensor_x, y - sensor_y) <= seed_radius_m &&
          ground[i] < sensor_z - 0.15 && ground[i] > sensor_z - 1.0) {
        seed_heights.push_back(ground[i]);
      }
    }
  }
  if (seed_heights.size() < 3U) return result;
  const auto seed_q = seed_heights.begin() + static_cast<std::ptrdiff_t>(seed_heights.size() / 4U);
  std::nth_element(seed_heights.begin(), seed_q, seed_heights.end());
  const float seed_z = *seed_q;
  std::queue<int> pending;
  std::vector<bool> connected(count, false);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const auto i = result.index(r, c);
      const double x = result.originX + (c + 0.5) * result.resolution;
      const double y = result.originY + (r + 0.5) * result.resolution;
      if (std::hypot(x - sensor_x, y - sensor_y) <= seed_radius_m &&
          std::isfinite(ground[i]) && std::abs(ground[i] - seed_z) <= 0.08) {
        connected[i] = true;
        pending.push(i);
      }
    }
  }
  const int dr[] = {-1, 1, 0, 0};
  const int dc[] = {0, 0, -1, 1};
  while (!pending.empty()) {
    const int i = pending.front();
    pending.pop();
    const int r = i / cols, c = i % cols;
    result.data[i] = 0.0F;
    for (int d = 0; d < 4; ++d) {
      const int nr = r + dr[d], nc = c + dc[d];
      if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
      const int n = result.index(nr, nc);
      const double dx = dc[d]*resolution, dy = dr[d]*resolution;
      const double from_i = ground[i] + surface.gradient_x.data[i]*dx + surface.gradient_y.data[i]*dy;
      const double from_n = ground[n] - surface.gradient_x.data[n]*dx - surface.gradient_y.data[n]*dy;
      if (!connected[n] && std::isfinite(ground[n]) &&
          std::abs(ground[n] - from_i) <= 0.06 && std::abs(ground[i] - from_n) <= 0.06) {
        connected[n] = true;
        pending.push(n);
      }
    }
  }
  // Nearby connected support supplies a reference for obstacle columns only.
  // It must never fill an unobserved column with a green support cell.
  std::vector<float> reference = ground;
  std::vector<int> reference_cell(count, -1);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const int i = result.index(r, c);
      if (connected[i]) { reference_cell[i] = i; continue; }
      reference[i] = nan;
      for (int d = 0; d < 4; ++d) {
        const int nr = r + dr[d], nc = c + dc[d];
        if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
        const int n = result.index(nr, nc);
        const float predicted = ground[n] - surface.gradient_x.data[n]*dc[d]*resolution
            - surface.gradient_y.data[n]*dr[d]*resolution;
        if (connected[n] && (!std::isfinite(reference[i]) || predicted < reference[i])) {
          reference[i] = predicted;
          reference_cell[i] = n;
        }
      }
    }
  }
  // Query only columns with a ground reference and their obstacle-height band.
  // Once a display cell is occupied, no other voxel can change its result.
  for (int y = 0; y < occupancy.size_y; ++y) {
    const double wy = occupancy.origin_y_m + (y + 0.5) * occupancy.resolution_m;
    const int r = static_cast<int>(std::floor((wy - result.originY) / result.resolution));
    if (r < 0 || r >= rows) continue;
    for (int x = 0; x < occupancy.size_x; ++x) {
      const double wx = occupancy.origin_x_m + (x + 0.5) * occupancy.resolution_m;
      const int c = static_cast<int>(std::floor((wx - result.originX) / result.resolution));
      if (c < 0 || c >= cols) continue;
      const int i = result.index(r, c);
      if (!std::isfinite(reference[i]) || result.data[i] == 100.0F) continue;
      const int support = reference_cell[i];
      const double floor_z = reference[i]
          + surface.gradient_x.data[support]*(wx-(origin_x+(c+.5)*resolution))
          + surface.gradient_y.data[support]*(wy-(origin_y+(r+.5)*resolution));
      const double lower =
          (floor_z + 0.12 - occupancy.origin_z_m) / occupancy.resolution_m - 0.5;
      const double upper =
          (floor_z + 0.8 - occupancy.origin_z_m) / occupancy.resolution_m - 0.5;
      const int first = static_cast<int>(std::clamp(
          std::floor(lower), 0.0, static_cast<double>(occupancy.size_z)));
      const int end = static_cast<int>(std::clamp(
          std::ceil(upper) + 1.0, 0.0, static_cast<double>(occupancy.size_z)));
      for (int z = first; z < end; ++z) {
        const double h = occupancy.origin_z_m + (z + 0.5) * occupancy.resolution_m;
        // Retain the exact height predicate at the two voxel boundaries.
        if (h - floor_z > 0.12 && h - floor_z <= 0.8 &&
            occupancy.state[occupancy.Index(x, y, z)] ==
                static_cast<std::uint8_t>(OccupancyState::kOccupied)) {
          result.data[i] = 100.0F;
          break;
        }
      }
    }
  }
  return result;
}

}  // namespace lingtu::maps::layers
