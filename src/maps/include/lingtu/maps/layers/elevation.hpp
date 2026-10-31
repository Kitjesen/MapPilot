#pragma once

#include "lingtu/maps/layers/grid.hpp"

#include <cstdint>
#include <limits>

namespace lingtu::maps::layers {

struct ElevationMapResult {
  Grid2D minZ;
  Grid2D maxZ;
  Grid2D clearance;
  std::vector<uint8_t> valid;
};

inline ElevationMapResult buildElevationMap(
    const std::vector<float>& xyzFlat,
    double robotX,
    double robotY,
    double resolution,
    double radius,
    double zFloor,
    double zCeil) {
  if (!(resolution > 0.0) || !(radius > 0.0)) {
    throw std::invalid_argument("resolution and radius must be positive");
  }
  const int size = static_cast<int>(std::floor((2.0 * radius) / resolution));
  if (size <= 0) {
    throw std::invalid_argument("computed elevation grid is empty");
  }
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  ElevationMapResult out;
  out.minZ = makeGrid2D(size, size, resolution, robotX - radius, robotY - radius, inf);
  out.maxZ = makeGrid2D(size, size, resolution, robotX - radius, robotY - radius, -inf);
  out.clearance = makeGrid2D(size, size, resolution, robotX - radius, robotY - radius, nan);
  out.valid.assign(static_cast<size_t>(size * size), 0);

  const size_t n = xyzFlat.size() / 3;
  for (size_t i = 0; i < n; ++i) {
    const float x = xyzFlat[i * 3 + 0];
    const float y = xyzFlat[i * 3 + 1];
    const float z = xyzFlat[i * 3 + 2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    if (!(z > zFloor && z < zCeil)) {
      continue;
    }
    const int col = static_cast<int>(std::floor((x - out.minZ.originX) / resolution));
    const int row = static_cast<int>(std::floor((y - out.minZ.originY) / resolution));
    if (row < 0 || row >= size || col < 0 || col >= size) {
      continue;
    }
    const int idx = out.minZ.index(row, col);
    out.valid[static_cast<size_t>(idx)] = 1;
    out.minZ.data[static_cast<size_t>(idx)] =
        std::min(out.minZ.data[static_cast<size_t>(idx)], z);
    out.maxZ.data[static_cast<size_t>(idx)] =
        std::max(out.maxZ.data[static_cast<size_t>(idx)], z);
  }

  for (int i = 0; i < size * size; ++i) {
    if (out.valid[static_cast<size_t>(i)] == 0) {
      out.minZ.data[static_cast<size_t>(i)] = nan;
      out.maxZ.data[static_cast<size_t>(i)] = nan;
      out.clearance.data[static_cast<size_t>(i)] = nan;
    } else {
      out.clearance.data[static_cast<size_t>(i)] =
          out.maxZ.data[static_cast<size_t>(i)] - out.minZ.data[static_cast<size_t>(i)];
    }
  }
  return out;
}

}  // namespace lingtu::maps::layers
