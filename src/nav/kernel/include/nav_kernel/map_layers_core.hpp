/**
 * nav_kernel/map_layers_core.hpp -- ROS-free L2 map layer algorithms.
 *
 * This is the commercial boundary for LingTu online maps: deterministic C++17
 * kernels with no ROS, grid_map, PCL, or Python dependency. Module code owns
 * transport and message packing; this file owns math.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace nav_kernel {

struct Grid2D {
  int rows = 0;
  int cols = 0;
  double resolution = 0.2;
  double originX = 0.0;
  double originY = 0.0;
  std::vector<float> data;

  bool empty() const { return rows <= 0 || cols <= 0 || data.empty(); }

  int index(int row, int col) const {
    return row * cols + col;
  }

  void validate(const char* name = "grid") const {
    if (rows < 0 || cols < 0) {
      throw std::invalid_argument(std::string(name) + ": rows/cols must be non-negative");
    }
    if (rows == 0 || cols == 0) {
      if (!data.empty()) {
        throw std::invalid_argument(std::string(name) + ": empty geometry with non-empty data");
      }
      return;
    }
    if (static_cast<int>(data.size()) != rows * cols) {
      throw std::invalid_argument(std::string(name) + ": data size does not match rows*cols");
    }
    if (!(resolution > 0.0) || !std::isfinite(resolution)) {
      throw std::invalid_argument(std::string(name) + ": resolution must be finite and positive");
    }
  }
};

inline Grid2D makeGrid2D(
    int rows,
    int cols,
    double resolution,
    double originX,
    double originY,
    float fill = 0.0f) {
  Grid2D g;
  g.rows = rows;
  g.cols = cols;
  g.resolution = resolution;
  g.originX = originX;
  g.originY = originY;
  if (rows > 0 && cols > 0) {
    g.data.assign(static_cast<size_t>(rows * cols), fill);
  }
  return g;
}

inline bool sameGeometry(const Grid2D& a, const Grid2D& b, double eps = 1e-9) {
  return a.rows == b.rows && a.cols == b.cols &&
         std::fabs(a.resolution - b.resolution) <= eps &&
         std::fabs(a.originX - b.originX) <= eps &&
         std::fabs(a.originY - b.originY) <= eps;
}

inline void requireSameGeometry(const Grid2D& reference, const Grid2D& layer, const char* name) {
  if (!layer.empty() && !sameGeometry(reference, layer)) {
    throw std::invalid_argument(std::string(name) + ": geometry does not match costmap");
  }
}

struct ElevationMapResult {
  Grid2D minZ;
  Grid2D maxZ;
  Grid2D clearance;
  std::vector<uint8_t> valid;
};

struct EsdfResult {
  Grid2D distance;
  Grid2D gradX;
  Grid2D gradY;
};

struct TerrainRiskParams {
  float maxSlopeDeg = 35.0f;
  float softSlopeStartDeg = 3.0f;
  float criticalStepM = 0.22f;
  float roughnessCriticalM = 0.08f;
};

struct TerrainRiskResult {
  Grid2D risk;
  Grid2D slopeDeg;
  Grid2D stepHeight;
  Grid2D roughness;
};

struct TraversabilityParams {
  float lethal = 100.0f;
  float inscribed = 99.0f;
  float maxSlopeDeg = 35.0f;
  float softSlopeStartDeg = 3.0f;
  float safeDistance = 1.5f;
  float proximityCap = 50.0f;
};

inline float clampFloat(float v, float lo, float hi) {
  return std::max(lo, std::min(hi, v));
}

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

inline std::vector<float> distanceTransform1D(const std::vector<float>& f) {
  const int n = static_cast<int>(f.size());
  const float inf = 1e20f;
  std::vector<float> d(static_cast<size_t>(n), inf);
  if (n == 0) {
    return d;
  }
  std::vector<int> features;
  features.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    if (f[static_cast<size_t>(i)] < inf * 0.5f) {
      features.push_back(i);
    }
  }
  if (features.empty()) {
    return d;
  }

  std::vector<int> v(static_cast<size_t>(n), 0);
  std::vector<float> z(static_cast<size_t>(n + 1), 0.0f);
  int k = 0;
  v[0] = features[0];
  z[0] = -inf;
  z[1] = inf;
  for (size_t qi = 1; qi < features.size(); ++qi) {
    const int q = features[qi];
    float s = 0.0f;
    do {
      const int vk = v[static_cast<size_t>(k)];
      s = ((f[static_cast<size_t>(q)] + static_cast<float>(q * q)) -
           (f[static_cast<size_t>(vk)] + static_cast<float>(vk * vk))) /
          (2.0f * static_cast<float>(q - vk));
      if (s <= z[static_cast<size_t>(k)]) {
        --k;
      }
    } while (k >= 0 && s <= z[static_cast<size_t>(k)]);
    ++k;
    v[static_cast<size_t>(k)] = q;
    z[static_cast<size_t>(k)] = s;
    z[static_cast<size_t>(k + 1)] = inf;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[static_cast<size_t>(k + 1)] < static_cast<float>(q)) {
      ++k;
    }
    const int vk = v[static_cast<size_t>(k)];
    const float diff = static_cast<float>(q - vk);
    d[static_cast<size_t>(q)] = diff * diff + f[static_cast<size_t>(vk)];
  }
  return d;
}

inline std::vector<float> distanceTransform2D(
    const std::vector<uint8_t>& feature,
    int rows,
    int cols) {
  if (rows <= 0 || cols <= 0 || static_cast<int>(feature.size()) != rows * cols) {
    throw std::invalid_argument("distanceTransform2D: invalid geometry");
  }
  const float inf = 1e20f;
  std::vector<float> temp(static_cast<size_t>(rows * cols), inf);
  std::vector<float> f(static_cast<size_t>(std::max(rows, cols)), inf);

  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      const int idx = row * cols + col;
      f[static_cast<size_t>(col)] = feature[static_cast<size_t>(idx)] ? 0.0f : inf;
    }
    auto d = distanceTransform1D(std::vector<float>(f.begin(), f.begin() + cols));
    for (int col = 0; col < cols; ++col) {
      temp[static_cast<size_t>(row * cols + col)] = d[static_cast<size_t>(col)];
    }
  }

  std::vector<float> out(static_cast<size_t>(rows * cols), inf);
  for (int col = 0; col < cols; ++col) {
    for (int row = 0; row < rows; ++row) {
      f[static_cast<size_t>(row)] = temp[static_cast<size_t>(row * cols + col)];
    }
    auto d = distanceTransform1D(std::vector<float>(f.begin(), f.begin() + rows));
    for (int row = 0; row < rows; ++row) {
      out[static_cast<size_t>(row * cols + col)] = d[static_cast<size_t>(row)];
    }
  }
  return out;
}

inline float gradientAt(const Grid2D& g, int row, int col, bool alongX) {
  const auto value = [&](int r, int c) -> float {
    const float v = g.data[static_cast<size_t>(g.index(r, c))];
    return std::isfinite(v) ? v : 0.0f;
  };
  if (alongX) {
    if (g.cols <= 1) return 0.0f;
    if (col == 0) return (value(row, 1) - value(row, 0)) / static_cast<float>(g.resolution);
    if (col == g.cols - 1) {
      return (value(row, col) - value(row, col - 1)) / static_cast<float>(g.resolution);
    }
    return (value(row, col + 1) - value(row, col - 1)) / static_cast<float>(2.0 * g.resolution);
  }
  if (g.rows <= 1) return 0.0f;
  if (row == 0) return (value(1, col) - value(0, col)) / static_cast<float>(g.resolution);
  if (row == g.rows - 1) {
    return (value(row, col) - value(row - 1, col)) / static_cast<float>(g.resolution);
  }
  return (value(row + 1, col) - value(row - 1, col)) / static_cast<float>(2.0 * g.resolution);
}

inline EsdfResult computeEsdf(const Grid2D& occupancy, float obstacleThreshold = 50.0f) {
  occupancy.validate("occupancy");
  EsdfResult out;
  out.distance = makeGrid2D(
      occupancy.rows, occupancy.cols, occupancy.resolution, occupancy.originX, occupancy.originY, 0.0f);
  out.gradX = makeGrid2D(
      occupancy.rows, occupancy.cols, occupancy.resolution, occupancy.originX, occupancy.originY, 0.0f);
  out.gradY = makeGrid2D(
      occupancy.rows, occupancy.cols, occupancy.resolution, occupancy.originX, occupancy.originY, 0.0f);

  const int n = occupancy.rows * occupancy.cols;
  std::vector<uint8_t> obstacles(static_cast<size_t>(n), 0);
  std::vector<uint8_t> freeCells(static_cast<size_t>(n), 0);
  bool anyObstacle = false;
  bool anyFree = false;
  for (int i = 0; i < n; ++i) {
    const bool obstacle = occupancy.data[static_cast<size_t>(i)] >= obstacleThreshold;
    obstacles[static_cast<size_t>(i)] = obstacle ? 1 : 0;
    freeCells[static_cast<size_t>(i)] = obstacle ? 0 : 1;
    anyObstacle = anyObstacle || obstacle;
    anyFree = anyFree || !obstacle;
  }

  if (!anyObstacle || !anyFree) {
    const float value = !anyObstacle
        ? std::numeric_limits<float>::infinity()
        : -std::numeric_limits<float>::infinity();
    std::fill(out.distance.data.begin(), out.distance.data.end(), value);
    return out;
  }

  const auto distObsSq = distanceTransform2D(obstacles, occupancy.rows, occupancy.cols);
  const auto distFreeSq = distanceTransform2D(freeCells, occupancy.rows, occupancy.cols);
  for (int i = 0; i < n; ++i) {
    const bool obstacle = obstacles[static_cast<size_t>(i)] != 0;
    const float cellDist = std::sqrt(obstacle ? distFreeSq[static_cast<size_t>(i)]
                                              : distObsSq[static_cast<size_t>(i)]);
    out.distance.data[static_cast<size_t>(i)] =
        (obstacle ? -cellDist : cellDist) * static_cast<float>(occupancy.resolution);
  }

  for (int row = 0; row < occupancy.rows; ++row) {
    for (int col = 0; col < occupancy.cols; ++col) {
      const int idx = out.distance.index(row, col);
      out.gradX.data[static_cast<size_t>(idx)] = gradientAt(out.distance, row, col, true);
      out.gradY.data[static_cast<size_t>(idx)] = gradientAt(out.distance, row, col, false);
    }
  }
  return out;
}

inline TerrainRiskResult computeTerrainRisk(
    const ElevationMapResult& elevation,
    const TerrainRiskParams& params = TerrainRiskParams()) {
  elevation.maxZ.validate("elevation.maxZ");
  if (static_cast<int>(elevation.valid.size()) != elevation.maxZ.rows * elevation.maxZ.cols) {
    throw std::invalid_argument("elevation.valid size does not match grid");
  }

  TerrainRiskResult out;
  out.risk = makeGrid2D(
      elevation.maxZ.rows, elevation.maxZ.cols, elevation.maxZ.resolution,
      elevation.maxZ.originX, elevation.maxZ.originY, 0.0f);
  out.slopeDeg = makeGrid2D(
      elevation.maxZ.rows, elevation.maxZ.cols, elevation.maxZ.resolution,
      elevation.maxZ.originX, elevation.maxZ.originY, 0.0f);
  out.stepHeight = makeGrid2D(
      elevation.maxZ.rows, elevation.maxZ.cols, elevation.maxZ.resolution,
      elevation.maxZ.originX, elevation.maxZ.originY, 0.0f);
  out.roughness = makeGrid2D(
      elevation.maxZ.rows, elevation.maxZ.cols, elevation.maxZ.resolution,
      elevation.maxZ.originX, elevation.maxZ.originY, 0.0f);

  const int rows = elevation.maxZ.rows;
  const int cols = elevation.maxZ.cols;
  const float maxSlope = std::max(1.0f, params.maxSlopeDeg);
  const float criticalStep = std::max(1e-3f, params.criticalStepM);
  const float roughCritical = std::max(1e-3f, params.roughnessCriticalM);

  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      const int idx = elevation.maxZ.index(row, col);
      if (elevation.valid[static_cast<size_t>(idx)] == 0 ||
          !std::isfinite(elevation.maxZ.data[static_cast<size_t>(idx)])) {
        continue;
      }

      const auto validAt = [&](int r, int c) -> bool {
        if (r < 0 || r >= rows || c < 0 || c >= cols) return false;
        const int vidx = elevation.maxZ.index(r, c);
        return elevation.valid[static_cast<size_t>(vidx)] != 0 &&
            std::isfinite(elevation.maxZ.data[static_cast<size_t>(vidx)]);
      };
      const auto zAt = [&](int r, int c) -> float {
        return elevation.maxZ.data[static_cast<size_t>(elevation.maxZ.index(r, c))];
      };
      const float centerZ = elevation.maxZ.data[static_cast<size_t>(idx)];
      float dzdx = 0.0f;
      float dzdy = 0.0f;
      if (validAt(row, col - 1) && validAt(row, col + 1)) {
        dzdx = (zAt(row, col + 1) - zAt(row, col - 1)) /
            static_cast<float>(2.0 * elevation.maxZ.resolution);
      } else if (validAt(row, col + 1)) {
        dzdx = (zAt(row, col + 1) - centerZ) / static_cast<float>(elevation.maxZ.resolution);
      } else if (validAt(row, col - 1)) {
        dzdx = (centerZ - zAt(row, col - 1)) / static_cast<float>(elevation.maxZ.resolution);
      }
      if (validAt(row - 1, col) && validAt(row + 1, col)) {
        dzdy = (zAt(row + 1, col) - zAt(row - 1, col)) /
            static_cast<float>(2.0 * elevation.maxZ.resolution);
      } else if (validAt(row + 1, col)) {
        dzdy = (zAt(row + 1, col) - centerZ) / static_cast<float>(elevation.maxZ.resolution);
      } else if (validAt(row - 1, col)) {
        dzdy = (centerZ - zAt(row - 1, col)) / static_cast<float>(elevation.maxZ.resolution);
      }
      const float slope = std::atan(std::sqrt(dzdx * dzdx + dzdy * dzdy)) *
          static_cast<float>(180.0 / M_PI);
      out.slopeDeg.data[static_cast<size_t>(idx)] = slope;

      float zMin = std::numeric_limits<float>::infinity();
      float zMax = -std::numeric_limits<float>::infinity();
      float maxStep = 0.0f;
      for (int rr = std::max(0, row - 1); rr <= std::min(rows - 1, row + 1); ++rr) {
        for (int cc = std::max(0, col - 1); cc <= std::min(cols - 1, col + 1); ++cc) {
          const int nidx = elevation.maxZ.index(rr, cc);
          if (elevation.valid[static_cast<size_t>(nidx)] == 0) {
            continue;
          }
          const float z = elevation.maxZ.data[static_cast<size_t>(nidx)];
          if (!std::isfinite(z)) {
            continue;
          }
          zMin = std::min(zMin, z);
          zMax = std::max(zMax, z);
          maxStep = std::max(maxStep, std::fabs(z - centerZ));
        }
      }
      const float rough = (std::isfinite(zMin) && std::isfinite(zMax)) ? (zMax - zMin) : 0.0f;
      out.stepHeight.data[static_cast<size_t>(idx)] = maxStep;
      out.roughness.data[static_cast<size_t>(idx)] = rough;

      const float slopeRisk = (slope <= params.softSlopeStartDeg)
          ? 0.0f
          : clampFloat(slope / maxSlope, 0.0f, 1.0f) * 100.0f;
      const float stepRisk = clampFloat(maxStep / criticalStep, 0.0f, 1.0f) * 100.0f;
      const float roughRisk = clampFloat(rough / roughCritical, 0.0f, 1.0f) * 100.0f;
      float risk = std::max(slopeRisk, std::max(stepRisk, roughRisk));
      if (slope >= maxSlope || maxStep >= criticalStep) {
        risk = 100.0f;
      }
      out.risk.data[static_cast<size_t>(idx)] = clampFloat(risk, 0.0f, 100.0f);
    }
  }
  return out;
}

inline Grid2D fuseTraversabilityCost(
    const Grid2D& costmap,
    const Grid2D& slopeDeg,
    const Grid2D& esdfDistance,
    const Grid2D& terrainRisk,
    const TraversabilityParams& params = TraversabilityParams()) {
  costmap.validate("costmap");
  requireSameGeometry(costmap, slopeDeg, "slopeDeg");
  requireSameGeometry(costmap, esdfDistance, "esdfDistance");
  requireSameGeometry(costmap, terrainRisk, "terrainRisk");

  Grid2D fused = costmap;
  const int n = costmap.rows * costmap.cols;
  const float inscribed = params.inscribed;
  const float lethal = params.lethal;
  const float maxSlope = std::max(1.0f, params.maxSlopeDeg);
  const float safeDistance = std::max(1e-3f, params.safeDistance);
  const float proxCap = clampFloat(params.proximityCap, 0.0f, 100.0f);

  for (int i = 0; i < n; ++i) {
    const size_t idx = static_cast<size_t>(i);
    const bool hard = costmap.data[idx] >= inscribed;
    if (!hard && !slopeDeg.empty()) {
      const float slope = slopeDeg.data[idx];
      if (std::isfinite(slope)) {
        if (slope >= maxSlope) {
          fused.data[idx] = lethal;
        } else if (slope > params.softSlopeStartDeg) {
          fused.data[idx] = std::max(
              fused.data[idx], clampFloat(slope / maxSlope, 0.0f, 0.97f) * 100.0f);
        }
      }
    }

    if (fused.data[idx] < inscribed && !esdfDistance.empty()) {
      const float d = esdfDistance.data[idx];
      if (std::isfinite(d)) {
        const float prox = clampFloat(1.0f - d / safeDistance, 0.0f, 1.0f) * proxCap;
        fused.data[idx] = std::max(fused.data[idx], prox);
      }
    }

    if (fused.data[idx] < inscribed && !terrainRisk.empty()) {
      const float risk = terrainRisk.data[idx];
      if (std::isfinite(risk)) {
        fused.data[idx] = std::max(fused.data[idx], clampFloat(risk, 0.0f, 100.0f));
      }
    }
    fused.data[idx] = clampFloat(fused.data[idx], 0.0f, 100.0f);
  }
  return fused;
}

}  // namespace nav_kernel
