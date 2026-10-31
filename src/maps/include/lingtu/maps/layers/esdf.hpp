#pragma once

#include "lingtu/maps/layers/grid.hpp"

#include <cstdint>
#include <limits>

namespace lingtu::maps::layers {

struct EsdfResult {
  Grid2D distance;
  Grid2D gradX;
  Grid2D gradY;
};

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

}  // namespace lingtu::maps::layers
