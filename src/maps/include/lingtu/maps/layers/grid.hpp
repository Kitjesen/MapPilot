#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace lingtu::maps::layers {

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

inline float clampFloat(float v, float lo, float hi) {
  return std::max(lo, std::min(hi, v));
}

}  // namespace lingtu::maps::layers
