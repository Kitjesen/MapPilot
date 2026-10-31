#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace nav_kernel::local {

// Keep Mapd's global-cell arithmetic while retaining a fractional grid origin.
// Subtracting the whole origin before floor can round across a published face.
struct GridAxis {
  GridAxis(double minimum, double cellSize) : resolution(cellSize) {
    const double origin = minimum / resolution;
    base = static_cast<std::int64_t>(std::llround(origin));
    phase = origin - static_cast<double>(base);
    const double rounding = 4 * std::numeric_limits<double>::epsilon() *
                            std::max(1.0, std::abs(origin));
    if (std::abs(phase) <= rounding) phase = 0;
  }

  [[nodiscard]] double coordinate(double value) const noexcept {
    return value / resolution - phase;
  }
  [[nodiscard]] std::int64_t index(double value) const noexcept {
    return static_cast<std::int64_t>(std::floor(coordinate(value))) - base;
  }

  double resolution;
  std::int64_t base;
  double phase;
};

}  // namespace nav_kernel::local
