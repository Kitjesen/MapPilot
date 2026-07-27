#pragma once

#include <algorithm>
#include <cmath>

namespace lingtu::nav::endpoint {

inline int inflationSearchRadiusCells(double radius_m, double resolution_m) {
  if (!std::isfinite(radius_m) || !std::isfinite(resolution_m) || radius_m <= 0.0 ||
      resolution_m <= 0.0) {
    return 0;
  }
  return std::max(0, static_cast<int>(std::ceil(radius_m / resolution_m)));
}

inline bool cellCenterWithinInflationRadius(int row_offset, int column_offset, double resolution_m,
                                            double radius_m) {
  if (row_offset == 0 && column_offset == 0) {
    return true;
  }
  if (!std::isfinite(radius_m) || !std::isfinite(resolution_m) || radius_m <= 0.0 ||
      resolution_m <= 0.0) {
    return false;
  }
  const double row_m = static_cast<double>(row_offset) * resolution_m;
  const double column_m = static_cast<double>(column_offset) * resolution_m;
  return row_m * row_m + column_m * column_m <= radius_m * radius_m + 1e-12;
}

}  // namespace lingtu::nav::endpoint
