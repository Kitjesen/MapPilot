#pragma once

#include <algorithm>
#include <cmath>

namespace lingtu::nav::endpoint {

// Convert a terrain metric into the shared 0..100 traversability contract.
// Values in [40, 80) request a slowdown and the hard threshold maps to 100 so
// producer thresholds match the endpoint's soft/hard decision semantics.
inline float gradedTerrainRiskCost(
    double value,
    double soft_threshold,
    double hard_threshold) {
  if (!std::isfinite(value) || !std::isfinite(soft_threshold) ||
      !std::isfinite(hard_threshold) || soft_threshold < 0.0 ||
      hard_threshold <= soft_threshold) {
    return 100.0F;
  }
  if (value < soft_threshold) {
    return 0.0F;
  }
  if (value >= hard_threshold) {
    return 100.0F;
  }
  const double ratio = std::clamp(
      (value - soft_threshold) / (hard_threshold - soft_threshold),
      0.0,
      1.0);
  return static_cast<float>(40.0 + 40.0 * ratio);
}

inline float terrainHeightRiskCost(
    double height_m,
    double soft_height_m,
    double hard_height_m) {
  return gradedTerrainRiskCost(height_m, soft_height_m, hard_height_m);
}

inline float terrainSlopeRiskCost(
    double slope_deg,
    double soft_slope_deg,
    double hard_slope_deg) {
  return gradedTerrainRiskCost(slope_deg, soft_slope_deg, hard_slope_deg);
}

}  // namespace lingtu::nav::endpoint
