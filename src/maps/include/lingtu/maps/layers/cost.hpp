#pragma once

#include "lingtu/maps/layers/grid.hpp"

namespace lingtu::maps::layers {

struct TraversabilityParams {
  float lethal = 100.0f;
  float inscribed = 99.0f;
  float maxSlopeDeg = 35.0f;
  float softSlopeStartDeg = 3.0f;
  float safeDistance = 1.5f;
  float proximityCap = 50.0f;
};

inline void requireSameGeometry(const Grid2D& reference, const Grid2D& layer, const char* name) {
  if (!layer.empty() && !sameGeometry(reference, layer)) {
    throw std::invalid_argument(std::string(name) + ": geometry does not match costmap");
  }
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

}  // namespace lingtu::maps::layers
