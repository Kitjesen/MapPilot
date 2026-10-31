#pragma once

#include "lingtu/maps/layers/elevation.hpp"

namespace lingtu::maps::layers {

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
          static_cast<float>(180.0 / 3.14159265358979323846);
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

}  // namespace lingtu::maps::layers
