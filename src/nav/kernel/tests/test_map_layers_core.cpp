#include "nav_kernel/map_layers_core.hpp"

#include <cmath>
#include <gtest/gtest.h>

using namespace nav_kernel;

TEST(MapLayersCore, ElevationUsesRowYColX) {
  const std::vector<float> xyz = {
    2.0f, 1.0f, 0.5f,
    2.0f, 1.0f, 1.5f,
  };

  auto result = buildElevationMap(xyz, 0.0, 0.0, 1.0, 5.0, -1.0, 3.0);

  ASSERT_EQ(result.maxZ.rows, 10);
  ASSERT_EQ(result.maxZ.cols, 10);
  // origin=(-5,-5), res=1: x=2 -> col 7, y=1 -> row 6.
  EXPECT_EQ(result.valid[static_cast<size_t>(6 * 10 + 7)], 1);
  EXPECT_EQ(result.valid[static_cast<size_t>(7 * 10 + 6)], 0);
  EXPECT_FLOAT_EQ(result.minZ.data[static_cast<size_t>(6 * 10 + 7)], 0.5f);
  EXPECT_FLOAT_EQ(result.maxZ.data[static_cast<size_t>(6 * 10 + 7)], 1.5f);
  EXPECT_FLOAT_EQ(result.clearance.data[static_cast<size_t>(6 * 10 + 7)], 1.0f);
}

TEST(MapLayersCore, EsdfGradientAxesMatchXY) {
  Grid2D occ = makeGrid2D(5, 5, 1.0, 0.0, 0.0, 0.0f);
  for (int row = 0; row < 5; ++row) {
    occ.data[static_cast<size_t>(occ.index(row, 0))] = 100.0f;
  }

  auto esdf = computeEsdf(occ, 50.0f);

  EXPECT_GT(std::fabs(esdf.gradX.data[static_cast<size_t>(esdf.gradX.index(2, 2))]), 0.5f);
  EXPECT_LT(std::fabs(esdf.gradY.data[static_cast<size_t>(esdf.gradY.index(2, 2))]), 0.1f);
  EXPECT_GT(esdf.distance.data[static_cast<size_t>(esdf.distance.index(2, 2))], 0.0f);
  EXPECT_LT(esdf.distance.data[static_cast<size_t>(esdf.distance.index(2, 0))], 0.0f);
}

TEST(MapLayersCore, TerrainRiskCombinesSlopeStepAndRoughness) {
  ElevationMapResult elevation;
  elevation.minZ = makeGrid2D(3, 3, 1.0, 0.0, 0.0, 0.0f);
  elevation.maxZ = makeGrid2D(3, 3, 1.0, 0.0, 0.0, 0.0f);
  elevation.clearance = makeGrid2D(3, 3, 1.0, 0.0, 0.0, 0.0f);
  elevation.valid.assign(9, 1);

  elevation.maxZ.data[static_cast<size_t>(elevation.maxZ.index(1, 1))] = 0.4f;

  TerrainRiskParams params;
  params.maxSlopeDeg = 30.0f;
  params.criticalStepM = 0.25f;
  params.roughnessCriticalM = 0.1f;

  auto risk = computeTerrainRisk(elevation, params);

  EXPECT_GE(risk.risk.data[static_cast<size_t>(risk.risk.index(1, 1))], 99.0f);
  EXPECT_GE(risk.stepHeight.data[static_cast<size_t>(risk.stepHeight.index(1, 1))], 0.39f);
  EXPECT_GT(risk.roughness.data[static_cast<size_t>(risk.roughness.index(1, 1))], 0.0f);
}

TEST(MapLayersCore, FusedCostPreservesHardCellsAndUsesRiskLayers) {
  Grid2D cost = makeGrid2D(3, 3, 1.0, 0.0, 0.0, 0.0f);
  cost.data[static_cast<size_t>(cost.index(0, 0))] = 100.0f;

  Grid2D slope = makeGrid2D(3, 3, 1.0, 0.0, 0.0, 0.0f);
  slope.data[static_cast<size_t>(slope.index(1, 1))] = 40.0f;

  Grid2D esdf = makeGrid2D(3, 3, 1.0, 0.0, 0.0, 2.0f);
  esdf.data[static_cast<size_t>(esdf.index(1, 2))] = 0.25f;

  Grid2D terrain = makeGrid2D(3, 3, 1.0, 0.0, 0.0, 0.0f);
  terrain.data[static_cast<size_t>(terrain.index(2, 2))] = 80.0f;

  TraversabilityParams params;
  params.maxSlopeDeg = 35.0f;
  params.safeDistance = 1.0f;
  params.proximityCap = 50.0f;

  auto fused = fuseTraversabilityCost(cost, slope, esdf, terrain, params);

  EXPECT_FLOAT_EQ(fused.data[static_cast<size_t>(fused.index(0, 0))], 100.0f);
  EXPECT_FLOAT_EQ(fused.data[static_cast<size_t>(fused.index(1, 1))], 100.0f);
  EXPECT_GE(fused.data[static_cast<size_t>(fused.index(1, 2))], 37.0f);
  EXPECT_FLOAT_EQ(fused.data[static_cast<size_t>(fused.index(2, 2))], 80.0f);
}
