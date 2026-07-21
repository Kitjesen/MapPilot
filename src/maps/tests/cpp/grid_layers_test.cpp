#include "lingtu/maps/layers/exploration.hpp"
#include "lingtu/maps/layers/grid.hpp"

#include <cassert>
#include <cmath>
#include <vector>

using namespace lingtu::maps::layers;

static void testElevationUsesRowYColX() {
  const std::vector<float> xyz = {
      2.0f, 1.0f, 0.5f,
      2.0f, 1.0f, 1.5f,
  };

  auto result = buildElevationMap(xyz, 0.0, 0.0, 1.0, 5.0, -1.0, 3.0);

  assert(result.maxZ.rows == 10);
  assert(result.maxZ.cols == 10);
  assert(result.valid[static_cast<size_t>(6 * 10 + 7)] == 1);
  assert(result.valid[static_cast<size_t>(7 * 10 + 6)] == 0);
  assert(result.minZ.data[static_cast<size_t>(6 * 10 + 7)] == 0.5f);
  assert(result.maxZ.data[static_cast<size_t>(6 * 10 + 7)] == 1.5f);
  assert(result.clearance.data[static_cast<size_t>(6 * 10 + 7)] == 1.0f);
}

static void testEsdfGradientAxesMatchXY() {
  Grid2D occ = makeGrid2D(5, 5, 1.0, 0.0, 0.0, 0.0f);
  for (int row = 0; row < 5; ++row) {
    occ.data[static_cast<size_t>(occ.index(row, 0))] = 100.0f;
  }

  auto esdf = computeEsdf(occ, 50.0f);

  assert(std::fabs(esdf.gradX.data[static_cast<size_t>(esdf.gradX.index(2, 2))]) > 0.5f);
  assert(std::fabs(esdf.gradY.data[static_cast<size_t>(esdf.gradY.index(2, 2))]) < 0.1f);
  assert(esdf.distance.data[static_cast<size_t>(esdf.distance.index(2, 2))] > 0.0f);
  assert(esdf.distance.data[static_cast<size_t>(esdf.distance.index(2, 0))] < 0.0f);
}

static void testTerrainRiskCombinesSlopeStepAndRoughness() {
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

  assert(risk.risk.data[static_cast<size_t>(risk.risk.index(1, 1))] >= 99.0f);
  assert(risk.stepHeight.data[static_cast<size_t>(risk.stepHeight.index(1, 1))] >= 0.39f);
  assert(risk.roughness.data[static_cast<size_t>(risk.roughness.index(1, 1))] > 0.0f);
}

static void testFusedCostPreservesHardCellsAndUsesRiskLayers() {
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

  assert(fused.data[static_cast<size_t>(fused.index(0, 0))] == 100.0f);
  assert(fused.data[static_cast<size_t>(fused.index(1, 1))] == 100.0f);
  assert(fused.data[static_cast<size_t>(fused.index(1, 2))] >= 37.0f);
  assert(fused.data[static_cast<size_t>(fused.index(2, 2))] == 80.0f);
}

static void testProjectedCollisionStopsOnHardCell() {
  Grid2D cost = makeGrid2D(5, 5, 1.0, -2.0, -2.0, 0.0f);
  cost.data[static_cast<size_t>(cost.index(2, 3))] = 100.0f;

  CmdVelCollisionParams params;
  params.horizonS = 1.0;
  params.stepS = 0.5;
  params.stopCost = 99.0f;

  const auto result =
      projectCmdVelCollision(cost, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, params);

  assert(result.action == 2);
  assert(result.reason == 2);
  assert(result.maxCost >= 99.0f);
}

static void testProjectedCollisionSlowsOnSoftCell() {
  Grid2D cost = makeGrid2D(5, 5, 1.0, -2.0, -2.0, 0.0f);
  cost.data[static_cast<size_t>(cost.index(2, 3))] = 70.0f;

  CmdVelCollisionParams params;
  params.horizonS = 1.0;
  params.stepS = 0.5;
  params.stopCost = 99.0f;
  params.slowCost = 60.0f;

  const auto result =
      projectCmdVelCollision(cost, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, params);

  assert(result.action == 1);
  assert(result.reason == 1);
  assert(result.maxCost == 70.0f);
}

static void testExplorationEncodingPreservesUnknownSpace() {
  Grid2D cost = makeGrid2D(2, 3, 0.2, -1.0, -1.0, 0.0f);
  cost.data = {0.0F, 20.0F, 65.0F, 100.0F, 0.0F, 90.0F};
  const std::vector<std::uint8_t> observed = {1, 1, 1, 1, 0, 0};

  const auto encoded = encodeExplorationOccupancy(cost, observed);

  assert((encoded == std::vector<std::int8_t>{0, 0, 100, 100, -1, -1}));
}

int main() {
  testElevationUsesRowYColX();
  testEsdfGradientAxesMatchXY();
  testTerrainRiskCombinesSlopeStepAndRoughness();
  testFusedCostPreservesHardCellsAndUsesRiskLayers();
  testProjectedCollisionStopsOnHardCell();
  testProjectedCollisionSlowsOnSoftCell();
  testExplorationEncodingPreservesUnknownSpace();
  return 0;
}
