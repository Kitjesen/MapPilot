#include <gtest/gtest.h>

#include "nav_kernel/terrain_core.hpp"

#include <chrono>
#include <cmath>
#include <vector>

using namespace nav_kernel;

namespace {

TerrainParams smallTerrainParams() {
  TerrainParams p;
  EXPECT_EQ(p.workerThreads, 2);
  p.terrainVoxelHalfWidth = 2;
  p.planarVoxelHalfWidth = 2;
  p.terrainVoxelSize = 1.0;
  p.planarVoxelSize = 0.5;
  p.decayTime = 100.0;
  p.noDecayDis = 0.25;
  p.voxelPointUpdateThre = 1000;
  p.voxelTimeUpdateThre = 100.0;
  p.minRelZ = -1.0;
  p.maxRelZ = 1.0;
  p.disRatioZ = 0.0;
  p.minBlockPointNum = 1;
  p.vehicleHeight = 1.5;
  return p;
}

}  // namespace

TEST(TerrainCore, ClearDynamicObstacleDropsStaleUnconfirmedVoxel) {
  TerrainParams p = smallTerrainParams();
  p.clearDyObs = true;
  p.minDyObsPointNum = 1;
  p.minDyObsDis = 0.3;
  p.minDyObsAngle = 0.0;
  p.minDyObsRelZ = -0.5;
  p.absDyObsRelZThre = 0.2;
  p.minDyObsVFOV = -16.0;
  p.maxDyObsVFOV = 16.0;

  TerrainAnalysisCore terrain(p);
  terrain.updateVehicle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  std::vector<float> first_scan = {
      1.0f, 0.0f, 0.0f, 0.0f,
  };
  TerrainResult first = terrain.process(first_scan.data(), 1, 0.0);
  EXPECT_GT(first.n_points, 0);

  std::vector<float> empty_scan;
  TerrainResult second = terrain.process(empty_scan.data(), 0, 0.1);
  EXPECT_EQ(second.n_points, 0);
}

TEST(TerrainCore, NoDataObstacleEmitsSyntheticBlockingCellsAfterMotion) {
  TerrainParams p = smallTerrainParams();
  p.noDataObstacle = true;
  p.noDataBlockSkipNum = 0;
  p.minBlockPointNum = 1;
  p.vehicleHeight = 1.25;

  TerrainAnalysisCore terrain(p);
  terrain.updateVehicle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  std::vector<float> empty_scan;
  TerrainResult before_motion = terrain.process(empty_scan.data(), 0, 0.0);
  EXPECT_EQ(before_motion.n_points, 0);

  terrain.updateVehicle(0.5, 0.0, 0.0, 0.0, 0.0, 0.0);
  TerrainResult after_motion = terrain.process(empty_scan.data(), 0, 0.1);
  EXPECT_GT(after_motion.n_points, 0);
  ASSERT_GE(after_motion.terrain_points.size(), 4u);
  EXPECT_FLOAT_EQ(after_motion.terrain_points[3], 1.25f);
}

void appendPoint(
    std::vector<float>& scan,
    float x,
    float y,
    float z,
    float intensity = 1.0f) {
  scan.insert(scan.end(), {x, y, z, intensity});
}

TEST(TerrainCore, RepeatedStationaryScansKeepRollingStorageBounded) {
  TerrainParams p = smallTerrainParams();
  p.terrainVoxelHalfWidth = 6;
  p.planarVoxelHalfWidth = 30;
  p.planarVoxelSize = 0.2;
  p.scanVoxelSize = 0.05;
  p.maxPointsPerVoxel = 192;
  p.maxStoredPoints = 200;
  p.noDecayDis = 4.0;
  p.voxelPointUpdateThre = 1;
  p.voxelTimeUpdateThre = 0.0;

  TerrainAnalysisCore terrain(p);
  terrain.updateVehicle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  std::vector<float> scan;
  scan.reserve(4000 * 4);
  for (int i = 0; i < 4000; ++i) {
    const float angle = static_cast<float>(i) * 0.0174532925f;
    const float radius = 0.5f + static_cast<float>(i % 60) * 0.08f;
    scan.push_back(std::cos(angle) * radius);
    scan.push_back(std::sin(angle) * radius);
    scan.push_back(static_cast<float>(i % 7) * 0.04f);
    scan.push_back(1.0f);
  }

  for (int frame = 0; frame < 120; ++frame) {
    terrain.process(scan.data(), static_cast<int>(scan.size() / 4), frame * 0.1);
  }

  EXPECT_LE(terrain.storedPointCount(), p.maxStoredPoints);

  const auto start = std::chrono::steady_clock::now();
  const TerrainResult result =
      terrain.process(scan.data(), static_cast<int>(scan.size() / 4), 12.1);
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - start)
                                .count();
  EXPECT_LE(result.terrain_points.size() / 4, terrain.storedPointCount());
  EXPECT_LT(elapsed_ms, 250.0);
}

TEST(TerrainCore, CompactionPreservesHighObstacleEvidence) {
  TerrainParams p = smallTerrainParams();
  p.scanVoxelSize = 0.01;
  p.maxPointsPerVoxel = 1;
  p.maxStoredPoints = 25;
  p.voxelPointUpdateThre = 1;
  p.voxelTimeUpdateThre = 0.0;

  TerrainAnalysisCore terrain(p);
  terrain.updateVehicle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  std::vector<float> scan;
  for (int i = 0; i < 100; ++i) {
    scan.insert(scan.end(), {
        0.001f * static_cast<float>(i % 10),
        0.001f * static_cast<float>(i / 10),
        0.0f,
        1.0f,
    });
  }
  scan.insert(scan.end(), {0.005f, 0.005f, 0.8f, 1.0f});

  const TerrainResult result =
      terrain.process(scan.data(), static_cast<int>(scan.size() / 4), 0.0);
  float max_z = -1000.0f;
  for (std::size_t i = 0; i < result.terrain_points.size() / 4; ++i) {
    max_z = std::max(max_z, result.terrain_points[i * 4 + 2]);
  }
  EXPECT_GE(max_z, 0.79f);
  EXPECT_LE(terrain.storedPointCount(), p.maxStoredPoints);
}

TEST(TerrainCore, ConnectivityRejectsDisconnectedOverheadSurface) {
  TerrainParams p = smallTerrainParams();
  p.planarVoxelHalfWidth = 10;
  p.terrainVoxelHalfWidth = 5;
  p.planarVoxelSize = 0.25;
  p.terrainVoxelSize = 1.0;
  p.minRelZ = -1.5;
  p.maxRelZ = 3.5;
  p.vehicleHeight = 4.0;
  p.checkTerrainConnectivity = true;
  p.terrainUnderVehicle = -0.5;
  p.terrainConnectionHeight = 0.35;
  p.ceilingFilteringHeight = 1.5;
  p.terrainConnectivityRadiusCells = 2;
  p.groundSeedSearchRadiusCells = 2;
  p.maxGroundSeedError = 0.5;

  TerrainAnalysisCore terrain(p);
  terrain.updateVehicle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  std::vector<float> scan;
  for (int ix = -4; ix <= 4; ++ix) {
    for (int iy = -4; iy <= 4; ++iy) {
      appendPoint(scan, ix * 0.2f, iy * 0.2f, -0.5f);
    }
  }
  for (int ix = 7; ix <= 9; ++ix) {
    for (int iy = -1; iy <= 1; ++iy) {
      appendPoint(scan, ix * 0.25f, iy * 0.25f, 2.3f);
    }
  }

  const TerrainResult result = terrain.process(
      scan.data(), static_cast<int>(scan.size() / 4), 0.0);

  EXPECT_GT(result.connected_cells, 0);
  ASSERT_EQ(
      result.connectivity_map.size(),
      static_cast<std::size_t>(result.map_width * result.map_width));
  for (std::size_t i = 0; i < result.terrain_points.size() / 4; ++i) {
    EXPECT_LT(result.terrain_points[i * 4 + 2], 2.0f);
  }
}

TEST(TerrainCore, ConnectivityKeepsGradualGroundRamp) {
  TerrainParams p = smallTerrainParams();
  p.planarVoxelHalfWidth = 12;
  p.terrainVoxelHalfWidth = 5;
  p.planarVoxelSize = 0.25;
  p.terrainVoxelSize = 1.0;
  p.minRelZ = -1.5;
  p.maxRelZ = 1.5;
  p.vehicleHeight = 2.0;
  p.checkTerrainConnectivity = true;
  p.terrainUnderVehicle = -0.5;
  p.terrainConnectionHeight = 0.30;
  p.ceilingFilteringHeight = 1.5;
  p.terrainConnectivityRadiusCells = 2;
  p.groundSeedSearchRadiusCells = 2;
  p.maxGroundSeedError = 0.5;

  TerrainAnalysisCore terrain(p);
  terrain.updateVehicle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  std::vector<float> scan;
  for (int step = 0; step <= 8; ++step) {
    const float x = step * 0.25f;
    const float z = -0.5f + step * 0.08f;
    for (int iy = -2; iy <= 2; ++iy) {
      appendPoint(scan, x, iy * 0.2f, z);
    }
  }

  const TerrainResult result = terrain.process(
      scan.data(), static_cast<int>(scan.size() / 4), 0.0);

  EXPECT_GT(result.connected_cells, 0);
  bool retained_far_ramp = false;
  for (std::size_t i = 0; i < result.terrain_points.size() / 4; ++i) {
    retained_far_ramp = retained_far_ramp || result.terrain_points[i * 4] > 1.75f;
  }
  EXPECT_TRUE(retained_far_ramp);
}
