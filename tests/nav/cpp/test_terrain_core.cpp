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
      for (const float dx : {-0.06f, 0.06f}) {
        for (const float dy : {-0.06f, 0.06f}) {
          appendPoint(scan, ix * 0.2f + dx, iy * 0.2f + dy, -0.5f);
        }
      }
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
      for (const float dx : {-0.06f, 0.06f}) {
        for (const float dy : {-0.06f, 0.06f}) {
          appendPoint(scan, x + dx, iy * 0.2f + dy, z + 0.32f * dx);
        }
      }
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

TEST(TerrainCore, RobustGroundRejectsSparseLowGhostAndKeepsObstacleHeight) {
  TerrainParams p = smallTerrainParams();
  p.checkTerrainConnectivity = true;
  p.terrainUnderVehicle = -0.5;
  TerrainAnalysisCore terrain(p);
  terrain.updateVehicle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  std::vector<float> scan;
  for (int x = -3; x <= 3; ++x) {
    for (int y = -3; y <= 3; ++y) {
      appendPoint(scan, x * 0.06f, y * 0.06f, -0.5f);
    }
  }
  appendPoint(scan, 0.0f, 0.0f, -0.9f);
  appendPoint(scan, 0.12f, 0.12f, 0.2f);
  const TerrainResult result = terrain.process(scan.data(), static_cast<int>(scan.size() / 4), 0.0);
  const int center = result.map_width * p.planarVoxelHalfWidth + p.planarVoxelHalfWidth;
  ASSERT_TRUE(std::isfinite(result.elevation_map[center]));
  EXPECT_NEAR(result.elevation_map[center], -0.5f, 0.01f);
  bool kept_obstacle = false;
  for (std::size_t i = 0; i < result.terrain_points.size(); i += 4) {
    EXPECT_GT(result.terrain_points[i + 2], -0.8f);
    if (result.terrain_points[i + 2] > 0.19f) {
      kept_obstacle = true;
      EXPECT_NEAR(result.terrain_points[i + 3], 0.7f, 0.02f);
    }
  }
  EXPECT_TRUE(kept_obstacle);
}

TEST(TerrainCore, FittedRampUsesPointPositionAndCorrectGridAxes) {
  TerrainParams p = smallTerrainParams();
  p.planarVoxelSize = 0.25;
  p.planarVoxelHalfWidth = 4;
  p.checkTerrainConnectivity = true;
  p.terrainUnderVehicle = -0.5;
  TerrainAnalysisCore terrain(p);
  terrain.updateVehicle(2.0, -1.0, 0.0, 0.0, 0.0, 0.0);

  std::vector<float> scan;
  for (int x = -2; x <= 2; ++x) {
    for (int y = -2; y <= 2; ++y) {
      for (const float dx : {-0.07f, 0.07f}) {
        for (const float dy : {-0.07f, 0.07f}) {
          const float rx = x * 0.25f + dx;
          const float ry = y * 0.25f + dy;
          appendPoint(scan, 2.0f + rx, -1.0f + ry, -0.5f + 0.30f * rx + 0.15f * ry);
        }
      }
    }
  }
  const TerrainResult result = terrain.process(scan.data(), static_cast<int>(scan.size() / 4), 0.0);
  ASSERT_EQ(result.n_points, static_cast<int>(scan.size() / 4));
  for (std::size_t i = 3; i < result.terrain_points.size(); i += 4) {
    EXPECT_NEAR(result.terrain_points[i], 0.0f, 1e-5f);
  }
  const int cell = result.map_width * (p.planarVoxelHalfWidth + 1) + p.planarVoxelHalfWidth - 1;
  EXPECT_NEAR(result.elevation_map[cell], -0.4625f, 1e-5f);
}

TEST(TerrainCore, NeighborEvidenceDoesNotFillUnobservedSupportHole) {
  TerrainParams p = smallTerrainParams();
  p.checkTerrainConnectivity = true;
  p.terrainUnderVehicle = -0.5;
  TerrainAnalysisCore terrain(p);
  terrain.updateVehicle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  std::vector<float> scan;
  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      if (x == 0 && y == 0) continue;
      for (const float dx : {-0.06f, 0.06f}) {
        for (const float dy : {-0.06f, 0.06f}) {
          appendPoint(scan, x * 0.5f + dx, y * 0.5f + dy, -0.5f);
        }
      }
    }
  }
  const TerrainResult result = terrain.process(scan.data(), static_cast<int>(scan.size() / 4), 0.0);
  const int center = result.map_width * p.planarVoxelHalfWidth + p.planarVoxelHalfWidth;
  EXPECT_GT(result.connected_cells, 0);
  EXPECT_TRUE(std::isnan(result.elevation_map[center]));
  EXPECT_EQ(result.connectivity_map[center], 0);
}

TEST(TerrainCore, IsolatedObstacleRemainsEvidenceWithoutInventingGround) {
  TerrainParams p = smallTerrainParams();
  p.checkTerrainConnectivity = true;
  p.terrainUnderVehicle = -0.5;
  TerrainAnalysisCore terrain(p);
  terrain.updateVehicle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  const std::vector<float> scan{0.0f, 0.0f, 0.3f, 0.0f};
  const TerrainResult result = terrain.process(scan.data(), 1, 0.0);
  const int center = result.map_width * p.planarVoxelHalfWidth + p.planarVoxelHalfWidth;
  EXPECT_EQ(result.connected_cells, 0);
  EXPECT_TRUE(std::isnan(result.elevation_map[center]));
  ASSERT_EQ(result.n_points, 1);
  EXPECT_FLOAT_EQ(result.terrain_points[2], 0.3f);
  EXPECT_GE(result.terrain_points[3], p.obstacleHeightThre);
}

TEST(TerrainCore, ObservedFloorOutsideLidarBlindRegionSeedsConnectivity) {
  TerrainParams p = smallTerrainParams();
  p.planarVoxelSize = 0.20;
  p.planarVoxelHalfWidth = 15;
  p.terrainVoxelHalfWidth = 4;
  p.checkTerrainConnectivity = true;
  p.terrainUnderVehicle = -0.5;
  EXPECT_EQ(p.groundSeedSearchRadiusCells, 10);
  TerrainAnalysisCore terrain(p);
  terrain.updateVehicle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  std::vector<float> scan;
  for (int x = -10; x <= 10; ++x) {
    for (int y = -10; y <= 10; ++y) {
      for (const float dx : {-0.06f, 0.06f}) {
        for (const float dy : {-0.06f, 0.06f}) {
          const float px = x * 0.20f + dx;
          const float py = y * 0.20f + dy;
          const double radius = std::hypot(px, py);
          if (radius >= 1.3 && radius <= 2.0) appendPoint(scan, px, py, -0.5f);
        }
      }
    }
  }
  const TerrainResult result = terrain.process(scan.data(), static_cast<int>(scan.size() / 4), 0.0);
  const int center = result.map_width * p.planarVoxelHalfWidth + p.planarVoxelHalfWidth;
  EXPECT_GT(result.connected_cells, 0);
  EXPECT_TRUE(std::isnan(result.elevation_map[center]));
  EXPECT_EQ(result.connectivity_map[center], 0);
  EXPECT_GT(result.n_points, 0);
}

TEST(TerrainCore, FiveCentimeterGridFitsObservedFloorAndPreservesFineHole) {
  TerrainParams p = smallTerrainParams();
  p.planarVoxelSize = 0.05;
  p.planarVoxelHalfWidth = 10;
  p.checkTerrainConnectivity = true;
  p.terrainUnderVehicle = -0.5;
  TerrainAnalysisCore terrain(p);
  terrain.updateVehicle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  std::vector<float> scan;
  for (int x = -8; x <= 8; ++x) {
    for (int y = -8; y <= 8; ++y) {
      if (x == 0 && y == 0) continue;
      const float px = x * 0.05f;
      const float py = y * 0.05f;
      appendPoint(scan, px, py, -0.5f + 0.20f * px - 0.10f * py);
    }
  }
  const TerrainResult result = terrain.process(scan.data(), static_cast<int>(scan.size() / 4), 0.0);
  const int center = result.map_width * p.planarVoxelHalfWidth + p.planarVoxelHalfWidth;
  EXPECT_TRUE(std::isnan(result.elevation_map[center]));
  EXPECT_EQ(result.connectivity_map[center], 0);
  EXPECT_EQ(result.connected_cells, static_cast<int>(scan.size() / 4));
  ASSERT_EQ(result.n_points, static_cast<int>(scan.size() / 4));
  for (std::size_t i = 3; i < result.terrain_points.size(); i += 4) {
    EXPECT_NEAR(result.terrain_points[i], 0.0f, 1e-5f);
  }
  const int observed = result.map_width * (p.planarVoxelHalfWidth + 1) + p.planarVoxelHalfWidth - 1;
  EXPECT_NEAR(result.elevation_map[observed], -0.485f, 1e-5f);
}
