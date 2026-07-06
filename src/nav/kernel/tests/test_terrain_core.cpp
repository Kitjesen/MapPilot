#include <gtest/gtest.h>

#include "nav_kernel/terrain_core.hpp"

#include <vector>

using namespace nav_kernel;

namespace {

TerrainParams smallTerrainParams() {
  TerrainParams p;
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
