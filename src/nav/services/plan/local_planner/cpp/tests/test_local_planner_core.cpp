#include <gtest/gtest.h>
#include "local_planner.hpp"
#include "local_planner_scoring.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace nav_kernel;

static std::filesystem::path writeMinimalPlannerPaths(const std::string& name) {
  auto dir = std::filesystem::temp_directory_path() / name;
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  {
    std::ofstream f(dir / "startPaths.ply");
    f << "ply\nformat ascii 1.0\nelement vertex " << kGroupNum << "\nend_header\n";
    for (int g = 0; g < kGroupNum; g++) {
      f << "1 0 0 " << g << "\n";
    }
  }

  {
    std::ofstream f(dir / "pathList.ply");
    f << "ply\nformat ascii 1.0\nelement vertex " << kPathNum << "\nend_header\n";
    for (int i = 0; i < kPathNum; i++) {
      f << "1 0 0 " << i << " 3\n";
    }
  }

  {
    std::ofstream f(dir / "correspondences.txt");
    for (int i = 0; i < 161 * 451; i++) {
      f << i << " -1\n";
    }
  }

  return dir;
}

static std::filesystem::path writePlannerPathsWithStartPoint(
    const std::string& name,
    double start_x) {
  auto dir = std::filesystem::temp_directory_path() / name;
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  {
    std::ofstream f(dir / "startPaths.ply");
    f << "ply\nformat ascii 1.0\nelement vertex " << kGroupNum << "\nend_header\n";
    for (int g = 0; g < kGroupNum; g++) {
      f << start_x << " 0 0 " << g << "\n";
    }
  }

  {
    std::ofstream f(dir / "pathList.ply");
    f << "ply\nformat ascii 1.0\nelement vertex " << kPathNum << "\nend_header\n";
    for (int i = 0; i < kPathNum; i++) {
      f << "1 0 0 " << i << " 3\n";
    }
  }

  {
    std::ofstream f(dir / "correspondences.txt");
    for (int i = 0; i < 161 * 451; i++) {
      f << i << " -1\n";
    }
  }

  return dir;
}

// 閳光偓閳光偓 RotLUT 閳光偓閳光偓

TEST(RotLUT, TablesCorrect) {
  const auto& lut = rotLUT();
  // rotDir=18 閳?0鎺?
  EXPECT_NEAR(lut.c[18], 1.0, 1e-10);
  EXPECT_NEAR(lut.s[18], 0.0, 1e-10);
  // rotDir=0 閳?-180鎺?
  EXPECT_NEAR(lut.c[0], -1.0, 1e-10);
  EXPECT_NEAR(lut.s[0], 0.0, 1e-6);
  // rotDir=9 閳?-90鎺?
  EXPECT_NEAR(lut.c[9], 0.0, 1e-6);
  EXPECT_NEAR(lut.s[9], -1.0, 1e-10);
  // rotDir=27 閳?+90鎺?
  EXPECT_NEAR(lut.c[27], 0.0, 1e-6);
  EXPECT_NEAR(lut.s[27], 1.0, 1e-10);
}

// 閳光偓閳光偓 angDiffDeg 閳光偓閳光偓

TEST(AngDiffDeg, SameAngle) {
  EXPECT_DOUBLE_EQ(angDiffDeg(45.0, 45.0), 0.0);
}

TEST(AngDiffDeg, OppositeAngles) {
  EXPECT_DOUBLE_EQ(angDiffDeg(0.0, 180.0), 180.0);
}

TEST(AngDiffDeg, SmallDiff) {
  EXPECT_NEAR(angDiffDeg(10.0, 20.0), 10.0, 1e-10);
}

TEST(AngDiffDeg, WrapAround) {
  // 350鎺?vs 10鎺?閳?20鎺?difference
  EXPECT_NEAR(angDiffDeg(350.0, 10.0), 20.0, 1e-10);
}

TEST(AngDiffDeg, MultipleTurns) {
  EXPECT_NEAR(angDiffDeg(0.0, 720.0), 0.0, 1e-10);
  EXPECT_NEAR(angDiffDeg(-450.0, 90.0), 180.0, 1e-10);
}

// 閳光偓閳光偓 worldToVoxel 閳光偓閳光偓

TEST(VoxelGridParams, DefaultsMatchPlannerGrid) {
  VoxelGridParams g;
  EXPECT_DOUBLE_EQ(g.gridVoxelOffsetY, 4.5);
  EXPECT_EQ(g.gridVoxelNumY, 451);
}

TEST(WorldToVoxel, CenterPoint) {
  VoxelGridParams g;
  int indX, indY;
  // 閸樼喓鍋ｉ梽鍕箮鎼存棁顕氶弰鐘茬殸閸掓壆缍夐弽闂磋厬韫囧啫灏崺?
  bool ok = worldToVoxel(1.6, 0.0, g, indX, indY);
  EXPECT_TRUE(ok);
  EXPECT_GE(indX, 0);
  EXPECT_LT(indX, g.gridVoxelNumX);
  EXPECT_GE(indY, 0);
  EXPECT_LT(indY, g.gridVoxelNumY);
}

TEST(WorldToVoxel, OutOfRange) {
  VoxelGridParams g;
  int indX, indY;
  // 瀵板牐绻欓惃鍕仯鎼存棁顕氱搾鍛毉缂冩垶鐗?
  bool ok = worldToVoxel(100.0, 100.0, g, indX, indY);
  EXPECT_FALSE(ok);
}

TEST(WorldToVoxel, NearZeroX) {
  VoxelGridParams g;
  int indX, indY;
  // x2 閹恒儴绻?gridVoxelOffsetX 閺?scaleY 閸欘垵鍏樺鍫濈毈
  bool ok = worldToVoxel(g.gridVoxelOffsetX, 0.0, g, indX, indY);
  // scaleY = x2/offsetX + searchRadius/offsetY * (offsetX-x2)/offsetX
  // = 1.0 + searchRadius/offsetY * 0 = 1.0 閳?濮濓絽鐖?
  EXPECT_TRUE(ok);
}

// 閳光偓閳光偓 scorePath 閳光偓閳光偓

TEST(ScorePath, PerfectAlignment) {
  PathScoreParams p;
  p.dirWeight = 0.02;
  // dirDiff=0 閳?dw=0 閳?sqrtSqrtDw=0 閳?(1-0)=1
  double s = scorePath(0.0, 10.0, 4.0, 0.0, 10.0, p);
  // 濮濓絽鐖跺Ο鈥崇础: 1.0 * 10^2 * 10^2 * 1.0 = 10000
  EXPECT_DOUBLE_EQ(s, 10000.0);
}

TEST(ScorePath, LargeAnglePenalty) {
  PathScoreParams p;
  p.dirWeight = 0.02;
  double s = scorePath(90.0, 10.0, 4.0, 0.0, 10.0, p);
  // dw = 0.02*90 = 1.8, sqrt(sqrt(1.8)) 閳?1.158
  // (1 - 1.158) < 0, but the formula can go negative
  EXPECT_LT(s, 0.0);  // C++ 閸樼喎顫愭禒锝囩垳閸忎浇顔忕拹鐔峰瀻
}

TEST(ScorePath, ModerateAngle) {
  PathScoreParams p;
  p.dirWeight = 0.02;
  // dirDiff=50鎺? dw=1.0, sqrt(sqrt(1.0))=1.0, (1-1)=0
  double s = scorePath(50.0, 10.0, 4.0, 0.0, 10.0, p);
  EXPECT_NEAR(s, 0.0, 1e-10);
}

TEST(ScorePath, DirWeightEffect) {
  PathScoreParams p1, p2;
  p1.dirWeight = 0.02;
  p2.dirWeight = 0.04;  // 閺囨潙宸遍惃鍕煙閸氭垶鍎电純?

  double s1 = scorePath(30.0, 10.0, 4.0, 0.0, 10.0, p1);
  double s2 = scorePath(30.0, 10.0, 4.0, 0.0, 10.0, p2);
  EXPECT_GT(s1, s2);  // 閺囨潙宸遍惃鍕劦缂?閳?閺囩繝缍嗛惃鍕瀻閺?
}

TEST(ScorePath, NearGoalUsesGroupDirW) {
  PathScoreParams p;
  p.dirWeight = 0.02;
  p.omniDirGoalThre = 5.0;
  // relativeGoalDis=3 < omniDirGoalThre=5 閳?娴ｈ法鏁?groupDirW
  double s = scorePath(0.0, 10.0, 4.0, 0.0, 3.0, p);
  // (1-0) * 4^2 * 1.0 = 16
  EXPECT_DOUBLE_EQ(s, 16.0);
}

TEST(ScorePath, FarGoalUsesRotDirW) {
  PathScoreParams p;
  p.dirWeight = 0.02;
  p.omniDirGoalThre = 5.0;
  // relativeGoalDis=10 > omniDirGoalThre=5 閳?娴ｈ法鏁?rotDirW^4
  double s = scorePath(0.0, 10.0, 4.0, 0.0, 10.0, p);
  // (1-0) * 10^4 * 1.0 = 10000
  EXPECT_DOUBLE_EQ(s, 10000.0);
}

TEST(ScorePath, TerrainPenalty) {
  PathScoreParams p;
  p.dirWeight = 0.02;
  p.slopeWeight = 3.0;

  double s_flat = scorePath(0.0, 10.0, 4.0, 0.0, 10.0, p);
  double s_slope = scorePath(0.0, 10.0, 4.0, 0.1, 10.0, p);
  // terrainFactor = max(0, 1 - 3*0.1) = 0.7
  EXPECT_NEAR(s_slope, s_flat * 0.7, 1e-6);
}

TEST(ScorePath, SteepTerrainClampedToZero) {
  PathScoreParams p;
  p.dirWeight = 0.02;
  p.slopeWeight = 5.0;
  // terrainPenalty=0.3 閳?factor = max(0, 1 - 5*0.3) = max(0, -0.5) = 0
  double s = scorePath(0.0, 10.0, 4.0, 0.3, 10.0, p);
  EXPECT_DOUBLE_EQ(s, 0.0);
}

// 閳光偓閳光偓 computeRotDirW 閳光偓閳光偓

TEST(RotDirW, Forward) {
  // rotDir=18 閳?0鎺?閳?|18-9|+1 = 10? No, 18 >= 18 閳?|18-27|+1 = 10
  EXPECT_DOUBLE_EQ(computeRotDirW(18), 10.0);
}

TEST(RotDirW, Backward) {
  // rotDir=0 閳?-180鎺?閳?|0-9|+1 = 10
  EXPECT_DOUBLE_EQ(computeRotDirW(0), 10.0);
}

TEST(RotDirW, SideLeft) {
  // rotDir=9 閳?-90鎺?閳?|9-9|+1 = 1
  EXPECT_DOUBLE_EQ(computeRotDirW(9), 1.0);
}

TEST(RotDirW, SideRight) {
  // rotDir=27 閳?+90鎺?閳?|27-27|+1 = 1
  EXPECT_DOUBLE_EQ(computeRotDirW(27), 1.0);
}

// 閳光偓閳光偓 computeGroupDirW 閳光偓閳光偓

TEST(GroupDirW, CenterGroup) {
  // pathGroup=3 閳?4 - |3-3| = 4
  EXPECT_DOUBLE_EQ(computeGroupDirW(3), 4.0);
}

TEST(GroupDirW, EdgeGroup) {
  // pathGroup=0 閳?4 - |0-3| = 1
  EXPECT_DOUBLE_EQ(computeGroupDirW(0), 1.0);
}

// 閳光偓閳光偓 selectBestGroup 閳光偓閳光偓

TEST(SelectBestGroup, FindsMaxScore) {
  int groupNum = 7;
  std::vector<double> scores(36 * groupNum, 0.0);
  // 鐠佸墽鐤?rotDir=18, group=3 (閸撳秵鏌熸稉顓炪亷) 娑撶儤娓舵妯哄瀻
  scores[18 * groupNum + 3] = 100.0;

  auto result = selectBestGroup(scores, groupNum, -180.0, 180.0, true, false);
  EXPECT_EQ(result.selectedGroupID, 18 * groupNum + 3);
  EXPECT_DOUBLE_EQ(result.maxScore, 100.0);
}

TEST(SelectBestGroup, NoPath) {
  int groupNum = 7;
  std::vector<double> scores(36 * groupNum, 0.0);  // 閸忋劑娴?

  auto result = selectBestGroup(scores, groupNum, -180.0, 180.0, true, false);
  EXPECT_EQ(result.selectedGroupID, -1);
}

TEST(SelectBestGroup, RotObstacleFilter) {
  int groupNum = 7;
  std::vector<double> scores(36 * groupNum, 0.0);
  // 閺堚偓妤傛ê鍨庨崷?rotDir=18 (0鎺?
  scores[18 * groupNum + 3] = 100.0;
  // 濞嗭繝鐝崚鍡楁躬 rotDir=15 (-30鎺?
  scores[15 * groupNum + 3] = 50.0;

  // 閺冨娴嗛梾婊咁暡閻椻晠妾洪崚? 閸欘亜鍘戠拋?[-40鎺? -20鎺砞 閳?0鎺?鐞氼偅甯撻梽?
  auto result = selectBestGroup(scores, groupNum, -40.0, -20.0, true, true);
  EXPECT_EQ(result.selectedGroupID, 15 * groupNum + 3);
  EXPECT_DOUBLE_EQ(result.maxScore, 50.0);
}

TEST(SelectBestGroup, RotationGateHelperExactlyMatchesSelection) {
  const std::vector<std::pair<double, double>> windows = {
      {-180.0, 180.0}, {0.0, 125.0}, {-125.0, 0.0}, {-10.0, 10.0}};
  for (bool check_obstacle : {false, true}) {
    for (bool two_way : {false, true}) {
      for (const auto& [cw, ccw] : windows) {
        for (int rotation = 0; rotation < kRotDirs; ++rotation) {
          std::vector<double> scores(kRotDirs * kGroupNum, 0.0);
          scores[rotation * kGroupNum + 3] = 1.0;
          const auto selected = selectBestGroup(
              scores, kGroupNum, cw, ccw, two_way, check_obstacle);
          EXPECT_EQ(selected.selectedGroupID >= 0,
                    rotationPassesObstacleGate(
                        rotation, cw, ccw, two_way, check_obstacle))
              << "rotation=" << rotation << " window=(" << cw << "," << ccw
              << ") two_way=" << two_way << " check=" << check_obstacle;
        }
      }
    }
  }
}

// 閳光偓閳光偓 LocalPlannerCore 閳光偓閳光偓

TEST(LocalPlannerCore, CheckRotObstacleFiltersPlanSelection) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_check_rot_obstacle_fixture");

  LocalPlannerParams p;
  EXPECT_FALSE(p.useCost);
  p.checkObstacle = false;
  p.checkRotObstacle = true;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.debugCandidateLimit = kRotDirs;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  const float angle = -10.0f * static_cast<float>(M_PI) / 180.0f;
  const float radius = 0.45f;
  std::vector<float> cloud = {
      radius * std::cos(angle), radius * std::sin(angle), 0.0f, 1.0f};

  auto result = planner.plan(cloud.data(), 1, 0.0);

  ASSERT_TRUE(result.pathFound);
  ASSERT_FALSE(result.path.empty());
  EXPECT_GT(result.path.front().y, 0.1);
  EXPECT_GT(result.path.front().x, 0.9);
  const auto snapshot = planner.debugSnapshot();
  ASSERT_TRUE(snapshot.valid);
  EXPECT_TRUE(std::any_of(
      snapshot.candidates.begin(),
      snapshot.candidates.end(),
      [](const LocalPlanCandidate& candidate) {
        return candidate.state == LocalCandidateState::RotationBlocked;
      }));
  EXPECT_TRUE(std::none_of(
      snapshot.candidates.begin(),
      snapshot.candidates.end(),
      [](const LocalPlanCandidate& candidate) {
        return candidate.selected &&
               candidate.state == LocalCandidateState::RotationBlocked;
      }));
}

TEST(LocalPlannerCore, TwoWayDriveKeepsCloseBehindGoalTrackable) {
  auto pathsDir = writePlannerPathsWithStartPoint(
      "nav_kernel_two_way_close_behind_fixture", 0.4);

  LocalPlannerParams p;
  p.twoWayDrive = true;
  p.checkObstacle = false;
  p.useTraversabilityCost = false;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.goalBehindRange = 0.8;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(-0.6, 0);

  auto result = planner.plan(nullptr, 0, 1.0);

  ASSERT_TRUE(result.pathFound);
  ASSERT_FALSE(result.path.empty());
  EXPECT_LT(result.path.front().x, -0.1);
}

TEST(LocalPlannerCore, SingleDirectionDriveFreezesCloseBehindGoal) {
  auto pathsDir = writePlannerPathsWithStartPoint(
      "nav_kernel_single_direction_close_behind_fixture", 0.4);

  LocalPlannerParams p;
  p.twoWayDrive = false;
  p.checkObstacle = false;
  p.useTraversabilityCost = false;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.goalBehindRange = 0.8;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(-0.6, 0);

  auto result = planner.plan(nullptr, 0, 1.0);

  EXPECT_FALSE(result.pathFound);
}

// 閳光偓閳光偓 RotLUT precomputed weights 閳光偓閳光偓

TEST(LocalPlannerCore, TraversabilityGridTriggersNearFieldStop) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_traversability_stop_fixture");

  LocalPlannerParams p;
  p.checkObstacle = true;
  p.useTraversabilityCost = true;
  p.traversabilityNearFieldStop = true;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  std::vector<float> riskGrid(5 * 5, 0.0f);
  riskGrid[2 * 5 + 2] = 95.0f;
  riskGrid[2 * 5 + 3] = 95.0f;
  planner.setTraversabilityGrid(riskGrid.data(), 5, 5, 0.25, 0.0, -0.5);

  auto result = planner.plan(nullptr, 0, 0.0);

  EXPECT_TRUE(result.nearFieldStop);
}

TEST(LocalPlannerCore, FrontTraversabilityDoesNotStopReverseIntent) {
  auto pathsDir = writeMinimalPlannerPaths(
      "nav_kernel_reverse_ignores_front_traversability_fixture");

  LocalPlannerParams p;
  p.checkObstacle = true;
  p.useTraversabilityCost = true;
  p.traversabilityNearFieldStop = true;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(-5, 0);

  constexpr int kSize = 25;
  constexpr double kResolution = 0.1;
  constexpr double kOrigin = -1.25;
  std::vector<float> riskGrid(kSize * kSize, 0.0f);
  const int row = static_cast<int>((0.0 - kOrigin) / kResolution);
  const int frontCol = static_cast<int>((0.6 - kOrigin) / kResolution);
  riskGrid[row * kSize + frontCol] = 100.0f;
  planner.setTraversabilityGrid(
      riskGrid.data(), kSize, kSize, kResolution, kOrigin, kOrigin);

  const auto result = planner.plan(nullptr, 0, 0.0);

  EXPECT_FALSE(result.nearFieldStop);
}

TEST(LocalPlannerCore, RearTraversabilityStopsReverseIntent) {
  auto pathsDir = writeMinimalPlannerPaths(
      "nav_kernel_reverse_checks_rear_traversability_fixture");

  LocalPlannerParams p;
  p.checkObstacle = true;
  p.useTraversabilityCost = true;
  p.traversabilityNearFieldStop = true;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(-5, 0);

  constexpr int kSize = 25;
  constexpr double kResolution = 0.1;
  constexpr double kOrigin = -1.25;
  std::vector<float> riskGrid(kSize * kSize, 0.0f);
  const int row = static_cast<int>((0.0 - kOrigin) / kResolution);
  const int rearCol = static_cast<int>((-0.6 - kOrigin) / kResolution);
  riskGrid[row * kSize + rearCol] = 100.0f;
  planner.setTraversabilityGrid(
      riskGrid.data(), kSize, kSize, kResolution, kOrigin, kOrigin);

  const auto result = planner.plan(nullptr, 0, 0.0);

  EXPECT_TRUE(result.nearFieldStop);
}

TEST(LocalPlannerCore, FootprintPointsDoNotTriggerNearFieldStop) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_footprint_filter_fixture");

  LocalPlannerParams p;
  p.vehicleLength = 1.0;
  p.vehicleWidth = 0.6;
  p.checkObstacle = true;
  p.useTraversabilityCost = false;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  std::vector<float> cloud = {
      0.45f, -0.20f, 0.45f, 0.45f,
      0.48f, -0.10f, 0.50f, 0.50f};

  auto result = planner.plan(cloud.data(), static_cast<int>(cloud.size() / 4), 0.0);

  EXPECT_FALSE(result.nearFieldStop);
  EXPECT_TRUE(result.pathFound);
}

TEST(LocalPlannerCore, ObstacleAheadOfFootprintTriggersNearFieldStop) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_front_edge_stop_fixture");

  LocalPlannerParams p;
  p.vehicleLength = 1.0;
  p.vehicleWidth = 0.6;
  p.checkObstacle = true;
  p.useTraversabilityCost = false;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  std::vector<float> cloud = {0.72f, 0.0f, 0.45f, 0.45f};

  auto result = planner.plan(cloud.data(), static_cast<int>(cloud.size() / 4), 0.0);

  EXPECT_TRUE(result.nearFieldStop);
}

TEST(LocalPlannerCore, FrontObstacleDoesNotStopReverseIntent) {
  auto pathsDir = writeMinimalPlannerPaths(
      "nav_kernel_reverse_ignores_front_obstacle_fixture");

  LocalPlannerParams p;
  p.vehicleLength = 1.0;
  p.vehicleWidth = 0.6;
  p.checkObstacle = true;
  p.useTraversabilityCost = false;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(-5, 0);

  std::vector<float> cloud = {0.72f, 0.0f, 0.45f, 0.45f};
  const auto result = planner.plan(
      cloud.data(), static_cast<int>(cloud.size() / 4), 0.0);

  EXPECT_FALSE(result.nearFieldStop);
}

TEST(LocalPlannerCore, RearObstacleStopsReverseIntent) {
  auto pathsDir = writeMinimalPlannerPaths(
      "nav_kernel_reverse_checks_rear_obstacle_fixture");

  LocalPlannerParams p;
  p.vehicleLength = 1.0;
  p.vehicleWidth = 0.6;
  p.checkObstacle = true;
  p.useTraversabilityCost = false;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(-5, 0);

  std::vector<float> cloud = {-0.72f, 0.0f, 0.45f, 0.45f};
  const auto result = planner.plan(
      cloud.data(), static_cast<int>(cloud.size() / 4), 0.0);

  EXPECT_TRUE(result.nearFieldStop);
}

TEST(LocalPlannerCore, OverheadPointAboveDefaultBodyEnvelopeDoesNotStop) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_overhead_filter_fixture");

  LocalPlannerParams p;
  p.vehicleLength = 1.0;
  p.vehicleWidth = 0.6;
  p.checkObstacle = true;
  p.useTerrainAnalysis = false;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  std::vector<float> cloud = {0.72f, 0.0f, 2.5f, 2.5f};

  const auto result = planner.plan(
      cloud.data(), static_cast<int>(cloud.size() / 4), 0.0);

  EXPECT_FALSE(result.nearFieldStop);
  EXPECT_TRUE(result.pathFound);
}

TEST(LocalPlannerCore, OverheadHeightLimitIsConfigurable) {
  auto pathsDir = writeMinimalPlannerPaths(
      "nav_kernel_configurable_overhead_filter_fixture");

  LocalPlannerParams p;
  p.vehicleLength = 1.0;
  p.vehicleWidth = 0.6;
  p.checkObstacle = true;
  p.useTerrainAnalysis = false;
  p.obstacleHeightMax = 3.0;
  p.nearFieldStopDis = 0.6;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  std::vector<float> cloud = {0.72f, 0.0f, 2.5f, 2.5f};
  const auto result = planner.plan(
      cloud.data(), static_cast<int>(cloud.size() / 4), 0.0);

  EXPECT_TRUE(result.nearFieldStop);
}

TEST(LocalPlannerCore, TraversabilityHardCostBlocksPathSelection) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_traversability_block_fixture");

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilityHardCost = 90.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  std::vector<float> riskGrid(9 * 9, 95.0f);
  planner.setTraversabilityGrid(riskGrid.data(), 9, 9, 0.5, -2.0, -2.0);

  auto result = planner.plan(nullptr, 0, 0.0);

  EXPECT_FALSE(result.pathFound);
}

TEST(LocalPlannerCore, ReportsRecoveryExhaustedWithoutChangingRecoveryStateAbi) {
  auto pathsDir = writeMinimalPlannerPaths(
      "nav_kernel_recovery_exhausted_fixture");

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilityHardCost = 90.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.recoveryBlockedThre = 0.0;
  p.recoveryMaxCycles = 0;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  std::vector<float> riskGrid(9 * 9, 95.0f);
  planner.setTraversabilityGrid(riskGrid.data(), 9, 9, 0.5, -2.0, -2.0);

  const auto result = planner.plan(nullptr, 0, 1.0);

  EXPECT_FALSE(result.pathFound);
  EXPECT_TRUE(result.recoveryExhausted);
  EXPECT_EQ(result.recoveryState, 0);
  EXPECT_TRUE(result.path.empty());
}
TEST(LocalPlannerCore, DebugSnapshotShowsRepresentativeCandidatesAndSelection) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_debug_candidates_fixture");

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = false;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.debugCandidateLimit = kRotDirs;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  const auto result = planner.plan(nullptr, 0, 0.0);
  const auto snapshot = planner.debugSnapshot();

  ASSERT_TRUE(result.pathFound);
  ASSERT_TRUE(snapshot.valid);
  ASSERT_FALSE(snapshot.candidates.empty());
  EXPECT_EQ(snapshot.validRotationCount, static_cast<int>(snapshot.candidates.size()));
  EXPECT_DOUBLE_EQ(snapshot.traversabilitySoftCost, p.traversabilitySoftCost);
  EXPECT_DOUBLE_EQ(snapshot.traversabilityHardCost, p.traversabilityHardCost);
  EXPECT_LE(snapshot.candidates.size(), static_cast<std::size_t>(kRotDirs));
  const auto selected_count = std::count_if(
      snapshot.candidates.begin(),
      snapshot.candidates.end(),
      [](const LocalPlanCandidate& candidate) { return candidate.selected; });
  ASSERT_EQ(selected_count, 1);
  const auto selected = std::find_if(
      snapshot.candidates.begin(),
      snapshot.candidates.end(),
      [](const LocalPlanCandidate& candidate) { return candidate.selected; });
  ASSERT_NE(selected, snapshot.candidates.end());
  EXPECT_EQ(selected->state, LocalCandidateState::Feasible);
  ASSERT_FALSE(selected->path.empty());
  EXPECT_NEAR(selected->path.front().x, result.path.front().x, 1e-9);
  EXPECT_NEAR(selected->path.front().y, result.path.front().y, 1e-9);
}

TEST(LocalPlannerCore, DebugSnapshotMarksSoftTerrainCostWithoutHardBlocking) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_debug_soft_terrain_fixture");

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilitySoftCost = 40.0;
  p.traversabilityHardCost = 90.0;
  p.traversabilityWeight = 0.02;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.debugCandidateLimit = kRotDirs;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);
  std::vector<float> riskGrid(9 * 9, 45.0f);
  planner.setTraversabilityGrid(riskGrid.data(), 9, 9, 0.5, -2.0, -2.0);

  const auto result = planner.plan(nullptr, 0, 0.0);
  const auto snapshot = planner.debugSnapshot();

  ASSERT_TRUE(result.pathFound);
  ASSERT_TRUE(snapshot.valid);
  EXPECT_TRUE(std::any_of(
      snapshot.candidates.begin(),
      snapshot.candidates.end(),
      [](const LocalPlanCandidate& candidate) {
        return candidate.state == LocalCandidateState::TerrainCost &&
               candidate.terrainRisk >= 40.0 &&
               candidate.terrainRisk < 90.0;
      }));
}

TEST(LocalPlannerCore, DebugSnapshotDoesNotClaimTerrainCostWhenWeightIsZero) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_debug_zero_terrain_weight_fixture");

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilitySoftCost = 40.0;
  p.traversabilityHardCost = 90.0;
  p.traversabilityWeight = 0.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.debugCandidateLimit = kRotDirs;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);
  std::vector<float> riskGrid(9 * 9, 45.0f);
  planner.setTraversabilityGrid(riskGrid.data(), 9, 9, 0.5, -2.0, -2.0);

  const auto result = planner.plan(nullptr, 0, 0.0);
  const auto snapshot = planner.debugSnapshot();

  ASSERT_TRUE(result.pathFound);
  const auto selected = std::find_if(
      snapshot.candidates.begin(), snapshot.candidates.end(),
      [](const LocalPlanCandidate& candidate) { return candidate.selected; });
  ASSERT_NE(selected, snapshot.candidates.end());
  EXPECT_EQ(selected->state, LocalCandidateState::Feasible);
  EXPECT_EQ(selected->terrainSoftPenalizedPathCount, 0);
}

TEST(LocalPlannerCore, DebugSnapshotClassifiesSoftTerrainZeroingAsTerrainBlocked) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_debug_soft_zero_fixture");

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilitySoftCost = 40.0;
  p.traversabilityHardCost = 90.0;
  p.traversabilityWeight = 1.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.debugCandidateLimit = kRotDirs;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);
  std::vector<float> riskGrid(9 * 9, 45.0f);
  planner.setTraversabilityGrid(riskGrid.data(), 9, 9, 0.5, -2.0, -2.0);

  const auto result = planner.plan(nullptr, 0, 0.0);
  const auto snapshot = planner.debugSnapshot();

  EXPECT_FALSE(result.pathFound);
  EXPECT_TRUE(std::any_of(
      snapshot.candidates.begin(), snapshot.candidates.end(),
      [](const LocalPlanCandidate& candidate) {
        return candidate.state == LocalCandidateState::TerrainBlocked &&
               candidate.terrainAllowedPathCount > 0 &&
               candidate.heightCostAllowedPathCount > 0 &&
               candidate.contributingPathCount == 0;
      }));
}

TEST(LocalPlannerCore, DebugSnapshotCollisionGatePrecedesHardTerrainGate) {
  auto pathsDir = writeMinimalPlannerPaths(
      "nav_kernel_debug_collision_before_terrain_fixture");

  LocalPlannerParams p;
  p.checkObstacle = true;
  p.pointPerPathThre = 0;
  p.useTraversabilityCost = true;
  p.traversabilityHardCost = 80.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.debugCandidateLimit = kRotDirs;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);
  std::vector<float> riskGrid(9 * 9, 95.0f);
  planner.setTraversabilityGrid(riskGrid.data(), 9, 9, 0.5, -2.0, -2.0);
  (void)planner.plan(nullptr, 0, 0.0);
  const auto snapshot = planner.debugSnapshot();
  const auto forward = std::find_if(
      snapshot.candidates.begin(), snapshot.candidates.end(),
      [](const LocalPlanCandidate& candidate) {
        return candidate.rotationIndex == 18;
      });

  ASSERT_NE(forward, snapshot.candidates.end());
  EXPECT_EQ(forward->state, LocalCandidateState::CollisionBlocked);
  EXPECT_GT(forward->totalPathCount, 0);
  EXPECT_EQ(forward->collisionFreePathCount, 0);
  EXPECT_LT(forward->terrainRisk, 0.0);
}

TEST(LocalPlannerCore, DebugSnapshotDistinguishesTerrainBlockedCandidates) {
  auto pathsDir = writeMinimalPlannerPaths("nav_kernel_debug_terrain_fixture");

  LocalPlannerParams p;
  p.checkObstacle = false;
  p.useTraversabilityCost = true;
  p.traversabilityHardCost = 80.0;
  p.pathScaleBySpeed = false;
  p.pathRangeBySpeed = false;
  p.debugCandidateLimit = kRotDirs;

  LocalPlannerCore planner(p);
  ASSERT_TRUE(planner.loadPaths(pathsDir.string()));
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);
  std::vector<float> riskGrid(9 * 9, 95.0f);
  planner.setTraversabilityGrid(riskGrid.data(), 9, 9, 0.5, -2.0, -2.0);

  const auto result = planner.plan(nullptr, 0, 0.0);
  const auto snapshot = planner.debugSnapshot();

  EXPECT_FALSE(result.pathFound);
  ASSERT_TRUE(snapshot.valid);
  ASSERT_FALSE(snapshot.candidates.empty());
  EXPECT_TRUE(std::any_of(
      snapshot.candidates.begin(),
      snapshot.candidates.end(),
      [](const LocalPlanCandidate& candidate) {
        return candidate.state == LocalCandidateState::TerrainBlocked &&
               candidate.terrainRisk >= 80.0;
      }));
}

TEST(RotLUT, PrecomputedRotDirW) {
  const auto& lut = rotLUT();
  // Verify precomputed matches runtime calculation
  for (int i = 0; i < 36; i++) {
    EXPECT_DOUBLE_EQ(lut.rotDirW[i], computeRotDirW(i))
        << "rotDirW mismatch at index " << i;
    double w = computeRotDirW(i);
    EXPECT_DOUBLE_EQ(lut.rotDirW4[i], w * w * w * w)
        << "rotDirW4 mismatch at index " << i;
  }
}

TEST(RotLUT, PrecomputedGroupDirW) {
  const auto& lut = rotLUT();
  for (int g = 0; g < 7; g++) {
    double w = computeGroupDirW(g);
    EXPECT_DOUBLE_EQ(lut.groupDirW[g], w);
    EXPECT_DOUBLE_EQ(lut.groupDirW2[g], w * w);
  }
}

TEST(RotLUT, Pow025LUT) {
  const auto& lut = rotLUT();
  // pow025[0] = 0
  EXPECT_FLOAT_EQ(lut.pow025[0], 0.0f);
  // pow025[100] = pow(1.0, 0.25) = 1.0
  EXPECT_NEAR(lut.pow025[100], 1.0f, 1e-5f);
  // Check a few random values
  for (int i = 1; i < RotLUT::kPow025Size; i += 37) {
    float expected = static_cast<float>(std::sqrt(std::sqrt(i * 0.01)));
    EXPECT_NEAR(lut.pow025[i], expected, 1e-5f)
        << "pow025 mismatch at index " << i;
  }
}

// 閳光偓閳光偓 scorePathFast 閳光偓閳光偓

TEST(ScorePathFast, MatchesScorePath) {
  const auto& lut = rotLUT();
  PathScoreParams p;
  p.dirWeight = 0.02;
  p.slopeWeight = 3.0;
  p.omniDirGoalThre = 5.0;

  // Test across multiple parameter combinations
  for (int rotDir : {0, 9, 18, 27, 35}) {
    for (int grp : {0, 3, 6}) {
      for (double dirDiff : {0.0, 10.0, 30.0, 45.0}) {
        for (double goalDis : {2.0, 10.0}) {
          double orig = scorePath(dirDiff, lut.rotDirW[rotDir], lut.groupDirW[grp],
                                  0.05, goalDis, p);
          double fast = scorePathFast(dirDiff, lut.rotDirW4[rotDir], lut.groupDirW2[grp],
                                      0.05f, goalDis, p, lut);
          // Allow small LUT quantization error
          if (orig > 0)
            EXPECT_NEAR(fast, orig, std::fabs(orig) * 0.02 + 1e-6)
                << "rotDir=" << rotDir << " grp=" << grp
                << " dirDiff=" << dirDiff << " goalDis=" << goalDis;
        }
      }
    }
  }
}

TEST(ScorePathFast, PerfectAlignment) {
  const auto& lut = rotLUT();
  PathScoreParams p;
  p.dirWeight = 0.02;
  // dirDiff=0 閳?LUT index 0 閳?sqrtSqrtDw=0 閳?score = rotDirW4
  double s = scorePathFast(0.0, lut.rotDirW4[18], lut.groupDirW2[3],
                           0.0f, 10.0, p, lut);
  EXPECT_NEAR(s, 10000.0, 1.0);  // rotDirW[18]=10, 10^4=10000
}
