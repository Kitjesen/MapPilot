#include <gtest/gtest.h>
#include "nav_core/local_planner_core.hpp"
#include <cmath>
#include <vector>

using namespace nav_core;

// ── RotLUT ──

TEST(RotLUT, TablesCorrect) {
  const auto& lut = rotLUT();
  // rotDir=18 → 0°
  EXPECT_NEAR(lut.c[18], 1.0, 1e-10);
  EXPECT_NEAR(lut.s[18], 0.0, 1e-10);
  // rotDir=0 → -180°
  EXPECT_NEAR(lut.c[0], -1.0, 1e-10);
  EXPECT_NEAR(lut.s[0], 0.0, 1e-6);
  // rotDir=9 → -90°
  EXPECT_NEAR(lut.c[9], 0.0, 1e-6);
  EXPECT_NEAR(lut.s[9], -1.0, 1e-10);
  // rotDir=27 → +90°
  EXPECT_NEAR(lut.c[27], 0.0, 1e-6);
  EXPECT_NEAR(lut.s[27], 1.0, 1e-10);
}

// ── angDiffDeg ──

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
  // 350° vs 10° → 20° difference
  EXPECT_NEAR(angDiffDeg(350.0, 10.0), 20.0, 1e-10);
}

// ── worldToVoxel ──

TEST(WorldToVoxel, CenterPoint) {
  VoxelGridParams g;
  int indX, indY;
  // 原点附近应该映射到网格中心区域
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
  // 很远的点应该超出网格
  bool ok = worldToVoxel(100.0, 100.0, g, indX, indY);
  EXPECT_FALSE(ok);
}

TEST(WorldToVoxel, NearZeroX) {
  VoxelGridParams g;
  int indX, indY;
  // x2 接近 gridVoxelOffsetX 时 scaleY 可能很小
  bool ok = worldToVoxel(g.gridVoxelOffsetX, 0.0, g, indX, indY);
  // scaleY = x2/offsetX + searchRadius/offsetY * (offsetX-x2)/offsetX
  // = 1.0 + searchRadius/offsetY * 0 = 1.0 → 正常
  EXPECT_TRUE(ok);
}

// ── scorePath ──

TEST(ScorePath, PerfectAlignment) {
  PathScoreParams p;
  p.dirWeight = 0.02;
  // dirDiff=0 → dw=0 → sqrtSqrtDw=0 → (1-0)=1
  double s = scorePath(0.0, 10.0, 4.0, 0.0, 10.0, p);
  // 正常模式: 1.0 * 10^2 * 10^2 * 1.0 = 10000
  EXPECT_DOUBLE_EQ(s, 10000.0);
}

TEST(ScorePath, LargeAnglePenalty) {
  PathScoreParams p;
  p.dirWeight = 0.02;
  double s = scorePath(90.0, 10.0, 4.0, 0.0, 10.0, p);
  // dw = 0.02*90 = 1.8, sqrt(sqrt(1.8)) ≈ 1.158
  // (1 - 1.158) < 0, but the formula can go negative
  EXPECT_LT(s, 0.0);  // C++ 原始代码允许负分
}

TEST(ScorePath, ModerateAngle) {
  PathScoreParams p;
  p.dirWeight = 0.02;
  // dirDiff=50°: dw=1.0, sqrt(sqrt(1.0))=1.0, (1-1)=0
  double s = scorePath(50.0, 10.0, 4.0, 0.0, 10.0, p);
  EXPECT_NEAR(s, 0.0, 1e-10);
}

TEST(ScorePath, DirWeightEffect) {
  PathScoreParams p1, p2;
  p1.dirWeight = 0.02;
  p2.dirWeight = 0.04;  // 更强的方向惩罚

  double s1 = scorePath(30.0, 10.0, 4.0, 0.0, 10.0, p1);
  double s2 = scorePath(30.0, 10.0, 4.0, 0.0, 10.0, p2);
  EXPECT_GT(s1, s2);  // 更强的惩罚 → 更低的分数
}

TEST(ScorePath, NearGoalUsesGroupDirW) {
  PathScoreParams p;
  p.dirWeight = 0.02;
  p.omniDirGoalThre = 5.0;
  // relativeGoalDis=3 < omniDirGoalThre=5 → 使用 groupDirW
  double s = scorePath(0.0, 10.0, 4.0, 0.0, 3.0, p);
  // (1-0) * 4^2 * 1.0 = 16
  EXPECT_DOUBLE_EQ(s, 16.0);
}

TEST(ScorePath, FarGoalUsesRotDirW) {
  PathScoreParams p;
  p.dirWeight = 0.02;
  p.omniDirGoalThre = 5.0;
  // relativeGoalDis=10 > omniDirGoalThre=5 → 使用 rotDirW^4
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
  // terrainPenalty=0.3 → factor = max(0, 1 - 5*0.3) = max(0, -0.5) = 0
  double s = scorePath(0.0, 10.0, 4.0, 0.3, 10.0, p);
  EXPECT_DOUBLE_EQ(s, 0.0);
}

// ── computeRotDirW ──

TEST(RotDirW, Forward) {
  // rotDir=18 → 0° → |18-9|+1 = 10? No, 18 >= 18 → |18-27|+1 = 10
  EXPECT_DOUBLE_EQ(computeRotDirW(18), 10.0);
}

TEST(RotDirW, Backward) {
  // rotDir=0 → -180° → |0-9|+1 = 10
  EXPECT_DOUBLE_EQ(computeRotDirW(0), 10.0);
}

TEST(RotDirW, SideLeft) {
  // rotDir=9 → -90° → |9-9|+1 = 1
  EXPECT_DOUBLE_EQ(computeRotDirW(9), 1.0);
}

TEST(RotDirW, SideRight) {
  // rotDir=27 → +90° → |27-27|+1 = 1
  EXPECT_DOUBLE_EQ(computeRotDirW(27), 1.0);
}

// ── computeGroupDirW ──

TEST(GroupDirW, CenterGroup) {
  // pathGroup=3 → 4 - |3-3| = 4
  EXPECT_DOUBLE_EQ(computeGroupDirW(3), 4.0);
}

TEST(GroupDirW, EdgeGroup) {
  // pathGroup=0 → 4 - |0-3| = 1
  EXPECT_DOUBLE_EQ(computeGroupDirW(0), 1.0);
}

// ── selectBestGroup ──

TEST(SelectBestGroup, FindsMaxScore) {
  int groupNum = 7;
  std::vector<double> scores(36 * groupNum, 0.0);
  // 设置 rotDir=18, group=3 (前方中央) 为最高分
  scores[18 * groupNum + 3] = 100.0;

  auto result = selectBestGroup(scores, groupNum, -180.0, 180.0, true, false);
  EXPECT_EQ(result.selectedGroupID, 18 * groupNum + 3);
  EXPECT_DOUBLE_EQ(result.maxScore, 100.0);
}

TEST(SelectBestGroup, NoPath) {
  int groupNum = 7;
  std::vector<double> scores(36 * groupNum, 0.0);  // 全零

  auto result = selectBestGroup(scores, groupNum, -180.0, 180.0, true, false);
  EXPECT_EQ(result.selectedGroupID, -1);
}

TEST(SelectBestGroup, RotObstacleFilter) {
  int groupNum = 7;
  std::vector<double> scores(36 * groupNum, 0.0);
  // 最高分在 rotDir=18 (0°)
  scores[18 * groupNum + 3] = 100.0;
  // 次高分在 rotDir=15 (-30°)
  scores[15 * groupNum + 3] = 50.0;

  // 旋转障碍物限制: 只允许 [-40°, -20°] → 0° 被排除
  auto result = selectBestGroup(scores, groupNum, -40.0, -20.0, true, true);
  EXPECT_EQ(result.selectedGroupID, 15 * groupNum + 3);
  EXPECT_DOUBLE_EQ(result.maxScore, 50.0);
}
