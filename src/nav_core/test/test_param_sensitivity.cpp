/**
 * test_param_sensitivity.cpp — nav_core 参数敏感性测试
 *
 * 验证关键参数在边界值/扫描值下的行为，防止调参引入回归。
 *
 * 测试覆盖:
 *   TEST(PathFollower, DirDiffThreVariation)  — 小/大 dirDiffThre 先转再走 vs 边走边转
 *   TEST(LocalPlanner, SlopeWeightImpact)     — slopeWeight=0 vs 5 对坡度评分影响
 *   TEST(PctAdapter,  StuckBoundaryExact)     — stuckTimeoutSec=2 精确边界
 *   TEST(PctAdapter,  ReplanCooldownDebounce) — replanCooldownSec=3 冷却防抖
 */

#include <gtest/gtest.h>
#include "nav_core/path_follower_core.hpp"
#include "nav_core/local_planner_core.hpp"
#include "nav_core/pct_adapter_core.hpp"
#include <cmath>

using namespace nav_core;

// MSVC 聚合初始化辅助
static Pose makePose(double x, double y, double z, double yaw = 0.0) {
  Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.yaw = yaw;
  return p;
}

// ═══════════════════════════════════════════════════
//  TEST 1: PathFollower — dirDiffThre 参数敏感性
//
//  场景：机器人在原点，目标路点在正右方 (x=2, y=0)，
//  但机器人当前朝向偏差 90°（面朝 y 轴方向）。
//
//  小阈值 (0.1 rad): dirDiff > thre → canAccel=false → vx≈0, |wz|>0 (先转再走)
//  大阈值 (1.5 rad): dirDiff < thre → canAccel=true  → vx>0 (边走边转)
// ═══════════════════════════════════════════════════

TEST(PathFollower, DirDiffThreVariation) {
  // 共用路径: 沿 +X 方向, 从 (0.5,0,0) 到 (3,0,0)
  std::vector<Vec3> path = {{0.5, 0, 0}, {1.0, 0, 0}, {2.0, 0, 0}, {3.0, 0, 0}};
  Vec3 robot{0, 0, 0};
  double yawDiff = M_PI / 2;  // 90° 朝向偏差

  // ── 小阈值: dirDiffThre = 0.1 rad (~5.7°) ──
  {
    PathFollowerParams p;
    p.dirDiffThre      = 0.1;
    p.maxSpeed         = 1.0;
    p.maxAccel         = 1.0;
    p.baseLookAheadDis = 0.3;
    p.lookAheadRatio   = 0.5;
    p.stopDisThre      = 0.2;
    p.omniDirGoalThre  = 0.1;   // 小值, 使 omni 例外不生效 (dis >> 0.1)
    p.omniDirDiffThre  = 1.5;
    p.twoWayDrive      = false;
    p.noRotAtGoal      = false;

    PathFollowerState state;
    auto out = computeControl(robot, yawDiff, path,
                              1.0, 0.0, 1.0, 0, p, state);

    // 90° >> 0.1 rad, dis >> omniDirGoalThre → canAccel = false
    EXPECT_FALSE(out.canAccel)
        << "dirDiffThre=0.1: 90deg error should block linear accel";
    // vx 应接近 0 (vehicleSpeed 从 0 开始, canAccel=false → 趋向 0)
    EXPECT_NEAR(out.cmd.vx, 0.0, 0.01)
        << "dirDiffThre=0.1: vx should be ~0 (turn in place first)";
    // wz 应非零 (转向修正)
    EXPECT_GT(std::fabs(out.cmd.wz), 0.0)
        << "dirDiffThre=0.1: |wz| should be >0 (turning to correct heading)";
  }

  // ── 大阈值: dirDiffThre = 1.6 rad (~92°, > PI/2) ──
  {
    PathFollowerParams p;
    p.dirDiffThre      = 1.6;   // 92° > 90° → 允许边走边转
    p.maxSpeed         = 1.0;
    p.maxAccel         = 10.0;  // step=0.1, threshold=0.1; 2次调用后 speed=0.2 > 0.1
    p.baseLookAheadDis = 0.3;
    p.lookAheadRatio   = 0.5;
    p.stopDisThre      = 0.2;
    p.omniDirGoalThre  = 0.1;
    p.omniDirDiffThre  = 1.5;
    p.twoWayDrive      = false;
    p.noRotAtGoal      = false;

    PathFollowerState state;
    // 调两次让 vehicleSpeed 积分 > maxAccel/100 阈值
    computeControl(robot, yawDiff, path, 1.0, 0.0, 1.0, 0, p, state);
    auto out = computeControl(robot, yawDiff, path, 1.0, 0.0, 1.0, 0, p, state);

    // 90° ≈ 1.571 rad < 1.6 rad → canAccel = true (允许边走边转)
    EXPECT_TRUE(out.canAccel)
        << "dirDiffThre=1.6: 90deg (1.571) < 1.6 rad should allow accel";
    // vx > 0 (omni 分解: vx = cos(dirDiff)*speed)
    EXPECT_GT(out.cmd.vx, 0.0)
        << "dirDiffThre=1.6: vx should be >0 (moving while turning)";
  }

  // ── 单调性: 更大的 dirDiffThre 允许更大的角度误差通过 ──
  {
    PathFollowerParams p;
    p.maxSpeed         = 1.0;
    p.maxAccel         = 1.0;
    p.baseLookAheadDis = 0.3;
    p.lookAheadRatio   = 0.5;
    p.stopDisThre      = 0.05;
    p.omniDirGoalThre  = 0.1;
    p.omniDirDiffThre  = 1.5;
    p.twoWayDrive      = false;
    p.noRotAtGoal      = false;

    // dirDiff = 0.12 rad
    double angle = 0.12;

    // thre = 0.1 → 0.12 > 0.1 → blocked
    p.dirDiffThre = 0.1;
    PathFollowerState s1;
    auto o1 = computeControl(robot, angle, path, 1.0, 0.0, 1.0, 0, p, s1);
    EXPECT_FALSE(o1.canAccel) << "dirDiffThre=0.1, angle=0.12 → blocked";

    // thre = 0.2 → 0.12 < 0.2 → allowed
    p.dirDiffThre = 0.2;
    PathFollowerState s2;
    auto o2 = computeControl(robot, angle, path, 1.0, 0.0, 1.0, 0, p, s2);
    EXPECT_TRUE(o2.canAccel) << "dirDiffThre=0.2, angle=0.12 → allowed";
  }
}

// ═══════════════════════════════════════════════════
//  TEST 2: LocalPlanner — slopeWeight 参数敏感性
//
//  方向A（平坦）：dirDiff=0.1 rad, slopePenalty=0.0
//  方向B（有坡）：dirDiff=0.1 rad, slopePenalty=1.0
//
//  slopeWeight=0: 两个方向得分接近 (坡度不影响)
//  slopeWeight=5: 方向A得分 > 方向B得分 (坡度明显影响)
// ═══════════════════════════════════════════════════

TEST(LocalPlanner, SlopeWeightImpact) {
  const double dirDiffDeg      = 5.73;  // ~0.1 rad → angDiffDeg format
  const double rotDirW         = 1.0;
  const double groupDirW       = 1.0;
  const double relativeGoalDis = 10.0;  // 远离目标, 走 rotDirW 分支

  const double slopePenaltyA = 0.0;  // 平坦
  const double slopePenaltyB = 1.0;  // 最大坡度

  PathScoreParams p;
  p.dirWeight       = 0.02;
  p.omniDirGoalThre = 5.0;

  // ── slopeWeight = 0: 坡度不影响 → 两个方向得分相同 ──
  p.slopeWeight = 0.0;
  double scoreA_sw0 = scorePath(dirDiffDeg, rotDirW, groupDirW,
                                slopePenaltyA, relativeGoalDis, p);
  double scoreB_sw0 = scorePath(dirDiffDeg, rotDirW, groupDirW,
                                slopePenaltyB, relativeGoalDis, p);

  EXPECT_NEAR(scoreA_sw0, scoreB_sw0, 1e-9)
      << "slopeWeight=0: terrain penalty should have no effect";
  EXPECT_GT(scoreA_sw0, 0.0)
      << "slopeWeight=0: scores should be positive";

  // ── slopeWeight = 5: 坡度明显影响 → A > B ──
  p.slopeWeight = 5.0;
  double scoreA_sw5 = scorePath(dirDiffDeg, rotDirW, groupDirW,
                                slopePenaltyA, relativeGoalDis, p);
  double scoreB_sw5 = scorePath(dirDiffDeg, rotDirW, groupDirW,
                                slopePenaltyB, relativeGoalDis, p);

  // 方向A (平坦): terrainFactor = max(0, 1-5*0) = 1.0 → 满分
  EXPECT_GT(scoreA_sw5, 0.0)
      << "slopeWeight=5, flat: score should be positive";
  // 方向B (坡): terrainFactor = max(0, 1-5*1.0) = max(0, -4) = 0 → 分数=0
  EXPECT_NEAR(scoreB_sw5, 0.0, 1e-9)
      << "slopeWeight=5, steep slope: score should be 0 (clamped)";
  // A 明显优于 B
  EXPECT_GT(scoreA_sw5, scoreB_sw5)
      << "slopeWeight=5: flat path should score higher than steep path";

  // ── 验证单调性: slopeWeight 越大, 有坡方向得分越低 ──
  p.slopeWeight = 1.0;
  double scoreB_sw1 = scorePath(dirDiffDeg, rotDirW, groupDirW,
                                slopePenaltyB, relativeGoalDis, p);
  // terrainFactor(sw=1) = max(0, 1-1*1.0) = 0 → 已经是0
  // 用中等坡度验证
  double midSlope = 0.3;
  p.slopeWeight = 1.0;
  double scoreMid_sw1 = scorePath(dirDiffDeg, rotDirW, groupDirW,
                                  midSlope, relativeGoalDis, p);
  p.slopeWeight = 3.0;
  double scoreMid_sw3 = scorePath(dirDiffDeg, rotDirW, groupDirW,
                                  midSlope, relativeGoalDis, p);
  EXPECT_GT(scoreMid_sw1, scoreMid_sw3)
      << "mid-slope: higher slopeWeight should yield lower score";
}

// ═══════════════════════════════════════════════════
//  TEST 3: PctAdapter — stuckTimeoutSec=2 精确边界
//
//  停留 2.0s (恰好到期) → 应触发 stuck
//  停留 1.9s (差0.1秒) → 不应触发 stuck
// ═══════════════════════════════════════════════════

TEST(PctAdapter, StuckBoundaryExact) {
  WaypointTrackerParams p;
  p.stuckTimeoutSec   = 2.0;
  p.replanCooldownSec = 0.001;  // 极小冷却, 不干扰边界测试
  p.maxReplanCount    = 3;
  p.waypointDistance  = 0.5;
  p.arrivalThreshold  = 0.5;
  p.searchWindow      = 5;

  WaypointTracker tracker(p);

  // 路径远离机器人 (x=10..14), 机器人在原点 → 永远不到达
  Path path;
  for (int i = 0; i < 5; ++i) {
    path.push_back(makePose(static_cast<double>(i + 10), 0, 0));
  }
  // setPath at t=100 → lastProgressTime_=100 > 0 (stuck detection enabled)
  tracker.setPath(path, 100.0);

  Vec3 robotPos{0, 0, 0};
  std::vector<Vec3> emptyOdom;

  // ── t=101.9: elapsed = 1.9s < timeout=2.0s → 不触发 stuck ──
  auto r1 = tracker.update(robotPos, emptyOdom, 101.9);
  EXPECT_NE(r1.event, WaypointEvent::kReplanning)
      << "elapsed=1.9s: should NOT trigger stuck (timeout=2.0s)";
  EXPECT_NE(r1.event, WaypointEvent::kStuckFinal)
      << "elapsed=1.9s: should NOT be stuck final";

  // ── t=102.001: elapsed = 2.001s > timeout=2.0s → 触发第1次重规划 ──
  auto r2 = tracker.update(robotPos, emptyOdom, 102.001);
  EXPECT_EQ(r2.event, WaypointEvent::kReplanning)
      << "elapsed=2.001s: should trigger kReplanning (> stuckTimeoutSec=2.0)";
  EXPECT_EQ(tracker.replanCount(), 1)
      << "first stuck should yield replanCount=1";
}

// ═══════════════════════════════════════════════════
//  TEST 4: PctAdapter — replanCooldownSec=3 防抖
//
//  第1次 stuck → 触发 replan (计数=1)
//  立即再次 stuck → 冷却期内, 不叠加 (计数仍=1)
//  等冷却期后再 stuck → 计数=2
// ═══════════════════════════════════════════════════

TEST(PctAdapter, ReplanCooldownDebounce) {
  WaypointTrackerParams p;
  p.stuckTimeoutSec   = 2.0;
  p.replanCooldownSec = 3.0;
  p.maxReplanCount    = 5;     // 充足余量
  p.waypointDistance  = 0.5;
  p.arrivalThreshold  = 0.5;
  p.searchWindow      = 5;

  WaypointTracker tracker(p);

  Path path;
  for (int i = 0; i < 5; ++i) {
    path.push_back(makePose(static_cast<double>(i + 20), 0, 0));
  }
  // setPath at t=100 → lastProgressTime_=100, lastReplanTime_=-1
  tracker.setPath(path, 100.0);

  Vec3 robotPos{0, 0, 0};
  std::vector<Vec3> emptyOdom;

  // ── 第1次 stuck: t=103 → elapsed=3 > 2 ✓, cooldown: 103-(-1)=104 > 3 ✓ ──
  // → kReplanning, replanCount=1
  // After replan: lastProgressTime_=103, lastReplanTime_=103
  auto r1 = tracker.update(robotPos, emptyOdom, 103.0);
  EXPECT_EQ(r1.event, WaypointEvent::kReplanning)
      << "t=103: first stuck should trigger replan";
  EXPECT_EQ(tracker.replanCount(), 1);

  // ── 立即再次 stuck: t=106 → elapsed=106-103=3 > 2 ✓ ──
  // BUT cooldown: 106-103=3 → NOT > 3 → 冷却期内, 不触发
  auto r2 = tracker.update(robotPos, emptyOdom, 106.0);
  EXPECT_NE(r2.event, WaypointEvent::kReplanning)
      << "t=106: cooldown not expired (106-103=3, need >3), should NOT replan";
  EXPECT_EQ(tracker.replanCount(), 1)
      << "replanCount should still be 1 (debounced)";

  // ── 等冷却期后: t=107 → elapsed=107-103=4 > 2 ✓, cooldown: 107-103=4 > 3 ✓ ──
  // → kReplanning, replanCount=2
  auto r3 = tracker.update(robotPos, emptyOdom, 107.0);
  EXPECT_EQ(r3.event, WaypointEvent::kReplanning)
      << "t=107: cooldown expired (107-103=4 > 3), second replan should fire";
  EXPECT_EQ(tracker.replanCount(), 2)
      << "replanCount should be 2 after second replan";
}
