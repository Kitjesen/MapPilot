/**
 * test_param_sensitivity.cpp 鈥?nav_kernel 鍙傛暟鏁忔劅鎬ф祴璇?
 *
 * 楠岃瘉鍏抽敭鍙傛暟鍦ㄨ竟鐣屽€?鎵弿鍊间笅鐨勮涓猴紝闃叉璋冨弬寮曞叆鍥炲綊銆?
 *
 * 娴嬭瘯瑕嗙洊:
 *   TEST(PathFollower, DirDiffThreVariation)  鈥?灏?澶?dirDiffThre 鍏堣浆鍐嶈蛋 vs 杈硅蛋杈硅浆
 *   TEST(LocalPlanner, SlopeWeightImpact)     鈥?slopeWeight=0 vs 5 瀵瑰潯搴﹁瘎鍒嗗奖鍝?
 *   TEST(PctAdapter,  StuckBoundaryExact)     鈥?stuckTimeoutSec=2 绮剧‘杈圭晫
 *   TEST(PctAdapter,  ReplanCooldownDebounce) 鈥?replanCooldownSec=3 鍐峰嵈闃叉姈
 */

#include <gtest/gtest.h>
#include "nav_kernel/path_follower_core.hpp"
#include "local_planner_scoring.hpp"
#include "nav_kernel/pct_adapter_core.hpp"
#include <cmath>

using namespace nav_kernel;

// MSVC 鑱氬悎鍒濆鍖栬緟鍔?
static Pose makePose(double x, double y, double z, double yaw = 0.0) {
  Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.yaw = yaw;
  return p;
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  TEST 1: PathFollower 鈥?dirDiffThre 鍙傛暟鏁忔劅鎬?
//
//  鍦烘櫙锛氭満鍣ㄤ汉鍦ㄥ師鐐癸紝鐩爣璺偣鍦ㄦ鍙虫柟 (x=2, y=0)锛?
//  浣嗘満鍣ㄤ汉褰撳墠鏈濆悜鍋忓樊 90掳锛堥潰鏈?y 杞存柟鍚戯級銆?
//
//  灏忛槇鍊?(0.1 rad): dirDiff > thre 鈫?canAccel=false 鈫?vx鈮?, |wz|>0 (鍏堣浆鍐嶈蛋)
//  澶ч槇鍊?(1.5 rad): dirDiff < thre 鈫?canAccel=true  鈫?vx>0 (杈硅蛋杈硅浆)
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

TEST(PathFollower, DirDiffThreVariation) {
  // 鍏辩敤璺緞: 娌?+X 鏂瑰悜, 浠?(0.5,0,0) 鍒?(3,0,0)
  std::vector<Vec3> path = {{0.5, 0, 0}, {1.0, 0, 0}, {2.0, 0, 0}, {3.0, 0, 0}};
  Vec3 robot{0, 0, 0};
  double yawDiff = M_PI / 2;  // 90掳 鏈濆悜鍋忓樊

  // 鈹€鈹€ 灏忛槇鍊? dirDiffThre = 0.1 rad (~5.7掳) 鈹€鈹€
  {
    PathFollowerParams p;
    p.dirDiffThre      = 0.1;
    p.maxSpeed         = 1.0;
    p.maxAccel         = 1.0;
    p.baseLookAheadDis = 0.3;
    p.lookAheadRatio   = 0.5;
    p.stopDisThre      = 0.2;
    p.omniDirGoalThre  = 0.1;   // 灏忓€? 浣?omni 渚嬪涓嶇敓鏁?(dis >> 0.1)
    p.omniDirDiffThre  = 1.5;
    p.twoWayDrive      = false;
    p.noRotAtGoal      = false;

    PathFollowerState state;
    auto out = computeControl(robot, yawDiff, path,
                              1.0, 0.0, 1.0, 0, p, state);

    // 90掳 >> 0.1 rad, dis >> omniDirGoalThre 鈫?canAccel = false
    EXPECT_FALSE(out.canAccel)
        << "dirDiffThre=0.1: 90deg error should block linear accel";
    // vx 搴旀帴杩?0 (vehicleSpeed 浠?0 寮€濮? canAccel=false 鈫?瓒嬪悜 0)
    EXPECT_NEAR(out.cmd.vx, 0.0, 0.01)
        << "dirDiffThre=0.1: vx should be ~0 (turn in place first)";
    // wz 搴旈潪闆?(杞悜淇)
    EXPECT_GT(std::fabs(out.cmd.wz), 0.0)
        << "dirDiffThre=0.1: |wz| should be >0 (turning to correct heading)";
  }

  // 鈹€鈹€ 澶ч槇鍊? dirDiffThre = 1.6 rad (~92掳, > PI/2) 鈹€鈹€
  {
    PathFollowerParams p;
    p.dirDiffThre      = 1.6;   // 92掳 > 90掳 鈫?鍏佽杈硅蛋杈硅浆
    p.maxSpeed         = 1.0;
    p.maxAccel         = 10.0;  // step=0.1, threshold=0.1; 2娆¤皟鐢ㄥ悗 speed=0.2 > 0.1
    p.baseLookAheadDis = 0.3;
    p.lookAheadRatio   = 0.5;
    p.stopDisThre      = 0.2;
    p.omniDirGoalThre  = 0.1;
    p.omniDirDiffThre  = 1.5;
    p.twoWayDrive      = false;
    p.noRotAtGoal      = false;

    PathFollowerState state;
    // 璋冧袱娆¤ vehicleSpeed 绉垎 > maxAccel/100 闃堝€?
    computeControl(robot, yawDiff, path, 1.0, 0.0, 1.0, 0, p, state);
    auto out = computeControl(robot, yawDiff, path, 1.0, 0.0, 1.0, 0, p, state);

    // 90掳 鈮?1.571 rad < 1.6 rad 鈫?canAccel = true (鍏佽杈硅蛋杈硅浆)
    EXPECT_TRUE(out.canAccel)
        << "dirDiffThre=1.6: 90deg (1.571) < 1.6 rad should allow accel";
    // vx > 0 (omni 鍒嗚В: vx = cos(dirDiff)*speed)
    EXPECT_GT(out.cmd.vx, 0.0)
        << "dirDiffThre=1.6: vx should be >0 (moving while turning)";
  }

  // 鈹€鈹€ 鍗曡皟鎬? 鏇村ぇ鐨?dirDiffThre 鍏佽鏇村ぇ鐨勮搴﹁宸€氳繃 鈹€鈹€
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

    // thre = 0.1 鈫?0.12 > 0.1 鈫?blocked
    p.dirDiffThre = 0.1;
    PathFollowerState s1;
    auto o1 = computeControl(robot, angle, path, 1.0, 0.0, 1.0, 0, p, s1);
    EXPECT_FALSE(o1.canAccel) << "dirDiffThre=0.1, angle=0.12 鈫?blocked";

    // thre = 0.2 鈫?0.12 < 0.2 鈫?allowed
    p.dirDiffThre = 0.2;
    PathFollowerState s2;
    auto o2 = computeControl(robot, angle, path, 1.0, 0.0, 1.0, 0, p, s2);
    EXPECT_TRUE(o2.canAccel) << "dirDiffThre=0.2, angle=0.12 鈫?allowed";
  }
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  TEST 2: LocalPlanner 鈥?slopeWeight 鍙傛暟鏁忔劅鎬?
//
//  鏂瑰悜A锛堝钩鍧︼級锛歞irDiff=0.1 rad, slopePenalty=0.0
//  鏂瑰悜B锛堟湁鍧★級锛歞irDiff=0.1 rad, slopePenalty=1.0
//
//  slopeWeight=0: 涓や釜鏂瑰悜寰楀垎鎺ヨ繎 (鍧″害涓嶅奖鍝?
//  slopeWeight=5: 鏂瑰悜A寰楀垎 > 鏂瑰悜B寰楀垎 (鍧″害鏄庢樉褰卞搷)
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

TEST(LocalPlanner, SlopeWeightImpact) {
  const double dirDiffDeg      = 5.73;  // ~0.1 rad 鈫?angDiffDeg format
  const double rotDirW         = 1.0;
  const double groupDirW       = 1.0;
  const double relativeGoalDis = 10.0;  // 杩滅鐩爣, 璧?rotDirW 鍒嗘敮

  const double slopePenaltyA = 0.0;  // 骞冲潶
  const double slopePenaltyB = 1.0;  // 鏈€澶у潯搴?

  PathScoreParams p;
  p.dirWeight       = 0.02;
  p.omniDirGoalThre = 5.0;

  // 鈹€鈹€ slopeWeight = 0: 鍧″害涓嶅奖鍝?鈫?涓や釜鏂瑰悜寰楀垎鐩稿悓 鈹€鈹€
  p.slopeWeight = 0.0;
  double scoreA_sw0 = scorePath(dirDiffDeg, rotDirW, groupDirW,
                                slopePenaltyA, relativeGoalDis, p);
  double scoreB_sw0 = scorePath(dirDiffDeg, rotDirW, groupDirW,
                                slopePenaltyB, relativeGoalDis, p);

  EXPECT_NEAR(scoreA_sw0, scoreB_sw0, 1e-9)
      << "slopeWeight=0: terrain penalty should have no effect";
  EXPECT_GT(scoreA_sw0, 0.0)
      << "slopeWeight=0: scores should be positive";

  // 鈹€鈹€ slopeWeight = 5: 鍧″害鏄庢樉褰卞搷 鈫?A > B 鈹€鈹€
  p.slopeWeight = 5.0;
  double scoreA_sw5 = scorePath(dirDiffDeg, rotDirW, groupDirW,
                                slopePenaltyA, relativeGoalDis, p);
  double scoreB_sw5 = scorePath(dirDiffDeg, rotDirW, groupDirW,
                                slopePenaltyB, relativeGoalDis, p);

  // 鏂瑰悜A (骞冲潶): terrainFactor = max(0, 1-5*0) = 1.0 鈫?婊″垎
  EXPECT_GT(scoreA_sw5, 0.0)
      << "slopeWeight=5, flat: score should be positive";
  // 鏂瑰悜B (鍧?: terrainFactor = max(0, 1-5*1.0) = max(0, -4) = 0 鈫?鍒嗘暟=0
  EXPECT_NEAR(scoreB_sw5, 0.0, 1e-9)
      << "slopeWeight=5, steep slope: score should be 0 (clamped)";
  // A 鏄庢樉浼樹簬 B
  EXPECT_GT(scoreA_sw5, scoreB_sw5)
      << "slopeWeight=5: flat path should score higher than steep path";

  // 鈹€鈹€ 楠岃瘉鍗曡皟鎬? slopeWeight 瓒婂ぇ, 鏈夊潯鏂瑰悜寰楀垎瓒婁綆 鈹€鈹€
  p.slopeWeight = 1.0;
  double scoreB_sw1 = scorePath(dirDiffDeg, rotDirW, groupDirW,
                                slopePenaltyB, relativeGoalDis, p);
  // terrainFactor(sw=1) = max(0, 1-1*1.0) = 0 鈫?宸茬粡鏄?
  // 鐢ㄤ腑绛夊潯搴﹂獙璇?
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

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  TEST 3: PctAdapter 鈥?stuckTimeoutSec=2 绮剧‘杈圭晫
//
//  鍋滅暀 2.0s (鎭板ソ鍒版湡) 鈫?搴旇Е鍙?stuck
//  鍋滅暀 1.9s (宸?.1绉? 鈫?涓嶅簲瑙﹀彂 stuck
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

TEST(PctAdapter, StuckBoundaryExact) {
  WaypointTrackerParams p;
  p.stuckTimeoutSec   = 2.0;
  p.replanCooldownSec = 0.001;  // 鏋佸皬鍐峰嵈, 涓嶅共鎵拌竟鐣屾祴璇?
  p.maxReplanCount    = 3;
  p.waypointDistance  = 0.5;
  p.arrivalThreshold  = 0.5;
  p.searchWindow      = 5;

  WaypointTracker tracker(p);

  // 璺緞杩滅鏈哄櫒浜?(x=10..14), 鏈哄櫒浜哄湪鍘熺偣 鈫?姘歌繙涓嶅埌杈?
  Path path;
  for (int i = 0; i < 5; ++i) {
    path.push_back(makePose(static_cast<double>(i + 10), 0, 0));
  }
  // setPath at t=100 鈫?lastProgressTime_=100 > 0 (stuck detection enabled)
  tracker.setPath(path, 100.0);

  Vec3 robotPos{0, 0, 0};
  std::vector<Vec3> emptyOdom;

  // 鈹€鈹€ t=101.9: elapsed = 1.9s < timeout=2.0s 鈫?涓嶈Е鍙?stuck 鈹€鈹€
  auto r1 = tracker.update(robotPos, emptyOdom, 101.9);
  EXPECT_NE(r1.event, WaypointEvent::kReplanning)
      << "elapsed=1.9s: should NOT trigger stuck (timeout=2.0s)";
  EXPECT_NE(r1.event, WaypointEvent::kStuckFinal)
      << "elapsed=1.9s: should NOT be stuck final";

  // 鈹€鈹€ t=102.001: elapsed = 2.001s > timeout=2.0s 鈫?瑙﹀彂绗?娆￠噸瑙勫垝 鈹€鈹€
  auto r2 = tracker.update(robotPos, emptyOdom, 102.001);
  EXPECT_EQ(r2.event, WaypointEvent::kReplanning)
      << "elapsed=2.001s: should trigger kReplanning (> stuckTimeoutSec=2.0)";
  EXPECT_EQ(tracker.replanCount(), 1)
      << "first stuck should yield replanCount=1";
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  TEST 4: PctAdapter 鈥?replanCooldownSec=3 闃叉姈
//
//  绗?娆?stuck 鈫?瑙﹀彂 replan (璁℃暟=1)
//  绔嬪嵆鍐嶆 stuck 鈫?鍐峰嵈鏈熷唴, 涓嶅彔鍔?(璁℃暟浠?1)
//  绛夊喎鍗存湡鍚庡啀 stuck 鈫?璁℃暟=2
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

TEST(PctAdapter, ReplanCooldownDebounce) {
  WaypointTrackerParams p;
  p.stuckTimeoutSec   = 2.0;
  p.replanCooldownSec = 3.0;
  p.maxReplanCount    = 5;     // 鍏呰冻浣欓噺
  p.waypointDistance  = 0.5;
  p.arrivalThreshold  = 0.5;
  p.searchWindow      = 5;

  WaypointTracker tracker(p);

  Path path;
  for (int i = 0; i < 5; ++i) {
    path.push_back(makePose(static_cast<double>(i + 20), 0, 0));
  }
  // setPath at t=100 鈫?lastProgressTime_=100, lastReplanTime_=-1
  tracker.setPath(path, 100.0);

  Vec3 robotPos{0, 0, 0};
  std::vector<Vec3> emptyOdom;

  // 鈹€鈹€ 绗?娆?stuck: t=103 鈫?elapsed=3 > 2 鉁? cooldown: 103-(-1)=104 > 3 鉁?鈹€鈹€
  // 鈫?kReplanning, replanCount=1
  // After replan: lastProgressTime_=103, lastReplanTime_=103
  auto r1 = tracker.update(robotPos, emptyOdom, 103.0);
  EXPECT_EQ(r1.event, WaypointEvent::kReplanning)
      << "t=103: first stuck should trigger replan";
  EXPECT_EQ(tracker.replanCount(), 1);

  // 鈹€鈹€ 绔嬪嵆鍐嶆 stuck: t=106 鈫?elapsed=106-103=3 > 2 鉁?鈹€鈹€
  // BUT cooldown: 106-103=3 鈫?NOT > 3 鈫?鍐峰嵈鏈熷唴, 涓嶈Е鍙?
  auto r2 = tracker.update(robotPos, emptyOdom, 106.0);
  EXPECT_NE(r2.event, WaypointEvent::kReplanning)
      << "t=106: cooldown not expired (106-103=3, need >3), should NOT replan";
  EXPECT_EQ(tracker.replanCount(), 1)
      << "replanCount should still be 1 (debounced)";

  // 鈹€鈹€ 绛夊喎鍗存湡鍚? t=107 鈫?elapsed=107-103=4 > 2 鉁? cooldown: 107-103=4 > 3 鉁?鈹€鈹€
  // 鈫?kReplanning, replanCount=2
  auto r3 = tracker.update(robotPos, emptyOdom, 107.0);
  EXPECT_EQ(r3.event, WaypointEvent::kReplanning)
      << "t=107: cooldown expired (107-103=4 > 3), second replan should fire";
  EXPECT_EQ(tracker.replanCount(), 2)
      << "replanCount should be 2 after second replan";
}
