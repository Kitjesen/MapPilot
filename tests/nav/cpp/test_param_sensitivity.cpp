/**
 * test_param_sensitivity.cpp 鈥?nav_kernel 鍙傛暟鏁忔劅鎬ф祴璇?
 *
 * 楠岃瘉鍏抽敭鍙傛暟鍦ㄨ竟鐣屽€?鎵弿鍊间笅鐨勮涓猴紝闃叉璋冨弬寮曞叆鍥炲綊銆?
 *
 * 娴嬭瘯瑕嗙洊:
 *   TEST(PathFollower, HeadingAlignEnterVariation)
 *   TEST(WaypointTracker,  StuckBoundaryExact)     鈥?stuckTimeoutSec=2 绮剧‘杈圭晫
 *   TEST(WaypointTracker,  ReplanCooldownDebounce) 鈥?replanCooldownSec=3 鍐峰嵈闃叉姈
 */

#include <gtest/gtest.h>
#include "planning/local/planner.hpp"
#include "tracking/follower.hpp"
#include "nav_kernel/waypoint_helpers_core.hpp"
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

static FollowerOutput followPath(
    Follower& follower,
    const Vec3& vehicle,
    double yaw,
    const std::vector<Vec3>& path,
    double requested_speed,
    double time,
    double slow_factor,
    int safety_stop,
    const FollowerParams& params) {
  FollowerState state;
  state.vehicleRelative = vehicle;
  state.vehicleYawRelative = yaw;
  state.requestedSpeed = requested_speed;
  state.currentTime = time;
  state.slowFactor = slow_factor;
  state.safetyStop = safety_stop;
  state.params = params;
  state.standardPathProfile = false;
  return follower.follow(LocalPlan::path(path), state);
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  TEST 1: PathFollower heading-alignment enter threshold sensitivity
//
//  鍦烘櫙锛氭満鍣ㄤ汉鍦ㄥ師鐐癸紝鐩爣璺偣鍦ㄦ鍙虫柟 (x=2, y=0)锛?
//  浣嗘満鍣ㄤ汉褰撳墠鏈濆悜鍋忓樊 90掳锛堥潰鏈?y 杞存柟鍚戯級銆?
//
//  Small enter angle freezes translation; a larger one permits coupled motion.
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

TEST(PathFollower, HeadingAlignEnterVariation) {
  // 鍏辩敤璺緞: 娌?+X 鏂瑰悜, 浠?(0.5,0,0) 鍒?(3,0,0)
  std::vector<Vec3> path = {{0.5, 0, 0}, {1.0, 0, 0}, {2.0, 0, 0}, {3.0, 0, 0}};
  Vec3 robot{0, 0, 0};
  double yawDiff = M_PI / 2;  // 90掳 鏈濆悜鍋忓樊

  // Small heading-alignment enter angle: 0.1 rad.
  {
    FollowerParams p;
    p.headingAlignEnterRad = 0.1;
    p.headingAlignExitRad = 0.05;
    p.maxSpeed         = 1.0;
    p.maxAccel         = 1.0;
    p.baseLookAheadDis = 0.3;
    p.lookAheadRatio   = 0.5;
    p.stopDisThre      = 0.2;
    p.omniDirGoalThre  = 0.1;   // 灏忓€? 浣?omni 渚嬪涓嶇敓鏁?(dis >> 0.1)
    p.omniDirDiffThre  = 1.5;
    p.twoWayDrive      = false;
    p.noRotAtGoal      = false;

    Follower follower;
    auto out = followPath(
        follower, robot, yawDiff, path, 1.0, 0.0, 1.0, 0, p);

    // 90掳 >> 0.1 rad, dis >> omniDirGoalThre 鈫?canAccel = false
    EXPECT_FALSE(out.canAccelerate)
        << "headingAlignEnterRad=0.1: 90deg error should block linear accel";
    // vx 搴旀帴杩?0 (vehicleSpeed 浠?0 寮€濮? canAccel=false 鈫?瓒嬪悜 0)
    EXPECT_NEAR(out.cmd.vx, 0.0, 0.01)
        << "headingAlignEnterRad=0.1: vx should be ~0 (turn in place first)";
    // wz 搴旈潪闆?(杞悜淇)
    EXPECT_GT(std::fabs(out.cmd.wz), 0.0)
        << "headingAlignEnterRad=0.1: |wz| should be >0";
  }

  // Large heading-alignment enter angle: 1.6 rad (> pi/2).
  {
    FollowerParams p;
    p.headingAlignEnterRad = 1.6;
    p.headingAlignExitRad = 0.8;
    p.maxSpeed         = 1.0;
    p.maxAccel         = 10.0;
    p.nominalDt        = 0.01;
    p.baseLookAheadDis = 0.3;
    p.lookAheadRatio   = 0.5;
    p.stopDisThre      = 0.2;
    p.omniDirGoalThre  = 0.1;
    p.omniDirDiffThre  = 1.5;
    p.twoWayDrive      = false;
    p.noRotAtGoal      = false;

    Follower follower;
    // Two 10 ms updates integrate speed to 0.2 m/s at 10 m/s^2.
    followPath(follower, robot, yawDiff, path, 1.0, 0.0, 1.0, 0, p);
    auto out = followPath(
        follower, robot, yawDiff, path, 1.0, 0.01, 1.0, 0, p);

    // 90掳 鈮?1.571 rad < 1.6 rad 鈫?canAccel = true (鍏佽杈硅蛋杈硅浆)
    EXPECT_TRUE(out.canAccelerate)
        << "headingAlignEnterRad=1.6 should allow this 90deg error";
    // vx > 0 (omni 鍒嗚В: vx = cos(dirDiff)*speed)
    EXPECT_GT(out.cmd.vx, 0.0)
        << "headingAlignEnterRad=1.6 should permit translation";
  }

  // A larger enter angle admits a larger heading error.
  {
    FollowerParams p;
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
    p.headingAlignEnterRad = 0.1;
    p.headingAlignExitRad = 0.05;
    Follower first;
    auto o1 = followPath(
        first, robot, angle, path, 1.0, 0.0, 1.0, 0, p);
    EXPECT_FALSE(o1.canAccelerate)
        << "headingAlignEnterRad=0.1 must block angle=0.12";

    // thre = 0.2 鈫?0.12 < 0.2 鈫?allowed
    p.headingAlignEnterRad = 0.2;
    p.headingAlignExitRad = 0.1;
    Follower second;
    auto o2 = followPath(
        second, robot, angle, path, 1.0, 0.0, 1.0, 0, p);
    EXPECT_TRUE(o2.canAccelerate)
        << "headingAlignEnterRad=0.2 must allow angle=0.12";
  }
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  TEST 2: WaypointTracker 鈥?stuckTimeoutSec=2 绮剧‘杈圭晫
//
//  鍋滅暀 2.0s (鎭板ソ鍒版湡) 鈫?搴旇Е鍙?stuck
//  鍋滅暀 1.9s (宸?.1绉? 鈫?涓嶅簲瑙﹀彂 stuck
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

TEST(WaypointTracker, StuckBoundaryExact) {
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
//  TEST 4: WaypointTracker 鈥?replanCooldownSec=3 闃叉姈
//
//  绗?娆?stuck 鈫?瑙﹀彂 replan (璁℃暟=1)
//  绔嬪嵆鍐嶆 stuck 鈫?鍐峰嵈鏈熷唴, 涓嶅彔鍔?(璁℃暟浠?1)
//  绛夊喎鍗存湡鍚庡啀 stuck 鈫?璁℃暟=2
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

TEST(WaypointTracker, ReplanCooldownDebounce) {
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
