/**
 * test_path_edge_cases.cpp — nav_core edge case / boundary tests
 *
 * Iter 13: PathFollower edge cases (empty, single, duplicate, long, NaN, clamp)
 * Iter 14: LocalPlanner edge cases (zero candidates, all blocked, negative slopeWeight, extreme velocity)
 * Iter 15: PctAdapter state machine (IDLE->TRACKING->REACHED, reset, empty, goalTolerance=0)
 */

#include <gtest/gtest.h>
#include "nav_core/path_follower_core.hpp"
#include "nav_core/local_planner_core.hpp"
#include "nav_core/pct_adapter_core.hpp"
#include <cmath>
#include <limits>
#include <vector>

using namespace nav_core;

// MSVC aggregate initialization helper
static Pose makePose(double x, double y, double z, double yaw = 0.0) {
  Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.yaw = yaw;
  return p;
}

// ═══════════════════════════════════════════════════
//  Iter 13: PathFollower Edge Cases
// ═══════════════════════════════════════════════════

TEST(PathFollower, EmptyPath) {
  PathFollowerParams p;
  PathFollowerState state;
  std::vector<Vec3> path;
  Vec3 robot{1.0, 2.0, 0.0};

  auto out = computeControl(robot, 0.5, path, 1.0, 0.0, 1.0, 0, p, state);

  EXPECT_DOUBLE_EQ(out.cmd.vx, 0.0) << "Empty path: vx must be zero";
  EXPECT_DOUBLE_EQ(out.cmd.vy, 0.0) << "Empty path: vy must be zero";
  EXPECT_DOUBLE_EQ(out.cmd.wz, 0.0) << "Empty path: wz must be zero";
  EXPECT_FALSE(out.canAccel) << "Empty path: canAccel must be false";
}

TEST(PathFollower, SinglePointPath) {
  PathFollowerParams p;
  p.stopDisThre = 0.2;
  p.noRotAtGoal = true;
  PathFollowerState state;

  // Single point 2m ahead
  std::vector<Vec3> path = {{2.0, 0.0, 0.0}};
  Vec3 robot{0.0, 0.0, 0.0};

  auto out = computeControl(robot, 0.0, path, 1.0, 0.0, 1.0, 0, p, state);

  // pathSize <= 1 -> joySpeed2 = 0, yawRate = 0
  EXPECT_DOUBLE_EQ(out.cmd.vx, 0.0)
      << "Single point path: vx should be 0 (pathSize<=1 branch)";
  EXPECT_DOUBLE_EQ(out.cmd.wz, 0.0)
      << "Single point path: wz should be 0 (noRotAtGoal + pathSize<=1)";
  // endDis should be computed correctly
  EXPECT_NEAR(out.endDis, 2.0, 0.01)
      << "Single point path: endDis should be distance to the single point";
}

TEST(PathFollower, DuplicatePoints) {
  PathFollowerParams p;
  p.maxAccel = 100.0;
  p.dirDiffThre = 0.5;
  p.baseLookAheadDis = 0.3;
  p.lookAheadRatio = 0.5;
  p.stopDisThre = 0.05;
  PathFollowerState state;

  // Path with consecutive duplicate points
  std::vector<Vec3> path = {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},  // duplicate
      {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0},  // duplicate
      {2.0, 0.0, 0.0}
  };
  Vec3 robot{0.0, 0.0, 0.0};

  // Must not crash or divide by zero
  auto out = computeControl(robot, 0.0, path, 1.0, 0.0, 1.0, 0, p, state);

  // The key check: no NaN or Inf in output
  EXPECT_FALSE(std::isnan(out.cmd.vx)) << "Duplicate points: vx must not be NaN";
  EXPECT_FALSE(std::isnan(out.cmd.vy)) << "Duplicate points: vy must not be NaN";
  EXPECT_FALSE(std::isnan(out.cmd.wz)) << "Duplicate points: wz must not be NaN";
  EXPECT_FALSE(std::isinf(out.cmd.vx)) << "Duplicate points: vx must not be Inf";
  EXPECT_FALSE(std::isinf(out.cmd.vy)) << "Duplicate points: vy must not be Inf";
  EXPECT_FALSE(std::isinf(out.cmd.wz)) << "Duplicate points: wz must not be Inf";
}

TEST(PathFollower, VeryLongPath) {
  PathFollowerParams p;
  p.maxAccel = 100.0;
  p.dirDiffThre = 0.5;
  p.baseLookAheadDis = 0.3;
  p.lookAheadRatio = 0.5;
  PathFollowerState state;

  // 10000-point path along +X
  std::vector<Vec3> path;
  path.reserve(10000);
  for (int i = 0; i < 10000; i++) {
    Vec3 pt;
    pt.x = 0.01 * i;
    pt.y = 0.0;
    pt.z = 0.0;
    path.push_back(pt);
  }
  Vec3 robot{0.0, 0.0, 0.0};

  // Must complete without timeout
  auto out = computeControl(robot, 0.0, path, 1.0, 0.0, 1.0, 0, p, state);

  EXPECT_FALSE(std::isnan(out.cmd.vx)) << "Long path: vx must not be NaN";
  EXPECT_GT(out.endDis, 90.0) << "Long path: endDis should be ~99m";
}

TEST(PathFollower, NaNPosition) {
  PathFollowerParams p;
  PathFollowerState state;

  std::vector<Vec3> path = {{1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}};
  Vec3 robot;
  robot.x = std::numeric_limits<double>::quiet_NaN();
  robot.y = 0.0;
  robot.z = 0.0;

  // Must not crash; NaN propagation is acceptable but no segfault
  auto out = computeControl(robot, 0.0, path, 1.0, 0.0, 1.0, 0, p, state);

  // NaN input -> NaN output is acceptable (safety filter at ROS2 shell layer)
  // The key assertion: no crash, test completes
  (void)out;
  SUCCEED() << "NaN position: did not crash";
}

TEST(PathFollower, MaxSpeedClamp) {
  PathFollowerParams p;
  p.maxSpeed = 0.5;       // Low max speed
  p.maxAccel = 10000.0;   // Instant acceleration (step = 100)
  p.dirDiffThre = 2.0;    // Allow any heading
  p.baseLookAheadDis = 0.3;
  p.lookAheadRatio = 0.5;
  p.stopDisThre = 0.1;
  p.slowDwnDisThre = 100.0;  // No slowdown
  PathFollowerState state;

  // Long straight path
  std::vector<Vec3> path;
  for (int i = 0; i <= 100; i++) {
    Vec3 pt;
    pt.x = 0.1 * i;
    pt.y = 0.0;
    pt.z = 0.0;
    path.push_back(pt);
  }
  Vec3 robot{0.0, 0.0, 0.0};

  // Run multiple iterations to let speed build up
  for (int i = 0; i < 20; i++) {
    computeControl(robot, 0.0, path, 1.0, static_cast<double>(i), 1.0, 0, p, state);
  }

  // vehicleSpeed should be clamped by joySpeed3 = maxSpeed * joySpeed * slowFactor = 0.5
  EXPECT_LE(std::fabs(state.vehicleSpeed), p.maxSpeed + 0.01)
      << "Max speed clamp: vehicleSpeed should not exceed maxSpeed";
}

// ═══════════════════════════════════════════════════
//  Iter 14: LocalPlanner Edge Cases
// ═══════════════════════════════════════════════════

TEST(LocalPlanner, ZeroCandidates) {
  // Empty group scores
  std::vector<double> scores;
  int groupNum = 7;

  auto result = selectBestGroup(scores, groupNum, -180.0, 180.0, true, false);

  EXPECT_EQ(result.selectedGroupID, -1)
      << "Zero candidates: selectedGroupID should be -1";
  EXPECT_DOUBLE_EQ(result.maxScore, 0.0)
      << "Zero candidates: maxScore should be 0";
}

TEST(LocalPlanner, AllBlockedPaths) {
  // All scores negative (large angle penalty) — simulates all paths blocked
  int groupNum = 7;
  std::vector<double> scores(36 * groupNum, -10.0);

  // With checkRotObstacle=false, all rotation angles are allowed
  // But all scores are negative, so maxScore stays at 0 and selectedGroupID = -1
  auto result = selectBestGroup(scores, groupNum, -180.0, 180.0, true, false);

  // When all scores are negative, selectBestGroup returns -1 (no path > 0)
  EXPECT_EQ(result.selectedGroupID, -1)
      << "All blocked: no path should be selected when all scores negative";

  // Now test with one path slightly less bad
  scores[18 * groupNum + 3] = -0.5;  // Still negative, still no selection
  result = selectBestGroup(scores, groupNum, -180.0, 180.0, true, false);
  EXPECT_EQ(result.selectedGroupID, -1)
      << "All blocked: negative score should not be selected (maxScore init = 0)";
}

TEST(LocalPlanner, NegativeSlopeWeight) {
  PathScoreParams p;
  p.dirWeight = 0.02;

  // Negative slopeWeight: the code uses `if (slopeWeight > 0.0)` guard,
  // so negative values effectively disable terrain penalty (treated as slopeWeight=0)
  p.slopeWeight = -5.0;
  double scoreNeg = scorePath(10.0, 5.0, 3.0, 0.5, 10.0, p);

  p.slopeWeight = 0.0;
  double scoreZero = scorePath(10.0, 5.0, 3.0, 0.5, 10.0, p);

  // Both should have terrainFactor = 1.0 (slope disabled)
  EXPECT_DOUBLE_EQ(scoreNeg, scoreZero)
      << "Negative slopeWeight should behave same as slopeWeight=0";
}

TEST(LocalPlanner, ExtremeVelocity) {
  // scorePath with extreme dirDiff values
  PathScoreParams p;
  p.dirWeight = 0.02;

  // Extremely large angle (999 degrees)
  double scoreExtreme = scorePath(999.0, 10.0, 4.0, 0.0, 10.0, p);
  // dw = 0.02 * 999 = 19.98, sqrt(sqrt(19.98)) ~ 2.115
  // (1 - 2.115) < 0 -> negative score (allowed by original C++ code)
  EXPECT_LT(scoreExtreme, 0.0)
      << "Extreme angle: score should be negative";
  EXPECT_FALSE(std::isnan(scoreExtreme))
      << "Extreme angle: score must not be NaN";
  EXPECT_FALSE(std::isinf(scoreExtreme))
      << "Extreme angle: score must not be Inf";

  // Zero rotDirW (edge case)
  double scoreZeroRot = scorePath(10.0, 0.0, 4.0, 0.0, 10.0, p);
  EXPECT_DOUBLE_EQ(scoreZeroRot, 0.0)
      << "Zero rotDirW: score should be 0 (0^4 = 0)";

  // computeRotDirW at boundary values
  EXPECT_GT(computeRotDirW(0), 0.0)
      << "computeRotDirW(0): should be positive";
  EXPECT_GT(computeRotDirW(35), 0.0)
      << "computeRotDirW(35): should be positive";

  // computeGroupDirW at extreme group indices
  EXPECT_GT(computeGroupDirW(0), 0.0);
  EXPECT_GT(computeGroupDirW(6), 0.0);
}

// ═══════════════════════════════════════════════════
//  Iter 15: PctAdapter State Machine Tests
// ═══════════════════════════════════════════════════

TEST(PctAdapter, StateTransitionSequence) {
  // IDLE -> TRACKING (via setPath) -> waypoint progression -> REACHED
  WaypointTrackerParams p;
  p.waypointDistance = 0.5;
  p.arrivalThreshold = 0.3;
  p.stuckTimeoutSec = 100.0;  // Won't trigger
  p.searchWindow = 5;

  WaypointTracker tracker(p);

  // Initially: no path, goalReached = false
  EXPECT_TRUE(tracker.path().empty()) << "Initial: path should be empty";
  EXPECT_FALSE(tracker.goalReached()) << "Initial: goal not reached";

  // Set a short path (3 waypoints after downsample)
  Path raw;
  raw.push_back(makePose(0.0, 0.0, 0.0));
  raw.push_back(makePose(1.0, 0.0, 0.0));
  raw.push_back(makePose(2.0, 0.0, 0.0));

  auto r1 = tracker.setPath(raw, 0.0);
  EXPECT_EQ(r1.event, WaypointEvent::kPathReceived)
      << "setPath: should emit kPathReceived";
  EXPECT_GT(r1.totalWaypoints, 0u);

  // Walk through each waypoint
  double t = 1.0;
  for (size_t i = 0; i < tracker.path().size(); i++) {
    Vec3 pos = tracker.path()[i].position;
    auto r = tracker.update(pos, {}, t);
    t += 1.0;

    if (tracker.goalReached()) {
      EXPECT_EQ(r.event, WaypointEvent::kGoalReached)
          << "Final waypoint: should emit kGoalReached";
      break;
    }
  }

  EXPECT_TRUE(tracker.goalReached())
      << "After walking all waypoints: goal should be reached";
}

TEST(PctAdapter, ResetMidTracking) {
  WaypointTrackerParams p;
  p.waypointDistance = 0.5;
  p.arrivalThreshold = 0.3;
  p.stuckTimeoutSec = 100.0;
  p.searchWindow = 5;

  WaypointTracker tracker(p);

  // Set initial path
  Path path1;
  for (int i = 0; i < 10; i++) {
    path1.push_back(makePose(0.5 * i, 0.0, 0.0));
  }
  tracker.setPath(path1, 0.0);

  // Progress partway
  Vec3 robot{0.0, 0.0, 0.0};
  tracker.update(robot, {}, 1.0);
  EXPECT_GE(tracker.currentIndex(), 1u) << "Should have progressed";

  // Reset with new path (mid-tracking)
  Path path2;
  for (int i = 0; i < 5; i++) {
    path2.push_back(makePose(0.0, 0.5 * i, 0.0));
  }
  auto r = tracker.setPath(path2, 10.0);

  EXPECT_EQ(r.event, WaypointEvent::kPathReceived)
      << "Reset: should emit kPathReceived";
  EXPECT_EQ(tracker.currentIndex(), 0u)
      << "Reset: currentIndex should be back to 0";
  EXPECT_FALSE(tracker.goalReached())
      << "Reset: goalReached should be false";
  EXPECT_EQ(tracker.replanCount(), 0)
      << "Reset: replanCount should be reset to 0";
}

TEST(PctAdapter, EmptyWaypoints) {
  WaypointTrackerParams p;
  WaypointTracker tracker(p);

  // Set empty path
  Path empty;
  auto r1 = tracker.setPath(empty, 0.0);
  EXPECT_EQ(r1.event, WaypointEvent::kPathReceived);

  // Update with empty path should not crash
  Vec3 robot{1.0, 2.0, 0.0};
  auto r2 = tracker.update(robot, {}, 1.0);

  EXPECT_EQ(r2.event, WaypointEvent::kNone)
      << "Empty waypoints: update should return kNone";
  EXPECT_FALSE(r2.hasTarget)
      << "Empty waypoints: should not have target";
  EXPECT_FALSE(tracker.goalReached())
      << "Empty waypoints: goal should not be reached";
}

TEST(PctAdapter, GoalToleranceZero) {
  WaypointTrackerParams p;
  p.waypointDistance = 0.5;
  p.arrivalThreshold = 0.0;  // Zero tolerance: never arrives
  p.stuckTimeoutSec = 100.0; // Won't trigger
  p.searchWindow = 5;

  WaypointTracker tracker(p);

  Path path;
  path.push_back(makePose(0.0, 0.0, 0.0));
  path.push_back(makePose(1.0, 0.0, 0.0));
  path.push_back(makePose(2.0, 0.0, 0.0));
  tracker.setPath(path, 0.0);

  // Stand exactly on the first waypoint
  Vec3 robot = tracker.path()[0].position;
  auto r = tracker.update(robot, {}, 1.0);

  // arrivalThreshold=0: distToTarget must be < 0 (impossible) -> never arrives
  // Distance 0.0 is NOT < 0.0
  EXPECT_NE(r.event, WaypointEvent::kWaypointReached)
      << "GoalTolerance=0: should never trigger arrival (0 < 0 is false)";
  EXPECT_NE(r.event, WaypointEvent::kGoalReached)
      << "GoalTolerance=0: should never reach goal";
  EXPECT_FALSE(tracker.goalReached())
      << "GoalTolerance=0: goalReached should remain false";
}
