/**
 * test_benchmark.cpp — nav_core performance throughput benchmarks
 *
 * Iter 16: Verify core algorithms meet real-time requirements.
 *   - PathFollower computeControl: 1000 iterations < 100ms
 *   - LocalPlanner scorePath: 1000 iterations < 100ms
 *   - PctAdapter update: 1000 iterations < 100ms
 *
 * Outputs average microseconds per call.
 */

#include <gtest/gtest.h>
#include "nav_core/path_follower_core.hpp"
#include "nav_core/local_planner_core.hpp"
#include "nav_core/pct_adapter_core.hpp"
#include <chrono>
#include <cmath>
#include <cstdio>
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

static constexpr int kIterations = 1000;
static constexpr int kMaxElapsedMs = 100;

// ═══════════════════════════════════════════════════
//  PathFollower Throughput
// ═══════════════════════════════════════════════════

TEST(Benchmark, PathFollowerThroughput) {
  PathFollowerParams p;
  p.maxSpeed = 1.0;
  p.maxAccel = 1.0;
  p.dirDiffThre = 0.1;
  p.baseLookAheadDis = 0.3;
  p.lookAheadRatio = 0.5;
  p.stopDisThre = 0.2;
  p.omniDirGoalThre = 1.0;
  p.omniDirDiffThre = 1.5;

  // 100-point path
  std::vector<Vec3> path;
  path.reserve(100);
  for (int i = 0; i < 100; i++) {
    Vec3 pt;
    pt.x = 0.1 * i;
    pt.y = 0.0;
    pt.z = 0.0;
    path.push_back(pt);
  }

  PathFollowerState state;
  Vec3 robot{0.0, 0.0, 0.0};

  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < kIterations; i++) {
    state = {};  // Reset state each iteration for consistency
    computeControl(robot, 0.0, path, 1.0, static_cast<double>(i) * 0.01,
                   1.0, 0, p, state);
  }
  auto end = std::chrono::high_resolution_clock::now();

  auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  auto elapsed_ms = elapsed_us / 1000.0;
  double avg_us = static_cast<double>(elapsed_us) / kIterations;

  std::printf("[BENCHMARK] PathFollower::computeControl: %d calls in %.1f ms "
              "(avg %.1f us/call)\n",
              kIterations, elapsed_ms, avg_us);

  EXPECT_LT(elapsed_ms, kMaxElapsedMs)
      << "PathFollower throughput: " << kIterations << " calls should complete within "
      << kMaxElapsedMs << " ms";
}

// ═══════════════════════════════════════════════════
//  LocalPlanner Scoring Throughput
// ═══════════════════════════════════════════════════

TEST(Benchmark, LocalPlannerScoring) {
  PathScoreParams p;
  p.dirWeight = 0.02;
  p.slopeWeight = 3.0;
  p.omniDirGoalThre = 5.0;

  auto start = std::chrono::high_resolution_clock::now();
  volatile double sink = 0;  // Prevent optimization
  for (int i = 0; i < kIterations; i++) {
    double dirDiff = static_cast<double>(i % 180);
    double rotDirW = computeRotDirW(i % 36);
    double groupDirW = computeGroupDirW(i % 7);
    double terrain = 0.1 * (i % 10);
    double goalDis = 1.0 + (i % 20);

    sink = scorePath(dirDiff, rotDirW, groupDirW, terrain, goalDis, p);
  }
  auto end = std::chrono::high_resolution_clock::now();

  auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  auto elapsed_ms = elapsed_us / 1000.0;
  double avg_us = static_cast<double>(elapsed_us) / kIterations;

  std::printf("[BENCHMARK] LocalPlanner::scorePath: %d calls in %.1f ms "
              "(avg %.1f us/call)\n",
              kIterations, elapsed_ms, avg_us);

  EXPECT_LT(elapsed_ms, kMaxElapsedMs)
      << "LocalPlanner scoring throughput: " << kIterations << " calls should complete within "
      << kMaxElapsedMs << " ms";

  (void)sink;
}

// ═══════════════════════════════════════════════════
//  PctAdapter Update Throughput
// ═══════════════════════════════════════════════════

TEST(Benchmark, PctAdapterUpdate) {
  WaypointTrackerParams p;
  p.waypointDistance = 0.5;
  p.arrivalThreshold = 0.3;
  p.stuckTimeoutSec = 1000.0;  // Won't trigger during benchmark
  p.searchWindow = 5;
  p.maxReplanCount = 2;
  p.replanCooldownSec = 5.0;

  WaypointTracker tracker(p);

  // 200-point path
  Path raw;
  raw.reserve(200);
  for (int i = 0; i < 200; i++) {
    raw.push_back(makePose(0.1 * i, 0.0, 0.0));
  }
  tracker.setPath(raw, 0.0);

  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < kIterations; i++) {
    Vec3 robot;
    robot.x = 0.01 * (i % 100);
    robot.y = 0.0;
    robot.z = 0.0;
    double t = static_cast<double>(i) * 0.01;
    tracker.update(robot, {}, t);
  }
  auto end = std::chrono::high_resolution_clock::now();

  auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  auto elapsed_ms = elapsed_us / 1000.0;
  double avg_us = static_cast<double>(elapsed_us) / kIterations;

  std::printf("[BENCHMARK] PctAdapter::update: %d calls in %.1f ms "
              "(avg %.1f us/call)\n",
              kIterations, elapsed_ms, avg_us);

  EXPECT_LT(elapsed_ms, kMaxElapsedMs)
      << "PctAdapter update throughput: " << kIterations << " calls should complete within "
      << kMaxElapsedMs << " ms";
}
