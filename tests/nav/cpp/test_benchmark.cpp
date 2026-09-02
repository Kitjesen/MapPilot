/**
 * test_benchmark.cpp 閳?nav_kernel performance throughput benchmarks
 *
 * Iter 16: Verify core algorithms meet real-time requirements.
 *   - Follower::follow: 1000 iterations < 100ms
 *   - WaypointTracker update: 1000 iterations < 100ms
 *
 * Outputs average microseconds per call.
 */

#include <gtest/gtest.h>
#include "tracking/follower.hpp"
#include "planning/local/planner.hpp"
#include "planner_fixture.hpp"

#include "nav_kernel/terrain_core.hpp"
#include "nav_kernel/waypoint_helpers_core.hpp"
#include "nav_kernel/simd_accel.hpp"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using namespace nav_kernel;

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

// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?
//  PathFollower Throughput
// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?

TEST(Benchmark, PathFollowerThroughput) {
  FollowerParams p;
  p.maxSpeed = 1.0;
  p.maxAccel = 1.0;
  p.headingAlignEnterRad = 0.1;
  p.headingAlignExitRad = 0.05;
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

  Follower follower;
  Vec3 robot{0.0, 0.0, 0.0};

  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < kIterations; i++) {
    follower.reset();
    FollowerState state;
    state.vehicleRelative = robot;
    state.currentTime = static_cast<double>(i) * 0.01;
    state.params = p;
    state.standardPathProfile = false;
    follower.follow(LocalPlan::path(path), state);
  }
  auto end = std::chrono::high_resolution_clock::now();

  auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  auto elapsed_ms = elapsed_us / 1000.0;
  double avg_us = static_cast<double>(elapsed_us) / kIterations;

  std::printf("[BENCHMARK] Follower::follow: %d calls in %.1f ms "
              "(avg %.1f us/call)\n",
              kIterations, elapsed_ms, avg_us);

  EXPECT_LT(elapsed_ms, kMaxElapsedMs)
      << "PathFollower throughput: " << kIterations << " calls should complete within "
      << kMaxElapsedMs << " ms";
}

// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?
//  LocalPlanner Scoring Throughput
// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?

TEST(Benchmark, WaypointTrackerUpdate) {
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

  std::printf("[BENCHMARK] WaypointTracker::update: %d calls in %.1f ms "
              "(avg %.1f us/call)\n",
              kIterations, elapsed_ms, avg_us);

  EXPECT_LT(elapsed_ms, kMaxElapsedMs)
      << "WaypointTracker update throughput: " << kIterations << " calls should complete within "
      << kMaxElapsedMs << " ms";
}

// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?
//  scorePathFast vs scorePath (LUT acceleration)
// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?

TEST(Benchmark, FullPlanCycle) {
  LocalPlannerParams lpp;
  lpp.adjacentRange = 3.5;
  lpp.checkObstacle = true;
  lpp.useTerrainAnalysis = true;
  lpp.dirWeight = 0.02;
  lpp.twoWayDrive = true;

  PlannerFixture planner(lpp);

  // Create synthetic path library 閳?just enough for the scoring loop
  // In real usage, loadPaths() reads from disk. Here we test plan() overhead.
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  // Generate 500 random obstacle points (typical outdoor scene)
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> posRange(-3.0f, 3.0f);
  std::uniform_real_distribution<float> zRange(-0.3f, 0.3f);

  std::vector<float> cloud(500 * 4);
  for (int i = 0; i < 500; i++) {
    cloud[i*4+0] = posRange(rng);  // x
    cloud[i*4+1] = posRange(rng);  // y
    cloud[i*4+2] = zRange(rng);    // z
    cloud[i*4+3] = std::fabs(zRange(rng));  // intensity/height
  }

  // Warm up (paths not loaded, so plan() returns early 閳?tests framework overhead)
  for (int i = 0; i < 10; i++)
    planner.plan(cloud.data(), 500, i * 0.1);

  auto t0 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < kIterations; i++) {
    planner.setVehicle(0.001 * i, 0, 0, 0.01 * (i % 10));
    planner.plan(cloud.data(), 500, i * 0.01);
  }
  auto t1 = std::chrono::high_resolution_clock::now();

  auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
  std::printf("[BENCHMARK] Full plan cycle (500 pts, no paths): %d calls in %lld us "
              "(avg %.1f us/call)\n",
              kIterations, static_cast<long long>(elapsed_us),
              static_cast<double>(elapsed_us) / kIterations);
}

// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?
//  Terrain analysis benchmark (nth_element optimization)
// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?

TEST(Benchmark, TerrainAnalysis) {
  TerrainParams tp;
  TerrainAnalysisCore terrain(tp);
  terrain.updateVehicle(0, 0, 0, 0, 0, 0);

  // Generate 2000 random points (typical LiDAR frame)
  std::mt19937 rng(123);
  std::uniform_real_distribution<float> posR(-5.0f, 5.0f);
  std::uniform_real_distribution<float> zR(-0.5f, 0.5f);

  std::vector<float> cloud(2000 * 4);
  for (int i = 0; i < 2000; i++) {
    cloud[i*4+0] = posR(rng);
    cloud[i*4+1] = posR(rng);
    cloud[i*4+2] = zR(rng);
    cloud[i*4+3] = 0.0f;
  }

  constexpr int kTerrainIter = 100;
  auto t0 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < kTerrainIter; i++) {
    terrain.updateVehicle(0.01 * i, 0, 0, 0, 0, 0);
    terrain.process(cloud.data(), 2000, i * 0.1);
  }
  auto t1 = std::chrono::high_resolution_clock::now();

  auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
  std::printf("[BENCHMARK] TerrainAnalysis: %d calls in %lld us "
              "(avg %.1f us/call, 2000 pts)\n",
              kTerrainIter, static_cast<long long>(elapsed_us),
              static_cast<double>(elapsed_us) / kTerrainIter);

  EXPECT_LT(elapsed_us / 1000.0, 5000.0)  // 5s for 100 iterations
      << "Terrain should process 2000 pts in reasonable time";
}

// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?
//  SIMD vs Scalar: 2D Rotation Benchmark
// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?

// Prevent compiler from optimizing away the result
static void doNotOptimize(const void* p) {
#if defined(_MSC_VER)
  (void)*reinterpret_cast<const volatile char*>(p);
#else
  asm volatile("" : : "r"(p) : "memory");
#endif
}

TEST(Benchmark, SimdRotation) {
  constexpr int N = 1000;
  constexpr int kReps = 10000;

  std::mt19937 rng(99);
  std::uniform_real_distribution<float> dist(-3.0f, 3.0f);

  std::vector<float> cx(N), cy(N), ox(N), oy(N);
  for (int i = 0; i < N; i++) { cx[i] = dist(rng); cy[i] = dist(rng); }

  float cosD = 0.866f, sinD = 0.5f;  // 30 degrees

  // Scalar baseline
  auto t0 = std::chrono::high_resolution_clock::now();
  for (int r = 0; r < kReps; r++) {
    for (int i = 0; i < N; i++) {
      ox[i] = cosD * cx[i] + sinD * cy[i];
      oy[i] = -sinD * cx[i] + cosD * cy[i];
    }
    doNotOptimize(ox.data());
    doNotOptimize(oy.data());
  }
  auto t1 = std::chrono::high_resolution_clock::now();

  // SIMD (xsimd via simd::rotateCloud)
  for (int r = 0; r < kReps; r++) {
    nav_kernel::simd::rotateCloud(cx.data(), cy.data(), cosD, sinD,
                                ox.data(), oy.data(), N);
    doNotOptimize(ox.data());
    doNotOptimize(oy.data());
  }
  auto t2 = std::chrono::high_resolution_clock::now();

  auto scalar_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
  auto simd_us   = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
  double speedup = (simd_us > 0) ? static_cast<double>(scalar_us) / simd_us : 999.0;

  std::printf("[BENCHMARK] Rotation %d pts x %d reps:\n", N, kReps);
  std::printf("  Scalar: %lld us (%.2f ns/pt)\n", static_cast<long long>(scalar_us),
              1000.0 * scalar_us / (N * kReps));
  std::printf("  SIMD:   %lld us (%.2f ns/pt)\n", static_cast<long long>(simd_us),
              1000.0 * simd_us / (N * kReps));
  std::printf("  Speedup: %.2fx\n", speedup);
#if NAV_KERNEL_HAS_XSIMD
  std::printf("  xsimd ENABLED (batch width = %zu floats)\n",
              nav_kernel::simd::kFloatWidth);
#else
  std::printf("  xsimd DISABLED (scalar fallback)\n");
#endif
}

// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?
//  SIMD Squared Distance Benchmark
// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?

TEST(Benchmark, SimdDistSq) {
  constexpr int N = 1000;
  constexpr int kReps = 10000;

  std::mt19937 rng(42);
  std::uniform_real_distribution<float> dist(-5.0f, 5.0f);

  std::vector<float> x(N), y(N), dsq(N);
  for (int i = 0; i < N; i++) { x[i] = dist(rng); y[i] = dist(rng); }

  // Scalar
  auto t0 = std::chrono::high_resolution_clock::now();
  for (int r = 0; r < kReps; r++) {
    for (int i = 0; i < N; i++)
      dsq[i] = x[i] * x[i] + y[i] * y[i];
    doNotOptimize(dsq.data());
  }
  auto t1 = std::chrono::high_resolution_clock::now();

  // SIMD
  for (int r = 0; r < kReps; r++) {
    nav_kernel::simd::distSqBatch(x.data(), y.data(), dsq.data(), N);
    doNotOptimize(dsq.data());
  }
  auto t2 = std::chrono::high_resolution_clock::now();

  auto scalar_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
  auto simd_us   = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
  double speedup = (simd_us > 0) ? static_cast<double>(scalar_us) / simd_us : 999.0;

  std::printf("[BENCHMARK] DistSq %d pts x %d reps:\n", N, kReps);
  std::printf("  Scalar: %lld us (%.2f ns/pt)\n", static_cast<long long>(scalar_us),
              1000.0 * scalar_us / (N * kReps));
  std::printf("  SIMD:   %lld us (%.2f ns/pt)\n", static_cast<long long>(simd_us),
              1000.0 * simd_us / (N * kReps));
  std::printf("  Speedup: %.2fx\n", speedup);
}

// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?
//  SIMD Correctness Check
// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?

// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?
//  End-to-end: scoreAndSelect with synthetic paths
// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?

TEST(Benchmark, ScoreAndSelectEndToEnd) {
  // Create a planner with synthetic path data
  LocalPlannerParams lpp;
  lpp.adjacentRange = 3.5;
  lpp.checkObstacle = true;
  lpp.useTerrainAnalysis = true;
  lpp.dirWeight = 0.02;
  lpp.twoWayDrive = true;
  lpp.pathScale = 1.0;
  lpp.minPathScale = 0.75;

  PlannerFixture planner(lpp);
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  // Generate 500 obstacle points (typical outdoor scene)
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> posR(-3.0f, 3.0f);
  std::uniform_real_distribution<float> zR(-0.3f, 0.3f);
  std::uniform_real_distribution<float> hR(0.0f, 0.4f);

  constexpr int nPts = 500;
  std::vector<float> cloud(nPts * 4);
  for (int i = 0; i < nPts; i++) {
    cloud[i*4+0] = posR(rng);
    cloud[i*4+1] = posR(rng);
    cloud[i*4+2] = zR(rng);
    cloud[i*4+3] = hR(rng);
  }

  // Measure plan() cycle (without loaded paths, tests buildPlannerCloud + checkNearField)
  constexpr int kReps = 5000;
  auto t0 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < kReps; i++) {
    planner.setVehicle(0.001 * (i % 100), 0, 0, 0.01 * (i % 36));
    planner.plan(cloud.data(), nPts, i * 0.01);
  }
  auto t1 = std::chrono::high_resolution_clock::now();

  auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
  double avg_us = static_cast<double>(elapsed_us) / kReps;
  std::printf("[BENCHMARK] E2E plan(%d pts): %d reps in %lld us "
              "(avg %.2f us/call, %.0f Hz)\n",
              nPts, kReps, static_cast<long long>(elapsed_us), avg_us,
              avg_us > 0 ? 1e6 / avg_us : 999999.0);
}

// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?
//  Terrain: Parallel estimateGround benchmark
// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?

TEST(Benchmark, TerrainParallelEstimateGround) {
  TerrainParams tp;
  TerrainAnalysisCore terrain(tp);
  terrain.updateVehicle(0, 0, 0, 0, 0, 0);

  // Generate dense cloud to fill many planar voxels (5000 pts typical LiDAR)
  std::mt19937 rng(456);
  std::uniform_real_distribution<float> posR(-5.0f, 5.0f);
  std::uniform_real_distribution<float> zR(-0.5f, 0.5f);

  std::vector<float> cloud(5000 * 4);
  for (int i = 0; i < 5000; i++) {
    cloud[i*4+0] = posR(rng);
    cloud[i*4+1] = posR(rng);
    cloud[i*4+2] = zR(rng);
    cloud[i*4+3] = 0.0f;
  }

  // Pre-fill the terrain grid with accumulated data
  for (int w = 0; w < 5; w++) {
    terrain.updateVehicle(0.01 * w, 0, 0, 0, 0, 0);
    terrain.process(cloud.data(), 5000, w * 0.1);
  }

  // Now benchmark the steady-state processing (estimateGround dominates)
  constexpr int kIter = 50;
  auto t0 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < kIter; i++) {
    terrain.updateVehicle(0.05 + 0.001 * i, 0, 0, 0, 0, 0);
    terrain.process(cloud.data(), 5000, 5.0 + i * 0.1);
  }
  auto t1 = std::chrono::high_resolution_clock::now();

  auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
  std::printf("[BENCHMARK] Terrain (parallel, 5000 pts, 2601 voxels): %d calls in %lld us "
              "(avg %.1f us/call)\n",
              kIter, static_cast<long long>(elapsed_us),
              static_cast<double>(elapsed_us) / kIter);
}

// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?
//  buildPlannerCloud: scalar vs SIMD batch
// 閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡鎰ㄦ櫜閳烘劏鏅查埡?

TEST(Benchmark, BuildPlannerCloudSIMD) {
  LocalPlannerParams lpp;
  lpp.adjacentRange = 3.5;
  lpp.useTerrainAnalysis = true;

  PlannerFixture planner(lpp);
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  // Generate 2000 obstacle points (typical dense scene)
  std::mt19937 rng(77);
  std::uniform_real_distribution<float> posR(-3.0f, 3.0f);
  std::uniform_real_distribution<float> zR(-0.3f, 0.3f);
  std::uniform_real_distribution<float> hR(0.0f, 0.4f);

  constexpr int nPts = 2000;
  std::vector<float> cloud(nPts * 4);
  for (int i = 0; i < nPts; i++) {
    cloud[i*4+0] = posR(rng);
    cloud[i*4+1] = posR(rng);
    cloud[i*4+2] = zR(rng);
    cloud[i*4+3] = hR(rng);
  }

  constexpr int kReps = 5000;
  auto t0 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < kReps; i++) {
    planner.setVehicle(0.001 * (i % 100), 0, 0, 0.01 * (i % 36));
    planner.plan(cloud.data(), nPts, i * 0.01);
  }
  auto t1 = std::chrono::high_resolution_clock::now();

  auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
  double avg_us = static_cast<double>(elapsed_us) / kReps;
  std::printf("[BENCHMARK] buildPlannerCloud SIMD (%d pts): %d reps in %lld us "
              "(avg %.2f us/call, %.0f Hz)\n",
              nPts, kReps, static_cast<long long>(elapsed_us), avg_us,
              avg_us > 0 ? 1e6 / avg_us : 999999.0);
}

TEST(Benchmark, SimdCorrectness) {
  constexpr int N = 128;
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> dist(-3.0f, 3.0f);

  std::vector<float> cx(N), cy(N);
  for (int i = 0; i < N; i++) { cx[i] = dist(rng); cy[i] = dist(rng); }

  float cosD = 0.6f, sinD = 0.8f;

  // Scalar reference
  std::vector<float> sx(N), sy(N);
  for (int i = 0; i < N; i++) {
    sx[i] = cosD * cx[i] + sinD * cy[i];
    sy[i] = -sinD * cx[i] + cosD * cy[i];
  }

  // SIMD
  std::vector<float> ox(N), oy(N);
  nav_kernel::simd::rotateCloud(cx.data(), cy.data(), cosD, sinD,
                              ox.data(), oy.data(), N);

  for (int i = 0; i < N; i++) {
    EXPECT_NEAR(ox[i], sx[i], 1e-5f) << "x2 mismatch at " << i;
    EXPECT_NEAR(oy[i], sy[i], 1e-5f) << "y2 mismatch at " << i;
  }

  // DistSq
  std::vector<float> dsq_ref(N), dsq_simd(N);
  for (int i = 0; i < N; i++)
    dsq_ref[i] = cx[i] * cx[i] + cy[i] * cy[i];
  nav_kernel::simd::distSqBatch(cx.data(), cy.data(), dsq_simd.data(), N);
  for (int i = 0; i < N; i++)
    EXPECT_NEAR(dsq_simd[i], dsq_ref[i], 1e-5f) << "dsq mismatch at " << i;
}
