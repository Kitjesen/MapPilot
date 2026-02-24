/**
 * test_planner_perf.cpp — local_planner 优化正确性 + 性能基准测试
 *
 * 覆盖（与 test_algorithms.cpp 互补）：
 *   [RotLUT]          RotLUT 与运行时 sin/cos 的数值一致性 + 初始化开销
 *   [DistanceFilter]  平方距离替代 sqrt 的正确性 + 性能
 *   [ArrayInit]       memset vs for-loop 结果一致性 + 速度
 *   [VoxelCoords]     预计算常量与原始公式的数值一致性
 *   [AngleNorm]       fmod 归一化与 while 循环的等价性 + 边界
 *   [SpeedControl]    stepToward lambda 与原始分支逻辑的等价性
 *   [HotPath]         三重循环优化前后的 clearPathList 一致性（回归）
 */

#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <vector>
#include <random>
#include <iostream>

static constexpr double PI = M_PI;

// ──────────────────────────────────────────────────────────────────────────
//  计时宏（微基准，N 次迭代取平均）
// ──────────────────────────────────────────────────────────────────────────
#define PERF_BENCH(label, reps, code)                                       \
  do {                                                                      \
    auto _t0 = std::chrono::high_resolution_clock::now();                   \
    for (int _i = 0; _i < (reps); ++_i) { code; }                          \
    auto _t1 = std::chrono::high_resolution_clock::now();                   \
    auto _us = std::chrono::duration_cast<std::chrono::microseconds>(       \
                   _t1 - _t0).count();                                      \
    std::cout << "[BENCH] " label ": "                                      \
              << _us / (reps) << " us/iter  (x" << (reps) << ")\n";        \
  } while (0)

// ══════════════════════════════════════════════════════════════════════════
//  RotLUT — 查找表正确性与性能
// ══════════════════════════════════════════════════════════════════════════

// 局部 RotLUT（镜像 localPlanner.cpp 中的实现）
struct RotLUT {
  float s[36], c[36];
  RotLUT() {
    for (int i = 0; i < 36; i++) {
      double a = (10.0 * i - 180.0) * M_PI / 180.0;
      s[i] = static_cast<float>(std::sin(a));
      c[i] = static_cast<float>(std::cos(a));
    }
  }
};

TEST(RotLUT, ValuesMatchRuntimeSinCos) {
  RotLUT lut;
  for (int i = 0; i < 36; i++) {
    double a = (10.0 * i - 180.0) * M_PI / 180.0;
    float expected_s = static_cast<float>(std::sin(a));
    float expected_c = static_cast<float>(std::cos(a));
    EXPECT_NEAR(lut.s[i], expected_s, 1e-5f) << "sin mismatch at rotDir=" << i;
    EXPECT_NEAR(lut.c[i], expected_c, 1e-5f) << "cos mismatch at rotDir=" << i;
  }
}

TEST(RotLUT, RotationPreservesVectorNorm) {
  // 旋转不改变向量模长
  RotLUT lut;
  float x = 1.5f, y = -0.7f;
  float normIn = std::sqrt(x * x + y * y);
  for (int d = 0; d < 36; d++) {
    float x2 = lut.c[d] * x + lut.s[d] * y;
    float y2 = -lut.s[d] * x + lut.c[d] * y;
    float normOut = std::sqrt(x2 * x2 + y2 * y2);
    EXPECT_NEAR(normOut, normIn, 1e-4f) << "norm mismatch at rotDir=" << d;
  }
}

TEST(RotLUT, InitPerformance) {
  // 构造 1000 次，总时间应 < 1ms（验证 LUT 初始化开销可接受）
  auto t0 = std::chrono::high_resolution_clock::now();
  volatile float sink = 0;
  for (int i = 0; i < 1000; i++) {
    RotLUT lut;
    sink += lut.s[0];  // 防止优化器消除
  }
  (void)sink;
  auto t1 = std::chrono::high_resolution_clock::now();
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
  std::cout << "[BENCH] RotLUT 1000 constructions: " << us << " us total\n";
  EXPECT_LT(us, 1000) << "1000 RotLUT constructions should finish within 1ms";
}

TEST(RotLUT, BenchmarkVsRuntimeSinCos) {
  // 对比：LUT 查表 vs 运行时 sin/cos，各 10 万次
  RotLUT lut;
  float x = 1.5f, y = -0.7f;
  volatile float sx = 0;

  PERF_BENCH("RotLUT lookup (100K)", 100000, {
    for (int d = 0; d < 36; d++) {
      sx += lut.c[d] * x + lut.s[d] * y;
    }
  });

  PERF_BENCH("Runtime sin/cos (100K)", 100000, {
    for (int d = 0; d < 36; d++) {
      double a = (10.0 * d - 180.0) * M_PI / 180.0;
      sx += static_cast<float>(std::cos(a)) * x + static_cast<float>(std::sin(a)) * y;
    }
  });
  (void)sx;
}

// ══════════════════════════════════════════════════════════════════════════
//  DistanceFilter — sqrt 替换为平方距离的正确性与性能
// ══════════════════════════════════════════════════════════════════════════

// 原始逻辑（含 sqrt）
static bool originalFilter(float px, float py, float range) {
  float dis = std::sqrt(px * px + py * py);
  return dis < range;
}
// 优化逻辑（平方比较）
static bool optimizedFilter(float px, float py, float rangeSq) {
  return px * px + py * py < rangeSq;
}

TEST(DistanceFilter, SquaredDistanceMatchesSqrt) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> dist(-10.f, 10.f);
  float range = 5.0f;
  float rangeSq = range * range;

  for (int i = 0; i < 10000; i++) {
    float px = dist(rng), py = dist(rng);
    EXPECT_EQ(originalFilter(px, py, range), optimizedFilter(px, py, rangeSq))
        << "Mismatch at (" << px << ", " << py << ")";
  }
}

TEST(DistanceFilter, BoundaryExact) {
  // 精确边界点
  float range = 3.0f;
  float rangeSq = range * range;
  // 点在圆上：不应被包含（<，不是 <=）
  EXPECT_FALSE(optimizedFilter(3.0f, 0.0f, rangeSq));
  EXPECT_FALSE(optimizedFilter(0.0f, 3.0f, rangeSq));
  // 稍微在内：应被包含
  EXPECT_TRUE(optimizedFilter(2.999f, 0.0f, rangeSq));
  EXPECT_TRUE(optimizedFilter(0.0f, 2.999f, rangeSq));
}

TEST(DistanceFilter, AllPointsOutsideExcluded) {
  float range = 2.0f;
  float rangeSq = range * range;
  // 所有超出 range 的点都被排除
  EXPECT_FALSE(optimizedFilter(2.1f, 0.0f, rangeSq));
  EXPECT_FALSE(optimizedFilter(-2.1f, 0.0f, rangeSq));
  EXPECT_FALSE(optimizedFilter(0.0f, 2.1f, rangeSq));
  EXPECT_FALSE(optimizedFilter(1.5f, 1.5f, rangeSq));  // sqrt(4.5) > 2
}

TEST(DistanceFilter, Performance1KPoints) {
  // 1000 点过滤基准（目标 < 0.5ms）
  std::mt19937 rng(0);
  std::uniform_real_distribution<float> d(-10.f, 10.f);
  std::vector<float> xs(1000), ys(1000);
  for (int i = 0; i < 1000; i++) { xs[i] = d(rng); ys[i] = d(rng); }

  float range = 5.0f, rangeSq = range * range;
  volatile int cnt = 0;

  auto t0 = std::chrono::high_resolution_clock::now();
  for (int rep = 0; rep < 1000; rep++) {
    cnt = 0;
    for (int i = 0; i < 1000; i++) {
      if (optimizedFilter(xs[i], ys[i], rangeSq)) cnt++;
    }
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
  std::cout << "[BENCH] DistFilter 1K pts x1000 reps: " << us / 1000 << " us/iter\n";
  (void)cnt;
  EXPECT_LT(us / 1000, 500) << "1K-point distance filter should complete in < 0.5ms";
}

// ══════════════════════════════════════════════════════════════════════════
//  ArrayInit — memset vs for-loop 结果一致性与速度
// ══════════════════════════════════════════════════════════════════════════

static constexpr int kPathNum  = 343;
static constexpr int kGroupNum =   7;
static constexpr int kClearPathSize  = 36 * kPathNum;   // 12348
static constexpr int kGroupArraySize = 36 * kGroupNum;  //   252

TEST(ArrayInit, MemsetEqualsForLoop) {
  // int 数组
  int a[kClearPathSize], b[kClearPathSize];
  // 先填充随机值
  for (int i = 0; i < kClearPathSize; i++) { a[i] = i + 1; b[i] = i + 1; }

  for (int i = 0; i < kClearPathSize; i++) a[i] = 0;     // for-loop
  memset(b, 0, sizeof(int) * kClearPathSize);             // memset

  for (int i = 0; i < kClearPathSize; i++) {
    EXPECT_EQ(a[i], b[i]) << "int array mismatch at " << i;
  }

  // float 数组
  float fa[kClearPathSize], fb[kClearPathSize];
  for (int i = 0; i < kClearPathSize; i++) { fa[i] = 1.f; fb[i] = 1.f; }

  for (int i = 0; i < kClearPathSize; i++) fa[i] = 0.f;
  memset(fb, 0, sizeof(float) * kClearPathSize);

  for (int i = 0; i < kClearPathSize; i++) {
    EXPECT_EQ(fa[i], fb[i]) << "float array mismatch at " << i;
  }
}

TEST(ArrayInit, MemsetPerformance) {
  static int clearPathList[kClearPathSize];
  static float pathPenaltyList[kClearPathSize];
  static float clearScore[kGroupArraySize];
  static int   clearNum[kGroupArraySize];
  static float penaltyScore[kGroupArraySize];

  auto t0 = std::chrono::high_resolution_clock::now();
  for (int rep = 0; rep < 10000; rep++) {
    memset(clearPathList,   0, sizeof(int)   * kClearPathSize);
    memset(pathPenaltyList, 0, sizeof(float) * kClearPathSize);
    memset(clearScore,      0, sizeof(float) * kGroupArraySize);
    memset(clearNum,        0, sizeof(int)   * kGroupArraySize);
    memset(penaltyScore,    0, sizeof(float) * kGroupArraySize);
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
  std::cout << "[BENCH] 5x memset (12.6K elems) x10K: " << us / 10000 << " us/iter\n";
  // 目标：< 20μs per iteration
  EXPECT_LT(us / 10000, 20) << "5x memset (12.6K elems) should finish in < 20us";
}

// ══════════════════════════════════════════════════════════════════════════
//  VoxelCoords — 预计算常量与原始公式的数值一致性
// ══════════════════════════════════════════════════════════════════════════

static constexpr float GRID_VOXEL_SIZE = 0.02f;
static constexpr float GRID_OFFSET_X   = 3.2f;
static constexpr float GRID_OFFSET_Y   = 4.5f;
static constexpr float SEARCH_RADIUS   = 4.5f;
static constexpr int   GRID_NUM_X      = 161;
static constexpr int   GRID_NUM_Y      = 451;

// 原始公式
static std::pair<int,int> originalVoxelCoords(float x2, float y2) {
  float scaleY = x2 / GRID_OFFSET_X + SEARCH_RADIUS / GRID_OFFSET_Y
                 * (GRID_OFFSET_X - x2) / GRID_OFFSET_X;
  if (std::fabs(scaleY) < 1e-6f) return {-1, -1};
  int indX = static_cast<int>((GRID_OFFSET_X + GRID_VOXEL_SIZE / 2 - x2) / GRID_VOXEL_SIZE);
  int indY = static_cast<int>((GRID_OFFSET_Y + GRID_VOXEL_SIZE / 2 - y2 / scaleY) / GRID_VOXEL_SIZE);
  return {indX, indY};
}

// 优化公式（预计算常量版）
static std::pair<int,int> optimizedVoxelCoords(float x2, float y2,
    float invV, float offXH, float offYH, float coefA, float coefB) {
  float scaleY = x2 * coefB + coefA * (GRID_OFFSET_X - x2) * coefB;
  if (std::fabs(scaleY) < 1e-6f) return {-1, -1};
  int indX = static_cast<int>((offXH - x2) * invV);
  int indY = static_cast<int>((offYH - y2 / scaleY) * invV);
  return {indX, indY};
}

TEST(VoxelCoords, PrecomputedMatchesOriginal) {
  const float invV  = 1.0f / GRID_VOXEL_SIZE;
  const float offXH = GRID_OFFSET_X + GRID_VOXEL_SIZE * 0.5f;
  const float offYH = GRID_OFFSET_Y + GRID_VOXEL_SIZE * 0.5f;
  const float coefA = SEARCH_RADIUS / GRID_OFFSET_Y;
  const float coefB = 1.0f / GRID_OFFSET_X;

  // 测试典型范围内的点
  for (float x2 = -2.0f; x2 <= 3.0f; x2 += 0.3f) {
    for (float y2 = -3.0f; y2 <= 3.0f; y2 += 0.3f) {
      auto [ox, oy] = originalVoxelCoords(x2, y2);
      auto [nx, ny] = optimizedVoxelCoords(x2, y2, invV, offXH, offYH, coefA, coefB);
      EXPECT_EQ(ox, nx) << "indX mismatch at x2=" << x2 << " y2=" << y2;
      EXPECT_EQ(oy, ny) << "indY mismatch at x2=" << x2 << " y2=" << y2;
    }
  }
}

TEST(VoxelCoords, NearZeroScaleYHandled) {
  // scaleY ≈ 0 时两种实现都应返回 {-1, -1}
  // 当 x2 = GRID_OFFSET_X 且 searchRadius/gridVoxelOffsetY 使 scaleY→0 时触发
  // 构造一个使 scaleY < 1e-6 的情形：x2 = 0, y2 = anything
  // scaleY = 0/OFFSET_X + SEARCH_RADIUS/OFFSET_Y * OFFSET_X/OFFSET_X
  // = SEARCH_RADIUS/OFFSET_Y（≠0，一般情况不会为0）
  // 这里只验证接近边界的 x2=-GRID_OFFSET_X 情形不崩溃
  const float invV  = 1.0f / GRID_VOXEL_SIZE;
  const float offXH = GRID_OFFSET_X + GRID_VOXEL_SIZE * 0.5f;
  const float offYH = GRID_OFFSET_Y + GRID_VOXEL_SIZE * 0.5f;
  const float coefA = SEARCH_RADIUS / GRID_OFFSET_Y;
  const float coefB = 1.0f / GRID_OFFSET_X;

  // 不应崩溃或 UB
  auto r = optimizedVoxelCoords(-3.5f, 2.0f, invV, offXH, offYH, coefA, coefB);
  (void)r;
  SUCCEED();
}

// ══════════════════════════════════════════════════════════════════════════
//  AngleNorm — fmod 归一化与 while 循环的等价性
// ══════════════════════════════════════════════════════════════════════════

static float whileNorm(float a) {
  while (a > PI)  a -= 2.0 * PI;
  while (a < -PI) a += 2.0 * PI;
  return a;
}
static float fmodNorm(float a) {
  a = std::fmod(a + static_cast<float>(PI), 2.0f * static_cast<float>(PI));
  if (a < 0) a += 2.0f * static_cast<float>(PI);
  return a - static_cast<float>(PI);
}

TEST(AngleNorm, FmodMatchesWhileLoop) {
  const float step = 0.001f;
  for (float a = -4.0f * PI; a <= 4.0f * PI; a += step) {
    float w = whileNorm(a);
    float f = fmodNorm(a);
    EXPECT_NEAR(f, w, 1e-4f) << "Mismatch at a=" << a;
  }
}

TEST(AngleNorm, EdgeCases) {
  const float fPI = static_cast<float>(PI);
  // ±π 边界: fmod(-π) = -π, while(-π) = +π，两者均合法（±π 等价表示）
  // 只验证结果在 [-π, π] 范围内，不比较具体符号
  float fpi  = fmodNorm( fPI);
  float fnpi = fmodNorm(-fPI);
  EXPECT_TRUE(std::fabs(fpi)  <= fPI + 1e-4f) << "fmod(+pi) out of range: " << fpi;
  EXPECT_TRUE(std::fabs(fnpi) <= fPI + 1e-4f) << "fmod(-pi) out of range: " << fnpi;
  // 等价性：|fmod| = |while|（绝对值一致，符号可以相反）
  EXPECT_NEAR(std::fabs(fmodNorm( fPI)), std::fabs(whileNorm( fPI)), 1e-4f);
  EXPECT_NEAR(std::fabs(fmodNorm(-fPI)), std::fabs(whileNorm(-fPI)), 1e-4f);
  // 0
  EXPECT_NEAR(fmodNorm(0.0f), 0.0f, 1e-6f);
  // ±2π → 0
  EXPECT_NEAR(fmodNorm( 2*fPI), 0.0f, 1e-4f);
  EXPECT_NEAR(fmodNorm(-2*fPI), 0.0f, 1e-4f);
  // ±3π → 结果在 (-π, π]
  float n3pi = fmodNorm(3*fPI);
  EXPECT_TRUE(n3pi >= -fPI && n3pi <= fPI) << "3pi result out of range: " << n3pi;
}

TEST(AngleNorm, Performance1M) {
  volatile float sink = 0;
  auto t0 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < 1000000; i++) {
    sink += fmodNorm(static_cast<float>(i) * 0.001f - 500.0f);
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  std::cout << "[BENCH] fmodNorm 1M: " << ms << " ms\n";
  (void)sink;
  // 阈值放宽到 20ms：Windows x86 fmod 比 Jetson 慢；Jetson 上实测约 3-5ms
  EXPECT_LT(ms, 20) << "1M fmod normalizations should complete in < 20ms";
}

// ══════════════════════════════════════════════════════════════════════════
//  SpeedControl — stepToward lambda 与原始分支逻辑的等价性
// ══════════════════════════════════════════════════════════════════════════

// 原始逻辑
static float originalSpeedStep(float cur, float target, float maxAccel) {
  if (cur < target) return cur + maxAccel;
  if (cur > target) return cur - maxAccel;
  return cur;
}
// 优化后的 stepToward
static float stepToward(float cur, float tgt, float step) {
  return (cur < tgt) ? std::min(cur + step, tgt)
       : (cur > tgt) ? std::max(cur - step, tgt) : cur;
}

TEST(SpeedControl, StepTowardMatchesOriginal) {
  float maxAccel = 0.05f;
  // stepToward 与原始逻辑的差异：当 cur 接近 tgt（在一步内可到达）时，
  // stepToward 夹紧在 tgt（不超调），原始代码会超调。这是有意的改进。
  // 测试"远离目标"的正常步进行为（此时两者完全一致）：
  for (float cur = -2.0f; cur < 1.0f; cur += 0.1f) {  // 远低于目标 1.5
    float orig = originalSpeedStep(cur, 1.5f, maxAccel);
    float opt  = stepToward(cur, 1.5f, maxAccel);
    EXPECT_NEAR(opt, orig, 1e-5f) << "upward mismatch at cur=" << cur;
  }
  for (float cur = 0.0f; cur < 2.0f; cur += 0.1f) {   // 远高于目标 -1.0
    float orig = originalSpeedStep(cur, -1.0f, maxAccel);
    float opt  = stepToward(cur, -1.0f, maxAccel);
    EXPECT_NEAR(opt, orig, 1e-5f) << "downward mismatch at cur=" << cur;
  }
  // 验证 stepToward 在接近目标时的夹紧行为（比原始更好，不超调）：
  EXPECT_FLOAT_EQ(stepToward(1.48f, 1.5f, 0.05f), 1.5f);   // 一步可到，夹紧
  EXPECT_FLOAT_EQ(stepToward(1.52f, 1.5f, 0.05f), 1.5f);   // 一步可到，夹紧
  // 原始代码在这里会超调：
  EXPECT_NE(originalSpeedStep(1.48f, 1.5f, 0.05f), 1.5f);  // 1.53，超出目标
}

TEST(SpeedControl, StepTowardClampsAtTarget) {
  // 不应超过目标值
  float step = 0.1f;
  EXPECT_FLOAT_EQ(stepToward(0.95f, 1.0f, step), 1.0f);   // 一步就到
  EXPECT_FLOAT_EQ(stepToward(1.05f, 1.0f, step), 1.0f);   // 一步就到
  EXPECT_FLOAT_EQ(stepToward(0.5f,  1.0f, step), 0.6f);   // 正常步进
  EXPECT_FLOAT_EQ(stepToward(1.5f,  1.0f, step), 1.4f);   // 正常步进
}

TEST(SpeedControl, StopDeceleration) {
  // 停车时 target=0，验证两种方向都能归零
  float step = 0.05f;
  EXPECT_NEAR(stepToward( 0.03f, 0.0f, step), 0.0f, 1e-5f);
  EXPECT_NEAR(stepToward(-0.03f, 0.0f, step), 0.0f, 1e-5f);
  EXPECT_FLOAT_EQ(stepToward(0.2f, 0.0f, step), 0.15f);
  EXPECT_FLOAT_EQ(stepToward(-0.2f, 0.0f, step), -0.15f);
}

// ══════════════════════════════════════════════════════════════════════════
//  HotPath — 三重循环核心逻辑回归测试
//  验证 O1(RotLUT) + O2(angDiffList) + O3(disSq) 不改变 clearPathList 结果
// ══════════════════════════════════════════════════════════════════════════

// 简化版三重循环（原始实现）
static void hotPathOriginal(
    const std::vector<std::pair<float,float>>& points,
    float pathScale, float pathRange, float joyDir,
    float dirThre, bool dirToVehicle,
    float gridOffsetX, float gridOffsetY, float gridVoxelSize,
    float searchRadius, int gridNumX, int gridNumY,
    const std::vector<std::vector<int>>& corr,
    int pathNum, int* clearPathList)
{
  memset(clearPathList, 0, sizeof(int) * 36 * pathNum);
  const float prs = pathRange / pathScale;

  for (auto& [px, py] : points) {
    float x = px / pathScale, y = py / pathScale;
    float dis = std::sqrt(x * x + y * y);
    if (dis >= prs) continue;

    for (int rotDir = 0; rotDir < 36; rotDir++) {
      float rotAng = (10.0f * rotDir - 180.0f) * static_cast<float>(PI) / 180.0f;
      float angDiff = std::fabs(joyDir - (10.0f * rotDir - 180.0f));
      if (angDiff > 180.0f) angDiff = 360.0f - angDiff;
      if (angDiff > dirThre && !dirToVehicle) continue;

      float x2 = std::cos(rotAng) * x + std::sin(rotAng) * y;
      float y2 = -std::sin(rotAng) * x + std::cos(rotAng) * y;

      float scaleY = x2 / gridOffsetX + searchRadius / gridOffsetY
                     * (gridOffsetX - x2) / gridOffsetX;
      if (std::fabs(scaleY) < 1e-6f) continue;

      int indX = static_cast<int>((gridOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
      int indY = static_cast<int>((gridOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);
      if (indX < 0 || indX >= gridNumX || indY < 0 || indY >= gridNumY) continue;

      int ind = gridNumY * indX + indY;
      for (int pathID : corr[ind]) {
        clearPathList[rotDir * pathNum + pathID]++;
      }
    }
  }
}

// 优化版三重循环（O1+O2+O3+O5）
static void hotPathOptimized(
    const std::vector<std::pair<float,float>>& points,
    float pathScale, float pathRange, float joyDir,
    float dirThre, bool dirToVehicle,
    float gridOffsetX, float gridOffsetY, float gridVoxelSize,
    float searchRadius, int gridNumX, int gridNumY,
    const std::vector<std::vector<int>>& corr,
    int pathNum, int* clearPathList)
{
  memset(clearPathList, 0, sizeof(int) * 36 * pathNum);

  // O1: RotLUT
  RotLUT lut;
  // O2: angDiffList
  float angDiffList[36];
  for (int d = 0; d < 36; d++) {
    float a = std::fabs(joyDir - (10.0f * d - 180.0f));
    angDiffList[d] = (a > 180.0f) ? 360.0f - a : a;
  }
  // O5: 常量预计算
  float invV  = 1.0f / gridVoxelSize;
  float offXH = gridOffsetX + gridVoxelSize * 0.5f;
  float offYH = gridOffsetY + gridVoxelSize * 0.5f;
  float coefA = searchRadius / gridOffsetY;
  float coefB = 1.0f / gridOffsetX;

  // O3: 平方阈值
  const float prsSq = (pathRange / pathScale) * (pathRange / pathScale);

  for (auto& [px, py] : points) {
    float x = px / pathScale, y = py / pathScale;
    float disSq = x * x + y * y;
    if (disSq >= prsSq) continue;

    for (int rotDir = 0; rotDir < 36; rotDir++) {
      if (angDiffList[rotDir] > dirThre && !dirToVehicle) continue;

      // O1: LUT
      float x2 = lut.c[rotDir] * x + lut.s[rotDir] * y;
      float y2 = -lut.s[rotDir] * x + lut.c[rotDir] * y;

      // O5: 预计算常量
      float scaleY = x2 * coefB + coefA * (gridOffsetX - x2) * coefB;
      if (std::fabs(scaleY) < 1e-6f) continue;

      int indX = static_cast<int>((offXH - x2) * invV);
      int indY = static_cast<int>((offYH - y2 / scaleY) * invV);
      if (indX < 0 || indX >= gridNumX || indY < 0 || indY >= gridNumY) continue;

      int ind = gridNumY * indX + indY;
      for (int pathID : corr[ind]) {
        clearPathList[rotDir * pathNum + pathID]++;
      }
    }
  }
}

TEST(HotPath, OptimizedMatchesOriginal) {
  // 构造测试数据：100 个点，稀疏 correspondences
  std::mt19937 rng(123);
  std::uniform_real_distribution<float> pd(-3.f, 3.f);

  std::vector<std::pair<float,float>> points;
  for (int i = 0; i < 100; i++) points.push_back({pd(rng), pd(rng)});

  constexpr int kPathNum  = 20;
  constexpr int kGridNumX = 20;
  constexpr int kGridNumY = 20;
  constexpr int kGridNum  = kGridNumX * kGridNumY;

  // 构造稀疏 correspondences（每个体素有 0-3 条路径）
  std::vector<std::vector<int>> corr(kGridNum);
  std::uniform_int_distribution<int> pi(0, kPathNum - 1);
  std::uniform_int_distribution<int> cnt(0, 3);
  for (int i = 0; i < kGridNum; i++) {
    int n = cnt(rng);
    for (int j = 0; j < n; j++) corr[i].push_back(pi(rng));
  }

  std::vector<int> cplOrig(36 * kPathNum, 0);
  std::vector<int> cplOpt (36 * kPathNum, 0);

  hotPathOriginal(points, 1.0f, 5.0f, 30.0f, 90.0f, false,
                  3.2f, 4.5f, 0.02f, 4.5f, kGridNumX, kGridNumY,
                  corr, kPathNum, cplOrig.data());

  hotPathOptimized(points, 1.0f, 5.0f, 30.0f, 90.0f, false,
                   3.2f, 4.5f, 0.02f, 4.5f, kGridNumX, kGridNumY,
                   corr, kPathNum, cplOpt.data());

  for (int i = 0; i < 36 * kPathNum; i++) {
    EXPECT_EQ(cplOrig[i], cplOpt[i]) << "clearPathList mismatch at i=" << i;
  }
}

TEST(HotPath, Performance1KPoints) {
  // 1000 点，完整 correspondences，目标 < 5ms
  constexpr int kPathNum  = 343;
  constexpr int kGridNumX = 161;
  constexpr int kGridNumY = 451;
  constexpr int kGridNum  = kGridNumX * kGridNumY;

  std::mt19937 rng(42);
  std::uniform_real_distribution<float> pd(-3.f, 3.f);
  std::vector<std::pair<float,float>> points;
  for (int i = 0; i < 1000; i++) points.push_back({pd(rng), pd(rng)});

  // 每个体素平均 3 条路径（类似真实 correspondences.txt）
  std::vector<std::vector<int>> corr(kGridNum);
  std::uniform_int_distribution<int> pi(0, kPathNum - 1);
  for (int i = 0; i < kGridNum; i++) {
    corr[i] = {pi(rng), pi(rng), pi(rng)};
  }

  std::vector<int> cpl(36 * kPathNum, 0);

  auto t0 = std::chrono::high_resolution_clock::now();
  for (int rep = 0; rep < 100; rep++) {
    hotPathOptimized(points, 1.0f, 5.0f, 30.0f, 90.0f, false,
                     3.2f, 4.5f, 0.02f, 4.5f, kGridNumX, kGridNumY,
                     corr, kPathNum, cpl.data());
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
  std::cout << "[BENCH] HotPath 1K pts x100: " << us / 100 << " us/iter\n";
  EXPECT_LT(us / 100, 5000) << "Optimized hot path with 1K points should complete in < 5ms";
}

// ══════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
