#include <gtest/gtest.h>

#include "nav_kernel/dynamic_clear_core.hpp"

#include <cmath>
#include <vector>

using namespace nav_kernel;

namespace {

DynamicClearParams testParams() {
  DynamicClearParams p;
  p.voxelSize = 0.2;
  p.weakTtlS = 0.5;
  p.staticTtlS = 3.0;
  p.staticMinHits = 3;
  p.staticMinFrames = 2;
  p.raycastClearing = true;
  p.raycastClearMinFrames = 2;
  p.raycastMaxRange = 4.0;
  return p;
}

std::vector<float> xyz(std::initializer_list<float> values) {
  return std::vector<float>(values);
}

std::vector<float> xyzi(std::initializer_list<float> values) {
  return std::vector<float>(values);
}

}  // namespace

TEST(DynamicClearCore, KeepsCurrentlyObservedObstacle) {
  DynamicClearCore core(testParams());
  const auto result = core.filter(
      xyzi({1.0f, 0.0f, 0.4f, 0.4f}),
      xyz({1.0f, 0.0f, 0.4f}),
      1.0);

  EXPECT_EQ(result.stats.input_points, 1u);
  EXPECT_EQ(result.stats.kept_points, 1u);
  EXPECT_EQ(result.stats.dynamic_points, 0u);
  ASSERT_EQ(result.kept_xyzi.size(), 4u);
  EXPECT_FLOAT_EQ(result.kept_xyzi[0], 1.0f);
}

TEST(DynamicClearCore, ClearsWeakStaleResidueAfterTtl) {
  DynamicClearCore core(testParams());
  core.filter(
      xyzi({1.0f, 0.0f, 0.4f, 0.4f}),
      xyz({1.0f, 0.0f, 0.4f}),
      1.0);

  const auto result = core.filter(
      xyzi({1.0f, 0.0f, 0.4f, 0.4f}),
      xyz({}),
      1.7);

  EXPECT_EQ(result.stats.kept_points, 0u);
  EXPECT_EQ(result.stats.dynamic_points, 1u);
  ASSERT_EQ(result.dynamic_xyzi.size(), 4u);
  EXPECT_LT(result.dynamic_xyzi[3], 0.0f);
}

TEST(DynamicClearCore, KeepsStaticEvidenceAfterWeakTtl) {
  DynamicClearCore core(testParams());
  core.filter(
      xyzi({1.0f, 0.0f, 0.4f, 0.4f}),
      xyz({1.0f, 0.0f, 0.4f}),
      1.0);
  core.filter(
      xyzi({1.0f, 0.0f, 0.4f, 0.4f}),
      xyz({1.0f, 0.0f, 0.4f}),
      1.2);
  core.filter(
      xyzi({1.0f, 0.0f, 0.4f, 0.4f}),
      xyz({1.0f, 0.0f, 0.4f}),
      1.3);

  const auto result = core.filter(
      xyzi({1.0f, 0.0f, 0.4f, 0.4f}),
      xyz({}),
      1.9);

  EXPECT_EQ(result.stats.kept_points, 1u);
  EXPECT_EQ(result.stats.dynamic_points, 0u);
}

TEST(DynamicClearCore, DoesNotPromoteSingleFrameHitsToStatic) {
  DynamicClearCore core(testParams());
  core.filter(
      xyzi({1.0f, 0.0f, 0.4f, 0.4f}),
      xyz({
          1.0f, 0.0f, 0.4f,
          1.01f, 0.0f, 0.4f,
          1.02f, 0.0f, 0.4f,
      }),
      1.0);

  const auto result = core.filter(
      xyzi({1.0f, 0.0f, 0.4f, 0.4f}),
      xyz({}),
      1.7);

  EXPECT_EQ(result.stats.kept_points, 0u);
  EXPECT_EQ(result.stats.dynamic_points, 1u);
}

TEST(DynamicClearCore, RaycastClearsStaleResidueBeforeWeakTtl) {
  DynamicClearParams params = testParams();
  params.weakTtlS = 5.0;
  DynamicClearCore core(params);
  const DynamicClearOrigin origin{0.0, 0.0, 0.0, true};

  core.filter(
      xyzi({1.0f, 0.0f, 0.0f, 0.1f}),
      xyz({1.0f, 0.0f, 0.0f}),
      origin,
      1.0);
  core.filter(
      xyzi({1.0f, 0.0f, 0.0f, 0.1f}),
      xyz({2.0f, 0.0f, 0.0f}),
      origin,
      1.1);

  const auto result = core.filter(
      xyzi({1.0f, 0.0f, 0.0f, 0.1f}),
      xyz({2.0f, 0.0f, 0.0f}),
      origin,
      1.2);

  EXPECT_EQ(result.stats.kept_points, 0u);
  EXPECT_EQ(result.stats.dynamic_points, 1u);
  EXPECT_EQ(result.stats.ray_cleared_points, 1u);
  EXPECT_GT(result.stats.free_voxels, 0u);
}

TEST(DynamicClearCore, CurrentHitWinsOverRaycastFreeEvidence) {
  DynamicClearParams params = testParams();
  params.weakTtlS = 5.0;
  DynamicClearCore core(params);
  const DynamicClearOrigin origin{0.0, 0.0, 0.0, true};

  core.filter(
      xyzi({1.0f, 0.0f, 0.0f, 0.1f}),
      xyz({1.0f, 0.0f, 0.0f}),
      origin,
      1.0);
  const auto result = core.filter(
      xyzi({1.0f, 0.0f, 0.0f, 0.1f}),
      xyz({
          1.0f, 0.0f, 0.0f,
          2.0f, 0.0f, 0.0f,
      }),
      origin,
      1.1);

  EXPECT_EQ(result.stats.kept_points, 1u);
  EXPECT_EQ(result.stats.dynamic_points, 0u);
  EXPECT_EQ(result.stats.ray_cleared_points, 0u);
}

TEST(DynamicClearCore, ResetDropsEvidence) {
  DynamicClearCore core(testParams());
  core.filter(
      xyzi({1.0f, 0.0f, 0.4f, 0.4f}),
      xyz({1.0f, 0.0f, 0.4f}),
      1.0);
  core.reset();

  const auto result = core.filter(
      xyzi({1.0f, 0.0f, 0.4f, 0.4f}),
      xyz({}),
      1.1);

  EXPECT_EQ(result.stats.kept_points, 0u);
  EXPECT_EQ(result.stats.dynamic_points, 1u);
}

TEST(DynamicClearCore, RaycastWorkIsBoundedAndDeduplicatedPerFrame) {
  DynamicClearParams params = testParams();
  params.maxRayCount = 32;
  DynamicClearCore core(params);
  const DynamicClearOrigin origin{0.0, 0.0, 0.0, true};

  std::vector<float> current;
  current.reserve(4000 * 3);
  for (int i = 0; i < 4000; ++i) {
    const float angle = static_cast<float>(i) * 0.0174532925f;
    current.push_back(std::cos(angle) * 3.0f);
    current.push_back(std::sin(angle) * 3.0f);
    current.push_back(0.0f);
  }

  const auto result = core.filter(xyzi({}), current, origin, 1.0);
  EXPECT_LE(result.stats.raycast_rays, params.maxRayCount);
  EXPECT_GT(result.stats.raycast_voxels, 0u);
  EXPECT_LT(result.stats.evidence_voxels, current.size() / 3 * 10u);
}
