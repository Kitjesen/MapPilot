/**
 * test_validation.cpp -- NaN/Inf input validation tests
 */
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <vector>
#include "nav_core/validation.hpp"

using namespace nav_core;

// ── isValidPosition ──

TEST(Validation, InvalidPosition_NaN) {
  Vec3 p{std::numeric_limits<double>::quiet_NaN(), 1.0, 2.0};
  EXPECT_FALSE(isValidPosition(p));

  Vec3 p2{1.0, std::numeric_limits<double>::quiet_NaN(), 2.0};
  EXPECT_FALSE(isValidPosition(p2));

  Vec3 p3{1.0, 2.0, std::numeric_limits<double>::quiet_NaN()};
  EXPECT_FALSE(isValidPosition(p3));
}

TEST(Validation, InvalidPosition_Inf) {
  Vec3 p{std::numeric_limits<double>::infinity(), 1.0, 2.0};
  EXPECT_FALSE(isValidPosition(p));

  Vec3 p2{1.0, -std::numeric_limits<double>::infinity(), 2.0};
  EXPECT_FALSE(isValidPosition(p2));
}

TEST(Validation, ValidPosition) {
  Vec3 p{1.0, -2.5, 0.0};
  EXPECT_TRUE(isValidPosition(p));

  Vec3 origin{0.0, 0.0, 0.0};
  EXPECT_TRUE(isValidPosition(origin));

  Vec3 large{1e6, -1e6, 1e3};
  EXPECT_TRUE(isValidPosition(large));
}

// ── isValidPath (Vec3 overload) ──

TEST(Validation, EmptyPath) {
  std::vector<Vec3> empty;
  EXPECT_FALSE(isValidPath(empty));
}

TEST(Validation, PathWithNaN) {
  std::vector<Vec3> path = {
    {0.0, 0.0, 0.0},
    {1.0, std::numeric_limits<double>::quiet_NaN(), 0.0},
    {2.0, 2.0, 0.0}
  };
  EXPECT_FALSE(isValidPath(path));
}

TEST(Validation, AllValidPath) {
  std::vector<Vec3> path = {
    {0.0, 0.0, 0.0},
    {1.0, 1.0, 0.0},
    {2.0, 2.0, 0.0}
  };
  EXPECT_TRUE(isValidPath(path));
}

// ── isValidPose ──

TEST(Validation, InvalidPose_NaNYaw) {
  Pose p;
  p.position = {1.0, 2.0, 0.0};
  p.yaw = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(isValidPose(p));
}

TEST(Validation, ValidPose) {
  Pose p;
  p.position = {1.0, 2.0, 0.0};
  p.yaw = 1.57;
  EXPECT_TRUE(isValidPose(p));
}

// ── isValidPath (Path/Pose overload) ──

TEST(Validation, PosePathEmpty) {
  Path empty;
  EXPECT_FALSE(isValidPath(empty));
}

TEST(Validation, PosePathWithInf) {
  Path path;
  Pose good;
  good.position = {1.0, 2.0, 0.0};
  good.yaw = 0.0;
  Pose bad;
  bad.position = {std::numeric_limits<double>::infinity(), 2.0, 0.0};
  bad.yaw = 0.0;
  path.push_back(good);
  path.push_back(bad);
  EXPECT_FALSE(isValidPath(path));
}

// ── filterInvalidPoses ──

TEST(Validation, FilterInvalidPoses) {
  Path path;
  Pose good1;
  good1.position = {1.0, 2.0, 0.0};
  good1.yaw = 0.0;
  Pose bad;
  bad.position = {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
  bad.yaw = 0.0;
  Pose good2;
  good2.position = {3.0, 4.0, 0.0};
  good2.yaw = 1.0;

  path.push_back(good1);
  path.push_back(bad);
  path.push_back(good2);

  Path filtered = filterInvalidPoses(path);
  ASSERT_EQ(filtered.size(), 2u);
  EXPECT_DOUBLE_EQ(filtered[0].position.x, 1.0);
  EXPECT_DOUBLE_EQ(filtered[1].position.x, 3.0);
}
