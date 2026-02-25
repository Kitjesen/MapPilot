#include <gtest/gtest.h>
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

// ── downsamplePath ──

TEST(DownsamplePath, EmptyInput) {
  Path empty;
  auto result = downsamplePath(empty, 0.5);
  EXPECT_TRUE(result.empty());
}

TEST(DownsamplePath, SinglePoint) {
  Path single = {makePose(1.0, 2.0, 3.0)};
  auto result = downsamplePath(single, 0.5);
  // front + back (same point) = 2
  EXPECT_EQ(result.size(), 2u);
}

TEST(DownsamplePath, DenseToSparse) {
  // 100 点直线, 间距 0.1m → 用 0.5m 降采样
  Path dense;
  for (int i = 0; i < 100; i++) {
    dense.push_back(makePose(0.1 * i, 0.0, 0.0));
  }
  auto result = downsamplePath(dense, 0.5);
  // 9.9m 路径 / 0.5m 间距 ≈ 20 段, +1 首 +1 尾 ≈ 22
  EXPECT_GT(result.size(), 15u);
  EXPECT_LT(result.size(), 30u);
}

TEST(DownsamplePath, Uses3DDistance) {
  // 路径有坡度: 水平 0.3m + 垂直 0.4m = 3D 距离 0.5m
  // 2D 距离只有 0.3m < 0.5, 如果用 2D 则不会保留, 用 3D 则会保留
  Path slope;
  for (int i = 0; i < 20; i++) {
    slope.push_back(makePose(0.3 * i, 0.0, 0.4 * i));
  }
  // 用 2D-only 距离阈值 0.4 测试 — 2D 距离只有 0.3/段, 但 3D 是 0.5/段
  auto result = downsamplePath(slope, 0.4);
  // 3D 距离 0.5 >= 0.4, 每段都应被保留
  // result = front + 19 (每个都 >= 0.4) + back = ~21
  EXPECT_GE(result.size(), 18u);
  // 如果用纯 2D 距离 0.3 < 0.4, 很多段会被跳过
}

TEST(DownsamplePath, AlwaysIncludesEndpoint) {
  Path path;
  for (int i = 0; i < 10; i++) {
    path.push_back(makePose(0.1 * i, 0.0, 0.0));
  }
  auto result = downsamplePath(path, 5.0);  // 很大间距
  // 首点 + 终点 (没有中间点满足距离)
  EXPECT_EQ(result.size(), 2u);
  EXPECT_DOUBLE_EQ(result.back().position.x, 0.9);
}

// ── WaypointTracker ──

class WaypointTrackerTest : public ::testing::Test {
protected:
  void SetUp() override {
    params.waypointDistance = 0.5;
    params.arrivalThreshold = 0.5;
    params.stuckTimeoutSec = 10.0;
    params.maxReplanCount = 2;
    params.replanCooldownSec = 5.0;
    params.searchWindow = 5;
    tracker = std::make_unique<WaypointTracker>(params);

    // 5m 直线路径
    Path raw;
    for (int i = 0; i <= 50; i++) {
      raw.push_back(makePose(0.1 * i, 0.0, 0.0));
    }
    tracker->setPath(raw, 0.0);
  }

  WaypointTrackerParams params;
  std::unique_ptr<WaypointTracker> tracker;
};

TEST_F(WaypointTrackerTest, PathReceivedEvent) {
  Path raw = {makePose(0.0, 0.0, 0.0), makePose(1.0, 0.0, 0.0)};
  auto r = tracker->setPath(raw, 0.0);
  EXPECT_EQ(r.event, WaypointEvent::kPathReceived);
  EXPECT_GT(r.totalWaypoints, 0u);
}

TEST_F(WaypointTrackerTest, WaypointProgression) {
  // 降采样后首点在 (0,0,0), 第二个约在 (0.5,0,0)
  // 机器人在 (0,0,0) → 离首个航点距离 0 < arrivalThreshold(0.5) → 到达
  Vec3 robot{0.0, 0.0, 0.0};
  auto r = tracker->update(robot, {}, 1.0);
  EXPECT_EQ(r.event, WaypointEvent::kWaypointReached);
  EXPECT_GE(tracker->currentIndex(), 1u);
}

TEST_F(WaypointTrackerTest, GoalReached) {
  // 直接跳到终点
  const auto& path = tracker->path();
  Vec3 robot = path.back().position;
  // 需要先推进到最后一个航点
  // 模拟逐步推进
  for (size_t i = 0; i < path.size(); i++) {
    Vec3 pos = path[i].position;
    pos.x += 0.01;  // 略偏
    auto r = tracker->update(pos, {}, static_cast<double>(i));
  }
  EXPECT_TRUE(tracker->goalReached());
}

TEST_F(WaypointTrackerTest, StuckTriggersReplan) {
  // 使用非零起始时间 (ROS2 时间永远 > 0, lastProgressTime_ > 0 guard)
  // 重新 setPath 用 t=100
  Path raw;
  for (int i = 0; i <= 50; i++) raw.push_back(makePose(0.1 * i, 0.0, 0.0));
  tracker->setPath(raw, 100.0);

  // 先推进过航点 0
  Vec3 atFirst{0.0, 0.0, 0.0};
  tracker->update(atFirst, {}, 100.0);  // 到达航点 0 → currentIdx=1

  // 卡住位置: 离航点 1 (~0.5,0,0) 太远
  Vec3 stuck{0.26, 0.5, 0.0};
  tracker->update(stuck, {}, 101.0);

  // t=112: 距上次进步 12 秒 > stuckTimeoutSec(10)
  auto r = tracker->update(stuck, {}, 112.0);
  EXPECT_EQ(r.event, WaypointEvent::kReplanning);
  EXPECT_EQ(tracker->replanCount(), 1);
}

TEST_F(WaypointTrackerTest, StuckFinalAfterMaxReplans) {
  Path raw;
  for (int i = 0; i <= 50; i++) raw.push_back(makePose(0.1 * i, 0.0, 0.0));
  tracker->setPath(raw, 100.0);

  Vec3 atFirst{0.0, 0.0, 0.0};
  tracker->update(atFirst, {}, 100.0);

  Vec3 stuck{0.26, 0.5, 0.0};
  tracker->update(stuck, {}, 101.0);

  // 第1次 stuck → replan
  auto r1 = tracker->update(stuck, {}, 112.0);
  EXPECT_EQ(r1.event, WaypointEvent::kReplanning);

  // 第2次 stuck → replan
  auto r2 = tracker->update(stuck, {}, 123.0);
  EXPECT_EQ(r2.event, WaypointEvent::kReplanning);

  // 第3次 stuck → stuck_final (maxReplanCount=2, 已用完)
  auto r3 = tracker->update(stuck, {}, 134.0);
  EXPECT_EQ(r3.event, WaypointEvent::kStuckFinal);
}

TEST_F(WaypointTrackerTest, ProgressResetsStuckTimer) {
  Vec3 robot{0.0, 0.0, 0.0};
  tracker->update(robot, {}, 0.0);

  // t=8: 还没超时
  tracker->update(robot, {}, 8.0);

  // 然后进步: 到达第一个航点
  Vec3 moved{0.3, 0.0, 0.0};
  tracker->update(moved, {}, 9.0);

  // t=18: 距离上次进步只过了 9 秒, 不触发 stuck
  auto r = tracker->update(moved, {}, 18.0);
  EXPECT_NE(r.event, WaypointEvent::kReplanning);
}

TEST_F(WaypointTrackerTest, WindowedSearch) {
  // 验证只在前方窗口内搜索 (不跳过整段路径)
  Vec3 robot{0.0, 0.0, 0.0};
  tracker->update(robot, {}, 0.0);

  // 机器人突然出现在很远的航点附近 (但在搜索窗口外)
  // searchWindow=5, 所以只搜索 [0, 5) 范围
  const auto& path = tracker->path();
  if (path.size() > 10) {
    Vec3 farPos = path[8].position;  // 超出窗口
    auto r = tracker->update(farPos, {}, 1.0);
    // 应该不会跳到 index 8 (超出搜索窗口)
    EXPECT_LT(tracker->currentIndex(), 6u);
  }
}

TEST_F(WaypointTrackerTest, ReplanCooldown) {
  Vec3 robot{0.0, 0.0, 0.0};
  tracker->update(robot, {}, 0.0);

  // t=11: 第一次 stuck → replan
  tracker->update(robot, {}, 11.0);

  // t=12: 只过了 1 秒 (< replanCooldownSec=5), 不应该再次 replan
  // 但 lastProgressTime_ 被重置了，所以需要再等 10 秒
  auto r = tracker->update(robot, {}, 12.0);
  EXPECT_NE(r.event, WaypointEvent::kReplanning);
}
