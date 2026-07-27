#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "plan/rolling_map_segment_executor.hpp"

namespace {

using lingtu::explore::DirectedTarget;
using lingtu::explore::ExploreMapIdentity;
using lingtu::explore::Grid2D;
using lingtu::explore::Pose2D;
using lingtu::nav::endpoint::RollingMapSegmentAction;
using lingtu::nav::endpoint::RollingMapSegmentExecutor;
using lingtu::nav::endpoint::RollingMapSegmentExecutorConfig;
using lingtu::nav::endpoint::RollingMapSegmentInput;
using lingtu::nav::endpoint::RollingMapSegmentRequest;
using lingtu::nav::endpoint::RollingMapSegmentSnapshot;
using lingtu::nav::endpoint::TerrainCostGrid;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

ExploreMapIdentity rollingIdentity(std::uint64_t generation, std::uint64_t reset_epoch = 1U,
                                   std::string session_id = "rolling-session") {
  ExploreMapIdentity identity;
  identity.frame_id = "map";
  identity.session_id = std::move(session_id);
  identity.reset_epoch = reset_epoch;
  identity.generation = generation;
  identity.live = true;
  return identity;
}

Grid2D grid(int width, int height, std::int8_t value) {
  Grid2D result;
  result.width = width;
  result.height = height;
  result.resolution = 1.0;
  result.origin_x = 0.0;
  result.origin_y = 0.0;
  result.cells.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), value);
  return result;
}

TerrainCostGrid riskLike(const Grid2D &occupancy, float value = 0.0F) {
  TerrainCostGrid result;
  result.width = occupancy.width;
  result.height = occupancy.height;
  result.resolution = occupancy.resolution;
  result.origin_x = occupancy.origin_x;
  result.origin_y = occupancy.origin_y;
  result.costs.assign(occupancy.cells.size(), value);
  return result;
}

void setCellAtPose(Grid2D &occupancy, const Pose2D &pose, std::int8_t value) {
  const int col =
      static_cast<int>(std::floor((pose.x - occupancy.origin_x) / occupancy.resolution));
  const int row =
      static_cast<int>(std::floor((pose.y - occupancy.origin_y) / occupancy.resolution));
  require(occupancy.inBounds(row, col), "test pose must remain inside the rolling occupancy grid");
  occupancy.cells[static_cast<std::size_t>(occupancy.index(row, col))] = value;
}

RollingMapSegmentInput inputFor(Grid2D occupancy, TerrainCostGrid terrain_cost,
                                ExploreMapIdentity identity, double now_s = 10.0) {
  RollingMapSegmentInput input;
  input.snapshot.occupancy = std::move(occupancy);
  input.snapshot.terrain_cost = std::move(terrain_cost);
  input.snapshot.identity = std::move(identity);
  input.snapshot.stamp_s = now_s;
  input.snapshot.terrain_risk_stamp_s = now_s;
  input.snapshot.terrain_risk_ready = true;
  input.robot_pose = {0.5, 1.5, 0.0};
  input.input_ready = true;
  input.now_s = now_s;
  return input;
}

RollingMapSegmentRequest target(double x, double y, std::uint64_t generation) {
  RollingMapSegmentRequest request;
  request.target = DirectedTarget{x, y};
  request.requested_generation = generation;
  return request;
}

void testSelectsBoundedSafePrefix() {
  auto occupancy = grid(10, 3, lingtu::explore::kFree);
  auto input = inputFor(occupancy, riskLike(occupancy), rollingIdentity(3U));

  RollingMapSegmentExecutorConfig config;
  config.max_segment_length_m = 2.5;
  config.max_waypoints = 3U;
  RollingMapSegmentExecutor executor(config);
  const auto decision = executor.plan(input, target(20.5, 1.5, 2U));

  require(decision.action == RollingMapSegmentAction::Accepted,
          "observed-free corridor must produce a segment");
  require(decision.executed_generation == 3U,
          "accepted segment must echo its exact execution generation");
  require(decision.path.size() >= 2U && decision.path.size() <= config.max_waypoints,
          "segment waypoint count must stay bounded");
  require(decision.path.back().x > input.robot_pose.x &&
              decision.path.back().x - input.robot_pose.x <= config.max_segment_length_m,
          "segment must be a bounded prefix toward the target");
}

void testRejectsUnknownOccupiedAndUnsafeTerrain() {
  auto constrained = grid(3, 3, lingtu::explore::kUnknown);
  constrained.cells[static_cast<std::size_t>(constrained.index(1, 0))] = lingtu::explore::kFree;

  auto unknown_input = inputFor(constrained, riskLike(constrained), rollingIdentity(1U));
  RollingMapSegmentExecutor unknown_executor;
  require(unknown_executor.plan(unknown_input, target(2.5, 1.5, 1U)).action ==
              RollingMapSegmentAction::Rejected,
          "unknown cells must never form a segment prefix");

  auto occupied = constrained;
  occupied.cells[static_cast<std::size_t>(occupied.index(1, 1))] = lingtu::explore::kOccupied;
  auto occupied_input = inputFor(occupied, riskLike(occupied), rollingIdentity(1U));
  RollingMapSegmentExecutor occupied_executor;
  require(occupied_executor.plan(occupied_input, target(2.5, 1.5, 1U)).action ==
              RollingMapSegmentAction::Rejected,
          "occupied cells must never form a segment prefix");

  auto free = constrained;
  free.cells[static_cast<std::size_t>(free.index(1, 1))] = lingtu::explore::kFree;
  auto unsafe_risk = riskLike(free);
  unsafe_risk.costs[static_cast<std::size_t>(free.index(1, 1))] = 75.0F;
  auto cost_input = inputFor(free, std::move(unsafe_risk), rollingIdentity(1U));
  RollingMapSegmentExecutor cost_executor;
  require(cost_executor.plan(cost_input, target(2.5, 1.5, 1U)).action ==
              RollingMapSegmentAction::Rejected,
          "terrain cost above the risk threshold must block a prefix");
}

void testAcceptsNewerGenerationWithinEpoch() {
  auto occupancy = grid(8, 3, lingtu::explore::kFree);
  auto first = inputFor(occupancy, riskLike(occupancy), rollingIdentity(4U), 10.0);
  RollingMapSegmentExecutor executor;
  require(executor.plan(first, target(20.5, 1.5, 4U)).action == RollingMapSegmentAction::Accepted,
          "first same-epoch generation must be accepted");

  auto next = inputFor(occupancy, riskLike(occupancy), rollingIdentity(5U), 11.0);
  const auto decision = executor.plan(next, target(20.5, 1.5, 5U));
  require(decision.action == RollingMapSegmentAction::Accepted,
          "newer generation in the same epoch must be accepted");
  require(decision.executed_generation == 5U, "newer accepted generation must be echoed exactly");
}

void testRevalidationCancelsOnResetAndStaleInputs() {
  auto occupancy = grid(8, 3, lingtu::explore::kFree);
  auto initial = inputFor(occupancy, riskLike(occupancy), rollingIdentity(7U), 10.0);
  RollingMapSegmentExecutor reset_executor;
  require(reset_executor.plan(initial, target(20.5, 1.5, 7U)).action ==
              RollingMapSegmentAction::Accepted,
          "reset cancellation setup must accept an initial segment");

  auto reset = inputFor(occupancy, riskLike(occupancy), rollingIdentity(1U, 2U), 11.0);
  const auto reset_decision = reset_executor.revalidate(reset);
  require(reset_decision.action == RollingMapSegmentAction::Cancel &&
              reset_decision.executed_generation == 7U,
          "a reset epoch must cancel the active generation");

  RollingMapSegmentExecutor stale_executor;
  require(stale_executor.plan(initial, target(20.5, 1.5, 7U)).action ==
              RollingMapSegmentAction::Accepted,
          "stale cancellation setup must accept an initial segment");
  auto stale = initial;
  stale.now_s = 12.0;
  const auto stale_decision = stale_executor.revalidate(stale);
  require(stale_decision.action == RollingMapSegmentAction::Cancel &&
              stale_decision.reason == "rolling_map_snapshot_stale",
          "stale rolling-map input must cancel the active segment");
}
void testRevalidationKeepsOnlyUnconsumedSafeSuffix() {
  const auto base_occupancy = grid(12, 3, lingtu::explore::kFree);
  RollingMapSegmentExecutorConfig config;
  config.max_segment_length_m = 5.5;
  config.max_waypoints = 8U;

  for (const std::int8_t consumed_state : {
           lingtu::explore::kOccupied,
           lingtu::explore::kUnknown,
       }) {
    auto initial = inputFor(base_occupancy, riskLike(base_occupancy), rollingIdentity(7U), 10.0);
    RollingMapSegmentExecutor executor(config);
    const auto accepted = executor.plan(initial, target(20.5, 1.5, 7U));
    require(accepted.action == RollingMapSegmentAction::Accepted && accepted.path.size() >= 4U,
            "test setup requires a multi-waypoint forward segment");

    auto updated_occupancy = base_occupancy;
    setCellAtPose(updated_occupancy, accepted.path[0U], consumed_state);
    setCellAtPose(updated_occupancy, accepted.path[1U], consumed_state);
    auto updated =
        inputFor(updated_occupancy, riskLike(updated_occupancy), rollingIdentity(8U), 11.0);
    updated.robot_pose = accepted.path[2U];
    const auto decision = executor.revalidate(updated);
    require(decision.action == RollingMapSegmentAction::Noop &&
                decision.reason == "segment_revalidated" && executor.active(),
            "a blocked or unknown cell behind the robot must not cancel a safe suffix");
  }

  auto initial = inputFor(base_occupancy, riskLike(base_occupancy), rollingIdentity(7U), 10.0);
  RollingMapSegmentExecutor executor(config);
  const auto accepted = executor.plan(initial, target(20.5, 1.5, 7U));
  require(accepted.action == RollingMapSegmentAction::Accepted && accepted.path.size() >= 4U,
          "forward-block test setup requires a multi-waypoint segment");

  auto unsafe_occupancy = base_occupancy;
  setCellAtPose(unsafe_occupancy, accepted.path[3U], lingtu::explore::kUnknown);
  auto unsafe = inputFor(unsafe_occupancy, riskLike(unsafe_occupancy), rollingIdentity(8U), 11.0);
  unsafe.robot_pose = accepted.path[2U];
  const auto decision = executor.revalidate(unsafe);
  require(decision.action == RollingMapSegmentAction::Cancel &&
              decision.reason == "segment_path_no_longer_safe" && !executor.active(),
          "an unsafe forward waypoint must still cancel the active segment");
}

void testRevalidationRejectsUnsafeDiagonalTransition() {
  const auto base_occupancy = grid(7, 7, lingtu::explore::kFree);
  RollingMapSegmentExecutorConfig config;
  config.max_segment_length_m = 5.5;
  config.max_waypoints = 8U;

  auto initial = inputFor(base_occupancy, riskLike(base_occupancy), rollingIdentity(7U), 10.0);
  initial.robot_pose = {0.5, 0.5, 0.0};
  RollingMapSegmentExecutor executor(config);
  const auto accepted = executor.plan(initial, target(20.5, 20.5, 7U));
  require(accepted.action == RollingMapSegmentAction::Accepted && accepted.path.size() >= 3U,
          "diagonal no-corner-cut setup requires a forward diagonal segment");

  const auto &from = accepted.path[1U];
  const auto &to = accepted.path[2U];
  const int from_col =
      static_cast<int>(std::floor((from.x - base_occupancy.origin_x) / base_occupancy.resolution));
  const int from_row =
      static_cast<int>(std::floor((from.y - base_occupancy.origin_y) / base_occupancy.resolution));
  const int to_col =
      static_cast<int>(std::floor((to.x - base_occupancy.origin_x) / base_occupancy.resolution));
  const int to_row =
      static_cast<int>(std::floor((to.y - base_occupancy.origin_y) / base_occupancy.resolution));
  require(std::abs(to_row - from_row) == 1 && std::abs(to_col - from_col) == 1,
          "diagonal no-corner-cut setup must retain a diagonal transition");

  auto unsafe_occupancy = base_occupancy;
  unsafe_occupancy.cells[static_cast<std::size_t>(unsafe_occupancy.index(from_row, to_col))] =
      lingtu::explore::kUnknown;
  auto updated = inputFor(unsafe_occupancy, riskLike(unsafe_occupancy), rollingIdentity(8U), 11.0);
  updated.robot_pose = from;
  const auto decision = executor.revalidate(updated);
  require(decision.action == RollingMapSegmentAction::Cancel &&
              decision.reason == "segment_path_no_longer_safe" && !executor.active(),
          "revalidation must reject a diagonal corner beside an unknown cell");
}
}  // namespace

int main() {
  testSelectsBoundedSafePrefix();
  testRejectsUnknownOccupiedAndUnsafeTerrain();
  testAcceptsNewerGenerationWithinEpoch();
  testRevalidationKeepsOnlyUnconsumedSafeSuffix();
  testRevalidationRejectsUnsafeDiagonalTransition();
  testRevalidationCancelsOnResetAndStaleInputs();
  std::cout << "test_rolling_map_segment_executor passed\n";
  return 0;
}
