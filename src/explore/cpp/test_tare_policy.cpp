#include "tare_policy.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using lingtu::explore::Grid2D;
using lingtu::explore::ExploreInput;
using lingtu::explore::IExplorePlanner;
using lingtu::explore::Pose2D;
using lingtu::explore::TarePolicy;
using lingtu::explore::TarePolicyConfig;
using lingtu::explore::kFree;
using lingtu::explore::kOccupied;
using lingtu::explore::kUnknown;

namespace {

Grid2D makeGrid(int width, int height, double resolution, std::int8_t fill) {
  Grid2D grid;
  grid.width = width;
  grid.height = height;
  grid.resolution = resolution;
  grid.cells.assign(static_cast<std::size_t>(width * height), fill);
  return grid;
}

void testSelectsReachableFrontierViewpoint() {
  Grid2D grid = makeGrid(12, 8, 1.0, kFree);
  for (int row = 0; row < grid.height; ++row) {
    grid.cells[static_cast<std::size_t>(grid.index(row, grid.width - 1))] = kUnknown;
  }

  TarePolicy policy;
  const auto decision = policy.select(grid, Pose2D{2.5, 3.5, 0.0});

  assert(decision.has_goal);
  assert(!decision.done);
  assert(decision.reason == "selected_viewpoint");
  assert(!decision.candidates.empty());
  assert(decision.goal_x > 2.5);
}

void testUsesCommonExplorePlannerContract() {
  Grid2D grid = makeGrid(12, 8, 1.0, kFree);
  for (int row = 0; row < grid.height; ++row) {
    grid.cells[static_cast<std::size_t>(grid.index(row, grid.width - 1))] = kUnknown;
  }

  TarePolicy policy;
  const IExplorePlanner& planner = policy;
  ExploreInput input;
  input.exploration_grid = grid;
  input.robot_pose = Pose2D{2.5, 3.5, 0.0};
  const auto decision = planner.plan(input);

  assert(std::string(planner.name()) == "frontier_viewpoint");
  assert(decision.has_goal);
  assert(decision.reason == "selected_viewpoint");
}

void testNoFrontiersMeansDone() {
  Grid2D grid = makeGrid(8, 8, 1.0, kFree);
  TarePolicy policy;
  const auto decision = policy.select(grid, Pose2D{2.5, 2.5, 0.0});

  assert(!decision.has_goal);
  assert(decision.done);
  assert(decision.reason == "no_frontiers");
}

void testRejectsRobotOutsideFreeSpace() {
  Grid2D grid = makeGrid(8, 8, 1.0, kFree);
  grid.cells[static_cast<std::size_t>(grid.index(2, 2))] = kOccupied;

  TarePolicy policy;
  const auto decision = policy.select(grid, Pose2D{2.5, 2.5, 0.0});

  assert(!decision.has_goal);
  assert(!decision.done);
  assert(decision.reason == "robot_not_in_free_space");
}

void testVisitedGoalSuppressesCandidate() {
  Grid2D grid = makeGrid(10, 6, 1.0, kFree);
  for (int row = 0; row < grid.height; ++row) {
    grid.cells[static_cast<std::size_t>(grid.index(row, grid.width - 1))] = kUnknown;
  }

  TarePolicyConfig config;
  config.novelty_radius_m = 20.0;
  TarePolicy policy(config);
  const auto decision = policy.select(grid, Pose2D{2.5, 2.5, 0.0}, {Pose2D{8.5, 2.5, 0.0}});

  assert(!decision.has_goal);
  assert(decision.reason == "no_reachable_viewpoint");
}

}  // namespace

int main() {
  testSelectsReachableFrontierViewpoint();
  testUsesCommonExplorePlannerContract();
  testNoFrontiersMeansDone();
  testRejectsRobotOutsideFreeSpace();
  testVisitedGoalSuppressesCandidate();
  std::cout << "test_tare_policy passed\n";
  return 0;
}
