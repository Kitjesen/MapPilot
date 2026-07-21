#include "tare_policy.hpp"

#include <cassert>
#include <iostream>
#include <string>

using lingtu::explore::ExploreInput;
using lingtu::explore::Grid2D;
using lingtu::explore::IExplorePlanner;
using lingtu::explore::Pose2D;
using lingtu::explore::TarePolicy;
using lingtu::explore::TarePolicyConfig;
using lingtu::explore::kFree;
using lingtu::explore::kOccupied;
using lingtu::explore::kUnknown;

namespace {

Grid2D MakeGrid(
    int width,
    int height,
    double resolution,
    std::int8_t fill) {
  Grid2D grid;
  grid.width = width;
  grid.height = height;
  grid.resolution = resolution;
  grid.cells.assign(
      static_cast<std::size_t>(width * height),
      fill);
  return grid;
}

Grid2D MakeFrontierGrid() {
  Grid2D grid = MakeGrid(16, 10, 1.0, kFree);
  for (int row = 0; row < grid.height; ++row) {
    grid.cells[static_cast<std::size_t>(
        grid.index(row, grid.width - 1))] = kUnknown;
  }
  return grid;
}

ExploreInput MakeInput(
    Grid2D grid,
    Pose2D robot,
    std::uint64_t generation,
    std::uint64_t reset_epoch = 1U,
    std::string session_id = "test-session") {
  ExploreInput input;
  input.exploration_grid = std::move(grid);
  input.robot_pose = robot;
  input.stamp_s = static_cast<double>(generation);
  input.map_frame = "map";
  input.map.frame_id = "map";
  input.map.session_id = std::move(session_id);
  input.map.reset_epoch = reset_epoch;
  input.map.generation = generation;
  input.map.live = true;
  return input;
}

void TestSelectsReachableFrontierViewpoint() {
  TarePolicy policy;
  const auto decision = policy.plan(MakeInput(
      MakeFrontierGrid(),
      Pose2D{2.5, 3.5, 0.0},
      1U));

  assert(decision.has_goal);
  assert(!decision.done);
  assert(decision.reason == "selected_global_route" ||
         decision.reason == "selected_local_coverage");
  assert(!decision.candidates.empty());
  assert(!decision.route.empty());
  assert(decision.goal_x > 2.5);
  assert(decision.diagnostics.state_committed);
  assert(decision.diagnostics.accepted_generation == 1U);
  assert(decision.diagnostics.keypose_nodes == 1U);
}

void TestUsesCommonPlannerContract() {
  TarePolicy policy;
  IExplorePlanner& planner = policy;
  const auto decision = planner.plan(MakeInput(
      MakeFrontierGrid(),
      Pose2D{2.5, 3.5, 0.0},
      1U));

  assert(std::string(planner.name()) == "tare_hierarchical");
  assert(decision.has_goal);
}

void TestRejectsMissingIdentityAndFrameMismatch() {
  TarePolicy policy;
  ExploreInput missing;
  missing.exploration_grid = MakeFrontierGrid();
  missing.robot_pose = {2.5, 3.5, 0.0};
  assert(policy.plan(missing).reason == "invalid_map_identity");

  ExploreInput mismatch = MakeInput(
      MakeFrontierGrid(),
      Pose2D{2.5, 3.5, 0.0},
      1U);
  mismatch.map_frame = "odom";
  const auto decision = policy.plan(mismatch);
  assert(decision.reason == "map_frame_mismatch");
  assert(!decision.diagnostics.state_committed);
}

void TestRejectsStaleGenerationWithoutStateCommit() {
  TarePolicy policy;
  const auto accepted = policy.plan(MakeInput(
      MakeFrontierGrid(),
      Pose2D{2.5, 3.5, 0.0},
      1U));
  assert(accepted.diagnostics.state_committed);

  const auto stale = policy.plan(MakeInput(
      MakeFrontierGrid(),
      Pose2D{3.5, 3.5, 0.0},
      1U));
  assert(stale.reason == "stale_map_generation");
  assert(!stale.diagnostics.state_committed);
  assert(policy.diagnostics().accepted_generation == 1U);
}

void TestResetEpochClearsPlannerState() {
  TarePolicy policy;
  const auto first = policy.plan(MakeInput(
      MakeFrontierGrid(),
      Pose2D{2.5, 3.5, 0.0},
      1U,
      1U));
  assert(first.diagnostics.keypose_nodes == 1U);

  const auto moved = policy.plan(MakeInput(
      MakeFrontierGrid(),
      Pose2D{5.5, 3.5, 0.0},
      2U,
      1U));
  assert(moved.diagnostics.keypose_nodes == 2U);

  const auto reset = policy.plan(MakeInput(
      MakeFrontierGrid(),
      Pose2D{5.5, 3.5, 0.0},
      1U,
      2U));
  assert(reset.diagnostics.state_committed);
  assert(reset.diagnostics.keypose_nodes == 1U);
  assert(reset.diagnostics.reset_count == 2U);
}

void TestCancellationDoesNotCommit() {
  TarePolicy policy;
  const auto decision = policy.plan(
      MakeInput(
          MakeFrontierGrid(),
          Pose2D{2.5, 3.5, 0.0},
          1U),
      []() { return true; });

  assert(decision.reason == "cancelled");
  assert(!decision.diagnostics.state_committed);
  assert(policy.diagnostics().accepted_generation == 0U);
}

void TestGridResourceLimitFailsClosed() {
  TarePolicyConfig config;
  config.max_grid_cells = 32U;
  TarePolicy policy(config);
  const auto decision = policy.plan(MakeInput(
      MakeFrontierGrid(),
      Pose2D{2.5, 3.5, 0.0},
      1U));

  assert(decision.reason == "resource_limit_grid_cells");
  assert(!decision.has_goal);
  assert(!decision.diagnostics.state_committed);
}

void TestReturnsAlongKeyposeGraphWhenCoverageCompletes() {
  TarePolicy policy;
  const auto first = policy.plan(MakeInput(
      MakeFrontierGrid(),
      Pose2D{2.5, 3.5, 0.0},
      1U));
  assert(first.has_goal);

  const auto moved = policy.plan(MakeInput(
      MakeFrontierGrid(),
      Pose2D{5.5, 3.5, 0.0},
      2U));
  assert(moved.diagnostics.keypose_nodes == 2U);

  Grid2D complete = MakeGrid(16, 10, 1.0, kFree);
  const auto returning = policy.plan(MakeInput(
      std::move(complete),
      Pose2D{5.5, 3.5, 0.0},
      3U));
  assert(returning.has_goal);
  assert(!returning.done);
  assert(returning.reason == "returning_home");
  assert(returning.diagnostics.phase == "return_home");
  assert(returning.route.size() >= 2U);
  assert(returning.goal_x < 5.5);
}

void TestNoFrontiersAtHomeMeansDone() {
  TarePolicy policy;
  Grid2D grid = MakeGrid(8, 8, 1.0, kFree);
  const auto decision = policy.plan(MakeInput(
      std::move(grid),
      Pose2D{2.5, 2.5, 0.0},
      1U));

  assert(!decision.has_goal);
  assert(decision.done);
  assert(decision.reason == "exploration_complete");
  assert(decision.diagnostics.phase == "complete");
}

void TestRejectsRobotOutsideFreeSpace() {
  Grid2D grid = MakeGrid(8, 8, 1.0, kFree);
  grid.cells[static_cast<std::size_t>(
      grid.index(2, 2))] = kOccupied;

  TarePolicy policy;
  const auto decision = policy.plan(MakeInput(
      std::move(grid),
      Pose2D{2.5, 2.5, 0.0},
      1U));

  assert(!decision.has_goal);
  assert(decision.reason == "robot_not_in_free_space");
  assert(!decision.diagnostics.state_committed);
}

void TestFloatGridMetadataKeepsBoundaryStable() {
  Grid2D grid = MakeGrid(
      150,
      150,
      static_cast<double>(0.2F),
      kUnknown);
  grid.origin_x = -15.0;
  grid.origin_y = -15.0;
  grid.cells[static_cast<std::size_t>(
      grid.index(75, 75))] = kFree;

  TarePolicyConfig config;
  config.min_frontier_size = 1;
  TarePolicy policy(config);
  const auto decision = policy.plan(MakeInput(
      std::move(grid),
      Pose2D{0.0, 0.0, 0.0},
      1U));

  assert(decision.reason != "robot_not_in_free_space");
}

void TestVisitedGoalSuppressesCandidate() {
  TarePolicyConfig config;
  config.novelty_radius_m = 30.0;
  TarePolicy policy(config);
  ExploreInput input = MakeInput(
      MakeFrontierGrid(),
      Pose2D{2.5, 2.5, 0.0},
      1U);
  input.visited_goals = {Pose2D{13.5, 2.5, 0.0}};
  const auto decision = policy.plan(input);

  assert(!decision.has_goal);
  assert(decision.reason == "no_reachable_viewpoint");
  assert(decision.diagnostics.state_committed);
}

}  // namespace

int main() {
  TestSelectsReachableFrontierViewpoint();
  TestUsesCommonPlannerContract();
  TestRejectsMissingIdentityAndFrameMismatch();
  TestRejectsStaleGenerationWithoutStateCommit();
  TestResetEpochClearsPlannerState();
  TestCancellationDoesNotCommit();
  TestGridResourceLimitFailsClosed();
  TestReturnsAlongKeyposeGraphWhenCoverageCompletes();
  TestNoFrontiersAtHomeMeansDone();
  TestRejectsRobotOutsideFreeSpace();
  TestFloatGridMetadataKeepsBoundaryStable();
  TestVisitedGoalSuppressesCandidate();
  std::cout << "test_tare_policy passed\n";
  return 0;
}
