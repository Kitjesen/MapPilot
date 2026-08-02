#include <algorithm>
#include <cassert>
#include <iostream>
#include <string>

#include "tare_policy.hpp"

using lingtu::explore::DirectedTarget;
using lingtu::explore::ExploreInput;
using lingtu::explore::Grid2D;
using lingtu::explore::IExplorePlanner;
using lingtu::explore::kFree;
using lingtu::explore::kOccupied;
using lingtu::explore::kUnknown;
using lingtu::explore::Pose2D;
using lingtu::explore::TarePolicy;
using lingtu::explore::TarePolicyConfig;

namespace {

Grid2D MakeGrid(int width, int height, double resolution, std::int8_t fill) {
  Grid2D grid;
  grid.width = width;
  grid.height = height;
  grid.resolution = resolution;
  grid.cells.assign(static_cast<std::size_t>(width * height), fill);
  return grid;
}

Grid2D MakeFrontierGrid() {
  Grid2D grid = MakeGrid(16, 10, 1.0, kFree);
  for (int row = 0; row < grid.height; ++row) {
    grid.cells[static_cast<std::size_t>(grid.index(row, grid.width - 1))] = kUnknown;
  }
  return grid;
}

Grid2D MakeSymmetricFrontierGrid() {
  Grid2D grid = MakeGrid(21, 11, 1.0, kFree);
  for (int row = 0; row < grid.height; ++row) {
    grid.cells[static_cast<std::size_t>(grid.index(row, 0))] = kUnknown;
    grid.cells[static_cast<std::size_t>(grid.index(row, grid.width - 1))] = kUnknown;
  }
  return grid;
}

ExploreInput MakeInput(Grid2D grid, Pose2D robot, std::uint64_t generation,
                       std::uint64_t reset_epoch = 1U, std::string session_id = "test-session") {
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

ExploreInput MakeKnownMapInput(Grid2D grid, Pose2D robot, std::uint64_t generation) {
  ExploreInput input = MakeInput(std::move(grid), robot, generation, 1U, "known-map-session");
  input.map.map_id = "known-map";
  input.map.map_version = 3;
  input.map.artifact_hash = std::string(64U, 'a');
  input.map.live = false;
  return input;
}

void TestSelectsReachableFrontierViewpoint() {
  TarePolicy policy;
  const auto decision = policy.plan(MakeInput(MakeFrontierGrid(), Pose2D{2.5, 3.5, 0.0}, 1U));

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
  IExplorePlanner &planner = policy;
  const auto decision = planner.plan(MakeInput(MakeFrontierGrid(), Pose2D{2.5, 3.5, 0.0}, 1U));

  assert(std::string(planner.name()) == "tare_hierarchical");
  assert(decision.has_goal);
}

void TestRejectsMissingIdentityAndFrameMismatch() {
  TarePolicy policy;
  ExploreInput missing;
  missing.exploration_grid = MakeFrontierGrid();
  missing.robot_pose = {2.5, 3.5, 0.0};
  assert(policy.plan(missing).reason == "invalid_map_identity");

  ExploreInput mismatch = MakeInput(MakeFrontierGrid(), Pose2D{2.5, 3.5, 0.0}, 1U);
  mismatch.map_frame = "odom";
  const auto decision = policy.plan(mismatch);
  assert(decision.reason == "map_frame_mismatch");
  assert(!decision.diagnostics.state_committed);
}

void TestRejectsStaleGenerationWithoutStateCommit() {
  TarePolicy policy;
  const auto accepted = policy.plan(MakeInput(MakeFrontierGrid(), Pose2D{2.5, 3.5, 0.0}, 1U));
  assert(accepted.diagnostics.state_committed);

  const auto stale = policy.plan(MakeInput(MakeFrontierGrid(), Pose2D{3.5, 3.5, 0.0}, 1U));
  assert(stale.reason == "stale_map_generation");
  assert(!stale.diagnostics.state_committed);
  assert(policy.diagnostics().accepted_generation == 1U);
}

void TestResetEpochClearsPlannerState() {
  TarePolicy policy;
  const auto first = policy.plan(MakeInput(MakeFrontierGrid(), Pose2D{2.5, 3.5, 0.0}, 1U, 1U));
  assert(first.diagnostics.keypose_nodes == 1U);

  const auto moved = policy.plan(MakeInput(MakeFrontierGrid(), Pose2D{5.5, 3.5, 0.0}, 2U, 1U));
  assert(moved.diagnostics.keypose_nodes == 2U);

  const auto reset = policy.plan(MakeInput(MakeFrontierGrid(), Pose2D{5.5, 3.5, 0.0}, 1U, 2U));
  assert(reset.diagnostics.state_committed);
  assert(reset.diagnostics.keypose_nodes == 1U);
  assert(reset.diagnostics.reset_count == 2U);
}

void TestCancellationDoesNotCommit() {
  TarePolicy policy;
  const auto decision =
      policy.plan(MakeInput(MakeFrontierGrid(), Pose2D{2.5, 3.5, 0.0}, 1U), []() { return true; });

  assert(decision.reason == "cancelled");
  assert(!decision.diagnostics.state_committed);
  assert(policy.diagnostics().accepted_generation == 0U);
}

void TestGridResourceLimitFailsClosed() {
  TarePolicyConfig config;
  config.max_grid_cells = 32U;
  TarePolicy policy(config);
  const auto decision = policy.plan(MakeInput(MakeFrontierGrid(), Pose2D{2.5, 3.5, 0.0}, 1U));

  assert(decision.reason == "resource_limit_grid_cells");
  assert(!decision.has_goal);
  assert(!decision.diagnostics.state_committed);
}

void TestReturnsAlongKeyposeGraphWhenCoverageCompletes() {
  TarePolicy policy;
  const auto first = policy.plan(MakeInput(MakeFrontierGrid(), Pose2D{2.5, 3.5, 0.0}, 1U));
  assert(first.has_goal);

  const auto moved = policy.plan(MakeInput(MakeFrontierGrid(), Pose2D{5.5, 3.5, 0.0}, 2U));
  assert(moved.diagnostics.keypose_nodes == 2U);

  Grid2D complete = MakeGrid(16, 10, 1.0, kFree);
  const auto returning = policy.plan(MakeInput(std::move(complete), Pose2D{5.5, 3.5, 0.0}, 3U));
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
  const auto decision = policy.plan(MakeInput(std::move(grid), Pose2D{2.5, 2.5, 0.0}, 1U));

  assert(!decision.has_goal);
  assert(decision.done);
  assert(decision.reason == "exploration_complete");
  assert(decision.diagnostics.phase == "complete");
}

void TestRejectsRobotOutsideFreeSpace() {
  Grid2D grid = MakeGrid(8, 8, 1.0, kFree);
  grid.cells[static_cast<std::size_t>(grid.index(2, 2))] = kOccupied;

  TarePolicy policy;
  const auto decision = policy.plan(MakeInput(std::move(grid), Pose2D{2.5, 2.5, 0.0}, 1U));

  assert(!decision.has_goal);
  assert(decision.reason == "robot_not_in_free_space");
  assert(!decision.diagnostics.state_committed);
}

void TestFloatGridMetadataKeepsBoundaryStable() {
  Grid2D grid = MakeGrid(150, 150, static_cast<double>(0.2F), kUnknown);
  grid.origin_x = -15.0;
  grid.origin_y = -15.0;
  grid.cells[static_cast<std::size_t>(grid.index(75, 75))] = kFree;

  TarePolicyConfig config;
  config.min_frontier_size = 1;
  TarePolicy policy(config);
  const auto decision = policy.plan(MakeInput(std::move(grid), Pose2D{0.0, 0.0, 0.0}, 1U));

  assert(decision.reason != "robot_not_in_free_space");
}

void TestVisitedGoalSuppressesCandidate() {
  TarePolicyConfig config;
  config.novelty_radius_m = 30.0;
  TarePolicy policy(config);
  ExploreInput input = MakeInput(MakeFrontierGrid(), Pose2D{2.5, 2.5, 0.0}, 1U);
  input.visited_goals = {Pose2D{13.5, 2.5, 0.0}};
  const auto decision = policy.plan(input);

  assert(!decision.has_goal);
  assert(decision.reason == "no_reachable_viewpoint");
  assert(decision.diagnostics.state_committed);
}

void TestDirectedTargetDefaultWeightIsNeutral() {
  ExploreInput baseline_input = MakeInput(MakeFrontierGrid(), Pose2D{2.5, 3.5, 0.0}, 1U);
  ExploreInput directed_input = baseline_input;
  directed_input.directed_target = DirectedTarget{100.0, 3.5};
  directed_input.directed_intent_revision = 1U;

  TarePolicy baseline_policy;
  TarePolicy directed_policy;
  const auto baseline = baseline_policy.plan(baseline_input);
  const auto directed = directed_policy.plan(directed_input);

  assert(directed.has_goal == baseline.has_goal);
  assert(directed.done == baseline.done);
  assert(directed.reason == baseline.reason);
  assert(directed.goal_x == baseline.goal_x);
  assert(directed.goal_y == baseline.goal_y);
  assert(directed.candidates.size() == baseline.candidates.size());
  for (std::size_t index = 0; index < baseline.candidates.size(); ++index) {
    assert(directed.candidates[index].x == baseline.candidates[index].x);
    assert(directed.candidates[index].y == baseline.candidates[index].y);
    assert(directed.candidates[index].score == baseline.candidates[index].score);
  }
  assert(directed.diagnostics.accepted_intent_revision == 1U);
}

void TestDirectedTargetFavorsProgress() {
  TarePolicyConfig config;
  config.travel_weight = 0.0;
  config.momentum_weight = 0.0;
  config.revisit_weight = 0.0;
  config.directed_progress_weight = 10.0;
  TarePolicy policy(config);

  ExploreInput input = MakeInput(MakeSymmetricFrontierGrid(), Pose2D{10.5, 5.5, 0.0}, 1U);
  input.directed_target = DirectedTarget{100.0, 5.5};
  input.directed_intent_revision = 1U;

  const auto decision = policy.plan(input);

  assert(decision.has_goal);
  assert(decision.goal_x > input.robot_pose.x);
  assert(decision.diagnostics.accepted_intent_revision == 1U);
}

void TestNewIntentRevisionReplansSameGeneration() {
  TarePolicy policy;
  ExploreInput first_input = MakeInput(MakeFrontierGrid(), Pose2D{2.5, 3.5, 0.0}, 1U);
  first_input.directed_target = DirectedTarget{100.0, 3.5};
  first_input.directed_intent_revision = 1U;
  const auto first = policy.plan(first_input);
  assert(first.diagnostics.state_committed);
  assert(first.diagnostics.accepted_intent_revision == 1U);

  ExploreInput clear_input = MakeInput(MakeFrontierGrid(), Pose2D{2.5, 3.5, 0.0}, 1U);
  clear_input.directed_intent_revision = 2U;
  const auto cleared = policy.plan(clear_input);
  assert(cleared.diagnostics.state_committed);
  assert(cleared.diagnostics.accepted_generation == 1U);
  assert(cleared.diagnostics.accepted_intent_revision == 2U);
  assert(cleared.diagnostics.covered_cells >= first.diagnostics.covered_cells);

  const auto stale = policy.plan(clear_input);
  assert(stale.reason == "stale_map_generation");
  assert(!stale.diagnostics.state_committed);
  assert(stale.diagnostics.accepted_intent_revision == 2U);
}

void TestDirectedTargetDoesNotBypassCandidateSafety() {
  Grid2D grid = MakeSymmetricFrontierGrid();
  for (int row = 0; row < grid.height; ++row) {
    grid.cells[static_cast<std::size_t>(grid.index(row, 10))] = kOccupied;
  }

  TarePolicyConfig config;
  config.directed_progress_weight = 100.0;
  TarePolicy policy(config);
  ExploreInput input = MakeInput(std::move(grid), Pose2D{4.5, 5.5, 0.0}, 1U);
  input.directed_target = DirectedTarget{100.0, 5.5};
  input.directed_intent_revision = 1U;

  const auto decision = policy.plan(input);

  assert(decision.has_goal);
  assert(decision.goal_x < 10.0);
  for (const auto &candidate : decision.candidates) {
    assert(candidate.x < 10.0);
  }
}

void TestKnownMapDoesNotCompleteBeforeReachableCoverage() {
  TarePolicyConfig config;
  config.return_home_when_done = false;
  config.sensor_range_m = 2.1;
  config.sensor_horizontal_fov_rad = 1.5707963267948966;
  TarePolicy policy(config);
  const auto decision =
      policy.plan(MakeKnownMapInput(MakeGrid(20, 10, 1.0, kFree), Pose2D{2.5, 5.5, 0.0}, 1U));

  assert(decision.has_goal);
  assert(!decision.done);
  assert(decision.diagnostics.reachable_free_cells == 200U);
  assert(decision.diagnostics.covered_reachable_cells > 0U);
  assert(decision.diagnostics.covered_reachable_cells < decision.diagnostics.reachable_free_cells);
  assert(decision.diagnostics.coverage_ratio > 0.0);
  assert(decision.diagnostics.coverage_ratio < 1.0);
  assert(std::isfinite(decision.goal_yaw));
  assert(!decision.route.empty());
  assert(std::abs(decision.goal_yaw - decision.route.front().yaw) < 1e-12);
  const auto selected = std::find_if(decision.candidates.begin(), decision.candidates.end(),
                                     [&decision](const auto &candidate) {
                                       return std::abs(candidate.x - decision.goal_x) < 1e-12 &&
                                              std::abs(candidate.y - decision.goal_y) < 1e-12;
                                     });
  assert(selected != decision.candidates.end());
  assert(std::abs(decision.goal_yaw - selected->yaw) < 1e-12);
}

void TestKnownMapCompletesAfterObservationCoversReachableComponent() {
  TarePolicyConfig config;
  config.return_home_when_done = false;
  TarePolicy policy(config);
  Grid2D known = MakeGrid(12, 8, 1.0, kFree);
  Grid2D first_observation = MakeGrid(6, 4, 2.0, kUnknown);
  Grid2D second_observation = MakeGrid(6, 4, 2.0, kUnknown);
  for (int row = 0; row < first_observation.height; ++row) {
    for (int col = 0; col < first_observation.width; ++col) {
      Grid2D &observation = col < 3 ? first_observation : second_observation;
      observation.cells[static_cast<std::size_t>(observation.index(row, col))] = kFree;
    }
  }
  ExploreInput first_input = MakeKnownMapInput(known, Pose2D{2.5, 3.5, 0.0}, 1U);
  first_input.live_observation_grid = std::move(first_observation);

  const auto partial = policy.plan(first_input);

  assert(!partial.done);
  assert(partial.diagnostics.coverage_ratio == 0.5);

  ExploreInput second_input = MakeKnownMapInput(std::move(known), Pose2D{8.5, 3.5, 0.0}, 2U);
  second_input.live_observation_grid = std::move(second_observation);

  const auto decision = policy.plan(second_input);

  assert(!decision.has_goal);
  assert(decision.done);
  assert(decision.reason == "exploration_complete");
  assert(decision.diagnostics.covered_reachable_cells == decision.diagnostics.reachable_free_cells);
  assert(decision.diagnostics.coverage_ratio == 1.0);
}

void TestKnownMapIgnoresUnreachableUnobservedFreeRegionForCompletion() {
  Grid2D known = MakeGrid(12, 6, 1.0, kFree);
  Grid2D observation = MakeGrid(12, 6, 1.0, kUnknown);
  for (int row = 0; row < known.height; ++row) {
    known.cells[static_cast<std::size_t>(known.index(row, 6))] = kOccupied;
    for (int col = 0; col < 6; ++col) {
      observation.cells[static_cast<std::size_t>(observation.index(row, col))] = kFree;
    }
  }

  TarePolicyConfig config;
  config.return_home_when_done = false;
  TarePolicy policy(config);
  ExploreInput input = MakeKnownMapInput(std::move(known), Pose2D{2.5, 2.5, 0.0}, 1U);
  input.live_observation_grid = std::move(observation);

  const auto decision = policy.plan(input);

  assert(decision.done);
  assert(decision.diagnostics.reachable_free_cells == 36U);
  assert(decision.diagnostics.covered_reachable_cells == 36U);
  assert(decision.diagnostics.coverage_ratio == 1.0);
}

void TestKnownMapForwardFovDoesNotMarkCellsBehindRobot() {
  TarePolicyConfig config;
  config.return_home_when_done = false;
  config.sensor_range_m = 2.1;
  config.sensor_horizontal_fov_rad = 1.5707963267948966;
  TarePolicy policy(config);
  const auto decision =
      policy.plan(MakeKnownMapInput(MakeGrid(9, 5, 1.0, kFree), Pose2D{4.5, 2.5, 0.0}, 1U));

  assert(!decision.done);
  assert(decision.diagnostics.covered_reachable_cells == 5U);
}

void TestKnownMapObservationEvidenceDoesNotFallBackToRadius() {
  TarePolicyConfig config;
  config.return_home_when_done = false;
  TarePolicy policy(config);
  ExploreInput input = MakeKnownMapInput(MakeGrid(9, 5, 1.0, kFree), Pose2D{4.5, 2.5, 0.0}, 1U);
  input.live_observation_grid = MakeGrid(5, 3, 1.0, kUnknown);
  input.live_observation_grid->origin_x = 2.0;
  input.live_observation_grid->origin_y = 1.0;

  const auto decision = policy.plan(input);

  assert(!decision.done);
  assert(decision.diagnostics.covered_reachable_cells == 0U);
}

void TestKnownMapLineOfSightDoesNotCoverBehindObstacle() {
  Grid2D known = MakeGrid(9, 7, 1.0, kFree);
  known.cells[static_cast<std::size_t>(known.index(3, 4))] = kOccupied;

  TarePolicyConfig config;
  config.return_home_when_done = false;
  config.sensor_range_m = 5.1;
  config.sensor_horizontal_fov_rad = 0.2;
  config.known_map_require_line_of_sight = true;
  TarePolicy policy(config);
  const auto decision = policy.plan(MakeKnownMapInput(std::move(known), Pose2D{2.5, 3.5, 0.0}, 1U));

  assert(!decision.done);
  assert(decision.diagnostics.covered_reachable_cells == 2U);
}

void TestKnownMapViewpointsStayReachableAndSamplingIsBounded() {
  Grid2D known = MakeGrid(12, 6, 1.0, kFree);
  for (int row = 0; row < known.height; ++row) {
    known.cells[static_cast<std::size_t>(known.index(row, 6))] = kOccupied;
  }

  TarePolicyConfig config;
  config.return_home_when_done = false;
  config.sensor_range_m = 0.6;
  config.max_candidates = 16;
  config.max_known_map_candidate_samples = 5U;
  TarePolicy policy(config);
  const auto decision = policy.plan(MakeKnownMapInput(std::move(known), Pose2D{2.5, 2.5, 0.0}, 1U));

  assert(decision.has_goal);
  assert(!decision.candidates.empty());
  assert(decision.candidates.size() <= 5U);
  for (const auto &candidate : decision.candidates) {
    assert(candidate.x < 6.0);
  }
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
  TestDirectedTargetDefaultWeightIsNeutral();
  TestDirectedTargetFavorsProgress();
  TestNewIntentRevisionReplansSameGeneration();
  TestDirectedTargetDoesNotBypassCandidateSafety();
  TestKnownMapDoesNotCompleteBeforeReachableCoverage();
  TestKnownMapCompletesAfterObservationCoversReachableComponent();
  TestKnownMapIgnoresUnreachableUnobservedFreeRegionForCompletion();
  TestKnownMapForwardFovDoesNotMarkCellsBehindRobot();
  TestKnownMapObservationEvidenceDoesNotFallBackToRadius();
  TestKnownMapLineOfSightDoesNotCoverBehindObstacle();
  TestKnownMapViewpointsStayReachableAndSamplingIsBounded();
  std::cout << "test_tare_policy passed\n";
  return 0;
}
