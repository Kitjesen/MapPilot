#include "semantic_views.hpp"

#include <cassert>
#include <cmath>
#include <limits>

using namespace lingtu::explore;

namespace {
ExploreInput Input() {
  ExploreInput input;
  input.map = {"map", "product-test", "saved-floor", 3, 1, 1, false};
  input.map_frame = "map";
  input.stamp_s = 10.0;
  input.robot_pose = {5.5, 5.5, 0.0};
  auto& grid = input.exploration_grid;
  grid.width = grid.height = 11;
  grid.resolution = 1.0;
  grid.cells.assign(121, kFree);
  return input;
}

TarePolicyConfig Config() {
  TarePolicyConfig config;
  config.sensor_range_m = 3.0;
  config.sensor_horizontal_fov_rad = 1.0;
  config.max_plan_time_ms = 5000.0;
  return config;
}

void KnownMapDoesNotProveCameraCoverage() {
  const auto input = Input();
  const auto heights = std::vector<double>(121, 1.25);
  const auto first = ProposeSemanticViews(input, heights, {input.map, {}}, Config());
  assert(!first.candidates.empty() && !first.geometry_exhausted);
  assert(first.diagnostics.covered_reachable_cells == 0);
  assert(!first.diagnostics.state_committed);
  assert(first.map.sameSource(input.map));
  assert(!first.candidates.empty());
  for (const auto& candidate : first.candidates) {
    assert(candidate.z == 1.25);
  }
  const auto repeated = ProposeSemanticViews(input, heights, {input.map, {}}, Config());
  assert(first.candidates.front().x == repeated.candidates.front().x);
  assert(first.candidates.front().y == repeated.candidates.front().y);
  assert(first.candidates.front().yaw == repeated.candidates.front().yaw);
  assert(input.exploration_grid.cells == std::vector<std::int8_t>(121, kFree));
}

void OppositeCameraViewsAddCoverageOnAnUnchangedMap() {
  const auto input = Input();
  const auto heights = std::vector<double>(121, 1.25);
  SemanticSearchHistory history{input.map, {{input.robot_pose, 4.0, 1.0}}};
  const auto front = ProposeSemanticViews(input, heights, history, Config());
  assert(!front.candidates.empty());
  assert(front.diagnostics.covered_reachable_cells > 0);
  assert(front.diagnostics.covered_reachable_cells < front.diagnostics.reachable_free_cells);
  auto reverse = history.views.front();
  reverse.pose.yaw = 3.14159265358979323846;
  history.views.push_back(reverse);
  const auto both = ProposeSemanticViews(input, heights, history, Config());
  assert(both.diagnostics.covered_reachable_cells > front.diagnostics.covered_reachable_cells);
  assert(both.diagnostics.uncovered_reachable_cells > 0);
}

void AReachableRoomBehindAWallRemainsUnobserved() {
  auto input = Input();
  const auto heights = std::vector<double>(121, 1.25);
  SemanticSearchHistory history{input.map, {{input.robot_pose, 20.0, 1.0}}};
  const auto open = ProposeSemanticViews(input, heights, history, Config());
  // The end rows remain open, so reachability is preserved around this wall.
  for (int row = 1; row < 10; ++row) {
    input.exploration_grid.cells[input.exploration_grid.index(row, 7)] = kOccupied;
  }
  const auto occluded = ProposeSemanticViews(input, heights, history, Config());
  assert(!occluded.candidates.empty());
  assert(occluded.diagnostics.reachable_free_cells == 112);
  assert(occluded.diagnostics.covered_reachable_cells < open.diagnostics.covered_reachable_cells);
}

void UnsupportedCellsCannotConnectCandidates() {
  const auto input = Input();
  auto heights = std::vector<double>(121, 1.25);
  for (int row = 0; row < 11; ++row) {
    heights[input.exploration_grid.index(row, 6)] = std::numeric_limits<double>::quiet_NaN();
  }
  const auto result = ProposeSemanticViews(input, heights, {input.map, {}}, Config());
  assert(!result.candidates.empty());
  assert(result.diagnostics.reachable_free_cells == 66);
  for (const auto& candidate : result.candidates) {
    assert(candidate.x < 6.0 && std::isfinite(candidate.z));
  }
}

void RejectsWrongHistoryAndInvalidViews() {
  const auto input = Input();
  const auto heights = std::vector<double>(121, 1.25);
  SemanticSearchHistory history{input.map, {}};
  history.map.map_content_epoch += 1;
  auto result = ProposeSemanticViews(input, heights, history, Config());
  assert(result.candidates.empty() && result.reason == "semantic_search_history_map_mismatch");
  history.map = input.map;
  history.map.reset_epoch += 1;
  result = ProposeSemanticViews(input, heights, history, Config());
  assert(result.candidates.empty() && result.reason == "semantic_search_history_map_mismatch");
  history.map = input.map;
  history.views.push_back({input.robot_pose, 0.0, 1.0});
  result = ProposeSemanticViews(input, heights, history, Config());
  assert(result.candidates.empty() && result.reason == "invalid_camera_search_view");
  assert(ProposeSemanticViews(input, {}, {input.map, {}}, Config()).candidates.empty());
}

void ExhaustionIsNotSemanticSuccess() {
  const auto input = Input();
  const auto result = ProposeSemanticViews(input, std::vector<double>(121, 1.25),
      {input.map, {{input.robot_pose, 20.0, 6.283185307179586}}}, Config());
  assert(result.geometry_exhausted && result.candidates.empty());
  assert(result.reason == "camera_search_geometry_covered");
  assert(!result.diagnostics.state_committed);
}

void CancellationReturnsNoProposals() {
  const auto input = Input();
  const auto result = ProposeSemanticViews(input, std::vector<double>(121, 1.25),
      {input.map, {{input.robot_pose, 4.0, 1.0}}}, Config(), [] { return true; });
  assert(result.candidates.empty() && !result.geometry_exhausted);
  assert(result.reason == "cancelled");
}

void ACameraCanTurnAtAnAlreadyVisitedPosition() {
  auto input = Input();
  auto& grid = input.exploration_grid;
  grid.width = grid.height = 7;
  grid.resolution = 0.1;
  grid.cells.assign(49, kFree);
  input.robot_pose = {0.35, 0.35, 0.0};
  const auto result = ProposeSemanticViews(input, std::vector<double>(49, 0.45),
      {input.map, {{input.robot_pose, 3.0, 1.0}}}, Config());
  assert(result.map.valid() && !result.geometry_exhausted);
  assert(!result.candidates.empty());
  const auto& next = result.candidates.front();
  assert(std::hypot(next.x - input.robot_pose.x, next.y - input.robot_pose.y) < 1e-9);
  assert(std::abs(std::remainder(next.yaw, 6.283185307179586)) > 0.5);
  assert(next.frontier_size > 0 && next.z == 0.45);
  assert(result.diagnostics.phase == "camera_turn");

  SemanticSearchHistory history{input.map, {{input.robot_pose, 3.0, 1.0}}};
  auto previous_coverage = result.diagnostics.covered_reachable_cells;
  for (int attempt = 0; attempt < 12; ++attempt) {
    const auto proposal = ProposeSemanticViews(input, std::vector<double>(49, 0.45), history, Config());
    if (proposal.geometry_exhausted) return;
    assert(!proposal.candidates.empty());
    const auto& view = proposal.candidates.front();
    history.views.push_back({{view.x, view.y, view.yaw}, 3.0, 1.0});
    const auto after = ProposeSemanticViews(input, std::vector<double>(49, 0.45), history, Config());
    assert(after.diagnostics.covered_reachable_cells > previous_coverage);
    previous_coverage = after.diagnostics.covered_reachable_cells;
  }
  assert(false && "turning search repeatedly selected a view with no coverage progress");
}
}  // namespace

int main() {
  KnownMapDoesNotProveCameraCoverage();
  OppositeCameraViewsAddCoverageOnAnUnchangedMap();
  AReachableRoomBehindAWallRemainsUnobserved();
  UnsupportedCellsCannotConnectCandidates();
  RejectsWrongHistoryAndInvalidViews();
  ExhaustionIsNotSemanticSuccess();
  CancellationReturnsNoProposals();
  ACameraCanTurnAtAnAlreadyVisitedPosition();
}
