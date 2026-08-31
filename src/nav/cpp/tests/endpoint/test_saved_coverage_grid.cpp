#include <iostream>
#include <stdexcept>
#include <string>

#include "explore/saved_coverage_grid.hpp"
#include "planning/global/far/planner.hpp"

namespace {

using lingtu::explore::ExploreMapIdentity;
using lingtu::nav::endpoint::buildSavedCoverageGrid;
using lingtu::nav::plan::far_planner::FarGridMap;

[[noreturn]] void fail(const std::string &message) {
  throw std::runtime_error(message);
}

void require(bool condition, const std::string &message) {
  if (!condition) {
    fail(message);
  }
}

FarGridMap artifact() {
  FarGridMap map;
  map.width = 3;
  map.height = 2;
  map.resolution_m = 0.5;
  map.origin_x_m = -1.0;
  map.origin_y_m = 2.0;
  map.frame_id = "map";
  map.generation = 1U;
  map.identity.map_id = "yard";
  map.identity.content_epoch = 7;
  map.identity.frame_id = "map";
  map.cells = {0, 0, 100, -1, 0, 100};
  return map;
}

ExploreMapIdentity identity() {
  ExploreMapIdentity value;
  value.frame_id = "map";
  value.session_id = "product-session-0001";
  value.map_id = "yard";
  value.map_content_epoch = 7;
  value.reset_epoch = 1U;
  value.generation = 1U;
  value.live = false;
  return value;
}

void testExactSavedMapIdentityConvertsTheStaticGrid() {
  const auto result = buildSavedCoverageGrid(artifact(), identity());

  require(result.ok(), "exact saved-map identity was rejected: " + result.reason);
  require(result.grid->width == 3 && result.grid->height == 2, "grid geometry changed");
  require(result.grid->resolution == 0.5, "grid resolution changed");
  require(result.grid->cells == artifact().cells, "grid cells changed");
}

void testMapVersionMismatchFailsClosed() {
  auto changed = identity();
  changed.map_content_epoch = 8;
  const auto result = buildSavedCoverageGrid(artifact(), changed);

  require(!result.ok(), "mismatched saved-map version was accepted");
}

void testLiveRouteCannotUseTheSavedGridAdapter() {
  auto live = identity();
  live.live = true;
  live.map_id.clear();
  live.map_content_epoch = 0;
  const auto result = buildSavedCoverageGrid(artifact(), live);

  require(!result.ok(), "live exploration accepted a saved coverage grid");
}

void testFrameMismatchIsRejected() {
  auto changed = artifact();
  changed.frame_id = "odom";
  const auto result = buildSavedCoverageGrid(changed, identity());

  require(!result.ok(), "saved coverage grid with the wrong frame was accepted");
}

void testMalformedGridIsRejected() {
  auto malformed = artifact();
  malformed.cells.pop_back();
  const auto result = buildSavedCoverageGrid(malformed, identity());

  require(!result.ok(), "saved coverage grid with incomplete cells was accepted");
}

}  // namespace

int main() {
  try {
    testExactSavedMapIdentityConvertsTheStaticGrid();
    testMapVersionMismatchFailsClosed();
    testLiveRouteCannotUseTheSavedGridAdapter();
    testFrameMismatchIsRejected();
    testMalformedGridIsRejected();
    std::cout << "test_saved_coverage_grid: PASS\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "test_saved_coverage_grid: FAIL: " << error.what() << "\n";
    return 1;
  }
}
