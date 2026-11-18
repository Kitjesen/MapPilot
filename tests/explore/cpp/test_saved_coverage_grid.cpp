#include <iostream>
#include <stdexcept>
#include <string>

#include "endpoint/saved_coverage_grid.hpp"
#include "planning/global/contract.hpp"

namespace {

using lingtu::explore::ExploreMapIdentity;
using lingtu::nav::endpoint::buildSavedCoverageGrid;
using lingtu::maps::OccupancyArtifactData;

[[noreturn]] void fail(const std::string &message) {
  throw std::runtime_error(message);
}

void require(bool condition, const std::string &message) {
  if (!condition) {
    fail(message);
  }
}

OccupancyArtifactData artifact() {
  OccupancyArtifactData map;
  map.cols = 3;
  map.rows = 2;
  map.resolution = 0.5;
  map.origin_x = -1.0;
  map.origin_y = 2.0;
  map.grid = {0, 0, 100, -1, 0, 100};
  return map;
}

lingtu::nav::plan::MapIdentity artifactIdentity() {
  return {"yard", 7, "map"};
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
  const auto result = buildSavedCoverageGrid(artifact(), artifactIdentity(), identity());

  require(result.ok(), "exact saved-map identity was rejected: " + result.reason);
  require(result.grid->width == 3 && result.grid->height == 2, "grid geometry changed");
  require(result.grid->resolution == 0.5, "grid resolution changed");
  require(result.grid->cells == artifact().grid, "grid cells changed");
}

void testMapVersionMismatchFailsClosed() {
  auto changed = identity();
  changed.map_content_epoch = 8;
  const auto result = buildSavedCoverageGrid(artifact(), artifactIdentity(), changed);

  require(!result.ok(), "mismatched saved-map version was accepted");
}

void testLiveRouteCannotUseTheSavedGridAdapter() {
  auto live = identity();
  live.live = true;
  live.map_id.clear();
  live.map_content_epoch = 0;
  const auto result = buildSavedCoverageGrid(artifact(), artifactIdentity(), live);

  require(!result.ok(), "live exploration accepted a saved coverage grid");
}

void testFrameMismatchIsRejected() {
  auto changed = artifactIdentity();
  changed.frame_id = "odom";
  const auto result = buildSavedCoverageGrid(artifact(), changed, identity());

  require(!result.ok(), "saved coverage grid with the wrong frame was accepted");
}

void testMalformedGridIsRejected() {
  auto malformed = artifact();
  malformed.grid.pop_back();
  const auto result = buildSavedCoverageGrid(malformed, artifactIdentity(), identity());

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
