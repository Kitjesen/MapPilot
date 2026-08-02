#include <iostream>
#include <stdexcept>
#include <string>

#include "explore/saved_coverage_grid.hpp"

namespace {

using lingtu::explore::ExploreMapIdentity;
using lingtu::nav::endpoint::buildSavedCoverageGrid;
using lingtu::nav::plan::far::FarGridMap;

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
  map.identity.version = 7;
  map.identity.artifact_sha256 = std::string(64U, 'b');
  map.identity.frame_id = "map";
  map.cells = {0, 0, 100, -1, 0, 100};
  return map;
}

ExploreMapIdentity identity() {
  ExploreMapIdentity value;
  value.frame_id = "map";
  value.session_id = "product-session-0001";
  value.map_id = "yard";
  value.map_version = 7;
  value.artifact_hash = std::string(64U, 'a');
  value.reset_epoch = 1U;
  value.generation = 1U;
  value.live = false;
  return value;
}

void testExactSavedMapIdentityConvertsTheStaticGrid() {
  const auto source_hash = std::string(64U, 'a');
  const auto result = buildSavedCoverageGrid(artifact(), source_hash, identity());

  require(result.ok(), "exact saved-map identity was rejected: " + result.reason);
  require(result.grid->width == 3 && result.grid->height == 2, "grid geometry changed");
  require(result.grid->resolution == 0.5, "grid resolution changed");
  require(result.grid->cells == artifact().cells, "grid cells changed");
}

void testPointCloudHashMismatchFailsClosed() {
  const auto result = buildSavedCoverageGrid(artifact(), std::string(64U, 'c'), identity());

  require(!result.ok(), "mismatched source point cloud was accepted");
  require(result.reason.find("source map hash") != std::string::npos,
          "hash rejection reason is not actionable");
}

void testMapVersionMismatchFailsClosed() {
  auto changed = identity();
  changed.map_version = 8;
  const auto result = buildSavedCoverageGrid(artifact(), std::string(64U, 'a'), changed);

  require(!result.ok(), "mismatched saved-map version was accepted");
}

void testLiveRouteCannotUseTheSavedGridAdapter() {
  auto live = identity();
  live.live = true;
  live.map_id.clear();
  live.map_version = 0;
  live.artifact_hash.clear();
  const auto result = buildSavedCoverageGrid(artifact(), {}, live);

  require(!result.ok(), "live exploration accepted a saved coverage grid");
}

}  // namespace

int main() {
  try {
    testExactSavedMapIdentityConvertsTheStaticGrid();
    testPointCloudHashMismatchFailsClosed();
    testMapVersionMismatchFailsClosed();
    testLiveRouteCannotUseTheSavedGridAdapter();
    std::cout << "test_saved_coverage_grid: PASS\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "test_saved_coverage_grid: FAIL: " << error.what() << "\n";
    return 1;
  }
}
