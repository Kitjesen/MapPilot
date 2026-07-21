#include "observed_free_cache.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

float costAt(const lingtu::maps::layers::Grid2D& grid, double x, double y) {
  int row = -1;
  int col = -1;
  require(
      lingtu::nav::endpoint::safetyGridCell(grid, x, y, row, col),
      "query must lie inside grid");
  return grid.data[static_cast<std::size_t>(grid.index(row, col))];
}

void testCoveragePersistsBrieflyAndExpires() {
  lingtu::nav::endpoint::ObservedFreeCache cache(0.2);
  cache.observeRay(0.0, 0.0, 1.0, 0.0, 10.0, 2.0);

  auto current = lingtu::nav::endpoint::makeUnknownSafetyGrid(
      20, 20, 0.2, -2.0, -2.0);
  require(cache.apply(current, 10.0, 0.6) > 0, "current ray must apply");
  require(costAt(current, 0.6, 0.0) == 0.0F, "ray cell must be observed free");

  auto retained = lingtu::nav::endpoint::makeUnknownSafetyGrid(
      20, 20, 0.2, -2.0, -2.0);
  cache.apply(retained, 10.5, 0.6);
  require(costAt(retained, 0.6, 0.0) == 0.0F, "coverage must bridge scan sparsity");

  auto expired = lingtu::nav::endpoint::makeUnknownSafetyGrid(
      20, 20, 0.2, -2.0, -2.0);
  cache.apply(expired, 10.7, 0.6);
  require(costAt(expired, 0.6, 0.0) == 100.0F, "expired coverage must return unknown");
}

void testClearStartsANewMapEpoch() {
  lingtu::nav::endpoint::ObservedFreeCache cache(0.2);
  cache.observeRay(0.0, 0.0, 1.0, 0.0, 10.0, 2.0);
  require(cache.size() > 0, "ray must populate cache");
  cache.clear();
  require(cache.size() == 0, "map epoch clear must discard old coverage");
}

void testRollingGridUsesTheCacheLattice() {
  constexpr double kResolution = 0.2;
  constexpr double kRadius = 2.0;
  constexpr double kRobotX = 0.259;
  constexpr double kRobotY = -0.004;

  lingtu::nav::endpoint::ObservedFreeCache cache(kResolution);
  cache.observeRay(kRobotX, kRobotY, 2.0, kRobotY, 10.0, 4.0);

  auto misaligned = lingtu::nav::endpoint::makeUnknownSafetyGrid(
      20,
      20,
      kResolution,
      kRobotX - kRadius,
      kRobotY - kRadius);
  cache.apply(misaligned, 10.0, 0.6);
  require(
      costAt(misaligned, kRobotX + 0.2, kRobotY) == 100.0F,
      "fixture must reproduce cache-to-rolling-grid aliasing");

  const double origin_x = lingtu::nav::endpoint::snappedSafetyGridOrigin(
      kRobotX,
      kRadius,
      kResolution);
  const double origin_y = lingtu::nav::endpoint::snappedSafetyGridOrigin(
      kRobotY,
      kRadius,
      kResolution);
  require(
      lingtu::nav::endpoint::safetyGridAlignmentResidual(
          origin_x,
          kResolution) < 1e-12,
      "x origin must share the observed-free cache lattice");
  require(
      lingtu::nav::endpoint::safetyGridAlignmentResidual(
          origin_y,
          kResolution) < 1e-12,
      "y origin must share the observed-free cache lattice");

  auto aligned = lingtu::nav::endpoint::makeUnknownSafetyGrid(
      20, 20, kResolution, origin_x, origin_y);
  cache.apply(aligned, 10.0, 0.6);
  for (const double distance : {0.2, 0.4, 0.6, 0.8, 1.0, 1.2}) {
    require(
        costAt(aligned, kRobotX + distance, kRobotY) == 0.0F,
        "aligned rolling grid must retain observed-free corridor cells");
  }
}

}  // namespace

int main() {
  testCoveragePersistsBrieflyAndExpires();
  testClearStartsANewMapEpoch();
  testRollingGridUsesTheCacheLattice();
  std::cout << "test_observed_free_cache passed\n";
  return 0;
}
