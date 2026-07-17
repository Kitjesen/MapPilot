#include "safety_grid_probe.hpp"

#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

using lingtu::maps::layers::Grid2D;
using lingtu::maps::layers::makeGrid2D;

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void setCost(Grid2D& grid, double x, double y, float cost) {
  int row = -1;
  int col = -1;
  require(
      lingtu::nav::endpoint::safetyGridProbeCell(grid, x, y, row, col),
      "test cost coordinate must be in bounds");
  grid.data[static_cast<std::size_t>(grid.index(row, col))] = cost;
}

void markObserved(
    const Grid2D& grid,
    std::vector<std::uint8_t>& observed,
    double x,
    double y) {
  int row = -1;
  int col = -1;
  require(
      lingtu::nav::endpoint::safetyGridProbeCell(grid, x, y, row, col),
      "test observed coordinate must be in bounds");
  observed[static_cast<std::size_t>(grid.index(row, col))] = 1;
}

void testCellSnapsOnlyFloatingPointBoundaryNoise() {
  const Grid2D grid = makeGrid2D(20, 20, 0.2, -2.0, -2.0, 0.0F);
  int row = -1;
  int col = -1;

  require(
      lingtu::nav::endpoint::safetyGridProbeCell(
          grid,
          0.6,
          0.0,
          row,
          col),
      "exact conceptual boundary must be in bounds");
  require(col == 13, "floating-point boundary noise must not select the prior cell");

  require(
      lingtu::nav::endpoint::safetyGridProbeCell(
          grid,
          0.6 - 1e-10,
          0.0,
          row,
          col),
      "coordinate slightly below a boundary must be in bounds");
  require(col == 12, "coordinate slightly below a boundary must not be lifted");
}

void testProbeAttributesEachFusedCostSource() {
  Grid2D fused = makeGrid2D(20, 20, 0.2, -2.0, -2.0, 0.0F);
  Grid2D occupancy = makeGrid2D(20, 20, 0.2, -2.0, -2.0, 0.0F);
  Grid2D height = makeGrid2D(20, 20, 0.2, -2.0, -2.0, 0.0F);
  Grid2D surface = makeGrid2D(20, 20, 0.2, -2.0, -2.0, 0.0F);
  std::vector<std::uint8_t> observed(fused.data.size(), 0);

  for (const double distance : {0.0, 0.2, 0.6, 0.8, 1.0, 1.2}) {
    markObserved(fused, observed, distance, 0.0);
  }
  setCost(fused, 0.4, 0.0, 100.0F);
  setCost(fused, 0.6, 0.0, 100.0F);
  setCost(occupancy, 0.6, 0.0, 100.0F);
  setCost(fused, 0.8, 0.0, 40.0F);
  setCost(height, 0.8, 0.0, 40.0F);
  setCost(fused, 1.0, 0.0, 100.0F);
  setCost(surface, 1.0, 0.0, 100.0F);

  const lingtu::nav::endpoint::SafetyGridProbeLayers layers{
      &fused,
      &observed,
      &occupancy,
      &height,
      &surface,
  };
  const auto probe =
      lingtu::nav::endpoint::buildStraightForwardSafetyGridProbe(
          {0.0, 0.0, 0.0},
          layers,
          1.2,
          0.2,
          11,
          7,
          12.5,
          12.4);

  require(probe.grid_generation == 11, "grid generation must be retained");
  require(probe.terrain_generation == 7, "terrain generation must be retained");
  require(probe.samples.size() == 7, "body plus six forward samples are required");
  require(!probe.samples[0].used_by_teleop, "body sample is diagnostic only");
  require(probe.samples[1].used_by_teleop, "forward samples model teleop decisions");
  require(
      probe.samples[2].unknown_before_overlays &&
          probe.samples[2].fused_cost == 100.0F,
      "unknown fused hard cost must be visible");
  require(
      probe.samples[3].observed_before_overlays &&
          probe.samples[3].occupancy_cost == 100.0F,
      "occupancy hard cost must be visible independently");
  require(
      probe.samples[4].height_risk_cost == 40.0F &&
          probe.samples[4].surface_risk_cost == 0.0F,
      "height risk must remain separate from surface risk");
  require(
      probe.samples[5].surface_risk_cost == 100.0F &&
          probe.samples[5].height_risk_cost == 0.0F,
      "surface risk must remain separate from height risk");
  require(
      probe.samples[6].fused_cost == 0.0F,
      "clear forward sample must remain zero");
}

}  // namespace

int main() {
  testCellSnapsOnlyFloatingPointBoundaryNoise();
  testProbeAttributesEachFusedCostSource();
  std::cout << "test_safety_grid_probe passed\n";
  return 0;
}
