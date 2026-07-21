#include "grid_inflation.hpp"
#include "terrain_risk.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    std::fprintf(stderr, "test_grid_inflation failed: %s\n", message);
    std::exit(1);
  }
}

}  // namespace

int main() {
  using lingtu::nav::endpoint::cellCenterWithinInflationRadius;
  using lingtu::nav::endpoint::inflationSearchRadiusCells;
  using lingtu::nav::endpoint::terrainHeightRiskCost;
  using lingtu::nav::endpoint::terrainSlopeRiskCost;

  require(inflationSearchRadiusCells(0.45, 0.20) == 3, "search window must cover the radius");
  require(cellCenterWithinInflationRadius(0, 0, 0.20, 0.45), "source cell must remain occupied");
  require(cellCenterWithinInflationRadius(2, 0, 0.20, 0.45), "0.40m cell must inflate");
  require(!cellCenterWithinInflationRadius(3, 0, 0.20, 0.45), "0.60m cell must not inflate");
  require(cellCenterWithinInflationRadius(1, 2, 0.20, 0.45), "0.447m diagonal cell must inflate");
  require(!cellCenterWithinInflationRadius(2, 2, 0.20, 0.45), "0.566m diagonal cell must not inflate");

  require(
      terrainHeightRiskCost(0.04, 0.08, 0.20) == 0.0F,
      "minor ground variation below the soft height must remain free");
  require(
      terrainHeightRiskCost(0.08, 0.08, 0.20) == 40.0F,
      "the configured soft height must begin the slowdown band");
  require(
      std::abs(terrainHeightRiskCost(0.10, 0.08, 0.20) - 46.6667F) < 1e-3F,
      "a moderate step must produce a graded soft cost");
  require(
      terrainHeightRiskCost(0.20, 0.08, 0.20) == 100.0F,
      "a hard step must produce a stop cost");
  require(
      terrainHeightRiskCost(0.50, 0.08, 0.20) == 100.0F,
      "heights above the hard threshold must remain saturated");
  require(
      terrainHeightRiskCost(NAN, 0.08, 0.20) == 100.0F,
      "non-finite terrain evidence must fail closed");
  require(
      terrainSlopeRiskCost(20.0, 12.0, 28.0) == 60.0F,
      "a moderate slope must produce a graded soft cost");
  require(
      terrainSlopeRiskCost(28.0, 12.0, 28.0) == 100.0F,
      "the configured hard slope must produce a stop cost");

  std::puts("test_grid_inflation passed");
  return 0;
}
