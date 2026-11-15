#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

#include "lingtu/maps/layers/ground_surface.hpp"

using namespace lingtu::maps::layers;

namespace {

constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

void AppendPoint(std::vector<float>& xyz, double x, double y, double z) {
  xyz.push_back(static_cast<float>(x));
  xyz.push_back(static_cast<float>(y));
  xyz.push_back(static_cast<float>(z));
}

void AppendDenseCell(
    std::vector<float>& xyz, const Grid2D& geometry, int row, int col,
    double a, double b, double c, double noise_m = 0.0) {
  constexpr double offsets[] = {-0.075, -0.025, 0.025, 0.075};
  const double center_x = geometry.originX + (col + 0.5) * geometry.resolution;
  const double center_y = geometry.originY + (row + 0.5) * geometry.resolution;
  int sample = 0;
  for (double dy : offsets) {
    for (double dx : offsets) {
      const double x = center_x + dx;
      const double y = center_y + dy;
      const double noise = sample++ % 2 == 0 ? noise_m : -noise_m;
      AppendPoint(xyz, x, y, a * x + b * y + c + noise);
    }
  }
}

void AppendDensePlane(
    std::vector<float>& xyz, const Grid2D& geometry,
    double a, double b, double c, double noise_m = 0.0) {
  for (int row = 0; row < geometry.rows; ++row) {
    for (int col = 0; col < geometry.cols; ++col) {
      AppendDenseCell(xyz, geometry, row, col, a, b, c, noise_m);
    }
  }
}

void TestDenseNoisyFloorProducesStableSurface() {
  const auto geometry = makeGrid2D(5, 5, 0.20, -0.50, -0.50, kNan);
  std::vector<float> xyz;
  AppendDensePlane(xyz, geometry, 0.0, 0.0, -0.30, 0.004);

  const auto surface = EstimateGroundSurface(xyz, geometry);
  const auto center = static_cast<std::size_t>(surface.height.index(2, 2));

  assert(std::isfinite(surface.height.data[center]));
  assert(std::abs(surface.height.data[center] + 0.30F) < 0.01F);
  assert(surface.support_count.data[center] >= 3.0F);
  assert(surface.roughness_m.data[center] < 0.01F);
  assert(std::isfinite(surface.variance_m2.data[center]));
  assert(surface.variance_m2.data[center] < 0.0001F);
}

void TestIsolatedLowGhostsDoNotPullFloorDown() {
  const auto geometry = makeGrid2D(3, 3, 0.20, -0.30, -0.30, kNan);
  std::vector<float> xyz;
  AppendDensePlane(xyz, geometry, 0.0, 0.0, -0.30);
  AppendPoint(xyz, -0.075, -0.075, -0.68);
  AppendPoint(xyz, 0.075, 0.075, -0.64);

  const auto surface = EstimateGroundSurface(xyz, geometry);
  const auto center = static_cast<std::size_t>(surface.height.index(1, 1));

  assert(std::isfinite(surface.height.data[center]));
  assert(std::abs(surface.height.data[center] + 0.30F) < 0.01F);
}

void TestDegenerateLowClusterFallsThroughToPlanarFloor() {
  const auto geometry = makeGrid2D(1, 1, 0.20, 0.0, 0.0, kNan);
  std::vector<float> xyz;
  for (double x : {0.025, 0.075, 0.125}) {
    AppendPoint(xyz, x, 0.025, -0.65);
  }
  for (double y : {0.025, 0.175}) {
    for (double x : {0.025, 0.175}) {
      AppendPoint(xyz, x, y, -0.30);
    }
  }

  const auto surface = EstimateGroundSurface(xyz, geometry);

  assert(std::isfinite(surface.height.data[0]));
  assert(std::abs(surface.height.data[0] + 0.30F) < 0.01F);
  assert(surface.support_count.data[0] == 4.0F);
}

void TestSlopedPlaneKeepsCorrectHeightAndLowDetrendedRoughness() {
  const auto geometry = makeGrid2D(5, 5, 0.20, -0.50, -0.50, kNan);
  std::vector<float> xyz;
  AppendDensePlane(xyz, geometry, 0.20, -0.10, -0.30, 0.002);

  const auto surface = EstimateGroundSurface(xyz, geometry);
  const auto center = static_cast<std::size_t>(surface.height.index(2, 2));
  const double expected_slope_deg = std::atan(std::hypot(0.20, -0.10)) * 180.0 / M_PI;

  assert(std::isfinite(surface.height.data[center]));
  assert(std::abs(surface.height.data[center] + 0.30F) < 0.01F);
  assert(std::abs(surface.gradient_x.data[center] - 0.20F) < 0.02F);
  assert(std::abs(surface.gradient_y.data[center] + 0.10F) < 0.02F);
  assert(std::abs(surface.slope_deg.data[center] - expected_slope_deg) < 1.0);
  assert(surface.roughness_m.data[center] < 0.01F);
}

void TestWithinColumnHeightSpreadContributesToRoughness() {
  const auto geometry = makeGrid2D(1, 1, 0.20, 0.0, 0.0, kNan);
  std::vector<float> xyz;
  constexpr double offsets[] = {0.025, 0.075, 0.125, 0.175};
  for (double y : offsets) {
    for (double x : offsets) {
      AppendPoint(xyz, x, y, -0.33);
      AppendPoint(xyz, x, y, -0.27);
    }
  }

  const auto surface = EstimateGroundSurface(xyz, geometry);

  assert(std::isfinite(surface.roughness_m.data[0]));
  assert(std::abs(surface.height.data[0] + 0.30F) < 0.01F);
  assert(std::abs(surface.roughness_m.data[0] - 0.03F) < 0.005F);
  assert(std::abs(surface.variance_m2.data[0] - 0.0009F) < 0.0002F);
}

void TestNeighborRefitCannotPublishExcessiveOwnResidual() {
  const auto geometry = makeGrid2D(3, 3, 0.20, -0.30, -0.30, kNan);
  GroundSurfaceConfig config;
  std::vector<float> xyz;
  constexpr double offsets[] = {-0.075, -0.025, 0.025, 0.075};
  for (int row = 0; row < geometry.rows; ++row) {
    for (int col = 0; col < geometry.cols; ++col) {
      const double center_x = geometry.originX + (col + 0.5) * geometry.resolution;
      const double center_y = geometry.originY + (row + 0.5) * geometry.resolution;
      const double mean_z = row == 1 && col == 1 ? -0.30 : -0.27;
      for (double dy : offsets) {
        for (double dx : offsets) {
          AppendPoint(xyz, center_x + dx, center_y + dy, mean_z - 0.03);
          AppendPoint(xyz, center_x + dx, center_y + dy, mean_z + 0.03);
        }
      }
    }
  }

  const auto surface = EstimateGroundSurface(xyz, geometry, config);
  const auto center = static_cast<std::size_t>(surface.height.index(1, 1));
  std::size_t valid_cells = 0;
  for (std::size_t cell = 0; cell < surface.height.data.size(); ++cell) {
    if (!std::isfinite(surface.height.data[cell])) continue;
    ++valid_cells;
    assert(std::isfinite(surface.roughness_m.data[cell]));
    assert(surface.roughness_m.data[cell] <= config.max_residual_m + 1e-6);
  }

  assert(valid_cells > 0U);
  assert(!std::isfinite(surface.height.data[center]));
}

void TestFewerThanThreeDistinctColumnsRemainUnknown() {
  const auto geometry = makeGrid2D(1, 1, 0.20, 0.0, 0.0, kNan);
  std::vector<float> sparse;
  AppendPoint(sparse, 0.025, 0.025, -0.30);
  AppendPoint(sparse, 0.075, 0.075, -0.30);

  const auto surface = EstimateGroundSurface(sparse, geometry);
  assert(!std::isfinite(surface.height.data[0]));
  assert(surface.support_count.data[0] == 0.0F);
}

void TestCollinearEvidenceRemainsUnknown() {
  const auto geometry = makeGrid2D(1, 1, 0.20, 0.0, 0.0, kNan);
  std::vector<float> collinear;
  for (double x : {0.025, 0.075, 0.125, 0.175}) {
    AppendPoint(collinear, x, 0.10, -0.30);
  }

  const auto surface = EstimateGroundSurface(collinear, geometry);
  assert(!std::isfinite(surface.height.data[0]));
  assert(surface.support_count.data[0] == 0.0F);
}

void TestMissingEvidenceRemainsUnknown() {
  const auto geometry = makeGrid2D(1, 1, 0.20, 0.0, 0.0, kNan);
  const auto surface = EstimateGroundSurface({}, geometry);

  assert(!std::isfinite(surface.height.data[0]));
  assert(surface.support_count.data[0] == 0.0F);
}

void TestTabletopReturnsPreserveTheFloorSurface() {
  const auto geometry = makeGrid2D(3, 3, 0.20, -0.30, -0.30, kNan);
  std::vector<float> xyz;
  AppendDensePlane(xyz, geometry, 0.0, 0.0, -0.30);
  constexpr double offsets[] = {-0.075, -0.025, 0.025, 0.075};
  for (double y : offsets) {
    for (double x : offsets) {
      AppendPoint(xyz, x, y, 0.15);
    }
  }

  const auto surface = EstimateGroundSurface(xyz, geometry);
  const auto center = static_cast<std::size_t>(surface.height.index(1, 1));

  assert(std::isfinite(surface.height.data[center]));
  assert(std::abs(surface.height.data[center] + 0.30F) < 0.01F);
  assert(surface.roughness_m.data[center] < 0.01F);
}

void TestStepHeightsAreNotAveragedAcrossTheBoundary() {
  const auto geometry = makeGrid2D(5, 6, 0.20, -0.60, -0.50, kNan);
  std::vector<float> xyz;
  for (int row = 0; row < geometry.rows; ++row) {
    for (int col = 0; col < geometry.cols; ++col) {
      AppendDenseCell(xyz, geometry, row, col, 0.0, 0.0, col < 3 ? -0.30 : -0.05);
    }
  }

  const auto surface = EstimateGroundSurface(xyz, geometry);
  const auto low = static_cast<std::size_t>(surface.height.index(2, 2));
  const auto high = static_cast<std::size_t>(surface.height.index(2, 3));

  assert(std::isfinite(surface.height.data[low]));
  assert(std::isfinite(surface.height.data[high]));
  assert(std::abs(surface.height.data[low] + 0.30F) < 0.03F);
  assert(std::abs(surface.height.data[high] + 0.05F) < 0.03F);
  assert(surface.height.data[high] - surface.height.data[low] > 0.20F);
}

}  // namespace

int main() {
  TestDenseNoisyFloorProducesStableSurface();
  TestIsolatedLowGhostsDoNotPullFloorDown();
  TestDegenerateLowClusterFallsThroughToPlanarFloor();
  TestSlopedPlaneKeepsCorrectHeightAndLowDetrendedRoughness();
  TestWithinColumnHeightSpreadContributesToRoughness();
  TestNeighborRefitCannotPublishExcessiveOwnResidual();
  TestFewerThanThreeDistinctColumnsRemainUnknown();
  TestCollinearEvidenceRemainsUnknown();
  TestMissingEvidenceRemainsUnknown();
  TestTabletopReturnsPreserveTheFloorSurface();
  TestStepHeightsAreNotAveragedAcrossTheBoundary();
  return 0;
}
