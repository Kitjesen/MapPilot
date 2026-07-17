#include "lingtu/maps/c_api/grid_layers.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

int main() {
  const std::vector<float> xyz = {
      2.0f, 1.0f, 0.5f,
      2.0f, 1.0f, 1.5f,
  };
  LingtuMapsGridSpec spec{};
  std::vector<float> min_z(100, 0.0f);
  std::vector<float> max_z(100, 0.0f);
  std::vector<float> clearance(100, 0.0f);
  std::vector<uint8_t> valid(100, 0);

  assert(lingtu_maps_build_elevation_map(
      xyz.data(),
      2,
      0.0,
      0.0,
      1.0,
      5.0,
      -1.0,
      3.0,
      &spec,
      min_z.data(),
      max_z.data(),
      clearance.data(),
      valid.data(),
      100) == 0);
  assert(spec.rows == 10);
  assert(spec.cols == 10);
  assert(valid[static_cast<size_t>(6 * 10 + 7)] == 1);
  assert(min_z[static_cast<size_t>(6 * 10 + 7)] == 0.5f);
  assert(max_z[static_cast<size_t>(6 * 10 + 7)] == 1.5f);

  LingtuMapsGridSpec small{};
  small.rows = 5;
  small.cols = 5;
  small.resolution_m = 1.0;
  small.origin_x_m = 0.0;
  small.origin_y_m = 0.0;
  std::vector<float> occupancy(25, 0.0f);
  for (int row = 0; row < 5; ++row) {
    occupancy[static_cast<size_t>(row * 5)] = 100.0f;
  }
  std::vector<float> distance(25, 0.0f);
  std::vector<float> grad_x(25, 0.0f);
  std::vector<float> grad_y(25, 0.0f);
  assert(lingtu_maps_compute_esdf(
      occupancy.data(),
      &small,
      50.0f,
      distance.data(),
      grad_x.data(),
      grad_y.data(),
      25) == 0);
  assert(std::fabs(grad_x[static_cast<size_t>(2 * 5 + 2)]) > 0.5f);
  assert(std::fabs(grad_y[static_cast<size_t>(2 * 5 + 2)]) < 0.1f);
  assert(distance[static_cast<size_t>(2 * 5 + 2)] > 0.0f);
  assert(distance[static_cast<size_t>(2 * 5 + 0)] < 0.0f);

  LingtuMapsGridSpec risk_spec{};
  risk_spec.rows = 3;
  risk_spec.cols = 3;
  risk_spec.resolution_m = 1.0;
  risk_spec.origin_x_m = 0.0;
  risk_spec.origin_y_m = 0.0;
  std::vector<float> flat(9, 0.0f);
  std::vector<float> max_height(9, 0.0f);
  std::vector<float> clear(9, 0.0f);
  std::vector<uint8_t> all_valid(9, 1);
  max_height[4] = 0.4f;
  std::vector<float> risk(9, 0.0f);
  std::vector<float> slope(9, 0.0f);
  std::vector<float> step(9, 0.0f);
  std::vector<float> rough(9, 0.0f);
  LingtuMapsTerrainRiskParams risk_params{};
  risk_params.max_slope_deg = 30.0f;
  risk_params.soft_slope_start_deg = 3.0f;
  risk_params.critical_step_m = 0.25f;
  risk_params.roughness_critical_m = 0.1f;
  assert(lingtu_maps_compute_terrain_risk(
      flat.data(),
      max_height.data(),
      clear.data(),
      all_valid.data(),
      &risk_spec,
      &risk_params,
      risk.data(),
      slope.data(),
      step.data(),
      rough.data(),
      9) == 0);
  assert(risk[4] >= 99.0f);
  assert(step[4] >= 0.39f);

  std::vector<float> cost(9, 0.0f);
  cost[0] = 100.0f;
  std::vector<float> esdf(9, 2.0f);
  esdf[5] = 0.25f;
  std::vector<float> terrain(9, 0.0f);
  terrain[8] = 80.0f;
  std::vector<float> fused(9, 0.0f);
  LingtuMapsTraversabilityParams trav_params{};
  trav_params.lethal = 100.0f;
  trav_params.inscribed = 99.0f;
  trav_params.max_slope_deg = 35.0f;
  trav_params.soft_slope_start_deg = 3.0f;
  trav_params.safe_distance_m = 1.0f;
  trav_params.proximity_cap = 50.0f;
  assert(lingtu_maps_fuse_traversability_cost(
      cost.data(),
      slope.data(),
      esdf.data(),
      terrain.data(),
      &risk_spec,
      &trav_params,
      fused.data(),
      9) == 0);
  assert(fused[0] == 100.0f);
  assert(fused[8] == 80.0f);
  assert(fused[5] >= 37.0f);

  const std::vector<float> occ_xyz = {
      1.0f, 0.0f, 1.0f,
      1.0f, 0.0f, 1.2f,
  };
  LingtuMapsGridSpec occ_spec{};
  std::vector<int8_t> occ_grid(100, 0);
  std::vector<float> occ_cost(100, 0.0f);
  LingtuMapsOccupancyCounts counts{};
  assert(lingtu_maps_build_occupancy_grid(
      occ_xyz.data(),
      2,
      0.0,
      0.0,
      0.0,
      1.0,
      5.0,
      0.3,
      2.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0,
      0,
      1800,
      0.0,
      &occ_spec,
      occ_grid.data(),
      occ_cost.data(),
      &counts,
      100) == 0);
  assert(occ_spec.rows == 10);
  assert(occ_spec.cols == 10);
  assert(occ_grid[static_cast<size_t>(5 * 10 + 6)] == 100);
  assert(occ_cost[static_cast<size_t>(5 * 10 + 6)] == 100.0f);
  assert(counts.occupied == 1);

  LingtuMapsGridSpec src_spec{};
  src_spec.rows = 2;
  src_spec.cols = 2;
  src_spec.resolution_m = 1.0;
  src_spec.origin_x_m = 0.0;
  src_spec.origin_y_m = 0.0;
  std::vector<float> src_grid = {0.0f, 0.0f, 0.0f, 100.0f};
  std::vector<float> resampled(1, -1.0f);
  assert(lingtu_maps_resample_grid_bilinear(
      src_grid.data(),
      &src_spec,
      1,
      1,
      1.0,
      0.5,
      0.5,
      0.0f,
      resampled.data(),
      1) == 0);
  assert(resampled[0] > 24.0f && resampled[0] < 26.0f);

  return 0;
}
