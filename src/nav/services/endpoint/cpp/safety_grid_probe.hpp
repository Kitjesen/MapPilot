#pragma once

#include "lingtu/maps/layers/grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace lingtu::nav::endpoint {

struct SafetyGridProbePose {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct SafetyGridProbeLayers {
  const lingtu::maps::layers::Grid2D* fused{nullptr};
  const std::vector<std::uint8_t>* observed_before_overlays{nullptr};
  const lingtu::maps::layers::Grid2D* occupancy_source{nullptr};
  const lingtu::maps::layers::Grid2D* height_risk{nullptr};
  const lingtu::maps::layers::Grid2D* surface_risk{nullptr};
};

struct SafetyGridProbeSample {
  double distance_m{0.0};
  double map_x{0.0};
  double map_y{0.0};
  int row{-1};
  int col{-1};
  bool used_by_teleop{false};
  bool in_bounds{false};
  bool observed_before_overlays{false};
  bool unknown_before_overlays{true};
  float occupancy_cost{-1.0F};
  float height_risk_cost{-1.0F};
  float surface_risk_cost{-1.0F};
  float fused_cost{-1.0F};
};

struct SafetyGridForwardProbe {
  std::uint64_t grid_generation{0};
  std::uint64_t terrain_generation{0};
  double source_stamp_s{0.0};
  double terrain_source_stamp_s{0.0};
  double horizon_m{0.0};
  double step_m{0.0};
  SafetyGridProbePose pose{};
  std::vector<SafetyGridProbeSample> samples;
};

inline bool safetyGridProbeCell(
    const lingtu::maps::layers::Grid2D& grid,
    double x,
    double y,
    int& row,
    int& col) {
  if (grid.empty() || !std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(grid.resolution) || grid.resolution <= 0.0) {
    return false;
  }
  const auto cellIndex = [&](double coordinate, double origin) {
    const double normalized = (coordinate - origin) / grid.resolution;
    const double conceptual_boundary = std::nearbyint(normalized);
    const double boundary_noise = 4.0 *
        std::numeric_limits<double>::epsilon() *
        std::max(1.0, std::fabs(normalized));
    return static_cast<int>(std::floor(
        std::fabs(normalized - conceptual_boundary) <= boundary_noise
            ? conceptual_boundary
            : normalized));
  };
  col = cellIndex(x, grid.originX);
  row = cellIndex(y, grid.originY);
  return row >= 0 && row < grid.rows && col >= 0 && col < grid.cols;
}

inline float safetyGridProbeCost(
    const lingtu::maps::layers::Grid2D* grid,
    double x,
    double y) {
  if (grid == nullptr) {
    return -1.0F;
  }
  int row = -1;
  int col = -1;
  if (!safetyGridProbeCell(*grid, x, y, row, col)) {
    return -1.0F;
  }
  return grid->data[static_cast<std::size_t>(grid->index(row, col))];
}

inline SafetyGridForwardProbe buildStraightForwardSafetyGridProbe(
    const SafetyGridProbePose& pose,
    const SafetyGridProbeLayers& layers,
    double horizon_m,
    double minimum_step_m,
    std::uint64_t grid_generation,
    std::uint64_t terrain_generation,
    double source_stamp_s,
    double terrain_source_stamp_s) {
  SafetyGridForwardProbe probe;
  probe.grid_generation = grid_generation;
  probe.terrain_generation = terrain_generation;
  probe.source_stamp_s = source_stamp_s;
  probe.terrain_source_stamp_s = terrain_source_stamp_s;
  probe.pose = pose;
  if (layers.fused == nullptr || layers.fused->empty()) {
    return probe;
  }

  probe.horizon_m = std::max(0.0, horizon_m);
  probe.step_m = std::max(
      std::max(0.01, minimum_step_m),
      layers.fused->resolution);
  const int forward_samples = static_cast<int>(
      std::floor((probe.horizon_m + 1e-9) / probe.step_m));
  probe.samples.reserve(static_cast<std::size_t>(forward_samples + 1));
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  for (int index = 0; index <= forward_samples; ++index) {
    SafetyGridProbeSample sample;
    sample.distance_m = static_cast<double>(index) * probe.step_m;
    sample.map_x = pose.x + c * sample.distance_m;
    sample.map_y = pose.y + s * sample.distance_m;
    sample.used_by_teleop = index > 0;
    sample.in_bounds = safetyGridProbeCell(
        *layers.fused,
        sample.map_x,
        sample.map_y,
        sample.row,
        sample.col);
    if (sample.in_bounds) {
      const std::size_t flat_index = static_cast<std::size_t>(
          layers.fused->index(sample.row, sample.col));
      const bool observed_mask_valid =
          layers.observed_before_overlays != nullptr &&
          layers.observed_before_overlays->size() == layers.fused->data.size();
      sample.observed_before_overlays = observed_mask_valid &&
          (*layers.observed_before_overlays)[flat_index] != 0;
      sample.unknown_before_overlays = !sample.observed_before_overlays;
      sample.fused_cost = layers.fused->data[flat_index];
    }
    sample.occupancy_cost = safetyGridProbeCost(
        layers.occupancy_source,
        sample.map_x,
        sample.map_y);
    sample.height_risk_cost = safetyGridProbeCost(
        layers.height_risk,
        sample.map_x,
        sample.map_y);
    sample.surface_risk_cost = safetyGridProbeCost(
        layers.surface_risk,
        sample.map_x,
        sample.map_y);
    probe.samples.push_back(sample);
  }
  return probe;
}

}  // namespace lingtu::nav::endpoint
