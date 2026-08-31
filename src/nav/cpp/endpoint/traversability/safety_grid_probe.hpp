#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "lingtu/maps/layers/grid.hpp"
#include "traversability/observed_safety_grid.hpp"

namespace lingtu::nav::endpoint {

struct SafetyGridProbePose {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct SafetyGridProbeLayers {
  const lingtu::maps::layers::Grid2D *fused{nullptr};
  const std::vector<std::uint8_t> *observed_before_overlays{nullptr};
  const lingtu::maps::layers::Grid2D *occupancy_source{nullptr};
  const lingtu::maps::layers::Grid2D *height_risk{nullptr};
  const lingtu::maps::layers::Grid2D *surface_risk{nullptr};
};

struct SafetyGridProbeSample {
  double distance_m{0.0};
  double body_x{0.0};
  double body_y{0.0};
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

struct SafetyGridNearBodyProbe {
  double radius_m{0.0};
  float hard_cost{100.0F};
  std::size_t total_count{0};
  bool truncated{false};
  SafetyGridProbePose pose{};
  std::vector<SafetyGridProbeSample> samples;
};

inline float safetyGridProbeCost(const lingtu::maps::layers::Grid2D *grid, double x, double y) {
  if (grid == nullptr) {
    return -1.0F;
  }
  int row = -1;
  int col = -1;
  if (!safetyGridCell(*grid, x, y, row, col)) {
    return -1.0F;
  }
  return grid->data[static_cast<std::size_t>(grid->index(row, col))];
}

inline SafetyGridForwardProbe buildStraightForwardSafetyGridProbe(
    const SafetyGridProbePose &pose, const SafetyGridProbeLayers &layers, double horizon_m,
    double minimum_step_m, std::uint64_t grid_generation, std::uint64_t terrain_generation,
    double source_stamp_s, double terrain_source_stamp_s) {
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
  probe.step_m = std::max(std::max(0.01, minimum_step_m), layers.fused->resolution);
  const int forward_samples = static_cast<int>(std::floor((probe.horizon_m + 1e-9) / probe.step_m));
  probe.samples.reserve(static_cast<std::size_t>(forward_samples + 1));
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  for (int index = 0; index <= forward_samples; ++index) {
    SafetyGridProbeSample sample;
    sample.distance_m = static_cast<double>(index) * probe.step_m;
    sample.body_x = sample.distance_m;
    sample.body_y = 0.0;
    sample.map_x = pose.x + c * sample.distance_m;
    sample.map_y = pose.y + s * sample.distance_m;
    sample.used_by_teleop = index > 0;
    sample.in_bounds =
        safetyGridCell(*layers.fused, sample.map_x, sample.map_y, sample.row, sample.col);
    if (sample.in_bounds) {
      const std::size_t flat_index =
          static_cast<std::size_t>(layers.fused->index(sample.row, sample.col));
      const bool observed_mask_valid =
          layers.observed_before_overlays != nullptr &&
          layers.observed_before_overlays->size() == layers.fused->data.size();
      sample.observed_before_overlays =
          observed_mask_valid && (*layers.observed_before_overlays)[flat_index] != 0;
      sample.unknown_before_overlays = !sample.observed_before_overlays;
      sample.fused_cost = layers.fused->data[flat_index];
    }
    sample.occupancy_cost =
        safetyGridProbeCost(layers.occupancy_source, sample.map_x, sample.map_y);
    sample.height_risk_cost = safetyGridProbeCost(layers.height_risk, sample.map_x, sample.map_y);
    sample.surface_risk_cost = safetyGridProbeCost(layers.surface_risk, sample.map_x, sample.map_y);
    probe.samples.push_back(sample);
  }
  return probe;
}

inline SafetyGridNearBodyProbe buildNearBodyHardSafetyGridProbe(
    const SafetyGridProbePose &pose, const SafetyGridProbeLayers &layers, double radius_m,
    float hard_cost, std::size_t max_samples) {
  SafetyGridNearBodyProbe probe;
  probe.radius_m = std::max(0.0, radius_m);
  probe.hard_cost = std::clamp(hard_cost, 0.0F, 100.0F);
  probe.pose = pose;
  if (layers.fused == nullptr || layers.fused->empty() || probe.radius_m <= 0.0) {
    return probe;
  }

  const bool observed_mask_valid =
      layers.observed_before_overlays != nullptr &&
      layers.observed_before_overlays->size() == layers.fused->data.size();
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  std::vector<SafetyGridProbeSample> samples;
  for (int row = 0; row < layers.fused->rows; ++row) {
    for (int col = 0; col < layers.fused->cols; ++col) {
      const std::size_t flat_index =
          static_cast<std::size_t>(layers.fused->index(row, col));
      const float fused_cost = layers.fused->data[flat_index];
      if (!std::isfinite(fused_cost) || fused_cost < probe.hard_cost) {
        continue;
      }
      SafetyGridProbeSample sample;
      sample.row = row;
      sample.col = col;
      sample.in_bounds = true;
      sample.map_x =
          layers.fused->originX + (static_cast<double>(col) + 0.5) * layers.fused->resolution;
      sample.map_y =
          layers.fused->originY + (static_cast<double>(row) + 0.5) * layers.fused->resolution;
      const double dx = sample.map_x - pose.x;
      const double dy = sample.map_y - pose.y;
      sample.body_x = dx * c + dy * s;
      sample.body_y = -dx * s + dy * c;
      sample.distance_m = std::hypot(sample.body_x, sample.body_y);
      if (sample.distance_m > probe.radius_m + 1e-9) {
        continue;
      }
      sample.observed_before_overlays =
          observed_mask_valid && (*layers.observed_before_overlays)[flat_index] != 0;
      sample.unknown_before_overlays = !sample.observed_before_overlays;
      sample.occupancy_cost =
          safetyGridProbeCost(layers.occupancy_source, sample.map_x, sample.map_y);
      sample.height_risk_cost =
          safetyGridProbeCost(layers.height_risk, sample.map_x, sample.map_y);
      sample.surface_risk_cost =
          safetyGridProbeCost(layers.surface_risk, sample.map_x, sample.map_y);
      sample.fused_cost = fused_cost;
      samples.push_back(sample);
    }
  }
  std::sort(samples.begin(), samples.end(), [](const auto &lhs, const auto &rhs) {
    if (std::abs(lhs.distance_m - rhs.distance_m) > 1e-9) {
      return lhs.distance_m < rhs.distance_m;
    }
    if (std::abs(lhs.body_x - rhs.body_x) > 1e-9) {
      return lhs.body_x < rhs.body_x;
    }
    return lhs.body_y < rhs.body_y;
  });
  probe.total_count = samples.size();
  probe.truncated = samples.size() > max_samples;
  if (probe.truncated) {
    samples.resize(max_samples);
  }
  probe.samples = std::move(samples);
  return probe;
}

}  // namespace lingtu::nav::endpoint
