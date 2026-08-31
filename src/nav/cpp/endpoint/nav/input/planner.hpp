#pragma once

#include <cstddef>
#include <vector>

#include "navigation/executor.hpp"

namespace lingtu::nav::endpoint {

struct TimingDiagnostics;
struct TraversabilityGrid;

struct PlanConfig {
  bool check_obstacle{false};
  bool use_traversability{false};
  double traversability_max_age_s{0.0};
  double terrain_max_age_s{0.0};
  std::size_t max_obstacle_points{0};
  bool use_terrain_cloud{false};
};

struct PlanData {
  const TraversabilityGrid &traversability;
  const double &traversability_received_s;
  const std::vector<float> &obstacles;
  const std::vector<float> &predicted_obstacles;
  const std::vector<float> &terrain;
  const double &terrain_received_s;
  const std::vector<float> &terrain_ext;
  const double &terrain_ext_received_s;
  std::vector<float> &planner_obstacles;
};

/// Borrowed planner inputs valid until the next makePlanView call.
struct PlanView {
  bool traversability_fresh{false};
  lingtu::nav::navigation::TraversabilityGridView traversability{};
  const std::vector<float> *obstacles{nullptr};
  bool terrain_fresh{false};
  bool terrain_ext_fresh{false};
  bool terrain_selected{false};
};

PlanView makePlanView(const PlanConfig &config, PlanData &data, double now_s,
                      TimingDiagnostics &timing, bool collision_authoritative = false);

}  // namespace lingtu::nav::endpoint
