#pragma once

#include <vector>

#include "endpoint_time.hpp"
#include "motion/teleop_safety.hpp"
#include "nav_endpoint_config.hpp"
#include "nav_endpoint_messages.hpp"
#include "nav_loop.hpp"

namespace lingtu::nav::endpoint {

/// Freshness and obstacle inputs shared by teleop, autonomy, and status paths.
struct PlannerInputs {
  bool traversability_available{false};
  bool traversability_fresh{false};
  bool terrain_map_fresh{false};
  bool terrain_ext_fresh{false};
  lingtu::nav::plan::TraversabilityGridView traversability_view{};
  const std::vector<float> *planner_obstacles_ptr{nullptr};
};

/// Compute traversability/terrain freshness and build the merged obstacle
/// cloud.  This is the exact logic that was triplicated across teleop,
/// autonomy, and status-publish sections.
///
/// `planner_terrain_xyzh` is an output scratch buffer owned by the caller.
inline PlannerInputs
computePlannerInputs(const CliConfig &cfg, const ObstacleMergeConfig &obstacle_merge_config,
                     const TraversabilityGrid &traversability_grid,
                     double last_traversability_receive_s, const std::vector<float> &obstacle_xyzh,
                     const std::vector<float> &terrain_xyzh, double last_terrain_map_receive_s,
                     const std::vector<float> &terrain_ext_xyzh, double last_terrain_ext_receive_s,
                     std::vector<float> &planner_terrain_xyzh, TimingDiagnostics &timing) {
  PlannerInputs out;
  const double tick_now = steadySeconds();

  out.traversability_available = !traversability_grid.values.empty();
  out.traversability_fresh =
      out.traversability_available && (last_traversability_receive_s > 0.0) &&
      (cfg.traversability_max_age_s <= 0.0 ||
       tick_now - last_traversability_receive_s <= cfg.traversability_max_age_s);

  out.terrain_map_fresh = !terrain_xyzh.empty() && (last_terrain_map_receive_s > 0.0) &&
                          (cfg.terrain_map_max_age_s <= 0.0 ||
                           tick_now - last_terrain_map_receive_s <= cfg.terrain_map_max_age_s);

  out.terrain_ext_fresh = !terrain_ext_xyzh.empty() && (last_terrain_ext_receive_s > 0.0) &&
                          (cfg.terrain_map_max_age_s <= 0.0 ||
                           tick_now - last_terrain_ext_receive_s <= cfg.terrain_map_max_age_s);

  if (cfg.check_obstacle) {
    const auto merge_start = SteadyClock::now();
    buildPlannerObstacleCloud(planner_terrain_xyzh, obstacle_xyzh, terrain_xyzh,
                              out.terrain_map_fresh, terrain_ext_xyzh, out.terrain_ext_fresh,
                              cfg.max_obstacle_points, obstacle_merge_config);
    timing.obstacle_merge_ms += elapsedMs(merge_start);
    out.planner_obstacles_ptr = &planner_terrain_xyzh;
  }

  out.traversability_view =
      (cfg.check_obstacle && cfg.use_traversability_cost && out.traversability_fresh)
          ? traversability_grid.view()
          : lingtu::nav::plan::TraversabilityGridView{};

  return out;
}

}  // namespace lingtu::nav::endpoint
