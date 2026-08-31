#include "input/planner.hpp"

#include <algorithm>
#include <chrono>

#include "input/obstacle.hpp"
#include "safety/command.hpp"
#include "status/nav_status_writer.hpp"

namespace lingtu::nav::endpoint {

PlanView makePlanView(const PlanConfig &config, PlanData &data, double now_s,
                      TimingDiagnostics &timing, bool collision_authoritative) {
  PlanView out;
  const bool traversability_available = !data.traversability.values.empty();
  out.traversability_fresh =
      traversability_available && data.traversability_received_s > 0.0 &&
      (config.traversability_max_age_s <= 0.0 ||
       now_s - data.traversability_received_s <= config.traversability_max_age_s);
  out.terrain_fresh =
      !data.terrain.empty() && data.terrain_received_s > 0.0 &&
      (config.terrain_max_age_s <= 0.0 ||
       now_s - data.terrain_received_s <= config.terrain_max_age_s);
  out.terrain_ext_fresh =
      !data.terrain_ext.empty() && data.terrain_ext_received_s > 0.0 &&
      (config.terrain_max_age_s <= 0.0 ||
       now_s - data.terrain_ext_received_s <= config.terrain_max_age_s);

  data.planner_obstacles.clear();
  if (config.check_obstacle && !collision_authoritative) {
    const auto merge_start = std::chrono::steady_clock::now();
    const std::vector<float> &measured =
        config.use_terrain_cloud && out.terrain_fresh ? data.terrain : data.obstacles;
    out.terrain_selected = &measured == &data.terrain;
    const std::size_t measured_points = measured.size() / 4U;
    const std::size_t measured_limit =
        config.max_obstacle_points == 0
            ? measured_points
            : std::min(measured_points, config.max_obstacle_points);
    data.planner_obstacles.assign(
        measured.begin(),
        measured.begin() + static_cast<std::ptrdiff_t>(measured_limit * 4U));

    const std::size_t prediction_limit =
        config.max_obstacle_points == 0
            ? kMaxDynamicPredictionPoints
            : std::min(kMaxDynamicPredictionPoints,
                       config.max_obstacle_points - measured_limit);
    appendPredictedObstacleCloud(data.planner_obstacles, data.predicted_obstacles,
                                 prediction_limit);
    timing.obstacle_merge_ms +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - merge_start)
            .count();
  }
  out.obstacles = &data.planner_obstacles;

  out.traversability =
      config.check_obstacle && config.use_traversability && out.traversability_fresh
          ? data.traversability.view()
          : lingtu::nav::navigation::TraversabilityGridView{};
  return out;
}

}  // namespace lingtu::nav::endpoint
