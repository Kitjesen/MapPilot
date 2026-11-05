#pragma once

#include <lingtu/settings.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>

#include "explore_contract.hpp"

namespace lingtu::explore {

struct TarePolicyConfig {
  int min_frontier_size{lingtu::settings::explore::explore::min_frontier_size};
  double sensor_range_m{lingtu::settings::explore::explore::sensor_range_m};
  double sensor_horizontal_fov_rad{lingtu::settings::explore::explore::sensor_horizontal_fov_rad};
  double sensor_yaw_offset_rad{lingtu::settings::explore::explore::sensor_yaw_offset_rad};
  double candidate_radius_m{lingtu::settings::explore::explore::candidate_radius_m};
  double min_goal_distance_m{lingtu::settings::explore::explore::min_goal_distance_m};
  double novelty_radius_m{lingtu::settings::explore::explore::novelty_radius_m};
  int max_candidates{lingtu::settings::explore::explore::max_candidates};

  double coverage_resolution_m{lingtu::settings::explore::explore::coverage_resolution_m};
  double local_route_radius_m{lingtu::settings::explore::explore::local_route_radius_m};
  double return_home_distance_m{lingtu::settings::explore::explore::return_home_distance_m};
  double keypose_min_distance_m{lingtu::settings::explore::explore::keypose_min_distance_m};
  double keypose_connect_distance_m{lingtu::settings::explore::explore::keypose_connect_distance_m};
  double gain_weight{lingtu::settings::explore::explore::gain_weight};
  double travel_weight{lingtu::settings::explore::explore::travel_weight};
  double momentum_weight{lingtu::settings::explore::explore::momentum_weight};
  double directed_progress_weight{lingtu::settings::explore::explore::directed_progress_weight};
  double revisit_weight{lingtu::settings::explore::explore::revisit_weight};
  double max_plan_time_ms{lingtu::settings::explore::explore::max_plan_time_ms};
  int route_2opt_iterations{lingtu::settings::explore::explore::route_2opt_iterations};
  std::size_t max_grid_cells{lingtu::settings::explore::explore::max_grid_cells};
  std::size_t max_frontier_cells{lingtu::settings::explore::explore::max_frontier_cells};
  std::size_t max_frontier_clusters{lingtu::settings::explore::explore::max_frontier_clusters};
  std::size_t max_coverage_cells{lingtu::settings::explore::explore::max_coverage_cells};
  std::size_t max_known_map_candidate_samples{lingtu::settings::explore::explore::max_known_map_candidate_samples};
  std::size_t max_known_map_gain_checks_per_candidate{lingtu::settings::explore::explore::max_known_map_gain_checks_per_candidate};
  std::size_t max_known_map_visibility_checks{lingtu::settings::explore::explore::max_known_map_visibility_checks};
  std::size_t max_keyposes{lingtu::settings::explore::explore::max_keyposes};
  std::size_t max_keypose_edges{lingtu::settings::explore::explore::max_keypose_edges};
  std::size_t max_keypose_neighbor_links{lingtu::settings::explore::explore::max_keypose_neighbor_links};
  std::size_t max_route_targets{lingtu::settings::explore::explore::max_route_targets};
  bool known_map_require_line_of_sight{lingtu::settings::explore::explore::known_map_require_line_of_sight};
  bool return_home_when_done{lingtu::settings::explore::explore::return_home};
};

using TareCandidate = ExploreCandidate;
using TareDecision = ExploreDecision;

class TarePolicy final : public IExplorePlanner {
 public:
  explicit TarePolicy(TarePolicyConfig config = {});
  ~TarePolicy();

  TarePolicy(const TarePolicy &) = delete;
  TarePolicy &operator=(const TarePolicy &) = delete;
  TarePolicy(TarePolicy &&) noexcept;
  TarePolicy &operator=(TarePolicy &&) noexcept;

  [[nodiscard]] const char *name() const override;
  [[nodiscard]] ExploreDecision plan(const ExploreInput &input) override;
  [[nodiscard]] ExploreDecision plan(const ExploreInput &input, const ExploreCancelCheck &cancel);

  [[nodiscard]] TareDecision select(const Grid2D &grid, const Pose2D &robot,
                                    const std::vector<Pose2D> &visited_goals = {});

  void reset();
  [[nodiscard]] ExploreDiagnostics diagnostics() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  TarePolicyConfig config_;
  std::uint64_t legacy_generation_{0U};
};

}  // namespace lingtu::explore
