#pragma once

#include "explore_contract.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>

namespace lingtu::explore {

struct TarePolicyConfig {
  int min_frontier_size{3};
  double sensor_range_m{5.0};
  double candidate_radius_m{2.5};
  double min_goal_distance_m{0.8};
  double novelty_radius_m{1.0};
  int max_candidates{16};

  double coverage_resolution_m{0.5};
  double local_route_radius_m{12.0};
  double return_home_distance_m{0.75};
  double keypose_min_distance_m{0.75};
  double keypose_connect_distance_m{6.0};
  double gain_weight{1.0};
  double travel_weight{0.20};
  double momentum_weight{2.0};
  double revisit_weight{3.0};
  double max_plan_time_ms{100.0};
  int route_2opt_iterations{4};
  std::size_t max_grid_cells{1'000'000U};
  std::size_t max_frontier_cells{200'000U};
  std::size_t max_frontier_clusters{512U};
  std::size_t max_coverage_cells{500'000U};
  std::size_t max_keyposes{4096U};
  std::size_t max_keypose_edges{16384U};
  std::size_t max_keypose_neighbor_links{6U};
  std::size_t max_route_targets{32U};
  bool return_home_when_done{true};
};

using TareCandidate = ExploreCandidate;
using TareDecision = ExploreDecision;

class TarePolicy final : public IExplorePlanner {
 public:
  explicit TarePolicy(TarePolicyConfig config = {});
  ~TarePolicy();

  TarePolicy(const TarePolicy&) = delete;
  TarePolicy& operator=(const TarePolicy&) = delete;
  TarePolicy(TarePolicy&&) noexcept;
  TarePolicy& operator=(TarePolicy&&) noexcept;

  [[nodiscard]] const char* name() const override;
  [[nodiscard]] ExploreDecision plan(const ExploreInput& input) override;
  [[nodiscard]] ExploreDecision plan(
      const ExploreInput& input,
      const ExploreCancelCheck& cancel);

  [[nodiscard]] TareDecision select(
      const Grid2D& grid,
      const Pose2D& robot,
      const std::vector<Pose2D>& visited_goals = {});

  void reset();
  [[nodiscard]] ExploreDiagnostics diagnostics() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  TarePolicyConfig config_;
  std::uint64_t legacy_generation_{0U};
};

}  // namespace lingtu::explore
