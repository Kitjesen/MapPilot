#pragma once

#include "explore_contract.hpp"

namespace lingtu::explore {

struct TarePolicyConfig {
  int min_frontier_size = 3;
  double sensor_range_m = 5.0;
  double candidate_radius_m = 2.5;
  double min_goal_distance_m = 0.8;
  double novelty_radius_m = 1.0;
  int max_candidates = 16;
};

using TareCandidate = ExploreCandidate;
using TareDecision = ExploreDecision;

class TarePolicy final : public IExplorePlanner {
 public:
  explicit TarePolicy(TarePolicyConfig config = {});

  [[nodiscard]] const char* name() const override;
  [[nodiscard]] ExploreDecision plan(const ExploreInput& input) const override;

  [[nodiscard]] TareDecision select(
      const Grid2D& grid,
      const Pose2D& robot,
      const std::vector<Pose2D>& visited_goals = {}) const;

 private:
  TarePolicyConfig config_;
};

}  // namespace lingtu::explore
