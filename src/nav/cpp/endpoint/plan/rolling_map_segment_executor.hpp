#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "explore_contract.hpp"

namespace lingtu::nav::endpoint {

// A terrain-cost grid is deliberately separate from the trinary occupancy
// grid. The enclosing snapshot binds both grids to one map identity and one
// execution instant.
struct TerrainCostGrid {
  int width{0};
  int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<float> costs;
};

struct RollingMapSegmentSnapshot {
  lingtu::explore::Grid2D occupancy;
  TerrainCostGrid terrain_cost;
  lingtu::explore::ExploreMapIdentity identity;
  double stamp_s{0.0};
  double terrain_risk_stamp_s{0.0};
  bool terrain_risk_ready{false};
};

struct RollingMapSegmentInput {
  RollingMapSegmentSnapshot snapshot;
  lingtu::explore::Pose2D robot_pose;
  bool input_ready{false};
  double now_s{0.0};
};

// A caller supplies a target only. It cannot inject a raw segment or path.
// `requested_generation` is a lower bound; a newer snapshot in the same map
// epoch is a valid execution source and is echoed in the decision.
struct RollingMapSegmentRequest {
  lingtu::explore::DirectedTarget target;
  std::uint64_t requested_generation{0U};
};

struct RollingMapSegmentExecutorConfig {
  std::size_t max_grid_cells{262'144U};
  double max_segment_length_m{5.0};
  std::size_t max_waypoints{32U};
  float terrain_risk_threshold{50.0F};
  float max_terrain_cost{100.0F};
  double snapshot_max_age_s{0.35};
  double future_tolerance_s{0.25};
  double max_coordinate_m{1'000'000.0};
};

enum class RollingMapSegmentAction {
  Noop,
  Accepted,
  Rejected,
  Cancel,
};

struct RollingMapSegmentDecision {
  RollingMapSegmentAction action{RollingMapSegmentAction::Noop};
  std::string reason;
  lingtu::explore::ExploreMapIdentity executed_map;
  std::uint64_t executed_generation{0U};
  std::vector<lingtu::explore::Pose2D> path;
};

// navd's DDS-free rolling-map planning seam. It owns validation, map-epoch
// tracking, A* search, safe-prefix selection, and active-path revalidation.
class RollingMapSegmentExecutor {
 public:
  explicit RollingMapSegmentExecutor(RollingMapSegmentExecutorConfig config = {});

  [[nodiscard]] RollingMapSegmentDecision plan(const RollingMapSegmentInput &input,
                                               const RollingMapSegmentRequest &request);

  // Revalidation is intentionally separate from planning: it never accepts a
  // raw replacement path, and cancels the active segment if its inputs no
  // longer satisfy the execution contract.
  [[nodiscard]] RollingMapSegmentDecision revalidate(const RollingMapSegmentInput &input);

  void reset() noexcept;
  [[nodiscard]] bool active() const noexcept;

 private:
  struct ActiveSegment {
    lingtu::explore::ExploreMapIdentity identity;
    std::uint64_t generation{0U};
    std::vector<lingtu::explore::Pose2D> path;
  };

  [[nodiscard]] RollingMapSegmentDecision cancelActive(std::string reason);

  RollingMapSegmentExecutorConfig config_;
  std::optional<ActiveSegment> active_;
  std::optional<lingtu::explore::ExploreMapIdentity> last_seen_;
};

}  // namespace lingtu::nav::endpoint
