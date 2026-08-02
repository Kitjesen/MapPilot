#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "explore_contract.hpp"

namespace lingtu::nav::rolling {

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

struct SegmentSnapshot {
  lingtu::explore::Grid2D occupancy;
  TerrainCostGrid terrain_cost;
  lingtu::explore::ExploreMapIdentity identity;
  double stamp_s{0.0};
  double terrain_risk_stamp_s{0.0};
  bool terrain_risk_ready{false};

  [[nodiscard]] lingtu::explore::MapStamp mapStamp() const {
    return identity.stamp(stamp_s);
  }

  [[nodiscard]] bool samePayload(const SegmentSnapshot &other) const;
};

struct SegmentInput {
  SegmentSnapshot snapshot;
  lingtu::explore::Pose2D robot_pose;
  bool input_ready{false};
  double now_s{0.0};
};

// A caller supplies a target only. It cannot inject a raw segment or path.
// `minimum_revision` is a lower bound; a newer snapshot in the same map
// epoch is a valid execution source and is echoed in the decision.
struct SegmentRequest {
  lingtu::explore::DirectedTarget target;
  std::uint64_t minimum_revision{0U};
};

struct SegmentPolicy {
  static constexpr double kMinimumDistanceM = 0.1;
  static constexpr double kMaximumDistanceM = 100.0;
  static constexpr std::size_t kMinimumWaypoints = 2U;
  static constexpr std::size_t kMaximumWaypoints = 4096U;

  double max_distance_m{5.0};
  std::size_t max_waypoints{32U};

  [[nodiscard]] bool valid() const {
    return std::isfinite(max_distance_m) && max_distance_m >= kMinimumDistanceM &&
           max_distance_m <= kMaximumDistanceM && max_waypoints >= kMinimumWaypoints &&
           max_waypoints <= kMaximumWaypoints;
  }
};

struct MapInputPolicy {
  static constexpr std::size_t kMinimumGridCells = 1U;
  static constexpr std::size_t kMaximumGridCells = 1'048'576U;
  static constexpr double kMinimumMaxAgeS = 0.01;
  static constexpr double kMaximumMaxAgeS = 10.0;

  std::size_t max_grid_cells{262'144U};
  double max_age_s{0.35};
  double future_tolerance_s{0.25};
  double max_coordinate_m{1'000'000.0};

  [[nodiscard]] bool valid() const {
    return max_grid_cells >= kMinimumGridCells && max_grid_cells <= kMaximumGridCells &&
           std::isfinite(max_age_s) && max_age_s >= kMinimumMaxAgeS &&
           max_age_s <= kMaximumMaxAgeS && std::isfinite(future_tolerance_s) &&
           future_tolerance_s >= 0.0 && future_tolerance_s <= 5.0 &&
           std::isfinite(max_coordinate_m) && max_coordinate_m > 0.0 &&
           max_coordinate_m <= 10'000'000.0;
  }
};

struct RiskPolicy {
  float stop_threshold{50.0F};
  float resume_threshold{40.0F};
  float max_cost{100.0F};

  [[nodiscard]] bool valid() const {
    return std::isfinite(stop_threshold) && std::isfinite(resume_threshold) &&
           std::isfinite(max_cost) && max_cost > 0.0F && max_cost <= 100.0F &&
           resume_threshold >= 0.0F && stop_threshold >= resume_threshold &&
           stop_threshold <= max_cost;
  }
};

struct SegmentExecutorConfig {
  SegmentPolicy segment;
  MapInputPolicy map_input;
  RiskPolicy risk;

  [[nodiscard]] bool valid() const {
    return segment.valid() && map_input.valid() && risk.valid();
  }
};

enum class SegmentAction {
  Noop,
  Accepted,
  Rejected,
  Cancel,
};

struct SegmentDecision {
  SegmentAction action{SegmentAction::Noop};
  std::string reason;
  lingtu::explore::ExploreMapIdentity executed_map;
  std::uint64_t executed_revision{0U};
  std::vector<lingtu::explore::Pose2D> path;
};

// navd's DDS-free rolling-map planning seam. It owns validation, map-epoch
// tracking, A* search, safe-prefix selection, and active-path revalidation.
class SegmentExecutor {
 public:
  explicit SegmentExecutor(SegmentExecutorConfig config = {});

  [[nodiscard]] SegmentDecision plan(const SegmentInput &input, const SegmentRequest &request);

  // Revalidation is intentionally separate from planning: it never accepts a
  // raw replacement path, and cancels the active segment if its inputs no
  // longer satisfy the execution contract.
  [[nodiscard]] SegmentDecision revalidate(const SegmentInput &input);

  void reset() noexcept;
  [[nodiscard]] bool active() const noexcept;

 private:
  struct ActiveSegment {
    lingtu::explore::MapStamp stamp;
    lingtu::explore::ExploreMapIdentity identity;
    std::vector<lingtu::explore::Pose2D> path;
  };

  [[nodiscard]] SegmentDecision cancelActive(std::string reason);

  SegmentExecutorConfig config_;
  std::optional<ActiveSegment> active_;
  std::optional<SegmentSnapshot> last_seen_;
};

}  // namespace lingtu::nav::rolling
