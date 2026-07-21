#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace lingtu::explore {

constexpr std::int8_t kFree = 0;
constexpr std::int8_t kOccupied = 100;
constexpr std::int8_t kUnknown = -1;

struct Grid2D {
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<std::int8_t> cells;

  [[nodiscard]] bool valid() const {
    return width > 0 && height > 0 && resolution > 0.0 &&
           cells.size() == static_cast<std::size_t>(width * height);
  }

  [[nodiscard]] int index(int row, int col) const {
    return row * width + col;
  }

  [[nodiscard]] bool inBounds(int row, int col) const {
    return row >= 0 && row < height && col >= 0 && col < width;
  }

  [[nodiscard]] std::int8_t at(int row, int col) const {
    return cells[static_cast<std::size_t>(index(row, col))];
  }
};

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct ExploreMapIdentity {
  std::string frame_id{"map"};
  std::string session_id;
  std::string map_id;
  std::int64_t map_version{0};
  std::string artifact_hash;
  std::uint64_t reset_epoch{0U};
  std::uint64_t generation{0U};
  bool live{true};

  [[nodiscard]] bool valid() const {
    if (frame_id.empty() || reset_epoch == 0U || generation == 0U) {
      return false;
    }
    if (live) {
      return !session_id.empty();
    }
    return !map_id.empty() && map_version > 0 && !artifact_hash.empty();
  }

  [[nodiscard]] bool sameSource(const ExploreMapIdentity& other) const {
    return frame_id == other.frame_id && session_id == other.session_id &&
           map_id == other.map_id && map_version == other.map_version &&
           artifact_hash == other.artifact_hash && live == other.live;
  }
};

struct ExploreInput {
  Grid2D exploration_grid;
  Pose2D robot_pose;
  std::vector<Pose2D> visited_goals;
  double stamp_s = 0.0;
  std::string map_frame = "map";
  ExploreMapIdentity map;
};

struct ExploreCandidate {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double score = 0.0;
  double distance_m = 0.0;
  int frontier_size = 0;
  int covered_frontier_cells = 0;
  double route_cost_m = 0.0;
  double revisit_penalty = 0.0;
  std::uint64_t cluster_id = 0U;
};

struct ExploreDiagnostics {
  std::string phase{"idle"};
  std::string frame_id;
  std::string session_id;
  std::string map_id;
  std::int64_t map_version{0};
  std::uint64_t reset_epoch{0U};
  std::uint64_t generation{0U};
  std::uint64_t accepted_generation{0U};
  std::size_t reachable_free_cells{0U};
  std::size_t frontier_cells{0U};
  std::size_t frontier_clusters{0U};
  std::size_t keypose_nodes{0U};
  std::size_t keypose_edges{0U};
  std::size_t covered_cells{0U};
  std::size_t route_targets{0U};
  std::size_t reset_count{0U};
  double route_length_m{0.0};
  double planning_time_ms{0.0};
  bool state_committed{false};
};

struct ExploreDecision {
  bool has_goal = false;
  bool done = false;
  double goal_x = 0.0;
  double goal_y = 0.0;
  double goal_z = 0.0;
  std::string reason;
  std::vector<ExploreCandidate> candidates;
  std::vector<Pose2D> route;
  ExploreDiagnostics diagnostics;
};

using ExploreCancelCheck = std::function<bool()>;

class IExplorePlanner {
 public:
  virtual ~IExplorePlanner() = default;

  [[nodiscard]] virtual const char* name() const = 0;
  [[nodiscard]] virtual ExploreDecision plan(const ExploreInput& input) = 0;
};

}  // namespace lingtu::explore
