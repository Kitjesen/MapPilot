#pragma once

#include <cstdint>
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

struct ExploreInput {
  Grid2D exploration_grid;
  Pose2D robot_pose;
  std::vector<Pose2D> visited_goals;
  double stamp_s = 0.0;
  std::string map_frame = "map";
};

struct ExploreCandidate {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double score = 0.0;
  double distance_m = 0.0;
  int frontier_size = 0;
  int covered_frontier_cells = 0;
};

struct ExploreDecision {
  bool has_goal = false;
  bool done = false;
  double goal_x = 0.0;
  double goal_y = 0.0;
  double goal_z = 0.0;
  std::string reason;
  std::vector<ExploreCandidate> candidates;
};

class IExplorePlanner {
 public:
  virtual ~IExplorePlanner() = default;

  [[nodiscard]] virtual const char* name() const = 0;
  [[nodiscard]] virtual ExploreDecision plan(const ExploreInput& input) const = 0;
};

}  // namespace lingtu::explore
