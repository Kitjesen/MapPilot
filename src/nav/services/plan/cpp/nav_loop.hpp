#pragma once

#include "local_planner.hpp"
#include "nav_kernel/path_follower_core.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace lingtu::nav::plan {

struct NavLoopConfig {
  std::string path_library_dir;
  double corridor_lookahead_m{3.0};
  double waypoint_reached_m{0.6};
  double goal_reached_m{0.35};
  double max_speed{0.4};
  double slow_rate_1{0.25};
  double slow_rate_2{0.5};
  double slow_rate_3{0.75};
  nav_kernel::LocalPlannerParams local_planner{};
  nav_kernel::PathFollowerParams path_follower{};
};

struct NavLoopOutput {
  bool active{false};
  bool goal_reached{false};
  bool path_found{false};
  bool near_field_stop{false};
  int slow_down{0};
  int recovery_state{0};
  std::size_t target_index{0};
  nav_kernel::Vec3 target{};
  std::vector<nav_kernel::Vec3> local_path_body;
  std::vector<nav_kernel::Vec3> local_path_map;
  nav_kernel::Twist cmd_vel{};
};

struct TraversabilityGridView {
  const float* values{nullptr};
  int rows{0};
  int cols{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};

  bool valid() const {
    return values != nullptr && rows > 0 && cols > 0 && resolution > 0.0;
  }
};

class NavLoop {
 public:
  explicit NavLoop(NavLoopConfig config = {});

  bool configure();
  bool configured() const;

  void setGlobalPath(const std::vector<nav_kernel::Vec3>& path);
  void clearGlobalPath();

  NavLoopOutput tick(
      const nav_kernel::Pose& odom_map_body,
      const float* obstacle_xyzh,
      int obstacle_count,
      double timestamp_s,
      TraversabilityGridView traversability = {});

 private:
  std::size_t selectTargetIndex(const nav_kernel::Pose& odom_map_body);
  std::vector<nav_kernel::Vec3> bodyPathToMap(
      const nav_kernel::Pose& odom_map_body,
      const std::vector<nav_kernel::Vec3>& body_path) const;
  double slowFactor(int slow_down) const;
  bool atGoal(const nav_kernel::Pose& odom_map_body) const;

  NavLoopConfig config_;
  nav_kernel::LocalPlannerCore local_planner_;
  nav_kernel::PathFollowerState follower_state_{};
  std::vector<nav_kernel::Vec3> global_path_;
  std::size_t cursor_{0};
  bool configured_{false};
};

}  // namespace lingtu::nav::plan
