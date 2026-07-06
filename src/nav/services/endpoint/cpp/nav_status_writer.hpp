#pragma once

#include "nav_kernel/types.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lingtu::nav::endpoint {

struct StatusWriterConfig {
  int domain_id{0};
  double tick_hz{0.0};
  bool publish_cmd_vel{false};
  bool check_obstacle{true};
  bool use_traversability_cost{false};
  std::string path_library_dir;
  std::string map_path;
  std::string status_file;
};

struct PlanDiagnostics {
  bool seen{false};
  bool accepted{false};
  bool reached_goal{false};
  std::string reason{"no_goal_received"};
  std::size_t waypoints{0};
  double goal_error_m{-1.0};
  double elapsed_ms{0.0};
  nav_kernel::Vec3 start{};
  nav_kernel::Vec3 goal{};
};

struct LocalDiagnostics {
  bool seen{false};
  bool active{false};
  bool goal_reached{false};
  bool path_found{false};
  bool near_field_stop{false};
  std::string reason{"not_seen"};
  int slow_down{0};
  int recovery_state{0};
  std::size_t target_index{0};
  double target_distance_m{0.0};
  std::size_t local_path_points{0};
  nav_kernel::Vec3 target{};
  nav_kernel::Twist cmd_vel{};
};

void writeStatusSnapshot(
    const StatusWriterConfig& cfg,
    double stamp_s,
    bool has_odom,
    bool has_map_odom_tf,
    bool has_path,
    bool has_traversability,
    bool has_terrain_map,
    bool has_terrain_map_ext,
    std::uint64_t odom_count,
    std::uint64_t tf_count,
    std::uint64_t goal_count,
    std::uint64_t cancel_count,
    std::uint64_t map_clearing_count,
    std::uint64_t cloud_clearing_count,
    std::uint64_t instruction_count,
    std::uint64_t cloud_count,
    std::uint64_t terrain_map_count,
    std::uint64_t terrain_map_ext_count,
    std::uint64_t traversability_count,
    std::uint64_t path_count,
    std::uint64_t plan_fail_count,
    std::uint64_t output_count,
    std::uint64_t cmd_vel_count,
    std::size_t obstacle_points,
    const PlanDiagnostics& plan,
    const LocalDiagnostics& local,
    const std::vector<nav_kernel::Vec3>& global_path,
    const std::vector<nav_kernel::Vec3>& local_path,
    const std::string& last_instruction);

}  // namespace lingtu::nav::endpoint
