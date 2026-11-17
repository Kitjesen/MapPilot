#include "octoplanner3d_core.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace r = octoplanner3d::runtime;

namespace {

void require(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}

r::PlannerOptions officialOptions() {
  r::PlannerOptions options;
  options.robot_radius = 0.4;
  options.body_clearance_below_m = 0.25;
  options.body_clearance_above_m = 0.35;
  options.body_offset_m = 0.25;
  options.max_iterations = 300000;
  options.snap_search_radius_cells = 24;
  options.ground_support = true;
  options.direct_support_only = true;
  options.support_depth_cells = 2;
  options.support_height_m = 0.45;
  options.support_height_tolerance_m = 0.11;
  options.support_patch_radius_cells = 0;
  options.support_patch_min_samples = 0;
  options.enable_preblocked_costmap = false;
  options.lowest_traversable_only = true;
  options.floor_change_penalty = 0.0;
  options.max_step_height = 0.45;
  options.max_slope = 0.0;
  options.obstacle_clearance_radius_cells = 0;
  options.obstacle_clearance_weight = 0.0;
  options.terminal_goal_tolerance_m = 0.5;
  options.terminal_goal_xy_tolerance_m = 0.6;
  options.terminal_goal_z_tolerance_m = 0.4;
  return options;
}

}  // namespace

int main(int argc, char **argv) try {
  if (argc != 2) {
    throw std::runtime_error("usage: octoplanner3d_official_park_mls_smoke <octomap.ot>");
  }
  const std::filesystem::path map = argv[1];
  require(std::filesystem::is_regular_file(map), "official park octomap is missing");
  const auto cache = std::filesystem::temp_directory_path() /
      ("lingtu-official-park-mls-" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::remove_all(cache);
  const lingtu::nav::plan::MapIdentity identity{"industrial-park-scan", 1, "map"};
  r::PlanRequest request;
  request.start = {3.0, 4.0, 0.44};
  request.goal = {56.0, 32.0, 0.3};
  request.options = officialOptions();

  r::PlanResult first;
  {
    r::PlannerSession session(cache);
    session.prepare(map, identity, request.options);
    require(session.prepareCount() == 1, "official park map was not prepared");
    require(session.cacheLoadCount() == 0, "cold prepare loaded a cache");
    bool saved = false;
    for (const auto &entry : std::filesystem::directory_iterator(cache)) {
      if (entry.is_regular_file() && entry.path().extension() == ".bin") saved = true;
    }
    require(saved, "prepared Rust MLS artifact was not saved");
    first = session.run(map, identity, request);
  }
  require(first.used_rust_graph && !first.rust_fallback, "official park route did not use Rust MLS");
  require(first.ok && first.reached_goal && !first.path.empty(),
          "official park Rust MLS route is empty");
  require(std::abs(first.path.front().x - request.start.x) < 1e-6 &&
              std::abs(first.path.front().y - request.start.y) < 1e-6 &&
              std::abs(first.path.front().z - request.start.z) < 1e-6,
          "official park route moved the measured start");
  require(first.goal_xy_error_m <= request.options.terminal_goal_xy_tolerance_m &&
              first.goal_z_error_m <= request.options.terminal_goal_z_tolerance_m,
          "official park route missed the goal tolerance");
  {
    r::PlannerSession restored(cache);
    const auto again = restored.run(map, identity, request);
    require(restored.cacheLoadCount() == 1, "saved artifact was not restored");
    require(again.used_rust_graph && again.ok && again.path.size() == first.path.size(),
            "restored artifact changed the Rust route");
  }
  std::filesystem::remove_all(cache);
  std::cout << "Official park MLS passed: " << first.path.size()
            << " poses, goal_xy=" << first.goal_xy_error_m
            << " goal_z=" << first.goal_z_error_m << "\n";
  return 0;
} catch (const std::exception &error) {
  std::cerr << error.what() << '\n';
  return 1;
}
