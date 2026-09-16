#include "global_planner.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {

unsigned cell(const global_planner::OctoPlanner3D::PlanningSlice &slice, double x, double y) {
  const int col = static_cast<int>(std::floor((x - slice.origin.x) / slice.resolution));
  const int row = static_cast<int>(std::floor((y - slice.origin.y) / slice.resolution));
  return slice.cells.at(static_cast<std::size_t>(row) * slice.cols + col);
}

void expect(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

}  // namespace

int main() {
  try {
    auto tree = std::make_shared<octomap::OcTree>(0.2);
    for (int x = -10; x <= 10; ++x) {
      for (int y = -10; y <= 10; ++y) {
        const float wx = static_cast<float>((x + 0.5) * 0.2);
        const float wy = static_cast<float>((y + 0.5) * 0.2);
        // Missing lower-floor support does not invalidate the independent upper floor.
        if (!(x >= -8 && x <= -4 && y >= -8 && y <= -4))
          tree->updateNode(octomap::point3d(wx, wy, -0.1F), true);
        tree->updateNode(octomap::point3d(wx, wy, 0.1F), false);
        tree->updateNode(octomap::point3d(wx, wy, 1.9F), true);
        tree->updateNode(octomap::point3d(wx, wy, 2.1F), false);
      }
    }
    for (int y = -3; y <= 3; ++y)
      tree->updateNode(octomap::point3d(0.9F, static_cast<float>((y + 0.5) * 0.2), 0.1F), true);
    tree->updateNode(octomap::point3d(-0.3F, 1.1F, 0.3F), true);
    tree->updateInnerOccupancy();
    global_planner::PlannerConfig config;
    config.robot_radius = 0.2;
    config.ground_support_depth_cells = 1;
    config.strict_direct_ground_support = true;
    config.require_ground_support = true;
    config.max_step_height = 0.0;
    global_planner::OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(tree);
    const auto lower = planner.projectPlanningSlice(0.1);
    expect(lower.available, "flat floor projection unavailable");
    expect(lower.cells.size() == static_cast<std::size_t>(lower.rows * lower.cols),
           "projection omitted unchecked cells");
    expect(cell(lower, -0.3, 0.1) == 1U, "supported floor was not allowed");
    expect(cell(lower, 0.9, 0.1) == 2U, "body obstacle was not marked blocked");
    expect(cell(lower, -0.3, 1.1) == 2U, "overhead body collision was not marked blocked");
    expect(cell(lower, -1.3, -1.3) == 0U, "missing ground support was marked walkable");
    const auto upper = planner.projectPlanningSlice(2.1);
    expect(upper.available && cell(upper, -1.3, -1.3) == 1U,
           "upper-floor fixture did not provide supported floor");
    expect(cell(lower, -1.3, -1.3) == 0U && upper.origin.z > lower.origin.z + 1.5,
           "projection merged the upper floor into the lower floor");
    const auto limited = planner.projectPlanningSlice(0.1, 4U);
    expect(!limited.available && limited.cells.empty() && limited.rows == 0 && limited.cols == 0,
           "oversized grid was downsampled into apparently checked space");
    planner.setCancelCheck([] { return true; });
    const auto cancelled = planner.projectPlanningSlice(0.1);
    expect(!cancelled.available && cancelled.cells.empty(), "partial cancelled grid was published");
    std::cout << "planning slice floor, obstacle, missing support, floor separation and limits passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
