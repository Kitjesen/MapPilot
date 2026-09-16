#include "global_planner.h"

#include <iostream>
#include <memory>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace global_planner {

struct OctoPlannerGridQueryTest
{
  static void footprintSamplesStayInEdgeCells()
  {
    auto tree = std::make_shared<octomap::OcTree>(0.2);
    for (int x = -2; x <= 2; ++x)
      for (int y = -2; y <= 2; ++y)
        tree->updateNode(octomap::point3d((x + 0.5) * 0.2, (y + 0.5) * 0.2, -0.1), true);
    tree->updateNode(octomap::point3d(2.1, 2.1, 0.9), false);
    PlannerConfig config;
    config.robot_radius = 0.43;
    config.max_step_height = 0.0;
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(tree);
    const auto center = planner.worldToGrid(0.1, 0.1, 0.1);
    if (!planner.hasFootprintGroundSupport(center, 0.43, 2))
      throw std::runtime_error("support query extended outside the physical footprint edge cells");
    tree = std::make_shared<octomap::OcTree>(*tree);
    tree->deleteNode(octomap::point3d(0.1, 0.5, -0.1));
    planner.setOctomap(tree);
    if (planner.hasFootprintGroundSupport(center, 0.43, 2))
      throw std::runtime_error("missing support at a physical footprint edge was ignored");
  }
  static void supportMustBeTheFirstSurfaceBelowTheBody()
  {
    auto tree = std::make_shared<octomap::OcTree>(0.1);
    for (int x = -10; x <= 10; ++x)
      for (int y = -10; y <= 10; ++y)
        for (int z = 0; z <= 10; ++z)
          tree->updateNode(octomap::point3d((x + 0.5) * 0.1, (y + 0.5) * 0.1,
                                           (z + 0.5) * 0.1), z <= 2);
    PlannerConfig config;
    config.robot_radius = 0.2;
    config.support_height_m = 0.4;
    config.support_height_tolerance_m = 0.01;
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(tree);
    const auto tooLow = planner.worldToGrid(0.05, 0.05, 0.45);
    const auto standing = planner.worldToGrid(0.05, 0.05, 0.65);
    for (bool strict : {false, true}) {
      if (planner.hasGroundSupport(tooLow, strict, 1, 5) ||
          !planner.hasGroundSupport(standing, strict, 1, 5))
        throw std::runtime_error("support height used slab interior instead of its top surface");
    }
    if (planner.hasFootprintGroundSupport(tooLow, 0.2, 5) ||
        !planner.hasFootprintGroundSupport(standing, 0.2, 5))
      throw std::runtime_error("footprint and centre disagree about the first support surface");
  }

  static void compareWithOctomap(double resolution)
  {
    auto tree = std::make_shared<octomap::OcTree>(resolution);
    for (const int base : {-16, 8}) {
      for (int x = base; x < base + 8; ++x) {
        for (int y = base; y < base + 8; ++y) {
          for (int z = base; z < base + 8; ++z) {
            tree->updateNode(octomap::point3d(
                (x + 0.5) * resolution, (y + 0.5) * resolution,
                (z + 0.5) * resolution), true);
          }
        }
      }
    }
    tree->updateNode(octomap::point3d(0.5 * resolution, 0.5 * resolution,
                                     0.5 * resolution), true);
    tree->updateNode(octomap::point3d(2.5 * resolution, 2.5 * resolution,
                                     2.5 * resolution), false);
    tree->updateInnerOccupancy();
    tree->prune();
    OctoPlanner3D planner;
    planner.setOctomap(tree);
    std::unordered_set<GridIndex, GridIndexHash> reference_centers;
    bool coarse_leaf = false;
    for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it) {
      if (tree->isNodeOccupied(*it)) {
        coarse_leaf = coarse_leaf || it.getDepth() < tree->getTreeDepth();
        reference_centers.insert(planner.worldToGrid(it.getX(), it.getY(), it.getZ()));
      }
    }
    if (!coarse_leaf) throw std::runtime_error("mixed-depth fixture was not pruned");
    for (int x = -18; x <= 18; ++x) {
      for (int y = -18; y <= 18; ++y) {
        for (int z = -18; z <= 18; ++z) {
          const GridIndex index{x, y, z};
          const auto * node = tree->search(planner.gridToWorld(index));
          const bool occupied = node && tree->isNodeOccupied(node);
          if (planner.isOccupiedCell(index) != occupied ||
              planner.isOccupiedLeafCell(index) != (reference_centers.count(index) != 0)) {
            throw std::runtime_error("cached occupancy/leaf centre differs from OctoMap");
          }
        }
      }
    }
  }
};

}  // namespace global_planner

namespace {

global_planner::PointPose point(int x, int y, int z, double resolution)
{
  return {(x + 0.5) * resolution, (y + 0.5) * resolution, (z + 0.5) * resolution};
}

std::shared_ptr<octomap::OcTree> makeMap(double resolution, bool obstacles)
{
  auto tree = std::make_shared<octomap::OcTree>(resolution);
  const auto occupy = [&](int x, int y, int z) {
    const auto p = point(x, y, z, resolution);
    tree->updateNode(octomap::point3d(p.x, p.y, p.z), true);
  };
  occupy(-32, -32, -32);
  occupy(32, 32, 32);
  if (obstacles) {
    occupy(-16, -16, -16);
    occupy(16, 16, 16);
  }
  tree->updateInnerOccupancy();
  return tree;
}

void expectPoint(global_planner::OctoPlanner3D & planner,
                 global_planner::PointPose p, bool free, const char * reason)
{
  planner.makePlan(p, p);
  std::vector<global_planner::PointPose> result;
  planner.getPlannerResults(result);
  if (result.empty() == free) throw std::runtime_error(reason);
}

void checkResolution(double resolution)
{
  global_planner::PlannerConfig config;
  config.robot_radius = 2.1 * resolution;
  config.require_ground_support = false;
  config.snap_search_radius_cells = 0;
  config.enable_preblocked_costmap = false;
  config.obstacle_clearance_radius_cells = 0;
  global_planner::OctoPlanner3D planner;
  planner.setConfig(config);
  planner.setOctomap(makeMap(resolution, true));

  // Body samples cross both positive and negative cache-block boundaries.
  for (const int edge : {-16, 16}) {
    expectPoint(planner, point(edge - 2, edge, edge, resolution), false,
                "obstacle at body radius was missed across a block boundary");
    expectPoint(planner, point(edge - 2, edge - 2, edge, resolution), true,
                "hemisphere was enlarged to its bounding box");
    expectPoint(planner, point(edge, edge, edge - 2, resolution), false,
                "obstacle above body was missed");
    expectPoint(planner, point(edge, edge, edge + 1, resolution), true,
                "support below hemisphere was treated as body collision");
  }

  config.body_clearance_below_m = 1.1 * resolution;
  config.body_clearance_above_m = 0.5 * resolution;
  planner.setConfig(config);
  const auto above = point(16, 16, 17, resolution);
  expectPoint(planner, above, false, "new cylinder did not reject a below-body obstacle");
  planner.setOctomap(makeMap(resolution, false));
  expectPoint(planner, above, true, "map replacement retained an occupied block");

  global_planner::ExternalBlockedRegion blocked;
  blocked.center = above;
  blocked.radius_xy_m = resolution * 0.2;
  blocked.min_z = above.z - resolution * 0.1;
  blocked.max_z = above.z + resolution * 0.1;
  planner.setExternalPreblockedRegions({blocked});
  expectPoint(planner, above, false, "temporary region did not invalidate free body result");
  planner.setExternalPreblockedRegions({});
  expectPoint(planner, above, true, "removed temporary region retained a blocked body result");
}

}  // namespace

int main()
{
  try {
    checkResolution(0.1);
    checkResolution(0.2);
    global_planner::OctoPlannerGridQueryTest::compareWithOctomap(0.1);
    global_planner::OctoPlannerGridQueryTest::compareWithOctomap(0.2);
    global_planner::OctoPlannerGridQueryTest::supportMustBeTheFirstSurfaceBelowTheBody();
    global_planner::OctoPlannerGridQueryTest::footprintSamplesStayInEdgeCells();
    std::cout << "grid query boundaries, body envelopes and invalidation passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
