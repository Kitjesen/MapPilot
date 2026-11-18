#include "global_planner.h"
#include "mls_backend.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace global_planner;

namespace global_planner {
struct OctoPlannerGridQueryTest {
  static void setStart(OctoPlanner3D& planner, const PointPose& start, bool actual) {
    planner.start_point_ = start;
    planner.has_start_ = actual;
  }
};
}

namespace {
std::shared_ptr<octomap::OcTree> scene(bool wall = false, bool gap = false,
                                    bool stairs = false) {
  auto tree = std::make_shared<octomap::OcTree>(.1);
  for (int x = -20; x < 20; ++x)
    for (int y = -8; y < 8; ++y)
      for (int z = -1; z < 35; ++z) {
        const int floor = stairs ? std::clamp((x + 8) / 4, 0, 6) - 1 : -1;
        const bool occupied = (z == floor && !(gap && x == 0)) ||
                              (wall && x == 0 && z < 20);
        tree->updateNode((x+.5)*.1, (y+.5)*.1, (z+.5)*.1, occupied, true);
      }
  tree->updateInnerOccupancy();
  return tree;
}

PlannerConfig configuration() {
  PlannerConfig c;
  c.robot_radius = .32;
  c.body_clearance_below_m = .05;
  c.body_clearance_above_m = .35;
  c.support_height_m = .435;
  c.support_height_tolerance_m = .10;
  c.max_step_height = .45;
  c.max_slope = .70;
  c.ground_support = true;
  c.direct_support_only = true;
  c.surface_closing_radius_m = 0;
  return c;
}

void compare(const char* name, const std::shared_ptr<octomap::OcTree>& tree,
             const std::vector<PointPose>& route, bool expected,
             const std::vector<lingtu::maps::NoGoRegion>& no_go = {},
             bool actual_start = false, bool prepared_surface = false,
             PlannerConfig c = configuration()) {
  if (prepared_surface) {
    c.direct_support_only = false;
    c.surface_closing_radius_m = .20;
  }
  OctoPlanner3D planner;
  planner.setConfig(c);
  planner.setOctomap(tree);
  planner.setNoGoRegions(std::make_shared<const std::vector<lingtu::maps::NoGoRegion>>(no_go));
  OctoPlannerGridQueryTest::setStart(planner, actual_start ? route.front() : PointPose{99,99,99}, actual_start);
  const auto cpp = planner.validateRoute(route);

  std::vector<LtMlsCell> occupied;
  const double r = tree->getResolution();
  for (auto leaf = tree->begin_leafs(); leaf != tree->end_leafs(); ++leaf) {
    if (!tree->isNodeOccupied(*leaf)) continue;
    const int span = static_cast<int>(std::lround(leaf.getSize()/r));
    const int x0 = static_cast<int>(std::lround((leaf.getX()-.5*leaf.getSize())/r));
    const int y0 = static_cast<int>(std::lround((leaf.getY()-.5*leaf.getSize())/r));
    const int z0 = static_cast<int>(std::lround((leaf.getZ()-.5*leaf.getSize())/r));
    for (int x = x0; x < x0+span; ++x)
      for (int y = y0; y < y0+span; ++y)
        for (int z = z0; z < z0+span; ++z) occupied.push_back({x,y,z});
  }
  const LtMlsConfig surface{
      static_cast<int>(std::ceil((c.support_height_m + std::max(r, c.body_clearance_above_m)) / r)),
      static_cast<unsigned>(std::ceil(c.surface_closing_radius_m / r)),
      static_cast<float>(c.surface_closing_radius_m / r)};
  const LtMlsGraphConfig graph{1,5,0,0,0,0,.70F,4,2};
  LtMlsArtifact* raw = nullptr;
  if (lt_mls_artifact_create(occupied.data(), occupied.size(), nullptr, 0,
                            &surface, &graph, 1, &raw) != 0)
    throw std::runtime_error("cannot prepare Rust route checker");
  std::unique_ptr<LtMlsArtifact, decltype(&lt_mls_artifact_free)> handle(raw, lt_mls_artifact_free);
  LtMlsMetricConfig config{};
  config.resolution = r;
  config.robot_radius = c.robot_radius;
  config.below = c.body_clearance_below_m;
  config.above = c.body_clearance_above_m;
  config.support_height = c.support_height_m;
  config.support_tolerance = c.support_height_tolerance_m;
  config.max_step = c.max_step_height;
  config.max_slope = c.max_slope;
  tree->getMetricMin(config.min.x, config.min.y, config.min.z);
  tree->getMetricMax(config.max.x, config.max.y, config.max.z);
  config.support_depth_cells = c.support_depth_cells;
  config.support_patch_radius_cells = c.support_patch_radius_cells;
  config.support_patch_min_samples = c.support_patch_min_samples;
  config.ground_support = c.ground_support;
  config.direct_support_only = c.direct_support_only;
  config.actual_start = actual_start;
  config.body_offset = c.body_offset_m;
  std::vector<LtMlsPose> points;
  for (const auto& p : route)
    points.push_back({{p.x,p.y,p.z},p.yaw.value_or(0.0),static_cast<std::uint8_t>(p.yaw.has_value())});
  std::vector<LtMlsMetricRegion> regions;
  for (const auto& region : no_go)
    regions.push_back({{region.min[0],region.min[1],region.min[2]},
                       {region.max[0],region.max[1],region.max[2]}});
  LtMlsRouteCheckResult rust{};
  LtMlsGeometry geometry{config,regions.data(),regions.size(),nullptr,0,nullptr,nullptr};
  const int status = lt_mls_artifact_check_metric_poses(raw, points.data(), points.size(),
      &geometry, &rust);
  std::cout << name << " cpp=" << cpp.valid << " rust=" << unsigned(rust.valid)
            << " reason=" << rust.failure_kind << " segment=" << rust.failed_segment << '\n';
  if (status != 0 || cpp.valid != expected || bool(rust.valid) != cpp.valid)
    throw std::runtime_error(std::string("metric route disagreement: ") + name);
  if (!expected && cpp.failed_segment != static_cast<std::size_t>(rust.failed_segment))
    throw std::runtime_error(std::string("failure segment disagreement: ") + name);
}
}

int main(int argc, char** argv) try {
  if (argc == 3) {
    auto tree = std::make_shared<octomap::OcTree>(.2);
    std::ifstream header(argv[1]);
    std::string first;
    std::getline(header, first);
    if (first == "# Octomap OcTree binary file") {
      if (!tree->readBinary(argv[1])) throw std::runtime_error("cannot read binary map");
    } else {
      auto* raw = dynamic_cast<octomap::OcTree*>(octomap::AbstractOcTree::read(argv[1]));
      if (!raw) throw std::runtime_error("cannot read map");
      tree.reset(raw);
    }
    std::ifstream input(argv[2]);
    std::vector<PointPose> route;
    PointPose point;
    while (input >> point.x >> point.y >> point.z) route.push_back(point);
    if (route.size() < 2) throw std::runtime_error("route needs two points");
    compare("planned_route", tree, route, true, {}, true, true);
    return 0;
  }
  const auto floor = scene();
  const std::vector<PointPose> flat{{-.85,.05,.385},{.85,.05,.385}};
  compare("flat", floor, flat, true);
  compare("wall", scene(true), flat, false);
  compare("gap", scene(false,true), flat, false);
  compare("height", floor, {{-.85,.05,.355},{-.78,.05,.405}}, true);
  compare("slope", floor, {{-.85,.05,.385},{-.75,.05,.635}}, false);
  compare("step", floor, {{-.85,.05,.385},{.85,.05,.985}}, false);
  compare("no_go", floor, flat, false, {{"test",{-.1,-.1,.2},{.1,.2,.6}}});
  compare("upper_no_go", floor, flat, true, {{"upper",{-.1,-.1,2},{.1,.2,3}}});
  compare("stairs", scene(false,false,true), {{-1.15,.05,.385},{-.55,.05,.385},
      {-.15,.05,.485},{.25,.05,.585},{.65,.05,.685},{1.05,.05,.785}}, true);
  compare("actual_start", floor, {{-.85,.05,.485},{-.45,.05,.385}}, true, {}, true);
  auto departure = std::make_shared<octomap::OcTree>(.1);
  departure->updateNode(-1,-1,-1,false);
  departure->updateNode(3,1,2,false);
  for (int x=3;x<25;++x) for (int y=-8;y<8;++y)
    departure->updateNode((x+.5)*.1,(y+.5)*.1,-.05,true,true);
  departure->updateInnerOccupancy();
  auto body = configuration();
  body.body_offset_m=.23;
  const std::vector<PointPose> entry{{.05,.05,.435,0.0},{1.25,.05,.435}};
  compare("departure",departure,entry,true,{},true,false,body);
  compare("departure_no_go",departure,entry,false,
      {{"blocked",{.15,-.1,.2},{.3,.2,.8}}},true,false,body);
  for (int x=0;x<3;++x) for (int y=-8;y<8;++y)
    departure->updateNode((x+.5)*.1,(y+.5)*.1,-.75,true,true);
  departure->updateInnerOccupancy();
  compare("measured_drop",departure,entry,false,{},true,false,body);
  auto corner = scene();
  corner->updateNode(.35,.35,.45,true,true);
  corner->updateInnerOccupancy();
  auto dual=configuration();
  dual.robot_radius=.05;
  dual.body_offset_m=.4;
  const std::vector<PointPose> into{{-.95,.05,.385},{.05,.05,.385}};
  const std::vector<PointPose> away{{.05,.05,.385},{.05,.65,.385}};
  compare("dual_in",corner,into,true,{},false,false,dual);
  compare("dual_out",corner,away,true,{},false,false,dual);
  // Vertices and the start yaw are not pivots; the terminal yaw still is.
  compare("dual_vertex",corner,{into.front(),into.back(),away.back()},true,{},false,false,dual);
  auto start_turn=away;
  start_turn.front().yaw=0.0;
  compare("start_yaw",corner,start_turn,true,{},true,false,dual);
  auto goal_turn=into;
  goal_turn.back().yaw=1.5707963267948966;
  compare("goal_yaw",corner,goal_turn,false,{},false,false,dual);
  compare("upper_turn_region",floor,{into.front(),into.back(),away.back()},true,
      {{"upper",{.3,.3,2},{.4,.4,3}}},false,false,dual);
  return 0;
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 1;
}
