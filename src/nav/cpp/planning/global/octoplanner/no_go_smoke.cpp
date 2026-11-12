#include "octoplanner3d_core.hpp"
#include "lingtu/maps/no_go.hpp"

#include <octomap/OcTree.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>

namespace {
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
std::string read(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(file), {}};
}
void writePolicy(const std::filesystem::path& dir,
                 const std::vector<lingtu::maps::NoGoRegion>& regions) {
  std::ofstream file(dir / lingtu::maps::kNoGoFilename);
  file << lingtu::maps::NoGoRegionsJson(regions);
}
}

int main() try {
  using namespace octoplanner3d::runtime;
  using namespace lingtu::maps;
  const auto dir = std::filesystem::temp_directory_path() /
      ("lingtu-no-go-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(dir);
  const auto path = dir / "octomap.ot";
  octomap::OcTree tree(.2);
  for (int x=-16; x<=16; ++x) for (int y=-12; y<=12; ++y) {
    for (int z=1; z<10; ++z)
      tree.updateNode(octomap::point3d((x+.5)*.2f,(y+.5)*.2f,(z+.5)*.2f),false);
    tree.updateNode(octomap::point3d((x+.5)*.2f,(y+.5)*.2f,.1f),true);
  }
  tree.updateInnerOccupancy();
  require(tree.write(path.string()), "cannot write map");
  const auto measured = read(path);
  PlanRequest request;
  request.start={-2.3,.1,.3}; request.goal={2.3,.1,.3};
  request.options.robot_radius=.1;
  request.options.body_clearance_below_m=.05;
  request.options.body_clearance_above_m=.3;
  request.options.direct_support_only=true;
  request.options.surface_closing_radius_m=0;
  request.options.snap_search_radius_cells=2;
  request.options.max_iterations=20000;
  const auto baseline=runPlan(path,request);
  require(baseline.ok && baseline.reached_goal, "unrestricted baseline failed");
  const NoGoRegion box{"middle",{-.3,-.6,0},{.3,.6,1}};
  writePolicy(dir,{box});
  const auto detour=runPlan(path,request);
  require(detour.ok && detour.reached_goal, "global failed to route around policy box");
  bool leftCenter=false;
  for(std::size_t i=1;i<detour.path.size();++i) {
    const auto& a=detour.path[i-1]; const auto& b=detour.path[i];
    require(!SweptCylinderIntersectsNoGo(box,{a.x,a.y,a.z},{b.x,b.y,b.z},.1,.05,.3),
            "route crosses no-go region");
    leftCenter=leftCenter || std::abs(a.y)>.65;
  }
  require(leftCenter,"policy did not change route");
  PlannerSession session;
  lingtu::nav::plan::MapIdentity identity{"fixture",1,"map"};
  const auto cached=session.run(path,identity,request);
  require(cached.ok && cached.reached_goal,"cached planner ignored policy loading");
  writePolicy(dir,{{"upstairs",{-.3,-.6,3},{.3,.6,4}}});
  const auto below=runPlan(path,request);
  require(below.ok && below.reached_goal,"upper-floor restriction blocked lower floor");
  writePolicy(dir,{{"thin",{-.005,-3,0},{.005,3,1.5}}});
  const auto blocked=runPlan(path,request);
  require(!blocked.ok && !blocked.reached_goal,"search crossed thin no-go slab between samples");
  writePolicy(dir,{});
  const auto cleared=session.run(path,identity,request);
  require(cleared.ok && cleared.reached_goal,"removing policy did not restore route");
  require(read(path)==measured,"policy mutated measured OctoMap");
  const NoGoRegion corner{"corner",{0,0,0},{1,1,1}};
  require(!SweptCylinderIntersectsNoGo(corner,{-0.09,-0.09,.5},{-0.09,-0.09,.5},.1,.1,.1),
          "cylindrical body was expanded into a square at box corners");
  require(SweptCylinderIntersectsNoGo(corner,{-.07,-.07,.5},{-.07,-.07,.5},.1,.1,.1),
          "body contact at box corner was missed");
  std::filesystem::remove_all(dir);
  std::cout<<"no-go global: baseline, detour, thin slab, height separation, removal and immutable occupancy passed\n";
  return 0;
} catch(const std::exception& error) {
  std::cerr<<error.what()<<'\n'; return 1;
}
