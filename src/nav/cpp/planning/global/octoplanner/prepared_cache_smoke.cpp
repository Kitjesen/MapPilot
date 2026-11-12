#include "octoplanner3d_core.hpp"
#include "lingtu/maps/no_go.hpp"
#include <octomap/OcTree.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace r=octoplanner3d::runtime;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
std::string bytes(const std::filesystem::path& path) {
  std::ifstream in(path,std::ios::binary);return {std::istreambuf_iterator<char>(in),{}};
}
bool same(const r::PlanResult& a,const r::PlanResult& b) {
  if(!a.ok || !b.ok || a.path.size()!=b.path.size())return false;
  for(std::size_t i=0;i<a.path.size();++i)
    if(a.path[i].x!=b.path[i].x || a.path[i].y!=b.path[i].y || a.path[i].z!=b.path[i].z)return false;
  return true;
}
}
int main() try {
  const auto directory=std::filesystem::temp_directory_path()/
      ("lingtu-prepared-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(directory);
  const auto map=directory/"octomap.ot";
  octomap::OcTree tree(.05);
  for(int x=-50;x<50;++x)for(int y=-8;y<8;++y)for(int z=-1;z<12;++z) {
    if(x==-8 && y==-5 && z==-1)continue;
    tree.updateNode((x+.5)*.05,(y+.5)*.05,(z+.5)*.05,z==-1,true);
  }
  tree.updateInnerOccupancy();require(tree.write(map.string()),"cannot write prepared-map fixture");
  const auto original=bytes(map);
  lingtu::nav::plan::MapIdentity identity{"prepared-test",1,"map"};
  r::PlanRequest request;
  request.start={-2.025,.125,.325};request.goal={2.025,.125,.325};
  auto& options=request.options;
  options.robot_radius=.1;options.body_clearance_below_m=.1;options.body_clearance_above_m=.2;
  options.support_height_m=.35;options.support_height_tolerance_m=.05;
  options.max_step_height=.15;options.max_slope=.6;options.max_iterations=20000;
  options.snap_search_radius_cells=4;options.enable_preblocked_costmap=false;
  options.terminal_goal_tolerance_m=.15;options.terminal_goal_xy_tolerance_m=.15;options.terminal_goal_z_tolerance_m=.05;
  r::PlanResult first;
  {
    r::PlannerSession session;
    session.prepare(map,identity,options);
    first=session.run(map,identity,request);
    require(first.ok && first.reached_goal,"fresh prepared route failed");
    auto precise=request;precise.options.terminal_goal_xy_tolerance_m=.14;
    require(same(first,session.run(map,identity,precise)),"query tolerance changed a valid exact route");
    auto limited=request;limited.options.max_iterations=1;
    const auto stopped=session.run(map,identity,limited);
    require(!stopped.ok && stopped.failure_reason=="search_iteration_limit","updated graph/grid budget was ignored");
    require(session.prepareCount()==1 && session.cacheLoadCount()==0,"query controls rebuilt prepared geometry");
    require(same(first,session.run(map,identity,request)),"query budget was not restored");
    auto hole=request;hole.goal={-.375,-.225,.325};
    hole.options.terminal_goal_tolerance_m=.001;hole.options.terminal_goal_xy_tolerance_m=.001;hole.options.terminal_goal_z_tolerance_m=.001;
    require(session.run(map,identity,hole).ok,"inferred support fixture failed");
  }
  require(std::filesystem::is_regular_file(directory/".nav/profile-0.bin"),"prepared geometry was not persisted");
  {
    r::PlannerSession session;
    require(same(first,session.run(map,identity,request)),"disk restore changed the route");
    require(session.cacheLoadCount()==1,"new session rebuilt instead of loading the saved graph");
    auto hole=request;hole.goal={-.375,-.225,.325};
    hole.options.terminal_goal_tolerance_m=.001;hole.options.terminal_goal_xy_tolerance_m=.001;hole.options.terminal_goal_z_tolerance_m=.001;
    require(session.run(map,identity,hole).ok,"persisted inferred support was lost");
    auto tall=request;tall.options.support_height_m=.435;tall.options.support_height_tolerance_m=.10;
    tall.start.z+=.085;tall.goal.z+=.085;
    require(session.run(map,identity,tall).ok,"second body-height profile failed");
    require(session.prepareCount()==2 && session.cacheLoadCount()==1,"different body geometry reused the wrong graph");
    require(same(first,session.run(map,identity,request)) && session.cacheLoadCount()==2,
            "returning to the first body did not restore its profile");
    auto blocked=request;
    blocked.no_go_regions=std::make_shared<const std::vector<lingtu::maps::NoGoRegion>>(
        std::vector<lingtu::maps::NoGoRegion>{{"wall",{-.1,-1.,-.2},{.1,1.,1.}}});
    require(!session.run(map,identity,blocked).ok,"prepared edges ignored a new request policy");
    require(same(first,session.run(map,identity,request)),"removed policy remained in prepared geometry");
    const auto prepares=session.prepareCount();++identity.content_epoch;
    require(session.run(map,identity,request).ok && session.prepareCount()==prepares+1 && session.cacheLoadCount()==2,
            "new map version reused stale prepared geometry");
  }
  require(bytes(map)==original,"preparation modified measured occupancy");
  std::filesystem::last_write_time(map,
      std::filesystem::last_write_time(map)+std::chrono::seconds(1));
  {
    r::PlannerSession session;
    require(session.run(map,identity,request).ok && session.cacheLoadCount()==0,
            "changed map source stamp reused stale prepared geometry");
  }
  std::filesystem::resize_file(directory/".nav/profile-0.bin",12);
  {
    r::PlannerSession session;
    require(session.run(map,identity,request).ok && session.cacheLoadCount()==0,
            "truncated cache did not rebuild from the saved map");
  }
  std::filesystem::remove_all(directory);
  std::cout<<"Prepared cache passed: query-only updates, disk roundtrip, inferred support, body profiles, map versions/source stamps, policy and truncation\n";
  return 0;
} catch(const std::exception& error) {
  std::cerr<<error.what()<<'\n';return 1;
}
