#include "global_planner.h"
#include "surface.hpp"
#include "surface_graph.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace global_planner;
namespace {
void require(bool value,const char *message) {
  if(!value)throw std::runtime_error(message);
}
std::shared_ptr<octomap::OcTree> fixture(bool closed_wall) {
  auto tree=std::make_shared<octomap::OcTree>(.05);
  for(int x=-24;x<24;++x)for(int y=-14;y<14;++y)for(int z=-1;z<55;++z) {
    const bool floor=z==-1 || z==39;
    const bool wall=x==0 && (closed_wall || y<0);
    tree->updateNode((x+.5)*.05,(y+.5)*.05,(z+.5)*.05,floor||wall,true);
  }
  tree->updateInnerOccupancy();
  return tree;
}
}
namespace global_planner {
struct OctoPlannerGridQueryTest {
  static void comparePreparedGeometry(OctoPlanner3D &planner) {
    const auto r=planner.octree_->getResolution();
    const auto surface=planner.surface_index_->surfaces(planner.support_height_m_+
        std::max(r,planner.body_clearance_above_m_));
    std::vector<PointPose> points;
    for(const auto &cell:surface)points.push_back({(cell.x+.5)*r,(cell.y+.5)*r,
        (cell.z+.5)*r+planner.support_height_m_});
    SurfaceGraph reference(std::move(points),r,planner.graph_config_,
      [&](const PointPose &p){return planner.queryWorld(octomap::point3d(p.x,p.y,p.z),planner.robot_radius_,true)==OctoPlanner3D::TraversabilityFailure::None;},
      [&](const PointPose &a,const PointPose &b){return planner.metricConnectionWithinLimits(a,b) &&
        planner.sweptRouteFree(octomap::point3d(a.x,a.y,a.z),octomap::point3d(b.x,b.y,b.z),nullptr,nullptr,true,true);});
    const auto &actual=*planner.surface_graph_;
    require(actual.cells_.size()==reference.cells_.size(),"prepare changed pose admission");
    require(actual.nodes_==reference.nodes_,"prepare changed sparse node placement");
    require(actual.edges_.size()==reference.edges_.size(),"prepare changed corridor count");
    for(std::size_t i=0;i<actual.cells_.size();++i) {
      require(actual.cells_[i].x==reference.cells_[i].x && actual.cells_[i].y==reference.cells_[i].y &&
        actual.cells_[i].z==reference.cells_[i].z,"prepare changed surface coordinates");
      const auto &a=actual.neighbors_[i],&b=reference.neighbors_[i];
      require(a.size()==b.size(),"swept AABB changed fine edge admission");
      for(std::size_t j=0;j<a.size();++j)
        require(a[j].to==b[j].to && a[j].cost==b[j].cost,"swept AABB changed fine edge or cost");
    }
    for(std::size_t i=0;i<actual.edges_.size();++i)
      require(actual.edges_[i].a==reference.edges_[i].a && actual.edges_[i].b==reference.edges_[i].b &&
        actual.edges_[i].cost==reference.edges_[i].cost && actual.edges_[i].cells==reference.edges_[i].cells,
        "prepare changed a sparse corridor");
  }
};
}
int main()try {
  octomap::OcTree mask_tree(.05);
  const std::vector<GridIndex> hits{{-9,-8,-2},{-8,-1,0},{-1,0,2},{0,7,-2},
      {7,8,0},{8,-9,2},{9,9,4}};
  for(const auto &p:hits)mask_tree.updateNode((p.x+.5)*.05,(p.y+.5)*.05,(p.z+.5)*.05,true,true);
  mask_tree.updateInnerOccupancy();
  SurfaceIndex mask_index(mask_tree,.55,0);
  for(int x=-10;x<=10;++x)for(int y=-10;y<=10;++y)
    for(int width:{0,1,3,8})for(int low_z=-3;low_z<=4;++low_z) {
      bool expected=false;
      for(const auto &p:hits)expected|=p.x>=x&&p.x<=x+width&&p.y>=y&&p.y<=y+width&&p.z>=low_z&&p.z<=low_z+1;
      require(mask_index.regionOccupied(x,y,x+width,y+width,low_z,low_z+1)==expected,
          "occupied block mask disagrees with independent XYZ hits");
    }
  PlannerConfig config;
  config.robot_radius=.1;config.body_clearance_below_m=.1;config.body_clearance_above_m=.2;
  config.support_height_m=.35;config.support_height_tolerance_m=.05;
  config.ground_support=true;config.direct_support_only=true;
  config.search_resolution_m=.1;config.snap_search_radius_cells=12;
  config.max_step_height=.15;config.max_slope=.6;config.max_iterations=10000;
  config.terminal_goal_tolerance_m=.1;config.terminal_goal_xy_tolerance_m=.1;
  config.terminal_goal_z_tolerance_m=.05;config.enable_preblocked_costmap=false;
  const PointPose start{-.975,.025,.325},goal{.975,.025,.325};
  OctoPlanner3D planner;planner.setConfig(config);planner.setOctomap(fixture(false));
  OctoPlannerGridQueryTest::comparePreparedGeometry(planner);
  const auto run=[&](const PointPose &target) {
    planner.makePlan(start,target);std::vector<PointPose> path;planner.getPlannerResults(path);return path;
  };
  const auto before=run(goal);
  require(!before.empty() && planner.searchInfo().used_surface_graph,
          "body-centre request only passed through XYZ fallback");
  require(std::hypot(before.front().x-start.x,before.front().y-start.y,before.front().z-start.z)<1e-6,
          "prepared route moved the actual start");
  require(std::hypot(before.back().x-goal.x,before.back().y-goal.y,before.back().z-goal.z)<1e-6,
          "valid raw goal was lost");
  for(const auto &p:before)require(std::abs(p.z-start.z)<.051,"route switched floors");
  planner.setObservedOccupiedCheck([](const PointPose &p,double radius,double below,double above,bool) {
    return std::abs(p.x-.025)<=radius+.025 && p.y+radius>=-.05 &&
        p.z+above>0 && p.z-below<.9;
  });
  require(run(goal).empty(),"current obstacle did not close the remaining passage");
  planner.setObservedOccupiedCheck({});
  require(!run(goal).empty() && planner.searchInfo().used_surface_graph,
          "clearing the current obstacle failed to restore the prepared route");
  require(run({goal.x,goal.y,2.325}).empty(),"separate map floors were connected");
  planner.setOctomap(fixture(true));
  OctoPlannerGridQueryTest::comparePreparedGeometry(planner);
  require(run(goal).empty(),"prepared graph crossed a complete static wall");
  auto steps=std::make_shared<octomap::OcTree>(.05);
  for(int x=-12;x<12;++x)for(int y=-8;y<8;++y)for(int z=-1;z<24;++z) {
    const int tread=x<0 ? -1 : x/4;
    const bool obstacle=x==1 && y==1 && z>=8 && z<=11;
    steps->updateNode((x+.5)*.05,(y+.5)*.05,(z+.5)*.05,z==tread||obstacle,true);
  }
  steps->updateInnerOccupancy();planner.setOctomap(steps);
  OctoPlannerGridQueryTest::comparePreparedGeometry(planner);
  std::cout<<"Prepared surface integration passed: native graph selected, raw endpoints, live block/release, separate floors and static wall\n";
  return 0;
}catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}
