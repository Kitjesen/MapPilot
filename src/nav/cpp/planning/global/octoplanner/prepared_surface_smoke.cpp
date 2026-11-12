#include "global_planner.h"
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
  static void checkBodyHeight(OctoPlanner3D& planner) {
    const PointPose from{-.5,0,.325},to{-.43,0,.375};
    require(planner.metricConnectionWithinLimits(from,to) && planner.routeConnectionFree(from,to),
            "body-height change over the same floor was treated as a terrain slope");
    require(!planner.metricConnectionWithinLimits(from,{-.4,0,.465}),
            "body-height exception accepted an unsupported steep climb");
    require(!planner.metricConnectionWithinLimits(from,{.5,0,.525}),
            "body-height exception bypassed the step limit");
    require(!planner.metricConnectionWithinLimits(from,{-.5,0,.375}),
            "body-height exception accepted a vertical-only climb");
    auto raised=fixture(false);
    raised->updateNode(-.375,.025,.075,true);raised->updateInnerOccupancy();
    OctoPlanner3D steps;steps.setConfig(planner.graph_config_);steps.setOctomap(raised);
    require(!steps.metricConnectionWithinLimits({-.475,.025,.325},{-.375,.025,.425}),
            "body-height exception ignored a real change in observed floor height");
  }
  static void checkLatticeCache(OctoPlanner3D &planner,const PointPose &start) {
    planner.anchorPlanningLattice(start,.1);
    const auto cell=planner.worldToPlanningGrid(.225,-.075,.325);
    const bool before=planner.isPlanningCellTraversableDetailed(cell,planner.robot_radius_,true,true,2,nullptr);
    require(before,"lattice cache fixture must start with a free supported cell");
    planner.anchorPlanningLattice(start,.05);
    const bool expected=planner.queryWorld(planner.planningPoint(cell),planner.robot_radius_,true)==
        OctoPlanner3D::TraversabilityFailure::None;
    require(!expected,"lattice cache fixture must change the physical cell");
    require(planner.isPlanningCellTraversableDetailed(cell,planner.robot_radius_,true,true,2,nullptr)==expected,
            "fine lattice reused a body result from a different physical position");
  }
  static void checkGoalPolicy(OctoPlanner3D &planner,const PointPose &start,const PointPose &goal) {
    planner.makePlan(start,goal);
    std::vector<PointPose> graph_path;planner.getPlannerResults(graph_path);
    require(!graph_path.empty() && planner.searchInfo().used_surface_graph,
            "graph rejected an in-tolerance terminal accepted by grid search");
    const auto graph=planner.surface_graph_;planner.surface_graph_.reset();
    planner.makePlan(start,goal);
    std::vector<PointPose> grid_path;planner.getPlannerResults(grid_path);
    planner.surface_graph_=graph;
    require(!grid_path.empty(),"resolved terminal could not attach to grid search");
    const auto &a=graph_path.back(),&b=grid_path.back();
    require(std::hypot(a.x-b.x,a.y-b.y,a.z-b.z)<1e-6,
            "graph and grid selected different terminal positions");
    require(planner.queryWorld({float(b.x),float(b.y),float(b.z)},planner.robot_radius_,true)==
                OctoPlanner3D::TraversabilityFailure::None,
            "resolved terminal violates support or body geometry");
    require(std::hypot(b.x-goal.x,b.y-goal.y,b.z-goal.z)<=planner.terminal_goal_tolerance_m_+1e-6 &&
            std::hypot(b.x-goal.x,b.y-goal.y)<=planner.terminal_goal_xy_tolerance_m_+1e-6 &&
            std::abs(b.z-goal.z)<=planner.terminal_goal_z_tolerance_m_+1e-6,
            "resolved terminal exceeds a configured goal tolerance");
  }
  static void checkPreparedSupport(OctoPlanner3D &planner) {
    const auto &graph=*planner.surface_graph_;
    require(!graph.cells_.empty(),"prepared support topology is empty");
    for(std::size_t i=0;i<graph.cells_.size();++i) {
      const auto &point=graph.cells_[i];
      require(planner.queryWorld({float(point.x),float(point.y),float(point.z)},planner.robot_radius_,true)==
                  OctoPlanner3D::TraversabilityFailure::None,
              "prepared cell violates static support or body geometry");
      for(const auto &edge:graph.neighbors_[i])
        require(planner.routeConnectionFree(point,graph.cells_[edge.to]),
                "prepared edge bypassed static geometry or metric motion limits");
    }
  }
  static void checkStandingBand(OctoPlanner3D &planner) {
    const PointPose nominal{-.475,.025,.325};
    require(planner.queryWorld({-.475f,.025f,.325f},planner.robot_radius_,true)==
                OctoPlanner3D::TraversabilityFailure::OccupiedBody,
            "standing-band fixture must block the nominal body height");
    const auto &graph=*planner.surface_graph_;
    const auto column=graph.columns_.find(graph.key(nominal));
    bool adjusted=false;
    if(column!=graph.columns_.end())for(int id:column->second) {
      const auto &point=graph.cells_[id];
      adjusted|=point.z>nominal.z+.01 && point.z<=nominal.z+planner.support_height_tolerance_m_;
    }
    require(adjusted,"graph discarded a valid stance within the existing height band");
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
  OctoPlannerGridQueryTest::checkBodyHeight(planner);
  OctoPlannerGridQueryTest::checkPreparedSupport(planner);
  OctoPlannerGridQueryTest::checkLatticeCache(planner,start);
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
  OctoPlannerGridQueryTest::checkGoalPolicy(planner,start,{.125,-.075,.325});
  OctoPlannerGridQueryTest::checkGoalPolicy(planner,start,{.975,.025,.41});
  require(run({goal.x,goal.y,2.325}).empty(),"separate map floors were connected");
  planner.setOctomap(fixture(true));
  OctoPlannerGridQueryTest::checkPreparedSupport(planner);
  require(run(goal).empty(),"prepared graph crossed a complete static wall");
  auto steps=std::make_shared<octomap::OcTree>(.05);
  for(int x=-12;x<12;++x)for(int y=-8;y<8;++y)for(int z=-1;z<24;++z) {
    const int tread=x<0 ? -1 : x/4;
    const bool obstacle=x==1 && y==1 && z>=8 && z<=11;
    steps->updateNode((x+.5)*.05,(y+.5)*.05,(z+.5)*.05,z==tread||obstacle,true);
  }
  steps->updateInnerOccupancy();planner.setOctomap(steps);
  OctoPlannerGridQueryTest::checkPreparedSupport(planner);
  auto low_rail=fixture(false);
  low_rail->updateNode(-.475,.125,.225,true);low_rail->updateInnerOccupancy();
  planner.setOctomap(low_rail);
  OctoPlannerGridQueryTest::checkStandingBand(planner);
  OctoPlannerGridQueryTest::checkPreparedSupport(planner);
  std::cout<<"Prepared surface integration passed: native graph selected, raw endpoints, separate floors and static wall\n";
  return 0;
}catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}
