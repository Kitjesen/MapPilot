#include "surface_graph.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace global_planner;
namespace {
void require(bool value,const char *message){if(!value)throw std::runtime_error(message);}
bool wall(const PointPose &p){return std::abs(p.x-2.)<.11&&std::abs(p.y)<.31;}
bool segment(const PointPose &a,const PointPose &b) {
  if(std::abs(a.z-b.z)>.21)return false;
  const int n=std::max(1,int(std::ceil(std::hypot(a.x-b.x,a.y-b.y,a.z-b.z)/.025)));
  for(int i=0;i<=n;++i){const double t=double(i)/n;
    if(wall({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z)}))return false;}
  return true;
}
double length(const std::vector<PointPose> &path) {
  double total=0;
  for(std::size_t i=1;i<path.size();++i)
    total+=std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y,path[i].z-path[i-1].z);
  return total;
}
bool same(const PointPose &a,const PointPose &b) {
  return std::hypot(a.x-b.x,a.y-b.y,a.z-b.z)<1e-8;
}
}
int main()try {
  PlannerConfig config;config.robot_radius=.1;config.search_resolution_m=.2;
  config.max_step_height=.2;config.max_slope=.6;config.snap_search_radius_cells=12;
  config.terminal_goal_tolerance_m=.15;config.terminal_goal_xy_tolerance_m=.15;config.terminal_goal_z_tolerance_m=.1;
  std::vector<PointPose> surfaces;
  for(int floor=0;floor<2;++floor)for(int x=0;x<=40;++x)for(int y=-10;y<=10;++y)
    surfaces.push_back({x*.1,y*.1,floor*2.});
  int pose_queries=0;bool build_cancelled=false;
  try {
    SurfaceGraph interrupted(surfaces,.1,config,
        [&](const auto&p){++pose_queries;return !wall(p);},segment,
        [&]{return pose_queries>=50;});
  }catch(const SurfaceGraph::Cancelled &){build_cancelled=true;}
  require(build_cancelled&&pose_queries==50,"cancel did not interrupt graph construction");
  SurfaceGraph graph(surfaces,.1,config,[](const auto&p){return !wall(p);},segment);
  require(graph.nodeCount()>0&&graph.nodeCount()<surfaces.size()/2,"graph was not thinned");
  const PointPose start{.035,.015,0},goal{3.95,.015,0};
  const auto route=graph.plan(start,goal,segment);
  require(!route.empty(),"wall detour not found");
  require(std::hypot(route.front().x-start.x,route.front().y-start.y)<1e-8,"raw start moved");
  for(std::size_t i=1;i<route.size();++i)require(segment(route[i-1],route[i]),"route crossed wall");
  require(graph.plan(start,{3.95,.015,2},segment).empty(),"separate floors were joined");
  require(graph.plan(start,goal,segment,[]{return true;}).empty(),"cancel ignored");
  require(graph.plan(start,goal,[](const auto&,const auto&){return false;}).empty(),"current obstacle ignored");
  require(!graph.plan(start,goal,segment).empty(),"request obstacle leaked");
  double side=0;
  for(const auto &p:route)if(std::abs(p.x-2)<.25){side=p.y;break;}
  require(std::abs(side)>.3,"detour did not clear wall");
  const auto alternate=graph.plan(start,goal,[&](const auto&a,const auto&b){
    if(!segment(a,b))return false;
    if((a.x<=2&&b.x>=2)||(a.x>=2&&b.x<=2)){
      const double t=std::abs(b.x-a.x)>1e-9?(2-a.x)/(b.x-a.x):0;
      if((a.y+t*(b.y-a.y))*side>0)return false;
    }
    return true;
  });
  require(!alternate.empty(),"blocked corridor was not rerouted");
  std::vector<PointPose> stairs;
  for(int x=0;x<=40;++x)for(int y=-5;y<=5;++y)
    stairs.push_back({(x+.5)*.05,(y+.5)*.05,(x/2)*.05});
  const auto slope=[](const PointPose &a,const PointPose &b) {
    return std::abs(a.z-b.z)<=.6*std::hypot(a.x-b.x,a.y-b.y)+1e-7;
  };
  config.search_resolution_m=.1;
  SurfaceGraph stair_graph(stairs,.05,config,[](const auto&){return true;},slope);
  require(!stair_graph.plan({.025,.025,0},{2.025,.025,1},slope).empty(),
          "5cm risers lost the 10cm slope-admissible connection");
  for(double resolution : {.05,.10,.20}) {
    std::vector<PointPose> flight;
    for(int x=0;x<=40;++x)
      flight.push_back({(x+.5)*resolution,.5*resolution,(x/2)*resolution});
    auto stair_config=config;
    stair_config.snap_search_radius_cells=2;
    const auto stair_edge=[&](const PointPose &a,const PointPose &b) {
      return slope(a,b) && std::hypot(a.x-b.x,a.y-b.y)<=2.01*resolution;
    };
    SurfaceGraph flight_graph(flight,resolution,stair_config,[](const auto&){return true;},stair_edge);
    for(bool reverse : {false,true}) {
      const auto path=flight_graph.plan(reverse?flight.back():flight.front(),
                                       reverse?flight.front():flight.back(),stair_edge);
      require(!path.empty(),"coarse surface graph disconnected a slope-admissible stair flight");
      for(std::size_t i=1;i<path.size();++i)
        require(stair_edge(path[i-1],path[i]),"stair route skipped its continuous edge check");
    }
    const double barrier=19.75*resolution;
    const auto blocked_stair=[&](const PointPose &a,const PointPose &b) {
      return stair_edge(a,b) && !((a.x<barrier&&b.x>=barrier)||(b.x<barrier&&a.x>=barrier));
    };
    SurfaceGraph blocked_flight(flight,resolution,stair_config,[](const auto&){return true;},blocked_stair);
    require(blocked_flight.plan(flight.front(),flight.back(),blocked_stair).empty(),
            "stair connection skipped a blocked edge");
    stair_config.max_step_height=resolution*.5;
    SurfaceGraph high_risers(flight,resolution,stair_config,[](const auto&){return true;},stair_edge);
    require(high_risers.plan(flight.front(),flight.back(),stair_edge).empty(),
            "longer connection bypassed the step-height limit");
    stair_config.max_step_height=config.max_step_height;
    stair_config.max_slope=.2;
    SurfaceGraph steep_flight(flight,resolution,stair_config,[](const auto&){return true;},stair_edge);
    require(steep_flight.plan(flight.front(),flight.back(),stair_edge).empty(),
            "longer connection bypassed the slope limit");
  }
  // Quantize the same 4m, 1:2 physical ramp instead of scaling its dimensions with the grid.
  for(double resolution : {.05,.10,.20}) {
    std::vector<PointPose> ramp;
    for(int x=0;x<int(std::lround(4./resolution));++x) {
      const double world_x=(x+.5)*resolution;
      ramp.push_back({world_x,0,std::floor(.5*world_x/resolution)*resolution});
    }
    auto ramp_config=config;
    ramp_config.max_slope=std::tan(std::acos(-1.)/6.);
    ramp_config.snap_search_radius_cells=2;
    const auto ramp_edge=[&](const PointPose &a,const PointPose &b) {
      return std::abs(a.z-b.z)<=ramp_config.max_slope*std::abs(a.x-b.x)+1e-7 &&
          std::abs(a.z-b.z)<=ramp_config.max_step_height+1e-7 &&
          std::abs(a.x-b.x)<=.401;
    };
    SurfaceGraph ramp_graph(ramp,resolution,ramp_config,[](const auto&){return true;},ramp_edge);
    const PointPose bottom{.4,0,.2},top{3.6,0,1.8};
    for(bool reverse : {false,true}) {
      const auto path=ramp_graph.plan(reverse?top:bottom,reverse?bottom:top,ramp_edge);
      require(!path.empty(),"map resolution changed connectivity of the same physical ramp");
      for(std::size_t i=1;i<path.size();++i)
        require(ramp_edge(path[i-1],path[i]),"quantized ramp bypassed its metric edge contract");
    }
  }
  std::vector<PointPose> elevated;
  for(int x=8;x<=60;++x)for(int y=-5;y<=5;++y)elevated.push_back({x*.05,y*.05,.2});
  const auto bent_entry=[&](const PointPose &a,const PointPose &b) {
    if(!slope(a,b))return false;
    const int n=std::max(1,int(std::ceil(std::hypot(a.x-b.x,a.y-b.y,a.z-b.z)/.01)));
    for(int i=0;i<=n;++i){const double t=double(i)/n;
      const double x=a.x+t*(b.x-a.x),y=a.y+t*(b.y-a.y),z=a.z+t*(b.z-a.z);
      if(x<-.01||x>3.01||std::abs(y)>.26||z<-.001||z>.201||(x<.2-1e-7&&std::abs(z)>1e-7))return false;
    }
    return true;
  };
  config.snap_search_radius_cells=24;
  SurfaceGraph elevated_graph(elevated,.05,config,[](const auto&){return true;},bent_entry);
  for(const auto &p:elevated)require(!bent_entry({0,0,0},p),"bent entry fixture has a direct shortcut");
  const auto bent=elevated_graph.plan({0,0,0},{3,0,.2},bent_entry);
  require(!bent.empty(),"local XYZ did not connect after level departure");
  require(std::abs(bent.front().x)+std::abs(bent.front().y)+std::abs(bent.front().z)<1e-8,
          "local attachment moved the original start");
  bool level_departure=false;
  for(std::size_t i=1;i<bent.size();++i){
    require(bent_entry(bent[i-1],bent[i]),"local attachment skipped an edge check");
    level_departure|=bent[i].x>=.19&&std::abs(bent[i].z)<1e-7;
  }
  require(level_departure,"local attachment did not preserve the required level first segment");
  std::vector<PointPose> plane;
  for(int x=0;x<=60;++x)for(int y=-15;y<=15;++y)plane.push_back({x*.1,y*.1,0});
  config.search_resolution_m=.2;
  config.snap_search_radius_cells=12;
  config.obstacle_clearance_weight=0;
  const auto open=[](const PointPose&,const PointPose&){return true;};
  SurfaceGraph plane_graph(plane,.1,config,[](const auto&){return true;},open);
  const PointPose plane_start{.035,.015,0},plane_goal{5.95,.215,0};
  const auto refined=plane_graph.plan(plane_start,plane_goal,open);
  // All sparse nodes consume this budget, leaving no work for fine refinement.
  config.max_iterations=static_cast<int>(plane_graph.nodeCount());
  SurfaceGraph budget_graph(plane,.1,config,[](const auto&){return true;},open);
  const auto coarse=budget_graph.plan(plane_start,plane_goal,open);
  require(!coarse.empty()&&!refined.empty(),"refinement exhausted the valid corridor fallback");
  require(length(refined)+.25<length(coarse),"route still detours through owner centres");
  require(same(refined.front(),plane_start)&&same(refined.back(),plane_goal),"refinement moved endpoints");
  std::size_t blocked_edge=refined.size();
  for(std::size_t i=1;i<refined.size();++i) {
    if(refined[i-1].x<2||refined[i].x>4)continue;
    bool existing=false;
    for(std::size_t j=1;j<coarse.size();++j)
      existing|=same(refined[i-1],coarse[j-1])&&same(refined[i],coarse[j]);
    if(!existing){blocked_edge=i;break;}
  }
  require(blocked_edge<refined.size(),"fixture lacks a fine-only interior edge");
  const auto blocked=[&](const PointPose &a,const PointPose &b) {
    return (same(a,refined[blocked_edge-1])&&same(b,refined[blocked_edge]))||
           (same(b,refined[blocked_edge-1])&&same(a,refined[blocked_edge]));
  };
  bool candidate_checked=false;
  const auto fallback=plane_graph.plan(plane_start,plane_goal,[&](const auto&a,const auto&b) {
    if(!blocked(a,b))return true;
    candidate_checked=true;
    return false;
  });
  require(candidate_checked,"refined candidate skipped current geometry");
  require(!fallback.empty(),"invalid fine edge prevented any valid route");
  require(same(fallback.front(),plane_start)&&same(fallback.back(),plane_goal),"detour moved endpoints");
  for(std::size_t i=1;i<fallback.size();++i)
    require(!blocked(fallback[i-1],fallback[i]),"detour used the blocked edge");
  require(length(fallback)<=length(coarse)+1e-8,"detour exceeds the valid corridor length");
  bool cancel_refinement=false;
  require(plane_graph.plan(plane_start,plane_goal,[&](const auto&a,const auto&b) {
    cancel_refinement|=blocked(a,b);
    return true;
  },[&]{return cancel_refinement;}).empty(),"candidate validation ignored cancellation");
  require(cancel_refinement,"fixture did not cancel during fine candidate validation");
  std::cout<<"Surface graph passed: sparse nodes="<<graph.nodeCount()<<" corridors="<<graph.edgeCount()
    <<", layers, wall detour, live reroute, request reset, raw start, build/query cancellation, stairs, bent local entry, owner refinement, budget/live fallback\n";
  return 0;
}catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}
