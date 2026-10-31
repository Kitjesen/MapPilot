// Offline comparison of native support/body predicates at recorded physical poses.
// Does not issue commands, plan a route or reinterpret a missing floor as free support.
#include "body_query.hpp"
#include "collision/body_io.hpp"
#include "global_planner.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace c=lingtu::nav::collision;
namespace r=octoplanner3d::runtime;
int main(int argc,char **argv) try {
  if(argc!=2) throw std::runtime_error("usage: support_gait_probe case_dir");
  const std::filesystem::path dir=argv[1];
  std::ifstream scene(dir/"scene.txt");
  double res; c::Vec3 lo,hi; int count;
  scene>>res>>lo.x>>lo.y>>lo.z>>hi.x>>hi.y>>hi.z>>count;
  std::vector<std::pair<c::Vec3,c::Vec3>> boxes;
  for(int i=0;i<count;++i) {c::Vec3 a,b; scene>>a.x>>a.y>>a.z>>b.x>>b.y>>b.z;boxes.emplace_back(a,b);}
  if(!scene || res<=0) throw std::runtime_error("invalid scene");
  octomap::OcTree tree(res);
  for(int x=std::lround(lo.x/res);x<std::lround(hi.x/res);++x)
    for(int y=std::lround(lo.y/res);y<std::lround(hi.y/res);++y)
      for(int z=std::lround(lo.z/res);z<std::lround(hi.z/res);++z) {
        const c::Vec3 p{(x+.5)*res,(y+.5)*res,(z+.5)*res};
        bool occupied=false;
        for(const auto &[a,b]:boxes) occupied=occupied ||
          (p.x>=a.x && p.x<b.x && p.y>=a.y && p.y<b.y && p.z>=a.z && p.z<b.z);
        tree.updateNode(p.x,p.y,p.z,occupied,true)->setLogOdds(occupied?3.5f:-3.5f);
      }
  tree.updateInnerOccupancy();tree.prune();tree.write((dir/"map.ot").string());
  std::ifstream body_file(dir/"nominal.body2");
  const auto nominal=c::readBody(body_file);
  auto map=std::make_shared<octomap::OcTree>(tree);
  global_planner::OctoPlanner3D strict,nearby;
  std::ifstream records(dir/"probe.txt");
  std::ofstream out(dir/"native.csv");
  out<<"t,strict_support,nearby_support,nominal_free,nominal_reason,nominal_link,observed_free,observed_reason,observed_link\n";
  std::string line;bool initialized=false;
  const lingtu::nav::plan::GlobalPlanTemporaryOverlay overlay;
  while(std::getline(records,line)) {
    std::istringstream row(line);double t,x,y,z,yaw;std::string file;
    if(!(row>>t>>x>>y>>z>>yaw>>file)) throw std::runtime_error("invalid pose record");
    const global_planner::PointPose p{x,y,z};
    if(!initialized) {
      global_planner::PlannerConfig cfg;cfg.support_height_m=z;cfg.support_height_tolerance_m=.05;
      cfg.max_slope=.5773502691896257;cfg.max_step_height=.15;cfg.ground_support=true;
      cfg.direct_support_only=true;cfg.support_radius_cells=0;cfg.enable_preblocked_costmap=false;
      strict.setConfig(cfg);strict.setOctomap(map);strict.preparePoseQueries(p);
      cfg.direct_support_only=false;cfg.support_radius_cells=1;
      nearby.setConfig(cfg);nearby.setOctomap(map);nearby.preparePoseQueries(p);
      initialized=true;
    }
    std::ifstream snapshot(dir/file);const auto observed=c::readBody(snapshot);
    const c::Pose pose{{x,y,z},yaw};
    const auto a=r::BodyQuery(tree,*nominal,overlay).pose(pose);
    const auto b=r::BodyQuery(tree,*observed,overlay).pose(pose);
    out<<std::setprecision(17)<<t<<','<<strict.poseSupportFree(p)<<','<<nearby.poseSupportFree(p)
       <<','<<a.free<<','<<std::quoted(a.reason)<<','<<std::quoted(a.link)
       <<','<<b.free<<','<<std::quoted(b.reason)<<','<<std::quoted(b.link)<<'\n';
  }
  if(!initialized) throw std::runtime_error("no recorded poses");
  return 0;
} catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
