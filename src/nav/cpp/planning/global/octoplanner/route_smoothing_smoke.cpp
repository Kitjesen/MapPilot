#include "route_smoothing.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>

using namespace global_planner;
namespace {
double distance(const PointPose &a,const PointPose &b) {
  return std::hypot(a.x-b.x,a.y-b.y,a.z-b.z);
}
double length(const std::vector<PointPose> &route) {
  double result=0;
  for(std::size_t i=1;i<route.size();++i) result+=distance(route[i-1],route[i]);
  return result;
}
void require(bool ok,const char *why) { if(!ok)throw std::runtime_error(why); }
void check(const std::vector<PointPose> &before,const std::vector<PointPose> &after,const EdgeCheck &valid) {
  require(!after.empty(),"smoothing lost a valid path");
  require(distance(before.front(),after.front())==0 && distance(before.back(),after.back())==0,
          "smoothing moved a requested endpoint");
  require(length(after)<=length(before)+1e-7,"smoothing increased path length");
  for(std::size_t i=1;i<after.size();++i) require(valid(after[i-1],after[i]),"smoothing accepted an invalid edge");
}
}
int main() try {
  const EdgeCheck wall=[](const auto &a,const auto &b) {
    const int count=std::max(1,static_cast<int>(std::ceil(distance(a,b)/.005)));
    for(int i=0;i<=count;++i) {
      const double t=static_cast<double>(i)/count;
      if(std::abs(a.x+t*(b.x-a.x))<=.4 && std::abs(a.y+t*(b.y-a.y))<=.4) return false;
    }
    return true;
  };
  const std::vector<PointPose> detour{{-1,0,.35},{-1,1,.35},{1,1,.35},{1,0,.35}};
  int wall_queries=0,height_queries=0;
  const auto rounded=smoothRoute(detour,[&](const auto &a,const auto &b){++wall_queries;return wall(a,b);},
      [&](const auto &p){++height_queries;return std::optional<double>(p.z);});
  check(detour,rounded,wall);
  require(length(rounded)<length(detour)-.1,"wall detour did not improve");
  std::cout<<std::setprecision(17)<<"Wall work: edge="<<wall_queries<<" height="<<height_queries
    <<" length="<<length(rounded)<<'\n';

  const std::vector<PointPose> upper{{-1,0,2.35},{-.4,.2,2.35},{1,0,2.35}};
  const EdgeCheck upper_only=[](const auto &a,const auto &b){return std::abs(a.z-2.35)<1e-8 && std::abs(b.z-2.35)<1e-8;};
  const auto upper_result=smoothRoute(upper,upper_only,[](const auto &p){return std::optional<double>(p.z>1?2.35:.35);});
  check(upper,upper_result,upper_only);
  for(const auto &point:upper_result) require(std::abs(point.z-2.35)<1e-8,"stacked-floor route changed its level");

  const std::vector<PointPose> measured_flat{{-1,0,-.01},{-.5,0,.025},{.5,0,.025},{1,0,-.01}};
  const EdgeCheck flat_clear=[](const auto &,const auto &){return true;};
  const auto measured_result=smoothRoute(measured_flat,flat_clear,[](const auto &){return std::optional<double>(.025);});
  check(measured_flat,measured_result,flat_clear);
  for(const auto &point:measured_result)
    require(std::abs(point.z+.01)<1e-8,"flat route replaced measured height with nominal height");

  const std::vector<PointPose> stairs{{0,0,.35},{.3,0,.35},{.6,0,.5},{1,0,.5}};
  const BodyHeight tread=[](const auto &p){return std::optional<double>(p.x<.5?.35:.5);};
  const EdgeCheck stair_edge=[&](const auto &a,const auto &b) {
    if(std::abs(a.z-*tread(a))>1e-8 || std::abs(b.z-*tread(b))>1e-8) return false;
    return std::abs(a.z-b.z)<1e-8 || (b.x-a.x>=.299 && std::abs(a.z-b.z)<=.151);
  };
  check(stairs,smoothRoute(stairs,stair_edge,tread),stair_edge);
  const std::vector<PointPose> platform{{0,0,.35},{2,0,.35},{4.7,0,.35},{5.1,0,.55},{8,0,.55}};
  const BodyHeight platform_height=[](const auto &p) -> std::optional<double> {
    const double z=p.x<5?.35:.55;
    return std::abs(p.z-z)<=.05 ? std::optional<double>(z) : std::nullopt;
  };
  const EdgeCheck platform_edge=[&](const auto &a,const auto &b) {
    const auto az=platform_height(a),bz=platform_height(b);
    return az && bz && std::abs(a.z-*az)<1e-6 && std::abs(b.z-*bz)<1e-6 &&
      std::abs(a.z-b.z)<=.6*std::hypot(a.x-b.x,a.y-b.y)+1e-6;
  };
  const auto preserved=smoothRoute(platform,platform_edge,platform_height);
  check(platform,preserved,platform_edge);
  for(const auto &turn:std::vector<PointPose>{platform[2],platform[3]})
    require(std::any_of(preserved.begin(),preserved.end(),[&](const auto &p){return distance(p,turn)<1e-8;}),
            "platform height transition moved during smoothing");
  const auto at_x=[&](double x) {
    for(std::size_t i=1;i<preserved.size();++i) if(preserved[i].x>=x) {
      const auto &a=preserved[i-1],&b=preserved[i];
      return a.z+(x-a.x)/(b.x-a.x)*(b.z-a.z);
    }
    throw std::runtime_error("platform route ended before the lookahead");
  };
  const double lookahead_z=at_x(3.5), low_platform_z=at_x(4.0);
  require(std::abs(lookahead_z-.35)<1e-8 && std::abs(low_platform_z-.35)<1e-8 &&
              low_platform_z-.35-.05<=0 && low_platform_z-.35+.05>=0,
          "lookahead interpolation lifted the reference above the low platform");
  const std::vector<PointPose> nominal_detour{{0,0,.35},{0,.6,.4},{1,.6,.35},{2,0,.37}};
  const BodyHeight nominal_height=[](const auto &p) -> std::optional<double> {
    return std::abs(p.z-.35)<=.05+1e-8 ? std::optional<double>(.35) : std::nullopt;
  };
  const auto nominal_result=smoothRoute(nominal_detour,flat_clear,nominal_height);
  check(nominal_detour,nominal_result,flat_clear);
  require(length(nominal_result)<2.001,"nominal height samples prevented a supported shortcut");
  const std::vector<PointPose> edge_route{{0,0,.35},{1,0,.35},{1,1,.35},{2,1,.35}};
  const BodyHeight edge_ground=[](const auto &p) -> std::optional<double> {
    if (p.x>.55 && p.x<.95 && p.y>.1) return std::nullopt;
    return .35;
  };
  const auto edge_result=smoothRoute(edge_route,flat_clear,edge_ground);
  require(edge_result.size()>2 && length(edge_result)>std::hypot(2.,1.)+.05,
          "start shortcut crossed the missing ground");
  const EdgeCheck denied=[](const auto &,const auto &){return false;};
  require(smoothRoute(stairs,denied,tread).empty(),"invalid original edge survived");
  int calls=0;
  require(smoothRoute(detour,wall,{},[&]{return ++calls>4;}).empty(),"cancellation returned a partial route");
  std::cout<<"Route smoothing passed: wall, stacked floor, stair resampling, platform lookahead, endpoints, length, cancellation\n";
  return 0;
} catch(const std::exception &error) {
  std::cerr<<error.what()<<'\n';return 1;
}
