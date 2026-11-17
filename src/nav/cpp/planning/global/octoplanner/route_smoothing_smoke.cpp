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
  const EdgeCost wall_cost=[](const auto &a,const auto &b) {
    const double dx=b.x-a.x,dy=b.y-a.y;
    const double t=std::clamp(-(a.x*dx+a.y*dy)/(dx*dx+dy*dy),0.,1.);
    return distance(a,b)+(std::hypot(a.x+t*dx,a.y+t*dy)<.7 ? 100. : 0.);
  };
  const auto preferred=smoothRoute(detour,flat_clear,[](const auto &p){return std::optional<double>(p.z);}, {},wall_cost);
  double preferred_cost=0;
  for(std::size_t i=1;i<preferred.size();++i) preferred_cost+=wall_cost(preferred[i-1],preferred[i]);
  require(preferred.size()>2 && preferred_cost<=length(detour)+1e-8,
          "smoothing erased the route's wall-distance preference");
  const auto measured_result=smoothRoute(measured_flat,flat_clear,[](const auto &){return std::optional<double>(.025);});
  check(measured_flat,measured_result,flat_clear);
  for(const auto &point:measured_result)
    require(std::abs(point.z+.01)<1e-8,"flat route replaced measured height with nominal height");

  auto measured_detour=detour;
  measured_detour.front().z=.37;
  measured_detour.back().z=.37;
  int first_probes=0,last_probes=0;
  const BodyHeight flat_band=[&](const auto &p) -> std::optional<double> {
    if(p.x==-1 && p.y==0) ++first_probes;
    if(p.x==1 && p.y==0) ++last_probes;
    return std::abs(p.z-.35)<=.05 ? std::optional<double>(p.z) : std::nullopt;
  };
  const auto measured_rounded=smoothRoute(measured_detour,wall,flat_band);
  check(measured_detour,measured_rounded,wall);
  require(length(measured_rounded)<length(rounded)+.05,
          "same-floor endpoint height froze the wall-detour corners");
  require(first_probes==1 && last_probes==1,"endpoint level was sampled repeatedly");

  auto final_step=detour;
  final_step.back().z=.55;
  const BodyHeight final_tread=[](const auto &p) -> std::optional<double> {
    const double z=p.x>.5 && p.y<.1 ? .55 : .35;
    return std::abs(p.z-z)<=.05 ? std::optional<double>(z) : std::nullopt;
  };
  const auto step_corner=smoothRoute(final_step,wall,final_tread);
  check(final_step,step_corner,wall);
  require(std::any_of(step_corner.begin(),step_corner.end(),[&](const auto &p) {
    return distance(p,final_step[final_step.size()-2])==0;
  }),"endpoint step released its lower-floor corner");

  bool probe_cancelled=false;
  require(smoothRoute(measured_detour,wall,[&](const auto &p) {
    probe_cancelled=probe_cancelled || (p.x==-1 && p.y==0);
    return std::optional<double>(p.z);
  },[&]{return probe_cancelled;}).empty(),"endpoint probe skipped a pending cancellation");

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
  const std::vector<PointPose> ramp{{0,0,.35},{.5,.4,.45},{1,0,.55}};
  const auto ramp_result=smoothRoute(ramp,flat_clear,[](const auto &p) {
    return std::optional<double>(.35+.2*p.x);
  });
  check(ramp,ramp_result,flat_clear);
  require(ramp_result.size()==2,"continuous ramp did not simplify");

  const std::vector<PointPose> transition{{0,0,.35},{.5,0,.45},{1,0,.55}};
  int transition_samples=0;
  const auto transition_result=smoothRoute(transition,flat_clear,[&](const auto &p) {
    ++transition_samples;
    return std::optional<double>(p.x<.5?.35:.55);
  });
  require(transition_result.size()==transition.size(),"rejected shortcut changed its terrain points");
  for(std::size_t i=0;i<transition.size();++i)
    require(distance(transition_result[i],transition[i])==0,"rejected shortcut moved a terrain point");
  require(transition_samples==8,"rejected shortcut kept sampling after its terrain result was final");
  bool transition_cancelled=false;
  require(smoothRoute(transition,flat_clear,[&](const auto &p) {
    transition_cancelled=p.x>=.5;
    return std::optional<double>(p.x<.5?.35:.55);
  },[&]{return transition_cancelled;}).empty(),"terrain rejection skipped a pending cancellation");

  const std::vector<PointPose> voxel_backtrack{
      {0,0,.35},{.1,0,.35},{.05,0,.35},{.15,0,.45},
      {.1,0,.45},{.2,0,.45},{.25,0,.5}};
  const BodyHeight stepped_surface=[](const auto &p) -> std::optional<double> {
    return std::optional<double>(p.x<.12?.35:p.x<.21?.45:.5);
  };
  const auto backtrack_result=smoothRoute(voxel_backtrack,flat_clear,stepped_surface);
  check(voxel_backtrack,backtrack_result,flat_clear);
  for (std::size_t i=1;i<backtrack_result.size();++i)
    require(backtrack_result[i].x>=backtrack_result[i-1].x-1e-8,
            "surface shortcut kept a horizontal backtrack");
  require(length(backtrack_result)<length(voxel_backtrack)-.05,
          "surface shortcut did not shorten the voxel backtrack");

  std::vector<PointPose> eight_steps{{2,0,.435},{3.9,0,.435}};
  for(int step=0;step<8;++step) {
    const double x=4+step*.5, z=.435+(step+1)*.1;
    eight_steps.push_back({x+.1,0,z});
    eight_steps.push_back({x+.4,0,z});
  }
  eight_steps.push_back({10,0,1.235});
  const BodyHeight step_profile=[](const auto &p) {
    const int step=std::clamp(static_cast<int>(std::floor((p.x-4)/.5))+1,0,8);
    return std::optional<double>(.435+step*.1);
  };
  const auto step_result=smoothRoute(eight_steps,flat_clear,step_profile);
  check(eight_steps,step_result,flat_clear);
  const auto step_z=[&](double x) {
    for(std::size_t i=1;i<step_result.size();++i) if(step_result[i].x>=x) {
      const auto &a=step_result[i-1],&b=step_result[i];
      return a.z+(x-a.x)/(b.x-a.x)*(b.z-a.z);
    }
    throw std::runtime_error("step route ended before its tread");
  };
  require(std::abs(step_z(3.5)-.435)<1e-8,"stair shortcut lifted the lower platform");
  for(int step=0;step<8;++step)
    require(std::abs(step_z(4.25+step*.5)-(.435+(step+1)*.1))<1e-8,
            "stair shortcut flattened a tread");
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
  std::cout<<"Route smoothing passed: wall, stacked floor, stair profile, ramp, platform lookahead, endpoints, length, cancellation\n";
  return 0;
} catch(const std::exception &error) {
  std::cerr<<error.what()<<'\n';return 1;
}
