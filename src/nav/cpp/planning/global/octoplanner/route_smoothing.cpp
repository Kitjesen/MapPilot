#include "route_smoothing.hpp"

#include <algorithm>
#include <cmath>

namespace global_planner {
namespace {
constexpr double kLevelTolerance=1e-6;
double distance(const PointPose &a, const PointPose &b) {
  return std::hypot(a.x-b.x,a.y-b.y,a.z-b.z);
}

bool pull(const std::vector<PointPose> &route, std::size_t first, std::size_t last,
          const EdgeCheck &valid, const BodyHeight &height, const RouteCancelCheck &cancel,
          std::vector<PointPose> &result) {
  if (cancel && cancel()) return false;
  const auto &a=route[first], &b=route[last];
  bool supported=true;
  if (last-first>1 && std::abs(a.z-b.z)>kLevelTolerance) {
    supported=static_cast<bool>(height);
    const int samples=std::max(1,static_cast<int>(std::ceil(distance(a,b)/.1)));
    // A long rising shortcut must admit standing poses, not only the stair sweep.
    for(int i=1;i<samples && supported;++i) {
      if(cancel && cancel())return false;
      const double t=static_cast<double>(i)/samples;
      supported=height({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z)}).has_value();
    }
  }
  if (supported && valid(a,b)) {
    result.push_back(route[last]);
    return true;
  }
  if (last-first==1) return false;
  const auto middle=first+(last-first)/2;
  return pull(route,first,middle,valid,height,cancel,result) &&
         pull(route,middle,last,valid,height,cancel,result);
}
}

std::vector<PointPose> smoothRoute(const std::vector<PointPose> &route,
                                  const EdgeCheck &valid,
                                  const BodyHeight &height,
                                  const RouteCancelCheck &cancel) {
  if (cancel && cancel()) return {};
  if (route.empty()) return {};
  if (route.size()==1) return {route.front()};
  std::vector<PointPose> pulled{route.front()};
  if (!pull(route,0,route.size()-1,valid,height,cancel,pulled)) return {};
  std::vector<PointPose> result{pulled.front()};
  for (std::size_t i=1;i<pulled.size();++i) {
    if (cancel && cancel()) return {};
    const auto &a=pulled[i-1], &b=pulled[i];
    if (std::abs(a.z-b.z)>kLevelTolerance) {
      result.push_back(b);
      continue;
    }
    const int count=std::max(1,static_cast<int>(std::ceil(distance(a,b)/.1)));
    std::vector<PointPose> segment{a};
    bool allowed=true;
    double length=0;
    for (int j=1;j<=count && allowed;++j) {
      if (cancel && cancel()) return {};
      const double t=static_cast<double>(j)/count;
      PointPose point=j==count ? b : PointPose{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z)};
      if (height && j<count) {
        const auto z=height(point);
        if (!z) { allowed=false; break; }
        point.z=*z;
      }
      allowed=valid(segment.back(),point);
      length+=distance(segment.back(),point);
      segment.push_back(point);
    }
    // A valid stair edge need not admit projected intermediate standing poses.
    if (allowed && length<=distance(a,b)+1e-8)
      result.insert(result.end(),segment.begin()+1,segment.end());
    else result.push_back(b);
  }
  // DimOS-inspired midpoint relaxation; no wall-gradient force without a distance field.
  std::vector<bool> dirty(result.size(),true);
  for (int pass=0;pass<30;++pass) {
    bool moved=false;
    for (std::size_t i=1;i+1<result.size();++i) {
      if (cancel && cancel()) return {};
      if (!dirty[i]) continue;
      dirty[i]=false;
      const auto &a=result[i-1], &b=result[i+1], &current=result[i];
      if (std::abs(a.z-current.z)>kLevelTolerance || std::abs(b.z-current.z)>kLevelTolerance)
        continue;
      PointPose candidate{.5*current.x+.25*(a.x+b.x),
                          .5*current.y+.25*(a.y+b.y),current.z};
      // Any accepted height projection changes Z by at most kLevelTolerance.
      if (std::hypot(candidate.x-current.x,candidate.y-current.y,kLevelTolerance)<1e-5)
        continue;
      if (height) {
        const auto z=height(candidate);
        if (!z) continue;
        candidate.z=*z;
        if (std::abs(candidate.z-current.z)>kLevelTolerance) continue;
      }
      if (distance(current,candidate)<1e-5 ||
          distance(a,candidate)+distance(candidate,b)>distance(a,current)+distance(current,b)+1e-9)
        continue;
      if (!valid(a,candidate) || !valid(candidate,b)) continue;
      result[i]=candidate;
      dirty[i-1]=dirty[i]=dirty[i+1]=true;
      moved=true;
    }
    if (!moved) break;
  }
  return cancel && cancel() ? std::vector<PointPose>{} : result;
}

}  // namespace global_planner
