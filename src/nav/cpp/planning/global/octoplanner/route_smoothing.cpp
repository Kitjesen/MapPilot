#include "route_smoothing.hpp"

#include <algorithm>
#include <cmath>

namespace global_planner {
namespace {
constexpr double kLevelTolerance=1e-6;
double distance(const PointPose &a, const PointPose &b) {
  return std::hypot(a.x-b.x,a.y-b.y,a.z-b.z);
}

}

bool checkRoute(std::vector<PointPose> &route, const EdgeCheck &valid,
                const RouteCancelCheck &cancel) {
  if (route.empty() || (cancel && cancel())) return false;
  std::vector<PointPose> compact{route.front()};
  for (std::size_t i=1;i<route.size();++i) {
    if (distance(compact.back(),route[i])>1e-7) compact.push_back(route[i]);
    else if (i+1==route.size()) {
      compact.back().yaw=route[i].yaw;
    }
  }
  route=std::move(compact);
  if (route.size()==1) return valid(route.front(),route.front());
  for (std::size_t i=1;i<route.size();++i)
    if ((cancel && cancel()) || !valid(route[i-1],route[i])) return false;
  return true;
}

std::vector<PointPose> smoothRoute(const std::vector<PointPose> &route,
                                  const EdgeCheck &valid,
                                  const BodyHeight &height,
                                  const RouteCancelCheck &cancel,
                                  const EdgeCost &cost) {
  if (cancel && cancel()) return {};
  auto result=route;
  if (result.size()<2) return result;
  if (!checkRoute(result,valid,cancel)) return {};
  const auto path_cost=[&](const auto &points,std::size_t first,std::size_t last) {
    double total=0;
    for(std::size_t i=first+1;i<=last;++i) total+=cost(points[i-1],points[i]);
    return total;
  };
  const auto route_length=[](const auto &points) {
    double total=0;
    for (std::size_t i=1;i<points.size();++i) total+=distance(points[i-1],points[i]);
    return total;
  };
  const auto has_backtrack=[&](std::size_t first,std::size_t last) {
    const auto &origin=result[first], &target=result[last];
    const double dx=target.x-origin.x,dy=target.y-origin.y;
    const double length_xy=std::hypot(dx,dy);
    if (length_xy<=1e-7) return false;
    const double ux=dx/length_xy,uy=dy/length_xy;
    double previous=0.0;
    for (std::size_t i=first+1;i<=last;++i) {
      const double progress=(result[i].x-origin.x)*ux+(result[i].y-origin.y)*uy;
      if (progress<previous-1e-6) return true;
      previous=progress;
    }
    return false;
  };
  const auto surface_shortcut=[&](std::size_t first,std::size_t last) {
    if (!height || !has_backtrack(first,last)) return std::vector<PointPose>{};
    const auto &a=result[first], &b=result[last];
    const double horizontal=std::hypot(b.x-a.x,b.y-a.y);
    const int samples=std::max(1,static_cast<int>(std::ceil(horizontal/.1)));
    std::vector<PointPose> candidate{a};
    candidate.reserve(static_cast<std::size_t>(samples)+1);
    for (int i=1;i<samples;++i) {
      if (cancel && cancel()) return std::vector<PointPose>{};
      const double t=static_cast<double>(i)/samples;
      const auto source=first+static_cast<std::size_t>(std::llround(
          t*static_cast<double>(last-first)));
      PointPose point{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),result[source].z};
      const auto z=height(point);
      if (!z) return std::vector<PointPose>{};
      point.z=*z;
      point.yaw.reset();
      candidate.push_back(point);
    }
    candidate.push_back(b);
    for (std::size_t i=1;i<candidate.size();++i)
      if (!valid(candidate[i-1],candidate[i])) return std::vector<PointPose>{};
    double baseline_length=0;
    for (std::size_t i=first+1;i<=last;++i) baseline_length+=distance(result[i-1],result[i]);
    if (route_length(candidate)>baseline_length+1e-8) return std::vector<PointPose>{};
    if (cost) {
      double candidate_cost=0;
      for (std::size_t i=1;i<candidate.size();++i)
        candidate_cost+=cost(candidate[i-1],candidate[i]);
      if (candidate_cost>path_cost(result,first,last)+1e-8)
        return std::vector<PointPose>{};
    }
    return candidate;
  };
  for (std::size_t first=0;first+2<result.size();++first) {
    for (std::size_t last=result.size()-1;last>first+1;--last) {
      if (cancel && cancel()) return {};
      const auto &a=result[first], &b=result[last];
      bool supported=true;
      std::optional<double> first_height;
      bool changing_height=false, follows_height=true;
      if ((first==0 && height) || std::abs(a.z-b.z)>kLevelTolerance) {
        supported=static_cast<bool>(height);
        const int samples=std::max(1,static_cast<int>(std::ceil(distance(a,b)/.1)));
        for(int i=1;i<samples && supported;++i) {
          if(cancel && cancel())return {};
          const double t=static_cast<double>(i)/samples;
          const PointPose point{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z)};
          const auto z=height(point);
          supported=z.has_value();
          if (!z) continue;
          if (first_height) changing_height=changing_height || std::abs(*z-*first_height)>kLevelTolerance;
          else first_height=z;
          follows_height=follows_height && std::abs(*z-point.z)<=kLevelTolerance;
          if (changing_height && !follows_height) break;
        }
      }
      // Keep terrain transitions while allowing a constant measured-height offset on a flat floor.
      if (!supported || (changing_height && !follows_height)) continue;
      if (valid(a,b) && (!cost || cost(a,b)<=path_cost(result,first,last)+1e-8)) {
        result.erase(result.begin()+first+1,result.begin()+last);
        break;
      }
      // Surface-following shortcut removes voxel-scale horizontal backtracks
      // without relaxing the existing edge, support, or clearance checks.
      auto candidate=surface_shortcut(first,last);
      if (!candidate.empty()) {
        result.erase(result.begin()+first+1,result.begin()+last);
        result.insert(result.begin()+first+1,candidate.begin()+1,candidate.end()-1);
        break;
      }
    }
  }
  std::vector<PointPose> dense{result.front()};
  for (std::size_t i=1;i<result.size();++i) {
    const auto &a=result[i-1], &b=result[i];
    const int count=std::abs(a.z-b.z)>kLevelTolerance ? 1 :
        std::max(1,static_cast<int>(std::ceil(distance(a,b)/.1)));
    for (int j=1;j<=count;++j) {
      if (cancel && cancel()) return {};
      const double t=static_cast<double>(j)/count;
      PointPose point=j==count ? b : PointPose{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z)};
      if (height && j<count) {
        const auto z=height(point);
        if (!z) { dense.clear(); break; }
        point.z=*z;
      }
      dense.push_back(point);
    }
    if (dense.empty()) break;
  }
  if (!dense.empty()) {
    if (route_length(dense)<=route_length(result)+1e-8 && checkRoute(dense,valid,cancel) &&
        (!cost || path_cost(dense,0,dense.size()-1)<=path_cost(result,0,result.size()-1)+1e-8))
      result=std::move(dense);
  }
  double first_level=result.front().z, last_level=result.back().z;
  if (height && result.size()>2) {
    const auto endpoint_level=[&](const PointPose &endpoint,double neighbor_z) {
      auto probe=endpoint;
      probe.z=neighbor_z;
      const auto level=height(probe);
      return level && std::abs(*level-probe.z)<=kLevelTolerance ? probe.z : endpoint.z;
    };
    // A measured endpoint may sit elsewhere within the same supported height band.
    first_level=endpoint_level(result.front(),result[1].z);
    last_level=endpoint_level(result.back(),result[result.size()-2].z);
  }
  std::vector<bool> dirty(result.size(),true);
  for (int pass=0;pass<30;++pass) {
    bool moved=false;
    for (std::size_t i=1;i+1<result.size();++i) {
      if (cancel && cancel()) return {};
      if (!dirty[i]) continue;
      dirty[i]=false;
      const auto &a=result[i-1], &b=result[i+1], &current=result[i];
      const double a_level=i==1 ? first_level : a.z;
      const double b_level=i+2==result.size() ? last_level : b.z;
      if (std::abs(a_level-current.z)>kLevelTolerance || std::abs(b_level-current.z)>kLevelTolerance)
        continue;
      PointPose point{.5*current.x+.25*(a.x+b.x),.5*current.y+.25*(a.y+b.y),current.z};
      if (distance(point,current)<1e-5) continue;
      if (height) {
        const auto z=height(point);
        if (!z || std::abs(*z-current.z)>kLevelTolerance) continue;
        point.z=*z;
      }
      if (distance(a,point)+distance(point,b)>distance(a,current)+distance(current,b)+1e-9)
        continue;
      if (!valid(a,point) || !valid(point,b)) continue;
      if (cost && cost(a,point)+cost(point,b)>cost(a,current)+cost(current,b)+1e-8) continue;
      result[i]=point;
      dirty[i-1]=dirty[i]=dirty[i+1]=true;
      moved=true;
    }
    if (!moved) break;
  }
  return cancel && cancel() ? std::vector<PointPose>{} : result;
}

}  // namespace global_planner
