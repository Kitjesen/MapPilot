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
                                  const RouteCancelCheck &cancel) {
  if (cancel && cancel()) return {};
  auto result=route;
  if (result.size()<2) return result;
  if (!checkRoute(result,valid,cancel)) return {};
  for (std::size_t first=0;first+2<result.size();++first) {
    for (std::size_t last=result.size()-1;last>first+1;--last) {
      if (cancel && cancel()) return {};
      const auto &a=result[first], &b=result[last];
      bool supported=true;
      if ((first==0 && height) || std::abs(a.z-b.z)>kLevelTolerance) {
        supported=static_cast<bool>(height);
        const int samples=std::max(1,static_cast<int>(std::ceil(distance(a,b)/.1)));
        for(int i=1;i<samples && supported;++i) {
          if(cancel && cancel())return {};
          const double t=static_cast<double>(i)/samples;
          supported=height({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z)}).has_value();
        }
      }
      if (!supported) continue;
      if (valid(a,b)) {
        result.erase(result.begin()+first+1,result.begin()+last);
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
    const auto length=[](const auto &points) {
      double total=0;
      for (std::size_t i=1;i<points.size();++i) total+=distance(points[i-1],points[i]);
      return total;
    };
    if (length(dense)<=length(result)+1e-8 && checkRoute(dense,valid,cancel))
      result=std::move(dense);
  }
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
      result[i]=point;
      dirty[i-1]=dirty[i]=dirty[i+1]=true;
      moved=true;
    }
    if (!moved) break;
  }
  return cancel && cancel() ? std::vector<PointPose>{} : result;
}

}  // namespace global_planner
