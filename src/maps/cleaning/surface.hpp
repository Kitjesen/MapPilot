#pragma once
#include <algorithm>
#include <cmath>
#include <span>
#include <vector>
namespace lingtu::maps::cleaning {
// Test distinct local planes independently: another tread above the same XY
// must not turn a valid lower tread into an outlier of one blended plane.
template<class Point>
bool HasStableLocalPlane(const Point& point, std::span<const Point> neighbors) {
  if(neighbors.size()<6) return false;
  const auto distance2=[&](const Point& q) {
    const double x=q.x-point.x,y=q.y-point.y,z=q.z-point.z;
    return x*x+y*y+z*z;
  };
  std::vector<Point> anchors(neighbors.begin(),neighbors.end());
  const auto count=std::min<std::size_t>(12,anchors.size());
  std::partial_sort(anchors.begin(),anchors.begin()+count,anchors.end(),
      [&](const auto& a,const auto& b){return distance2(a)<distance2(b);});
  for(std::size_t a=0;a<count;++a) for(std::size_t b=a+1;b<count;++b) {
    const double u[3]={anchors[a].x-point.x,anchors[a].y-point.y,anchors[a].z-point.z};
    const double v[3]={anchors[b].x-point.x,anchors[b].y-point.y,anchors[b].z-point.z};
    double normal[3]={u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]};
    const double norm2=normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2];
    if(norm2<.0001) continue;
    for(auto& n:normal)n/=std::sqrt(norm2);
    std::size_t inliers=0;
    double off_axis2=0,extent2=0;
    const double edge2=u[0]*u[0]+u[1]*u[1]+u[2]*u[2];
    if(edge2<.0025)continue;
    for(const auto& q:neighbors) {
      const double d[3]={q.x-point.x,q.y-point.y,q.z-point.z};
      if(std::abs(d[0]*normal[0]+d[1]*normal[1]+d[2]*normal[2])>.04) continue;
      ++inliers;
      const double along=d[0]*u[0]+d[1]*u[1]+d[2]*u[2];
      const double length2=d[0]*d[0]+d[1]*d[1]+d[2]*d[2];
      off_axis2=std::max(off_axis2,length2-along*along/edge2);
      extent2=std::max(extent2,length2);
    }
    if(inliers>=6 && extent2>=.04 && off_axis2>=.01) return true;
  }
  return false;
}
}
