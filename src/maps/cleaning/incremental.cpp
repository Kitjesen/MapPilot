#include "cleaning/incremental.hpp"
#include "cleaning/surface.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace lingtu::maps::cleaning {
namespace {
bool Finite(Point p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
double Dist2(Point a, Point b) {
  const double x = a.x-b.x, y = a.y-b.y, z = a.z-b.z;
  return x*x+y*y+z*z;
}
}
std::size_t IncrementalCleaner::Hash::operator()(Key k) const {
  return static_cast<std::uint32_t>(k.x)*73856093U ^
         static_cast<std::uint32_t>(k.y)*19349663U ^ static_cast<std::uint32_t>(k.z)*83492791U;
}
IncrementalCleaner::Key IncrementalCleaner::ToKey(Point p) const {
  return {static_cast<int>(std::floor(p.x/config_.resolution_m)),
          static_cast<int>(std::floor(p.y/config_.resolution_m)),
          static_cast<int>(std::floor(p.z/config_.resolution_m))};
}
IncrementalCleaner::IncrementalCleaner(Config config) : config_(config) {
  if (!std::isfinite(config.resolution_m) || config.resolution_m < .02F ||
      !std::isfinite(config.ray_tolerance_m) || config.ray_tolerance_m <= 0 ||
      !std::isfinite(config.endpoint_margin_m) || config.endpoint_margin_m < config.ray_tolerance_m ||
      !std::isfinite(config.confirm_radius_m) || config.confirm_radius_m <= 0 ||
      config.confirm_radius_m > config.endpoint_margin_m ||
      !std::isfinite(config.free_neighborhood_m) || config.free_neighborhood_m < 0 ||
      config.free_neighborhood_m > config.ray_tolerance_m ||
      !std::isfinite(config.max_ray_length_m) || config.max_ray_length_m <= 0 ||
      config.min_free_frames < 2 || config.max_cells == 0)
    throw std::invalid_argument("invalid incremental cleaning geometry");
}
void IncrementalCleaner::Reset() {
  cells_.clear(); frame_ = removed_ = rejected_ = 0; ground_.Reset();
}
void IncrementalCleaner::Observe(std::span<const Point> points, Point origin,
                                std::span<const std::uint8_t> ground_mask) {
  if (!Finite(origin)) throw std::invalid_argument("nonfinite cleaning sensor origin");
  if (!ground_mask.empty() && ground_mask.size() != points.size())
    throw std::invalid_argument("cleaning ground mask size mismatch");
  ++frame_;
  ground::GroundSegmentation segmented;
  if (ground_mask.empty()) {
    std::vector<ground::Point3f> local;
    local.reserve(points.size());
    for (auto p : points) local.push_back({p.x-origin.x, p.y-origin.y, p.z-origin.z});
    segmented = ground_.Segment(local);
    ground_mask = segmented.ground_mask;
  }
  std::unordered_set<Key, Hash> hit_cells;
  std::vector<Point> rays;
  const int confirm_neighbors = static_cast<int>(std::ceil(config_.confirm_radius_m/config_.resolution_m));
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto p = points[i];
    if (!Finite(p)) continue;
    const auto key = ToKey(p);
    if (hit_cells.insert(key).second) rays.push_back(p);
    // Confirm before quantized insertion, including neighboring geometry cells.
    for (int x=-confirm_neighbors;x<=confirm_neighbors;++x)
      for (int y=-confirm_neighbors;y<=confirm_neighbors;++y)
        for (int z=-confirm_neighbors;z<=confirm_neighbors;++z) {
          auto found=cells_.find({key.x+x,key.y+y,key.z+z});
          if(found==cells_.end() || Dist2(found->second.point,p)>
              config_.confirm_radius_m*config_.confirm_radius_m) continue;
          auto& r=found->second;
          if(r.hit_stamp!=frame_) {
            r.hits=std::min(2U,r.hits+1); r.hit_stamp=frame_;
            if (!ground_mask.empty() && ground_mask[i]) r.ground_hits=std::min(2U,r.ground_hits+1);
          }
          r.free_frames=0; r.removed=false;
        }
    auto found=cells_.find(key);
    if(found==cells_.end()) {
      if(cells_.size()>=config_.max_cells) { ++rejected_; continue; }
      cells_.emplace(key,Record{p,frame_,0,1,
          static_cast<std::uint32_t>(!ground_mask.empty() && ground_mask[i]),0,0,false});
    } else if (found->second.hit_stamp != frame_) {
      // This coarse geometry cell now has a real endpoint even if the old
      // representative is farther than the exact confirmation radius.
      found->second = Record{p,frame_,0,1,
          static_cast<std::uint32_t>(!ground_mask.empty() && ground_mask[i]),0,0,false};
    }
  }
  // Angular bins only accelerate lookup. Every vote still checks exact distance
  // to measured rays at all seven samples; no angular cell authorizes deletion.
  constexpr int azimuth_bins=360, elevation_bins=180;
  constexpr double pi=3.14159265358979323846, angle_step=pi/180.;
  struct Ray { double x,y,z,length,end; };
  const auto angular_key=[](double x,double y,double z) {
    const int a=std::clamp(static_cast<int>((std::atan2(y,x)+pi)/angle_step),0,azimuth_bins-1);
    const int e=std::clamp(static_cast<int>((std::atan2(z,std::hypot(x,y))+pi/2)/angle_step),0,elevation_bins-1);
    return std::array<int,2>{a,e};
  };
  const auto wrap=[](int a) { return (a%azimuth_bins+azimuth_bins)%azimuth_bins; };
  const auto radius=[](double distance,double width,int limit) {
    return std::min(limit,1+static_cast<int>(std::ceil(std::asin(std::min(1.,width/std::max(distance,1e-9)))/angle_step)));
  };
  std::vector<std::vector<Ray>> bins(azimuth_bins*elevation_bins);
  std::vector<double> blockers(azimuth_bins*elevation_bins,std::numeric_limits<double>::infinity());
  std::vector<double> max_ends(azimuth_bins*elevation_bins,0.);
  for(auto endpoint:rays) {
    const double x=endpoint.x-origin.x,y=endpoint.y-origin.y,z=endpoint.z-origin.z;
    const double length=std::sqrt(x*x+y*y+z*z);
    if(length<1e-9) continue;
    const auto key=angular_key(x,y,z);
    const double hit_width=std::sqrt(3.)*config_.resolution_m;
    const int da=radius(std::hypot(x,y),hit_width,azimuth_bins/2);
    const int de=radius(length,hit_width,elevation_bins/2);
    // Inflated current hits can only withhold free evidence from nearby rays.
    for(int e=std::max(0,key[1]-de);e<=std::min(elevation_bins-1,key[1]+de);++e)
      for(int a=key[0]-da;a<=key[0]+da;++a) {
        auto& blocker=blockers[e*azimuth_bins+wrap(a)];
        blocker=std::min(blocker,length);
      }
    bins[key[1]*azimuth_bins+key[0]].push_back({x/length,y/length,z/length,length,0});
  }
  for(std::size_t i=0;i<bins.size();++i)
    for(auto& ray:bins[i]) {
      ray.end=std::min(static_cast<double>(config_.max_ray_length_m),
          std::min(ray.length,blockers[i])-config_.endpoint_margin_m);
      max_ends[i]=std::max(max_ends[i],ray.end);
    }
  std::vector<Key> touched;
  const double tube=config_.ray_tolerance_m;
  for(auto& [cell_key,r]:cells_) {
    if(r.hit_stamp==frame_ || r.removed) continue;
    const double x=r.point.x-origin.x,y=r.point.y-origin.y,z=r.point.z-origin.z;
    const double length=std::sqrt(x*x+y*y+z*z);
    if(length<=config_.endpoint_margin_m || length>config_.max_ray_length_m+config_.free_neighborhood_m) continue;
    const auto key=angular_key(x,y,z);
    const double width=tube+config_.free_neighborhood_m;
    const int da=radius(std::hypot(x,y),width,azimuth_bins/2);
    const int de=radius(length,width,elevation_bins/2);
    const double min_along=std::sqrt(std::max(0.,
        (length-config_.free_neighborhood_m)*(length-config_.free_neighborhood_m)-tube*tube));
    std::uint8_t coverage=0;
    for(int e=std::max(0,key[1]-de);e<=std::min(elevation_bins-1,key[1]+de) && coverage!=127;++e)
      for(int a=key[0]-da;a<=key[0]+da && coverage!=127;++a) {
        const auto bin=e*azimuth_bins+wrap(a);
        if(max_ends[bin]<min_along) continue;
        for(const auto& ray:bins[bin]) {
          if(ray.end<min_along) continue;
          for(int sample=0;sample<7;++sample) {
            if(coverage & (1U<<sample)) continue;
            double v[3]={x,y,z};
            if(sample>0) v[(sample-1)/2]+=(sample%2?1.:-1.)*config_.free_neighborhood_m;
            const double along=v[0]*ray.x+v[1]*ray.y+v[2]*ray.z;
            const double cross2=std::max(0.,v[0]*v[0]+v[1]*v[1]+v[2]*v[2]-along*along);
            if(along>config_.endpoint_margin_m && along<ray.end && cross2<=tube*tube)
              coverage|=static_cast<std::uint8_t>(1U<<sample);
          }
          if(coverage==127) break;
        }
      }
    if(coverage==127) {r.free_stamp=frame_;r.coverage=coverage;touched.push_back(cell_key);}
  }
  // Complete all votes before checking support, so disappearing clusters cannot
  // protect one another merely because of hash/iteration order.
  for(auto key:touched) {
    auto& r=cells_.at(key);
    if(r.coverage==127 && r.hit_stamp!=frame_)
      r.free_frames=std::min(config_.min_free_frames,r.free_frames+1);
  }
  for(auto key:touched) {
    auto& r=cells_.at(key);
    if(r.free_frames>=config_.min_free_frames && !Supported(r)) { r.removed=true; ++removed_; }
  }
}
bool IncrementalCleaner::Supported(const Record& record) const {
  const auto p=record.point; const auto key=ToKey(p);
  const int n=static_cast<int>(std::ceil(.5F/config_.resolution_m));
  std::vector<Point> neighbors;
  for(int x=-n;x<=n;++x) for(int y=-n;y<=n;++y) for(int z=-n;z<=n;++z) {
    const auto it=cells_.find({key.x+x,key.y+y,key.z+z});
    if(it==cells_.end()) continue;
    const auto& r=it->second;
    if(&r!=&record && !r.removed && r.hits>=2 && r.free_frames==0 && Dist2(p,r.point)<=.25)
      neighbors.push_back(r.point);
  }
  if(neighbors.size()<6) return false;
  const auto anchor=*std::min_element(neighbors.begin(),neighbors.end(),[&](auto a,auto b){return Dist2(p,a)<Dist2(p,b);});
  const auto far=*std::max_element(neighbors.begin(),neighbors.end(),[&](auto a,auto b){return Dist2(anchor,a)<Dist2(anchor,b);});
  const double edge[3]={far.x-anchor.x,far.y-anchor.y,far.z-anchor.z};
  const double edge2=Dist2(anchor,far); if(edge2<.04) return false;
  double normal[3]{},area2=0;
  for(auto q:neighbors) {
    const double v[3]={q.x-anchor.x,q.y-anchor.y,q.z-anchor.z};
    const double cross[3]={edge[1]*v[2]-edge[2]*v[1],edge[2]*v[0]-edge[0]*v[2],edge[0]*v[1]-edge[1]*v[0]};
    const double a=cross[0]*cross[0]+cross[1]*cross[1]+cross[2]*cross[2];
    if(a>area2) {area2=a;std::copy(cross,cross+3,normal);}
  }
  if(area2<edge2*.01) return false;
  for(auto& v:normal) v/=std::sqrt(area2);
  auto distance=[&](Point q){return std::abs((q.x-anchor.x)*normal[0]+(q.y-anchor.y)*normal[1]+(q.z-anchor.z)*normal[2]);};
  if(distance(p)>.04) return HasStableLocalPlane(p,std::span<const Point>(neighbors));
  // Ground labels require repeated votes; other planes still protect walls and stairs.
  if(record.ground_hits>0 && record.ground_hits<2) return false;
  return std::count_if(neighbors.begin(),neighbors.end(),[&](auto q){return distance(q)<=.04;})*5 >=
      static_cast<std::ptrdiff_t>(neighbors.size()*4) || HasStableLocalPlane(p,std::span<const Point>(neighbors));
}
std::vector<Point> IncrementalCleaner::Snapshot() const {
  std::vector<Point> points; points.reserve(cells_.size());
  for(const auto& [key,r]:cells_) if(!r.removed) points.push_back(r.point);
  return points;
}
std::size_t IncrementalCleaner::Count() const {
  return std::count_if(cells_.begin(),cells_.end(),[](const auto& item){return !item.second.removed;});
}
}
