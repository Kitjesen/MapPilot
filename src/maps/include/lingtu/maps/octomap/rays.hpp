#pragma once
#include "lingtu/maps/io/scans.hpp"
#include "lingtu/maps/io/pcd.hpp"
#include "lingtu/maps/layers/ground_surface.hpp"
#include <octomap/OcTree.h>
#include <array>
#include <stdexcept>
#include <cmath>
#include <cstdint>
#include <map>
#include <unordered_map>
#include <vector>
#include <utility>

namespace lingtu::maps {

inline std::size_t OccupiedVoxelCount(const octomap::OcTree& tree) {
  std::size_t count=0;
  for (auto it=tree.begin_leafs(),end=tree.end_leafs();it!=end;++it) {
    if (!tree.isNodeOccupied(*it)) continue;
    const auto side=static_cast<std::size_t>(std::llround(it.getSize()/tree.getResolution()));
    count+=side*side*side;
  }
  return count;
}

// Near its endpoint a ray grazing a floor stays within one voxel of the
// surface: from a sensor 0.4 m up, a 5 cm cell for the last metre of a ray that
// lands 8 m away. OctoMap marks those surface cells free although the ray never
// passed below the surface.
inline constexpr double kGrazingGuardM = 1.0;
// Match prune's metric endpoint confirmation across voxel boundaries.
inline constexpr double kRetainedEndpointMatchM = 0.05;

struct SavedRayOctomapStats {
  std::size_t valid_endpoints{0};
  std::size_t retained_endpoints{0};
  std::size_t dropped_endpoints{0};
  std::size_t free_updates{0};
  std::size_t hit_updates{0};
  std::size_t guarded_miss_suppressions{0};
  std::size_t surface_guarded_miss_suppressions{0};

  [[nodiscard]] std::size_t inserted_points() const noexcept {
    return retained_endpoints;
  }
};

namespace saved_ray_detail {
inline constexpr double kSurfaceRadiusM = 0.30;
inline constexpr double kSurfaceToleranceM = 0.04;
using Bucket = std::array<std::int64_t, 3>;

inline Bucket MatchBucket(const PointXyz& p) {
  return {static_cast<std::int64_t>(std::floor(p.x / kRetainedEndpointMatchM)),
          static_cast<std::int64_t>(std::floor(p.y / kRetainedEndpointMatchM)),
          static_cast<std::int64_t>(std::floor(p.z / kRetainedEndpointMatchM))};
}

inline Bucket SurfaceBucket(const PointXyz& p) {
  return {static_cast<std::int64_t>(std::floor(p.x / kSurfaceRadiusM)),
          static_cast<std::int64_t>(std::floor(p.y / kSurfaceRadiusM)),
          static_cast<std::int64_t>(std::floor(p.z / kSurfaceRadiusM))};
}

class RetainedPointIndex {
 public:
  explicit RetainedPointIndex(const std::vector<PointXyz>& points,
                              const octomap::OcTree& tree) : points_(points) {
    entries_.reserve(points.size());
    for (std::size_t i=0;i<points.size();++i) {
      const auto& p=points[i];
      octomap::OcTreeKey key;
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
          tree.coordToKeyChecked(p.x,p.y,p.z,key)) entries_.push_back({MatchBucket(p),i});
    }
    std::sort(entries_.begin(),entries_.end(),[](const Entry& a,const Entry& b) {
      return a.bucket<b.bucket || (a.bucket==b.bucket && a.index<b.index);
    });
    for (std::size_t i=0;i<entries_.size();++i)
      if (i==0 || entries_[i].bucket!=entries_[i-1].bucket)
        surface_order_.push_back(entries_[i].index);
    std::sort(surface_order_.begin(),surface_order_.end(),[&](std::size_t a,std::size_t b) {
      const auto ka=SurfaceBucket(points_[a]),kb=SurfaceBucket(points_[b]);
      return ka<kb || (ka==kb && a<b);
    });
  }

  bool Matches(const octomap::point3d& end) const {
    const auto center=MatchBucket({end.x(),end.y(),end.z()});
    for (int dx=-1;dx<=1;++dx) for (int dy=-1;dy<=1;++dy) for (int dz=-1;dz<=1;++dz) {
      const Bucket bucket{center[0]+dx,center[1]+dy,center[2]+dz};
      auto it=std::lower_bound(entries_.begin(),entries_.end(),bucket,
        [](const Entry& entry,const Bucket& key){return entry.bucket<key;});
      for (;it!=entries_.end() && it->bucket==bucket;++it) {
        const auto& p=points_[it->index];
        const double x=static_cast<double>(p.x)-end.x();
        const double y=static_cast<double>(p.y)-end.y();
        const double z=static_cast<double>(p.z)-end.z();
        if (x*x+y*y+z*z<=kRetainedEndpointMatchM*kRetainedEndpointMatchM) return true;
      }
    }
    return false;
  }

  template<class Visitor>
  void VisitSurfaceNeighbors(const PointXyz& anchor,Visitor visit) const {
    const auto center=SurfaceBucket(anchor);
    for (int dx=-1;dx<=1;++dx) for (int dy=-1;dy<=1;++dy) for (int dz=-1;dz<=1;++dz) {
      const Bucket bucket{center[0]+dx,center[1]+dy,center[2]+dz};
      auto it=std::lower_bound(surface_order_.begin(),surface_order_.end(),bucket,
        [&](std::size_t index,const Bucket& key){return SurfaceBucket(points_[index])<key;});
      for (;it!=surface_order_.end() && SurfaceBucket(points_[*it])==bucket;++it)
        visit(points_[*it]);
    }
  }

 private:
  struct Entry { Bucket bucket; std::size_t index; };
  const std::vector<PointXyz>& points_;
  std::vector<Entry> entries_;
  std::vector<std::size_t> surface_order_;
};

struct MeasuredSurface {
  PointXyz anchor;
  layers::ground_detail::Plane plane;
  bool checked{false};
};

inline void FitSurface(MeasuredSurface& surface,const RetainedPointIndex& index) {
  surface.checked = true;
  // One observed XY column gets one vote. Choose the height closest to this
  // measured hit so another floor at the same XY is not averaged into it.
  std::map<std::array<std::int64_t, 2>, PointXyz> columns;
  index.VisitSurfaceNeighbors(surface.anchor,[&](const PointXyz& p) {
          const double x=p.x-surface.anchor.x, y=p.y-surface.anchor.y, z=p.z-surface.anchor.z;
          if (x*x+y*y+z*z > kSurfaceRadiusM*kSurfaceRadiusM) return;
          const std::array<std::int64_t, 2> column{
            static_cast<std::int64_t>(std::floor(p.x/kRetainedEndpointMatchM)),
            static_cast<std::int64_t>(std::floor(p.y/kRetainedEndpointMatchM))};
          auto previous = columns.find(column);
          if (previous == columns.end()) columns.emplace(column,p);
          else if (std::abs(z) < std::abs(previous->second.z-surface.anchor.z))
            previous->second=p;
  });
  if (columns.size() < 6U) return;
  std::vector<layers::ground_detail::Point> points;
  points.reserve(columns.size());
  for (const auto& entry : columns) {
    const auto& p=entry.second;
    const double x=p.x-surface.anchor.x, y=p.y-surface.anchor.y;
    points.push_back({x,y,p.z-surface.anchor.z,0,0,{}});
  }
  // Remove returns outside the existing 4 cm surface band, then refit only
  // that height layer. Every iteration removes evidence; it cannot grow a
  // plane into a different level or turn a line into an area patch.
  layers::ground_detail::Plane plane;
  while (points.size()>=6U) {
    plane=layers::ground_detail::Fit(points);
    if (!plane.valid || std::abs(plane.z)>kSurfaceToleranceM) return;
    const double normal_length=std::sqrt(1+plane.x*plane.x+plane.y*plane.y);
    std::vector<layers::ground_detail::Point> inliers;
    for (const auto& p : points)
      if (std::abs(p.z-plane.At(p.x,p.y))/normal_length<=kSurfaceToleranceM)
        inliers.push_back(p);
    if (inliers.size()==points.size()) break;
    points=std::move(inliers);
  }
  if (points.size()<6U) return;
  double mx=0,my=0;
  for (const auto& p : points) {mx+=p.x;my+=p.y;}
  mx/=points.size(); my/=points.size();
  double xx=0,yy=0,xy=0;
  for (const auto& p : points) {
    xx+=(p.x-mx)*(p.x-mx); yy+=(p.y-my)*(p.y-my); xy+=(p.x-mx)*(p.y-my);
  }
  // A line of returns cannot establish the orientation of a surface patch.
  if ((xx+yy-std::hypot(xx-yy,2*xy))/(2*points.size()) < .001) return;
  surface.plane=plane;
}

inline bool RayMissesSurface(const octomap::OcTree& tree, const octomap::OcTreeKey& key,
                             const octomap::point3d& origin, const octomap::point3d& end,
                             const MeasuredSurface& surface) {
  if (!surface.plane.valid) return false;
  const auto center=tree.keyToCoord(key);
  const double half=.5*tree.getResolution();
  double enter=0,leave=1;
  for (unsigned axis=0;axis<3;++axis) {
    const double direction=static_cast<double>(end(axis))-origin(axis);
    const double low=static_cast<double>(center(axis))-half;
    const double high=static_cast<double>(center(axis))+half;
    if (std::abs(direction)<1e-12) {
      if (origin(axis)<low || origin(axis)>high) return false;
      continue;
    }
    double a=(low-origin(axis))/direction,b=(high-origin(axis))/direction;
    if(a>b)std::swap(a,b);
    enter=std::max(enter,a); leave=std::min(leave,b);
    if(enter>leave)return false;
  }
  const auto distance=[&](double t) {
    const double x=origin.x()+t*(end.x()-origin.x())-surface.anchor.x;
    const double y=origin.y()+t*(end.y()-origin.y())-surface.anchor.y;
    const double z=origin.z()+t*(end.z()-origin.z())-surface.anchor.z;
    return (z-surface.plane.At(x,y))/
      std::sqrt(1+surface.plane.x*surface.plane.x+surface.plane.y*surface.plane.y);
  };
  const auto within_patch=[&](double t) {
    const double x=origin.x()+t*(end.x()-origin.x())-surface.anchor.x;
    const double y=origin.y()+t*(end.y()-origin.y())-surface.anchor.y;
    const double z=origin.z()+t*(end.z()-origin.z())-surface.anchor.z;
    return x*x+y*y+z*z<=kSurfaceRadiusM*kSurfaceRadiusM;
  };
  if (!within_patch(enter) || !within_patch(leave)) return false;
  const double a=distance(enter),b=distance(leave);
  constexpr double margin=kSurfaceToleranceM+1e-5;
  return (a>margin && b>margin) || (a < -margin && b < -margin);
}
}  // namespace saved_ray_detail

// Replays saved scans into `tree` from their measured sensor origins. The
// cleaned map.pcd decides which endpoints remain static. A removed endpoint
// still contributes the measured free prefix before it, but contributes no
// hit and does not infer anything at or beyond the endpoint. Near every raw
// endpoint, misses against retained same-layer surface cells are suppressed
// using a metric distance. At an already measured retained hit, a local
// surface can also distinguish a long grazing ray from a real crossing;
// ambiguous or unmeasured cells retain ordinary free updates.
// Updates are de-duplicated per scan and measured hits win over misses.
inline SavedRayOctomapStats PopulateSavedRayOctomap(
    octomap::OcTree& tree, const std::filesystem::path& directory,
    const std::vector<PointXyz>& retained_points,
    const std::function<bool()>& cancelled = {}) {
  if (retained_points.empty())
    throw std::runtime_error("saved ray build requires retained map.pcd geometry");
  octomap::KeySet retained_keys;
  for (const auto& point : retained_points) {
    octomap::OcTreeKey key;
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
        tree.coordToKeyChecked(point.x, point.y, point.z, key)) {
      retained_keys.insert(key);
    }
  }
  if (retained_keys.empty())
    throw std::runtime_error("saved ray build retained map.pcd has no valid OctoMap keys");
  const saved_ray_detail::RetainedPointIndex retained_index(retained_points,tree);
  octomap::KeySet guarded;
  for (const auto& key : retained_keys)
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        guarded.insert(octomap::OcTreeKey(key[0] + dx, key[1] + dy, key[2]));

  SavedRayOctomapStats stats;
  std::unordered_map<octomap::OcTreeKey, saved_ray_detail::MeasuredSurface,
      octomap::OcTreeKey::KeyHash> measured_surfaces;
  octomap::KeyRay ray;
  VisitSavedScans(directory, [&](const SavedScan& scan) {
    if (cancelled && cancelled()) throw std::runtime_error("saved ray build cancelled");
    const octomap::point3d origin(scan.origin[0], scan.origin[1], scan.origin[2]);
    // One scan updates each cell once, and its hits win over its misses, as in
    // OcTree::insertPointCloud.
    octomap::KeySet free_cells, occupied_cells;
    for (std::size_t i = 0; i + 2 < scan.xyz.size(); i += 3) {
      const octomap::point3d end(scan.xyz[i], scan.xyz[i + 1], scan.xyz[i + 2]);
      octomap::OcTreeKey key;
      if (!std::isfinite(end.x()) || !std::isfinite(end.y()) || !std::isfinite(end.z()) ||
          !tree.coordToKeyChecked(end, key))
        continue;
      ++stats.valid_endpoints;
      bool retained_endpoint = retained_keys.count(key) != 0U;
      if (!retained_endpoint) {
        // A sampled surface point may be in the adjacent voxel. Confirm the
        // measured endpoint against its position, then keep the real hit key.
        retained_endpoint=retained_index.Matches(end);
      }
      if (!retained_endpoint) {
        ++stats.dropped_endpoints;
      } else {
        ++stats.retained_endpoints;
        occupied_cells.insert(key);
        measured_surfaces.try_emplace(key,
          saved_ray_detail::MeasuredSurface{{end.x(),end.y(),end.z()},{},false});
      }
      if (!tree.computeRayKeys(origin, end, ray)) continue;
      for (const auto& cell : ray) {
        if (guarded.count(cell) != 0U) {
          const auto delta = tree.keyToCoord(cell) - end;
          if (delta.norm_sq() <= kGrazingGuardM * kGrazingGuardM) {
            ++stats.guarded_miss_suppressions;
            continue;
          }
        }
        const auto measured=measured_surfaces.find(cell);
        if (measured!=measured_surfaces.end()) {
          auto& surface=measured->second;
          if (!surface.checked) saved_ray_detail::FitSurface(surface,retained_index);
          if (saved_ray_detail::RayMissesSurface(tree,cell,origin,end,surface)) {
            ++stats.surface_guarded_miss_suppressions;
            continue;
          }
        }
        free_cells.insert(cell);
      }
    }
    for (const auto& cell : free_cells)
      if (occupied_cells.count(cell) == 0U) {
        tree.updateNode(cell, false, true);
        ++stats.free_updates;
      }
    for (const auto& cell : occupied_cells) {
      tree.updateNode(cell, true, true);
      ++stats.hit_updates;
    }
  });
  if (stats.retained_endpoints == 0U)
    throw std::runtime_error("saved ray build retained map.pcd matched no saved scan endpoints");
  tree.updateInnerOccupancy();
  return stats;
}

inline SavedRayOctomapStats PopulateSavedRayOctomap(
    octomap::OcTree& tree, const std::filesystem::path& directory,
    const std::function<bool()>& cancelled = {}) {
  const auto retained = LoadPcdXyz(directory / "map.pcd");
  if (!retained.ok)
    throw std::runtime_error("saved ray build requires retained map.pcd geometry");
  return PopulateSavedRayOctomap(tree, directory, retained.points, cancelled);
}

}  // namespace lingtu::maps
