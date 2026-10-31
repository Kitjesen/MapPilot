#include "collision/terrain.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>

namespace lingtu::nav::collision {
namespace {

double dot(const Vec3 &a, const Vec3 &b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

bool finite(const Vec3 &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

template <class Callback>
void corners(const Aabb &box, Callback &&callback) {
  const double low[3] = {box.center.x-box.half_size.x, box.center.y-box.half_size.y,
                         box.center.z-box.half_size.z};
  const double high[3] = {box.center.x+box.half_size.x, box.center.y+box.half_size.y,
                          box.center.z+box.half_size.z};
  for (int x=0;x<(low[0]==high[0]?1:2);++x)
    for (int y=0;y<(low[1]==high[1]?1:2);++y)
      for (int z=0;z<(low[2]==high[2]?1:2);++z)
        callback(Vec3{x?high[0]:low[0],y?high[1]:low[1],z?high[2]:low[2]});
}

struct Column {
  int x, y;
  std::vector<Vec3> points;
};

// Local coordinates and equal column weight follow ground_detail::Fit; keeping
// each column's original hits also retains the residual hidden by its mean.
std::optional<TerrainSurface> fit(const std::vector<Column> &columns, const Vec3 &origin,
                                 double seed_z, double resolution, double tolerance) {
  std::vector<Vec3> means;
  std::vector<std::vector<Vec3>> layers;
  for (const auto &column : columns) {
    const auto nearest = std::min_element(column.points.begin(), column.points.end(),
        [seed_z](const Vec3 &a, const Vec3 &b) {
          return std::abs(a.z-seed_z) < std::abs(b.z-seed_z);
        });
    std::vector<Vec3> layer;
    Vec3 mean{};
    for (const auto &point : column.points) {
      if (std::abs(point.z-nearest->z) > resolution*.5) continue;
      const Vec3 local{point.x-origin.x, point.y-origin.y, point.z-origin.z};
      layer.push_back(local);
      mean.x += local.x; mean.y += local.y; mean.z += local.z;
    }
    const double n = static_cast<double>(layer.size());
    means.push_back({mean.x/n, mean.y/n, mean.z/n});
    layers.push_back(std::move(layer));
  }
  if (means.size() < 3U) return {};
  Vec3 mean{};
  for (const auto &p : means) { mean.x += p.x; mean.y += p.y; mean.z += p.z; }
  const double count = static_cast<double>(means.size());
  mean.x /= count; mean.y /= count; mean.z /= count;
  double xx=0, yy=0, xy=0, xz=0, yz=0;
  for (const auto &p : means) {
    const double x=p.x-mean.x, y=p.y-mean.y, z=p.z-mean.z;
    xx += x*x; yy += y*y; xy += x*y; xz += x*z; yz += y*z;
  }
  const double determinant=xx*yy-xy*xy;
  if (xx+yy <= 0 || determinant <= 1e-10*(xx+yy)*(xx+yy)) return {};
  const double a=(xz*yy-yz*xy)/determinant;
  const double b=(yz*xx-xz*xy)/determinant;
  const double intercept=mean.z-a*mean.x-b*mean.y;
  const double norm=std::hypot(std::hypot(a,b),1.0);
  double squared_residual=0;
  double max_positive_residual=0;
  Vec3 low{std::numeric_limits<double>::infinity(),
           std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity()};
  Vec3 high{-low.x,-low.y,-low.z};
  for (const auto &layer : layers) {
    double column_residual=0;
    for (const auto &p : layer) {
      const double residual=(p.z-a*p.x-b*p.y-intercept)/norm;
      if (std::abs(residual)>2*tolerance) return {};
      max_positive_residual=std::max(max_positive_residual,residual);
      column_residual += residual*residual;
      low.x=std::min(low.x,p.x+origin.x); high.x=std::max(high.x,p.x+origin.x);
      low.y=std::min(low.y,p.y+origin.y); high.y=std::max(high.y,p.y+origin.y);
      low.z=std::min(low.z,p.z+origin.z); high.z=std::max(high.z,p.z+origin.z);
    }
    squared_residual += column_residual/layer.size();
  }
  const double rms=std::sqrt(squared_residual/count);
  if (rms>tolerance) return {};
  return TerrainSurface{{-a/norm,-b/norm,1.0/norm},
      (a*origin.x+b*origin.y-origin.z-intercept)/norm,
      {{(low.x+high.x)*.5,(low.y+high.y)*.5,(low.z+high.z)*.5},
       {(high.x-low.x+resolution)*.5,(high.y-low.y+resolution)*.5,
        (high.z-low.z)*.5}},rms,max_positive_residual};
}

}  // namespace

double TerrainSurface::heightAt(double x, double y) const {
  return -(normal.x*x+normal.y*y+offset_m)/normal.z;
}

std::size_t Terrain::CellHash::operator()(const Cell &cell) const {
  std::size_t value=std::hash<int>{}(cell.x);
  value ^= std::hash<int>{}(cell.y)+0x9e3779b9U+(value<<6)+(value>>2);
  value ^= std::hash<int>{}(cell.z)+0x9e3779b9U+(value<<6)+(value>>2);
  return value;
}

Terrain::Cell Terrain::cell(const Vec3 &point) const {
  // Retrieval buckets are coarser than evidence columns; raw coordinates and
  // the support/fit resolution remain unchanged.
  const double bucket_resolution=5*resolution_m_;
  return {static_cast<int>(std::floor(point.x/bucket_resolution)),
          static_cast<int>(std::floor(point.y/bucket_resolution)),
          static_cast<int>(std::floor(point.z/bucket_resolution))};
}

Terrain::Cell Terrain::hitCell(const Vec3 &point) const {
  return {static_cast<int>(std::floor(point.x/resolution_m_)),
          static_cast<int>(std::floor(point.y/resolution_m_)),
          static_cast<int>(std::floor(point.z/resolution_m_))};
}

Terrain::Terrain(std::vector<Vec3> points, double sample_resolution_m)
    : resolution_m_(sample_resolution_m) {
  if (!std::isfinite(resolution_m_) || resolution_m_<=0)
    throw std::invalid_argument("terrain sample resolution must be positive");
  samples_.reserve(points.size());
  for (const auto &point : points) {
    if (!finite(point)) continue;
    cells_[cell(point)].push_back(samples_.size());
    samples_.push_back({point,{}});
  }
}

Terrain::Terrain(std::vector<Aabb> hit_bounds, double sample_resolution_m)
    : resolution_m_(sample_resolution_m), bounded_hits_(true) {
  if (!std::isfinite(resolution_m_) || resolution_m_<=0)
    throw std::invalid_argument("terrain sample resolution must be positive");
  samples_.reserve(hit_bounds.size());
  for (const auto &box : hit_bounds) {
    if (!finite(box.center) || !finite(box.half_size) || box.half_size.x<0 ||
        box.half_size.y<0 || box.half_size.z<0) continue;
    cells_[cell(box.center)].push_back(samples_.size());
    hit_cells_.emplace(hitCell(box.center),samples_.size());
    samples_.push_back(box);
  }
}

bool Terrain::hitBoundsClear(const PreparedPart &part, const Aabb &occupied) const {
  if (!bounded_hits_) return false;
  const Vec3 minimum{occupied.center.x-occupied.half_size.x,
                     occupied.center.y-occupied.half_size.y,
                     occupied.center.z-occupied.half_size.z};
  const Vec3 maximum{occupied.center.x+occupied.half_size.x,
                     occupied.center.y+occupied.half_size.y,
                     occupied.center.z+occupied.half_size.z};
  const double contact_bottom=part.primitive_bounds.center.z-part.primitive_bounds.half_size.z;
  bool observed=false;
  const auto range=hit_cells_.equal_range(hitCell(occupied.center));
  for (auto it=range.first;it!=range.second;++it) {
    const auto &sample=samples_[it->second];
    if (sample.center.x<minimum.x || sample.center.x>=maximum.x ||
        sample.center.y<minimum.y || sample.center.y>=maximum.y ||
        sample.center.z<minimum.z || sample.center.z>=maximum.z) continue;
    observed=true;
    if (part.support_contact &&
        sample.center.z+sample.half_size.z+resolution_m_*1e-9 < contact_bottom) continue;
    if (intersects(part, sample)) return false;
  }
  return observed;
}

std::optional<TerrainSurface> Terrain::surface(
    const Vec3 &expected, double height_tolerance_m, double radius_m) const {
  return fitSurface(expected, height_tolerance_m, radius_m, std::max(.001,resolution_m_*.1));
}

std::optional<TerrainSurface> Terrain::classificationSurface(
    const Vec3 &expected, double height_tolerance_m, double radius_m) const {
  // The measured height noise is distinct from the strict collision-fit residual.
  return fitSurface(expected, height_tolerance_m, radius_m, .05, true);
}

std::optional<TerrainSurface> Terrain::fitSurface(
    const Vec3 &expected, double height_tolerance_m, double radius_m,
    double residual_tolerance_m, bool classification) const {
  if (!finite(expected) || !std::isfinite(height_tolerance_m) || height_tolerance_m<0 ||
      !std::isfinite(radius_m) || radius_m<=0) return {};
  const auto fine_cell=[&](double value) {
    return static_cast<int>(std::floor(value/resolution_m_));
  };
  const std::pair<int,int> center{fine_cell(expected.x),fine_cell(expected.y)};
  const auto low=cell({expected.x-radius_m-resolution_m_,expected.y-radius_m-resolution_m_,
                       expected.z-height_tolerance_m-resolution_m_});
  const auto high=cell({expected.x+radius_m+resolution_m_,expected.y+radius_m+resolution_m_,
                        expected.z+height_tolerance_m+resolution_m_});
  std::map<std::pair<int,int>,std::vector<Vec3>> grouped;
  for (int x=low.x;x<=high.x;++x) for (int y=low.y;y<=high.y;++y)
    for (int z=low.z;z<=high.z;++z) {
      const auto found=cells_.find({x,y,z});
      if (found==cells_.end()) continue;
      for (const auto index : found->second) {
        const auto collect = [&](const Vec3 &point) {
          if (std::hypot(point.x-expected.x,point.y-expected.y)>radius_m ||
              std::abs(point.z-expected.z)>height_tolerance_m) return;
          grouped[{fine_cell(point.x),fine_cell(point.y)}].push_back(point);
        };
        if (classification) collect(samples_[index].center);
        else corners(samples_[index], collect);
      }
    }
  std::vector<Column> columns;
  columns.reserve(grouped.size());
  std::optional<double> seed;
  double seed_distance=std::numeric_limits<double>::infinity();
  double seed_height_error=std::numeric_limits<double>::infinity();
  for (auto &entry : grouped) {
    auto &points=entry.second;
    // Preserve the fine-bin iteration order, including nearest-layer ties.
    std::stable_sort(points.begin(),points.end(),[&](const Vec3 &a,const Vec3 &b) {
      return fine_cell(a.z)<fine_cell(b.z);
    });
    const double distance=std::hypot(static_cast<double>(entry.first.first-center.first),
                                     static_cast<double>(entry.first.second-center.second));
    for (const auto &point : points) {
      const double height_error=std::abs(point.z-expected.z);
      if (distance<seed_distance || (distance==seed_distance && height_error<seed_height_error)) {
        seed=point.z;
        seed_distance=distance;
        seed_height_error=height_error;
      }
    }
    columns.push_back({entry.first.first,entry.first.second,std::move(points)});
  }
  if (!seed) return {};
  auto result=fit(columns,expected,*seed,resolution_m_,residual_tolerance_m);
  if (!result || std::abs(result->heightAt(expected.x,expected.y)-expected.z)>
                     height_tolerance_m) return {};
  return result;
}

bool Terrain::explains(const Aabb &occupied, const TerrainSurface &surface,
                        double *max_positive_residual_m) const {
  if (max_positive_residual_m) *max_positive_residual_m=0;
  const auto &patch=surface.observed_bounds;
  constexpr double tolerance=1e-10;
  if (std::abs(occupied.center.x-patch.center.x)+occupied.half_size.x>
          patch.half_size.x+tolerance ||
      std::abs(occupied.center.y-patch.center.y)+occupied.half_size.y>
          patch.half_size.y+tolerance) return false;
  const Vec3 minimum{occupied.center.x-occupied.half_size.x,
                     occupied.center.y-occupied.half_size.y,
                     occupied.center.z-occupied.half_size.z};
  const Vec3 maximum{occupied.center.x+occupied.half_size.x,
                     occupied.center.y+occupied.half_size.y,
                     occupied.center.z+occupied.half_size.z};
  const auto low=cell({minimum.x-resolution_m_,minimum.y-resolution_m_,minimum.z-resolution_m_});
  const auto high=cell({maximum.x+resolution_m_,maximum.y+resolution_m_,maximum.z+resolution_m_});
  const double residual_tolerance=std::max(.001,resolution_m_*.1);
  bool observed=false;
  double maximum_positive_residual=0;
  for (int x=low.x;x<=high.x;++x) for (int y=low.y;y<=high.y;++y)
    for (int z=low.z;z<=high.z;++z) {
      const auto found=cells_.find({x,y,z});
      if (found==cells_.end()) continue;
      for (const auto index : found->second) {
        bool rejected=false;
        corners(samples_[index],[&](const Vec3 &point) {
          if (point.x<minimum.x || point.x>=maximum.x ||
              point.y<minimum.y || point.y>=maximum.y ||
              point.z<minimum.z || point.z>=maximum.z) return;
          observed=true;
          const double residual=dot(surface.normal,point)+surface.offset_m;
          if (std::abs(residual)>residual_tolerance) rejected=true;
          maximum_positive_residual=std::max(maximum_positive_residual,residual);
        });
        if (rejected) return false;
      }
    }
  if (observed && max_positive_residual_m)
    *max_positive_residual_m=maximum_positive_residual;
  return observed;
}

double primitivePlaneClearance(const Part &part, const Pose &pose,
                               const TerrainSurface &surface, double margin) {
  const double cosine=std::cos(pose.yaw), sine=std::sin(pose.yaw);
  const Vec3 center{pose.position.x+cosine*part.center.x-sine*part.center.y,
                    pose.position.y+sine*part.center.x+cosine*part.center.y,
                    pose.position.z+part.center.z};
  const Vec3 local_normal{cosine*surface.normal.x+sine*surface.normal.y,
                         -sine*surface.normal.x+cosine*surface.normal.y,surface.normal.z};
  Vec3 q{};
  q.x=part.rotation[0]*local_normal.x+part.rotation[3]*local_normal.y+
      part.rotation[6]*local_normal.z;
  q.y=part.rotation[1]*local_normal.x+part.rotation[4]*local_normal.y+
      part.rotation[7]*local_normal.z;
  q.z=part.rotation[2]*local_normal.x+part.rotation[5]*local_normal.y+
      part.rotation[8]*local_normal.z;
  double radius=0;
  if (part.shape==Shape::Box)
    radius=std::abs(q.x)*(part.half_size.x+margin)+
           std::abs(q.y)*(part.half_size.y+margin)+
           std::abs(q.z)*(part.half_size.z+margin);
  else if (part.shape==Shape::Sphere) radius=part.radius_m+margin;
  else if (part.shape==Shape::Cylinder)
    radius=(part.half_length_m+margin)*std::abs(q.z)+
           (part.radius_m+margin)*std::hypot(q.x,q.y);
  else radius=part.half_length_m*std::abs(q.z)+part.radius_m+margin;
  return dot(surface.normal,center)+surface.offset_m-radius;
}

}  // namespace lingtu::nav::collision
