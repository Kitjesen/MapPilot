// Copyright 2025 Dimensional Inc.
// Adapted in 2026 for LingTu; Apache-2.0. See src/maps/SURFACE_NOTICE.
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <set>

#include <octomap/OcTree.h>

#include "lingtu/maps/layers/ground_surface.hpp"
#include "lingtu/maps/layers/surface.hpp"
#include "lingtu/maps/io/binary.hpp"

namespace lingtu::maps::layers {
namespace {
int block(int cell) { return cell >= 0 ? cell / 8 : (cell + 1) / 8 - 1; }
}

void SurfaceIndex::write(BinaryWriter& out) const {
  out.value(resolution_);
  out.value<std::uint64_t>(observed_surfaces_);out.value<std::uint64_t>(inferred_surfaces_);
  out.value<std::uint64_t>(stats_.bounded_holes);out.value<std::uint64_t>(stats_.filled_holes);
  out.value<std::uint64_t>(stats_.free_rejected);
  for(const auto* columns : {&occupied_,&inferred_}) {
    out.value<std::uint64_t>(columns->size());
    for(const auto& [key,levels]:*columns) {
      out.value<std::int32_t>(key.x);out.value<std::int32_t>(key.y);out.vector(levels);
    }
  }
  out.value<std::uint64_t>(blocks_.size());
  for(const auto& [key,levels]:blocks_) {
    out.value<std::int32_t>(key.x);out.value<std::int32_t>(key.y);
    out.value<std::uint64_t>(levels.size());
    for(const auto& level:levels){out.value<std::int32_t>(level.z);out.value(level.xy);}
  }
}

std::shared_ptr<SurfaceIndex> SurfaceIndex::read(BinaryReader& in) {
  static_assert(sizeof(int)==sizeof(std::int32_t));
  auto result=std::shared_ptr<SurfaceIndex>(new SurfaceIndex);
  result->resolution_=in.value<double>();
  if(!std::isfinite(result->resolution_) || result->resolution_<=0)
    throw std::runtime_error("invalid prepared surface resolution");
  result->observed_surfaces_=in.value<std::uint64_t>();result->inferred_surfaces_=in.value<std::uint64_t>();
  result->stats_.bounded_holes=in.value<std::uint64_t>();result->stats_.filled_holes=in.value<std::uint64_t>();
  result->stats_.free_rejected=in.value<std::uint64_t>();
  for(auto* columns : {&result->occupied_,&result->inferred_}) {
    const auto count=in.count(16);columns->reserve(count);
    for(std::size_t i=0;i<count;++i) {
      const auto x=in.value<std::int32_t>(),y=in.value<std::int32_t>();
      auto levels=in.vector<int>();
      if(!std::is_sorted(levels.begin(),levels.end()) ||
          !columns->emplace(SurfaceCell{x,y,0},std::move(levels)).second)
        throw std::runtime_error("invalid prepared surface columns");
    }
  }
  const auto blocks=in.count(16);result->blocks_.reserve(blocks);
  for(std::size_t i=0;i<blocks;++i) {
    const auto x=in.value<std::int32_t>(),y=in.value<std::int32_t>();
    const auto count=in.count(12);
    std::vector<OccupiedLevel> levels;levels.reserve(count);
    for(std::size_t j=0;j<count;++j) {
      const auto z=in.value<std::int32_t>();const auto xy=in.value<std::uint64_t>();
      if(!levels.empty() && z<=levels.back().z)throw std::runtime_error("invalid prepared occupancy levels");
      levels.push_back({z,xy});
    }
    if(!result->blocks_.emplace(SurfaceCell{x,y,0},std::move(levels)).second)
      throw std::runtime_error("duplicate prepared occupancy block");
  }
  return result;
}

bool SurfaceIndex::regionOccupied(int min_x, int min_y, int max_x, int max_y, int min_z, int max_z) const {
  for (int x=block(min_x); x<=block(max_x); ++x) for (int y=block(min_y); y<=block(max_y); ++y) {
    const auto column=blocks_.find({x,y,0});
    if (column==blocks_.end()) continue;
    const auto &levels=column->second;
    auto it=std::lower_bound(levels.begin(),levels.end(),min_z,
        [](const OccupiedLevel &level,int z){return level.z<z;});
    if (it==levels.end() || it->z>max_z) continue;
    const int x0=std::max(min_x-x*8,0),x1=std::min(max_x-x*8,7);
    const int y0=std::max(min_y-y*8,0),y1=std::min(max_y-y*8,7);
    const auto row=((std::uint64_t{1}<<(x1-x0+1))-1)<<x0;
    std::uint64_t mask=0;
    for(int row_y=y0;row_y<=y1;++row_y)mask|=row<<(8*row_y);
    for(;it!=levels.end() && it->z<=max_z;++it)
      if(it->xy&mask)return true;
  }
  return false;
}

std::optional<int> SurfaceIndex::below(int x, int y, int z) const {
  const auto column = occupied_.find({x,y,0});
  if (column == occupied_.end()) return {};
  const auto &levels = column->second;
  auto it = std::lower_bound(levels.begin(), levels.end(), z);
  return it == levels.begin() ? std::optional<int>{} : *--it;
}

bool SurfaceIndex::occupied(int x, int y, int minimum_z, int maximum_z) const {
  const auto column = occupied_.find({x,y,0});
  if (column == occupied_.end()) return false;
  const auto &levels = column->second;
  const auto it = std::lower_bound(levels.begin(), levels.end(), minimum_z);
  return it != levels.end() && *it <= maximum_z;
}

bool SurfaceIndex::inferred(int x, int y, int minimum_z, int maximum_z) const {
  const auto column = inferred_.find({x,y,0});
  if (column == inferred_.end()) return false;
  const auto &levels = column->second;
  const auto it = std::lower_bound(levels.begin(), levels.end(), minimum_z);
  return it != levels.end() && *it <= maximum_z;
}

bool SurfaceIndex::headroomFree(int x, int y, int z, int height) const {
  return !occupied(x,y,z+1,z+height);
}

std::vector<SurfaceCell> SurfaceIndex::surfaces(double headroom) const {
  const int clearance=static_cast<int>(std::ceil(headroom/resolution_));
  std::vector<SurfaceCell> points;
  for (const auto &[xy,levels] : occupied_)
    for (std::size_t i=0; i<levels.size(); ++i)
      if (i+1==levels.size() || levels[i+1]-levels[i]>clearance)
        points.push_back({xy.x,xy.y,levels[i]});
  for (const auto &[xy,levels] : inferred_)
    for (int z : levels) points.push_back({xy.x,xy.y,z});
  std::sort(points.begin(),points.end(),[](const auto &a,const auto &b) {
    return std::array<int,3>{a.x,a.y,a.z}<std::array<int,3>{b.x,b.y,b.z};
  });
  points.erase(std::unique(points.begin(),points.end()),points.end());
  return points;
}

std::optional<double> SurfaceIndex::height(const std::array<double, 3>& expected, double tolerance) const {
  const int x=static_cast<int>(std::floor(expected[0]/resolution_));
  const int y=static_cast<int>(std::floor(expected[1]/resolution_));
  const int low=static_cast<int>(std::floor((expected[2]-tolerance)/resolution_))-1;
  const int high=static_cast<int>(std::floor((expected[2]+tolerance)/resolution_));
  std::optional<double> result;
  for (const auto* columns : {&occupied_, &inferred_}) {
    const auto column=columns->find({x,y,0});
    if (column==columns->end()) continue;
    const auto& levels=column->second;
    for (auto it=std::lower_bound(levels.begin(),levels.end(),low); it!=levels.end() && *it<=high; ++it) {
      // Ground is an exposed surface; body clearance remains a separate collision query.
      const auto next=std::next(it);
      if (columns==&occupied_ && next!=levels.end() && *next==*it+1) continue;
      const double z=std::clamp(expected[2],*it*resolution_,(*it+1)*resolution_);
      if (std::abs(z-expected[2])>tolerance+1e-7) continue;
      if (z==expected[2]) return z;
      if (!result || std::abs(z-expected[2])<std::abs(*result-expected[2])) result=z;
    }
  }
  return result;
}

std::vector<SurfaceCell> SurfaceIndex::inferredPoints() const {
  std::vector<SurfaceCell> points;
  points.reserve(inferred_surfaces_);
  for (const auto& [xy, levels] : inferred_)
    for (int z : levels) points.push_back({xy.x, xy.y, z});
  std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) {
    return std::array<int,3>{a.x,a.y,a.z} < std::array<int,3>{b.x,b.y,b.z};
  });
  return points;
}

SurfaceIndex::SurfaceIndex(const octomap::OcTree& tree, double headroom, double radius,
                           double max_hole_area_m2, double max_slope) {
  const double resolution = tree.getResolution();
  resolution_ = resolution;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    if (!tree.isNodeOccupied(*it)) continue;
    const int span = static_cast<int>(std::lround(it.getSize()/resolution));
    const int x0 = static_cast<int>(std::lround((it.getX()-.5*it.getSize())/resolution));
    const int y0 = static_cast<int>(std::lround((it.getY()-.5*it.getSize())/resolution));
    const int z0 = static_cast<int>(std::lround((it.getZ()-.5*it.getSize())/resolution));
    for (int x=x0; x<x0+span; ++x) for (int y=y0; y<y0+span; ++y) {
      auto& column = occupied_[{x,y,0}];
      for (int z=z0; z<z0+span; ++z) column.push_back(z);
    }
  }
  const int clearance = static_cast<int>(std::ceil(headroom/resolution));
  std::map<int,std::vector<SurfaceCell>> observed;
  for (auto& [key,column] : occupied_) {
    std::sort(column.begin(),column.end());
    column.erase(std::unique(column.begin(),column.end()),column.end());
    const int bx=block(key.x), by=block(key.y);
    const auto xy=std::uint64_t{1}<<((key.y-by*8)*8+key.x-bx*8);
    auto& levels=blocks_[{bx,by,0}];
    for (std::size_t i=0; i<column.size(); ++i) {
      levels.push_back({column[i],xy});
      if (i+1==column.size() || column[i+1]-column[i]>clearance) {
        observed[column[i]].push_back({key.x,key.y,column[i]});
        ++observed_surfaces_;
      }
    }
  }
  for (auto& [key,levels] : blocks_) {
    std::sort(levels.begin(),levels.end(),[](const auto& a,const auto& b){return a.z<b.z;});
    std::size_t size=0;
    for (const auto level : levels) {
      if (size && levels[size-1].z==level.z) levels[size-1].xy|=level.xy;
      else levels[size++]=level;
    }
    levels.resize(size);
  }
  if (radius <= 0 || max_hole_area_m2 <= 0 || observed.empty()) return;

  const int band = static_cast<int>(std::ceil(radius*max_slope/resolution))+1;
  const std::size_t max_cells = static_cast<std::size_t>(
      std::floor(max_hole_area_m2/(resolution*resolution)+1e-7));
  const int missing = std::numeric_limits<int>::min();
  const std::array<std::array<int,2>,4> directions{{{{-1,0}},{{1,0}},{{0,-1}},{{0,1}}}};
  // Each expected Z selects its nearest observed sheet, preserving stacked surfaces.
  for (int z=observed.begin()->first; z<=observed.rbegin()->first; ++z) {
    std::vector<SurfaceCell> samples;
    for (auto it=observed.lower_bound(z-band); it!=observed.end() && it->first<=z+band; ++it)
      samples.insert(samples.end(),it->second.begin(),it->second.end());
    if (samples.empty()) continue;
    int min_x=samples.front().x, max_x=min_x, min_y=samples.front().y, max_y=min_y;
    for (const auto& p : samples) {
      min_x=std::min(min_x,p.x); max_x=std::max(max_x,p.x);
      min_y=std::min(min_y,p.y); max_y=std::max(max_y,p.y);
    }
    --min_x; --min_y; ++max_x; ++max_y;
    const int cols=max_x-min_x+1, rows=max_y-min_y+1;
    const auto index=[&](int x,int y){return static_cast<std::size_t>(y)*cols+x;};
    std::vector<int> grid(static_cast<std::size_t>(cols)*rows,missing);
    std::vector<bool> visited(grid.size(),false);
    for (const auto& p : samples) {
      auto& value=grid[index(p.x-min_x,p.y-min_y)];
      if (value==missing || std::abs(p.z-z)<std::abs(value-z)) value=p.z;
    }
    for (int sy=0; sy<rows; ++sy) for (int sx=0; sx<cols; ++sx) {
      if (grid[index(sx,sy)]!=missing || visited[index(sx,sy)]) continue;
      std::vector<std::array<int,2>> hole{{sx,sy}};
      std::set<std::array<int,2>> rim;
      visited[index(sx,sy)]=true;
      bool outside=false;
      int low_x=sx,high_x=sx,low_y=sy,high_y=sy;
      for (std::size_t next=0; next<hole.size(); ++next) {
        const auto cell=hole[next];
        low_x=std::min(low_x,cell[0]); high_x=std::max(high_x,cell[0]);
        low_y=std::min(low_y,cell[1]); high_y=std::max(high_y,cell[1]);
        if (cell[0]==0 || cell[1]==0 || cell[0]+1==cols || cell[1]+1==rows) outside=true;
        for (const auto& direction : directions) {
          const int x=cell[0]+direction[0], y=cell[1]+direction[1];
          if (x<0 || y<0 || x>=cols || y>=rows) continue;
          const auto i=index(x,y);
          if (grid[i]!=missing) { rim.insert({x,y}); continue; }
          if (!visited[i]) { visited[i]=true; hole.push_back({x,y}); }
        }
      }
      if (outside || hole.size()>max_cells ||
          (high_x-low_x+1)*resolution>2*radius+1e-7 ||
          (high_y-low_y+1)*resolution>2*radius+1e-7) continue;
      const double center_x=.5*(low_x+high_x), center_y=.5*(low_y+high_y);
      std::vector<ground_detail::Point> boundary;
      boundary.reserve(rim.size());
      for (const auto& p : rim) boundary.push_back({
          (p[0]-center_x)*resolution,(p[1]-center_y)*resolution,
          (grid[index(p[0],p[1])]+.5)*resolution,p[0],p[1],{}});
      const auto plane=ground_detail::Fit(boundary);
      if (!plane.valid || std::hypot(plane.x,plane.y)>max_slope+1e-7 ||
          plane.rms>.35*resolution) continue;
      if (std::any_of(boundary.begin(),boundary.end(),[&](const auto& p){
            return std::abs(p.z-plane.At(p.x,p.y))>.6*resolution;
          })) continue;
      bool matched=false,filled=false;
      for (const auto& p : hole) {
        const double fitted=plane.At((p[0]-center_x)*resolution,(p[1]-center_y)*resolution);
        const int level=static_cast<int>(std::floor(fitted/resolution+1e-7));
        if (level!=z) continue;
        matched=true;
        const bool nearby=std::any_of(rim.begin(),rim.end(),[&](const auto& q){
          return std::hypot(static_cast<double>(p[0]-q[0]),static_cast<double>(p[1]-q[1]))
              *resolution<=radius+1e-7;
        });
        if (!nearby) continue;
        const int x=p[0]+min_x,y=p[1]+min_y;
        const auto* evidence=tree.search((x+.5)*resolution,(y+.5)*resolution,(level+.5)*resolution);
        if (evidence && !tree.isNodeOccupied(evidence)) { ++stats_.free_rejected; continue; }
        if (occupied(x,y,level,level) || !headroomFree(x,y,level,clearance)) continue;
        inferred_[{x,y,0}].push_back(level);
        filled=true;
      }
      if (matched) ++stats_.bounded_holes;
      if (filled) ++stats_.filled_holes;
    }
  }
  for (auto& [xy,levels] : inferred_) {
    std::sort(levels.begin(),levels.end());
    levels.erase(std::unique(levels.begin(),levels.end()),levels.end());
    inferred_surfaces_+=levels.size();
  }
}

}  // namespace lingtu::maps::layers
