// Copyright 2025 Dimensional Inc.
// Adapted in 2026 for LingTu; Apache-2.0. See SURFACE_NOTICE and LICENSE-Apache-2.0.
#include "surface.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <map>

namespace global_planner {
namespace {
int block(int cell) { return cell >= 0 ? cell / 8 : (cell + 1) / 8 - 1; }
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

bool SurfaceIndex::nearbyEvidence(int x, int y, int z) const {
  for (int dx=-3; dx<=3; ++dx) for (int dy=-3; dy<=3; ++dy)
    if (occupied(x+dx,y+dy,z-3,z+3)) return true;
  return false;
}

std::vector<GridIndex> SurfaceIndex::surfaces(double headroom) const {
  const int clearance=static_cast<int>(std::ceil(headroom/resolution_));
  std::vector<GridIndex> points;
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

std::optional<double> SurfaceIndex::height(const PointPose &expected, double tolerance, int neighbor_radius) const {
  const int x=static_cast<int>(std::floor(expected.x/resolution_));
  const int y=static_cast<int>(std::floor(expected.y/resolution_));
  const int low=static_cast<int>(std::floor((expected.z-tolerance)/resolution_))-1;
  const int high=static_cast<int>(std::floor((expected.z+tolerance)/resolution_));
  const auto columnHeight = [&](int cx, int cy) -> std::optional<double> {
    std::optional<double> result;
    for (const auto *columns : {&occupied_, &inferred_}) {
      const auto column=columns->find({cx,cy,0});
      if (column==columns->end()) continue;
      const auto &levels=column->second;
      for (auto it=std::lower_bound(levels.begin(),levels.end(),low); it!=levels.end() && *it<=high; ++it) {
        // Ground is an exposed surface; body clearance remains a separate collision query.
        const auto next=std::next(it);
        if (columns==&occupied_ && next!=levels.end() && *next==*it+1) continue;
        const double z=std::clamp(expected.z,*it*resolution_,(*it+1)*resolution_);
        if (std::abs(z-expected.z)>tolerance+1e-7) continue;
        if (z==expected.z) return z;
        if (!result || std::abs(z-expected.z)<std::abs(*result-expected.z)) result=z;
      }
    }
    return result;
  };
  if (const auto center=columnHeight(x,y)) return center;
  std::vector<double> directions;
  std::optional<double> result;
  for (int dx=-neighbor_radius; dx<=neighbor_radius; ++dx)
    for (int dy=-neighbor_radius; dy<=neighbor_radius; ++dy) {
      if (!dx && !dy) continue;
      const auto height=columnHeight(x+dx,y+dy);
      if (!height) continue;
      directions.push_back(std::atan2((y+dy+.5)*resolution_-expected.y,
                                      (x+dx+.5)*resolution_-expected.x));
      if (!result || std::abs(*height-expected.z)<std::abs(*result-expected.z)) result=height;
    }
  if (directions.size()<3) return {};
  std::sort(directions.begin(),directions.end());
  const double pi=std::acos(-1.0);
  double gap=directions.front()+2*pi-directions.back();
  for (std::size_t i=1; i<directions.size(); ++i) gap=std::max(gap,directions[i]-directions[i-1]);
  if (gap>=pi-1e-12) return {};
  return result;
}

SurfaceIndex::SurfaceIndex(const octomap::OcTree &tree, double headroom, double closing_radius) {
  const double resolution = tree.getResolution();
  resolution_=resolution;
  for (auto it=tree.begin_leafs(); it!=tree.end_leafs(); ++it) {
    if (!tree.isNodeOccupied(*it)) continue;
    const int span = static_cast<int>(std::lround(it.getSize()/resolution));
    const int x0 = static_cast<int>(std::lround((it.getX()-.5*it.getSize())/resolution));
    const int y0 = static_cast<int>(std::lround((it.getY()-.5*it.getSize())/resolution));
    const int z0 = static_cast<int>(std::lround((it.getZ()-.5*it.getSize())/resolution));
    for (int x=x0; x<x0+span; ++x) for (int y=y0; y<y0+span; ++y) {
      auto &column = occupied_[{x,y,0}];
      for (int z=z0; z<z0+span; ++z) column.push_back(z);
    }
  }
  for (auto &[key,column] : occupied_) {
    std::sort(column.begin(),column.end());
    const int bx=block(key.x),by=block(key.y);
    const auto xy=std::uint64_t{1}<<((key.y-by*8)*8+key.x-bx*8);
    auto &levels=blocks_[{bx,by,0}];
    for(int z:column)levels.push_back({z,xy});
  }
  for (auto &[key,levels] : blocks_) {
    std::sort(levels.begin(),levels.end(),[](const auto&a,const auto&b){return a.z<b.z;});
    std::size_t size=0;
    for(const auto level:levels) {
      if(size && levels[size-1].z==level.z)levels[size-1].xy|=level.xy;
      else levels[size++]=level;
    }
    levels.resize(size);
  }
  if (closing_radius <= 0) return;
  const int height = static_cast<int>(std::ceil(headroom/resolution));
  const int passes = static_cast<int>(std::ceil(closing_radius/resolution));
  std::map<int,std::vector<GridIndex>> layers;
  for (const auto &[xy,column] : occupied_) for (std::size_t i=0; i<column.size(); ++i) {
    if (i+1==column.size() || column[i+1]-column[i]>height) {
      layers[column[i]].push_back({xy.x,xy.y,column[i]});
      ++observed_surfaces_;
    }
  }
  // Same per-height L1 closing and evidence test as pinned DimOS MLS surfaces.rs.
  // Keep the occupied columns separate: repaired support is not a new obstacle.
  for (const auto &[z,points] : layers) {
    int min_x=points.front().x, max_x=min_x, min_y=points.front().y, max_y=min_y;
    for (const auto &p : points) {
      min_x=std::min(min_x,p.x); max_x=std::max(max_x,p.x);
      min_y=std::min(min_y,p.y); max_y=std::max(max_y,p.y);
    }
    min_x-=passes; min_y-=passes;
    const int width=max_x-min_x+passes+1, rows=max_y-min_y+passes+1;
    const int words=(width+63)/64;
    const std::uint64_t tail=width%64 ? (std::uint64_t{1}<<(width%64))-1 : ~std::uint64_t{0};
    std::vector<std::uint64_t> grid(static_cast<std::size_t>(words)*rows), scratch(grid.size());
    for (const auto &p : points)
      grid[static_cast<std::size_t>(p.y-min_y)*words+(p.x-min_x)/64] |= std::uint64_t{1}<<((p.x-min_x)%64);
    const auto original=grid;
    for (int phase=0; phase<2; ++phase) for (int n=0; n<passes; ++n) {
      for (int y=0; y<rows; ++y) for (int w=0; w<words; ++w) {
        const auto i=static_cast<std::size_t>(y)*words+w;
        const auto a=grid[i];
        const auto left=(a<<1) | (w>0 ? grid[i-1]>>63 : 0);
        const auto right=(a>>1) | (w+1<words ? grid[i+1]<<63 : 0);
        const auto up=y>0 ? grid[i-words] : 0;
        const auto down=y+1<rows ? grid[i+words] : 0;
        scratch[i]=phase==0 ? a|left|right|up|down : a&left&right&up&down;
        if (w+1==words) scratch[i]&=tail;
      }
      grid.swap(scratch);
    }
    for (int y=0; y<rows; ++y) for (int w=0; w<words; ++w) {
      const auto i=static_cast<std::size_t>(y)*words+w;
      const auto added=grid[i]&~original[i];
      if (!added) continue;
      for (int bit=0; bit<64; ++bit) if (added&(std::uint64_t{1}<<bit)) {
        const int x=min_x+w*64+bit, world_y=min_y+y;
        if (headroomFree(x,world_y,z,height) && nearbyEvidence(x,world_y,z)) {
          inferred_[{x,world_y,0}].push_back(z);
          ++inferred_surfaces_;
        }
      }
    }
  }
}

}  // namespace global_planner
