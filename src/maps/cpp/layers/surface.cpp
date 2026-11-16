// Copyright 2025 Dimensional Inc.
// Adapted in 2026 for LingTu; Apache-2.0. See src/maps/SURFACE_NOTICE.
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include <octomap/OcTree.h>

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
                           double max_hole_area_m2, double max_slope,
                           bool collect_rejections)
    : collect_rejections_(collect_rejections) {
  (void)max_hole_area_m2;
  (void)max_slope;
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
  const int passes = radius > 0.0
      ? static_cast<int>(std::ceil(radius / resolution)) : 0;
  if (passes <= 0 || observed.empty()) return;
  std::map<int, std::set<std::pair<int, int>>> layers;
  for (const auto &[z, cells] : observed) {
    auto &layer = layers[z];
    for (const auto &cell : cells) layer.insert({cell.x, cell.y});
  }
  const std::array<std::array<int, 2>, 4> directions{{{{-1, 0}}, {{1, 0}},
                                                        {{0, -1}}, {{0, 1}}}};
  const auto supported = [&](int x, int y, int z) {
    for (int dx = -3; dx <= 3; ++dx) for (int dy = -3; dy <= 3; ++dy) {
      const auto column = occupied_.find({x + dx, y + dy, 0});
      if (column == occupied_.end()) continue;
      if (std::any_of(column->second.begin(), column->second.end(),
                      [&](int occupied_z) { return std::abs(occupied_z - z) <= 3; }))
        return true;
    }
    return false;
  };
  for (auto &[z, original] : layers) {
    const int min_x = original.begin()->first - passes;
    const int max_x = original.rbegin()->first + passes;
    int min_y = original.begin()->second - passes;
    int max_y = original.begin()->second + passes;
    for (const auto &[x, y] : original) {
      (void)x;
      min_y = std::min(min_y, y - passes);
      max_y = std::max(max_y, y + passes);
    }
    auto closed = original;
    for (int pass = 0; pass < passes; ++pass) {
      auto next = closed;
      for (const auto &[x, y] : closed)
        for (const auto &direction : directions)
          if (x + direction[0] >= min_x && x + direction[0] <= max_x &&
              y + direction[1] >= min_y && y + direction[1] <= max_y)
            next.insert({x + direction[0], y + direction[1]});
      closed.swap(next);
    }
    for (int pass = 0; pass < passes; ++pass) {
      std::set<std::pair<int, int>> next;
      for (const auto &[x, y] : closed) {
        if (std::all_of(directions.begin(), directions.end(), [&](const auto &direction) {
              return closed.count({x + direction[0], y + direction[1]}) != 0;
            })) next.insert({x, y});
      }
      closed.swap(next);
    }
    closed.insert(original.begin(), original.end());
    bool filled = false;
    for (const auto &[x, y] : closed) {
      if (original.count({x, y}) != 0 || !headroomFree(x, y, z, clearance) ||
          !supported(x, y, z)) continue;
      inferred_[{x, y, 0}].push_back(z);
      filled = true;
    }
    if (filled) ++stats_.filled_holes;
  }
  for (auto& [xy,levels] : inferred_) {
    std::sort(levels.begin(),levels.end());
    levels.erase(std::unique(levels.begin(),levels.end()),levels.end());
    inferred_surfaces_+=levels.size();
  }
}

}  // namespace lingtu::maps::layers
