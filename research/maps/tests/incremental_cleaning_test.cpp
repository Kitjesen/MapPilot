#include "cleaning/incremental.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace lingtu::maps::cleaning;
namespace {
void Require(bool ok, const char* message) { if(!ok) throw std::runtime_error(message); }
void Observe(IncrementalCleaner& c, std::initializer_list<Point> p) { c.Observe(std::span(p.begin(),p.size()),{}); }
bool HasPoint(const IncrementalCleaner& cleaner, Point expected) {
  const auto points = cleaner.Snapshot();
  return std::any_of(points.begin(), points.end(), [&](Point p) {
    return p.x == expected.x && p.y == expected.y && p.z == expected.z;
  });
}
void EvidenceRules() {
  IncrementalCleaner c;
  Observe(c,{{2.001F,.1F,.1F},{5,1,0}});
  Observe(c,{{3.998F,.2F,.2F},{3.998F,.2F,.2F}});
  Require(HasPoint(c,{2.001F,.1F,.1F}),"duplicate rays counted as frames");
  Observe(c,{{3.998F,.2F,.2F}});
  Require(HasPoint(c,{2.001F,.1F,.1F}),"two free frames deleted an object");
  Observe(c,{{3.998F,.2F,.2F}});
  Require(!HasPoint(c,{2.001F,.1F,.1F}) && HasPoint(c,{5,1,0}),"cross-voxel ghost/unobserved point handling");
  Observe(c,{{2.001F,.1F,.1F},{3.998F,.2F,.2F}});
  Require(HasPoint(c,{2.001F,.1F,.1F}),"same-frame endpoint failed to restore object");
  for(int i=0;i<5;++i) { Observe(c,{{1,0,0}}); Observe(c,{}); }
  Require(HasPoint(c,{2.001F,.1F,.1F}),"occlusion/missing return became free evidence");
  c.Reset();
  for(int i=0;i<3;++i) Observe(c,{{4,0,0}});
  Observe(c,{{2,0,0}});
  Require(HasPoint(c,{2,0,0}),"prior free space deleted a new object");
}
void NeighborhoodAndOcclusion() {
  IncrementalCleaner c;
  Observe(c,{{2,.045F,0}});
  for(int i=0;i<3;++i) Observe(c,{{4,0,0}});
  Require(HasPoint(c,{2,.045F,0}),"one ray cleared an incompletely observed neighborhood");
  for(int i=0;i<3;++i) Observe(c,{{4,0,0},{4,.18F,0}});
  Require(!HasPoint(c,{2,.045F,0}),"neighboring measured rays failed to establish reliable free space");
  c.Reset(); Observe(c,{{4,0,0}});
  for(int i=0;i<4;++i) Observe(c,{{2,0,0},{8,0,0}});
  Require(HasPoint(c,{4,0,0}),"far return passed through a nearer current occupied cell");
  c.Reset(); Observe(c,{{4,.18F,0}});
  for(int i=0;i<4;++i) Observe(c,{{2,.101F,0},{8,.36F,0}});
  Require(HasPoint(c,{4,.18F,0}),"neighbor-voxel current endpoint failed to block a far ray");
  c.Reset(); Observe(c,{{2,0,0}});
  for(int i=0;i<3;++i)Observe(c,{{2.14F,0,0}});
  Require(HasPoint(c,{2,0,0}),"endpoint margin was counted as measured free space");
  for(int i=0;i<3;++i)Observe(c,{{2.20F,0,0}});
  Require(!HasPoint(c,{2,0,0}),"valid measured neighborhood beyond margin was ignored");
  c.Reset();Observe(c,{{2,-.001F,0}});
  for(int i=0;i<3;++i)Observe(c,{{4,.002F,0}});
  Require(!HasPoint(c,{2,-.001F,0}),"angular bin boundary lost an exact measured-ray contradiction");
}
void SurfacesAndLowGhost() {
  for(int orientation=0;orientation<4;++orientation) {
    IncrementalCleaner c;
    Point center{2,0,-.3F};
    std::vector<Point> neighbors;
    for(int u=-1;u<=1;++u) for(int v=-1;v<=1;++v) {
      if(u==0 && v==0) continue;
      const float a=.3F*u,b=.3F*v;
      if(orientation==0) neighbors.push_back({2+a,b,-.3F});
      else if(orientation==1) neighbors.push_back({2,a,-.3F+b});
      else if(orientation==2) neighbors.push_back({2+a,b,-.3F+.4F*a});
      else neighbors.push_back({2.3F,a,-.3F+b});
    }
    auto initial=neighbors; initial.push_back(center);
    c.Observe(initial,{});c.Observe(initial,{});
    neighbors.push_back({4,0,-.6F});
    for(int i=0;i<3;++i)c.Observe(neighbors,{});
    Require(HasPoint(c,center)==(orientation!=3),"floor/wall/ramp support or detached wall ghost handling");
  }
  IncrementalCleaner c;
  const std::vector<Point> low{{1,0,-.6F}};
  const std::vector<std::uint8_t> mask{1};
  c.Observe(low,{},mask); c.Observe(low,{},mask);
  for(int i=0;i<3;++i)Observe(c,{{2,0,-1.2F}});
  Require(!HasPoint(c,low[0]),"low classified object acquired permanent ground immunity");
}
void BoundsAndGround() {
  IncrementalCleaner stairs;
  std::vector<Point> treads;
  for(int u=-1;u<=1;++u) for(int v=-1;v<=1;++v) {
    treads.push_back({2+u*.15F,v*.15F,-.6F});
    treads.push_back({2+u*.15F,v*.15F,-.3F});
  }
  stairs.Observe(treads,{});stairs.Observe(treads,{});
  for(int i=0;i<3;++i) Observe(stairs,{{4,0,-1.2F}});
  Require(HasPoint(stairs,{2,0,-.6F}) && HasPoint(stairs,{2,0,-.3F}),"overlapping stair treads were blended or erased");
  Config config;config.max_cells=2;
  IncrementalCleaner c(config);
  Observe(c,{{1,0,0},{2,0,0},{3,0,0}});
  Require(c.Count()==2 && c.CapacityRejections()==1,"unbounded evidence storage");
  c.Reset();Require(c.Count()==0 && c.RemovedCount()==0,"epoch reset retained evidence");
  lingtu::maps::ground::PatchworkGroundSegmenter ground;
  auto sparse=ground.Segment(std::vector<lingtu::maps::ground::Point3f>{{1,0,-.6F}});
  Require(!sparse.segmented && sparse.ground_mask[0]==0,"sparse low return guessed ground");
  std::vector<lingtu::maps::ground::Point3f> points;
  for(int x=-40;x<=40;++x) for(int y=-40;y<=40;++y) points.push_back({x*.1F,y*.1F,-.6F});
  const auto floor_size=points.size();
  for(int y=-20;y<=20;++y) for(int z=0;z<=20;++z) points.push_back({3.F,y*.1F,z*.1F-.4F});
  const auto segmented=ground.Segment(points);
  Require(segmented.segmented && std::abs(segmented.sensor_height_m-.6F)<.05F,"measured sensor height failed");
  const auto floors=std::count(segmented.ground_mask.begin(),segmented.ground_mask.begin()+floor_size,1);
  const auto walls=std::count(segmented.ground_mask.begin()+floor_size,segmented.ground_mask.end(),1);
  Require(floors>static_cast<int>(floor_size/2) && walls<20,"Patchwork floor/wall separation failed");
}
}
int main() {
  try {EvidenceRules();NeighborhoodAndOcclusion();SurfacesAndLowGhost();BoundsAndGround();}
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
