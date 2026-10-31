#include "ground/segmenter.hpp"
#include "patchwork/patchworkpp.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace lingtu::maps::ground;
namespace {
void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

std::vector<Point3f> FloorAndWall(float height) {
  std::vector<Point3f> points;
  for (int x=-60; x<=60; ++x)
    for (int y=-60; y<=60; ++y) {
      const float xx=x*.05F, yy=y*.05F, radius=std::hypot(xx,yy);
      if (radius>=.5F && radius<=4.F) points.push_back({xx,yy,-height});
    }
  for (int y=-60; y<=60; ++y)
    for (int z=0; z<35; ++z)
      points.push_back({2.8F,y*.05F,-height+.15F+z*.05F});
  return points;
}

std::vector<Point3f> Narrow(const std::vector<Point3f>& points) {
  std::vector<Point3f> narrow;
  for (auto p:points)
    if (p.x>=.5F && std::abs(p.y)<=.4F*p.x) narrow.push_back(p);
  return narrow;
}

patchwork::Params Params(float height) {
  patchwork::Params p;
  p.enable_RNR=false;
  p.verbose=false;
  p.sensor_height=height;
  p.min_range=.3;
  p.max_range=40.;
  p.th_dist=.04;
  p.th_seeds=.08;
  return p;
}

std::vector<std::uint8_t> Reference(patchwork::PatchWorkpp& engine,
                                  const std::vector<Point3f>& points) {
  Eigen::MatrixXf input(points.size(),3);
  for (std::size_t i=0;i<points.size();++i) input.row(i)<<points[i].x,points[i].y,points[i].z;
  engine.estimateGround(std::move(input));
  std::vector<std::uint8_t> mask(points.size());
  for (int i:engine.getGroundIndices()) {
    Require(i>=0 && static_cast<std::size_t>(i)<mask.size(),"reference index out of range");
    mask[i]=1;
  }
  return mask;
}

void Compare(PatchworkGroundSegmenter& adapter,patchwork::PatchWorkpp& core,
             const std::vector<Point3f>& points) {
  const auto actual=adapter.Segment(points);
  const auto expected=Reference(core,points);
  Require(actual.segmented,"initialized adapter skipped a valid finite frame");
  Require(actual.ground_mask==expected,"adapter labels differ from persistent upstream core");
  Require(std::abs(actual.sensor_height_m-core.getHeight())<1e-5,
          "reported height differs from the engine's adaptive height");
}

void NarrowContinuity() {
  PatchworkGroundSegmenter adapter;
  patchwork::PatchWorkpp core(Params(.6F));
  const auto full=FloorAndWall(.6F);
  Compare(adapter,core,full);
  const auto narrow=Narrow(full);
  Compare(adapter,core,narrow);
}

void HeightContinuity() {
  PatchworkGroundSegmenter adapter;
  patchwork::PatchWorkpp core(Params(.6F));
  const auto full=FloorAndWall(.6F);
  for (int frame=0;frame<5;++frame) Compare(adapter,core,full);
  // A 30 cm change used to reconstruct the engine, discarding learned thresholds.
  for (int frame=0;frame<3;++frame) Compare(adapter,core,FloorAndWall(.9F));
  Compare(adapter,core,full);
}

void Reset() {
  PatchworkGroundSegmenter adapter;
  const auto full=FloorAndWall(.6F);
  Require(adapter.Segment(full).segmented,"initialization failed");
  adapter.Reset();
  const auto result=adapter.Segment(Narrow(full));
  Require(!result.segmented && result.sensor_height_m==0,"Reset did not restore cold start");
  for (auto label:result.ground_mask) Require(label==0,"cold start returned ground labels");
  patchwork::PatchWorkpp fresh(Params(.6F));
  Compare(adapter,fresh,full);
}

void ColdInputs() {
  PatchworkGroundSegmenter adapter;
  for (const auto& points:std::vector<std::vector<Point3f>>{
           {},{{1.F,0.F,-.6F}},Narrow(FloorAndWall(.6F))}) {
    const auto r=adapter.Segment(points);
    Require(!r.segmented && r.sensor_height_m==0,"insufficient cold input initialized engine");
    Require(r.ground_mask.size()==points.size(),"cold mask lost input correspondence");
    for (auto label:r.ground_mask) Require(label==0,"insufficient input returned ground labels");
  }
  auto uneven=FloorAndWall(.6F);
  for(auto& p:uneven) if(p.y<0) p.z-=.6F;
  Require(!adapter.Segment(uneven).segmented,"inconsistent cold floor initialized engine");
  patchwork::PatchWorkpp core(Params(.6F));
  Compare(adapter,core,FloorAndWall(.6F));
}

void Nonfinite() {
  const float nan=std::numeric_limits<float>::quiet_NaN();
  const float inf=std::numeric_limits<float>::infinity();
  PatchworkGroundSegmenter adapter;
  const std::vector<Point3f> invalid{{nan,0,-.6F},{0,inf,-.6F},{1,1,nan}};
  Require(!adapter.Segment(invalid).segmented,"nonfinite input initialized engine");
  const auto finite=FloorAndWall(.6F);
  std::vector<Point3f> mixed{invalid.front()};
  for (std::size_t i=0;i<finite.size();++i) {
    mixed.push_back(finite[i]);
    if(i==100) mixed.push_back(invalid[1]);
  }
  mixed.push_back(invalid.back());
  auto actual=adapter.Segment(mixed);
  patchwork::PatchWorkpp core(Params(.6F));
  const auto expected=Reference(core,finite);
  Require(actual.segmented && actual.ground_mask.size()==mixed.size(),"mixed nonfinite input failed");
  std::size_t j=0;
  for(std::size_t i=0;i<mixed.size();++i) {
    const auto p=mixed[i];
    if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))
      Require(actual.ground_mask[i]==expected[j++],"finite point index mapping changed");
    else Require(actual.ground_mask[i]==0,"nonfinite input labeled ground");
  }
  Require(!adapter.Segment(invalid).segmented,"all-nonfinite frame invoked engine");
  Compare(adapter,core,Narrow(finite));
}
}

int main(int argc,char** argv) {
  try {
    Require(argc==2,"expected one test case");
    const std::string test=argv[1];
    if(test=="narrow_continuity") NarrowContinuity();
    else if(test=="height_continuity") HeightContinuity();
    else if(test=="reset") Reset();
    else if(test=="cold_inputs") ColdInputs();
    else if(test=="nonfinite") Nonfinite();
    else throw std::runtime_error("unknown test case");
    std::cout<<test<<" passed\n";
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<e.what()<<'\n';
    return 1;
  }
}
