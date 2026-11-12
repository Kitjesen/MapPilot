#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <octomap/OcTree.h>

#include "lingtu/maps/layers/surface.hpp"

using lingtu::maps::layers::SurfaceIndex;

namespace {
void Require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

void Hit(octomap::OcTree& tree, int x, int y, int z) {
  const double r=tree.getResolution();
  tree.updateNode((x+.5)*r,(y+.5)*r,(z+.5)*r,true,true);
}

octomap::OcTree Flat(double resolution=.05) {
  octomap::OcTree tree(resolution);
  for (int x=-12;x<=12;++x) for (int y=-12;y<=12;++y)
    if (std::abs(x)>1 || std::abs(y)>1) Hit(tree,x,y,-1);
  tree.updateInnerOccupancy();
  return tree;
}

std::string Bytes(const octomap::OcTree& tree) {
  std::ostringstream out;
  tree.write(out);
  return out.str();
}

void FlatHole() {
  const auto tree=Flat();
  const auto before=Bytes(tree);
  SurfaceIndex raw(tree,.6,0), filled(tree,.6,.2);
  Require(raw.observedSurfaces()==616,"disabled completion lost observed count");
  Require(raw.inferredSurfaces()==0,"disabled completion inserted points");
  Require(filled.inferredSurfaces()==9,"small inner hole was not completed exactly");
  Require(filled.inferred(0,0,-1,-1),"hole center is missing");
  Require(filled.height({.025,.025,0},.05)==0.0,"height query did not consume completion");
  Require(!filled.occupied(0,0,-1,-1),"completion was classified as an observed obstacle");
  Require(!filled.inferred(13,0,-1,-1),"completion expanded the outer edge");
  Require(Bytes(tree)==before,"completion changed source OctoMap bytes");
}

void OpenAndLargeHoles() {
  auto open=Flat();
  for (int y=-12;y<=0;++y) for (int x=-1;x<=1;++x)
    open.deleteNode((x+.5)*.05,(y+.5)*.05,-.025);
  SurfaceIndex edge(open,.6,.2);
  Require(!edge.inferred(0,0,-1,-1),"boundary-connected gap was closed");
  const auto flat=Flat(.1);
  SurfaceIndex area(flat,.6,.3,.04);
  Require(area.inferredSurfaces()==0,"area limit was ignored");
  SurfaceIndex radius(flat,.6,.05,.16);
  Require(radius.inferredSurfaces()==0,"physical completion radius was ignored");
}

void LayersAndSlope() {
  auto stacked=Flat();
  for (int x=-12;x<=12;++x) for (int y=-12;y<=12;++y)
    if (std::abs(x)>1 || std::abs(y)>1) Hit(stacked,x,y,39);
  stacked.updateInnerOccupancy();
  SurfaceIndex layers(stacked,.6,.2);
  Require(layers.inferred(0,0,-1,-1) && layers.inferred(0,0,39,39),
          "stacked floor completion lost a layer");
  Require(!layers.inferred(0,0,10,30),"completion joined the space between floors");
  stacked.updateNode(.025,.025,1.975,false);
  SurfaceIndex changed(stacked,.6,.2);
  Require(changed.inferred(0,0,-1,-1) && !changed.inferred(0,0,39,39),
          "upper-floor free evidence changed the lower floor");

  octomap::OcTree slope(.05);
  for (int x=-12;x<=12;++x) for (int y=-12;y<=12;++y) {
    if (std::abs(x)<=1 && std::abs(y)<=1) continue;
    const int z=static_cast<int>(std::floor((1+.3*(x+.5)*.05+.15*(y+.5)*.05)/.05));
    Hit(slope,x,y,z);
  }
  slope.updateInnerOccupancy();
  SurfaceIndex ramp(slope,.6,.2);
  const auto z=ramp.height({.025,.025,1.02},.06);
  Require(z.has_value() && std::abs(*z-1.02)<.06,"sloped hole crossing Z bins was not completed");
  Require(ramp.inferredSurfaces()>0,"sloped completion produced no points");

  octomap::OcTree step(.1);
  for (int x=-10;x<=10;++x) for (int y=-10;y<=10;++y) {
    if (std::abs(x)<=1 && std::abs(y)<=1) continue;
    Hit(step,x,y,x<0?-1:3);
  }
  step.updateInnerOccupancy();
  SurfaceIndex discontinuous(step,.6,.3);
  Require(discontinuous.inferredSurfaces()==0,"completion bridged a real step discontinuity");
}

void ObservationEvidence() {
  auto tree=Flat();
  tree.updateNode(.025,.025,.225,false);
  SurfaceIndex above(tree,.6,.2);
  Require(above.inferred(0,0,-1,-1),"free space above ground incorrectly removed support");
  tree.updateNode(.025,.025,-.025,false);
  SurfaceIndex seenThrough(tree,.6,.2);
  Require(!seenThrough.inferred(0,0,-1,-1),"known-free ground was filled");
  Require(seenThrough.stats().free_rejected>0,"free evidence rejection was not recorded");
  Require(above.inferred(0,0,-1,-1),"rebuilding modified the previous immutable index");
  tree.updateNode(.025,.025,-.025,true);
  tree.updateNode(.025,.025,-.025,true);
  tree.updateInnerOccupancy();
  SurfaceIndex observed(tree,.6,.2);
  Require(observed.occupied(0,0,-1,-1),"new occupied observation did not take precedence");
  Require(!observed.inferred(0,0,-1,-1),"observed cell remained inferred");

  auto obstructed=Flat();
  Hit(obstructed,0,0,5);
  SurfaceIndex blocked(obstructed,.6,.2);
  Require(!blocked.inferred(0,0,-1,-1),"completion ignored occupied headroom");
}

void PreparedSupport() {
  auto tree=Flat();
  for (int x=-1;x<=1;++x) for (int y=-1;y<=1;++y)
    if (x || y) Hit(tree,x,y,-1);
  tree.updateInnerOccupancy();
  SurfaceIndex raw(tree,.6,0), completed(tree,.6,.2);
  Require(!raw.height({.025,.025,0},.05),
          "height query created support outside the prepared index");
  Require(completed.height({.025,.025,0},.05)==0.0,
          "prepared closed-hole completion did not supply height");
  Require(completed.height({.025,.025,.04},.05).has_value() &&
          !completed.height({.025,.025,.08},.05),
          "prepared support lost its vertical tolerance");
  tree.updateNode(.025,.025,.025,false);
  SurfaceIndex above(tree,.6,.2);
  Require(above.height({.025,.025,0},.05)==0.0,
          "free air above ground incorrectly removed prepared support");
  tree.updateNode(.025,.025,-.025,false);
  SurfaceIndex contradicted(tree,.6,.2);
  Require(!contradicted.height({.025,.025,0},.05),
          "prepared support ignored known-free ground");
  tree.updateNode(.025,.025,-.025,true);
  tree.updateNode(.025,.025,-.025,true);
  SurfaceIndex observed(tree,.6,.2);
  Require(observed.height({.025,.025,0},.05)==0.0,
          "new observed ground did not restore support");

  octomap::OcTree slot(.05);
  for (int x=-12;x<=12;++x) for (int y=-12;y<=12;++y)
    if (x) Hit(slot,x,y,-1);
  slot.updateInnerOccupancy();
  SurfaceIndex disconnected(slot,.6,.2);
  Require(!disconnected.inferred(0,0,-1,-1) && !disconnected.height({.025,.025,0},.05),
          "long open slot bypassed prepared completion");
  Require(disconnected.height({.075,.025,0},.05)==0.0,
          "slot rejection removed real support beside it");
}

void CoarseLeaves() {
  auto tree=Flat();
  tree.prune();
  SurfaceIndex index(tree,.6,.2);
  for (int x=-14;x<=14;++x) for (int y=-14;y<=14;++y) {
    const auto* node=tree.search((x+.5)*.05,(y+.5)*.05,-.025);
    const bool occupied=node && tree.isNodeOccupied(node);
    Require(index.occupied(x,y,-1,-1)==occupied,"coarse leaves changed occupied columns");
    Require(index.regionOccupied(x,y,x,y,-1,-1)==occupied,"coarse leaves changed block occupancy");
  }
}
}

int main() try {
  FlatHole();
  OpenAndLargeHoles();
  LayersAndSlope();
  ObservationEvidence();
  PreparedSupport();
  CoarseLeaves();
  std::cout<<"Surface completion: bounded holes, slope, layers, visibility and immutable occupancy passed\n";
  return 0;
} catch (const std::exception& error) {
  std::cerr<<error.what()<<'\n';
  return 1;
}
