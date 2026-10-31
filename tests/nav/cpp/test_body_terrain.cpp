#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "collision/body.hpp"
#include "collision/terrain.hpp"
#include "collision/terrain.hpp"

namespace {
namespace c = lingtu::nav::collision;

std::vector<c::Vec3> plane(double offset=.13, double slope=.08) {
  std::vector<c::Vec3> points;
  for (int x=-30;x<=30;++x) for (int y=-30;y<=30;++y) {
    const double px=(x+.5)*.01, py=(y+.5)*.01;
    points.push_back({px,py,offset+slope*px});
  }
  return points;
}

TEST(BodyTerrain, FitsTheOriginalInclinedSurfaceRatherThanVoxelTopFaces) {
  c::Terrain terrain(plane(),.05);
  const auto surface=terrain.surface({.025,.025,.132},.06,.13);
  ASSERT_TRUE(surface);
  EXPECT_NEAR(surface->heightAt(.025,.025),.132,1e-12);
  EXPECT_NEAR(surface->heightAt(-.05,.05),.126,1e-12);
  const double norm=std::hypot(1.,.08);
  EXPECT_NEAR(surface->normal.x,-.08/norm,1e-12);
  EXPECT_NEAR(surface->normal.z,1./norm,1e-12);
  EXPECT_LT(surface->rms_m,1e-12);
  EXPECT_TRUE(terrain.explains({{.025,.025,.125},{.025,.025,.025}},*surface));
}

TEST(BodyTerrain, HeightConditioningPreservesBothFloorsAtTheSameXY) {
  auto points=plane();
  const auto upper=plane(1.13);
  points.insert(points.end(),upper.begin(),upper.end());
  c::Terrain terrain(std::move(points),.05);
  const auto lower=terrain.surface({.025,.025,.132},1.1,.13);
  const auto higher=terrain.surface({.025,.025,1.132},1.1,.13);
  ASSERT_TRUE(lower);
  ASSERT_TRUE(higher);
  EXPECT_NEAR(lower->heightAt(.025,.025),.132,1e-12);
  EXPECT_NEAR(higher->heightAt(.025,.025),1.132,1e-12);
  EXPECT_FALSE(terrain.surface({.025,.025,.632},.2,.13));
  EXPECT_FALSE(terrain.explains({{.025,.025,1.125},{.025,.025,.025}},*lower));
}

TEST(BodyTerrain, NeighbouringSamplesCanFitButCannotClearAnUnobservedCell) {
  auto points=plane();
  std::vector<c::Vec3> with_hole;
  for (const auto &point : points)
    if (point.x<0 || point.x>=.05 || point.y<0 || point.y>=.05)
      with_hole.push_back(point);
  c::Terrain terrain(std::move(with_hole),.05);
  const auto around_hole=terrain.surface({.025,.025,.132},.06,.13);
  ASSERT_TRUE(around_hole);
  EXPECT_FALSE(terrain.explains({{.025,.025,.125},{.025,.025,.025}},*around_hole));
  const auto beside=terrain.surface({-.025,.025,.128},.06,.13);
  ASSERT_TRUE(beside);
  EXPECT_FALSE(terrain.explains({{.025,.025,.125},{.025,.025,.025}},*beside));
}

TEST(BodyTerrain, AVerticalWallOrAnObservedStepIsNotInventedAsASmoothRamp) {
  std::vector<c::Vec3> wall;
  for (int y=-15;y<=15;++y) for (int z=-15;z<=15;++z)
    wall.push_back({.025,(y+.5)*.01,.13+(z+.5)*.01});
  EXPECT_FALSE(c::Terrain(std::move(wall),.05).surface({.025,.025,.13},.1,.13));
  auto step=plane(0,0);
  for (auto &point : step) if (point.x>=.025) point.z=.05;
  EXPECT_FALSE(c::Terrain(std::move(step),.05).surface({.025,.025,.025},.1,.18));
}

TEST(BodyTerrain, RetrievalBucketsDoNotClearAnUnobservedFineCell) {
  auto points=plane(.01,0);
  std::vector<c::Vec3> with_hole;
  for (const auto &point : points)
    if (point.x<0 || point.x>=.01 || point.y<0 || point.y>=.01)
      with_hole.push_back(point);
  const c::Terrain terrain(std::move(with_hole),.01);
  const auto around_hole=terrain.surface({.005,.005,.01},.03,.1);
  ASSERT_TRUE(around_hole);
  EXPECT_FALSE(terrain.explains({{.005,.005,.01},{.005,.005,.005}},*around_hole));
  EXPECT_TRUE(terrain.surface({.015,.005,.01},.03,.1));
}

TEST(BodyTerrain, ARealBumpInTheSameVoxelPreventsTheGroundCellExemption) {
  const auto points=plane();
  c::Terrain original(points,.05);
  const auto surface=original.surface({.025,.025,.132},.06,.13);
  ASSERT_TRUE(surface);
  const c::Aabb cell{{.025,.025,.125},{.025,.025,.025}};
  ASSERT_TRUE(original.explains(cell,*surface));
  auto obstructed=points;
  obstructed.push_back({.025,.025,.145});
  c::Terrain bumped(std::move(obstructed),.05);
  EXPECT_FALSE(bumped.explains(cell,*surface));
  EXPECT_FALSE(original.explains({{.025,.025,.175},{.025,.025,.025}},*surface));
  EXPECT_FALSE(original.explains({{.5,.5,.175},{.025,.025,.025}},*surface));
}

TEST(BodyTerrain, TruePrimitiveClearanceCanResolveAnIntersectingGroundCube) {
  c::Terrain terrain(plane(),.05);
  const auto surface=terrain.surface({.025,.025,.132},.06,.13);
  ASSERT_TRUE(surface);
  c::Part calf;
  calf.shape=c::Shape::Cylinder;
  calf.radius_m=.01;
  calf.half_length_m=.05;
  calf.half_size={.01,.01,.05};
  constexpr double clearance=.0147683698159;
  const double support=.05*surface->normal.z+.01*std::abs(surface->normal.x);
  calf.center={.025,.025,.132+(support+clearance)/surface->normal.z};
  const c::Aabb voxel{{.025,.025,.125},{.025,.025,.025}};
  ASSERT_TRUE(c::intersects(calf,{},voxel));
  EXPECT_NEAR(c::primitivePlaneClearance(calf,{},*surface),clearance,1e-12);
  EXPECT_TRUE(terrain.explains(voxel,*surface));
  calf.center.z-=.02;
  EXPECT_LT(c::primitivePlaneClearance(calf,{},*surface),0);
}

TEST(BodyTerrain, HorizontalMarginRetainsSlopeCollisionWithoutAddingFlatFloorPadding) {
  c::Terrain terrain(plane(.13,.4),.05);
  const auto surface=terrain.surface({.025,.025,.14},.06,.13);
  ASSERT_TRUE(surface);
  c::Part box;
  box.half_size={.01,.01,.01};
  box.center={.025,.025,.17};
  const c::Aabb cell{{.025,.025,.125},{.025,.025,.025}};
  ASSERT_TRUE(terrain.explains(cell,*surface));
  EXPECT_TRUE(c::terrainCellClear(terrain,box,{},0,cell,*surface));
  EXPECT_FALSE(c::terrainCellClear(terrain,box,{},0,cell,*surface,.08));
  c::Terrain flat(plane(.13,0),.05);
  const auto floor=flat.surface({.025,.025,.13},.06,.13);
  ASSERT_TRUE(floor);
  EXPECT_TRUE(c::terrainCellClear(flat,box,{},0,cell,*floor,.08));
}

TEST(BodyTerrain, ASubVoxelBumpKeepsItsPositiveHeightAboveTheFittedPlane) {
  auto points=plane(.01,0);
  const c::Vec3 bump{.025,.025,.014};
  points.push_back(bump);
  c::Terrain terrain(std::move(points),.05);
  const auto surface=terrain.surface({.025,.025,.01},.06,.13);
  ASSERT_TRUE(surface);
  c::Part sphere;
  sphere.shape=c::Shape::Sphere;
  sphere.radius_m=.01;
  sphere.half_size={.01,.01,.01};
  sphere.center={.025,.025,.022};
  const c::Aabb cell{{.025,.025,.025},{.025,.025,.025}};
  double actual_rise=0;
  ASSERT_TRUE(terrain.explains(cell,*surface,&actual_rise));
  const double clearance=c::primitivePlaneClearance(sphere,{},*surface);
  EXPECT_GT(clearance,0);
  EXPECT_GT(actual_rise,clearance);
  EXPECT_GE(surface->max_positive_residual_m,actual_rise-1e-12);
  EXPECT_LT(std::abs(sphere.center.z-bump.z),sphere.radius_m);
  EXPECT_LE(clearance-actual_rise,0);
  EXPECT_FALSE(c::terrainCellClear(terrain,sphere,{},0,cell,*surface));
}

TEST(BodyTerrain, EachMeasuredContactSelectsTheLayerAtItsOwnFootHeight) {
  auto points=plane(0,0);
  const auto upper=plane(.05,0);
  points.insert(points.end(),upper.begin(),upper.end());
  const c::Terrain terrain(std::move(points),.05);
  c::Part foot;
  foot.shape=c::Shape::Sphere;
  foot.radius_m=.025;
  foot.half_size={.025,.025,.025};
  foot.center={.025,.025,.075};
  foot.support_contact = true;
  const auto surface=c::partTerrainSurface(&terrain,foot,{},0,.05,0,0);
  ASSERT_TRUE(surface);
  EXPECT_NEAR(surface->heightAt(.025,.025),.05,1e-12);
  EXPECT_NEAR(c::primitivePlaneClearance(foot,{},*surface),0,1e-12);
}

TEST(BodyTerrain, PrimitiveSupportFunctionsRetainTheDeclaredBodyPadding) {
  const c::TerrainSurface level{{0,0,1},0,{},0};
  c::Part part;
  part.center={0,0,.4};
  part.half_size={.1,.2,.3};
  part.radius_m=.1;
  part.half_length_m=.2;
  for (const auto shape : {c::Shape::Box,c::Shape::Sphere,c::Shape::Cylinder,c::Shape::Capsule}) {
    part.shape=shape;
    const double expected=shape==c::Shape::Sphere ? .3 : shape==c::Shape::Cylinder ? .2 : .1;
    EXPECT_NEAR(c::primitivePlaneClearance(part,{},level),expected,1e-12);
    EXPECT_NEAR(c::primitivePlaneClearance(part,{},level,.02),expected-.02,1e-12);
  }
  const c::TerrainSurface inclined{{0,-.6,.8},0,{},0};
  part.shape=c::Shape::Box;
  const double unpadded=c::primitivePlaneClearance(part,{},inclined);
  EXPECT_NEAR(unpadded-c::primitivePlaneClearance(part,{},inclined,.02),.028,1e-12);
  part.shape=c::Shape::Cylinder;
  const double cylinder=c::primitivePlaneClearance(part,{},inclined);
  EXPECT_NEAR(cylinder-c::primitivePlaneClearance(part,{},inclined,.02),.028,1e-12);
  part.shape=c::Shape::Capsule;
  const double capsule=c::primitivePlaneClearance(part,{},inclined);
  EXPECT_NEAR(capsule-c::primitivePlaneClearance(part,{},inclined,.02),.02,1e-12);
}

TEST(BodyTerrain, ClearanceUsesTheActualPartRotationBodyYawAndWorldTranslation) {
  c::Part part;
  part.shape=c::Shape::Box;
  part.center={.1,.2,.5};
  part.half_size={.1,.2,.3};
  // A quarter roll interchanges the source Y and Z half dimensions.
  part.rotation={1,0,0,0,0,-1,0,1,0};
  const double pi=std::acos(-1.);
  const c::Pose pose{{.3,.4,.2},pi/2};
  const c::TerrainSurface surface{{0,-.6,.8},-.1,{},0};
  // World centre=(.1,.5,.7); world support=.6*.1+.8*.2.
  EXPECT_NEAR(c::primitivePlaneClearance(part,pose,surface),-.6*.5+.8*.7-.1-.22,1e-12);
}

TEST(BodyTerrain, InvalidSampleResolutionIsRejectedAndMissingObservationsStayMissing) {
  EXPECT_THROW(c::Terrain(std::vector<c::Vec3>{},0),std::invalid_argument);
  c::Terrain empty(std::vector<c::Vec3>{{0,0,std::numeric_limits<double>::quiet_NaN()}},.05);
  EXPECT_FALSE(empty.surface({0,0,0},.1,.1));
}

TEST(BodyTerrain, UsesCompleteMeasuredHitBoundsWithinOccupiedVoxel) {
  c::Part body;
  body.shape=c::Shape::Box;
  body.center={.025,.025,.025};
  body.half_size={.01,.01,.01};
  const auto part=c::preparePart(body,{},0,0);
  const c::Aabb voxel{{.025,.025,.025},{.025,.025,.025}};
  ASSERT_TRUE(c::intersects(part,voxel));

  // The occupied voxel touches the body, but its measured hit is outside it.
  const c::Terrain clear(std::vector<c::Aabb>{{{.049,.049,.025},{}}},.05);
  EXPECT_TRUE(clear.hitBoundsClear(part,voxel));

  // A hit inside the body, including a newly seen obstacle, remains blocked.
  const c::Terrain obstacle(std::vector<c::Aabb>{{{.025,.025,.025},{}}},.05);
  EXPECT_FALSE(obstacle.hitBoundsClear(part,voxel));
  const auto padded=c::preparePart(body,{},0,.02);
  EXPECT_FALSE(clear.hitBoundsClear(padded,voxel));
  const c::Terrain spanning(std::vector<c::Aabb>{{{.035,.035,.025},{.014,.014,0}}},.05);
  EXPECT_FALSE(spanning.hitBoundsClear(part,voxel));
  const c::Terrain absent(std::vector<c::Aabb>{},.05);
  EXPECT_FALSE(absent.hitBoundsClear(part,voxel));
}

}  // namespace
