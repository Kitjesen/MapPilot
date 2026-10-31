#include <gtest/gtest.h>

#include "collision/body.hpp"
#include "collision/terrain.hpp"
#include "planning/global/octoplanner/body_query.hpp"
#include "planning/local/scan/grid.hpp"

#include <octomap/OcTree.h>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

namespace {
namespace c = lingtu::nav::collision;
using nav_kernel::local::scan::Grid;

class BodySupportMapTest : public ::testing::Test {
 protected:
  static constexpr double resolution = .05;
  static constexpr int size = 40;
  BodySupportMapTest() : tree(resolution), occupied((size * size * size + 7) / 8, 0),
                         free(occupied.size(), 255) {
    for (int z = 0; z < size; ++z) for (int y = 0; y < size; ++y)
      for (int x = 0; x < size; ++x)
        tree.updateNode(-1 + (x + .5) * resolution, -1 + (y + .5) * resolution,
                        -.5 + (z + .5) * resolution, false, true);
    for (int x = 0; x < size; ++x) for (int y = 0; y < size; ++y)
      set(x, y, 9, 1);
    body = std::make_shared<c::Body>();
    body->margin_m = .02;
    c::Part foot;
    foot.link = "foot";
    foot.shape = c::Shape::Sphere;
    foot.radius_m = .1;
    foot.half_size = {.1,.1,.1};
    foot.center = {0,0,.1};
    foot.support_contact = true;
    body->parts.push_back(foot);
    params.scan.voxelResolution = resolution;
    params.scan.cylinderOffset = 0;
    request.body = body;
  }

  void set(int x, int y, int z, int state) {
    const std::size_t index = (static_cast<std::size_t>(z) * size + y) * size + x;
    const auto mask = static_cast<std::uint8_t>(1U << (index % 8U));
    occupied[index / 8] &= static_cast<std::uint8_t>(~mask);
    free[index / 8] &= static_cast<std::uint8_t>(~mask);
    const double px = -1 + (x + .5) * resolution;
    const double py = -1 + (y + .5) * resolution;
    const double pz = -.5 + (z + .5) * resolution;
    if (state == -1) tree.deleteNode(px, py, pz);
    else {
      auto *node = tree.updateNode(px, py, pz, state == 1, true);
      node->setLogOdds(state == 1 ? 3.5f : -3.5f);
      if (state == 1) occupied[index / 8] |= mask;
      else free[index / 8] |= mask;
    }
  }

  Grid local() {
    auto &view = request.environment.collision;
    view.inflatedStorage = std::make_shared<const std::vector<std::uint8_t>>(occupied);
    view.inflatedBits = view.inflatedStorage->data();
    view.inflatedBytes = occupied.size();
    view.measuredOccupiedStorage = std::make_shared<const std::vector<std::uint8_t>>(occupied);
    view.knownFreeStorage = std::make_shared<const std::vector<std::uint8_t>>(free);
    view.sizeX = view.sizeY = view.sizeZ = size;
    view.resolution = resolution;
    view.aabbMin = {-1,-1,-.5};
    view.aabbMax = {1,1,1.5};
    return Grid(params,request);
  }

  bool global(c::Pose pose = {}) {
    return octoplanner3d::runtime::BodyQuery(tree,*body,overlay).pose(pose).free;
  }

  void expect(bool accepted, c::Pose pose = {}) {
    EXPECT_EQ(global(pose), accepted);
    auto grid = local();
    ASSERT_TRUE(grid.valid()) << grid.reason();
    EXPECT_EQ(grid.inflatedOccupancy({pose.position.x,pose.position.y,pose.position.z},pose.yaw) == 0,
              accepted);
  }

  octomap::OcTree tree;
  std::vector<std::uint8_t> occupied, free;
  std::shared_ptr<c::Body> body;
  nav_kernel::LocalPlannerParams params;
  nav_kernel::LocalPlanRequest request;
  lingtu::nav::plan::GlobalPlanTemporaryOverlay overlay;
};

TEST_F(BodySupportMapTest, DesignatedFootContactAndSweepsAreAccepted) {
  expect(true);
  EXPECT_TRUE(octoplanner3d::runtime::BodyQuery(tree,*body,overlay).edge({{},0},{{.3,0,0},.5}));
  EXPECT_EQ(local().segmentInflatedOccupancy({0,0,0},0,{.3,0,0},.5),0);
  EXPECT_EQ(local().brakingOccupancy({{0,0,0},0},{.3,0,0},.1,1,1),0);
}

TEST_F(BodySupportMapTest, CollisionDoesNotRequirePerFootGroundObservations) {
  // Occlusion above a measured floor must not turn a clear body into a collision.
  set(20,20,10,-1);
  expect(true);
  // Ground-support admission belongs to the global route, not each body part.
  set(20,20,9,-1);
  expect(true);
  set(20,20,10,1);
  expect(false);
}

TEST_F(BodySupportMapTest, OriginalHitsDistinguishFloorAndObstacleInSameVoxel) {
  body->margin_m = 0;
  body->parts.front().center.z = .136;  // Bottom at 3.6 cm in a 0--5 cm cell.
  for (int x = 0; x < size; ++x) for (int y = 0; y < size; ++y) {
    set(x,y,9,0);
    set(x,y,10,1);
  }
  std::vector<c::Vec3> hits;
  for (int x = -40; x < 40; ++x) for (int y = -40; y < 40; ++y)
    hits.push_back({(x+.5)*.01,(y+.5)*.01,.002});
  auto grid = local();
  ASSERT_TRUE(grid.valid()) << grid.reason();
  EXPECT_NE(grid.inflatedOccupancy({},0),0);
  EXPECT_FALSE(global());

  const auto bitmap = occupied;
  auto check = [&](const std::vector<c::Vec3>& points) {
    auto terrain = std::make_shared<const c::Terrain>(points,resolution);
    auto local_request = request;
    local_request.environment.collision.terrain = terrain;
    const Grid refined(params,local_request);
    EXPECT_TRUE(refined.valid()) << refined.reason();
    return std::make_pair(refined.inflatedOccupancy({},0),
        octoplanner3d::runtime::BodyQuery(tree,*body,overlay,{},terrain).pose({}).free);
  };
  const auto flat = check(hits);
  EXPECT_EQ(flat.first,0);
  EXPECT_TRUE(flat.second);

  hits.push_back({.025,.025,.045});
  const auto obstructed = check(hits);
  EXPECT_NE(obstructed.first,0);
  EXPECT_FALSE(obstructed.second);
  EXPECT_EQ(occupied,bitmap);
}

TEST_F(BodySupportMapTest, GroundRefinementCoversTheHorizontalCollisionMargin) {
  body->margin_m=0;
  body->horizontal_margin_m=.08;
  body->parts.front().center.z=.136;
  for (int x=0;x<size;++x) for (int y=0;y<size;++y) {
    set(x,y,9,0);
    set(x,y,10,1);
  }
  std::vector<c::Vec3> hits;
  for (int x=-40;x<40;++x) for (int y=-40;y<40;++y)
    hits.push_back({(x+.5)*.01,(y+.5)*.01,.002});
  auto check=[&] {
    const auto terrain=std::make_shared<const c::Terrain>(hits,resolution);
    auto grid=local();
    EXPECT_TRUE(grid.valid()) << grid.reason();
    request.environment.collision.terrain=terrain;
    const Grid refined(params,request);
    EXPECT_TRUE(refined.valid()) << refined.reason();
    return std::make_pair(refined.inflatedOccupancy({},0),
        octoplanner3d::runtime::BodyQuery(tree,*body,overlay,{},terrain).pose({}).free);
  };
  const auto floor=check();
  EXPECT_EQ(floor.first,0);
  EXPECT_TRUE(floor.second);
  hits.push_back({.115,.025,.045});
  const auto obstacle=check();
  EXPECT_NE(obstacle.first,0);
  EXPECT_FALSE(obstacle.second);
}

TEST_F(BodySupportMapTest, ARecordedHitOutsideTheBodyDoesNotNeedASurfaceFit) {
  body->margin_m=0;
  body->parts.front().center.z=.136;
  set(20,20,10,1);
  auto setup=local();
  ASSERT_TRUE(setup.valid());
  const auto check=[&](std::vector<c::Aabb> bounds) {
    request.environment.collision.terrain=
        std::make_shared<const c::Terrain>(std::move(bounds),resolution);
    const Grid grid(params,request);
    EXPECT_TRUE(grid.valid()) << grid.reason();
    return std::make_pair(grid.inflatedOccupancy({},0),
        octoplanner3d::runtime::BodyQuery(tree,*body,overlay,{},
            request.environment.collision.terrain).pose({}).free);
  };
  EXPECT_EQ(check({{{.025,.025,.002},{}}}),std::make_pair(0,true));
  EXPECT_EQ(check({{{.025,.025,.045},{}}}),std::make_pair(1,false));
  // The hit is above the foot's lowest point but outside the sphere itself.
  EXPECT_EQ(check({{{.049,.049,.045},{}}}),std::make_pair(0,true));
  EXPECT_EQ(check({{{.025,.025,.002},{}},{{.025,.025,.045},{}}}),std::make_pair(1,false));
  EXPECT_EQ(check({{{.075,.025,.002},{}}}),std::make_pair(1,false));
  EXPECT_EQ(check({}),std::make_pair(1,false));
}

TEST_F(BodySupportMapTest, OriginalSlopeHitsResolveTheSameGlobalAndLocalVoxelRejection) {
  body->margin_m = 0;
  const double normal_z = 1 / std::hypot(1., .08);
  auto &foot = body->parts.front();
  foot.center = {.025, .025, .025 + .08 * .025 + (.1 + .01) / normal_z};
  for (int x = 0; x < size; ++x) for (int y = 0; y < size; ++y)
    for (int z = 0; z < size; ++z)
      set(x, y, z, -.5 + (z + .5) * resolution < .025 + .08 * (-1 + (x + .5) * resolution));
  std::vector<c::Vec3> hits;
  for (int x = -90; x < 90; ++x) for (int y = -90; y < 90; ++y) {
    const double px = (x + .5) * .01, py = (y + .5) * .01;
    hits.push_back({px, py, .025 + .08 * px});
  }
  const auto terrain = std::make_shared<const c::Terrain>(hits, resolution);
  EXPECT_FALSE(global());
  EXPECT_NE(local().inflatedOccupancy({0,0,0},0), 0);
  const auto global_query = [&] {
    return octoplanner3d::runtime::BodyQuery(tree, *body, overlay, {}, terrain);
  };
  EXPECT_TRUE(global_query().pose({}).free);
  auto local_request = request;
  local_request.environment.collision.terrain = terrain;
  const Grid refined(params, local_request);
  ASSERT_TRUE(refined.valid()) << refined.reason();
  EXPECT_EQ(refined.inflatedOccupancy({0,0,0},0), 0);
  EXPECT_TRUE(global_query().edge({{},0},{{.1,0,.008},0}));
  EXPECT_EQ(refined.segmentInflatedOccupancy({0,0,0},0,{.1,0,.008},0), 0);
  EXPECT_EQ(refined.brakingOccupancy({{0,0,0},0},{.4,0,0},.1,1,1), 0);
  auto rotated_request=local_request;
  rotated_request.environment.collision.gridFromPlanningYaw=std::acos(-1.)/2;
  rotated_request.robot.pose.yaw=-std::acos(-1.)/2;
  EXPECT_EQ(Grid(params,rotated_request).brakingOccupancy(
      {{0,0,0},-std::acos(-1.)/2},{.4,0,0},.1,1,1),0);

  // Collision queries do not impose a separate per-foot ground requirement.
  auto cleared_request=local_request;
  for(int x=16;x<25;++x) for(int y=16;y<25;++y)
    for(int z=7;z<=11;++z) set(x,y,z,0);
  const auto cleared_global=global_query().pose({});
  EXPECT_TRUE(cleared_global.free);
  (void)local();
  cleared_request.environment.collision=request.environment.collision;
  cleared_request.environment.collision.terrain=terrain;
  EXPECT_EQ(Grid(params,cleared_request).inflatedOccupancy({0,0,0},0),0);

  // Restore the unchanged inclined map for the independent obstacle check.
  for(int x=16;x<25;++x) for(int y=16;y<25;++y)
    for(int z=7;z<=11;++z)
      set(x,y,z,-.5+(z+.5)*resolution < .025+.08*(-1+(x+.5)*resolution));
  (void)local();
  local_request.environment.collision=request.environment.collision;

  // A real hit inside the same ground voxel cannot be erased by the plane.
  hits.push_back({.025, .025, .049});
  const auto obstructed = std::make_shared<const c::Terrain>(std::move(hits), resolution);
  const auto rejected = octoplanner3d::runtime::BodyQuery(tree, *body, overlay, {},
                                                         obstructed).pose({});
  EXPECT_FALSE(rejected.free);
  local_request.environment.collision.terrain = obstructed;
  EXPECT_NE(Grid(params,local_request).inflatedOccupancy({0,0,0},0), 0);
}

TEST_F(BodySupportMapTest, FootNameAloneNeverExcusesCollision) {
  body->parts[0].support_contact = false;
  expect(false);
}

TEST_F(BodySupportMapTest, TiltedWheelUsesItsPrimitiveBottomInsteadOfRotatedBoxCorner) {
  auto &wheel = body->parts[0];
  wheel.shape = c::Shape::Cylinder;
  wheel.radius_m = .1;
  wheel.half_length_m = .1;
  const double angle = std::acos(-1.) / 4;
  wheel.rotation = {1,0,0, 0,std::cos(angle),-std::sin(angle),
                        0,std::sin(angle),std::cos(angle)};
  wheel.center.z = .1 * (std::cos(angle) + std::sin(angle));
  expect(true);
}

TEST_F(BodySupportMapTest, OtherRobotPartsStillCollideWithTheSameFloor) {
  c::Part shin;
  shin.link = "shin";
  shin.center = {.3,0,0};
  shin.half_size = {.03,.03,.1};
  body->parts.push_back(shin);
  expect(false);
}

TEST_F(BodySupportMapTest, RealPenetrationCollidesButRaisedFeetNeedNoGroundProof) {
  expect(false,{{0,0,-.006},0});
  expect(true,{{0,0,.029},0});
  expect(true,{{0,0,.031},0});
}

TEST_F(BodySupportMapTest, WallOrStepRiserCannotBecomeSupport) {
  set(21,20,10,1);
  expect(false);
}

TEST_F(BodySupportMapTest, ContactDoesNotAddAnotherVoxelOfSideClearance) {
  // The foot ends at x=.10, its required .02 m margin ends at x=.12,
  // and this side wall begins at x=.15: the checked body has .03 m clearance.
  set(23,20,10,1);
  const c::Aabb wall{{.175,.025,.025},{.025,.025,.025}};
  ASSERT_FALSE(c::intersects(body->parts[0],{},wall,body->margin_m));
  expect(true);
}

TEST_F(BodySupportMapTest, FloorContactAtAnExactGridLineRemainsValid) {
  auto &foot = body->parts[0];
  foot.radius_m = .08;
  foot.half_size = {.08,.08,.08};
  foot.center.z = .08;
  expect(true);
}

TEST_F(BodySupportMapTest, WallTouchingTheMarginBoundaryStillCollides) {
  auto &foot = body->parts[0];
  foot.radius_m = .08;
  foot.half_size = {.08,.08,.08};
  foot.center.z = .08;
  set(22,20,11,1);
  ASSERT_TRUE(c::intersects(foot,{},{{.125,.025,.075},{.025,.025,.025}},body->margin_m));
  expect(false);
}

TEST_F(BodySupportMapTest, UnknownAtMarginBoundaryDoesNotBlockBodyCollisionQueries) {
  auto &foot = body->parts[0];
  foot.radius_m = .08;
  foot.half_size = {.08,.08,.08};
  foot.center.z = .08;
  set(22,20,11,-1);
  EXPECT_TRUE(global());
  const auto evidence = free;
  EXPECT_EQ(local().inflatedOccupancy({0,0,0},0),0);
  EXPECT_EQ(free,evidence);
  EXPECT_EQ(*request.environment.collision.knownFreeStorage,evidence);
}

TEST_F(BodySupportMapTest, StackedFloorsUseTheCurrentContactHeight) {
  for (int x = 0; x < size; ++x) for (int y = 0; y < size; ++y) set(x,y,19,1);
  expect(true);
  expect(true,{{0,0,.5},.7});
  expect(false,{{0,0,.3},0});
}

TEST_F(BodySupportMapTest, MeasuredSwingFootDoesNotRequireUnsupportedContact) {
  expect(true,{{0,0,.3},0});
  auto observed = std::make_shared<c::Body>(*body);
  observed->parts[0].center.z += .3;
  request.observedBody = observed;
  EXPECT_EQ(local().inflatedOccupancy({0,0,0},0),0);
}

TEST_F(BodySupportMapTest, JointSnapshotCannotReuseTheNominalCollisionShape) {
  set(29,20,13,1);
  request.environment.collision.resetEpoch = 11;
  request.environment.collision.generation = 37;
  const auto stable_model = request.body;
  EXPECT_EQ(local().inflatedOccupancy({0,0,0},0),0);
  // Simulate FK moving the foot sideways into a real wall while model,
  // planning frame and map generation remain identical.
  auto observed = std::make_shared<c::Body>(*body);
  observed->parts[0].center.x = .45;
  request.observedBody = observed;
  EXPECT_EQ(local().inflatedOccupancy({0,0,0},0),1);
  EXPECT_EQ(request.body,stable_model);
  EXPECT_EQ(request.environment.collision.resetEpoch,11U);
  EXPECT_EQ(request.environment.collision.generation,37U);
  request.observedBody.reset();
  EXPECT_EQ(local().inflatedOccupancy({0,0,0},0),0);
}

TEST_F(BodySupportMapTest, TemporaryHardObstacleIsNeverReclassifiedAsSupport) {
  lingtu::nav::plan::GlobalPlanBlockedRegion region;
  region.center = {0,0,0};
  region.radius_xy_m = .1;
  region.min_z = -.05;
  region.max_z = 0;
  overlay.blocked_regions.push_back(region);
  const auto result = octoplanner3d::runtime::BodyQuery(tree,*body,overlay).pose({});
  EXPECT_FALSE(result.free);
  EXPECT_EQ(result.reason,"temporary_overlay");
}

TEST_F(BodySupportMapTest, CollisionMarginDoesNotRequireGroundOutsideTheUnpaddedFootprint) {
  // The real foot ends at x=.10. The missing floor column starts there,
  // so it has no positive-area overlap with the unpadded footprint.
  set(22,20,9,0);
  body->margin_m = .08;
  expect(true);
}

TEST_F(BodySupportMapTest, SweepPaddingDoesNotCreateAnExtraGroundRequirement) {
  for (int y = 0; y < size; ++y) set(22,y,9,0);
  body->margin_m = 0;
  const octoplanner3d::runtime::BodyQuery query(tree,*body,overlay);
  ASSERT_TRUE(query.pose({}).free);
  EXPECT_TRUE(query.pose({},.008).free);
  // Motion is parallel to the edge; the true footprint never crosses it.
  EXPECT_TRUE(query.edge({{},0},{{0,.004,0},0}));
  EXPECT_EQ(local().segmentInflatedOccupancy({0,0,0},0,{0,.004,0},0),0);
}

TEST_F(BodySupportMapTest, LargeCollisionMarginStillAllowsContactWithObservedFlatFloor) {
  body->margin_m = .16;
  expect(true);
}

TEST_F(BodySupportMapTest, MarginOnlySideWallIsReportedAsCollisionInsteadOfMissingSupport) {
  // The wall lies outside the sphere but within its .02 m collision margin.
  const c::Aabb wall{{.125,.025,.025},{.025,.025,.025}};
  ASSERT_FALSE(c::intersects(body->parts[0],{},wall,0));
  ASSERT_TRUE(c::intersects(body->parts[0],{},wall,body->margin_m));
  set(22,20,10,1);
  const auto result = octoplanner3d::runtime::BodyQuery(tree,*body,overlay).pose({});
  EXPECT_FALSE(result.free);
  EXPECT_EQ(result.reason,"occupied_body");
  EXPECT_EQ(local().inflatedOccupancy({0,0,0},0),1);
}

TEST_F(BodySupportMapTest, ObstacleAboveTheFootBottomStillCollides) {
  set(20,20,9,0);
  set(20,20,11,1);
  expect(false);
}

TEST_F(BodySupportMapTest, LocalUnknownAirStillRejectsNewMeasuredObstacle) {
  set(22,20,11,-1);
  const auto result = octoplanner3d::runtime::BodyQuery(tree,*body,overlay).pose({});
  EXPECT_TRUE(result.free);
  EXPECT_TRUE(result.reason.empty());
  const auto evidence = free;
  EXPECT_EQ(local().inflatedOccupancy({0,0,0},0),0);
  EXPECT_EQ(*request.environment.collision.knownFreeStorage,evidence);
  set(22,20,11,1);
  EXPECT_EQ(local().inflatedOccupancy({0,0,0},0),1);
}

TEST_F(BodySupportMapTest, LiveFreeBodyClearingDoesNotMutateSavedMap) {
  set(22,20,10,1);
  ASSERT_FALSE(global());
  const auto result = octoplanner3d::runtime::BodyQuery(
      tree,*body,overlay,[](const c::Aabb &) { return true; }).pose({});
  EXPECT_TRUE(result.free);
  EXPECT_TRUE(tree.isNodeOccupied(tree.search(.025,.025,-.025)));
  EXPECT_TRUE(tree.isNodeOccupied(tree.search(.125,.025,.025)));
}

TEST_F(BodySupportMapTest, LiveFreeBodyClearingDoesNotOverrideTemporaryHardObstacle) {
  lingtu::nav::plan::GlobalPlanBlockedRegion region;
  region.center = {0,0,0};
  region.radius_xy_m = .1;
  region.min_z = -.05;
  region.max_z = 0;
  overlay.blocked_regions.push_back(region);
  const auto result = octoplanner3d::runtime::BodyQuery(
      tree,*body,overlay,[](const c::Aabb &) { return true; }).pose({});
  EXPECT_FALSE(result.free);
  EXPECT_EQ(result.reason,"temporary_overlay");
}

}  // namespace
