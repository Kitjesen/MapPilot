#include <gtest/gtest.h>

#include "collision/body.hpp"
#include "collision/body_io.hpp"
#include "collision/terrain.hpp"
#include "planning/local/scan/grid.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <utility>
#include <vector>

namespace {
namespace c = lingtu::nav::collision;
using nav_kernel::local::scan::Grid;

class BodySupportMapTest : public ::testing::Test {
 protected:
  static constexpr double resolution = .05;
  static constexpr int size = 40;
  BodySupportMapTest() : occupied((size * size * size + 7) / 8, 0),
                         free(occupied.size(), 255) {
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
    if (state == 1) occupied[index / 8] |= mask;
    if (state == 0) free[index / 8] |= mask;
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

  void expect(bool accepted, c::Pose pose = {}) {
    auto grid = local();
    ASSERT_TRUE(grid.valid()) << grid.reason();
    EXPECT_EQ(grid.inflatedOccupancy({pose.position.x,pose.position.y,pose.position.z},pose.yaw) == 0,
              accepted);
  }

  std::vector<std::uint8_t> occupied, free;
  std::shared_ptr<c::Body> body;
  nav_kernel::LocalPlannerParams params;
  nav_kernel::LocalPlanRequest request;
};

TEST_F(BodySupportMapTest, DesignatedFootContactAndSweepsAreAccepted) {
  expect(true);
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

  const auto bitmap = occupied;
  auto check = [&](const std::vector<c::Vec3>& points) {
    auto terrain = std::make_shared<const c::Terrain>(points,resolution);
    auto local_request = request;
    local_request.environment.collision.terrain = terrain;
    const Grid refined(params,local_request);
    EXPECT_TRUE(refined.valid()) << refined.reason();
    return refined.inflatedOccupancy({},0);
  };
  const auto flat = check(hits);
  EXPECT_EQ(flat,0);

  hits.push_back({.025,.025,.045});
  const auto obstructed = check(hits);
  EXPECT_NE(obstructed,0);
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
    return refined.inflatedOccupancy({},0);
  };
  const auto floor=check();
  EXPECT_EQ(floor,0);
  hits.push_back({.115,.025,.045});
  const auto obstacle=check();
  EXPECT_NE(obstacle,0);
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
    return grid.inflatedOccupancy({},0);
  };
  EXPECT_EQ(check({{{.025,.025,.002},{}}}),0);
  EXPECT_EQ(check({{{.025,.025,.045},{}}}),1);
  // The hit is above the foot's lowest point but outside the sphere itself.
  EXPECT_EQ(check({{{.049,.049,.045},{}}}),0);
  EXPECT_EQ(check({{{.025,.025,.002},{}},{{.025,.025,.045},{}}}),1);
  EXPECT_EQ(check({{{.075,.025,.002},{}}}),1);
  EXPECT_EQ(check({}),1);
}

TEST_F(BodySupportMapTest, OriginalSlopeHitsRefineLocalVoxelCollision) {
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
  EXPECT_NE(local().inflatedOccupancy({0,0,0},0), 0);
  auto local_request = request;
  local_request.environment.collision.terrain = terrain;
  const Grid refined(params, local_request);
  ASSERT_TRUE(refined.valid()) << refined.reason();
  EXPECT_EQ(refined.inflatedOccupancy({0,0,0},0), 0);
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
  // Motion is parallel to the edge; the true footprint never crosses it.
  EXPECT_EQ(local().segmentInflatedOccupancy({0,0,0},0,{0,.004,0},0),0);
}

TEST_F(BodySupportMapTest, LargeCollisionMarginStillAllowsContactWithObservedFlatFloor) {
  body->margin_m = .16;
  expect(true);
}

TEST_F(BodySupportMapTest, MarginOnlySideWallStillCollides) {
  // The wall lies outside the sphere but within its .02 m collision margin.
  const c::Aabb wall{{.125,.025,.025},{.025,.025,.025}};
  ASSERT_FALSE(c::intersects(body->parts[0],{},wall,0));
  ASSERT_TRUE(c::intersects(body->parts[0],{},wall,body->margin_m));
  set(22,20,10,1);
  EXPECT_EQ(local().inflatedOccupancy({0,0,0},0),1);
}

TEST_F(BodySupportMapTest, ObstacleAboveTheFootBottomStillCollides) {
  set(20,20,9,0);
  set(20,20,11,1);
  expect(false);
}

TEST_F(BodySupportMapTest, LocalUnknownAirStillRejectsNewMeasuredObstacle) {
  set(22,20,11,-1);
  const auto evidence = free;
  EXPECT_EQ(local().inflatedOccupancy({0,0,0},0),0);
  EXPECT_EQ(*request.environment.collision.knownFreeStorage,evidence);
  set(22,20,11,1);
  EXPECT_EQ(local().inflatedOccupancy({0,0,0},0),1);
}

TEST_F(BodySupportMapTest, CurrentTerrainEvidenceControlsMeasuredCellCollision) {
  body->margin_m = 0;
  body->parts.front().center.z = .136;
  set(20,20,10,1);
  const auto floor_terrain = std::make_shared<const c::Terrain>(
      std::vector<c::Aabb>{{{.025,.025,.002},{}}},resolution);
  (void)local();
  for (const auto &live_terrain : {
      std::shared_ptr<const c::Terrain>{},
      std::make_shared<const c::Terrain>(
          std::vector<c::Aabb>{{{.025,.025,.045},{}}},resolution)}) {
    request.environment.collision.terrain = live_terrain;
    EXPECT_EQ(Grid(params,request).inflatedOccupancy({},0),1);
  }
  request.environment.collision.terrain = floor_terrain;
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({},0),0);
}

TEST_F(BodySupportMapTest, MeasuredBlockRetainsItsCollisionBoundary) {
  body->margin_m = 0;
  auto &part = body->parts.front();
  part.shape = c::Shape::Box;
  part.half_size = {.005,.005,.005};
  part.center = {};
  part.support_contact = false;
  std::fill(occupied.begin(),occupied.end(),0);
  for (int x=20;x<28;++x) for (int y=20;y<28;++y) for (int z=10;z<18;++z)
    set(x,y,z,1);
  expect(false,{{.395,.025,.025},0});
  expect(false,{{.405,.025,.025},0});
  expect(true,{{.406,.025,.025},0});
}

TEST_F(BodySupportMapTest, ClearEndpointHeadingsCannotSkipTheOccupiedTurnMiddle) {
  body->margin_m = 0;
  auto &part = body->parts.front();
  part.shape = c::Shape::Box;
  part.half_size = {.35,.04,.04};
  part.center = {};
  part.support_contact = false;
  set(24,24,20,1);
  const double quarter_turn = std::acos(-1.) / 2;
  const c::Pose start{{0,0,.5},0}, end{{0,0,.5},quarter_turn};
  expect(true,start);
  expect(true,end);
  expect(false,{start.position,quarter_turn*.5});
  EXPECT_EQ(local().segmentInflatedOccupancy({0,0,.5},0,{0,0,.5},quarter_turn),1);
}

TEST_F(BodySupportMapTest, ThunderRaisedRearSensorPreservesFrontOpening) {
  std::ifstream input(LINGTU_TEST_THUNDER_BODY);
  ASSERT_TRUE(input.is_open());
  body = c::readBody(input);
  body->margin_m = .02;
  body->horizontal_margin_m = .08;
  request.body = body;
  // Both cells sit above the main body at the nominal base height.
  set(26,24,23,1);
  expect(true,{{0,0,.435},0});
  set(26,24,23,0);
  set(13,20,23,1);
  expect(false,{{0,0,.435},0});
}

}  // namespace
