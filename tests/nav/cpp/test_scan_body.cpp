#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <memory>
#include <vector>

#include "collision/body.hpp"
#include "collision/body_io.hpp"
#include "collision/terrain.hpp"
#include "collision_bitmap.hpp"
#include "planning/local/scan/grid.hpp"
#include "planning/local/scan/upstream/bspline_opt/bspline_optimizer.h"
#include "planning/local/scan/upstream/plan_env/grid_map.h"

namespace {
namespace c = lingtu::nav::collision;
using nav_kernel::LocalPlannerParams;
using nav_kernel::LocalPlanRequest;
using nav_kernel::Vec3;
using nav_kernel::local::scan::Grid;
using lingtu::nav::tests::CollisionBitmap;

class ScanBodyTest : public ::testing::Test {
 protected:
  ScanBodyTest() : raw({-2,-2,-1}, {2,2,2}, .1), inflated({-2,-2,-1}, {2,2,2}, .1) {
    params.scan.voxelResolution = .1;
    params.scan.cylinderOffset = 0;
    body = std::make_shared<c::Body>();
    body->margin_m = 0;
    c::Part part;
    part.link = "base";
    part.half_size = {.3,.1,.1};
    body->parts.push_back(part);
    request.body = body;
    request.robot.pose = {{0,0,.5},0};
  }

  void snapshot() {
    request.environment.collision = inflated.view();
    const auto view = raw.view();
    request.environment.collision.measuredOccupiedStorage =
        std::make_shared<const std::vector<std::uint8_t>>(view.inflatedBits,
                                                        view.inflatedBits + view.inflatedBytes);
    request.environment.collision.knownFreeStorage =
        std::make_shared<const std::vector<std::uint8_t>>(view.inflatedBytes, 255U);
  }

  LocalPlannerParams params;
  LocalPlanRequest request;
  std::shared_ptr<c::Body> body;
  CollisionBitmap raw;
  CollisionBitmap inflated;
};

TEST_F(ScanBodyTest, UsesRawEvidenceWithoutInflatingBodyTwice) {
  inflated.occupy({0,0,.5});
  snapshot();
  Grid grid(params, request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  EXPECT_EQ(grid.inflatedOccupancy({0,0,.5},0),0);
  request.body.reset();
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({0,0,.5},0),1);
}

TEST_F(ScanBodyTest, ThunderCompoundClearsGroundAndRejectsBodyObstacles) {
  std::ifstream input(LINGTU_TEST_THUNDER_BODY);
  ASSERT_TRUE(input.is_open());
  body = c::readBody(input);
  ASSERT_EQ(body->parts.size(), 8U);
  body->margin_m = 0;
  body->horizontal_margin_m = .08;
  request.body = body;
  const Vec3 center{0, 0, .435};
  for (int y = -8; y < 8; ++y) for (int x = -8; x < 8; ++x)
    raw.occupy({(x + .5) * .1, (y + .5) * .1, -.05});
  snapshot();
  EXPECT_EQ(Grid(params, request).inflatedOccupancy(center, 0), 0);

  raw.occupy({.45, .05, .45});
  snapshot();
  EXPECT_EQ(Grid(params, request).inflatedOccupancy(center, 0), 1);

  raw.clear();
  raw.occupy({.05, .35, .45});
  snapshot();
  EXPECT_EQ(Grid(params, request).inflatedOccupancy(center, 0), 1);

  raw.clear();
  raw.occupy({-.35, .05, .65});
  snapshot();
  EXPECT_EQ(Grid(params, request).inflatedOccupancy(center, 0), 1);

  raw.clear();
  raw.occupy({.05, .45, .45});
  snapshot();
  EXPECT_EQ(Grid(params, request).inflatedOccupancy(center, 0), 0);
  EXPECT_EQ(Grid(params, request).inflatedOccupancy(center, std::acos(-1.) / 2), 1);

  // The raised rear sensor does not fill the space over the front shoulders.
  const c::Aabb upper_front{{.32, .20, .64}, {.025, .025, .025}};
  for (const auto& part : body->parts) {
    EXPECT_FALSE(part.support_contact);
    EXPECT_FALSE(c::intersects(part, {{0, 0, .435}, 0}, upper_front, 0, .08));
  }
}

TEST_F(ScanBodyTest, ThunderMarginComparisonUsesSameMeasuredAndPredictedObstacles) {
  std::ifstream input(LINGTU_TEST_THUNDER_BODY);
  ASSERT_TRUE(input.is_open());
  body = c::readBody(input);
  ASSERT_EQ(body->parts.size(), 8U);
  body->margin_m = 0;
  request.body = body;
  raw = CollisionBitmap({-2,-2,-1},{2,2,2},.05);
  inflated = CollisionBitmap({-2,-2,-1},{2,2,2},.05);
  params.scan.voxelResolution = .05;
  const Vec3 center{0,.03,.435};
  for (const double margin : {0.0,.04,.08}) {
    raw.clear();
    for (double x=-.575; x<=.575; x+=.05) {
      raw.occupy({x,.425,.425});
      raw.occupy({x,-.425,.425});
    }
    snapshot();
    body->horizontal_margin_m = margin;
    const Grid corridor(params,request);
    ASSERT_TRUE(corridor.valid()) << corridor.reason();
    const int straight = corridor.segmentInflatedOccupancy(
        {-.3,.03,.435},0,{.3,.03,.435},0);
    const int turn = corridor.segmentInflatedOccupancy(
        center,0,center,.12);
    raw.clear();
    snapshot();
    const nav_kernel::PredictedObstacle crossing{{0,-.8,.435},{0,.8,.435},.05,.3,.6};
    request.clock.timestampS=1;
    request.environment.predictions={&crossing,1,4,1,1};
    const Grid dynamic(params,request);
    const int moving = dynamic.trajectoryOccupancy(center,{},center,{},0,1);
    const int after = dynamic.trajectoryOccupancy(center,{},center,{},1,2);
    request.environment.predictions={};
    std::cout << "Thunder margin " << margin << " corridor=" << straight
              << " turn=" << turn << " crossing=" << moving << '\n';
    EXPECT_EQ(straight, margin == .08 ? 1 : 0);
    EXPECT_EQ(turn, margin == 0.0 ? 0 : 1);
    EXPECT_EQ(moving,1);
    EXPECT_EQ(after,0);
  }
}

TEST_F(ScanBodyTest, HorizontalMarginRejectsWallWithoutAddingVerticalClearance) {
  body->parts.front().half_size = {.2, .1, .1};
  raw.occupy({.35, .05, .55});
  snapshot();
  body->horizontal_margin_m = .05;
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({0,0,.5},0),0);
  body->horizontal_margin_m = .12;
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({0,0,.5},0),1);
  raw.clear();
  raw.occupy({.05, .05, .25});
  snapshot();
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({0,0,.5},0),0);
  raw.occupy({.05, .05, .45});
  snapshot();
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({0,0,.5},0),1);
}

TEST_F(ScanBodyTest, UsesActualHitBoundsWithoutLosingTheObstacleMargin) {
  raw.occupy({.35,.05,.55});
  snapshot();
  body->horizontal_margin_m = .02;
  auto& terrain = request.environment.collision.terrain;
  terrain = std::make_shared<const c::Terrain>(
      std::vector<c::Aabb>{{{.39,.05,.55},{}}}, .1);
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({0,0,.5},0),0);
  terrain = std::make_shared<const c::Terrain>(
      std::vector<c::Aabb>{{{.31,.05,.55},{}}}, .1);
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({0,0,.5},0),1);
  terrain.reset();
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({0,0,.5},0),1);
}

TEST_F(ScanBodyTest, TwinCylindersKeepUpstreamInflatedQueryWithHitBounds) {
  request.body.reset();
  params.scan.cylinderOffset = 0;
  params.scan.cylinderRadius = .32;
  raw.occupy({.35,.05,.55});
  inflated.occupy({0,0,.5});
  snapshot();
  request.environment.collision.terrain = std::make_shared<const c::Terrain>(
      std::vector<c::Aabb>{{{.39,.05,.55},{}}}, .1);
  const Grid grid(params,request);
  EXPECT_EQ(grid.cylinderOccupancy({0,0,.5},0),1);
  EXPECT_EQ(grid.brakingOccupancy({{0,0,.5},0},{},.35,4,4),1);
}

TEST_F(ScanBodyTest, HorizontalMarginKeepsFootFloorContactAndRejectsSideObstacle) {
  auto& foot = body->parts.front();
  foot.shape = c::Shape::Sphere;
  foot.radius_m = .03;
  foot.half_size = {.03,.03,.03};
  foot.support_contact = true;
  body->horizontal_margin_m = .15;
  for(int y=-5;y<5;++y) for(int x=-5;x<5;++x)
    raw.occupy({(x+.5)*.1,(y+.5)*.1,-.05});
  snapshot();
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({0,0,.03},0),0);
  raw.occupy({.15,.05,.05});
  snapshot();
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({0,0,.03},0),1);
}

TEST_F(ScanBodyTest, HorizontalMarginRemainsInsideLocalQueryCoverage) {
  snapshot();
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({1.65,0,.5},0),0);
  body->horizontal_margin_m = .1;
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({1.65,0,.5},0),-1);
}

TEST_F(ScanBodyTest, InflatedStartDoesNotTrapAClearBodyDepartureOrBraking) {
  raw.occupy({-.35,.05,.55});
  inflated.occupyInflated({-.35F,.05F,.55F},.5,.2,.2);
  snapshot();
  const auto measured = *request.environment.collision.measuredOccupiedStorage;
  const auto known = *request.environment.collision.knownFreeStorage;
  const nav_kernel::Pose start{{.0005,0,.5},0};
  Grid grid(params,request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  EXPECT_EQ(grid.inflatedOccupancy(start.position,0),0);
  EXPECT_EQ(grid.segmentInflatedOccupancy(start.position,0,{.2005,0,.5},0),0);
  EXPECT_EQ(grid.segmentInflatedOccupancy({.2005,0,.5},0,start.position,0),0);
  EXPECT_TRUE(grid.boundaryDepartureFree(start,{.2005,0,.5}));
  EXPECT_EQ(grid.brakingOccupancy(start,{.2,0,0},.1,1,1),0);
  EXPECT_DOUBLE_EQ(grid.brakingScale(start,{.2,0,0},.1,1,1),1.0);
  EXPECT_EQ(*request.environment.collision.measuredOccupiedStorage,measured);
  EXPECT_EQ(*request.environment.collision.knownFreeStorage,known);
  request.body.reset();
  EXPECT_EQ(Grid(params,request).inflatedOccupancy(start.position,0),1);
}

TEST_F(ScanBodyTest, DepartureCannotExcuseMeasuredContactAtEitherEndpoint) {
  raw.occupy({-.35,.05,.55});
  snapshot();
  Grid grid(params,request);
  EXPECT_EQ(grid.segmentInflatedOccupancy({-.001,0,.5},0,{.2,0,.5},0),1);
  EXPECT_EQ(grid.segmentInflatedOccupancy({.2,0,.5},0,{-.001,0,.5},0),1);
  EXPECT_FALSE(grid.boundaryDepartureFree({{-.001,0,.5},0},{.2,0,.5}));
  EXPECT_EQ(grid.brakingOccupancy({{.0005,0,.5},0},{-.2,0,0},.1,1,1),1);
}

TEST_F(ScanBodyTest, ClearanceOnlyDepartureChecksTheWholeExitWithoutChangingNormalQueries) {
  body->parts.front().half_size = {.2,.1,.1};
  body->horizontal_margin_m = .08;
  raw.occupy({-.25,.05,.55});
  snapshot();
  const auto evidence = *request.environment.collision.measuredOccupiedStorage;
  const nav_kernel::Pose start{{.0005,0,.5},0};
  Grid grid(params,request);
  ASSERT_EQ(grid.inflatedOccupancy(start.position,0),1);
  EXPECT_EQ(grid.brakingOccupancy(start,{.1,0,0},.1,1,1),1);
  EXPECT_TRUE(grid.boundaryDepartureFree(start,{.2005,0,.5}));
  EXPECT_TRUE(grid.boundaryDepartureMotionFree(start,{.1,0,0},{},.2,.1,1));
  EXPECT_FALSE(grid.boundaryDepartureFree(start,{.0105,0,.5}));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(start,{.1,0,0},{-.1,0,0},.2,.1,1));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(start,{.1,0,.2},{},.2,.1,1));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(start,{.5,0,0},{},.2,.1,1));
  EXPECT_EQ(grid.inflatedOccupancy(start.position,0),1);
  EXPECT_EQ(*request.environment.collision.measuredOccupiedStorage,evidence);
  EXPECT_DOUBLE_EQ(body->horizontal_margin_m,.08);
}

TEST_F(ScanBodyTest, ClearanceDepartureRejectsSourceCollisionAndNonplanarOrLongExits) {
  body->parts.front().half_size = {.2,.1,.1};
  body->horizontal_margin_m = .08;
  raw.occupy({-.15,.05,.55});
  snapshot();
  EXPECT_FALSE(Grid(params,request).boundaryDepartureFree({{0,0,.5},0},{.3,0,.5}));
  raw.clear();
  raw.occupy({-.25,.05,.55});
  snapshot();
  Grid grid(params,request);
  EXPECT_FALSE(grid.boundaryDepartureFree({{0,0,.5},0},{0,0,.6}));
  EXPECT_FALSE(grid.boundaryDepartureFree({{0,0,.5},0},{.6,0,.5}));
}

TEST_F(ScanBodyTest, ClearanceDepartureSharesTheInitialSweepCoverage) {
  body->parts.front().half_size = {.2,.1,.099};
  body->horizontal_margin_m = .08;
  raw.occupy({-.25,.05,.55});
  raw.occupy({-.25,.05,.65});
  snapshot();
  const auto evidence = *request.environment.collision.measuredOccupiedStorage;
  const nav_kernel::Pose start{{.0005,0,.5},0};
  Grid grid(params,request);
  ASSERT_EQ(grid.inflatedOccupancy(start.position,0),1);
  ASSERT_EQ(grid.inflatedOccupancy({.3505,0,.5},0),0);
  EXPECT_TRUE(grid.boundaryDepartureFree(start,{.3505,0,.5}));
  EXPECT_TRUE(grid.boundaryDepartureMotionFree(start,{.1,0,0},{},.2,.1,1));
  EXPECT_FALSE(grid.boundaryDepartureFree(start,{-.3495,0,.5}));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(start,{.1,0,0},{-.1,0,0},.2,.1,1));
  EXPECT_EQ(*request.environment.collision.measuredOccupiedStorage,evidence);
}

TEST_F(ScanBodyTest, ClearStartCannotExcludeNearbyBufferDuringMeasuredBraking) {
  body->parts.front().half_size = {.3,.11,.1};
  body->horizontal_margin_m = .08;
  raw.occupy({-.45,-.15,.55});
  snapshot();
  const nav_kernel::Pose start{{-.019,0,.5},0};
  const nav_kernel::Twist measured{0,0,-.08};
  Grid grid(params,request);
  ASSERT_EQ(grid.inflatedOccupancy(start.position,0),0);
  ASSERT_TRUE(grid.boundaryDepartureFree(start,{.331,0,.5}));
  ASSERT_EQ(grid.brakingOccupancy(start,measured,.35,.5,1),1);
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(start,{.1,0,0},measured,.2,.35,.5,1));
}

TEST_F(ScanBodyTest, ClearanceDepartureCannotEnterAnotherObstaclesBuffer) {
  body->parts.front().half_size = {.05,.05,.05};
  body->horizontal_margin_m = .08;
  raw.occupy({-.15,.05,.55});
  snapshot();
  const nav_kernel::Pose start{{0,0,.5},0};
  ASSERT_TRUE(Grid(params,request).boundaryDepartureFree(start,{.5,0,.5}));
  raw.occupy({.25,.15,.55});
  snapshot();
  Grid grid(params,request);
  ASSERT_EQ(grid.inflatedOccupancy({.5,0,.5},0),0);
  EXPECT_FALSE(grid.boundaryDepartureFree(start,{.5,0,.5}));
}

TEST_F(ScanBodyTest, ClearanceDepartureRetainsMeasuredHitGeometryAndDynamicObstacles) {
  body->parts.front().half_size = {.2,.1,.1};
  body->horizontal_margin_m = .08;
  raw.occupy({-.25,.05,.55});
  snapshot();
  request.environment.collision.terrain = std::make_shared<const c::Terrain>(
      std::vector<c::Aabb>{{{-.201,.05,.55},{}}},.1);
  const nav_kernel::Pose start{{0,0,.5},0};
  ASSERT_TRUE(Grid(params,request).boundaryDepartureFree(start,{.3,0,.5}));
  const nav_kernel::PredictedObstacle beside{{.15,.23,.5},{.15,.23,.5},.01,.4,.6};
  request.clock.timestampS = 1;
  request.bodyHeading = std::acos(-1.) / 2;
  request.environment.predictions = {&beside,1,4,1,1};
  Grid heldHeading(params,request);
  EXPECT_TRUE(heldHeading.predictionIntersects(start.position,0,{.3,0,.5},0));
  EXPECT_TRUE(heldHeading.boundaryDepartureFree(start,{.3,0,.5}));
  EXPECT_TRUE(heldHeading.boundaryDepartureMotionFree(start,{.1,0,0},{},.2,.1,1));
  const nav_kernel::PredictedObstacle crossing{{.3,-.4,.5},{.3,.4,.5},.04,.4,.6};
  request.environment.predictions = {&crossing,1,4,1,1};
  EXPECT_FALSE(Grid(params,request).boundaryDepartureFree(start,{.3,0,.5}));
}

TEST_F(ScanBodyTest, ClearanceDepartureChecksMeasuredYawDuringReactionAndBraking) {
  body->parts.front().half_size = {.3,.094,.1};
  body->horizontal_margin_m = .08;
  raw.occupy({-.15,.15,.55});
  snapshot();
  const nav_kernel::Pose start{{0,0,.5},0};
  const nav_kernel::Twist moving{.1,0,-.08};
  Grid grid(params,request);
  ASSERT_TRUE(grid.boundaryDepartureFree(start,{.35,0,.5}));
  const auto padded = request.body;
  auto source = std::make_shared<c::Body>(*body);
  source->horizontal_margin_m = 0;
  request.body = source;
  ASSERT_EQ(Grid(params,request).brakingOccupancy(start,moving,.35,.5,1),1);
  request.body = padded;
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(start,{.1,0,0},moving,.2,.35,.5));
  EXPECT_TRUE(grid.boundaryDepartureMotionFree(start,{.1,0,0},{.1,0,0},.2,.35,.5));
}

TEST_F(ScanBodyTest, ClearanceDepartureBrakingKeepsNewObstacleBuffersAndPredictions) {
  body->parts.front().half_size = {.3,.11,.1};
  body->horizontal_margin_m = .08;
  raw.occupy({-.35,.05,.55});
  snapshot();
  const nav_kernel::Pose start{{.01,0,.5},0};
  const nav_kernel::Twist moving{.1,0,-.08};
  ASSERT_TRUE(Grid(params,request).boundaryDepartureMotionFree(start,{.1,0,0},moving,.2,.35,.5,1));
  raw.occupy({.45,-.25,.55});
  snapshot();
  Grid newBuffer(params,request);
  ASSERT_TRUE(newBuffer.boundaryDepartureFree(start,{.36,0,.5}));
  EXPECT_FALSE(newBuffer.boundaryDepartureMotionFree(start,{.1,0,0},moving,.2,.35,.5,1));
  raw.clear();
  raw.occupy({-.35,.05,.55});
  snapshot();
  const nav_kernel::PredictedObstacle beside{{.4,-.2,.5},{.4,-.2,.5},.001,.4,.6};
  request.clock.timestampS = 1;
  request.environment.predictions = {&beside,1,4,1,1};
  Grid predicted(params,request);
  ASSERT_TRUE(predicted.boundaryDepartureFree(start,{.36,0,.5}));
  EXPECT_FALSE(predicted.boundaryDepartureMotionFree(start,{.1,0,0},moving,.2,.35,.5,1));
}

TEST_F(ScanBodyTest, ClearanceDepartureRetainsTheCoupledStopDistance) {
  body->parts.front().half_size = {.2,.05,.1};
  body->horizontal_margin_m = .08;
  raw.occupy({-.25,.05,.55});
  snapshot();
  const nav_kernel::Pose start{{.01,0,.5},0};
  const nav_kernel::Twist moving{.1,0,-.08};
  ASSERT_TRUE(Grid(params,request).boundaryDepartureMotionFree(start,{.1,0,0},moving,.2,.35,.5,1));
  EXPECT_FALSE(Grid(params,request).boundaryDepartureMotionFree(start,{.1,0,0},moving,.2,.35,.5,.01));
}

TEST_F(ScanBodyTest, OptionalFeetDoNotRequireFullBodyGroundColumns) {
  request.supportHeightM = .5;
  request.supportHeightToleranceM = .01;
  for (double x : {-.45, .45}) for (double y : {-.25, .25}) {
    c::Part foot;
    foot.link = "optional_foot";
    foot.shape = c::Shape::Sphere;
    foot.center = {x, y, -.45};
    foot.radius_m = .03;
    foot.half_size = {.03, .03, .03};
    foot.support_contact = true;
    body->parts.push_back(foot);
  }
  for (int y = -20; y < 20; ++y) for (int x = -20; x < 20; ++x)
    raw.occupy({(x + .5) * .1, (y + .5) * .1, -.05});
  snapshot();
  auto known = std::make_shared<std::vector<std::uint8_t>>(
      *request.environment.collision.knownFreeStorage);
  const auto& map = request.environment.collision;
  const auto index = [](double value, double minimum, double resolution) {
    return static_cast<int>(std::floor((value - minimum) / resolution));
  };
  const int x = index(.05, map.aabbMin.x, map.resolution);
  const int y = index(.05, map.aabbMin.y, map.resolution);
  const int z = index(.05, map.aabbMin.z, map.resolution);
  const auto linear = (static_cast<std::size_t>(z) * map.sizeY + y) * map.sizeX + x;
  (*known)[linear / 8U] &= static_cast<std::uint8_t>(~(1U << (linear % 8U)));
  request.environment.collision.knownFreeStorage = known;
  const auto evidence = *known;
  Grid grid(params, request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  EXPECT_EQ(grid.inflatedOccupancy({0, 0, .5}, 0), 0);
  EXPECT_EQ(grid.segmentInflatedOccupancy({0, 0, .5}, 0, {.2, 0, .5}, 0), 0);
  EXPECT_EQ(grid.brakingOccupancy({{0, 0, .5}, 0}, {.2, 0, 0}, .1, 1, 1), 0);
  EXPECT_EQ(*known, evidence);

  raw.occupy({.35, .05, .55});
  snapshot();
  request.environment.collision.knownFreeStorage = known;
  EXPECT_EQ(Grid(params, request).inflatedOccupancy({0, 0, .5}, 0), 1);
}

TEST_F(ScanBodyTest, ContactPartsDoNotRequirePerFootGroundEvidence) {
  request.supportHeightM = .5;
  request.supportHeightToleranceM = .01;
  c::Part foot;
  foot.link = "contact_foot";
  foot.shape = c::Shape::Sphere;
  foot.center = {.55, 0, -.47};
  foot.radius_m = .03;
  foot.half_size = {.03, .03, .03};
  foot.support_contact = true;
  body->parts.push_back(foot);
  for (double y : {-.05, .05}) raw.occupy({.55, y, -.05});
  snapshot();
  EXPECT_EQ(Grid(params, request).inflatedOccupancy({0, 0, .5}, 0), 0);
  raw.clear();
  snapshot();
  EXPECT_EQ(Grid(params, request).inflatedOccupancy({0, 0, .5}, 0), 0);
  for (double y : {-.05, .05}) raw.occupy({.55, y, .95});
  snapshot();
  EXPECT_EQ(Grid(params, request).inflatedOccupancy({0, 0, .5}, 0), 0);
}

TEST_F(ScanBodyTest, RejectsContactOnAllSixFacesAndAllowsSeparatedControls) {
  for (const Vec3 contact : std::vector<Vec3>{{.35,.05,.55},{-.35,.05,.55},
          {.05,.15,.55},{.05,-.15,.55},{.05,.05,.65},{.05,.05,.35}}) {
    SCOPED_TRACE(::testing::Message() << contact.x << ',' << contact.y << ',' << contact.z);
    raw.clear();
    raw.occupy(contact);
    snapshot();
    EXPECT_EQ(Grid(params,request).inflatedOccupancy({0,0,.5},0),1);
  }
  raw.clear();
  raw.occupy({.45,.05,.55});
  snapshot();
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({0,0,.5},0),0);
}

TEST_F(ScanBodyTest, RequiresRawEvidenceAndCompleteBodyCoverageWithoutPromotingUnknown) {
  snapshot();
  request.environment.collision.measuredOccupiedStorage.reset();
  Grid missing(params,request);
  EXPECT_FALSE(missing.valid());
  EXPECT_EQ(missing.reason(),"body_raw_evidence_missing");
  snapshot();
  request.environment.collision.knownFreeStorage =
      std::make_shared<const std::vector<std::uint8_t>>(
          request.environment.collision.inflatedBytes,0U);
  const auto unknown = request.environment.collision.knownFreeStorage;
  const auto evidence = *unknown;
  Grid unknownAir(params,request);
  ASSERT_TRUE(unknownAir.valid()) << unknownAir.reason();
  EXPECT_EQ(unknownAir.inflatedOccupancy({0,0,.5},0),0);
  EXPECT_EQ(unknownAir.segmentInflatedOccupancy({0,0,.5},0,{.4,0,.5},0),0);
  EXPECT_EQ(*unknown,evidence);
  raw.occupy({.05,.05,.55});
  snapshot();
  request.environment.collision.knownFreeStorage = unknown;
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({0,0,.5},0),1);
  EXPECT_EQ(*unknown,evidence);
  snapshot();
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({1.9,0,.5},0),-1);
}

TEST_F(ScanBodyTest, DoesNotRequireUnusedFreeBitmapForBodyCollision) {
  snapshot();
  request.environment.collision.knownFreeStorage.reset();
  ASSERT_TRUE(Grid(params,request).valid());
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({0,0,.5},0),0);
  raw.occupy({.05,.05,.55});
  snapshot();
  request.environment.collision.knownFreeStorage.reset();
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({0,0,.5},0),1);
}

TEST_F(ScanBodyTest, OrientationAndIntermediateTurnUseParts) {
  raw.occupy({.25,.25,.55});
  snapshot();
  Grid grid(params,request);
  ASSERT_TRUE(grid.valid());
  EXPECT_EQ(grid.inflatedOccupancy({0,0,.5},0),0);
  EXPECT_EQ(grid.inflatedOccupancy({0,0,.5},std::acos(-1.)/2),0);
  EXPECT_EQ(grid.segmentInflatedOccupancy({0,0,.5},0,{0,0,.5},std::acos(-1.)/2),1);
  request.bodyHeading = std::acos(-1.)/2;
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({0,0,.5},*request.bodyHeading),1);
}

TEST_F(ScanBodyTest, ExplicitGuideHeadingChecksActualTurnInsteadOfCandidateTangent) {
  raw.occupy({.25,.25,.55});
  snapshot();
  request.bodyHeading = std::acos(-1.)/2;
  Grid guided(params,request);
  ASSERT_TRUE(guided.valid());
  // The candidate velocity is eastward, but the declared body must turn north.
  EXPECT_EQ(guided.trajectoryOccupancy({0,0,.5},{.1,0,0},
                                       {.01,0,.5},{.1,0,0},0,.1),1);
  request.bodyHeading.reset();
  EXPECT_EQ(Grid(params,request).trajectoryOccupancy({0,0,.5},{.1,0,0},
                                                     {.01,0,.5},{.1,0,0},0,.1),0);
}

TEST_F(ScanBodyTest, FixedHeadingDoesNotRepeatAlignmentInsideFutureNarrowSegments) {
  for (bool multipart : {true, false}) {
    SCOPED_TRACE(multipart);
    request.body = multipart ? body : nullptr;
    params.scan.cylinderOffset = .25;
    request.robot.pose = {{-1,0,.5},std::acos(-1.)/2};
    request.bodyHeading = 0;
    raw.occupy({.05,.25,.55});
    inflated.occupy({.05,.25,.55});
    snapshot();
    Grid grid(params,request);
    ASSERT_TRUE(grid.valid()) << grid.reason();
    EXPECT_EQ(grid.inflatedOccupancy(request.robot.pose.position,0),0);
    EXPECT_EQ(grid.inflatedOccupancy({0,0,.5},0),0);
    EXPECT_EQ(grid.segmentInflatedOccupancy({0,0,.5},0,{.4,0,.5},0),0);
    EXPECT_EQ(grid.trajectoryOccupancy({0,0,.5},{.2,0,0},
                                       {.4,0,.5},{.2,0,0},1,3),0);

    // Actual alignment at the occupied location remains required at time zero.
    request.robot.pose.position = {0,0,.5};
    Grid atNarrowSegment(params,request);
    EXPECT_EQ(atNarrowSegment.inflatedOccupancy({0,0,.5},0),1);
    EXPECT_EQ(atNarrowSegment.trajectoryOccupancy({0,0,.5},{.2,0,0},
                                                  {.4,0,.5},{.2,0,0},0,2),1);
    // A later visit to the same coordinates must not replay the initial yaw.
    EXPECT_EQ(atNarrowSegment.trajectoryOccupancy({0,0,.5},{.2,0,0},
                                                  {.4,0,.5},{.2,0,0},1,3),0);
  }
}

TEST_F(ScanBodyTest, FixedHeadingPredictionsOnlyCheckAlignmentAtTheMeasuredStart) {
  const nav_kernel::PredictedObstacle obstacle{{0,.25,.5},{0,.25,.5},.01,.4,.6};
  for (bool multipart : {true, false}) {
    SCOPED_TRACE(multipart);
    request.body = multipart ? body : nullptr;
    params.scan.cylinderOffset = .25;
    params.scan.cylinderRadius = .01;
    request.robot.pose = {{-1,0,.5},std::acos(-1.)/2};
    request.bodyHeading = 0;
    snapshot();
    request.clock.timestampS = 1;
    request.environment.predictions = {&obstacle,1,3,1,1};
    Grid grid(params,request);
    ASSERT_TRUE(grid.valid()) << grid.reason();
    EXPECT_EQ(grid.inflatedOccupancy(request.robot.pose.position,0),0);
    EXPECT_EQ(grid.inflatedOccupancy({0,0,.5},0),0);
    EXPECT_EQ(grid.segmentInflatedOccupancy({0,0,.5},0,{.4,0,.5},0),0);
    EXPECT_FALSE(grid.predictionIntersects({0,0,.5},0,{.4,0,.5},0));
    EXPECT_EQ(grid.trajectoryOccupancy({0,0,.5},{.2,0,0},
                                       {.4,0,.5},{.2,0,0},1,3),0);

    request.robot.pose.position = {0,0,.5};
    Grid atNarrowSegment(params,request);
    EXPECT_EQ(atNarrowSegment.inflatedOccupancy({0,0,.5},0),1);
    EXPECT_TRUE(atNarrowSegment.predictionIntersects({0,0,.5},0,{.4,0,.5},0));
    EXPECT_EQ(atNarrowSegment.trajectoryOccupancy({0,0,.5},{.2,0,0},
                                                  {.4,0,.5},{.2,0,0},0,2),1);
    EXPECT_EQ(atNarrowSegment.trajectoryOccupancy({0,0,.5},{.2,0,0},
                                                  {.4,0,.5},{.2,0,0},1,3),0);
  }
}

TEST_F(ScanBodyTest, SlowingFutureSegmentRetainsItsApproachHeading) {
  raw.occupy({.25,.05,.55});
  snapshot();
  request.robot.pose.position = {-1,-1,.5};
  Grid grid(params,request);
  ASSERT_TRUE(grid.valid());
  EXPECT_EQ(grid.trajectoryOccupancy({0,0,.5},{0,.04,0},
                                     {0,.01,.5},{0,.02,0},1,1.5),0);
  // With no reliable approach tangent, quiet sideways travel keeps actual yaw.
  EXPECT_EQ(grid.trajectoryOccupancy({0,0,.5},{0,.02,0},
                                     {0,.01,.5},{},1,1.5),1);
}

TEST_F(ScanBodyTest, UpstreamCandidateDoesNotRequireMeasuredTurnSweep) {
  using namespace nav_kernel::local::scan::upstream;
  raw.occupy({.25,.25,.55});
  snapshot();
  Grid grid(params,request);
  ASSERT_EQ(grid.segmentInflatedOccupancy({0,0,.5},0,
      {0,0,.5},std::acos(-1.)/2),1);
  auto map = std::make_shared<GridMap>(grid);
  EXPECT_EQ(map->getInflateOccupancy({0,0,.5},0),0);
  EXPECT_EQ(map->getInflateOccupancy({0,0,.5},std::acos(-1.)/2),0);
  Eigen::MatrixXd controls(3,6);
  controls.row(0).setZero();
  controls.row(1) << -.2,0,.2,.4,.6,.8;
  controls.row(2).setConstant(.5);
  BsplineOptimizer optimizer;
  optimizer.setEnvironment(map);
  EXPECT_TRUE(optimizer.checkTrajectoryCollisionFree(UniformBspline(controls,3,.5)));

  inflated.occupy({0,params.scan.cylinderOffset,.5});
  snapshot();
  Grid occupied(params,request);
  map->setGrid(&occupied);
  EXPECT_NE(map->getInflateOccupancy({0,0,.5},std::acos(-1.)/2),0);
  EXPECT_FALSE(optimizer.checkTrajectoryCollisionFree(UniformBspline(controls,3,.5)));
}

TEST_F(ScanBodyTest, UpstreamRefinementChecksLeadingTwoThirds) {
  using namespace nav_kernel::local::scan::upstream;
  inflated.occupy({1.05,0,.5});
  snapshot();
  Grid grid(params,request);
  auto map = std::make_shared<GridMap>(grid);
  Eigen::MatrixXd controls(3,9);
  for (int i=0; i<9; ++i) controls.col(i) = Eigen::Vector3d((i-1)*.2,0,.5);
  const UniformBspline curve(controls,3,.5);
  BsplineOptimizer optimizer;
  optimizer.setEnvironment(map);
  ASSERT_NE(map->getInflateOccupancy({1.05,0,.5},0),0);
  EXPECT_TRUE(optimizer.checkTrajectoryCollisionFree(curve));
  inflated.occupy({.35,0,.5});
  snapshot();
  Grid blocked(params,request);
  map->setGrid(&blocked);
  EXPECT_FALSE(optimizer.checkTrajectoryCollisionFree(curve));
}

TEST(ScanHeading, PreviewClampsAtEndpointAndFallsBackLikeController) {
  using namespace nav_kernel::local::scan::upstream;
  Eigen::MatrixXd controls(3,6);
  controls.row(0) << 0,.01,.02,.03,.04,.05;
  controls.row(1).setZero();
  controls.row(2).setConstant(.5);
  const UniformBspline curve(controls,3,.5);
  const auto velocity=curve.getDerivative();
  const double end=curve.getTimeSum();
  EXPECT_GT(curve.headingDirection(end-.01,velocity,.8).x(),.9);
  EXPECT_GT(curve.headingDirection(end,velocity,.8).x(),.9);
  EXPECT_EQ(curve.headingDirection(end,velocity,0).squaredNorm(),0);
  controls.row(0).setZero();
  const UniformBspline stopped(controls,3,.5);
  EXPECT_EQ(stopped.headingDirection(end,stopped.getDerivative(),.8).squaredNorm(),0);
}

TEST_F(ScanBodyTest, TranslationChecksBetweenFreeEndpointsAndBrakingUsesActualYaw) {
  raw.occupy({.05,.05,.55});
  snapshot();
  Grid grid(params,request);
  EXPECT_EQ(grid.inflatedOccupancy({-1,0,.5},0),0);
  EXPECT_EQ(grid.inflatedOccupancy({1,0,.5},0),0);
  EXPECT_EQ(grid.segmentInflatedOccupancy({-1,0,.5},0,{1,0,.5},0),1);
  EXPECT_FALSE(grid.boundaryDepartureFree({{0,0,.5},0},{.4,0,.5}));
  raw.clear();
  raw.occupy({.05,.25,.55});
  snapshot();
  request.bodyHeading = 0;
  Grid withHeading(params,request);
  // A planned heading cannot erase the measured body's current orientation.
  EXPECT_EQ(withHeading.brakingOccupancy({{0,0,.5},std::acos(-1.)/2},{},.1,1,1),1);
}

TEST_F(ScanBodyTest, PlanningToGridTransformRotatesTheBody) {
  raw.occupy({.55,.45,.55});
  snapshot();
  request.environment.collision.gridFromPlanningTranslation = {.5,.25,0};
  request.environment.collision.gridFromPlanningYaw = std::acos(-1.)/2;
  Grid grid(params,request);
  ASSERT_TRUE(grid.valid());
  EXPECT_EQ(grid.inflatedOccupancy({0,0,.5},0),1);
  EXPECT_EQ(grid.inflatedOccupancy({0,0,.5},std::acos(-1.)/2),0);
}

TEST_F(ScanBodyTest, DynamicPredictionUsesBodyOnceWithoutLegacyCylinderInflation) {
  snapshot();
  params.scan.cylinderRadius = 5;
  params.scan.bodyClearanceAbove = 5;
  const nav_kernel::PredictedObstacle prediction{{.8,0,.5},{.8,0,.5},.1,.2,.8};
  request.clock.timestampS = 1;
  request.environment.predictions = {&prediction,1,2,1,1};
  Grid grid(params,request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  EXPECT_EQ(grid.predictionCount(),1U);
  EXPECT_EQ(grid.inflatedOccupancy({0,0,.5},0),0);
  EXPECT_EQ(grid.inflatedOccupancy({.5,0,.5},0),1);
  EXPECT_EQ(grid.inflatedOccupancy({.5,0,1.2},0),0);
}

TEST_F(ScanBodyTest, DynamicPredictionUsesHorizontalMarginWithoutVerticalPadding) {
  snapshot();
  const nav_kernel::PredictedObstacle obstacle{{.46,0,.5},{.46,0,.5},.1,.4,.6};
  request.clock.timestampS = 1;
  request.environment.predictions = {&obstacle,1,2,1,1};
  body->horizontal_margin_m = .05;
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({0,0,.5},0),0);
  body->horizontal_margin_m = .08;
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({0,0,.5},0),1);
  EXPECT_EQ(Grid(params,request).inflatedOccupancy({.46,0,.8},0),0);
}

TEST_F(ScanBodyTest, NearbyPredictionDoesNotPadTheDepartureOutsideItsTimeInterval) {
  snapshot();
  const nav_kernel::PredictedObstacle obstacle{{-.351,0,.5},{-.351,0,.5},.05,.4,.6};
  request.clock.timestampS=1;
  request.environment.predictions={&obstacle,1,3,1,1};
  Grid grid(params,request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  EXPECT_EQ(grid.inflatedOccupancy({0,0,.5},0),0);
  EXPECT_EQ(grid.trajectoryOccupancy({0,0,.5},{.2,0,0},{.2,0,.5},{.2,0,0},0,1),0);
  EXPECT_EQ(grid.brakingOccupancy({{0,0,.5},0},{.2,0,0},.1,1,1),0);
  EXPECT_EQ(grid.trajectoryOccupancy({0,0,.5},{-.2,0,0},{-.2,0,.5},{-.2,0,0},0,1),1);
}

TEST_F(ScanBodyTest, DynamicCrossingUsesMatchedArrivalTimeAndRetainsHorizonEndpoint) {
  snapshot();
  const nav_kernel::PredictedObstacle crossing{{0,-.8,.5},{0,.8,.5},.04,.4,.6};
  request.clock.timestampS = 1;
  request.environment.predictions = {&crossing,1,4,1,1};
  Grid grid(params,request);
  ASSERT_TRUE(grid.valid());
  EXPECT_EQ(grid.trajectoryOccupancy({-.8,0,.5},{1.6,0,0},
                                     {.8,0,.5},{1.6,0,0},0,1),1);
  EXPECT_EQ(grid.trajectoryOccupancy({-.8,0,.5},{1.6,0,0},
                                     {.8,0,.5},{1.6,0,0},1,2),0);
  // Waiting at the crossing is unsafe until the moving obstacle has passed.
  EXPECT_EQ(grid.trajectoryOccupancy({0,0,.5},{},{0,0,.5},{},0,1),1);
  EXPECT_EQ(grid.trajectoryOccupancy({0,0,.5},{},{0,0,.5},{},1,2),0);
  EXPECT_EQ(grid.trajectoryOccupancy({0,.8,.5},{},{0,.8,.5},{},1,2),1);
}

TEST_F(ScanBodyTest, DynamicObstacleChecksIntermediateRotationAndBrakingMotion) {
  snapshot();
  const nav_kernel::PredictedObstacle corner{{.2,.2,.5},{.2,.2,.5},.01,.4,.6};
  request.clock.timestampS = 1;
  request.environment.predictions = {&corner,1,3,1,1};
  Grid grid(params,request);
  const double halfTurn = std::acos(-1.)/2;
  ASSERT_TRUE(grid.valid());
  EXPECT_EQ(grid.inflatedOccupancy({0,0,.5},0),0);
  EXPECT_EQ(grid.inflatedOccupancy({0,0,.5},halfTurn),0);
  EXPECT_EQ(grid.segmentInflatedOccupancy({0,0,.5},0,{0,0,.5},halfTurn),1);
  EXPECT_EQ(grid.brakingOccupancy({{0,0,.5},0},{0,0,1.5},.1,1,1),1);
}

TEST_F(ScanBodyTest, StationaryBrakingIncludesFreshCrossingAndAgedPredictionDoesNotReplayPast) {
  snapshot();
  const nav_kernel::PredictedObstacle crossing{{0,-.4,.5},{0,.4,.5},.04,.4,.6};
  request.clock.timestampS = 1;
  request.environment.predictions = {&crossing,1,4,1,.2};
  Grid grid(params,request);
  EXPECT_EQ(grid.inflatedOccupancy({0,0,.5},0),0);
  EXPECT_EQ(grid.brakingOccupancy({{0,0,.5},0},{},.2,1,1),1);
  EXPECT_FALSE(grid.boundaryDepartureFree({{0,0,.5},0},{.3,0,.5}));
  request.clock.timestampS = 1.3;
  Grid aged(params,request);
  EXPECT_EQ(aged.inflatedOccupancy({0,0,.5},0),0);
  EXPECT_EQ(aged.brakingOccupancy({{0,0,.5},0},{},.2,1,1),0);
  EXPECT_TRUE(aged.boundaryDepartureFree({{0,0,.5},0},{.3,0,.5}));
}

TEST_F(ScanBodyTest, PlanarSweepPreservesSmallObservedFloorClearance) {
  body->margin_m = .02;
  for (double x = -1.45; x < 1.5; x += .1)
    for (double y = -1.45; y < 1.5; y += .1) raw.occupy({x,y,-.05});
  snapshot();
  Grid grid(params,request);
  ASSERT_TRUE(grid.valid());
  // Expanded body bottom is 9 mm above the measured floor, matching the
  // nominal Go2 model's tight floor clearance in the native global demo.
  const nav_kernel::Pose pose{{0,0,.129},0};
  EXPECT_EQ(grid.inflatedOccupancy(pose.position,pose.yaw),0);
  EXPECT_EQ(grid.segmentInflatedOccupancy(pose.position,0,{.4,0,.129},0),0);
  EXPECT_EQ(grid.brakingOccupancy(pose,{.6,0,0},.1,1,1),0);
}

TEST_F(ScanBodyTest, BrakingRejectsAnObstacleBetweenFreeStartAndStoppedPoses) {
  raw.occupy({-.25,.05,.55});
  snapshot();
  Grid grid(params,request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  // 1.4 m/s, 100 ms reaction and 1 m/s^2 braking travel 1.12 m.
  const nav_kernel::Pose start{{-.8,0,.5},0};
  EXPECT_EQ(grid.inflatedOccupancy(start.position,start.yaw),0);
  EXPECT_EQ(grid.inflatedOccupancy({.32,0,.5},0),0);
  EXPECT_EQ(grid.brakingOccupancy(start,{1.4,0,0},.1,1,1),1);
}

TEST_F(ScanBodyTest, BrakingRetainsUnknownEvidenceWithoutTreatingItAsMeasuredObstacle) {
  snapshot();
  CollisionBitmap unknown({-2,-2,-1},{2,2,2},.1);
  unknown.occupy({-.25,.05,.55});
  const auto unknownView = unknown.view();
  auto known = std::make_shared<std::vector<std::uint8_t>>(
      unknownView.inflatedBytes,255U);
  for (std::size_t i=0; i<known->size(); ++i)
    (*known)[i] &= static_cast<std::uint8_t>(~unknownView.inflatedBits[i]);
  request.environment.collision.knownFreeStorage = known;
  const auto evidence = *known;
  Grid grid(params,request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  EXPECT_EQ(grid.inflatedOccupancy({-.8,0,.5},0),0);
  EXPECT_EQ(grid.inflatedOccupancy({.32,0,.5},0),0);
  EXPECT_EQ(grid.brakingOccupancy({{-.8,0,.5},0},{1.4,0,0},.1,1,1),0);
  EXPECT_EQ(*known,evidence);
}

TEST_F(ScanBodyTest, BrakingRotationChecksCornerBetweenFreeOrientations) {
  raw.occupy({.25,.25,.55});
  snapshot();
  Grid grid(params,request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  const double pi = std::acos(-1.);
  // This angular speed stops at pi/2 after 100 ms reaction and 1 rad/s^2 braking.
  const double yawSpeed = std::sqrt(.01 + pi) - .1;
  EXPECT_EQ(grid.inflatedOccupancy({0,0,.5},0),0);
  EXPECT_EQ(grid.inflatedOccupancy({0,0,.5},pi/2),0);
  EXPECT_EQ(grid.brakingOccupancy({{0,0,.5},0},{0,0,yawSpeed},.1,1,1),1);
  // Two full revolutions finish at the starting orientation but must still
  // check the intermediate corners, rather than taking the shortest yaw arc.
  const double twoRevolutionSpeed = std::sqrt(.01 + 8*pi) - .1;
  EXPECT_EQ(grid.brakingOccupancy({{0,0,.5},0},{0,0,twoRevolutionSpeed},.1,1,1),1);
}

TEST_F(ScanBodyTest, BrakingCurveChecksBodyCornerAwayFromEndpointChord) {
  raw.occupy({.55,.75,.55});
  snapshot();
  Grid grid(params,request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  const double pi = std::acos(-1.);
  // Radius .5 m and effective duration pi/2 make a half circle. At its
  // midpoint the north-facing front corner reaches this obstacle.
  EXPECT_EQ(grid.inflatedOccupancy({0,0,.5},0),0);
  EXPECT_EQ(grid.inflatedOccupancy({0,1,.5},pi),0);
  EXPECT_EQ(grid.brakingOccupancy({{0,0,.5},0},{1,0,2},0,1,2/pi),1);
}

TEST_F(ScanBodyTest, ContactBodyBrakingDoesNotAddPerFootGroundAdmission) {
  c::Part foot;
  foot.link = "contact_foot";
  foot.shape = c::Shape::Sphere;
  foot.center = {0,0,-.4};
  foot.half_size = {.1,.1,.1};
  foot.radius_m = .1;
  foot.support_contact = true;
  body->parts.push_back(foot);
  for (int x=-15; x<15; ++x)
    for (int y=-5; y<5; ++y)
      raw.occupy({(x+.5)*.1,(y+.5)*.1,-.05});
  snapshot();
  const nav_kernel::Pose start{{-.8,0,.5},0};
  Grid supported(params,request);
  ASSERT_TRUE(supported.valid()) << supported.reason();
  ASSERT_EQ(supported.brakingOccupancy(start,{1.4,0,0},.1,1,1),0);
  raw.clear();
  for (int x=-15; x<15; ++x)
    for (int y=-5; y<5; ++y)
      if (x < -4 || x >= -2)
        raw.occupy({(x+.5)*.1,(y+.5)*.1,-.05});
  snapshot();
  Grid hole(params,request);
  ASSERT_TRUE(hole.valid()) << hole.reason();
  EXPECT_EQ(hole.inflatedOccupancy(start.position,start.yaw),0);
  EXPECT_EQ(hole.inflatedOccupancy({.32,0,.5},0),0);
  EXPECT_EQ(hole.brakingOccupancy(start,{1.4,0,0},.1,1,1),0);
}

}  // namespace
