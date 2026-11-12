#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include <memory>
#include <limits>
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <utility>
#include <vector>

#include "collision_bitmap.hpp"
#include "collision/body.hpp"
#include "planning/local/scan/backend.hpp"
#include "planning/local/scan/grid.hpp"
#include "planning/local/scan/reference.hpp"
#include "planning/local/scan/task.hpp"
#include "planning/local/scan/upstream/path_searching/dyn_a_star.h"
#include "planning/local/scan/upstream/plan_env/grid_map.h"
#include "planning/local/scan/upstream/plan_manage/closed_loop_controller.h"
#include "planning/local/scan/upstream/plan_manage/planner_fsm.h"
#include "trajectory/spline.hpp"
#include "tracking/follower.hpp"

namespace {

using lingtu::nav::tests::CollisionBitmap;
using nav_kernel::LocalPlan;
using nav_kernel::LocalPlannerBackend;
using nav_kernel::LocalPlannerParams;
using nav_kernel::LocalPlanRequest;
using nav_kernel::RouteTarget;
using nav_kernel::SplineTarget;
using nav_kernel::Vec3;

TEST(ScanGridAdapter, InflationUsesWholeCells) {
  CollisionBitmap bitmap;
  bitmap.occupyInflated({.5F, 0, .5F}, .4, .2, .2);
  LocalPlannerParams params;
  params.scan.voxelResolution = .1;
  params.scan.cylinderOffset = 0;
  LocalPlanRequest request;
  request.environment.collision = bitmap.view();
  const nav_kernel::local::scan::Grid grid(params, request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  EXPECT_EQ(grid.occupiedCellCount(), 245);
  for (int z = 2; z <= 8; ++z) {
    for (int y = -5; y <= 5; ++y) {
      for (int x = 0; x <= 10; ++x) {
        const Vec3 point{(x + .5) * .1, (y + .5) * .1, (z + .5) * .1};
        const bool occupied = z >= 3 && z <= 7 &&
                              (x - 5) * (x - 5) + y * y <= 16;
        EXPECT_EQ(grid.inflatedOccupancy(point, 0), occupied ? 1 : 0)
            << "cell " << x << ',' << y << ',' << z;
      }
    }
  }
}

TEST(ScanNoGo, PolicySurvivesDisabledObstacleChecksAndIsThreeDimensional) {
  LocalPlannerParams params;
  params.checkObstacle = false;
  params.scan.voxelResolution = .1;
  params.scan.cylinderRadius = .1;
  params.scan.cylinderOffset = .15;
  params.scan.bodyClearanceBelow = .1;
  params.scan.bodyClearanceAbove = .1;
  LocalPlanRequest request;
  request.environment.no_go_regions = std::make_shared<const std::vector<lingtu::maps::NoGoRegion>>(
      std::vector<lingtu::maps::NoGoRegion>{{"room", {.45,-.2,.2}, {.55,.2,.8}}});
  const nav_kernel::local::scan::Grid grid(params, request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  EXPECT_EQ(grid.cylinderOccupancy({.5,0,.5}, 0), 1);
  EXPECT_EQ(grid.cylinderOccupancy({.5,0,2}, 0), 0);
  EXPECT_EQ(grid.segmentInflatedOccupancy({0,0,.5},0,{1,0,.5},0), 1);
  EXPECT_EQ(grid.trajectoryOccupancy({0,0,.5},{1,0,0},{1,0,.5},{1,0,0},0,1), 1);
  EXPECT_FALSE(grid.boundaryDepartureFree({{.5,0,.5},0},{.8,0,.5}));
  request.environment.no_go_regions.reset();
  const nav_kernel::local::scan::Grid removed(params, request);
  EXPECT_EQ(removed.segmentInflatedOccupancy({0,0,.5},0,{1,0,.5},0), 0);
}

TEST(ScanNoGo, MapPolicyUsesPlanningTransformAndBrakingSweep) {
  LocalPlannerParams params;
  params.checkObstacle = false;
  params.scan.voxelResolution = .1;
  params.scan.cylinderRadius = .05;
  params.scan.cylinderOffset = 0;
  params.scan.bodyClearanceBelow = .1;
  params.scan.bodyClearanceAbove = .1;
  LocalPlanRequest request;
  request.environment.no_go_regions = std::make_shared<const std::vector<lingtu::maps::NoGoRegion>>(
      std::vector<lingtu::maps::NoGoRegion>{{"gate", {9.9,20.3,.3}, {10.1,20.35,.7}}});
  request.environment.collision.gridFromPlanningTranslation = {10,20,0};
  request.environment.collision.gridFromPlanningYaw = std::acos(-1.)*.5;
  const nav_kernel::local::scan::Grid grid(params, request);
  ASSERT_TRUE(grid.valid());
  EXPECT_EQ(grid.cylinderOccupancy({.32,0,.5},0), 1);
  EXPECT_EQ(grid.cylinderOccupancy({0,.32,.5},0), 0);
  EXPECT_EQ(grid.brakingOccupancy({{0,0,.5},0},{1,0,0},.1,1,1), 1);
  EXPECT_LT(grid.brakingScale({{0,0,.5},0},{1,0,0},.1,1,1), 1);
}

TEST(ScanNoGo, BodyCannotUseGroundContactExceptionForPolicy) {
  LocalPlannerParams params;
  params.scan.voxelResolution = .1;
  auto body = std::make_shared<lingtu::nav::collision::Body>();
  body->margin_m = 0;
  body->parts.push_back({"contact", {0,0,-.1}, {}, {.1,.1,.1}});
  body->parts.back().rotation = {1,0,0,0,1,0,0,0,1};
  body->parts.back().support_contact = true;
  CollisionBitmap bitmap({-2,-2,-1},{2,2,2},.1);
  LocalPlanRequest request;
  request.body = body;
  request.environment.collision = bitmap.view();
  request.environment.collision.measuredOccupiedStorage =
      std::make_shared<const std::vector<std::uint8_t>>(
          request.environment.collision.inflatedBytes, 0U);
  request.environment.no_go_regions = std::make_shared<const std::vector<lingtu::maps::NoGoRegion>>(
      std::vector<lingtu::maps::NoGoRegion>{{"floor", {-.05,-.05,.15}, {.05,.05,.3}}});
  const nav_kernel::local::scan::Grid grid(params, request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  EXPECT_EQ(grid.inflatedOccupancy({0,0,.4},0),1);
  EXPECT_FALSE(grid.boundaryDepartureFree({{0,0,.4},0},{.3,0,.4}));
}

TEST(ScanGridAdapter, InflationClipsOutsideSources) {
  CollisionBitmap bitmap({0, 0, 0}, {1, 1, 1}, .1);
  bitmap.occupyInflated({-.15F, .55F, .55F}, .3, 0, 0);
  const auto view = bitmap.view();
  EXPECT_TRUE(view.occupied({.05, .55, .55}));
  EXPECT_TRUE(view.occupied({.15, .55, .55}));
  EXPECT_FALSE(view.occupied({.25, .55, .55}));
  EXPECT_TRUE(view.occupied({.05, .35, .55}));
  EXPECT_TRUE(view.occupied({.05, .75, .55}));
  EXPECT_FALSE(view.occupied({.05, .85, .55}));
}

TEST(ScanGridAdapter, PointQueriesUseSnapshotOriginOnEveryAxis) {
  LocalPlannerParams params;
  params.scan.voxelResolution = .1;
  params.scan.cylinderOffset = 0;
  std::vector<std::uint8_t> bits(64, 0);
  bits[0] = 1;
  LocalPlanRequest request;
  auto &view = request.environment.collision;
  view.inflatedBits = bits.data();
  view.inflatedBytes = bits.size();
  view.sizeX = view.sizeY = view.sizeZ = 8;
  view.resolution = .1;
  for (double origin : {.03, .07, -.03, -.07}) {
    SCOPED_TRACE(origin);
    view.aabbMin = {origin, origin - .2, origin + .3};
    view.aabbMax = {origin + .8, origin + .6, origin + 1.1};
    const Vec3 occupied{origin + .075, origin - .125, origin + .375};
    const Vec3 clear{occupied.x + .1, occupied.y + .1, occupied.z + .1};
    ASSERT_TRUE(view.valid());
    ASSERT_TRUE(view.occupied(occupied));
    ASSERT_FALSE(view.occupied(clear));
    const nav_kernel::local::scan::Grid grid(params, request);
    ASSERT_TRUE(grid.valid()) << grid.reason();
    EXPECT_EQ(grid.inflatedOccupancy(occupied, 0), 1);
    EXPECT_EQ(grid.inflatedOccupancy(clear, 0), 0);
  }
}

TEST(ScanGridAdapter, SegmentsUseSnapshotOriginInBothDirectionsAndPlanningFrames) {
  LocalPlannerParams params;
  params.scan.voxelResolution = .1;
  params.scan.cylinderOffset = 0;
  std::vector<std::uint8_t> bits(9, 0);
  constexpr std::size_t blocked = (1 * 3 + 1) * 8 + 3;
  bits[blocked / 8] = static_cast<std::uint8_t>(1U << (blocked % 8));
  LocalPlanRequest request;
  auto &view = request.environment.collision;
  view.inflatedBits = bits.data();
  view.inflatedBytes = bits.size();
  view.sizeX = 8;
  view.sizeY = view.sizeZ = 3;
  view.resolution = .1;
  view.aabbMin = {.03, -.17, .23};
  view.aabbMax = {.83, .13, .53};
  const Vec3 start{.08, .005, .405}, end{.68, .005, .405};
  const auto toPlanning = [&](const Vec3 &point) {
    const double x = point.x - view.gridFromPlanningTranslation.x;
    const double y = point.y - view.gridFromPlanningTranslation.y;
    const double cosine = std::cos(view.gridFromPlanningYaw);
    const double sine = std::sin(view.gridFromPlanningYaw);
    return Vec3{cosine*x + sine*y, -sine*x + cosine*y,
                point.z - view.gridFromPlanningTranslation.z};
  };
  for (bool transformed : {false, true}) {
    SCOPED_TRACE(transformed);
    if (transformed) {
      view.gridFromPlanningTranslation = {.4, -.6, .2};
      view.gridFromPlanningYaw = std::acos(-1.) / 2;
    }
    const auto from = toPlanning(start), to = toPlanning(end);
    const nav_kernel::local::scan::Grid grid(params, request);
    ASSERT_TRUE(grid.valid()) << grid.reason();
    ASSERT_EQ(grid.inflatedOccupancy(from, 0), 0);
    ASSERT_EQ(grid.inflatedOccupancy(to, 0), 0);
    EXPECT_EQ(grid.segmentInflatedOccupancy(from, 0, to, 0), 1);
    EXPECT_EQ(grid.segmentInflatedOccupancy(to, 0, from, 0), 1);
    const auto clearFrom = toPlanning({start.x, start.y + .1, start.z});
    const auto clearTo = toPlanning({end.x, end.y + .1, end.z});
    EXPECT_EQ(grid.segmentInflatedOccupancy(clearFrom, 0, clearTo, 0), 0);
  }
}

TEST(ScanGridAdapter, TransportCountersDoNotOverrideValidCollisionGeometry) {
  LocalPlannerParams params;
  params.scan.voxelResolution = .1;
  params.scan.cylinderOffset = 0;
  CollisionBitmap bitmap({-2,-2,-1}, {2,2,2}, .1);
  bitmap.occupy({.05,.05,.45});
  LocalPlanRequest request;
  request.environment.collision = bitmap.view();
  request.environment.collision.complete = false;
  request.environment.collision.resetEpoch = 0;
  request.environment.collision.observationSequence = 0;
  request.environment.collision.generation = 0;
  const nav_kernel::local::scan::Grid grid(params, request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  EXPECT_EQ(grid.inflatedOccupancy({.05,.05,.45},0),1);
  EXPECT_EQ(grid.inflatedOccupancy({1,1,.45},0),0);

  request.environment.collision.inflatedBytes = 0;
  const nav_kernel::local::scan::Grid malformed(params, request);
  EXPECT_FALSE(malformed.valid());
}

TEST(ScanGridAdapter, InvalidOptionalPredictionKeepsMeasuredObstacles) {
  LocalPlannerParams params;
  params.scan.voxelResolution = .1;
  params.scan.cylinderOffset = 0;
  CollisionBitmap bitmap({-2,-2,-1}, {2,2,2}, .1);
  bitmap.occupy({.05,.05,.45});
  const nav_kernel::PredictedObstacle predictions[] = {
      {{1,1,.4},{1,1,.4},-.1,.1,.8},
      {{1,0,.4},{1,0,.4},.1,.1,.8},
  };
  LocalPlanRequest request;
  request.clock.timestampS = 1;
  request.environment.collision = bitmap.view(1,1);
  request.environment.predictions = {predictions,2,2,1};
  const nav_kernel::local::scan::Grid grid(params, request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  EXPECT_EQ(grid.predictionCount(),1U);
  EXPECT_EQ(grid.inflatedOccupancy({.05,.05,.45},0),1);
  EXPECT_EQ(grid.inflatedOccupancy({1,0,.45},0),1);

  request.environment.predictions.horizonS = 0;
  const nav_kernel::local::scan::Grid invalid_time(params, request);
  ASSERT_TRUE(invalid_time.valid()) << invalid_time.reason();
  EXPECT_EQ(invalid_time.predictionCount(),0U);
  EXPECT_EQ(invalid_time.inflatedOccupancy({.05,.05,.45},0),1);
}

TEST(ScanGridAdapter, SparseGroundEvidenceDoesNotGateCollisionQueries) {
  LocalPlannerParams params;
  params.vehicleLength = .76;
  params.vehicleWidth = .31;
  params.scan.voxelResolution = .05;
  params.scan.cylinderRadius = .25;
  params.scan.cylinderOffset = .19;
  CollisionBitmap inflated({-2,-2,-1}, {2,2,2}, .05);
  CollisionBitmap ground({-2,-2,-1}, {2,2,2}, .05);
  // A return under the centre, with no returns under the former corner probes.
  ground.occupy({.025,.025,-.325});
  LocalPlanRequest request;
  request.robot.pose.position = {.025,.025,.025};
  request.environment.collision = inflated.view();
  const auto evidence = ground.view();
  for (bool sparseEvidence : {false, true}) {
    SCOPED_TRACE(sparseEvidence);
    if (sparseEvidence) {
      request.environment.collision.measuredOccupiedStorage =
          std::make_shared<const std::vector<std::uint8_t>>(
              evidence.inflatedBits, evidence.inflatedBits + evidence.inflatedBytes);
    }
    ASSERT_FALSE(request.environment.collision.knownFreeStorage);
    const nav_kernel::local::scan::Grid grid(params, request);
    ASSERT_TRUE(grid.valid()) << grid.reason();
    EXPECT_EQ(grid.inflatedOccupancy(request.robot.pose.position, 0), 0);
    EXPECT_EQ(grid.segmentInflatedOccupancy(request.robot.pose.position, 0, {.5,.025,.025}, 0), 0);
    EXPECT_EQ(grid.brakingOccupancy(request.robot.pose, {.3,0,0}, .1, 1, 1), 0);
    EXPECT_TRUE(grid.boundaryDepartureFree(request.robot.pose, {.3,.025,.025}));
  }
  // Support removal must not disable the live double-cylinder collision query.
  inflated.occupy({.215,.025,.025});
  request.environment.collision = inflated.view();
  const nav_kernel::local::scan::Grid blocked(params, request);
  ASSERT_TRUE(blocked.valid());
  EXPECT_EQ(blocked.inflatedOccupancy(request.robot.pose.position, 0), 1);
  EXPECT_EQ(blocked.inflatedOccupancy(request.robot.pose.position, std::acos(-1.0)/2), 0);
  EXPECT_EQ(blocked.segmentInflatedOccupancy(request.robot.pose.position, 0, {.5,.025,.025}, 0), 1);
  EXPECT_EQ(blocked.brakingOccupancy(request.robot.pose, {.3,0,0}, .1, 1, 1), 1);
}

TEST(ScanPredictions, ContinuousCrossingBlocksEmptyMeasuredGridAndBraking) {
  auto params = LocalPlannerParams{};
  params.backend = LocalPlannerBackend::Scan;
  params.scan.voxelResolution = .1;
  params.scan.cylinderOffset = 0.0;
  params.vehicleWidth = .2;
  params.scan.cylinderRadius = .1;
  params.footprintPadding = 0.0;
  CollisionBitmap bitmap({-3,-3,-1}, {3,3,2}, .1);
  const nav_kernel::PredictedObstacle crossing{{.4,-1,.4},{.4,1,.4},.1,.1,.8};
  LocalPlanRequest request;
  request.clock.timestampS = 1;
  request.environment.collision = bitmap.view(1, 1);
  request.environment.predictions = {&crossing, 1, 1.35, 1.0};
  nav_kernel::local::scan::Grid grid(params, request);
  ASSERT_TRUE(grid.valid());
  EXPECT_EQ(grid.inflatedOccupancy({.4,0,.4},0), 0);
  EXPECT_EQ(grid.segmentInflatedOccupancy({0,0,.4},0,{1,0,.4},0), 0);
  EXPECT_EQ(grid.brakingOccupancy({{0,0,.4},0},{.5,0,0},.35,.5,1), 1);
  EXPECT_LT(grid.brakingScale({{0,0,.4},0},{.5,0,0},.35,.5,1), 1);
  EXPECT_EQ(grid.segmentInflatedOccupancy({0,-2,.4},0,{1,-2,.4},0), 0);
  EXPECT_EQ(bitmap.view(1,1).occupiedCount(), 0U);
  request.clock.timestampS = 1.36;
  nav_kernel::local::scan::Grid expired(params, request);
  EXPECT_EQ(expired.predictionCount(), 0U);
  EXPECT_EQ(expired.brakingScale({{0,0,.4},0},{.5,0,0},.35,.5,1), 1);
  bitmap.occupy({.4,0,.4});
  request.environment.collision = bitmap.view(1.36,2);
  nav_kernel::local::scan::Grid measured(params, request);
  EXPECT_EQ(measured.inflatedOccupancy({.4,0,.4},0),1);
}

TEST(ScanPredictions, DoesNotTreatOverheadPredictionAsGroundObstacle) {
  LocalPlannerParams params;
  params.scan.voxelResolution = .1;
  params.scan.cylinderOffset = 0;
  CollisionBitmap bitmap({-3,-3,-1}, {3,3,3}, .1);
  const nav_kernel::PredictedObstacle overhead{{.4,-1,2},{.4,1,2},.2,1.8,2.2};
  LocalPlanRequest request;
  request.clock.timestampS = 1;
  request.environment.collision = bitmap.view(1,1);
  request.environment.predictions = {&overhead,1,2};
  const nav_kernel::local::scan::Grid grid(params,request);
  EXPECT_EQ(grid.inflatedOccupancy({.4,0,.4},0),0);
}

TEST(ScanPredictions, UpstreamMapUsesMeasuredOccupancyWhileFinalControlRetainsPredictions) {
  LocalPlannerParams params;
  params.scan.voxelResolution = .1;
  params.scan.cylinderOffset = 0;
  params.scan.cylinderRadius = .1;
  CollisionBitmap bitmap({-3,-3,-1}, {3,3,2}, .1);
  const nav_kernel::PredictedObstacle crossing{{0,-1,.4},{0,1,.4},.1,.1,.8};
  LocalPlanRequest request;
  request.robot.pose = {{0,0,.4},0};
  request.clock.timestampS = 1;
  request.environment.collision = bitmap.view(1,1);
  request.environment.predictions = {&crossing,1,2,1};
  const nav_kernel::local::scan::Grid grid(params,request);
  const nav_kernel::local::scan::upstream::GridMap adapter(grid);
  // Use the same snapshot queries as SCAN's target selection and A* search.
  EXPECT_EQ(adapter.getInflateOccupancy({0,0,.4},0),0);
  EXPECT_EQ(adapter.getInflateOccupancySegment({-.5,0,.4},0,{.5,0,.4},0),0);
  EXPECT_EQ(adapter.getInflateOccupancy({0,-1,.4},0),0);
  EXPECT_EQ(adapter.getInflateOccupancySegment({-.5,-1,.4},0,{.5,-1,.4},0),0);
  EXPECT_EQ(grid.trajectoryOccupancy({0,0,.4},{.8,0,0},{.8,0,.4},{.8,0,0},0,1),0);
  EXPECT_EQ(grid.trajectoryOccupancy({0,0,.4},{},{0,0,.4},{},.4,.6),1);
  EXPECT_EQ(grid.brakingOccupancy(request.robot.pose,{},.6,.5,1),1);
  request.clock.timestampS = 1.5;
  const nav_kernel::local::scan::Grid aged(params,request);
  EXPECT_EQ(aged.inflatedOccupancy({0,0,.4},0),1);
  EXPECT_EQ(aged.inflatedOccupancy({0,-1,.4},0),0);
  bitmap.occupy({0,-1,.4});
  request.environment.collision = bitmap.view(1.5,2);
  const nav_kernel::local::scan::Grid measured(params,request);
  EXPECT_EQ(measured.inflatedOccupancy({0,-1,.4},0),1);
}

TEST(ScanPredictions, FutureSweepAtStartAllowsTimedDepartureButRejectsSimultaneousContact) {
  LocalPlannerParams params;
  params.scan.voxelResolution = .1;
  params.scan.cylinderOffset = 0;
  params.scan.cylinderRadius = .1;
  CollisionBitmap bitmap({-3,-3,-1}, {3,3,2}, .1);
  const nav_kernel::PredictedObstacle crossing{{0,-1,.4},{0,1,.4},.1,.1,.8};
  LocalPlanRequest request;
  request.robot.pose = {{0,0,.4},0};
  request.clock.timestampS = 1;
  request.environment.collision = bitmap.view(1,1);
  request.environment.predictions = {&crossing,1,2,1};
  const nav_kernel::local::scan::Grid grid(params,request);
  EXPECT_EQ(grid.inflatedOccupancy({0,0,.4},0),0);
  EXPECT_EQ(grid.trajectoryOccupancy({0,0,.4},{.8,0,0},{.8,0,.4},{.8,0,0},0,1),0);
  EXPECT_EQ(grid.trajectoryOccupancy({0,0,.4},{},{0,0,.4},{},.4,.6),1);
  EXPECT_EQ(grid.trajectoryOccupancy({0,-2,.4},{0,2,0},{0,0,.4},{0,2,0},0,1),0);
  // Past the prediction horizon, retain its terminal obstacle rather than erase it.
  EXPECT_EQ(grid.trajectoryOccupancy({0,1,.4},{},{0,1,.4},{},1.2,1.3),1);
  bitmap.occupy({0,0,.4});
  request.environment.collision = bitmap.view(1,2);
  const nav_kernel::local::scan::Grid measured(params,request);
  EXPECT_EQ(measured.trajectoryOccupancy({0,0,.4},{.8,0,0},{.8,0,.4},{.8,0,0},0,1),1);
}

TEST(ScanPredictions, TimedChecksAccountForObservationAgeAndBrakingTime) {
  LocalPlannerParams params;
  params.scan.voxelResolution = .1;
  params.scan.cylinderOffset = 0;
  params.scan.cylinderRadius = .1;
  CollisionBitmap bitmap({-3,-3,-1}, {3,3,2}, .1);
  const nav_kernel::PredictedObstacle crossing{{0,-1,.4},{0,1,.4},.1,.1,.8};
  LocalPlanRequest request;
  request.robot.pose = {{0,0,.4},0};
  request.clock.timestampS = 1;
  request.environment.collision = bitmap.view(1,1);
  request.environment.predictions = {&crossing,1,2,1};
  const nav_kernel::local::scan::Grid fresh(params,request);
  EXPECT_EQ(fresh.brakingOccupancy(request.robot.pose,{},.1,.5,1),0);
  EXPECT_EQ(fresh.brakingOccupancy(request.robot.pose,{},.6,.5,1),1);
  request.clock.timestampS = 1.5;
  const nav_kernel::local::scan::Grid aged(params,request);
  EXPECT_EQ(aged.trajectoryOccupancy({0,0,.4},{},{0,0,.4},{},0,.01),1);
  EXPECT_EQ(aged.brakingOccupancy(request.robot.pose,{},.1,.5,1),1);
}

TEST(ScanPredictions, QuietTrajectoryUsesMeasuredHeadingWithoutClearingRealObstacles) {
  LocalPlannerParams params;
  params.scan.voxelResolution = .05;
  params.scan.cylinderOffset = .25;
  CollisionBitmap bitmap({-3,-3,-1}, {3,3,2}, .05);
  bitmap.occupy({.25,0,.4});
  LocalPlanRequest request;
  request.robot.pose = {{0,0,.4},std::acos(-1.0)*.5};
  request.clock.timestampS = 1;
  request.environment.collision = bitmap.view(1,1);
  const nav_kernel::local::scan::Grid grid(params,request);
  const Vec3 noise{.00025,0,0};
  EXPECT_EQ(grid.segmentInflatedOccupancy({0,0,.4},0,{0,0,.4},0),1);
  EXPECT_EQ(grid.trajectoryOccupancy({0,0,.4},noise,{0,0,.4},noise,0,.01),0);
  bitmap.occupy({0,.25,.4});
  request.environment.collision = bitmap.view(1,2);
  const nav_kernel::local::scan::Grid blocked(params,request);
  EXPECT_EQ(blocked.trajectoryOccupancy({0,0,.4},noise,{0,0,.4},noise,0,.01),1);
}

TEST(ScanPredictions, BoundaryDepartureCannotIgnorePredictionAtStart) {
  LocalPlannerParams params;
  params.scan.voxelResolution = .1;
  params.scan.cylinderOffset = 0;
  params.vehicleWidth = .1;
  params.scan.cylinderRadius = .05;
  params.footprintPadding = 0;
  CollisionBitmap bitmap({-3,-3,-1}, {3,3,2}, .1);
  const nav_kernel::PredictedObstacle crossing{{0,-1,.4},{0,1,.4},.05,.1,.8};
  LocalPlanRequest request;
  request.clock.timestampS = 1;
  request.environment.collision = bitmap.view(1,1);
  request.environment.predictions = {&crossing,1,2,1};
  const nav_kernel::local::scan::Grid grid(params,request);
  EXPECT_FALSE(grid.boundaryDepartureFree({{0,0,.4},0},{.35,0,.4}));
}

LocalPlannerParams scanParams() {
  LocalPlannerParams params;
  params.backend = LocalPlannerBackend::Scan;
  params.checkObstacle = true;
  params.useTraversabilityCost = false;
  params.autonomySpeed = 0.5;
  params.maxSpeed = 0.5;
  params.adjacentRange = 2.5;
  params.scan.voxelResolution = 0.1;
  params.scan.controlPointSpacing = 0.2;
  params.scan.cylinderOffset = 0.0;
  params.scan.maxAcceleration = 0.5;
  return params;
}

struct RequestFixture {
  explicit RequestFixture(std::vector<Vec3> value, double resolution = 0.1)
      : route(std::move(value)), bitmap({-5.0, -5.0, -1.0}, {5.0, 5.0, 2.0}, resolution) {
    request.robot.pose = {route.front(), 0.0};
    request.objective = RouteTarget{{route.data(), static_cast<int>(route.size()), 1}};
    request.identity = {1, 1, 0};
    request.clock.timestampS = 1.0;
    request.environment.collision = bitmap.view(1.0, 1);
  }

  void refreshCollision(std::uint64_t generation) {
    request.identity.obstacleGeneration = generation;
    request.environment.collision = bitmap.view(request.clock.timestampS, generation);
  }

  std::vector<Vec3> route;
  CollisionBitmap bitmap;
  LocalPlanRequest request;
};

}  // namespace

TEST(ScanPredictions, WorkerConsumesPredictionUpdatesWithoutChangingMeasuredMap) {
  RequestFixture fixture({{0, 0, .5}, {2, 0, .5}});
  const nav_kernel::PredictedObstacle crossing{{2,0,.5},{2,.1,.5},.15,.1,1.0};
  fixture.request.environment.predictions = {&crossing,1,100.0,1.0};
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  nav_kernel::local::scan::Update update;
  for (int tick = 0; tick < 400; ++tick) {
    fixture.request.clock.timestampS += .005;
    update = task.update(fixture.request);
    if (update.debug.predictedObstacleCount == 1) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_EQ(update.debug.predictedObstacleCount, 1U);
  EXPECT_EQ(fixture.bitmap.view(1,1).occupiedCount(), 0U);
  fixture.request.environment.predictions = {nullptr,0,100.0};
  for (int tick = 0; tick < 400; ++tick) {
    fixture.request.clock.timestampS += .005;
    update = task.update(fixture.request);
    if (update.plan.ready() && update.debug.predictedObstacleCount == 0) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_TRUE(update.plan.ready()) << update.debug.searchReason;
  EXPECT_EQ(update.debug.predictedObstacleCount, 0U);
  EXPECT_EQ(fixture.bitmap.view(1,1).occupiedCount(), 0U);
}

TEST(ScanPredictions, UpstreamCandidateUsesLiveMapWithoutForecastDetour) {
  RequestFixture fixture({{0, 0, .5}, {3, 0, .5}});
  const nav_kernel::PredictedObstacle crossing{{1.2,-.2,.5},{1.2,.2,.5},.1,.1,1.0};
  fixture.request.environment.predictions = {&crossing,1,100.0,1.0};
  auto params = scanParams();
  params.vehicleWidth = .4;
  params.scan.cylinderRadius = .3;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 100 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += .01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
  const nav_kernel::local::scan::Grid grid(params,fixture.request);
  auto previous = spline.position(0);
  double lateral = 0;
  for (double t = .01; t <= spline.duration(); t += .01) {
    const auto current = spline.position(t);
    EXPECT_EQ(grid.cylinderOccupancy(current,0),0);
    lateral = std::max(lateral,std::abs(current.y));
    previous = current;
  }
  EXPECT_LT(lateral,params.scan.voxelResolution);
  EXPECT_NE(grid.brakingOccupancy({{1.2,0,.5},0},{},.6,.5,1),0);
  EXPECT_GT(spline.position(spline.duration()).x,1.5);
}

TEST(ScanPredictions, CandidateTargetRecoversAfterAnObstacleHasCrossed) {
  RequestFixture fixture({{0, 0, .5}, {.3, 0, .5}});
  fixture.request.clock.timestampS = 1.25;
  const nav_kernel::PredictedObstacle crossing{{.3,-.25,.5},{.3,2.25,.5},.05,.1,1.0};
  fixture.request.environment.predictions = {&crossing,1,1.35,1.0,1.0};
  fixture.refreshCollision(1);
  auto params = scanParams();
  params.scan.cylinderRadius = .1;
  const nav_kernel::local::scan::Grid grid(params,fixture.request);
  // The object crossed y=0 at t=1.1. It is now at y=.375: its past
  // sweep is clear. Geometry sees only its current position; the timed
  // trajectory query remains responsible for the future position.
  EXPECT_EQ(grid.inflatedOccupancy({.3,0,.5},0),0);
  EXPECT_EQ(grid.inflatedOccupancy({.3,.375,.5},0),1);
  EXPECT_EQ(grid.inflatedOccupancy({.3,1,.5},0),0);
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 5 && !plan.ready(); ++tick) {
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
  for (double t = 0; t < spline.duration(); t += .01) {
    const double next = std::min(t + .01, spline.duration());
    EXPECT_EQ(grid.trajectoryOccupancy(spline.position(t), spline.velocity(t),
        spline.position(next), spline.velocity(next), t, next), 0);
  }
  EXPECT_NEAR(spline.position(spline.duration()).x, .3, .01);
  // Trimming prediction history must never clear independent measured occupancy.
  fixture.bitmap.occupy({.3,0,.5});
  fixture.refreshCollision(2);
  const nav_kernel::local::scan::Grid measured(params,fixture.request);
  EXPECT_EQ(measured.inflatedOccupancy({.3,0,.5},0),1);
}

TEST(ScanPredictions, PlansToTargetAfterPredictedCrossingWithoutWaitingForExpiry) {
  RequestFixture fixture({{0, 0, .5}, {.3, 0, .5}});
  fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
  // Cross at 0.1 s, before the robot can reach this target. The terminal
  // prediction remains away from the route throughout the final approach.
  const nav_kernel::PredictedObstacle crossing{{.3,-.25,.5},{.3,2.25,.5},.05,.1,1.0};
  fixture.request.environment.predictions = {&crossing,1,2.0,1.0,1.0};
  auto params = scanParams();
  params.scan.cylinderRadius = .1;
  const nav_kernel::local::scan::Grid grid(params,fixture.request);
  ASSERT_TRUE(grid.predictionIntersects(fixture.route.back(),0,fixture.route.back(),0));
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 5 && !plan.ready(); ++tick) {
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason << ": "
      << backend.debugSnapshot().scanAttempt.reason;
  const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
  for (double t = 0; t < spline.duration(); t += .01) {
    const double next = std::min(t + .01, spline.duration());
    EXPECT_EQ(grid.trajectoryOccupancy(spline.position(t), spline.velocity(t),
        spline.position(next), spline.velocity(next), t, next), 0);
  }
  EXPECT_NEAR(spline.position(spline.duration()).x, .3, .01);
  EXPECT_NEAR(spline.position(spline.duration()).y, 0, .01);
  EXPECT_EQ(grid.predictionCount(), 1U);
}

TEST(ScanPredictions, UpstreamCandidateDoesNotTreatForecastAsMeasuredOccupancy) {
  RequestFixture fixture({{0, 0, .5}, {.3, 0, .5}});
  fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
  const nav_kernel::PredictedObstacle crossing{{.3,-1,.5},{.3,1,.5},.05,.1,1.0};
  // Cross at t=1 s, during the acceleration-feasible short-route approach.
  fixture.request.environment.predictions = {&crossing,1,3.0,1.0,2.0};
  auto params = scanParams();
  params.scan.cylinderRadius = .1;
  const nav_kernel::local::scan::Grid grid(params,fixture.request);
  ASSERT_EQ(grid.inflatedOccupancy(fixture.route.back(),0),0);
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 5; ++tick) plan = backend.tick(fixture.request);
  EXPECT_TRUE(plan.ready());
  EXPECT_FALSE(backend.debugSnapshot().scanAttempt.collisionValid)
      << backend.debugSnapshot().scanAttempt.reason;
}

TEST(ScanPredictions, StationaryRetryResumesAfterMeasuredObstacleClears) {
  RequestFixture fixture({{0, 0, .5}, {.3, 0, .5}});
  fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
  const nav_kernel::PredictedObstacle crossing{{.3,-1,.5},{.3,1,.5},.05,.1,1.0};
  fixture.request.environment.predictions = {&crossing,1,3.0,1.0,2.0};
  auto params = scanParams();
  params.scan.cylinderRadius = .1;
  fixture.bitmap.occupyInflated({0,0,.5},.4,.2,.2);
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 5; ++tick) plan = backend.tick(fixture.request);
  ASSERT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Blocked);

  fixture.bitmap.clear();
  fixture.refreshCollision(3);
  fixture.request.clock.timestampS = 1.8;
  for (int tick = 0; tick < 5 && !plan.ready(); ++tick)
    plan = backend.tick(fixture.request);
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason << ": "
      << backend.debugSnapshot().scanAttempt.reason;
  const nav_kernel::local::scan::Grid aged(params,fixture.request);
  ASSERT_EQ(aged.predictionCount(), 1U);
  const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
  for (double t = 0; t < spline.duration(); t += .01) {
    const double next = std::min(t + .01, spline.duration());
    EXPECT_EQ(aged.trajectoryOccupancy(spline.position(t), spline.velocity(t),
        spline.position(next), spline.velocity(next), t, next), 0);
  }
  EXPECT_NEAR(spline.position(spline.duration()).x, .3, .01);
  EXPECT_EQ(fixture.request.environment.collision.generation, 3U);
  EXPECT_EQ(fixture.bitmap.view().occupiedCount(), 0U);
}

namespace {

struct ScanAttemptFixture {
  RequestFixture request{{{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}}};
  nav_kernel::local::scan::Grid grid{scanParams(), request.request};
  nav_kernel::local::scan::upstream::SCANPlannerManager manager;

  explicit ScanAttemptFixture(bool motion_intent = false) {
    using namespace nav_kernel::local::scan::upstream;
    PlanParameters plan;
    plan.max_vel_ = motion_intent ? 0.2 : 0.5;
    plan.max_acc_ = 0.5;
    plan.max_jerk_ = 4.0;
    plan.ctrl_pt_dist = 0.2;
    plan.motion_intent_ = motion_intent;
    BsplineOptimizerParams optimizer;
    optimizer.lambda_smooth = 1.0;
    optimizer.lambda_collision = 0.5;
    optimizer.lambda_feasibility = 0.1;
    optimizer.lambda_fitness = 1.0;
    optimizer.dist0 = 0.2;
    optimizer.max_vel = plan.max_vel_;
    optimizer.max_acc = plan.max_acc_;
    manager.initPlanModules(plan, optimizer, std::make_shared<GridMap>(grid));
  }

  bool replan(const Eigen::Vector3d &velocity = Eigen::Vector3d::Zero(),
              const Eigen::Vector3d &acceleration = Eigen::Vector3d::Zero()) {
    return manager.reboundReplan({0.0, 0.0, 0.5}, velocity, acceleration,
                                 {2.0, 0.0, 0.5}, Eigen::Vector3d::Zero(),
                                 true, false, 1.0);
  }
};

}  // namespace

TEST(ScanRouteContinuity, MovingCruiseBoundaryDoesNotRestartAStopToStopTimingProfile) {
  using namespace nav_kernel::local::scan::upstream;
  ScanAttemptFixture fixture;
  PlanParameters plan;
  plan.max_vel_ = .6;
  plan.max_acc_ = 2;
  plan.max_jerk_ = 4;
  plan.ctrl_pt_dist = .2;
  plan.preserve_boundary_ = true;
  BsplineOptimizerParams optimizer;
  optimizer.lambda_smooth = 1;
  optimizer.lambda_collision = .5;
  optimizer.lambda_feasibility = .1;
  optimizer.lambda_fitness = 1;
  optimizer.dist0 = .2;
  optimizer.max_vel = plan.max_vel_;
  optimizer.max_acc = plan.max_acc_;
  fixture.manager.updatePlanParameters(plan,optimizer);
  const Eigen::Vector3d start{0,0,.5}, velocity{.58,0,0}, acceleration{.005,0,0};
  const Eigen::Vector3d goal{3.5,0,.5}, terminal{.6,0,0};
  ASSERT_TRUE(fixture.manager.reboundReplan(start,velocity,acceleration,
      goal,terminal,true,false,1)) << fixture.manager.reboundDebug().reason;
  const auto &trajectory=fixture.manager.local_data_.position_traj_;
  const auto derivative=trajectory.getDerivative();
  const auto second=derivative.getDerivative();
  const double duration=trajectory.getTimeSum();
  EXPECT_LT((trajectory.evaluateDeBoorT(0)-start).norm(),1e-9);
  EXPECT_LT((derivative.evaluateDeBoorT(0)-velocity).norm(),1e-9);
  EXPECT_LT((second.evaluateDeBoorT(0)-acceleration).norm(),1e-9);
  EXPECT_LT((trajectory.evaluateDeBoorT(duration)-goal).norm(),1e-9);
  EXPECT_LT((derivative.evaluateDeBoorT(duration)-terminal).norm(),1e-9);
  double minimum=.6,maximum=0;
  for(double t=0;t<=duration;t+=.01) {
    const double speed=derivative.evaluateDeBoorT(t).norm();
    minimum=std::min(minimum,speed);
    maximum=std::max(maximum,speed);
  }
  std::cout << "CRUISE_REPLAN duration="<<duration<<" min_speed="<<minimum
            <<" max_speed="<<maximum<<'\n';
  // An already cruising robot should not lose more than 5% of the speed cap
  // solely because a replan preserves its small nonzero acceleration.
  EXPECT_GE(minimum,.57);
  EXPECT_LE(maximum,.6+1e-9);
}

TEST(ScanRouteContinuity, TinyMeasuredVelocityDoesNotPreventAValidStraightRoute) {
  using namespace nav_kernel::local::scan::upstream;
  ScanAttemptFixture fixture;
  PlanParameters plan;
  plan.max_vel_ = 1;
  plan.max_acc_ = 4;
  plan.max_jerk_ = 4;
  plan.ctrl_pt_dist = .2;
  plan.preserve_boundary_ = true;
  plan.max_body_vx_ = 1;
  plan.max_body_vy_ = 1;
  plan.body_yaw_ = -.00012878145854466894;
  BsplineOptimizerParams optimizer;
  optimizer.lambda_smooth = 1;
  optimizer.lambda_collision = 1;
  optimizer.lambda_feasibility = .1;
  optimizer.lambda_fitness = 1;
  optimizer.dist0 = .2;
  optimizer.max_vel = plan.max_vel_;
  optimizer.max_acc = plan.max_acc_;
  fixture.manager.updatePlanParameters(plan, optimizer);
  const Eigen::Vector3d start{1.503324777581161, -.00035954394675755287, .4451614209995494};
  const Eigen::Vector3d velocity{.00010550980882699271, -.000040096237653109774, -.0002806421528990183};
  const Eigen::Vector3d target{2.7, 0, .435};
  const std::vector<Eigen::Vector3d> waypoints{{1.5923230808036637, .00014264825003915135,
                                                .45185763203631735}, target};
  ASSERT_TRUE(fixture.manager.planGlobalTrajWaypoints(start, velocity,
      Eigen::Vector3d::Zero(), waypoints, Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), 1));
  ASSERT_TRUE(fixture.manager.reboundReplan(start, velocity, Eigen::Vector3d::Zero(),
      target, Eigen::Vector3d::Zero(), true, false, 1,
      std::pair<double, double>{0, fixture.manager.global_data_.global_duration_}))
      << fixture.manager.reboundDebug().stage << ':' << fixture.manager.reboundDebug().reason;
  const auto &trajectory = fixture.manager.local_data_.position_traj_;
  const auto derivative = trajectory.getDerivative();
  const auto acceleration = derivative.getDerivative();
  ASSERT_LT((derivative.evaluateDeBoorT(0)-velocity).norm(), 1e-9);
  ASSERT_LT((trajectory.evaluateDeBoorT(trajectory.getTimeSum())-target).norm(), 1e-9);
  for (double t=0; t<=trajectory.getTimeSum(); t+=.01) {
    EXPECT_LE(plan.speedRatio(derivative.evaluateDeBoorT(t)), 1+1e-9);
    EXPECT_LE(acceleration.evaluateDeBoorT(t).norm(), plan.max_acc_+1e-9);
  }
}

TEST(ScanRouteContinuity,
     ReversingBoundaryProjectionDoesNotStretchTheMovingHandoff) {
  using namespace nav_kernel::local::scan::upstream;
  ScanAttemptFixture fixture;
  PlanParameters plan;
  plan.max_vel_ = .6;
  plan.max_acc_ = 2;
  plan.max_jerk_ = 4;
  plan.ctrl_pt_dist = .2;
  plan.preserve_boundary_ = true;
  BsplineOptimizerParams optimizer;
  optimizer.lambda_smooth = 1;
  optimizer.lambda_collision = .5;
  optimizer.lambda_feasibility = .1;
  optimizer.lambda_fitness = 1;
  optimizer.dist0 = .2;
  optimizer.max_vel = plan.max_vel_;
  optimizer.max_acc = plan.max_acc_;
  fixture.manager.updatePlanParameters(plan, optimizer);

  const Eigen::Vector3d start{0, 0, .5};
  const Eigen::Vector3d goal{3.5, 0, .5};
  const Eigen::Vector3d start_velocity{.5, 0, 0};
  const Eigen::Vector3d reversing_terminal{-.1, 0, 0};
  ASSERT_TRUE(fixture.manager.reboundReplan(
      start, start_velocity, Eigen::Vector3d::Zero(), goal,
      reversing_terminal, true, false, 1));
  const double duration = fixture.manager.local_data_.position_traj_.getTimeSum();
  // The reverse terminal tangent must not be interpreted as extra forward
  // travel. The old projection extended this handoff beyond the baseline
  // stop-to-stop estimate and caused early measured braking in MuJoCo.
  EXPECT_LT(duration, 8.0);
}

TEST(ScanRouteContinuity, DiagonalTrajectoryUsesTheFollowersExistingBodyAxisSpeeds) {
  using namespace nav_kernel::local::scan::upstream;
  for (double yaw : {0.0,.6}) {
    SCOPED_TRACE(yaw);
    ScanAttemptFixture fixture;
    PlanParameters plan;
    plan.max_vel_ = .6;
    plan.max_acc_ = 2;
    plan.max_jerk_ = 4;
    plan.ctrl_pt_dist = .2;
    plan.preserve_boundary_ = true;
    plan.max_body_vx_ = .6;
    plan.max_body_vy_ = .35;
    plan.body_yaw_ = yaw;
    BsplineOptimizerParams optimizer;
    optimizer.lambda_smooth = 1;
    optimizer.lambda_collision = .5;
    optimizer.lambda_feasibility = .1;
    optimizer.lambda_fitness = 1;
    optimizer.dist0 = .2;
    optimizer.max_vel = plan.max_vel_;
    optimizer.max_acc = plan.max_acc_;
    fixture.manager.updatePlanParameters(plan,optimizer);
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d start{0,0,.5};
    const Eigen::Vector3d goal = start + rotation * Eigen::Vector3d(1.5,-1.5,0);
    const Eigen::Vector3d terminal = rotation * Eigen::Vector3d(.35,-.35,0);
    ASSERT_TRUE(fixture.manager.reboundReplan(start,Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),goal,terminal,true,false,1))
        << fixture.manager.reboundDebug().stage << ':' << fixture.manager.reboundDebug().reason;
    const auto &trajectory = fixture.manager.local_data_.position_traj_;
    const auto velocity = trajectory.getDerivative();
    const double duration = trajectory.getTimeSum();
    EXPECT_LT((trajectory.evaluateDeBoorT(duration)-goal).norm(),1e-9);
    EXPECT_LT((velocity.evaluateDeBoorT(duration)-terminal).norm(),1e-9);
    for(double t=0;t<=duration;t+=.01) {
      const Eigen::Vector3d body_velocity = rotation.transpose()*velocity.evaluateDeBoorT(t);
      EXPECT_LE(std::abs(body_velocity.x()),.6+1e-8);
      EXPECT_LE(std::abs(body_velocity.y()),.35+1e-8);
      EXPECT_LE(body_velocity.norm(),.6+1e-8);
    }
  }
}

TEST(ScanMotionIntentBoundary, KeepsNonzeroInitialVelocityAndAccelerationWithinFinalBounds) {
  ScanAttemptFixture fixture(true);
  const Eigen::Vector3d start(0.0, 0.0, 0.5);
  const Eigen::Vector3d initial_velocity(0.1, 0.0, 0.0);
  const Eigen::Vector3d initial_acceleration(0.1, 0.0, 0.0);
  ASSERT_TRUE(fixture.replan(initial_velocity, initial_acceleration))
      << fixture.manager.reboundDebug().stage << ":" << fixture.manager.reboundDebug().reason;
  const auto &position = fixture.manager.local_data_.position_traj_;
  const auto velocity = position.getDerivative();
  const auto acceleration = velocity.getDerivative();
  const double duration = position.getTimeSum();
  EXPECT_LE((position.evaluateDeBoorT(0.0) - start).norm(), 1e-9);
  EXPECT_LE((velocity.evaluateDeBoorT(0.0) - initial_velocity).norm(), 1e-9);
  EXPECT_LE((acceleration.evaluateDeBoorT(0.0) - initial_acceleration).norm(), 1e-9);
  EXPECT_LE((position.evaluateDeBoorT(duration) - Eigen::Vector3d(2.0, 0.0, 0.5)).norm(), 1e-9);
  EXPECT_LE(velocity.evaluateDeBoorT(duration).norm(), 1e-9);
  EXPECT_LE(acceleration.evaluateDeBoorT(duration).norm(), 1e-9);
  EXPECT_LE(velocity.getControlPoint().colwise().norm().maxCoeff(), 0.2 + 1e-9);
  EXPECT_LE(acceleration.getControlPoint().colwise().norm().maxCoeff(), 0.5 + 1e-9);
}

TEST(ScanMotionIntentBoundary, ClearThreeDimensionalGuideRollsThroughWhileRouteStops) {
  for (const double requestedSpeed : {0.2, 0.5}) {
    SCOPED_TRACE(requestedSpeed);
    for (const bool motionIntent : {true, false}) {
      SCOPED_TRACE(motionIntent ? "motion intent" : "route target");
      RequestFixture fixture({{0.0, 0.0, 0.5}, {1.6, 0.8, 1.1}});
      fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
      fixture.request.maxLinearSpeedMps = requestedSpeed;
      if (motionIntent) {
        fixture.request.objective = nav_kernel::MotionIntentTarget{
            {std::atan2(0.8, 1.6) * 180.0 / M_PI, 1.0, 3.5, 90.0},
            {fixture.route.data(), 2, 1}};
      }
      auto params = scanParams();
      params.scan.maxVelocity = 0.3;
      const double effectiveSpeed = std::min(requestedSpeed, params.scan.maxVelocity);
      nav_kernel::local::scan::Backend backend(params);
      LocalPlan plan;
      for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
        fixture.request.clock.timestampS += 0.01;
        plan = backend.tick(fixture.request);
      }
      ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason << ": "
                               << backend.debugSnapshot().scanAttempt.reason;
      const auto &target = std::get<SplineTarget>(plan.target());
      EXPECT_DOUBLE_EQ(target.maxLinearSpeedMps, effectiveSpeed);
      const nav_kernel::SplineView spline(target);
      ASSERT_TRUE(spline.valid());
      EXPECT_LT(nav_kernel::distance3D(spline.position(spline.duration()),
                                      fixture.route.back()), 1e-3);
      const auto terminalVelocity = spline.velocity(spline.duration());
      if (motionIntent) {
        const Eigen::Vector3d tangent = Eigen::Vector3d(1.6, 0.8, 0.6).normalized();
        EXPECT_NEAR(terminalVelocity.x, effectiveSpeed * tangent.x(), 1e-3);
        EXPECT_NEAR(terminalVelocity.y, effectiveSpeed * tangent.y(), 1e-3);
        EXPECT_NEAR(terminalVelocity.z, effectiveSpeed * tangent.z(), 1e-3);
      } else {
        EXPECT_NEAR(terminalVelocity.x, 0.0, 1e-3);
        EXPECT_NEAR(terminalVelocity.y, 0.0, 1e-3);
        EXPECT_NEAR(terminalVelocity.z, 0.0, 1e-3);
      }
    }
  }
}

TEST(ScanMotionIntentBoundary, LateralGuideUsesObservedBodyAxisCapacityWithoutFixingCollisionHeading) {
  namespace collision = lingtu::nav::collision;
  for (double yaw : {0.0, .6}) {
    for (double side : {-1.0, 1.0}) {
      SCOPED_TRACE(::testing::Message() << "yaw=" << yaw << " side=" << side);
      const double c = std::cos(yaw), s = std::sin(yaw);
      RequestFixture fixture({{0,0,.5},{-s * side * 2,c * side * 2,.5}});
      fixture.request.robot.pose.yaw = yaw;
      fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
      fixture.request.maxLinearSpeedMps = .6;
      fixture.request.objective = nav_kernel::MotionIntentTarget{
          {side * 90,1,3.5,90},{fixture.route.data(),2,1}};
      auto body = std::make_shared<collision::Body>();
      body->margin_m = 0;
      collision::Part part;
      part.half_size = {.2,.1,.1};
      body->parts.push_back(part);
      fixture.request.body = body;
      const auto &map = fixture.request.environment.collision;
      fixture.request.environment.collision.measuredOccupiedStorage =
          std::make_shared<const std::vector<std::uint8_t>>(map.inflatedBytes,0U);
      fixture.request.environment.collision.knownFreeStorage =
          std::make_shared<const std::vector<std::uint8_t>>(map.inflatedBytes,255U);
      auto params = scanParams();
      params.scan.maxVelocity = .6;
      params.scan.maxAcceleration = 2;
      params.scan.maxBodyVx = .6;
      params.scan.maxBodyVy = .35;
      nav_kernel::local::scan::Backend backend(params);
      LocalPlan plan;
      for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
        fixture.request.clock.timestampS += .01;
        plan = backend.tick(fixture.request);
      }
      ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason << ':'
                               << backend.debugSnapshot().scanAttempt.reason;
      ASSERT_FALSE(fixture.request.bodyHeading.has_value());
      const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
      ASSERT_TRUE(spline.valid());
      double maximumSideSpeed = 0;
      for (double t = 0; t <= spline.duration(); t += .01) {
        const auto velocity = spline.velocity(t);
        const double vx = c * velocity.x + s * velocity.y;
        const double vy = -s * velocity.x + c * velocity.y;
        maximumSideSpeed = std::max(maximumSideSpeed,side * vy);
        EXPECT_LE(std::abs(vx),.6 + 1e-8);
        EXPECT_LE(std::abs(vy),.35 + 1e-8);
        EXPECT_LE(std::hypot(vx,vy),.6 + 1e-8);
      }
      EXPECT_GT(maximumSideSpeed,.3);
      const auto terminal = spline.velocity(spline.duration());
      EXPECT_NEAR(side * (-s * terminal.x + c * terminal.y),.35,1e-8);
      EXPECT_LT(nav_kernel::distance3D(spline.position(spline.duration()),
                                      fixture.route.back()),1e-8);
    }
  }
}

TEST(ScanMotionIntentRetiming,
     PreservesMovingStartForShortLateralTargetWithinLimits) {
  using namespace nav_kernel::local::scan::upstream;
  const Eigen::Vector3d start(0.6034440286763203, -0.1933650899937829,
                              -0.0011775607026522361);
  const Eigen::Vector3d initial_acceleration = Eigen::Vector3d::Zero();
  const Eigen::Vector3d target(0.82179419696400013, 0.25643842712090337,
                               -0.0011775607026522361);
  const std::vector<Eigen::Vector3d> initial_velocities{
      {0.32530685229850964, -0.0578586930675915, -0.026038490671518016},
      {0.5, 0.0, 0.0}};
  for (const Eigen::Vector3d &initial_velocity : initial_velocities) {
    SCOPED_TRACE(::testing::Message() << "initial_velocity="
                                      << initial_velocity.transpose());
    RequestFixture request({{start.x(), start.y(), start.z()},
                            {target.x(), target.y(), target.z()}},
                           0.05);
    auto params = scanParams();
    params.scan.voxelResolution = 0.05;
    params.scan.maxVelocity = 0.5;
    params.scan.maxAcceleration = 0.5;
    nav_kernel::local::scan::Grid grid(params, request.request);

    PlanParameters plan;
    plan.max_vel_ = 0.5;
    plan.max_acc_ = 0.5;
    plan.max_jerk_ = 4.0;
    plan.ctrl_pt_dist = 0.2;
    plan.motion_intent_ = true;
    BsplineOptimizerParams optimizer;
    optimizer.lambda_smooth = 1.0;
    optimizer.lambda_collision = 1.0;
    optimizer.lambda_feasibility = 0.1;
    optimizer.lambda_fitness = 1.0;
    optimizer.dist0 = 0.2;
    optimizer.max_vel = plan.max_vel_;
    optimizer.max_acc = plan.max_acc_;
    SCANPlannerManager manager;
    manager.initPlanModules(plan, optimizer, std::make_shared<GridMap>(grid));

    ASSERT_TRUE(manager.reboundReplan(
        start, initial_velocity, initial_acceleration, target,
        Eigen::Vector3d::Zero(), true, false, 1.0))
        << manager.reboundDebug().stage << ":" << manager.reboundDebug().reason;
    const auto &position = manager.local_data_.position_traj_;
    const auto velocity = position.getDerivative();
    const auto acceleration = velocity.getDerivative();
    const double duration = position.getTimeSum();
    EXPECT_LE((position.evaluateDeBoorT(0.0) - start).norm(), 1e-9);
    EXPECT_LE((velocity.evaluateDeBoorT(0.0) - initial_velocity).norm(), 1e-9);
    EXPECT_LE((acceleration.evaluateDeBoorT(0.0) - initial_acceleration).norm(),
              1e-9);
    EXPECT_LE((position.evaluateDeBoorT(duration) - target).norm(), 1e-9);
    EXPECT_LE(velocity.evaluateDeBoorT(duration).norm(), 1e-9);
    EXPECT_LE(acceleration.evaluateDeBoorT(duration).norm(), 1e-9);
    EXPECT_LE(velocity.getControlPoint().colwise().norm().maxCoeff(),
              0.5 + 1e-9);
    EXPECT_LE(acceleration.getControlPoint().colwise().norm().maxCoeff(),
              0.5 + 1e-9);
  }
}

TEST(ScanMotionIntentRetiming, ConvergesWithNonzeroThreeAxisStartAcceleration) {
  using namespace nav_kernel::local::scan::upstream;
  // Physical boundary from a Go2 replan that exhausted three dilation passes.
  const Eigen::Vector3d start(1.2191940642, -0.2191074321, 0.0653090089);
  const Eigen::Vector3d initial_velocity(0.3141680282, -0.0369077086, 0.1247349412);
  const Eigen::Vector3d initial_acceleration(-0.1668969950, 0.0574541903, -0.1276989702);
  const Eigen::Vector3d target(4.0680212247, 0.1674350790, -0.0102147490);
  RequestFixture request({{start.x(), start.y(), start.z()},
                          {target.x(), target.y(), target.z()}}, 0.05);
  auto params = scanParams();
  params.scan.voxelResolution = 0.05;
  nav_kernel::local::scan::Grid grid(params, request.request);
  PlanParameters plan;
  plan.max_vel_ = 0.5;
  plan.max_acc_ = 0.5;
  plan.max_jerk_ = 4.0;
  plan.ctrl_pt_dist = 0.2;
  plan.motion_intent_ = true;
  BsplineOptimizerParams optimizer;
  optimizer.lambda_smooth = 1.0;
  optimizer.lambda_collision = 1.0;
  optimizer.lambda_feasibility = 0.1;
  optimizer.lambda_fitness = 1.0;
  optimizer.dist0 = 0.2;
  optimizer.max_vel = plan.max_vel_;
  optimizer.max_acc = plan.max_acc_;
  SCANPlannerManager manager;
  manager.initPlanModules(plan, optimizer, std::make_shared<GridMap>(grid));

  ASSERT_TRUE(manager.reboundReplan(start, initial_velocity, initial_acceleration,
                                  target, Eigen::Vector3d::Zero(), true, false, 1.0))
      << manager.reboundDebug().stage << ":" << manager.reboundDebug().reason;
  const auto &position = manager.local_data_.position_traj_;
  const auto velocity = position.getDerivative();
  const auto acceleration = velocity.getDerivative();
  const double duration = position.getTimeSum();
  EXPECT_LE((position.evaluateDeBoorT(0.0) - start).norm(), 1e-9);
  EXPECT_LE((velocity.evaluateDeBoorT(0.0) - initial_velocity).norm(), 1e-9);
  EXPECT_LE((acceleration.evaluateDeBoorT(0.0) - initial_acceleration).norm(), 1e-9);
  EXPECT_LE((position.evaluateDeBoorT(duration) - target).norm(), 1e-9);
  EXPECT_LE(velocity.evaluateDeBoorT(duration).norm(), 1e-9);
  EXPECT_LE(acceleration.evaluateDeBoorT(duration).norm(), 1e-9);
  EXPECT_LE(velocity.getControlPoint().colwise().norm().maxCoeff(), 0.5 + 1e-9);
  EXPECT_LE(acceleration.getControlPoint().colwise().norm().maxCoeff(), 0.5 + 1e-9);
}

TEST(ScanBsplineEvaluation, MatchesVectorRecurrence) {
  using nav_kernel::local::scan::upstream::UniformBspline;
  for (int dimension : {1, 3, 5}) {
    for (int degree = 0; degree <= 5; ++degree) {
      Eigen::MatrixXd controls(dimension, 10);
      for (int column = 0; column < controls.cols(); ++column)
        for (int row = 0; row < dimension; ++row)
          controls(row, column) = std::sin(.3 * column + .2 * row);
      UniformBspline spline(controls, degree, .2);
      auto knots = spline.getKnot();
      for (int index = 1; index < knots.size(); ++index)
        knots(index) = knots(index - 1) + .1 + .03 * (index % 3);
      spline.setKnot(knots);
      const double start = knots(degree);
      const double end = knots(controls.cols());
      for (int sample = -1; sample <= 21; ++sample) {
        const double time = start + (end - start) * sample / 20.0;
        const double bounded = std::clamp(time, start, end);
        int knot = degree;
        while (knots(knot + 1) < bounded) ++knot;
        std::vector<Eigen::VectorXd> values;
        for (int index = 0; index <= degree; ++index)
          values.push_back(controls.col(knot - degree + index));
        for (int level = 1; level <= degree; ++level)
          for (int index = degree; index >= level; --index) {
            const double alpha = (bounded - knots(index + knot - degree)) /
                (knots(index + 1 + knot - level) - knots(index + knot - degree));
            values[index] = (1 - alpha) * values[index - 1] + alpha * values[index];
          }
        EXPECT_LE((spline.evaluateDeBoor(time) - values.back()).norm(), 1e-12)
            << dimension << ',' << degree << ',' << sample;
      }
    }
  }
}

TEST(ScanBsplineFeasibility, SpeedOnlyViolationUsesActualSpeedRatio) {
  using nav_kernel::local::scan::upstream::UniformBspline;
  constexpr double interval = 0.2;
  Eigen::MatrixXd controls = Eigen::MatrixXd::Zero(3, 10);
  for (int index = 0; index < controls.cols(); ++index)
    controls(0, index) = 0.051 * index * interval;
  UniformBspline spline(controls, 3, interval);
  spline.setPhysicalLimits(0.05, 0.5, 0.0);
  double ratio = 0.0;
  EXPECT_FALSE(spline.checkFeasibility(ratio, false));
  EXPECT_NEAR(ratio, 1.02, 1e-12);
}

TEST(ScanBsplineParameterization, MatchesDenseLeastSquaresWithThreeAxisBoundaryData) {
  using nav_kernel::local::scan::upstream::UniformBspline;
  // Keep the original dense mathematical problem as a reference for the sparse
  // solver, including inconsistent observations that require least squares.
  for (const int count : {7, 93, 772}) {
    SCOPED_TRACE(count);
    const double interval = count == 772 ? 0.09128709291752769 : 0.2;
    Eigen::MatrixXd collocation = Eigen::MatrixXd::Zero(count + 4, count + 2);
    Eigen::MatrixXd observations(count + 4, 3);
    std::vector<Eigen::Vector3d> points;
    for (int index = 0; index < count; ++index) {
      const double phase = static_cast<double>(index) / (count - 1);
      points.emplace_back(3.5 * phase, 0.3 * std::sin(4.0 * phase),
                          0.5 - 0.1 * phase + 0.02 * std::sin(7.0 * phase));
      observations.row(index) = points.back().transpose();
      collocation.block<1, 3>(index, index) << 1.0 / 6.0, 4.0 / 6.0, 1.0 / 6.0;
    }
    const std::vector<Eigen::Vector3d> derivatives{
        {-0.05, 0.10, -0.01}, {0.03, -0.04, 0.02},
        {0.02, -0.03, 0.01}, {-0.01, 0.02, -0.03}};
    for (int end = 0; end < 2; ++end) {
      const int column = end * (count - 1);
      collocation.block<1, 3>(count + end, column) <<
          -0.5 / interval, 0.0, 0.5 / interval;
      collocation.block<1, 3>(count + 2 + end, column) <<
          1.0 / (interval * interval), -2.0 / (interval * interval),
          1.0 / (interval * interval);
    }
    for (int index = 0; index < 4; ++index)
      observations.row(count + index) = derivatives[index].transpose();

    const Eigen::MatrixXd dense =
        collocation.colPivHouseholderQr().solve(observations);
    Eigen::MatrixXd sparse;
    UniformBspline::parameterizeToBspline(interval, points, derivatives, sparse);
    ASSERT_EQ(sparse.rows(), 3);
    ASSERT_EQ(sparse.cols(), count + 2);
    ASSERT_TRUE(sparse.allFinite());
    const double control_error = (sparse.transpose() - dense).cwiseAbs().maxCoeff();
    const double residual_error =
        (collocation * sparse.transpose() - observations).squaredNorm() -
        (collocation * dense - observations).squaredNorm();
    EXPECT_LE(control_error, 1e-9);
    EXPECT_NEAR(residual_error, 0.0, 1e-10);
    RecordProperty("control_error_" + std::to_string(count), std::to_string(control_error));
    const UniformBspline dense_spline(dense.transpose(), 3, interval);
    const UniformBspline sparse_spline(sparse, 3, interval);
    for (const double time : {0.0, sparse_spline.getTimeSum()}) {
      EXPECT_LE((sparse_spline.evaluateDeBoorT(time) -
                 dense_spline.evaluateDeBoorT(time)).norm(), 1e-9);
      EXPECT_LE((sparse_spline.getDerivative().evaluateDeBoorT(time) -
                 dense_spline.getDerivative().evaluateDeBoorT(time)).norm(), 1e-8);
      EXPECT_LE((sparse_spline.getDerivative().getDerivative().evaluateDeBoorT(time) -
                 dense_spline.getDerivative().getDerivative().evaluateDeBoorT(time)).norm(), 1e-7);
    }
  }
}

TEST(ScanBsplineParameterization, ReproducesConsistentNonzeroPositionVelocityAcceleration) {
  using nav_kernel::local::scan::upstream::UniformBspline;
  const Eigen::Vector3d position(2.0, -1.0, 0.4);
  const Eigen::Vector3d velocity(-0.12, 0.08, -0.03);
  const Eigen::Vector3d acceleration(0.01, 0.02, -0.003);
  const Eigen::Vector3d jerk(0.001, -0.002, 0.0004);
  const auto point = [&](double time) {
    return Eigen::Vector3d(position + time * velocity +
        0.5 * time * time * acceleration + time * time * time * jerk / 6.0);
  };
  const auto speed = [&](double time) {
    return Eigen::Vector3d(velocity + time * acceleration + 0.5 * time * time * jerk);
  };
  constexpr double interval = 0.2;
  constexpr int count = 25;
  constexpr double duration = (count - 1) * interval;
  std::vector<Eigen::Vector3d> points;
  for (int index = 0; index < count; ++index) points.push_back(point(index * interval));
  const std::vector<Eigen::Vector3d> derivatives{
      velocity, speed(duration), acceleration, acceleration + duration * jerk};
  Eigen::MatrixXd controls;
  UniformBspline::parameterizeToBspline(interval, points, derivatives, controls);
  const UniformBspline spline(controls, 3, interval);
  EXPECT_NEAR(spline.getTimeSum(), duration, 1e-12);
  for (const double time : {0.0, duration / 2.0, duration}) {
    EXPECT_LE((spline.evaluateDeBoorT(time) - point(time)).norm(), 1e-10);
    EXPECT_LE((spline.getDerivative().evaluateDeBoorT(time) - speed(time)).norm(), 1e-10);
    EXPECT_LE((spline.getDerivative().getDerivative().evaluateDeBoorT(time) -
               (acceleration + time * jerk)).norm(), 1e-9);
  }
}

TEST(ScanAttemptDiagnostics, DistinguishesCollisionFromDynamicFailureAndClearsOnSuccess) {
  ScanAttemptFixture fixture;
  for (double x = -0.25; x < 0.3; x += 0.1)
    for (double y = -0.25; y < 0.3; y += 0.1)
      for (double z = 0.25; z < 0.8; z += 0.1)
        fixture.request.bitmap.occupy({x, y, z});

  ASSERT_FALSE(fixture.replan());
  const auto failed = fixture.manager.reboundDebug();
  EXPECT_TRUE(failed.attempted);
  EXPECT_FALSE(failed.success);
  EXPECT_EQ(failed.attemptId, 1U);
  EXPECT_EQ(failed.stage, "rebound_optimization");
  EXPECT_EQ(failed.reason, "collision_at_trajectory_start");
  EXPECT_TRUE(failed.optimizerReturnCodeValid);
  EXPECT_TRUE(failed.collisionValid);
  EXPECT_EQ(failed.collisionState, 1);
  EXPECT_GE(failed.collisionTimeS, 0.0);
  EXPECT_EQ(fixture.grid.inflatedOccupancy(
                {failed.collisionPosition.x(), failed.collisionPosition.y(),
                 failed.collisionPosition.z()}, 0.0), 1);
  EXPECT_FALSE(failed.dynamicViolationValid);

  ASSERT_GT(failed.candidateControlPoints.cols(), 3);
  ASSERT_GT(failed.candidateIntervalS, 0.0);
  const nav_kernel::local::scan::upstream::UniformBspline rejected(
      failed.candidateControlPoints, 3, failed.candidateIntervalS);
  EXPECT_LT((rejected.evaluateDeBoorT(failed.collisionTimeS) -
             failed.collisionPosition).norm(), 1e-10);

  fixture.request.bitmap.clear();
  fixture.request.refreshCollision(2);
  fixture.grid = nav_kernel::local::scan::Grid(scanParams(), fixture.request.request);
  ASSERT_TRUE(fixture.replan()) << fixture.manager.reboundDebug().reason;
  const auto &ready = fixture.manager.reboundDebug();
  EXPECT_EQ(ready.attemptId, 2U);
  EXPECT_TRUE(ready.success);
  EXPECT_EQ(ready.stage, "complete");
  EXPECT_EQ(ready.reason, "accepted");
  EXPECT_FALSE(ready.collisionValid);
  EXPECT_FALSE(ready.dynamicViolationValid);
  EXPECT_EQ(ready.candidateControlPoints.size(), 0);

  EXPECT_FALSE(fixture.manager.reboundReplan(
      {0.0, 0.0, 0.5}, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      {0.0, 0.0, 0.5}, Eigen::Vector3d::Zero(), true, false, 2.0));
  const auto &empty = fixture.manager.reboundDebug();
  EXPECT_EQ(empty.attemptId, 3U);
  EXPECT_EQ(empty.stage, "initialization");
  EXPECT_EQ(empty.reason, "target_too_close");
  EXPECT_FALSE(empty.optimizerReturnCodeValid);
  EXPECT_FALSE(empty.collisionValid);
  EXPECT_EQ(empty.candidateControlPoints.size(), 0);
}

TEST(ScanAttemptDiagnostics, RecordsMeasuredBoundaryVelocityAndAccelerationViolations) {
  for (const bool speed : {true, false}) {
    SCOPED_TRACE(speed ? "speed" : "acceleration");
    ScanAttemptFixture fixture;
    fixture.manager.pp_.vel_tolerance_ = 0.0;
    fixture.manager.pp_.acc_tolerance_ = 0.0;
    const Eigen::Vector3d velocity = speed ? Eigen::Vector3d{1.0, 0.0, 0.0}
                                          : Eigen::Vector3d::Zero();
    const Eigen::Vector3d acceleration = speed ? Eigen::Vector3d::Zero()
                                              : Eigen::Vector3d{2.0, 0.0, 0.0};
    ASSERT_FALSE(fixture.replan(velocity, acceleration));
    const auto &failed = fixture.manager.reboundDebug();
    EXPECT_EQ(failed.stage, "dynamic_feasibility");
    EXPECT_EQ(failed.reason, speed ? "speed_limit_exceeded" : "acceleration_limit_exceeded");
    EXPECT_EQ(failed.dynamicQuantity, speed ? "speed" : "acceleration");
    EXPECT_TRUE(failed.dynamicViolationValid);
    EXPECT_GT(failed.dynamicValue, failed.dynamicLimit);
    EXPECT_DOUBLE_EQ(failed.dynamicLimit, 0.5);
    EXPECT_GE(failed.dynamicTimeS, 0.0);
    EXPECT_TRUE(failed.optimizerReturnCodeValid);
    EXPECT_FALSE(failed.collisionValid);
    EXPECT_TRUE(failed.polyInit);
    EXPECT_FALSE(failed.randomPolyInit);
    EXPECT_EQ(failed.startVelocity, velocity);
    EXPECT_EQ(failed.startAcceleration, acceleration);
    ASSERT_GT(failed.candidateControlPoints.cols(), 3);
    const nav_kernel::local::scan::upstream::UniformBspline rejected(
        failed.candidateControlPoints, 3, failed.candidateIntervalS);
    const auto derivative = speed ? rejected.getDerivative()
                                  : rejected.getDerivative().getDerivative();
    EXPECT_NEAR(derivative.evaluateDeBoorT(failed.dynamicTimeS).norm(),
                failed.dynamicValue, 1e-10);
  }
}

TEST(ScanAttemptDiagnostics, FixedBodyAxisViolationDoesNotStretchRejectedTrajectory) {
  using namespace nav_kernel::local::scan::upstream;
  for (const double yaw : {0.0, 1.5707963267948966}) {
    for (const bool atTarget : {false, true}) {
      SCOPED_TRACE(::testing::Message() << "yaw=" << yaw << " target=" << atTarget);
      ScanAttemptFixture fixture(true);
      auto plan = fixture.manager.pp_;
      plan.max_vel_ = 1.0;
      plan.max_body_vx_ = 1.0;
      plan.max_body_vy_ = .35;
      plan.body_yaw_ = yaw;
      BsplineOptimizerParams optimizer;
      optimizer.lambda_smooth = optimizer.lambda_collision = optimizer.lambda_fitness = 1;
      optimizer.lambda_feasibility = .1;
      optimizer.dist0 = .2;
      optimizer.max_vel = plan.max_vel_;
      optimizer.max_acc = plan.max_acc_;
      fixture.manager.updatePlanParameters(plan, optimizer);
      const Eigen::Vector3d lateral{-std::sin(yaw) * .4, std::cos(yaw) * .4, 0};
      const auto zero = Eigen::Vector3d::Zero().eval();
      ASSERT_LT(lateral.norm(), plan.max_vel_);
      ASSERT_GT(plan.speedRatio(lateral), 1.0);
      ASSERT_FALSE(fixture.manager.reboundReplan({0,0,.5}, atTarget ? zero : lateral,
          zero, {2,0,.5}, atTarget ? lateral : zero, true, false, 1));
      const auto &failed = fixture.manager.reboundDebug();
      EXPECT_EQ(failed.stage, "dynamic_feasibility");
      // This 20 cm / 1 m/s seed is below 0.25 s. Its fixed bad velocity
      // cannot be repaired by stretching the candidate's execution time.
      EXPECT_LT(failed.candidateIntervalS, .25);
      ASSERT_TRUE(fixture.manager.reboundReplan({0,0,.5}, zero, zero, {2,0,.5},
          zero, true, false, 2)) << fixture.manager.reboundDebug().reason;
    }
  }
}

TEST(ScanReferenceSeed, CollinearSamplesDoNotChangeTheOffsetStartConnector) {
  const std::vector<Eigen::Vector3d> route{{0,0,.5},{2,0,.5},{2,2,.5},{3,2,1}};
  std::vector<Eigen::Vector3d> sparseResult;
  for (bool dense : {false, true}) {
    SCOPED_TRACE(dense);
    ScanAttemptFixture fixture;
    std::vector<Eigen::Vector3d> waypoints;
    for (std::size_t i = 1; i < route.size(); ++i) {
      const int divisions = dense ? 10 : 1;
      for (int j = 1; j <= divisions; ++j)
        waypoints.push_back(route[i-1] + (route[i]-route[i-1])*j/divisions);
    }
    auto &manager = fixture.manager;
    const auto zero = Eigen::Vector3d::Zero().eval();
    ASSERT_TRUE(manager.planGlobalTrajWaypoints(route.front(), zero, zero,
        waypoints, zero, zero, 1));
    ASSERT_TRUE(manager.reboundReplan({.5,-.3,.5}, zero, zero, route.back(), zero,
        true, false, 1, std::pair<double,double>{0, manager.global_data_.global_duration_}))
        << manager.reboundDebug().reason;
    const auto &curve = manager.local_data_.position_traj_;
    for (int i = 0; i <= 100; ++i) {
      const auto point = curve.evaluateDeBoorT(curve.getTimeSum()*i/100);
      if (dense)
        EXPECT_LT((point-sparseResult[i]).norm(), 1e-6) << i;
      else
        sparseResult.push_back(point);
    }
  }
}

TEST(ScanReboundInitialization, UpstreamStopsGuidanceAtCheckedPrefix) {
  using namespace nav_kernel::local::scan::upstream;
  RequestFixture fixture({{0.0, 0.0, 0.5}, {4.0, 0.0, 0.5}});
  fixture.request.objective = nav_kernel::MotionIntentTarget{
      {0.0, 0.4, 4.0, 90.0}, {fixture.route.data(), 2, 1}};
  for (int x = 12; x <= 33; ++x)
    for (int y = -3; y <= 2; ++y)
      fixture.bitmap.occupy({x * 0.1 + 0.05, y * 0.1 + 0.05, 0.55});
  nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
  const auto map = std::make_shared<GridMap>(grid);
  BsplineOptimizer optimizer;
  BsplineOptimizerParams params;
  params.lambda_smooth = params.lambda_collision = params.lambda_fitness = 1.0;
  params.lambda_feasibility = 0.1;
  params.dist0 = 0.2;
  params.max_vel = params.max_acc = 0.5;
  optimizer.setParam(params);
  optimizer.setEnvironment(map);
  optimizer.a_star_ = std::make_shared<AStar>();
  optimizer.a_star_->initGridMap(map, {100, 100, 10});
  Eigen::MatrixXd controls(3, 43);
  for (int i = 0; i < controls.cols(); ++i)
    controls.col(i) = Eigen::Vector3d{0.1 * (i - 1), 0.0, 0.5};
  ASSERT_EQ(grid.inflatedOccupancy({2.7, 0.0, 0.5}, 0.0), 1);
  const auto guides = optimizer.initControlPoints(controls);
  EXPECT_TRUE(guides.empty());
}

TEST(ScanReboundInitialization, UpstreamSeedsFromSampledControls) {
  using namespace nav_kernel::local::scan::upstream;
  RequestFixture fixture({{0.0, -0.4, 0.5}, {2.0, -0.4, 0.5}});
  fixture.bitmap.occupy({0.65, -0.25, 0.55});
  nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
  const auto map = std::make_shared<GridMap>(grid);
  BsplineOptimizer optimizer;
  BsplineOptimizerParams params;
  params.lambda_smooth = params.lambda_collision = params.lambda_fitness = 1.0;
  params.lambda_feasibility = 0.1;
  params.dist0 = 0.2;
  params.max_vel = params.max_acc = 0.5;
  optimizer.setParam(params);
  optimizer.setEnvironment(map);
  optimizer.a_star_ = std::make_shared<AStar>();
  optimizer.a_star_->initGridMap(map, {100, 100, 10});
  Eigen::MatrixXd controls(3, 56);
  for (int i = 0; i < controls.cols(); ++i)
    controls.col(i) = Eigen::Vector3d{2.0 * i / 55.0, -0.4, 0.5};
  controls.col(16) = Eigen::Vector3d{0.538384, -0.221012, 0.5};
  controls.col(17) = Eigen::Vector3d{0.578548, -0.261720, 0.5};
  controls.col(18) = Eigen::Vector3d{0.618674, -0.302492, 0.5};
  for (int i = 0; i < controls.cols(); ++i)
    ASSERT_EQ(map->getInflateOccupancy(controls.col(i), 0.0), 0) << i;
  const auto guides = optimizer.initControlPoints(controls);
  EXPECT_TRUE(guides.empty());
}

TEST(ScanMotionIntentBoundary, UpstreamRejectsWhenPrefixHasNoCompleteDetour) {
  ScanAttemptFixture fixture(true);
  for (int x = 6; x <= 16; ++x)
    for (int y = -3; y <= 2; ++y)
      fixture.request.bitmap.occupy({x * 0.1 + 0.05, y * 0.1 + 0.05, 0.55});
  EXPECT_FALSE(fixture.replan());
  EXPECT_EQ(fixture.manager.reboundDebug().reason,"collision_restart_limit");
}

TEST(ScanAttemptDiagnostics, RefineReportsItsCollisionAndClearsItOnNextSuccess) {
  using namespace nav_kernel::local::scan::upstream;
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
  BsplineOptimizer optimizer;
  BsplineOptimizerParams params;
  params.lambda_smooth = 1.0;
  params.lambda_collision = 0.5;
  params.lambda_feasibility = 0.1;
  params.lambda_fitness = 1.0;
  params.dist0 = 0.2;
  params.max_vel = params.max_acc = 0.5;
  optimizer.setParam(params);
  optimizer.setEnvironment(std::make_shared<GridMap>(grid));
  Eigen::MatrixXd controls(3, 9);
  for (int i = 0; i < controls.cols(); ++i)
    controls.col(i) = Eigen::Vector3d{0.25 * (i - 1), 0.0, 0.5};
  const UniformBspline candidate(controls, 3, 0.5);
  const double sampleInterval = candidate.getTimeSum() / (controls.cols() - 3);
  for (double t = 0.0; t < candidate.getTimeSum() + 1e-4; t += sampleInterval)
    optimizer.ref_pts_.push_back(candidate.evaluateDeBoorT(t));
  for (double x = -0.25; x < 0.3; x += 0.1)
    for (double y = -0.25; y < 0.3; y += 0.1)
      for (double z = 0.25; z < 0.8; z += 0.1)
        fixture.bitmap.occupy({x, y, z});
  Eigen::MatrixXd output;
  ASSERT_FALSE(optimizer.BsplineOptimizeTrajRefine(controls, 0.5, output));
  const auto failed = optimizer.optimizationDebug();
  EXPECT_EQ(failed.reason, "collision_after_refine");
  EXPECT_TRUE(failed.optimizerReturnCodeValid);
  EXPECT_TRUE(failed.collisionValid);
  EXPECT_EQ(failed.collisionState, 1);
  EXPECT_GE(failed.collisionTimeS, 0.0);
  const UniformBspline rejected(output, 3, 0.5);
  EXPECT_LT((rejected.evaluateDeBoorT(failed.collisionTimeS) -
             failed.collisionPosition).norm(), 1e-10);
  fixture.bitmap.clear();
  ASSERT_TRUE(optimizer.BsplineOptimizeTrajRefine(controls, 0.5, output));
  const auto &ready = optimizer.optimizationDebug();
  EXPECT_EQ(ready.reason, "accepted");
  EXPECT_TRUE(ready.optimizerReturnCodeValid);
  EXPECT_FALSE(ready.collisionValid);
}

TEST(ScanTrajectoryCollision, UpstreamRefinementLeavesFinalThirdToExecutionChecks) {
  using namespace nav_kernel::local::scan::upstream;
  RequestFixture fixture({{0.0, 0.0, 0.525}, {4.0, 0.0, 0.525}});
  nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
  BsplineOptimizer optimizer;
  optimizer.setEnvironment(std::make_shared<GridMap>(grid));
  Eigen::MatrixXd controls(3, 43);
  for (int i = 0; i < controls.cols(); ++i)
    controls.col(i) = Eigen::Vector3d{0.1 * (i - 1), 0.0, 0.525};
  const UniformBspline candidate(controls, 3, 0.2);
  fixture.bitmap.occupy({3.55, 0.025, 0.525});
  ASSERT_EQ(grid.inflatedOccupancy({3.55, 0.0, 0.525}, 0.0), 1);
  EXPECT_TRUE(optimizer.checkTrajectoryCollisionFree(candidate));
  EXPECT_FALSE(optimizer.optimizationDebug().collisionValid);
  fixture.bitmap.clear();
  EXPECT_TRUE(optimizer.checkTrajectoryCollisionFree(candidate));
  EXPECT_FALSE(optimizer.optimizationDebug().collisionValid);
}

TEST(ScanMotionIntentBoundary, UpstreamOptimizerDoesNotCertifyFinalApproach) {
  ScanAttemptFixture fixture(true);
  for (int x = 19; x <= 23; ++x)
    for (int y = -3; y <= 2; ++y)
      for (int z = 2; z <= 8; ++z)
        fixture.request.bitmap.occupy({x * 0.1 + 0.05, y * 0.1 + 0.05, z * 0.1 + 0.05});
  ASSERT_EQ(fixture.grid.inflatedOccupancy({2.0, 0.0, 0.5}, 0.0), 1);
  EXPECT_TRUE(fixture.replan());
  EXPECT_TRUE(fixture.manager.reboundDebug().success);
  EXPECT_FALSE(fixture.manager.reboundDebug().collisionValid);
}

TEST(ScanBackend, ResetUsesUpdatedMap) {
  RequestFixture fixture({{0, 0, .5}, {2, 0, .5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  const auto tick = [&] {
    fixture.request.clock.timestampS += .01;
    return backend.tick(fixture.request);
  };
  LocalPlan plan;
  for (int i = 0; i < 100 && !plan.ready(); ++i) plan = tick();
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;

  for (double x = -.25; x < .3; x += .1)
    for (double y = -.25; y < .3; y += .1)
      for (double z = .25; z < .8; z += .1)
        fixture.bitmap.occupy({x, y, z});
  fixture.refreshCollision(2);
  backend.reset();
  for (int i = 0; i < 20; ++i) {
    plan = tick();
    EXPECT_FALSE(plan.ready());
  }
  ASSERT_TRUE(backend.debugSnapshot().lastScanFailure);
  EXPECT_EQ(backend.debugSnapshot().lastScanFailure->attempt.reason,
            "collision_at_trajectory_start");

  fixture.bitmap.clear();
  fixture.request.robot.pose.position.y = .3;
  fixture.refreshCollision(3);
  backend.reset();
  plan = tick();
  EXPECT_FALSE(plan.ready());
  for (int i = 0; i < 100 && !plan.ready(); ++i) plan = tick();
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
  EXPECT_NEAR(spline.position(0).x, 0.0, 1e-6);
  EXPECT_NEAR(spline.position(0).y, .3, 1e-6);
  EXPECT_NEAR(spline.position(0).z, .5, 1e-6);
}

TEST(ScanAttemptDiagnostics, BackendRetainsFirstFailedTickAndOwnsItsCompleteMapAcrossReset) {
  for (const bool shared : {true, false}) {
    SCOPED_TRACE(shared ? "shared" : "borrowed");
    RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
    const auto occupyStart = [&] {
      for (double x = -0.25; x < 0.3; x += 0.1)
        for (double y = -0.25; y < 0.3; y += 0.1)
          for (double z = 0.25; z < 0.8; z += 0.1)
            fixture.bitmap.occupy({x, y, z});
    };
    occupyStart();
    fixture.refreshCollision(2);
    auto &collision = fixture.request.environment.collision;
    std::shared_ptr<const std::vector<std::uint8_t>> originalStorage;
    if (shared) {
      originalStorage = std::make_shared<const std::vector<std::uint8_t>>(
          collision.inflatedBits, collision.inflatedBits + collision.inflatedBytes);
      collision.inflatedStorage = originalStorage;
      collision.inflatedBits = originalStorage->data();
    }
    nav_kernel::local::scan::Backend backend(scanParams());
    const auto tick = [&] {
      fixture.request.clock.timestampS += 0.01;
      return backend.tick(fixture.request);
    };
    for (int i = 0; i < 20 && !backend.debugSnapshot().lastScanFailure; ++i) tick();
    const auto first = backend.debugSnapshot().lastScanFailure;
    ASSERT_TRUE(first);
    EXPECT_DOUBLE_EQ(first->attempt.timestampS, fixture.request.clock.timestampS);
    EXPECT_EQ(first->sequence, 1U);
    EXPECT_FALSE(first->attempt.success);
    EXPECT_EQ(first->attempt.stage, "rebound_optimization");
    EXPECT_EQ(first->attempt.reason, "collision_at_trajectory_start");
    EXPECT_TRUE(first->checkObstacle);
    EXPECT_EQ(first->referenceGeneration, 1U);
    EXPECT_EQ(first->reference.size(), 2U);
    EXPECT_GT(first->candidateControlPoints.size(), 3U);
    ASSERT_TRUE(first->collision.inflatedStorage);
    EXPECT_EQ(first->collision.inflatedBytes, collision.inflatedBytes);
    EXPECT_EQ(first->collision.inflatedBits, first->collision.inflatedStorage->data());
    if (shared) {
      EXPECT_EQ(first->collision.inflatedStorage, originalStorage);
    } else {
      EXPECT_NE(first->collision.inflatedBits, collision.inflatedBits);
      fixture.bitmap.clear();
      nav_kernel::LocalPlanRequest capturedRequest;
      capturedRequest.environment.collision = first->collision;
      nav_kernel::local::scan::Grid capturedGrid(scanParams(), capturedRequest);
      EXPECT_EQ(capturedGrid.inflatedOccupancy({0.0, 0.0, 0.5}, 0.0), 1);
      occupyStart();
    }
    for (int i = 0; i < 4; ++i) tick();
    EXPECT_EQ(backend.debugSnapshot().lastScanFailure, first);
    const auto lastAttempt = backend.debugSnapshot().scanAttempt.attemptId;
    const double lastAttemptTime = backend.debugSnapshot().scanAttempt.timestampS;
    EXPECT_GE(lastAttempt, first->attempt.attemptId);
    fixture.request.clock.timestampS += 0.1;
    (void)backend.tick(fixture.request, [] { return true; });
    EXPECT_EQ(backend.debugSnapshot().lastScanFailure, first);
    EXPECT_EQ(backend.debugSnapshot().scanAttempt.attemptId, lastAttempt);
    EXPECT_DOUBLE_EQ(backend.debugSnapshot().scanAttempt.timestampS, lastAttemptTime);
    backend.reset();
    EXPECT_EQ(backend.debugSnapshot().lastScanFailure, first);
    tick();
    EXPECT_EQ(backend.debugSnapshot().lastScanFailure, first);
    for (int i = 0; i < 20 && backend.debugSnapshot().lastScanFailure == first; ++i) tick();
    const auto second = backend.debugSnapshot().lastScanFailure;
    ASSERT_TRUE(second);
    EXPECT_NE(second, first);
    EXPECT_EQ(second->sequence, 2U);
    EXPECT_GT(second->attempt.timestampS, lastAttemptTime);
  }
}

TEST(ScanAttemptDiagnostics, BackendSuccessfulAttemptRearmsFailureCapture) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  auto params = scanParams();
  fixture.request.robot.kinematics.valid = true;
  fixture.request.robot.kinematics.linearVelocity = {2.0, 0.0, 0.0};
  nav_kernel::local::scan::Backend backend(params);
  const auto tick = [&] {
    fixture.request.clock.timestampS += 0.01;
    return backend.tick(fixture.request);
  };
  for (int i = 0; i < 20 && !backend.debugSnapshot().lastScanFailure; ++i) tick();
  const auto first = backend.debugSnapshot().lastScanFailure;
  ASSERT_TRUE(first);
  EXPECT_EQ(first->attempt.stage, "dynamic_feasibility");
  fixture.request.robot.kinematics.linearVelocity = {};
  LocalPlan ready;
  for (int i = 0; i < 30 && !ready.ready(); ++i) ready = tick();
  ASSERT_TRUE(ready.ready()) << backend.debugSnapshot().searchReason;
  EXPECT_TRUE(backend.debugSnapshot().scanAttempt.success);
  EXPECT_FALSE(backend.debugSnapshot().scanAttempt.collisionValid);
  EXPECT_FALSE(backend.debugSnapshot().scanAttempt.dynamicViolationValid);
  EXPECT_EQ(backend.debugSnapshot().lastScanFailure, first);

  // A supported speed reduction restarts from measured motion in the same manager.
  fixture.request.maxLinearSpeedMps = 0.3;
  fixture.request.robot.kinematics.linearVelocity = {2.0, 0.0, 0.0};
  for (int i = 0; i < 30 && backend.debugSnapshot().lastScanFailure == first; ++i) tick();
  const auto second = backend.debugSnapshot().lastScanFailure;
  ASSERT_TRUE(second);
  EXPECT_NE(second, first);
  EXPECT_EQ(second->sequence, first->sequence + 1U);
  EXPECT_EQ(second->attempt.stage, "dynamic_feasibility");
}

TEST(ScanAttemptDiagnostics, FacadeRetainsCompletedFailureAcrossResetAndPending) {
  for (const bool consumeCompletion : {false, true}) {
    SCOPED_TRACE(consumeCompletion ? "completion polled" : "completion not polled");
    RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
    for (double x = -0.25; x < 0.3; x += 0.1)
      for (double y = -0.25; y < 0.3; y += 0.1)
        for (double z = 0.25; z < 0.8; z += 0.1)
          fixture.bitmap.occupy({x, y, z});
    fixture.refreshCollision(2);
    nav_kernel::LocalPlanner planner(scanParams());
    ASSERT_TRUE(planner.configure(""));
    ASSERT_EQ(planner.plan(fixture.request).status(), nav_kernel::LocalPlanStatus::Pending);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!planner.debugSnapshot().lastScanFailure &&
           std::chrono::steady_clock::now() < deadline) {
      if (consumeCompletion)
        (void)planner.plan(fixture.request);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    const auto failed = planner.debugSnapshot().lastScanFailure;
    ASSERT_TRUE(failed);
    if (consumeCompletion) {
      (void)planner.plan(fixture.request);
      EXPECT_TRUE(planner.debugSnapshot().scanAttempt.attempted);
    } else {
      // The completed worker attempt is visible before the normal plan poll.
      EXPECT_FALSE(planner.debugSnapshot().scanAttempt.attempted);
    }
    planner.reset();
    EXPECT_EQ(planner.debugSnapshot().lastScanFailure, failed);
    fixture.bitmap.clear();
    fixture.refreshCollision(3);
    const LocalPlan pending = planner.plan(fixture.request);
    EXPECT_EQ(pending.status(), nav_kernel::LocalPlanStatus::Pending);
    EXPECT_FALSE(pending.ready());
    EXPECT_EQ(planner.debugSnapshot().lastScanFailure, failed);
  }
}

TEST(ScanAttemptDiagnostics, CancelledAttemptIsNotCapturedByNextCollisionTick) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  for (double x = -0.25; x < 0.3; x += 0.1)
    for (double y = -0.25; y < 0.3; y += 0.1)
      for (double z = 0.25; z < 0.8; z += 0.1)
        fixture.bitmap.occupy({x, y, z});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Backend backend(scanParams());
  for (int i = 0; i < 10 &&
       backend.debugSnapshot().searchReason != "scan_generate_trajectory"; ++i) {
    fixture.request.clock.timestampS += 0.01;
    (void)backend.tick(fixture.request);
  }
  ASSERT_EQ(backend.debugSnapshot().searchReason, "scan_generate_trajectory");
  ASSERT_FALSE(backend.debugSnapshot().scanAttempt.attempted);
  int cancelChecks = 0;
  const LocalPlan cancelled = backend.tick(fixture.request, [&] {
    return ++cancelChecks == 2;
  });
  ASSERT_EQ(cancelChecks, 2);
  EXPECT_EQ(cancelled.status(), nav_kernel::LocalPlanStatus::Cancelled);
  EXPECT_FALSE(backend.debugSnapshot().scanAttempt.attempted);
  EXPECT_FALSE(backend.debugSnapshot().lastScanFailure);
  fixture.request.clock.timestampS += 0.01;
  (void)backend.checkCollision(fixture.request);
  EXPECT_FALSE(backend.debugSnapshot().scanAttempt.attempted);
  EXPECT_FALSE(backend.debugSnapshot().lastScanFailure);
  fixture.request.clock.timestampS += 0.01;
  (void)backend.tick(fixture.request);
  const auto accepted = backend.debugSnapshot().lastScanFailure;
  ASSERT_TRUE(accepted);
  EXPECT_EQ(accepted->sequence, 1U);
  EXPECT_GE(accepted->attempt.attemptId, 2U);
  EXPECT_EQ(accepted->attempt.reason, "collision_at_trajectory_start");
}

TEST(ScanTrajectoryValidation, SamplesFullTrajectoriesAcrossRoutesAndObstacles) {
  const std::vector<std::vector<Vec3>> routes{
      {{0, 0, 0.5}, {2, 0, 0.5}},
      {{0, 0, 0.5}, {0, 2, 0.5}},
      {{0, 0, 0.5}, {-2, -1, 0.5}},
      {{0, 0, 0.5}, {1, 0.75, 0.5}, {2, 0, 0.5}},
      {{0, 0, 0.5}, {1, 0, 0.5}, {2.5, 0, 0.5}},
  };
  for (std::size_t scene = 0; scene < routes.size(); ++scene) {
    SCOPED_TRACE(scene);
    RequestFixture fixture(routes[scene], 0.05);
    LocalPlannerParams params;
    params.backend = LocalPlannerBackend::Scan;
    params.scan.maxVelocity = 0.75;
    params.scan.maxAcceleration = 0.50;
    if (scene == 4) {
      // Already inflated Mapd cells: the core must detour, not inflate again.
      for (int x = 16; x <= 24; ++x)
        for (int y = -8; y <= 8; ++y)
          for (int z = -20; z < 40; ++z)
            fixture.bitmap.occupy({0.05 * x + 0.025, 0.05 * y + 0.025, 0.05 * z + 0.025});
    }
    fixture.refreshCollision(2);
    nav_kernel::local::scan::Backend backend(params);
    LocalPlan plan;
    for (int tick = 0; tick < 50 && !plan.ready(); ++tick) {
      fixture.request.clock.timestampS = 1.0 + tick * 0.01;
      plan = backend.tick(fixture.request);
    }
    ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
    nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
    ASSERT_TRUE(spline.valid());
    ASSERT_GT(spline.duration(), 0.0);
    nav_kernel::local::scan::Grid grid(params, fixture.request);
    ASSERT_TRUE(grid.valid()) << grid.reason();
    const auto start = spline.position(0.0);
    EXPECT_NEAR(start.x, fixture.route.front().x, 0.02);
    EXPECT_NEAR(start.y, fixture.route.front().y, 0.02);
    const auto end = spline.position(spline.duration());
    EXPECT_NEAR(end.x, fixture.route.back().x, 0.15);
    EXPECT_NEAR(end.y, fixture.route.back().y, 0.15);
    const int samples = static_cast<int>(std::ceil(spline.duration() / 0.002));
    int occupiedSamples = 0;
    double maxVelocity = 0.0;
    double maxAcceleration = 0.0;
    for (int i = 0; i <= samples; ++i) {
      const double t = spline.duration() * i / samples;
      const auto position = spline.position(t);
      const auto velocity = spline.velocity(t);
      ASSERT_TRUE(std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z));
      ASSERT_TRUE(std::isfinite(velocity.x) && std::isfinite(velocity.y) && std::isfinite(velocity.z));
      const double yaw = std::atan2(velocity.y, velocity.x);
      if (!grid.obstacleFree(position, yaw)) ++occupiedSamples;
      maxVelocity = std::max(maxVelocity, std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z));
      // Finite differences audit the emitted trajectory independently of the optimizer's feasibility check.
      constexpr double delta = 0.0001;
      if (t >= delta && t <= spline.duration() - delta) {
        const auto before = spline.velocity(t - delta);
        const auto after = spline.velocity(t + delta);
        const double ax = (after.x - before.x) / (2 * delta);
        const double ay = (after.y - before.y) / (2 * delta);
        const double az = (after.z - before.z) / (2 * delta);
        maxAcceleration = std::max(maxAcceleration, std::sqrt(ax * ax + ay * ay + az * az));
      }
    }
    EXPECT_EQ(occupiedSamples, 0);
    EXPECT_LE(maxVelocity, params.scan.maxVelocity + params.scan.velocityTolerance + 0.001);
    EXPECT_LE(maxAcceleration, params.scan.maxAcceleration + params.scan.accelerationTolerance + 0.001);
    std::cout << "SCAN scene=" << scene << " samples=" << samples + 1
              << " max_velocity=" << maxVelocity << " max_acceleration=" << maxAcceleration
              << " occupied_samples=" << occupiedSamples << '\n';
  }
}

TEST(ScanDefaults, UsesOfficialGridAndSearchScale) {
  const nav_kernel::ScanPlannerParams params;
  EXPECT_DOUBLE_EQ(params.voxelResolution, 0.05);
  EXPECT_DOUBLE_EQ(params.cylinderOffset, 0.18);
  EXPECT_DOUBLE_EQ(nav_kernel::local::scan::Backend::fsmPeriodS(), 0.01);
  EXPECT_DOUBLE_EQ(nav_kernel::local::scan::Backend::collisionPeriodS(), 0.05);
}

TEST(ScanBackend, FailedInitializationReportsBlockedAndRecoversAfterMapChange) {
  RequestFixture fixture({{0.025, 0.025, 0.525}, {2.025, 0.025, 0.525}}, 0.05);
  for (int x = -3; x <= 3; ++x)
    for (int y = -3; y <= 3; ++y)
      fixture.bitmap.occupy({x * 0.05 + 0.025, y * 0.05 + 0.025, 0.525});
  fixture.refreshCollision(2);
  auto params = scanParams();
  params.scan.voxelResolution = 0.05;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 8; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    if (plan.status() == nav_kernel::LocalPlanStatus::Blocked) break;
  }
  ASSERT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Blocked);
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_initialization_failed");
  EXPECT_EQ(backend.checkCollision(fixture.request).status(), nav_kernel::LocalPlanStatus::Blocked);
  fixture.bitmap = CollisionBitmap({-5.0, -5.0, -1.0}, {5.0, 5.0, 2.0}, 0.05);
  fixture.refreshCollision(3);
  for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  EXPECT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
}

TEST(ScanBackend, BlockedForwardTargetsDoNotRemainPendingAndRecoverWhenClear) {
  // The distant goal is outside the rolling map, and every forward reference
  // target inside the map is occupied.
  RequestFixture fixture({{0, 0, 0.5}, {6, 0, 0.5}}, 0.05);
  for (int x = 0; x < 100; ++x)
    fixture.bitmap.occupy({x * 0.05 + 0.025, 0.025, 0.525});
  fixture.refreshCollision(2);
  auto params = scanParams();
  params.scan.voxelResolution = 0.05;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 20; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Blocked);
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_local_target_blocked");
  EXPECT_FALSE(plan.ready());
  fixture.bitmap.clear();
  fixture.refreshCollision(3);
  for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  EXPECT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
}

TEST(ScanBackend, ApproachesStopAtTheClosestClearGoalCells) {
  for (bool moving : {true, false}) {
    SCOPED_TRACE(moving);
    RequestFixture fixture({{1.5, 0, .44}, {2.6, 0, .435}}, .05);
    for (int x = 55; x < 80; ++x)
      for (int y = -8; y <= 8; ++y)
        for (int z = 4; z < 16; ++z)
          fixture.bitmap.occupy({(x+.5)*.05, (y+.5)*.05, (z+.5)*.05});
    fixture.refreshCollision(2);
    fixture.request.robot.pose.position = moving ? Vec3{2.2881, .00127, .44185}
                                                : Vec3{2.4202, .0091, .44866};
    fixture.request.robot.kinematics.valid = true;
    fixture.request.robot.kinematics.linearVelocity = moving ? Vec3{.61853, .00306, .09104}
                                                            : Vec3{};
    fixture.request.maxLinearSpeedMps = 1.0;
    auto params = scanParams();
    params.scan.voxelResolution = .05;
    params.scan.cylinderOffset = .25;
    params.scan.maxVelocity = std::sqrt(2.0);
    params.scan.maxAcceleration = 4.0;
    params.scan.accelerationTolerance = 1.2;
    nav_kernel::local::scan::Backend backend(params);
    LocalPlan plan;
    for (int i = 0; i < 20 && !plan.ready(); ++i) {
      fixture.request.clock.timestampS += .01;
      plan = backend.tick(fixture.request);
    }
    ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().scanAttempt.reason;
    const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
    EXPECT_LT(nav_kernel::distance3D(spline.position(0), fixture.request.robot.pose.position), 1e-8);
    EXPECT_LT(nav_kernel::distance3D(spline.velocity(0),
        fixture.request.robot.kinematics.linearVelocity), 1e-8);
    EXPECT_LT(nav_kernel::distance3D(spline.position(spline.duration()), fixture.route.back()), .15);
    EXPECT_LT(nav_kernel::distance3D(spline.velocity(spline.duration()), {}), 1e-8);
    const nav_kernel::local::scan::Grid grid(params, fixture.request);
    EXPECT_EQ(grid.cylinderOccupancy(spline.position(spline.duration()), 0), 0);
    for (double t = 0; t + .001 <= spline.duration(); t += .001)
      EXPECT_LE(nav_kernel::distance3D(spline.velocity(t + .001), spline.velocity(t)) / .001, 5.2);
  }
}

TEST(ScanBackend, StoppedBesideBoxDoesNotRejoinTheBlockedStraightReference) {
  RequestFixture fixture({{1.5, 0, .435}, {4.5, 0, .435}, {5.1, 0, .435}}, .05);
  fixture.bitmap = CollisionBitmap({-1, -2, -1}, {7, 2, 2}, .05);
  for (int x = 0; x < 140; ++x)
    for (int y = -40; y < 40; ++y)
      if (std::abs((y+.5)*.05) >= 1.05 ||
          (x >= 56 && x < 76 && std::abs((y+.5)*.05) < .6))
        fixture.bitmap.occupy({(x+.5)*.05, (y+.5)*.05, .435});
  fixture.refreshCollision(2);
  fixture.request.robot.pose = {{3.4132, -.8704, .435}, -.2132};
  fixture.request.robot.kinematics.valid = true;
  fixture.request.maxLinearSpeedMps = 1;
  auto params = scanParams();
  params.scan.voxelResolution = .05;
  params.scan.cylinderOffset = .25;
  params.scan.maxVelocity = std::sqrt(2.0);
  params.scan.maxAcceleration = 4;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int i = 0; i < 30 && !plan.ready(); ++i) {
    fixture.request.clock.timestampS += .01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().scanAttempt.reason;
  const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
  const nav_kernel::local::scan::Grid grid(params, fixture.request);
  const auto start = fixture.request.robot.pose.position, end = fixture.route.back();
  const double yaw = std::atan2(end.y-start.y, end.x-start.x);
  double deviation = 0;
  for (double t = 0; t < spline.duration(); t += .005) {
    const auto position = spline.position(t), next = spline.position(t + .005);
    ASSERT_EQ(grid.cylinderOccupancy(position,
        std::atan2(next.y-position.y, next.x-position.x)), 0) << t;
    const double fraction = t / spline.duration();
    ASSERT_EQ(grid.cylinderOccupancy({start.x + fraction*(end.x-start.x),
        start.y + fraction*(end.y-start.y), start.z}, yaw), 0);
    deviation = std::max(deviation, std::abs((position.x-start.x)*std::sin(yaw) -
        (position.y-start.y)*std::cos(yaw)));
  }
  EXPECT_LT(deviation, .05);
  EXPECT_LT(nav_kernel::distance3D(spline.position(spline.duration()), fixture.route.back()), .15);
}

TEST(ScanBackend, SettledRestartPassesTheExecutionCollisionSampling) {
  // Recorded MID360 occupancy around the stair post, cropped without reinflation.
  RequestFixture fixture({{7.003047734271758,-0.7974033732901403,0.441843961866934},
      {7.002899398133565,-0.7973314393409831,0.441843961866934},
      {4.68550065731679,1.107836027329815,0.4595357234541344},
      {1.35,3.85,0.485},
      {1.359153810984217,3.929773517950581,0.485},
      {1.369512784922856,4.009078361418888,0.485},
      {1.381579444972146,4.087719104733054,0.485},
      {1.395246196450511,4.165737590269246,0.485},
      {1.40970389026566,4.24344848711891,0.485},
      {1.426477670881355,4.320258683546141,0.485},
      {1.457123137501822,4.391674335415958,0.485},
      {1.503309292322005,4.45704638631922,0.485},
      {1.552178702802601,4.5213749489101,0.485},
      {1.601079920371524,4.58569114207774,0.485},
      {1.65,4.65,0.485},
      {2.85,4.65,0.885},
      {3.75,4.65,1.285},
      {5.65,4.65,1.685},
      {6.65,4.65,2.085},
      {6.850000000000001,4.65,2.285},
      {6.944247358366724,4.641371037550083,2.285},
      {7.037833755824101,4.631750633736149,2.285},
      {7.130480230801153,4.620720346201725,2.285},
      {7.222170688112456,4.608256032168681,2.285},
      {7.313140999523471,4.594711499285205,2.285},
      {7.403798512239467,4.580697768359198,2.285},
      {7.481943023504682,4.54791453525702,2.285},
      {7.551613039881721,4.502419559822579,2.285},
      {7.617758558224743,4.451637837337113,2.285},
      {7.68388495049556,4.400827425743341,2.285},
      {7.75,4.350000000000001,2.285},
      {7.4,1.55,2.235}}, .05);
  const auto path = std::filesystem::path(__FILE__).parent_path() /
      "fixtures/scan_stair_post.bits";
  std::ifstream file(path, std::ios::binary);
  ASSERT_TRUE(file.is_open());
  const std::vector<std::uint8_t> bits((std::istreambuf_iterator<char>(file)), {});
  ASSERT_EQ(bits.size(), 15000U);
  auto &map = fixture.request.environment.collision;
  map.inflatedBits = bits.data();
  map.inflatedBytes = bits.size();
  map.sizeX = 80; map.sizeY = 50; map.sizeZ = 30;
  map.resolution = .05;
  map.aabbMin = {.5, 2.75, .25};
  map.aabbMax = {4.5, 5.25, 1.75};
  fixture.request.robot.pose = {{1.9792515135284106,3.4447798822858062,.44640039107286},2.418857814053};
  fixture.request.robot.kinematics.valid = true;
  fixture.request.robot.kinematics.linearVelocity = {.007881182897,.008279941134,-.013874996858};
  fixture.request.maxLinearSpeedMps = 1;
  fixture.request.clock.executionFrozen = true;
  auto params = scanParams();
  params.scan.voxelResolution = .05;
  params.scan.cylinderOffset = .25;
  params.scan.cylinderRadius = .390512483795333;
  params.scan.maxVelocity = std::sqrt(2.0);
  params.scan.maxAcceleration = 4;
  params.scan.accelerationTolerance = 1.2;
  params.scan.velocityTolerance = 1;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int i = 0; i < 8 && !plan.ready(); ++i) {
    fixture.request.clock.timestampS += .01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().scanAttempt.reason;
  const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
  const nav_kernel::local::scan::Grid grid(params, fixture.request);
  EXPECT_LT(nav_kernel::distance3D(spline.position(0),fixture.request.robot.pose.position), .005);
  for (double t = 0; t < spline.duration()*2/3; t += .01) {
    const auto position=spline.position(t), next=spline.position(t+.01);
    EXPECT_EQ(grid.cylinderOccupancy(position,
        std::atan2(next.y-position.y,next.x-position.x)),0) << t;
  }
}

TEST(ScanTrajectoryValidation, SmallNorthboundSamplesDoNotBecomeEastFacing) {
  using namespace nav_kernel::local::scan::upstream;
  RequestFixture fixture({{0,0,.5},{0,1,.5}}, .05);
  fixture.request.robot.pose.yaw = std::acos(-1.) / 2;
  fixture.bitmap.occupy({.225,.025,.525});
  fixture.refreshCollision(2);
  auto params = scanParams();
  params.scan.voxelResolution = .05;
  params.scan.cylinderOffset = .23;
  const nav_kernel::local::scan::Grid grid(params, fixture.request);
  ASSERT_EQ(grid.cylinderOccupancy({0,0,.5},0),1);
  ASSERT_EQ(grid.cylinderOccupancy({0,0,.5},std::acos(-1.)/2),0);
  BsplineOptimizer optimizer;
  optimizer.setEnvironment(std::make_shared<GridMap>(grid));
  Eigen::MatrixXd controls(3,7);
  controls.row(0).setZero();
  controls.row(1) << 0,0,0,.5,1,1,1;
  controls.row(2).setConstant(.5);
  // Slower timing changes displacement per sample, not the geometric heading.
  for (double interval : {.2, 1.0}) {
    const UniformBspline spline(controls,3,interval);
    EXPECT_TRUE(optimizer.checkTrajectoryCollisionFree(spline))
        << interval << ':' << optimizer.optimizationDebug().reason;
  }
}

TEST(ScanTrajectoryValidation, QuietMotionKeepsMeasuredHeadingButMovingTurnIsChecked) {
  using namespace nav_kernel::local::scan::upstream;
  RequestFixture fixture({{0,0,.5},{1,0,.5}}, .05);
  fixture.bitmap.occupy({.025,.225,.525});
  fixture.refreshCollision(2);
  auto params = scanParams();
  params.scan.voxelResolution = .05;
  params.scan.cylinderOffset = .23;
  const nav_kernel::local::scan::Grid grid(params, fixture.request);
  ASSERT_EQ(grid.cylinderOccupancy({0,0,.5},0),0);
  ASSERT_EQ(grid.cylinderOccupancy({0,0,.5},std::acos(-1.)/2),1);
  BsplineOptimizer optimizer;
  optimizer.setEnvironment(std::make_shared<GridMap>(grid));
  Eigen::MatrixXd controls(3,7);
  controls.row(0).setZero();
  controls.row(2).setConstant(.5);
  for (double step : {.0001, .004}) {
    controls.row(1) << 0,0,0,step,2*step,3*step,4*step;
    EXPECT_TRUE(optimizer.checkTrajectoryCollisionFree(UniformBspline(controls,3,.2)));
  }
  controls.row(1) << 0,0,0,.5,1,1.5,2;
  EXPECT_FALSE(optimizer.checkTrajectoryCollisionFree(UniformBspline(controls,3,.2)));
  EXPECT_GT(optimizer.optimizationDebug().collisionTimeS, 0);
}

TEST(ScanTrajectoryValidation, FutureMovingBoundaryUsesItsVelocityHeading) {
  using namespace nav_kernel::local::scan::upstream;
  RequestFixture fixture({{-1,0,.5},{0,1,.5}}, .05);
  fixture.bitmap.occupy({.225,.025,.525});
  fixture.refreshCollision(2);
  auto params = scanParams();
  params.scan.voxelResolution = .05;
  params.scan.cylinderOffset = .23;
  const nav_kernel::local::scan::Grid grid(params, fixture.request);
  const GridMap map(grid);
  ASSERT_EQ(grid.cylinderOccupancy({0,0,.5},0),1);
  EXPECT_EQ(map.getTrajectoryOccupancy({0,0,.5},{0,.2,0},
                                     {0,.002,.5},{0,.2,0},0,.01),0);
}

TEST(ScanBackend, RestToRestCornerKeepsTheClearCurveWithinAccelerationLimits) {
  const auto directory = std::filesystem::path(__FILE__).parent_path() / "fixtures";
  std::ifstream routeFile(directory / "scan_rest_corner.route");
  ASSERT_TRUE(routeFile.is_open());
  std::vector<Vec3> route;
  Vec3 point;
  while (routeFile >> point.x >> point.y >> point.z) route.push_back(point);
  ASSERT_EQ(route.size(), 58U);
  RequestFixture fixture(route, .05);
  std::ifstream file(directory / "scan_rest_corner.bits", std::ios::binary);
  ASSERT_TRUE(file.is_open());
  const std::vector<std::uint8_t> bits((std::istreambuf_iterator<char>(file)), {});
  ASSERT_EQ(bits.size(), 18750U);
  auto &map = fixture.request.environment.collision;
  map.inflatedBits = bits.data();
  map.inflatedBytes = bits.size();
  map.sizeX = 100; map.sizeY = 50; map.sizeZ = 30;
  map.resolution = .05;
  map.aabbMin = {.5, 2.75, .25};
  map.aabbMax = {5.5, 5.25, 1.75};
  fixture.request.robot.pose = {{2.3879,3.37746,.428},2.4};
  fixture.request.robot.kinematics.valid = true;
  fixture.request.maxLinearSpeedMps = 1;
  fixture.request.clock.executionFrozen = true;
  auto params = scanParams();
  params.scan.voxelResolution = .05;
  params.scan.cylinderOffset = .23;
  params.scan.cylinderRadius = .32;
  params.scan.maxVelocity = std::sqrt(2.0);
  params.scan.maxAcceleration = 4;
  params.scan.accelerationTolerance = 1.2;
  params.scan.velocityTolerance = 1;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int i = 0; i < 4; ++i) {
    fixture.request.clock.timestampS += .01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().scanAttempt.reason;
  EXPECT_EQ(backend.debugSnapshot().scanAttempt.attemptId, 1U);
  const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
  EXPECT_LT(nav_kernel::distance3D(spline.position(0), fixture.request.robot.pose.position), 1e-8);
  EXPECT_LT(nav_kernel::distance3D(spline.velocity(0), {}), 1e-8);
  EXPECT_LT(nav_kernel::distance3D(spline.velocity(spline.duration()), {}), 1e-8);
  const nav_kernel::local::scan::Grid grid(params, fixture.request);
  for (double t = 0; t < spline.duration(); t += .01) {
    EXPECT_LE(nav_kernel::distance3D(spline.velocity(t), spline.velocity(t+.001))/.001,
              params.scan.maxAcceleration + 1e-6);
    if (t < spline.duration()*2/3) {
      const auto position=spline.position(t), next=spline.position(t+.01);
      EXPECT_EQ(grid.cylinderOccupancy(position,
          std::atan2(next.y-position.y,next.x-position.x)),0) << t;
    }
  }
}

TEST(ScanBackend, ThousandBlockedRoundsStopAndWaitForNewReference) {
  using nav_kernel::LocalPlanStatus;
  RequestFixture fixture({{0, 0, 0.5}, {6, 0, 0.5}}, 0.05);
  for (int x = 0; x < 100; ++x)
    fixture.bitmap.occupy({x * 0.05 + 0.025, 0.025, 0.525});
  fixture.refreshCollision(2);
  auto params = scanParams();
  params.scan.voxelResolution = 0.05;
  nav_kernel::local::scan::Backend backend(params);
  const auto tick = [&] {
    fixture.request.clock.timestampS += 0.01;
    return backend.tick(fixture.request);
  };
  LocalPlan plan;
  for (int i = 0; i < 5 && plan.status() != LocalPlanStatus::Blocked; ++i)
    plan = tick();
  ASSERT_EQ(plan.status(), LocalPlanStatus::Blocked);
  ASSERT_EQ(backend.debugSnapshot().searchReason, "scan_local_target_blocked");
  for (int round = 2; round < 1000; ++round)
    ASSERT_EQ(tick().status(), LocalPlanStatus::Blocked) << round;
  EXPECT_EQ(tick().status(), LocalPlanStatus::NearFieldStop);
  EXPECT_EQ(tick().status(), LocalPlanStatus::NearFieldStop);
  EXPECT_EQ(tick().status(), LocalPlanStatus::NoPath);
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_replan_failed");

  fixture.bitmap.clear();
  fixture.refreshCollision(3);
  for (int i = 0; i < 5; ++i)
    EXPECT_EQ(tick().status(), LocalPlanStatus::NoPath);
  fixture.request.objective = RouteTarget{{fixture.route.data(), 2, 2}};
  for (int i = 0; i < 10 && !plan.ready(); ++i) plan = tick();
  EXPECT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
}

TEST(ScanBackend, MotionIntentFindsEitherOpenSideWhenReferenceTargetOrConnectionIsBlocked) {
  for (const int wallEnd : {18, 45}) {
    SCOPED_TRACE(wallEnd);
    for (const double openSide : {-1.0, 1.0}) {
      SCOPED_TRACE(openSide);
      RequestFixture fixture({{0.025, 0.025, 0.525}, {3.525, 0.025, 0.525}});
      for (int x = 1; x < wallEnd; ++x)
        for (int y = -2; y <= 2; ++y)
          for (int z = 1; z <= 9; ++z)
            fixture.bitmap.occupy({x * 0.1 + 0.025, y * 0.1 + 0.025, z * 0.1 + 0.025});
      for (int x = -45; x < 45; ++x)
        for (int y = 1; y < 45; ++y)
          for (int z = 1; z <= 9; ++z)
            fixture.bitmap.occupy({x * 0.1 + 0.025, -openSide * y * 0.1 + 0.025,
                                  z * 0.1 + 0.025});
      fixture.refreshCollision(2);
      fixture.request.objective = nav_kernel::MotionIntentTarget{
          {0.0, 1.0, 3.5, 90.0}, {fixture.route.data(), 2, 1}};
      fixture.request.maxLinearSpeedMps = 0.5;
      const auto params = scanParams();
      const nav_kernel::local::scan::Grid grid(params, fixture.request);
      ASSERT_EQ(grid.inflatedOccupancy(fixture.route.back(), 0.0), wallEnd == 45 ? 1 : 0);
      ASSERT_EQ(grid.segmentInflatedOccupancy(fixture.route.front(), 0.0,
                                            fixture.route.back(), 0.0), 1);
      nav_kernel::local::scan::Backend backend(params);
      LocalPlan plan;
      for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
        fixture.request.clock.timestampS += 0.01;
        plan = backend.tick(fixture.request);
      }
      ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
      const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
      const auto endpoint = spline.position(spline.duration());
      ASSERT_GT(openSide * (endpoint.y - 0.025), 0.5);
      EXPECT_GE(endpoint.x, 0.025 - 1e-8);
      const auto terminalVelocity = spline.velocity(spline.duration());
      EXPECT_NEAR(terminalVelocity.x, 0.0, 1e-6);
      EXPECT_NEAR(terminalVelocity.y, 0.0, 1e-6);
      EXPECT_NEAR(terminalVelocity.z, 0.0, 1e-6);
      for (double time = 0.0; time < spline.duration(); time += 0.01) {
        const auto position = spline.position(time);
        const auto next = spline.position(std::min(time + 0.01, spline.duration()));
        ASSERT_EQ(grid.segmentInflatedOccupancy(position, 0.0, next, 0.0), 0);
        const auto velocity = spline.velocity(time);
        const double nextTime = std::min(time + 0.01, spline.duration());
        const auto nextVelocity = spline.velocity(nextTime);
        EXPECT_LE(std::hypot(velocity.x, velocity.y), 0.5 + 1e-6);
        EXPECT_LE(std::hypot(nextVelocity.x - velocity.x, nextVelocity.y - velocity.y) /
                      (nextTime - time), 0.5 + 1e-6);
      }
    }
  }
}

TEST(ScanBackend, MotionIntentShortensBlockedConnectionWhenSideTargetsAreUnavailable) {
  RequestFixture fixture({{0.025, 0.025, 0.525}, {3.525, 0.025, 0.525}});
  for (int x = -49; x < 49; ++x)
    for (int z = -9; z < 19; ++z) {
      for (const double y : {-0.175, 0.225})
        fixture.bitmap.occupy({x * 0.1 + 0.025, y, z * 0.1 + 0.025});
    }
  for (int y = -2; y <= 2; ++y)
    for (int z = -9; z < 19; ++z)
      fixture.bitmap.occupy({0.825, y * 0.1 + 0.025, z * 0.1 + 0.025});
  fixture.refreshCollision(2);
  fixture.request.objective = nav_kernel::MotionIntentTarget{
      {0.0, 1.0, 3.5, 90.0}, {fixture.route.data(), 2, 1}};
  fixture.request.maxLinearSpeedMps = 0.5;
  const auto params = scanParams();
  const nav_kernel::local::scan::Grid grid(params, fixture.request);
  ASSERT_EQ(grid.inflatedOccupancy(fixture.route.back(), 0.0), 0);
  ASSERT_EQ(grid.segmentInflatedOccupancy(fixture.route.front(), 0.0,
                                        fixture.route.back(), 0.0), 1);
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
  const auto endpoint = spline.position(spline.duration());
  EXPECT_GT(endpoint.x, 0.4);
  EXPECT_LT(endpoint.x, 0.8);
  EXPECT_NEAR(endpoint.y, 0.025, 1e-6);
  for (double t = 0.0; t < spline.duration(); t += 0.01) {
    ASSERT_EQ(grid.segmentInflatedOccupancy(spline.position(t), 0.0,
        spline.position(std::min(t + 0.01, spline.duration())), 0.0), 0);
    const auto velocity = spline.velocity(t);
    EXPECT_LE(std::hypot(velocity.x, velocity.y), 0.5 + 1e-6);
  }
}

TEST(ScanBackend, MotionIntentPrefersAvailableRequestedDirectionOverLongSideDetour) {
  RequestFixture fixture({{0.025, 0.025, 0.525}, {0.025, 3.525, 0.525}}, 0.05);
  // Left has room for a shorter advance before the wall. Forward/backward
  // detours are longer, but should not override an immediately usable A input.
  for (int x = -99; x < 99; ++x)
    fixture.bitmap.occupy({x * 0.05 + 0.025, 1.025, 0.525});
  fixture.refreshCollision(2);
  fixture.request.objective = nav_kernel::MotionIntentTarget{
      {90.0, 1.0, 3.5, 90.0}, {fixture.route.data(), 2, 1}};
  fixture.request.maxLinearSpeedMps = 0.5;
  auto params = scanParams();
  params.scan.voxelResolution = 0.05;
  nav_kernel::local::scan::Grid grid(params, fixture.request);
  ASSERT_EQ(grid.segmentInflatedOccupancy(fixture.route.front(), 0.0,
                                        fixture.route.back(), 0.0), 1);
  ASSERT_EQ(grid.segmentInflatedOccupancy(fixture.route.front(), 0.0,
                                        {0.025, 0.900, 0.525}, 0.0), 0);
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
  const auto endpoint = spline.position(spline.duration());
  EXPECT_NEAR(endpoint.x, 0.025, 1e-6);
  EXPECT_GT(endpoint.y, 0.5);
  EXPECT_LT(endpoint.y, 1.025);
  for (double t = 0.0; t < spline.duration(); t += 0.01)
    ASSERT_EQ(grid.segmentInflatedOccupancy(spline.position(t), 0.0,
        spline.position(std::min(t + 0.01, spline.duration())), 0.0), 0);
}

TEST(ScanBackend, MotionIntentUsesShortSideTargetsWhenLongCandidatesAreBlocked) {
  for (const double distance : {0.4, 0.3, 0.2}) {
    SCOPED_TRACE(distance);
    for (const double openSide : {-1.0, 1.0}) {
      SCOPED_TRACE(openSide);
      for (const bool referenceBlocked : {false, true}) {
        SCOPED_TRACE(referenceBlocked);
        RequestFixture fixture({{0.025, 1.025, 0.525}, {3.525, 1.025, 0.525}}, 0.05);
        // A narrow lateral pocket has room for this short target, while the
        // forward connection and all longer side targets cross a wall.
        for (int x = -99; x < 99; ++x) {
          fixture.bitmap.occupy({x * 0.05 + 0.025, 1.025 - openSide * 0.05, 0.525});
          fixture.bitmap.occupy({x * 0.05 + 0.025,
                                1.025 + openSide * (distance + 0.05), 0.525});
        }
        for (int y = -99; y < 99; ++y)
          fixture.bitmap.occupy({0.075, y * 0.05 + 0.025, 0.525});
        if (referenceBlocked) {
          for (int x = 2; x < 75; ++x)
            fixture.bitmap.occupy({x * 0.05 + 0.025, 1.025, 0.525});
        }
        fixture.refreshCollision(2);
        fixture.request.objective = nav_kernel::MotionIntentTarget{
            {0.0, 1.0, 3.5, 90.0}, {fixture.route.data(), 2, 1}};
        fixture.request.maxLinearSpeedMps = 0.5;
        auto params = scanParams();
        params.scan.voxelResolution = 0.05;
        const nav_kernel::local::scan::Grid grid(params, fixture.request);
        ASSERT_EQ(grid.segmentInflatedOccupancy(fixture.route.front(), 0.0,
                                              fixture.route.back(), 0.0), 1);
        ASSERT_EQ(grid.segmentInflatedOccupancy(fixture.route.front(), 0.0,
            {0.025, 1.025 + openSide * 0.5, 0.525}, 0.0), 1);
        nav_kernel::local::scan::Backend backend(params);
        LocalPlan plan;
        bool referenceAttempted = false;
        for (int tick = 0; tick < 100 && !plan.ready(); ++tick) {
          fixture.request.clock.timestampS += 0.01;
          plan = backend.tick(fixture.request);
          const auto failure = backend.debugSnapshot().lastScanFailure;
          if (!referenceBlocked && !referenceAttempted && failure) {
            // Preserve the existing reference optimizer before adding short
            // targets; some blocked straight guides still yield a safe curve.
            EXPECT_GT(failure->targetPosition.x, 1.0);
            EXPECT_NEAR(failure->targetPosition.y, 1.025, 1e-6);
            referenceAttempted = true;
          }
        }
        EXPECT_EQ(referenceAttempted, !referenceBlocked);
        ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
        const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
        const auto endpoint = spline.position(spline.duration());
        EXPECT_NEAR(endpoint.x, 0.025, 1e-6);
        EXPECT_NEAR(openSide * (endpoint.y - 1.025), distance, 1e-6);
        for (double t = 0.0; t < spline.duration(); t += 0.01) {
          const double nextTime = std::min(t + 0.01, spline.duration());
          ASSERT_EQ(grid.segmentInflatedOccupancy(spline.position(t), 0.0,
              spline.position(nextTime), 0.0), 0);
          const auto velocity = spline.velocity(t);
          const auto nextVelocity = spline.velocity(nextTime);
          EXPECT_LE(std::hypot(velocity.x, velocity.y), 0.5 + 1e-6);
          EXPECT_LE(std::hypot(nextVelocity.x - velocity.x, nextVelocity.y - velocity.y) /
                        (nextTime - t), 0.5 + 1e-6);
        }
      }
    }
  }
}

TEST(ScanBackend, MotionIntentDoesNotDetourOutsideAllowedDirection) {
  RequestFixture fixture({{0.025, 0.025, 0.525}, {3.525, 0.025, 0.525}});
  for (int x = 1; x < 45; ++x)
    for (int y = -2; y <= 2; ++y)
      for (int z = 1; z <= 9; ++z)
        fixture.bitmap.occupy({x * 0.1 + 0.025, y * 0.1 + 0.025, z * 0.1 + 0.025});
  fixture.refreshCollision(2);
  fixture.request.objective = nav_kernel::MotionIntentTarget{
      {0.0, 1.0, 3.5, 0.0}, {fixture.route.data(), 2, 1}};
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 10; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Blocked);
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_local_target_blocked");
}

TEST(ScanReplanFsm, FailedMotionIntentDetourAdvancesOnNextTimerCallback) {
  using namespace nav_kernel::local::scan::upstream;
  RequestFixture fixture({{0.025, 0.025, 0.525}, {3.525, 0.025, 0.525}});
  for (int x = 1; x < 45; ++x)
    for (int y = -2; y <= 2; ++y)
      for (int z = 1; z <= 9; ++z)
        fixture.bitmap.occupy(
            {x * 0.1 + 0.025, y * 0.1 + 0.025, z * 0.1 + 0.025});
  fixture.refreshCollision(2);
  auto params = scanParams();
  params.scan.maxVelocity = 0.05;
  nav_kernel::local::scan::Grid grid(params, fixture.request);
  PlanParameters plan;
  plan.max_vel_ = 0.05;
  plan.max_acc_ = 0.5;
  plan.max_jerk_ = 4.0;
  plan.ctrl_pt_dist = 0.2;
  plan.planning_horizon_ = 3.5;
  plan.motion_intent_ = true;
  BsplineOptimizerParams optimizer;
  optimizer.lambda_smooth = 1.0;
  optimizer.lambda_collision = 1.0;
  optimizer.lambda_feasibility = 0.1;
  optimizer.lambda_fitness = 1.0;
  optimizer.dist0 = 0.2;
  optimizer.max_vel = plan.max_vel_;
  optimizer.max_acc = plan.max_acc_;
  SCANPlannerManager manager;
  manager.initPlanModules(plan, optimizer, std::make_shared<GridMap>(grid));
  ScanReplanParams fsmParams;
  fsmParams.navigationMode = ScanNavigationMode::REFERENCE_PATH;
  fsmParams.planningHorizon = 3.5;
  fsmParams.noReplanThreshold = 0.1;
  fsmParams.replanThreshold = 1.0;
  PlannerFsm fsm(manager, fsmParams);
  FsmInput input;
  input.nowS = 1.0;
  input.odometry = FsmOdometry{};
  input.odometry->position = {0.025, 0.025, 0.525};
  input.odometry->velocity = {0.1, 0.0, 0.0};
  input.motionIntentMaxDeviationRad = M_PI / 2.0;
  input.referencePath = std::vector<Eigen::Vector3d>{
      {0.025, 0.025, 0.525}, {3.525, 0.025, 0.525}};
  ASSERT_TRUE(fsm.tick(input).targetAccepted);
  input.referencePath.reset();
  input.nowS += 0.01;
  (void)fsm.tick(input);
  input.nowS += 0.01;
  (void)fsm.tick(input);

  input.nowS += 0.01;
  ASSERT_TRUE(fsm.tick(input).initializationFailed);
  const Eigen::Vector3d firstTarget = manager.reboundDebug().targetPosition;
  ASSERT_FALSE(manager.reboundDebug().success);
  EXPECT_TRUE(manager.reboundDebug().polyInit);
  EXPECT_FALSE(manager.reboundDebug().randomPolyInit);
  input.nowS += 0.01;
  ASSERT_TRUE(fsm.tick(input).initializationFailed);
  const Eigen::Vector3d retriedTarget = manager.reboundDebug().targetPosition;
  ASSERT_FALSE(manager.reboundDebug().success);
  EXPECT_TRUE(manager.reboundDebug().polyInit);
  EXPECT_TRUE(manager.reboundDebug().randomPolyInit);
  EXPECT_LE((retriedTarget - firstTarget).norm(), 1e-9);
  fixture.bitmap.occupyInflated(
      {static_cast<float>(firstTarget.x()), static_cast<float>(firstTarget.y()),
       static_cast<float>(firstTarget.z())},
      0.15, 0.3, 0.3);
  input.nowS += 0.01;
  ASSERT_TRUE(fsm.tick(input).initializationFailed);
  const Eigen::Vector3d secondTarget = manager.reboundDebug().targetPosition;
  ASSERT_FALSE(manager.reboundDebug().success);
  EXPECT_TRUE(manager.reboundDebug().polyInit);
  EXPECT_FALSE(manager.reboundDebug().randomPolyInit);
  EXPECT_GT((secondTarget - firstTarget).head<2>().norm(), 0.2);
  EXPECT_NEAR((secondTarget - input.odometry->position).head<2>().norm(),
              (firstTarget - input.odometry->position).head<2>().norm(), 1e-6);
}

TEST(ScanBackend, MotionIntentKeepsSideTargetThenRejoinsClearedReference) {
  RequestFixture fixture({{0.025, 0.025, 0.525}, {3.525, 0.025, 0.525}});
  for (int x = 1; x < 45; ++x)
    for (int y = -2; y <= 2; ++y)
      for (int z = 1; z <= 9; ++z)
        fixture.bitmap.occupy({x * 0.1 + 0.025, y * 0.1 + 0.025, z * 0.1 + 0.025});
  fixture.refreshCollision(2);
  fixture.request.objective = nav_kernel::MotionIntentTarget{
      {0.0, 1.0, 3.5, 90.0}, {fixture.route.data(), 2, 1}};
  fixture.request.maxLinearSpeedMps = 0.5;
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const auto first = std::get<SplineTarget>(plan.target());
  const nav_kernel::SplineView firstView(first);
  const auto firstTarget = firstView.position(firstView.duration());
  fixture.request.robot.pose.position = firstView.position(firstView.duration() * 0.5);
  fixture.request.robot.kinematics.valid = true;
  fixture.request.robot.kinematics.linearVelocity = firstView.velocity(firstView.duration() * 0.5);
  fixture.request.clock.timestampS = first.startTimeS + firstView.duration() * 0.5;
  for (int tick = 0; tick < 30; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    if (plan.ready() && std::get<SplineTarget>(plan.target()).trajectoryId > first.trajectoryId) break;
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const auto continued = std::get<SplineTarget>(plan.target());
  ASSERT_GT(continued.trajectoryId, first.trajectoryId);
  const nav_kernel::SplineView continuedView(continued);
  const auto continuedTarget = continuedView.position(continuedView.duration());
  EXPECT_NEAR(continuedTarget.x, firstTarget.x, 1e-6);
  EXPECT_NEAR(continuedTarget.y, firstTarget.y, 1e-6);

  fixture.bitmap.clear();
  fixture.refreshCollision(3);
  fixture.request.robot.kinematics.linearVelocity = {};
  fixture.request.clock.timestampS = continued.startTimeS + continuedView.duration() + 0.2;
  for (int tick = 0; tick < 30; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    if (plan.ready() && std::get<SplineTarget>(plan.target()).trajectoryId > continued.trajectoryId) break;
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const auto rejoined = std::get<SplineTarget>(plan.target());
  ASSERT_GT(rejoined.trajectoryId, continued.trajectoryId);
  const nav_kernel::SplineView rejoinedView(rejoined);
  const auto rejoinedTarget = rejoinedView.position(rejoinedView.duration());
  EXPECT_NEAR(rejoinedTarget.x, fixture.route.back().x, 0.02);
  EXPECT_NEAR(rejoinedTarget.y, fixture.route.back().y, 0.02);
}

TEST(ScanBackend, MotionIntentRejectsFreeSideEndpointsBehindAnEnclosure) {
  RequestFixture fixture({{0.025, 0.025, 0.525}, {3.525, 0.025, 0.525}});
  for (int x = 1; x < 45; ++x)
    for (int y = -2; y <= 2; ++y)
      for (int z = 1; z <= 9; ++z)
        fixture.bitmap.occupy({x * 0.1 + 0.025, y * 0.1 + 0.025, z * 0.1 + 0.025});
  for (int i = -2; i <= 2; ++i)
    for (int z = 1; z <= 9; ++z)
      for (const double side : {-1.0, 1.0}) {
        fixture.bitmap.occupy({i * 0.1 + 0.025, side * 0.2 + 0.025, z * 0.1 + 0.025});
        fixture.bitmap.occupy({side * 0.2 + 0.025, i * 0.1 + 0.025, z * 0.1 + 0.025});
      }
  fixture.refreshCollision(2);
  fixture.request.objective = nav_kernel::MotionIntentTarget{
      {0.0, 1.0, 3.5, 90.0}, {fixture.route.data(), 2, 1}};
  const auto params = scanParams();
  const nav_kernel::local::scan::Grid grid(params, fixture.request);
  ASSERT_EQ(grid.inflatedOccupancy({0.025, 2.025, 0.525}, 0.0), 0);
  ASSERT_EQ(grid.inflatedOccupancy({0.025, -1.975, 0.525}, 0.0), 0);
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 10; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Blocked);
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_local_target_blocked");
}

TEST(ScanBackend, FinalApproachBelowTwentyCentimetresProducesMotionTrajectory) {
  RequestFixture fixture({{0, 0, 0.5}, {0.18, 0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
  EXPECT_NEAR(spline.position(spline.duration()).x, 0.18, 0.02);
}

TEST(ScanBackend, BodyPoseSegmentsPreserveExactTurnCentreAndStoppedEndpoint) {
  namespace collision = lingtu::nav::collision;
  const std::vector<std::vector<Vec3>> segments{
      {{-1.1,0,.35},{-.25,.1,.35}},
      {{-.268,.0979,.35},{-.25,.1,.35}},
      {{0,0,.35},{.18,0,.35}}};
  for (const auto &segment : segments) {
    SCOPED_TRACE(::testing::Message() << "target=" << segment.back().x << ',' << segment.back().y);
    RequestFixture fixture(segment);
    fixture.request.bodyHeading = 0;
    auto body = std::make_shared<collision::Body>();
    collision::Part part;
    part.link = "base";
    part.half_size = {.15,.08,.08};
    body->parts.push_back(part);
    fixture.request.body = body;
    const auto &map = fixture.request.environment.collision;
    fixture.request.environment.collision.measuredOccupiedStorage =
        std::make_shared<const std::vector<std::uint8_t>>(map.inflatedBytes,0U);
    fixture.request.environment.collision.knownFreeStorage =
        std::make_shared<const std::vector<std::uint8_t>>(map.inflatedBytes,255U);
    auto params = scanParams();
    params.scan.noReplanDistance = .001;
    nav_kernel::local::scan::Backend backend(params);
    LocalPlan plan;
    for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
      fixture.request.clock.timestampS += .01;
      plan = backend.tick(fixture.request);
    }
    ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason << ':'
                             << backend.debugSnapshot().scanAttempt.reason;
    const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
    EXPECT_LE(nav_kernel::distance3D(spline.position(0),segment.front()),1e-8);
    EXPECT_LE(nav_kernel::distance3D(spline.position(spline.duration()),segment.back()),1e-8);
    EXPECT_LE(nav_kernel::distance3D(spline.velocity(spline.duration()),{}),1e-8);
    const nav_kernel::local::scan::Grid grid(params,fixture.request);
    for (double t = 0; t < spline.duration(); t += .01) {
      const double next = std::min(t + .01,spline.duration());
      EXPECT_EQ(grid.trajectoryOccupancy(spline.position(t),spline.velocity(t),
          spline.position(next),spline.velocity(next),t,next),0);
    }
  }
}

TEST(ScanBackend, BodyMetadataDoesNotChangeOrdinaryRouteFit) {
  namespace collision=lingtu::nav::collision;
  RequestFixture fixture({{0,0,.5},{1.1,0,.5}});
  auto body=std::make_shared<collision::Body>();body->margin_m=0;
  collision::Part part;part.half_size={.2,.2,.1};body->parts.push_back(part);
  fixture.request.body=body;
  fixture.request.supportHeightM=.5;
  fixture.request.supportHeightToleranceM=.01;
  fixture.request.robot.kinematics.valid=true;
  fixture.request.robot.kinematics.linearVelocity={.12,0,0};
  // Initial reference acquisition uses the upstream odometry P/V contract,
  // which has no acceleration observation. It starts with zero acceleration.
  // Running handoff is a different contract: it inherits the executing spline's
  // full P/V/A, covered by DelayedHandoffMatchesExecutingPositionVelocityAndAcceleration.
  fixture.request.robot.kinematics.linearAcceleration={.03,0,0};
  CollisionBitmap floor({-5,-5,-1},{5,5,2},.1);
  for(int y=-50;y<50;++y) for(int x=-50;x<50;++x)
    floor.occupy({(x+.5)*.1,(y+.5)*.1,-.05});
  const auto raw=floor.view();
  fixture.request.environment.collision.measuredOccupiedStorage=
      std::make_shared<const std::vector<std::uint8_t>>(raw.inflatedBits,raw.inflatedBits+raw.inflatedBytes);
  fixture.request.environment.collision.knownFreeStorage=
      std::make_shared<const std::vector<std::uint8_t>>(raw.inflatedBytes,255);
  auto params=scanParams();params.scan.noReplanDistance=.001;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for(int tick=0;tick<30 && !plan.ready();++tick) {
    fixture.request.clock.timestampS+=.01;
    plan=backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason << ':'
                           << backend.debugSnapshot().scanAttempt.reason;
  const auto withBody = std::get<SplineTarget>(plan.target());
  fixture.request.body.reset();
  fixture.request.supportHeightM.reset();
  fixture.request.supportHeightToleranceM.reset();
  nav_kernel::local::scan::Backend reference(params);
  plan = {};
  for (int tick=0; tick<30 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += .01;
    plan = reference.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto withoutBody = std::get<SplineTarget>(plan.target());
  ASSERT_EQ(withBody.controls.size(),withoutBody.controls.size());
  for (std::size_t i=0;i<withBody.controls.size();++i)
    EXPECT_LT(nav_kernel::distance3D(withBody.controls[i],withoutBody.controls[i]),1e-10);
}

TEST(ScanBackend, RemovesShortCollinearCorridorCutBeforeEndpoint) {
  RequestFixture fixture({{0, 0, 0.5}, {3.5, 0, 0.5}, {3.7, 0, 0.5}});
  fixture.request.robot.pose.yaw = M_PI;
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
  double minimumX = 0.0;
  for (double time = 0.0; time <= spline.duration(); time += 0.01)
    minimumX = std::min(minimumX, spline.position(time).x);
  EXPECT_GE(minimumX, -0.02);
  const auto debug = backend.debugSnapshot();
  ASSERT_TRUE(debug.localTargetValid);
  EXPECT_GT(debug.localTarget.x, 3.0);
  EXPECT_LE(debug.localTarget.x, 3.7);
  EXPECT_NEAR(spline.position(spline.duration()).x, debug.localTarget.x, 0.02);
}

TEST(ScanReferenceAdapter, FoldsOnlyShortCollinearCorridorCuts) {
  const std::vector<Vec3> collinear{
      {0, 0, 0.5}, {3.5, 0, 0.5}, {3.7, 0, 0.5}};
  const auto folded = nav_kernel::local::scan::detail::adaptReferencePath(
      {collinear.data(), static_cast<int>(collinear.size()), 1}, 0.4);
  ASSERT_EQ(folded.size(), 2U);
  EXPECT_DOUBLE_EQ(folded.back().x(), 3.7);
  EXPECT_NEAR(folded.back().z(), 0.1, 1e-12);

  const std::vector<Vec3> lateralBend{
      {0, 0, 0.5}, {3.5, 0, 0.5}, {3.5, 0.2, 0.5}, {4.0, 0.2, 0.5}};
  const auto lateral = nav_kernel::local::scan::detail::adaptReferencePath(
      {lateralBend.data(), static_cast<int>(lateralBend.size()), 1}, 0.4);
  ASSERT_EQ(lateral.size(), lateralBend.size());
  EXPECT_DOUBLE_EQ(lateral[2].y(), 0.2);
  EXPECT_DOUBLE_EQ(lateral.back().x(), 4.0);

  const std::vector<Vec3> elevationBend{
      {0, 0, 0.5}, {3.5, 0, 0.5}, {3.7, 0, 0.7}, {4.0, 0, 0.7}};
  const auto elevated = nav_kernel::local::scan::detail::adaptReferencePath(
      {elevationBend.data(), static_cast<int>(elevationBend.size()), 1}, 0.4);
  ASSERT_EQ(elevated.size(), elevationBend.size());
  EXPECT_NEAR(elevated[2].z(), 0.3, 1e-12);
  EXPECT_DOUBLE_EQ(elevated.back().x(), 4.0);
}

TEST(ScanBackend, OccupiedEndpointFallbackRetainsGoalForMapRecovery) {
  RequestFixture fixture({{0, 0, 0.5}, {2, 0, 0.5}}, 0.05);
  for (int x = 1; x < 50; ++x)
    fixture.bitmap.occupy({x * 0.05 + 0.025, 0.025, 0.525});
  fixture.refreshCollision(2);
  auto params = scanParams();
  params.scan.voxelResolution = 0.05;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 20; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const nav_kernel::SplineView prefix(std::get<SplineTarget>(plan.target()));
  ASSERT_GT(prefix.position(prefix.duration()).x, 0.001);
  ASSERT_LT(prefix.position(prefix.duration()).x, 0.05);
  fixture.request.robot.pose.position = prefix.position(prefix.duration());
  fixture.request.clock.timestampS += prefix.duration();
  fixture.bitmap.clear();
  fixture.refreshCollision(3);
  for (int tick = 0; tick < 30; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    if (plan.ready()) {
      const nav_kernel::SplineView candidate(std::get<SplineTarget>(plan.target()));
      if (std::abs(candidate.position(candidate.duration()).x - 2.0) < 0.02) break;
    }
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
  EXPECT_NEAR(spline.position(spline.duration()).x, 2.0, 0.02);
}

TEST(ScanTrajectoryValidation, TranslatedWallHasSafePrefixAndReplansBeforeUnsafeTail) {
  RequestFixture fixture({{10.0, -3.0, 0.0}, {10.0, -2.2, 0.0},
                          {10.0, -1.4, 0.0}, {10.0, -0.6, 0.0}});
  fixture.bitmap = CollisionBitmap({5.0, -8.0, -2.0}, {15.0, 2.0, 2.0}, 0.1);
  for (int forward = 8; forward <= 16; ++forward)
    for (int lateral = -11; lateral <= 11; ++lateral)
      for (int vertical = -4; vertical <= 4; ++vertical)
        fixture.bitmap.occupy({static_cast<float>(10.0 - lateral * 0.1),
                               static_cast<float>(-3.0 + forward * 0.1),
                               static_cast<float>(vertical) * 0.1F});
  fixture.refreshCollision(2);
  fixture.request.robot.pose.yaw = std::acos(-1.0) * 0.5;
  fixture.request.maxLinearSpeedMps = 0.5;
  auto params = scanParams();
  params.scan.cylinderOffset = 0.18;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 50 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const SplineTarget original = std::get<SplineTarget>(plan.target());
  const nav_kernel::SplineView spline(original);
  ASSERT_TRUE(spline.valid());
  nav_kernel::local::scan::Grid grid(params, fixture.request);
  const int samples = static_cast<int>(std::ceil(spline.duration() / 0.002));
  const double checkedPrefix = spline.duration() * 2.0 / 3.0;
  double firstUnsafeTail = spline.duration();
  for (int index = 0; index <= samples; ++index) {
    const double time = spline.duration() * index / samples;
    const Vec3 position = spline.position(time);
    const Vec3 velocity = spline.velocity(time);
    ASSERT_TRUE(std::isfinite(position.x) && std::isfinite(position.y) &&
                std::isfinite(position.z));
    ASSERT_TRUE(std::isfinite(velocity.x) && std::isfinite(velocity.y) &&
                std::isfinite(velocity.z));
    const bool free = grid.obstacleFree(position, std::atan2(velocity.y, velocity.x));
    if (time <= checkedPrefix) {
      EXPECT_TRUE(free) << "time=" << time << " fraction=" << time / spline.duration();
    } else if (!free) {
      firstUnsafeTail = std::min(firstUnsafeTail, time);
    }
  }
  if (firstUnsafeTail < spline.duration()) {
    // The upstream rolling-horizon contract checks the tail as execution enters it.
    double time = checkedPrefix + 0.01;
    bool replaced = false;
    for (int tick = 0; tick < 20 && time < firstUnsafeTail && !replaced; ++tick, time += 0.01) {
      fixture.request.clock.timestampS = original.startTimeS + time;
      fixture.request.robot.pose.position = spline.position(time);
      fixture.request.robot.kinematics.linearVelocity = spline.velocity(time);
      fixture.request.robot.kinematics.valid = true;
      const Vec3 velocity = fixture.request.robot.kinematics.linearVelocity;
      fixture.request.robot.pose.yaw = std::atan2(velocity.y, velocity.x);
      plan = tick == 0 ? backend.checkCollision(fixture.request) : backend.tick(fixture.request);
      replaced = plan.status() == nav_kernel::LocalPlanStatus::NearFieldStop ||
                 (plan.ready() && std::get<SplineTarget>(plan.target()).trajectoryId != original.trajectoryId);
    }
    ASSERT_TRUE(replaced) << "the unsafe tail must be replaced or stopped before its first collision";
    if (plan.status() == nav_kernel::LocalPlanStatus::NearFieldStop) {
      EXPECT_FALSE(plan.ready());
      return;
    }
    const nav_kernel::SplineView replacement(std::get<SplineTarget>(plan.target()));
    ASSERT_TRUE(replacement.valid());
    for (double time = 0.0; time < replacement.duration() * 2.0 / 3.0; time += 0.002) {
      const Vec3 position = replacement.position(time);
      const Vec3 velocity = replacement.velocity(time);
      EXPECT_TRUE(grid.obstacleFree(position, std::atan2(velocity.y, velocity.x)));
    }
  }
}

TEST(ScanTrajectoryValidation, EmergencyStopIsNotReportedAsReadyAndRecoversWhenClear) {
  RequestFixture fixture({{0, 0, 0.5}, {2, 0, 0.5}}, 0.05);
  LocalPlannerParams params;
  params.backend = LocalPlannerBackend::Scan;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS = 1.0 + tick * 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto firstId = std::get<SplineTarget>(plan.target()).trajectoryId;
  // A wall fills the complete local volume ahead; there is no detour.
  for (int x = 7; x <= 17; ++x)
    for (int y = -100; y < 100; ++y)
      for (int z = -20; z < 40; ++z)
        fixture.bitmap.occupy({x * 0.05 + 0.025, y * 0.05 + 0.025, z * 0.05 + 0.025});
  fixture.refreshCollision(2);
  {
    const auto &target = std::get<SplineTarget>(plan.target());
    const nav_kernel::SplineView original(target);
    const nav_kernel::local::scan::Grid collision_grid(params, fixture.request);
    double first_collision = original.duration();
    for (double time = 0.0; time < original.duration(); time += .01) {
      const auto position = original.position(time);
      const auto velocity = original.velocity(time);
      if (!collision_grid.obstacleFree(position, std::atan2(velocity.y, velocity.x))) {
        first_collision = time;
        break;
      }
    }
    ASSERT_LT(first_collision, original.duration());
    // Exercise the same imminent collision even when seed timing changes.
    fixture.request.clock.timestampS = target.startTimeS +
        std::max(0.0, first_collision - .25);
  }
  plan = backend.checkCollision(fixture.request);
  ASSERT_EQ(plan.status(), nav_kernel::LocalPlanStatus::NearFieldStop);
  ASSERT_FALSE(plan.ready());
  EXPECT_TRUE(plan.previewPath().empty());
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_emergency_stop");
  fixture.request.robot.kinematics.valid = true;
  fixture.request.robot.kinematics.linearVelocity = {0.0, 0.0, 0.12};
  for (int tick = 0; tick < 3; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    EXPECT_FALSE(plan.ready());
  }

  fixture.bitmap.clear();
  fixture.refreshCollision(3);
  for (int tick = 0; tick < 3; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    EXPECT_EQ(backend.tick(fixture.request).status(), nav_kernel::LocalPlanStatus::NearFieldStop);
  }
  fixture.request.robot.kinematics.linearVelocity = {};
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const auto &target = std::get<SplineTarget>(plan.target());
  EXPECT_GT(target.trajectoryId, firstId);
  const nav_kernel::SplineView spline(target);
  ASSERT_TRUE(spline.valid());
  EXPECT_GT(spline.position(spline.duration()).x, 1.0);
}

TEST(ScanGridAdapter, RequiresTheConfiguredCollisionMapResolution) {
  const auto params = scanParams();
  const std::vector<Vec3> route{{0, 0, .5}, {2, 0, .5}};
  RequestFixture matching(route, .10);
  const nav_kernel::local::scan::Grid valid(params, matching.request);
  ASSERT_TRUE(valid.valid()) << valid.reason();
  EXPECT_DOUBLE_EQ(valid.resolution(), .10);

  RequestFixture mismatching(route, .05);
  const nav_kernel::local::scan::Grid invalid(params, mismatching.request);
  EXPECT_FALSE(invalid.valid());
  EXPECT_EQ(invalid.reason(), "collision_map_resolution_mismatch");
}

TEST(ScanGridAdapter, DisabledCollisionCheckingPreservesSamplingResolution) {
  auto params = scanParams();
  params.checkObstacle = false;
  LocalPlanRequest request;
  nav_kernel::local::scan::Grid grid(params, request);
  ASSERT_TRUE(grid.valid());
  EXPECT_DOUBLE_EQ(grid.resolution(), params.scan.voxelResolution);
  EXPECT_GT(grid.resolution(), 0.0);
  const nav_kernel::local::scan::upstream::GridMap map(grid);
  EXPECT_DOUBLE_EQ(map.getResolution(), params.scan.voxelResolution);
  EXPECT_EQ(map.getInflateOccupancy({0.0, 0.0, 0.5}, 0.0), 0);
  EXPECT_EQ(map.getInflateOccupancySegment({-100.0, 0.0, 0.5}, 0.0,
                                          {100.0, 0.0, 0.5}, 0.0), 0);

  params.scan.voxelResolution = 0.0;
  nav_kernel::local::scan::Grid invalid(params, request);
  EXPECT_FALSE(invalid.valid());
  EXPECT_EQ(invalid.reason(), "grid_resolution_invalid");
}

TEST(ScanGridAdapter, SegmentDetectsThinCellsBetweenBothCylinderCenters) {
  const Vec3 start{-0.45, 0.05, 0.55};
  const Vec3 end{0.65, 0.05, 0.55};
  const double yaw = std::acos(-1.0) * 0.5;
  auto params = scanParams();
  params.scan.cylinderOffset = 0.2;
  for (const double sign : {-1.0, 1.0}) {
    RequestFixture fixture({start, end});
    fixture.bitmap.occupy({0.05, 0.05 + sign * params.scan.cylinderOffset, 0.55});
    fixture.refreshCollision(2);
    nav_kernel::local::scan::Grid grid(params, fixture.request);
    ASSERT_EQ(grid.inflatedOccupancy(start, yaw), 0);
    ASSERT_EQ(grid.inflatedOccupancy(end, yaw), 0);
    EXPECT_EQ(grid.segmentInflatedOccupancy(start, yaw, end, yaw), 1);
    EXPECT_EQ(grid.segmentInflatedOccupancy(end, yaw, start, yaw), 1);
    EXPECT_EQ(grid.segmentInflatedOccupancy(start, yaw, start, yaw), 0);
    EXPECT_EQ(grid.segmentInflatedOccupancy({-5.1, 0.05, 0.55}, yaw, start, yaw), -1);
  }
}

TEST(ScanGridAdapter, SegmentTransformsPlanningCoordinatesOnce) {
  const Vec3 start{-0.45, 0.05, 0.55};
  const Vec3 end{0.65, 0.05, 0.55};
  RequestFixture fixture({start, end});
  fixture.bitmap.occupy({1.25, -0.55, 0.75});
  fixture.refreshCollision(2);
  fixture.request.environment.collision.gridFromPlanningTranslation = {1.3, -0.6, 0.2};
  fixture.request.environment.collision.gridFromPlanningYaw = std::acos(-1.0) * 0.5;
  nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
  const nav_kernel::local::scan::upstream::GridMap map(grid);
  ASSERT_EQ(map.getInflateOccupancy({start.x, start.y, start.z}, 0.0), 0);
  ASSERT_EQ(map.getInflateOccupancy({end.x, end.y, end.z}, 0.0), 0);
  EXPECT_EQ(map.getInflateOccupancySegment({start.x, start.y, start.z}, 0.0,
                                          {end.x, end.y, end.z}, 0.0), 1);
}

TEST(ScanGridAdapter, SegmentChecksVerticalCellsAndCornerContact) {
  RequestFixture fixture({{0.05, 0.05, 0.05}, {0.05, 0.05, 0.95}});
  fixture.bitmap.occupy({0.05, 0.05, 0.55});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Grid vertical(scanParams(), fixture.request);
  EXPECT_EQ(vertical.segmentInflatedOccupancy(fixture.route.front(), 0.0,
                                              fixture.route.back(), 0.0), 1);

  fixture.bitmap.clear();
  fixture.bitmap.occupy({0.15, 0.15, 0.55});
  fixture.refreshCollision(3);
  nav_kernel::local::scan::Grid corner(scanParams(), fixture.request);
  ASSERT_EQ(corner.inflatedOccupancy({0.15, 0.05, 0.55}, 0.0), 0);
  ASSERT_EQ(corner.inflatedOccupancy({0.05, 0.15, 0.55}, 0.0), 0);
  EXPECT_EQ(corner.segmentInflatedOccupancy({0.15, 0.05, 0.55}, 0.0,
                                           {0.05, 0.15, 0.55}, 0.0), 1);
}

TEST(ScanGridAdapter, LateralIntentUsesBodyFootprintInsteadOfPathTangent) {
  RequestFixture fixture({{0.05, 0.05, 0.55}, {0.05, 2.05, 0.55}});
  auto params = scanParams();
  params.scan.cylinderOffset = 0.25;
  fixture.bitmap.occupy({0.30, 0.75, 0.55});
  fixture.refreshCollision(2);
  const Vec3 end{0.05, 1.05, 0.55};
  nav_kernel::local::scan::Grid autonomous(params, fixture.request);
  EXPECT_EQ(autonomous.segmentInflatedOccupancy(fixture.route.front(), M_PI / 2, end, M_PI / 2), 0);
  fixture.request.objective = nav_kernel::MotionIntentTarget{
      {90.0, 0.5, 2.0, 90.0}, {fixture.route.data(), 2, 1}};
  nav_kernel::local::scan::Grid lateral(params, fixture.request);
  EXPECT_EQ(lateral.segmentInflatedOccupancy(fixture.route.front(), M_PI / 2, end, M_PI / 2), 1);
  EXPECT_EQ(lateral.inflatedOccupancy({0.05, 0.75, 0.55}, M_PI / 2), 1);
}

TEST(ScanGridAdapter, RouteBodyHeadingKeepsThreeDimensionalCollisionFootprint) {
  RequestFixture fixture({{0.05, 0.05, 0.55}, {0.05, 2.05, 0.55}});
  auto params = scanParams();
  params.scan.cylinderOffset = 0.25;
  const Vec3 end{0.05, 1.05, 0.55};
  fixture.bitmap.occupy({0.30, 0.75, 0.55});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Grid ordinary(params, fixture.request);
  EXPECT_EQ(ordinary.segmentInflatedOccupancy(fixture.route.front(), M_PI / 2, end, M_PI / 2), 0);
  fixture.request.bodyHeading = 0.0;
  nav_kernel::local::scan::Grid fixed(params, fixture.request);
  EXPECT_EQ(fixed.segmentInflatedOccupancy(fixture.route.front(), M_PI / 2, end, M_PI / 2), 1);
  EXPECT_EQ(fixed.trajectoryOccupancy(fixture.route.front(), {0, .15, 0}, end, {0, .15, 0}, 1, 2), 1);
  EXPECT_EQ(fixed.inflatedOccupancy({0.05, 0.75, 0.55}, M_PI / 2), 1);
  EXPECT_EQ(fixed.segmentInflatedOccupancy({.05, .05, 1.55}, M_PI / 2,
                                          {.05, 1.05, 1.55}, M_PI / 2), 0);

  fixture.bitmap.clear();
  fixture.bitmap.occupy({0.05, 1.30, 0.55});
  fixture.refreshCollision(3);
  nav_kernel::local::scan::Grid sideStep(params, fixture.request);
  EXPECT_EQ(sideStep.segmentInflatedOccupancy(fixture.route.front(), M_PI / 2, end, M_PI / 2), 0);
  fixture.request.bodyHeading.reset();
  nav_kernel::local::scan::Grid tangent(params, fixture.request);
  EXPECT_EQ(tangent.segmentInflatedOccupancy(fixture.route.front(), M_PI / 2, end, M_PI / 2), 1);
}

TEST(ScanGridAdapter, RouteBodyHeadingChecksAlignmentArcAndKeepsActualBrakingYaw) {
  RequestFixture fixture({{0.05, 0.05, 0.55}, {2.05, 0.05, 0.55}});
  auto params = scanParams();
  params.scan.cylinderOffset = 0.25;
  fixture.request.robot.pose.yaw = M_PI / 2;
  fixture.request.bodyHeading = 0.0;
  fixture.bitmap.occupy({0.22, 0.22, 0.55});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Grid turning(params, fixture.request);
  EXPECT_EQ(turning.inflatedOccupancy(fixture.route.front(), 0), 1);
  EXPECT_EQ(turning.trajectoryOccupancy(fixture.route.front(), {}, fixture.route.front(), {}, 0, 1), 1);
  // The final brake query follows actual twist, not a desired future heading.
  EXPECT_EQ(turning.brakingOccupancy(fixture.request.robot.pose, {}, .1, .5, 1), 0);
  EXPECT_EQ(turning.brakingOccupancy(fixture.request.robot.pose, {0, 0, -1}, .35, .5, 1), 1);

  fixture.request.robot.pose.yaw = 0;
  nav_kernel::local::scan::Grid aligned(params, fixture.request);
  EXPECT_EQ(aligned.inflatedOccupancy(fixture.route.front(), 0), 0);
  // A model that is already facing the flight must not inherit the previous turn.
  fixture.request.robot.pose.yaw = -M_PI + .12;
  fixture.request.bodyHeading = M_PI;
  nav_kernel::local::scan::Grid wrapped(params, fixture.request);
  EXPECT_EQ(wrapped.inflatedOccupancy(fixture.route.front(), 0), 0);
}

TEST(ScanGridAdapter, RouteBodyHeadingUsesPlanningFrameForMeasuredAndPredictedObstacles) {
  RequestFixture fixture({{0.05, 0.05, 0.55}, {0.05, 2.05, 0.55}});
  auto params = scanParams();
  params.scan.cylinderOffset = .24;
  fixture.request.bodyHeading = 0.0;
  fixture.bitmap.occupy({-.75, .25, .55});
  fixture.refreshCollision(2);
  fixture.request.environment.collision.gridFromPlanningYaw = M_PI / 2;
  nav_kernel::local::scan::Grid rotated(params, fixture.request);
  EXPECT_EQ(rotated.segmentInflatedOccupancy(fixture.route.front(), M_PI / 2,
                                            {.05, 1.05, .55}, M_PI / 2), 1);
  fixture.bitmap.clear();
  fixture.refreshCollision(3);
  fixture.request.robot.pose.yaw = M_PI / 2;
  params.scan.cylinderOffset = .25;
  params.scan.cylinderRadius = .001;
  const nav_kernel::PredictedObstacle obstacle{{.225, .225, .55}, {.225, .225, .55},
                                               .001, .50, .60};
  fixture.request.environment.predictions = {&obstacle, 1, 2, 1};
  nav_kernel::local::scan::Grid predicted(params, fixture.request);
  EXPECT_EQ(predicted.trajectoryOccupancy(fixture.route.front(), {}, fixture.route.front(), {}, 0, .2), 1);
  EXPECT_EQ(predicted.trajectoryOccupancy({.05, .05, 1.55}, {}, {.05, .05, 1.55}, {}, 0, .2), 0);
}

TEST(ScanGridAdapter, BrakingSweepUsesActualPoseAndLatencyBeforeStopping) {
  RequestFixture fixture({{0.05, 0.05, 0.55}, {2.05, 0.05, 0.55}});
  auto params = scanParams();
  params.scan.cylinderOffset = 0.25;
  fixture.bitmap.occupy({0.70, 0.45, 0.55});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Grid grid(params, fixture.request);
  const nav_kernel::Pose actual{{0.05, 0.45, 0.55}, 0.0};
  // The planned centreline is clear; the physical robot has deviated sideways.
  EXPECT_EQ(grid.brakingOccupancy(fixture.request.robot.pose, {0.5, 0, 0}, 0.35, 0.5, 1.0), 0);
  EXPECT_EQ(grid.brakingOccupancy(actual, {0.5, 0, 0}, 0.35, 0.5, 1.0), 1);
  EXPECT_EQ(grid.brakingOccupancy(actual, {0.5, 0, 0}, 0.0, 0.5, 1.0), 0);
  EXPECT_EQ(grid.brakingOccupancy(actual, {-0.5, 0, 0}, 0.35, 0.5, 1.0), 0);
  EXPECT_EQ(grid.brakingOccupancy(actual, {0, -0.5, 0}, 0.35, 0.5, 1.0), 0);
}

TEST(ScanGridAdapter, NarrowCorridorAllowsForwardBodyButBlocksIntermediateTurn) {
  RequestFixture fixture({{.025, .025, .525}, {1.025, .025, .525}}, .05);
  auto params = scanParams();
  params.scan.voxelResolution = .05;
  params.scan.cylinderRadius = .25;
  params.scan.cylinderOffset = .19;
  params.scan.bodyClearanceBelow = .10;
  params.scan.bodyClearanceAbove = .25;
  std::vector<float> walls;
  // Source wall cells have inner faces at Y +/-0.30 m. The supplied bitmap
  // already contains their 0.25 m radial inflation, as a Mapd snapshot does.
  for (int x = -40; x <= 40; ++x)
    for (int z = 0; z <= 20; ++z)
      for (double y : {-.325, .325}) {
        walls.push_back(static_cast<float>((x + .5) * .05));
        walls.push_back(static_cast<float>(y));
        walls.push_back(static_cast<float>((z + .5) * .05));
      }
  fixture.bitmap.occupyInflated(walls, params.scan.cylinderRadius, 0, 0);
  fixture.refreshCollision(2);
  const nav_kernel::local::scan::Grid grid(params, fixture.request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  const auto body = fixture.request.robot.pose;
  const double pi = std::acos(-1.0);
  EXPECT_EQ(grid.inflatedOccupancy(body.position, 0), 0);
  EXPECT_EQ(grid.inflatedOccupancy(body.position, pi), 0);
  EXPECT_EQ(grid.segmentInflatedOccupancy(body.position, 0, fixture.route.back(), 0), 0);
  EXPECT_EQ(grid.brakingOccupancy(body, {.3, 0, 0}, .35, .5, 1), 0);
  EXPECT_DOUBLE_EQ(grid.brakingScale(body, {.3, 0, 0}, .35, .5, 1), 1);

  // Both ends of a half turn fit, but its intermediate perpendicular pose
  // does not. Actual-command braking must check the turning arc, not its ends.
  EXPECT_EQ(grid.inflatedOccupancy(body.position, pi / 2), 1);
  EXPECT_EQ(grid.brakingOccupancy(body, {0, 0, 1}, .35, .5, 1), 1);
  EXPECT_EQ(grid.brakingOccupancy(body, {0, 0, -1}, .35, .5, 1), 1);
  const double scale = grid.brakingScale(body, {0, 0, 1}, .35, .5, 1);
  EXPECT_LT(scale, 1);
  EXPECT_EQ(grid.brakingOccupancy(body, {0, 0, scale}, .35, .5, 1), 0);
  fixture.request.bodyHeading = pi;
  const nav_kernel::local::scan::Grid alignment(params, fixture.request);
  EXPECT_EQ(alignment.inflatedOccupancy(body.position, 0), 1);
}

TEST(ScanGridAdapter, GarageRearCollisionBlocksActualCommandDespiteClearForwardGuide) {
  auto params = scanParams();
  params.scan.voxelResolution = .05;
  params.scan.cylinderRadius = .25;
  params.scan.cylinderOffset = .19;
  params.scan.bodyClearanceBelow = .10;
  params.scan.bodyClearanceAbove = .25;
  CollisionBitmap bitmap({2, -2, -2}, {5, 2, 0}, .05);
  std::vector<float> leaf;
  // The fixed Garage witness is one original occupied 0.10 m leaf centred at
  // (3.45, -0.15, -1.25), represented by its eight 0.05 m lookup subcells.
  for (double x : {3.425, 3.475})
    for (double y : {-.175, -.125})
      for (double z : {-1.275, -1.225}) {
        leaf.push_back(static_cast<float>(x));
        leaf.push_back(static_cast<float>(y));
        leaf.push_back(static_cast<float>(z));
      }
  bitmap.occupyInflated(leaf, params.scan.cylinderRadius,
                       params.scan.bodyClearanceAbove, params.scan.bodyClearanceBelow);
  LocalPlanRequest request;
  request.robot.pose = {{3.675405740737915, -.20373751223087311, -1.1807905435562134},
                        -.090659887607880671};
  const std::vector<Vec3> guide{{4.0, -.20373751223087311, -1.1807905435562134},
                               {4.5, -.20373751223087311, -1.1807905435562134}};
  request.objective = RouteTarget{{guide.data(), static_cast<int>(guide.size()), 1}};
  request.environment.collision = bitmap.view();
  const nav_kernel::local::scan::Grid grid(params, request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  const auto body = request.robot.pose;
  ASSERT_EQ(grid.segmentInflatedOccupancy(guide.front(), body.yaw, guide.back(), body.yaw), 0);
  const Vec3 offset{params.scan.cylinderOffset * std::cos(body.yaw),
                    params.scan.cylinderOffset * std::sin(body.yaw), 0};
  params.scan.cylinderOffset = 0;
  const nav_kernel::local::scan::Grid single(params, request);
  EXPECT_EQ(single.inflatedOccupancy({body.position.x + offset.x,
                                      body.position.y + offset.y, body.position.z}, body.yaw), 0);
  EXPECT_EQ(single.inflatedOccupancy({body.position.x - offset.x,
                                      body.position.y - offset.y, body.position.z}, body.yaw), 1);
  EXPECT_EQ(grid.inflatedOccupancy(body.position, body.yaw), 1);
  // A clear reference ahead cannot replace the measured starting footprint.
  for (const nav_kernel::Twist command : {nav_kernel::Twist{.3, 0, 0},
                                          nav_kernel::Twist{0, 0, .8}}) {
    EXPECT_EQ(grid.brakingOccupancy(body, command, .35, .5, 1), 1);
    EXPECT_DOUBLE_EQ(grid.brakingScale(body, command, .35, .5, 1), 0);
  }
}

TEST(ScanGridAdapter, BrakingSweepChecksRotationAndMapBoundary) {
  RequestFixture fixture({{0.05, 0.05, 0.55}, {2.05, 0.05, 0.55}});
  auto params = scanParams();
  params.scan.cylinderOffset = 0.25;
  fixture.bitmap.occupy({0.20, 0.20, 0.55});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Grid grid(params, fixture.request);
  EXPECT_EQ(grid.brakingOccupancy(fixture.request.robot.pose, {0, 0, 1.0}, 0.35, 0.5, 1.0), 1);
  EXPECT_EQ(grid.brakingOccupancy(fixture.request.robot.pose, {0, 0, -1.0}, 0.35, 0.5, 1.0), 0);
  EXPECT_EQ(grid.brakingOccupancy({{4.55, 0.05, 0.55}, 0}, {0.5, 0, 0}, 0.35, 0.5, 1.0), -1);
}

TEST(ScanGridAdapter, BrakingLimitApproachesStairsWithoutInventingBodyHeight) {
  RequestFixture fixture({{0.05, 0.05, 0.47}, {2.05, 0.05, 1.47}}, 0.05);
  auto params = scanParams();
  params.scan.voxelResolution = 0.05;
  params.scan.cylinderOffset = 0.25;
  // Inflated upper treads intersect a fast, level braking sweep even though
  // the robot can still approach the first step at a lower speed.
  for (int x = 18; x <= 40; ++x)
    for (int y = -10; y <= 10; ++y)
      fixture.bitmap.occupy({x * 0.05 + 0.025, y * 0.05 + 0.025, 0.475});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Grid grid(params, fixture.request);
  const auto body = fixture.request.robot.pose;
  ASSERT_EQ(grid.brakingOccupancy(body, {0.75, 0, 0}, 0.35, 0.5, 1.0), 1);
  const double scale = grid.brakingScale(body, {0.75, 0, 0}, 0.35, 0.5, 1.0);
  EXPECT_GT(scale, 0.3);
  EXPECT_LT(scale, 1.0);
  EXPECT_EQ(grid.brakingOccupancy(body, {0.75 * scale, 0, 0}, 0.35, 0.5, 1.0), 0);
  // Moving the real body into that tread must still stop. The ascending
  // reference endpoint cannot authorize lifting the safety footprint.
  EXPECT_DOUBLE_EQ(grid.brakingScale({{0.70, 0.05, 0.47}, 0},
                                      {0.75, 0, 0}, 0.35, 0.5, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(grid.brakingScale(body, {-0.5, 0, 0}, 0.35, 0.5, 1.0), 1.0);
}

TEST(ScanGridAdapter, BrakingLimitRetainsCollisionCheckedTurningArc) {
  RequestFixture fixture({{0.05, 0.05, 0.55}, {2.05, 0.05, 0.55}});
  auto params = scanParams();
  params.scan.cylinderOffset = 0.25;
  fixture.bitmap.occupy({0.50, 0.25, 0.55});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Grid grid(params, fixture.request);
  const auto body = fixture.request.robot.pose;
  const nav_kernel::Twist command{0.5, 0, 1.0};
  ASSERT_NE(grid.brakingOccupancy(body, command, 0.35, 0.5, 1.0), 0);
  const double scale = grid.brakingScale(body, command, 0.35, 0.5, 1.0);
  EXPECT_GT(scale, 0.0);
  EXPECT_LT(scale, 1.0);
  EXPECT_EQ(grid.brakingOccupancy(body, {0.5 * scale, 0, scale}, 0.35, 0.5, 1.0), 0);
}

TEST(ScanGridAdapter, ReadsMapdInflatedBitsWithoutReinflating) {
  RequestFixture fixture({{-1.0, 0.0, 0.5}, {1.0, 0.0, 0.5}});
  fixture.bitmap.occupy({0.0, 0.0, 0.5});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  EXPECT_EQ(grid.occupiedCellCount(), 1);
  EXPECT_EQ(grid.collisionPointCount(), 1);

  auto map = std::make_shared<nav_kernel::local::scan::upstream::GridMap>(grid);
  EXPECT_EQ(map->getInflateOccupancy({0.0, 0.0, 0.5}, 0.0), 1);
  EXPECT_EQ(map->getInflateOccupancy({0.5, 0.0, 0.5}, 0.0), 0);
}

TEST(ScanGridAdapter, PreservesOfficialBoundaryAndDoubleCylinderSemantics) {
  RequestFixture fixture({{-1.0, 0.0, 0.5}, {1.0, 0.0, 0.5}});
  fixture.refreshCollision(2);
  auto params = scanParams();
  params.scan.cylinderOffset = 0.25;
  nav_kernel::local::scan::Grid grid(params, fixture.request);
  ASSERT_TRUE(grid.valid()) << grid.reason();

  EXPECT_EQ(grid.inflatedOccupancy({-4.7501, 0.0, 0.5}, 3.14159265358979323846), -1);
  EXPECT_EQ(grid.inflatedOccupancy({-4.7498, 0.0, 0.5}, 3.14159265358979323846), 0);

  fixture.bitmap.occupy({0.25, 0.0, 0.5});
  fixture.refreshCollision(3);
  nav_kernel::local::scan::Grid occupiedGrid(params, fixture.request);
  EXPECT_EQ(occupiedGrid.inflatedOccupancy({0.0, 0.0, 0.5}, 0.0), 1);
}

TEST(ScanDynAStar, MovesOccupiedStartBackwardLikeUpstream) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {1.5, 0.0, 0.5}});
  fixture.bitmap.occupy({0.0, 0.0, 0.5});
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
  ASSERT_TRUE(grid.valid()) << grid.reason();

  auto map = std::make_shared<nav_kernel::local::scan::upstream::GridMap>(grid);
  nav_kernel::local::scan::upstream::AStar search;
  search.initGridMap(map, {40, 40, 20});
  ASSERT_EQ(search.search(0.1, {0.0, 0.0, 0.5}, {1.5, 0.0, 0.5}),
            nav_kernel::local::scan::upstream::AStarResult::Success);
  const auto path = search.path();
  ASSERT_GE(path.size(), 2U);
  EXPECT_LT(path.front().x(), -0.05);
  EXPECT_NEAR(path.back().x(), 1.5, 0.11);
}

TEST(ScanDynAStar, AdjustsGuideEndToAnAdmissibleIncomingHeading) {
  using namespace nav_kernel::local::scan;
  using namespace upstream;
  const auto path = std::filesystem::path(__FILE__).parent_path() /
      "fixtures/scan_guide_endpoint.bits";
  std::ifstream file(path, std::ios::binary);
  ASSERT_TRUE(file.is_open());
  const std::vector<std::uint8_t> bits((std::istreambuf_iterator<char>(file)), {});
  ASSERT_EQ(bits.size(), 750U);
  LocalPlanRequest request;
  auto &view = request.environment.collision;
  view.inflatedBits = bits.data(); view.inflatedBytes = bits.size();
  view.sizeX = 60; view.sizeY = 50; view.sizeZ = 2;
  view.resolution = .05;
  view.aabbMin = {2, -1, .4}; view.aabbMax = {5, 1.5, .5};
  view.complete = true; view.live = true;
  auto params = scanParams();
  params.scan.voxelResolution = .05;
  params.scan.cylinderOffset = .23;
  Grid grid(params, request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  auto map = std::make_shared<GridMap>(grid);
  const Eigen::Vector3d start(3.1569167801104676, .69728865808315, .4386563541387469);
  const Eigen::Vector3d end(3.560668099140927, .6293777752508265, .4379328672213102);
  const Eigen::Vector3d center = .5 * (start + end);
  const Eigen::Vector3d snapped = center + ((end - center) / .05).array().round().matrix() * .05;
  const double yaw = std::atan2(end.y() - start.y(), end.x() - start.x());
  ASSERT_EQ(map->getInflateOccupancy(snapped, yaw), 0);
  // The recorded endpoint admits the chord heading, but none of the search directions.
  for (int dx = -1; dx <= 1; ++dx)
    for (int dy = -1; dy <= 1; ++dy)
      if (dx != 0 || dy != 0)
        ASSERT_EQ(map->getInflateOccupancy(snapped, std::atan2(double(dy), double(dx))), 1);
  AStar search;
  search.initGridMap(map, {80, 80, 6});
  ASSERT_EQ(search.search(.05, start, end), AStarResult::Success);
  const auto guide = search.path();
  ASSERT_GE(guide.size(), 2U);
  EXPECT_GT((guide.back() - snapped).norm(), .049);
  for (std::size_t i = 1; i < guide.size(); ++i) {
    const auto direction = guide[i] - guide[i - 1];
    EXPECT_EQ(map->getInflateOccupancy(guide[i], std::atan2(direction.y(), direction.x())), 0);
  }
}

TEST(ScanDynAStar, SearchReuseDoesNotHideAnAdmissibleDetour) {
  using namespace nav_kernel::local::scan;
  using namespace upstream;
  lingtu::nav::tests::CollisionBitmap bitmap({.8, 3., .45}, {3.3, 5.5, .5}, .05);
  // Recorded inflated post slice: 5 cm cells at Z = 0.475 m.
  const std::uint64_t rows[] = {
      0x0ULL, 0x0ULL, 0x0ULL, 0x0ULL, 0x0ULL,
      0x0ULL, 0x0ULL, 0x0ULL, 0x0ULL, 0x0ULL,
      0x0ULL, 0x0ULL, 0x0ULL, 0x3f80000ULL, 0x3fffffffe0000ULL,
      0x3ffffffff0000ULL, 0x3ffffffff8000ULL, 0x3ffffffff8000ULL, 0x3ffffffffc000ULL, 0x3ffffffffc000ULL,
      0x3ffffffffc000ULL, 0x3ffffffffc000ULL, 0x3ffffffffc000ULL, 0x3ffffffffc000ULL, 0x3ffffffffc000ULL,
      0x3ffffffffc000ULL, 0x3ffffffffc000ULL, 0x3ffffffff8000ULL, 0x3ffffffff8000ULL, 0x3ffffffff0000ULL,
      0x3fffffffc0000ULL, 0x3ffffc0000000ULL, 0x3ffffc0000000ULL, 0x3ffffc0000000ULL, 0x3ffffe0000000ULL,
      0x3ffffe0000000ULL, 0x3ffffe0000000ULL, 0x3ffffe0000000ULL, 0x3ffffe0000000ULL, 0x3ffffe0000000ULL,
      0x3ffffe0000000ULL, 0x3ffffc0000000ULL, 0x3ffffc0000000ULL, 0x3ffffc0000000ULL, 0x3ffffc0000000ULL,
      0x3ffffc0000000ULL, 0x3ffffc0000000ULL, 0x3ffffc0000000ULL, 0x3ffffc0000000ULL, 0x3ffffc0000000ULL};
  for (int y = 0; y < 50; ++y) {
    for (int x = 0; x < 50; ++x) {
      if (rows[y] & (1ULL << x))
        bitmap.occupy({.8 + (x+.5)*.05, 3. + (y+.5)*.05, .475});
    }
  }
  LocalPlannerParams params;
  params.scan.voxelResolution = .05;
  params.scan.cylinderOffset = .25;
  LocalPlanRequest request;
  request.environment.collision = bitmap.view();
  Grid grid(params, request);
  auto adapter = std::make_shared<GridMap>(grid);
  AStar reused;
  reused.initGridMap(adapter, {80, 80, 20});
  const Eigen::Vector3d points[] = {
      {1.90642277, 3.49869806, .47970177}, {1.56521229, 3.83470721, .48098684},
      {1.42603009, 4.18632639, .48381457}, {1.67962456, 4.5347187, .48430411},
      {1.8552175, 4.70102441, .48731245}};
  const int segments[][2] = {{0, 1}, {1, 2}, {3, 4}};
  for (const auto &segment : segments) {
    const auto &start = points[segment[0]], &end = points[segment[1]];
    AStar fresh;
    fresh.initGridMap(adapter, {80, 80, 20});
    ASSERT_EQ(fresh.search(.05, start, end), AStarResult::Success);
    ASSERT_EQ(reused.search(.05, start, end), AStarResult::Success);
    const auto expected = fresh.path(), actual = reused.path();
    ASSERT_EQ(actual.size(), expected.size()) << segment[0];
    for (std::size_t i = 0; i < actual.size(); ++i)
      EXPECT_LT((actual[i] - expected[i]).norm(), 1e-10) << segment[0] << ": " << i;
  }
}

TEST(ScanDynAStar, DoesNotPublishPartialProgressPath) {
  RequestFixture fixture({{-1.0, 0.0, 0.5}, {1.0, 0.0, 0.5}});
  for (int x = -5; x <= 5; ++x) {
    for (int y = -50; y <= 50; ++y)
      fixture.bitmap.occupy(
          {0.1 * static_cast<double>(x), 0.1 * static_cast<double>(y), 0.5});
  }
  fixture.refreshCollision(2);
  ASSERT_TRUE(fixture.request.environment.collision.occupied({0.0, 0.1, 0.5}));
  nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
  ASSERT_TRUE(grid.valid()) << grid.reason();

  auto map = std::make_shared<nav_kernel::local::scan::upstream::GridMap>(grid);
  ASSERT_EQ(map->getInflateOccupancy({0.0, 0.0, 0.5}, 0.0), 1);
  ASSERT_EQ(map->getInflateOccupancy({0.0, 0.1, 0.5}, 0.0), 1);
  ASSERT_EQ(map->getInflateOccupancy({0.0, 1.9, 0.5}, 0.0), 1);
  nav_kernel::local::scan::upstream::AStar search;
  search.initGridMap(map, {40, 40, 20});

  const auto result = search.search(0.1, {-1.0, 0.0, 0.5}, {1.0, 0.0, 0.5});
  const auto blockedPath = search.path();
  EXPECT_EQ(result, nav_kernel::local::scan::upstream::AStarResult::SearchError);
  EXPECT_TRUE(blockedPath.empty());
}

TEST(ScanBackend, EmitsOfficialBsplineAfterFsmTransitions) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());

  LocalPlan plan;
  for (int tick = 0; tick < 8 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS = 1.0 + 0.01 * tick;
    fixture.refreshCollision(1);
    plan = backend.tick(fixture.request);
  }

  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const auto *spline = std::get_if<SplineTarget>(&plan.target());
  ASSERT_NE(spline, nullptr);
  EXPECT_EQ(spline->order, 3);
  EXPECT_GT(spline->trajectoryId, 0);
  EXPECT_GT(spline->startTimeS, fixture.request.clock.timestampS);
  EXPECT_GE(spline->controls.size(), 4U);
  EXPECT_EQ(spline->knots.size(),
            spline->controls.size() + static_cast<std::size_t>(spline->order) + 1U);
  const auto &preview = plan.previewPath();
  ASSERT_FALSE(preview.empty());
  EXPECT_EQ(preview.data(), plan.previewPath().data());
  EXPECT_NEAR(preview.back().z, 0.5, 1e-6);
}

TEST(ScanBackend, StairBodyHeadingEntranceReportsPlanningCost) {
  for (bool fixedHeading : {false, true}) {
    RequestFixture fixture({{-.774514, -.004625, .452461},
                             {2.692723, -.000676, .719014}}, .05);
    auto params = scanParams();
    params.scan.voxelResolution = .05;
    params.scan.cylinderOffset = .25;
    params.scan.cylinderRadius = .4;
    params.scan.planningHorizon = 5.0;
    fixture.request.robot.pose.yaw = .5;
    fixture.request.robot.kinematics.valid = true;
    fixture.request.maxLinearSpeedMps = .15;
    if (fixedHeading) fixture.request.bodyHeading = 0.0;
    nav_kernel::local::scan::Backend backend(params);
    LocalPlan plan;
    double maxPlanningMs = 0.0, maxGridMs = 0.0;
    for (int tick = 0; tick < 12 && !plan.ready(); ++tick) {
      fixture.request.clock.timestampS = 1.0 + .01 * tick;
      plan = backend.tick(fixture.request);
      maxPlanningMs = std::max(maxPlanningMs, backend.debugSnapshot().planningTimeMs);
      maxGridMs = std::max(maxGridMs, backend.debugSnapshot().gridTimeMs);
    }
    ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
    std::cout << "[heading-cost] fixed=" << fixedHeading << " yaw=.5 speed=.15"
              << " distance=" << nav_kernel::distance3D(fixture.route.front(), fixture.route.back())
              << " max_planning_ms=" << maxPlanningMs << " max_grid_ms=" << maxGridMs << '\n';
    nav_kernel::local::scan::Grid grid(params, fixture.request);
    const auto& path = plan.previewPath();
    for (std::size_t i = 1; i < path.size(); ++i)
      EXPECT_EQ(grid.segmentInflatedOccupancy(path[i-1], 0.0, path[i], 0.0), 0);
  }
}

TEST(ScanBackend, KeepsCommittedTrajectoryWhileOfficialFsmReplans) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());

  LocalPlan ready;
  for (int tick = 0; tick < 8 && !ready.ready(); ++tick) {
    fixture.request.clock.timestampS = 1.0 + 0.01 * tick;
    ready = backend.tick(fixture.request);
  }
  ASSERT_TRUE(ready.ready());
  const auto firstId = std::get<SplineTarget>(ready.target()).trajectoryId;

  fixture.request.clock.timestampS += 0.01;
  fixture.refreshCollision(2);
  const LocalPlan retained = backend.tick(fixture.request);

  ASSERT_TRUE(retained.ready());
  EXPECT_GE(std::get<SplineTarget>(retained.target()).trajectoryId, firstId);
}

TEST(ScanBackend, CollisionTickReplacesOrStopsThePublishedSpline) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan ready;
  for (int tick = 0; tick < 8 && !ready.ready(); ++tick) {
    fixture.request.clock.timestampS = 1.0 + 0.01 * tick;
    ready = backend.tick(fixture.request);
  }
  ASSERT_TRUE(ready.ready());

  fixture.bitmap.occupyInflated({0.5F, 0.0F, 0.5F}, 0.4, 0.2, 0.2);
  fixture.request.clock.timestampS += 0.05;
  fixture.refreshCollision(2);
  const LocalPlan retained = backend.tick(fixture.request);
  ASSERT_TRUE(retained.ready());
  const auto retainedId =
      std::get<SplineTarget>(retained.target()).trajectoryId;
  const LocalPlan updated = backend.checkCollision(fixture.request);

  if (!updated.ready()) {
    EXPECT_TRUE(updated.status() == nav_kernel::LocalPlanStatus::NearFieldStop ||
                updated.status() == nav_kernel::LocalPlanStatus::Blocked);
    return;
  }
  ASSERT_TRUE(updated.ready());
  const auto updatedId =
      std::get<SplineTarget>(updated.target()).trajectoryId;
  EXPECT_GE(updatedId, retainedId);
  if (updatedId > retainedId) {
    nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
    ASSERT_TRUE(grid.valid()) << grid.reason();
    for (const Vec3 &point : updated.previewPath())
      EXPECT_TRUE(grid.obstacleFree(point, 0.0));
  }
}

TEST(ScanBackend, KeepsPublishedSplineUntilNewReferenceIsPlanned) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan ready;
  for (int tick = 0; tick < 8 && !ready.ready(); ++tick) {
    fixture.request.clock.timestampS = 1.0 + 0.01 * tick;
    ready = backend.tick(fixture.request);
  }
  ASSERT_TRUE(ready.ready());
  ASSERT_TRUE(backend.debugSnapshot().localTargetValid);
  EXPECT_GT(backend.debugSnapshot().localTarget.x, 0.0);
  const auto previousId = std::get<SplineTarget>(ready.target()).trajectoryId;

  fixture.route = {{0.0, 0.0, 0.5}, {-2.0, 0.0, 0.5}};
  fixture.request.objective = RouteTarget{{
      fixture.route.data(), static_cast<int>(fixture.route.size()), 2}};
  fixture.request.clock.timestampS += 0.01;
  const LocalPlan changed = backend.tick(fixture.request);

  ASSERT_TRUE(changed.ready());
  EXPECT_FALSE(backend.debugSnapshot().localTargetValid)
      << "a newly accepted reference has no generated local target yet";
  EXPECT_EQ(std::get<SplineTarget>(changed.target()).trajectoryId, previousId);

  LocalPlan replacement = changed;
  for (int tick = 0; tick < 8 &&
                     std::get<SplineTarget>(replacement.target()).trajectoryId == previousId;
       ++tick) {
    fixture.request.clock.timestampS += 0.01;
    replacement = backend.tick(fixture.request);
  }
  ASSERT_TRUE(replacement.ready());
  EXPECT_GT(std::get<SplineTarget>(replacement.target()).trajectoryId, previousId);
}

TEST(ScanBackend, AcceptsChangedReferenceShapeWithANewGeneration) {
  RequestFixture fixture(
      {{0.0, 0.0, 0.5}, {1.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan ready;
  for (int tick = 0; tick < 8 && !ready.ready(); ++tick) {
    fixture.request.clock.timestampS = 1.0 + 0.01 * tick;
    ready = backend.tick(fixture.request);
  }
  ASSERT_TRUE(ready.ready());
  const auto previousId = std::get<SplineTarget>(ready.target()).trajectoryId;

  fixture.route[1] = {1.0, 0.8, 0.5};
  fixture.request.objective = RouteTarget{{
      fixture.route.data(), static_cast<int>(fixture.route.size()), 2}};
  fixture.request.clock.timestampS += 0.01;
  LocalPlan replacement = backend.tick(fixture.request);
  for (int tick = 0; tick < 8 &&
                     std::get<SplineTarget>(replacement.target()).trajectoryId == previousId;
       ++tick) {
    fixture.request.clock.timestampS += 0.01;
    replacement = backend.tick(fixture.request);
  }

  ASSERT_TRUE(replacement.ready());
  EXPECT_GT(std::get<SplineTarget>(replacement.target()).trajectoryId, previousId);
}

TEST(ScanReplanFsm, HeldMotionIntentRecoversAfterFailureLimitWithoutAnotherReference) {
  using namespace nav_kernel::local::scan::upstream;
  for (const bool motion_intent : {false, true}) {
    SCOPED_TRACE(motion_intent ? "held motion intent" : "route");
    ScanAttemptFixture fixture(motion_intent);
    for (double x = -0.25; x < 0.3; x += 0.1)
      for (double y = -0.25; y < 0.3; y += 0.1)
        for (double z = 0.25; z < 0.8; z += 0.1)
          fixture.request.bitmap.occupy({x, y, z});

    ScanReplanParams params;
    params.navigationMode = ScanNavigationMode::REFERENCE_PATH;
    params.planningHorizon = 3.5;
    params.noReplanThreshold = 0.1;
    params.replanThreshold = 1.0;
    params.maxReplanFailCount = 1;
    PlannerFsm fsm(fixture.manager, params);
    FsmInput input;
    input.nowS = 1.0;
    input.executionFrozen = true;
    input.odometry = FsmOdometry{};
    input.odometry->position = {0.0, 0.0, 0.5};
    input.referencePath = std::vector<Eigen::Vector3d>{
        {0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}};
    ASSERT_TRUE(fsm.tick(input).targetAccepted);
    input.referencePath.reset();
    const auto tick = [&] {
      input.nowS += PlannerFsm::kTickPeriodS;
      return fsm.tick(input);
    };
    FsmOutput output;
    for (int i = 0; i < 5 && fsm.state() != ScanReplanState::EMERGENCY_STOP; ++i)
      output = tick();
    ASSERT_EQ(fsm.state(), ScanReplanState::EMERGENCY_STOP);
    ASSERT_EQ(fixture.manager.reboundDebug().reason, "collision_at_trajectory_start");
    EXPECT_FALSE(output.trajectory.has_value());
    ASSERT_TRUE(tick().emergencyStopIssued);

    fixture.request.bitmap.clear();
    fixture.request.refreshCollision(2);
    fixture.grid = nav_kernel::local::scan::Grid(scanParams(), fixture.request.request);
    // Upstream waits for the full measured 3-D velocity to settle.
    input.odometry->velocity = {0.2, 0.0, 0.12};
    EXPECT_EQ(tick().state, ScanReplanState::EMERGENCY_STOP);
    EXPECT_TRUE(fsm.hasTarget());
    input.odometry->velocity = {0.0, 0.0, std::numeric_limits<double>::quiet_NaN()};
    EXPECT_EQ(tick().state, ScanReplanState::EMERGENCY_STOP);
    input.odometry->velocity = {0.0, 0.0, 0.12};
    EXPECT_EQ(tick().state, ScanReplanState::EMERGENCY_STOP);
    input.odometry->velocity.setZero();
    output = tick();
    if (!motion_intent) {
      EXPECT_EQ(output.state, ScanReplanState::WAIT_TARGET);
      EXPECT_FALSE(fsm.hasTarget());
      continue;
    }

    ASSERT_EQ(output.state, ScanReplanState::GEN_NEW_TRAJ);
    ASSERT_TRUE(fsm.hasTarget());
    output = tick();
    ASSERT_EQ(output.state, ScanReplanState::EXEC_TRAJ)
        << fixture.manager.reboundDebug().reason;
    ASSERT_TRUE(output.trajectory.has_value());
    EXPECT_TRUE(fixture.manager.reboundDebug().success);
    EXPECT_LE((fixture.manager.local_data_.position_traj_.evaluateDeBoorT(0.0) -
               input.odometry->position).norm(), 1e-9);
  }
}

TEST(ScanReplanFsm, CurvedReferenceTargetUsesItsOwnTangentInsteadOfTheStartChord) {
  using namespace nav_kernel::local::scan::upstream;
  namespace geometry = lingtu::nav::collision;
  for (const bool stoppedEndpoint : {false, true}) {
    SCOPED_TRACE(stoppedEndpoint);
    ScanAttemptFixture fixture;
    auto body = std::make_shared<geometry::Body>();
    body->margin_m = body->horizontal_margin_m = 0;
    geometry::Part part;
    part.link = "long_body";
    part.half_size = {.4, .08, .08};
    body->parts.push_back(part);
    fixture.request.request.body = body;
    fixture.request.request.bodyHeading.reset();
    fixture.request.request.robot.pose.yaw = M_PI / 2;
    fixture.request.bitmap.occupy({1.25, 1.25, .55});
    fixture.request.refreshCollision(2);
    auto &collision = fixture.request.request.environment.collision;
    collision.measuredOccupiedStorage = std::make_shared<const std::vector<std::uint8_t>>(
        collision.inflatedBits, collision.inflatedBits + collision.inflatedBytes);
    fixture.grid = nav_kernel::local::scan::Grid(scanParams(), fixture.request.request);
    ASSERT_TRUE(fixture.grid.valid()) << fixture.grid.reason();
    ASSERT_FALSE(fixture.request.request.bodyHeading);
    ASSERT_EQ(fixture.grid.inflatedOccupancy({1, 1, .5}, 0), 0);
    ASSERT_EQ(fixture.grid.inflatedOccupancy({1, 1, .5}, M_PI / 4), 1);

    ScanReplanParams params;
    params.navigationMode = ScanNavigationMode::REFERENCE_PATH;
    params.planningHorizon = 3.5;
    params.noReplanThreshold = .1;
    params.replanThreshold = 1;
    PlannerFsm fsm(fixture.manager, params);
    FsmInput input;
    input.nowS = 1;
    input.odometry = FsmOdometry{};
    input.odometry->position = {0, 0, .5};
    input.odometry->orientation = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
    input.referencePath = std::vector<Eigen::Vector3d>{{0, 0, .5}, {0, 1, .5}, {1, 1, .5}};
    ASSERT_TRUE(fsm.tick(input).targetAccepted);
    input.referencePath.reset();
    // Give the target an unambiguous +X incoming tangent, also at zero terminal speed.
    PolynomialTraj reference;
    reference.addSegment({0, 0, 0, 0}, {-2, 3, 0, 0}, {0, 0, 0, .5}, 1);
    reference.addSegment(stoppedEndpoint ? std::vector<double>{-2, 3, 0, 0}
                                        : std::vector<double>{0, 0, 1, 0},
                         {0, 0, 0, 1}, {0, 0, 0, .5}, 1);
    fixture.manager.global_data_.setGlobalTraj(reference, input.nowS);
    FsmOutput output;
    for (int tick = 0; tick < 5 && !fixture.manager.reboundDebug().attempted; ++tick) {
      input.nowS += PlannerFsm::kTickPeriodS;
      output = fsm.tick(input);
    }
    ASSERT_TRUE(fixture.manager.reboundDebug().attempted);
    EXPECT_FALSE(output.localTargetBlocked);
    EXPECT_LT((fixture.manager.reboundDebug().targetPosition - Eigen::Vector3d(1, 1, .5)).norm(),
              1e-9);
  }
}

TEST(ScanReplanFsm, LongStraightReferenceDoesNotAnticipateDistantBends) {
  using namespace nav_kernel::local::scan::upstream;
  namespace collision_geometry = lingtu::nav::collision;
  ScanAttemptFixture fixture;
  auto body = std::make_shared<collision_geometry::Body>();
  body->margin_m = 0;
  collision_geometry::Part part;
  part.link = "navigation_body";
  part.center.z = .07;
  part.half_size = {.48,.255,.17};
  body->parts.push_back(part);
  fixture.request.request.body = body;
  fixture.request.request.bodyHeading = 0;
  auto &collision = fixture.request.request.environment.collision;
  collision.measuredOccupiedStorage =
      std::make_shared<const std::vector<std::uint8_t>>(collision.inflatedBytes,0U);
  collision.knownFreeStorage =
      std::make_shared<const std::vector<std::uint8_t>>(collision.inflatedBytes,255U);
  fixture.grid = nav_kernel::local::scan::Grid(scanParams(),fixture.request.request);
  fixture.manager.pp_.preserve_boundary_ = true;
  ScanReplanParams params;
  params.navigationMode = ScanNavigationMode::REFERENCE_PATH;
  params.planningHorizon = 3.5;
  params.noReplanThreshold = .1;
  params.replanThreshold = .2;
  PlannerFsm fsm(fixture.manager,params);
  FsmInput input;
  input.nowS = 1;
  input.odometry = FsmOdometry{};
  input.odometry->position = {0,0,.5};
  input.referencePath = std::vector<Eigen::Vector3d>{
      {0,0,.5},{8,0,.5},{10,2,.5},{12,2,.5}};
  ASSERT_TRUE(fsm.tick(input).targetAccepted);
  input.referencePath.reset();
  FsmOutput output;
  for (int tick = 0; tick < 8 && !output.trajectory; ++tick) {
    input.nowS += PlannerFsm::kTickPeriodS;
    output = fsm.tick(input);
  }
  ASSERT_TRUE(output.trajectory) << fixture.manager.reboundDebug().reason;
  EXPECT_GT(fixture.manager.reboundDebug().targetPosition.x(),3);
  EXPECT_LT(std::abs(fixture.manager.reboundDebug().targetPosition.y()),.001);
  const auto &local = fixture.manager.local_data_;
  for (double time = 0; time < local.duration_; time += .01) {
    const auto point = local.position_traj_.evaluateDeBoorT(time);
    EXPECT_LT(std::abs(point.y()),.005) << "time=" << time << " x=" << point.x();
    EXPECT_NEAR(point.z(),.5,1e-8);
  }
}

TEST(ScanReplanFsm, SubmillimeterStartConnectorKeepsTheMeasuredDetourForward) {
  using namespace nav_kernel::local::scan::upstream;
  ScanAttemptFixture fixture;
  fixture.manager.pp_.max_vel_=1;
  fixture.manager.pp_.max_acc_=2;
  ScanReplanParams params;
  params.navigationMode=ScanNavigationMode::REFERENCE_PATH;
  params.planningHorizon=3.5;
  PlannerFsm fsm(fixture.manager,params);
  // Recorded final-clean reference, including the real 0.164 mm start connector.
  const std::vector<Eigen::Vector3d> path{
      {1.5033557708314125, 0.0024403008390908765, 0.4411855611030417},
      {1.5033557708314125, 0.0026039814110845327, 0.4411855611030417},
      {1.6031277179718018, 0.0026039814110845327, 0.44156184792518616},
      {1.7031277418136597, 0.0026039814110845327, 0.44156184792518616},
      {1.8031277656555176, 0.0026039814110845327, 0.44156184792518616},
      {1.9031277894973755, 0.0026039814110845327, 0.44156184792518616},
      {2.0031278133392334, 0.0026039814110845327, 0.44156184792518616},
      {2.1031277179718018, 0.0026039814110845327, 0.44156184792518616},
      {2.203127861022949, -0.09739601612091064, 0.44156184792518616},
      {2.3031277656555176, -0.19739602506160736, 0.44156184792518616},
      {2.403127670288086, -0.2973960041999817, 0.44156184792518616},
      {2.5031278133392334, -0.2973960041999817, 0.44156184792518616},
      {2.6031277179718018, -0.3973960280418396, 0.44156184792518616},
      {2.703127861022949, -0.4973960220813751, 0.44156184792518616},
      {2.8031277656555176, -0.5973960161209106, 0.44156184792518616},
      {2.903127670288086, -0.6973960399627686, 0.44156184792518616},
      {3.0031278133392334, -0.6973960399627686, 0.44156184792518616},
      {3.1031277179718018, -0.6973960399627686, 0.44156184792518616},
      {3.203127861022949, -0.6973960399627686, 0.44156184792518616},
      {3.3031277656555176, -0.6973960399627686, 0.44156184792518616},
      {3.403127670288086, -0.6973960399627686, 0.44156184792518616},
      {3.5031278133392334, -0.6973960399627686, 0.44156184792518616},
      {3.6031277179718018, -0.6973960399627686, 0.44156184792518616},
      {3.703127861022949, -0.5973960161209106, 0.44156184792518616},
      {3.8031277656555176, -0.5973960161209106, 0.44156184792518616},
      {3.903127670288086, -0.4973960220813751, 0.44156184792518616},
      {4.003127574920654, -0.4973960220813751, 0.44156184792518616},
      {4.103127956390381, -0.4973960220813751, 0.44156184792518616},
      {4.122527803141947, -0.4779961579849783, 0.44156184792518616},
      {4.203127861022949, -0.3973960280418396, 0.44156184792518616},
      {4.303127765655518, -0.3973960280418396, 0.44156184792518616},
      {4.403127670288086, -0.2973960041999817, 0.44156184792518616},
      {4.503127574920654, -0.2973960041999817, 0.44156184792518616},
      {4.603127956390381, -0.19739602506160736, 0.44156184792518616},
      {4.703127861022949, -0.19739602506160736, 0.44156184792518616},
      {4.803127765655518, -0.09739601612091064, 0.44156184792518616},
      {4.903127670288086, -0.09739601612091064, 0.44156184792518616},
      {5.003127574920654, 0.0026039814110845327, 0.44156184792518616},
      {5.103127956390381, 0.0026039814110845327, 0.44156184792518616}};
  FsmInput input;
  input.nowS=1;
  input.odometry=FsmOdometry{};
  input.odometry->position=path.front();
  input.referencePath=path;
  ASSERT_TRUE(fsm.tick(input).targetAccepted);
  auto &reference=fixture.manager.global_data_;
  Eigen::Vector3d minimum=path.front(),maximum=path.front();
  for (double time=0;time<=reference.global_duration_;time+=.001) {
    const auto point=reference.getPosition(time);
    minimum=minimum.cwiseMin(point);
    maximum=maximum.cwiseMax(point);
  }
  EXPECT_GT(minimum.x(),1.4);
  EXPECT_LT(maximum.x(),5.2);
  EXPECT_GT(minimum.y(),-.8);
  EXPECT_LT(maximum.y(),.1);
  EXPECT_GT(minimum.z(),.43);
  EXPECT_LT(maximum.z(),.46);
  RecordProperty("minimum_y_m",std::to_string(minimum.y()));
  RecordProperty("maximum_y_m",std::to_string(maximum.y()));
  RecordProperty("first_interval_s",std::to_string(reference.global_traj_.getTimes().front()));
  EXPECT_GE(reference.global_traj_.getTimes().front(),
      std::sqrt(2*(path[1]-path[0]).norm()/fixture.manager.pp_.max_acc_)-1e-12);
  input.referencePath.reset();
  for (int tick=0;tick<3;++tick) {
    input.nowS+=.01;
    (void)fsm.tick(input);
  }
  const auto &attempt=fixture.manager.reboundDebug();
  EXPECT_GT(attempt.targetPosition.x(),path.front().x()+1);
  EXPECT_LT(attempt.targetPosition.y(),-.1);
  double routeDistance=std::numeric_limits<double>::infinity();
  for (std::size_t i=1;i<path.size();++i) {
    const Eigen::Vector3d edge=path[i]-path[i-1];
    const double fraction=std::clamp((attempt.targetPosition-path[i-1]).dot(edge)/edge.squaredNorm(),0.0,1.0);
    routeDistance=std::min(routeDistance,(attempt.targetPosition-path[i-1]-fraction*edge).norm());
  }
  EXPECT_LT(routeDistance,.05);
  RecordProperty("target_x_m",std::to_string(attempt.targetPosition.x()));
  RecordProperty("target_y_m",std::to_string(attempt.targetPosition.y()));
}

TEST(ScanReplanFsm, ReferenceFitRetainsShortBendsAndThreeDimensionalHeights) {
  using namespace nav_kernel::local::scan::upstream;
  ScanAttemptFixture fixture;
  ScanReplanParams params;
  params.navigationMode = ScanNavigationMode::REFERENCE_PATH;
  PlannerFsm fsm(fixture.manager,params);
  const std::vector<Eigen::Vector3d> path{
      {0,0,.5},{.2,0,.5},{.2,.2,.7},{.4,.2,.7},{.4,.4,.9},{2,.4,.9}};
  FsmInput input;
  input.nowS = 1;
  input.odometry = FsmOdometry{};
  input.odometry->position = path.front();
  input.referencePath = path;
  ASSERT_TRUE(fsm.tick(input).targetAccepted);
  auto &reference = fixture.manager.global_data_.global_traj_;
  std::vector<Eigen::Vector3d> knots{reference.evaluate(0)};
  double time = 0;
  for (const double duration : reference.getTimes()) {
    time += duration;
    knots.push_back(reference.evaluate(time));
  }
  for (const auto &expected : path) {
    EXPECT_TRUE(std::any_of(knots.begin(),knots.end(),[&](const Eigen::Vector3d &point) {
      return (point - expected).norm() < 1e-7;
    })) << "missing original 3-D bend " << expected.transpose();
  }
}

TEST(ScanReplanFsm, OccupiedReferenceTailStillAllowsAShortClearForwardPrefix) {
  using namespace nav_kernel::local::scan::upstream;
  ScanAttemptFixture fixture;
  for (double x=.25;x<2.1;x+=.1)
    fixture.request.bitmap.occupy({x,.05,.55});
  fixture.request.refreshCollision(2);
  fixture.grid=nav_kernel::local::scan::Grid(scanParams(),fixture.request.request);
  ASSERT_EQ(fixture.grid.inflatedOccupancy({2,0,.5},0),1);
  ASSERT_EQ(fixture.grid.segmentInflatedOccupancy({0,0,.5},0,{.15,0,.5},0),0);
  ScanReplanParams params;
  params.navigationMode=ScanNavigationMode::REFERENCE_PATH;
  params.planningHorizon=3.5;
  params.noReplanThreshold=.01;
  params.replanThreshold=.2;
  PlannerFsm fsm(fixture.manager,params);
  FsmInput input;
  input.nowS=1;
  input.odometry=FsmOdometry{};
  input.odometry->position={0,0,.5};
  input.referencePath=std::vector<Eigen::Vector3d>{{0,0,.5},{2,0,.5}};
  ASSERT_TRUE(fsm.tick(input).targetAccepted);
  input.referencePath.reset();
  FsmOutput output;
  for (int tick=0;tick<8 && !output.trajectory;++tick) {
    input.nowS+=PlannerFsm::kTickPeriodS;
    output=fsm.tick(input);
  }
  ASSERT_TRUE(output.trajectory)<<fixture.manager.reboundDebug().reason;
  const auto& attempt=fixture.manager.reboundDebug();
  EXPECT_GT(attempt.targetPosition.x(),.001);
  EXPECT_LT(attempt.targetPosition.x(),.2);
  EXPECT_EQ(fixture.grid.segmentInflatedOccupancy({0,0,.5},0,
      {attempt.targetPosition.x(),attempt.targetPosition.y(),attempt.targetPosition.z()},0),0);
  const auto& local=fixture.manager.local_data_;
  for (double t=0;t<=local.duration_;t+=.01) {
    const auto point=local.position_traj_.evaluateDeBoorT(t);
    EXPECT_EQ(fixture.grid.inflatedOccupancy({point.x(),point.y(),point.z()},0),0);
  }
}

TEST(ScanReplanFsm, FutureCollisionKeepsNorthHeadingThroughItsQuietTail) {
  using namespace nav_kernel::local::scan::upstream;
  namespace geometry = lingtu::nav::collision;
  ScanAttemptFixture fixture;
  ScanReplanParams params;
  params.navigationMode = ScanNavigationMode::REFERENCE_PATH;
  params.planningHorizon = 3.5;
  params.noReplanThreshold = .1;
  params.replanThreshold = 1;
  PlannerFsm fsm(fixture.manager, params);
  FsmInput input;
  input.nowS = 1;
  input.odometry = FsmOdometry{};
  input.odometry->position = {0, -4, .5};
  input.referencePath = std::vector<Eigen::Vector3d>{{0, -4, .5}, {0, 2.01, .5}};
  ASSERT_TRUE(fsm.tick(input).targetAccepted);
  input.referencePath.reset();
  FsmOutput output;
  for (int tick = 0; tick < 8 && !output.trajectory; ++tick) {
    input.nowS += .01;
    output = fsm.tick(input);
  }
  ASSERT_TRUE(output.trajectory);

  Eigen::MatrixXd controls(3, 9);
  controls.row(0).setZero();
  controls.row(1) << -6, -4, -2, -1, 0, 2, 2.005, 2.01, 2.015;
  controls.row(2).setConstant(.5);
  auto &active = fixture.manager.local_data_;
  active.position_traj_ = UniformBspline(controls, 3, 1);
  active.velocity_traj_ = active.position_traj_.getDerivative();
  active.acceleration_traj_ = active.velocity_traj_.getDerivative();
  active.start_time_ = 10;
  active.start_pos_ = active.position_traj_.evaluateDeBoorT(0);
  active.duration_ = active.position_traj_.getTimeSum();
  input.nowS = 14;
  input.odometry->position = active.position_traj_.evaluateDeBoorT(4);
  input.odometry->velocity = active.velocity_traj_.evaluateDeBoorT(4);
  ASSERT_GT(input.odometry->velocity.head<2>().norm(), .03);
  ASSERT_LT(active.velocity_traj_.evaluateDeBoorT(active.duration_).head<2>().norm(), .03);

  auto body = std::make_shared<geometry::Body>();
  body->margin_m = body->horizontal_margin_m = 0;
  geometry::Part part;
  part.link = "long_body";
  part.half_size = {.2, .04, .04};
  body->parts.push_back(part);
  fixture.request.request.body = body;
  fixture.request.request.robot.pose = {{input.odometry->position.x(), input.odometry->position.y(), .5}, 0};
  fixture.request.bitmap.occupy({.125, 2.025, .55});
  const auto refresh = [&] {
    fixture.request.refreshCollision(2);
    auto &collision = fixture.request.request.environment.collision;
    collision.measuredOccupiedStorage = std::make_shared<const std::vector<std::uint8_t>>(
        collision.inflatedBits, collision.inflatedBits + collision.inflatedBytes);
    fixture.grid = nav_kernel::local::scan::Grid(scanParams(), fixture.request.request);
  };
  refresh();
  ASSERT_EQ(fixture.grid.inflatedOccupancy({0, 2.01, .5}, M_PI / 2), 0);
  ASSERT_EQ(fixture.grid.inflatedOccupancy({0, 2.01, .5}, 0), 1);
  EXPECT_FALSE(fsm.checkFutureCollision(input).collisionDetected);

  fixture.request.bitmap.occupy({.025, input.odometry->position.y() + .025, .55});
  refresh();
  output = fsm.checkFutureCollision(input);
  EXPECT_TRUE(output.collisionDetected);
  EXPECT_EQ(output.state, ScanReplanState::EMERGENCY_STOP);
}

TEST(ScanReplanFsm, DelayedHandoffMatchesExecutingPositionVelocityAndAcceleration) {
  using namespace nav_kernel::local::scan::upstream;
  for (const int handoffCase : {0, 1, 2, 3, 4, 5, 6}) {
    SCOPED_TRACE(handoffCase);
    const bool missesDeadline = handoffCase == 1;
    ScanAttemptFixture fixture;
    ScanReplanParams params;
    params.navigationMode = ScanNavigationMode::REFERENCE_PATH;
    params.planningHorizon = 3.5;
    params.noReplanThreshold = 0.1;
    params.replanThreshold = 0.2;
    PlannerFsm fsm(fixture.manager, params);
    FsmInput input;
    input.nowS = 1.0;
    input.odometry = FsmOdometry{};
    input.odometry->position = {0.0, 0.0, 0.5};
    input.referencePath = std::vector<Eigen::Vector3d>{
        {0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}};
    ASSERT_TRUE(fsm.tick(input).targetAccepted);
    input.referencePath.reset();
    FsmOutput output;
    for (int tick = 0; tick < 8 && !output.trajectory; ++tick) {
      input.nowS += PlannerFsm::kTickPeriodS;
      output = fsm.tick(input);
    }
    ASSERT_TRUE(output.trajectory);

    // The old curve accelerates while the replacement optimizer runs. The
    // former snapshot-plus-age handoff compared two different accelerations.
    Eigen::MatrixXd controls(3, 7);
    for (int i = 0; i < controls.cols(); ++i) {
      const double t = i - 1.0;
      controls.col(i) = Eigen::Vector3d(0.2 * t + 0.025 * t * t - 0.05 / 6.0,
                                       0.0, 0.5);
    }
    auto &active = fixture.manager.local_data_;
    active.position_traj_ = UniformBspline(controls, 3, 1.0);
    active.velocity_traj_ = active.position_traj_.getDerivative();
    active.acceleration_traj_ = active.velocity_traj_.getDerivative();
    active.start_time_ = 10.0;
    active.start_pos_ = active.position_traj_.evaluateDeBoorT(0.0);
    active.duration_ = active.position_traj_.getTimeSum();
    const LocalTrajData previous = active;
    input.nowS = 11.2;
    input.executingTrajectoryId = previous.traj_id_;
    input.executionStartTimeS = previous.start_time_;
    input.handoffLeadTimeS = 0.3;
    input.odometry->position = previous.position_traj_.evaluateDeBoorT(1.2);
    input.odometry->velocity = previous.velocity_traj_.evaluateDeBoorT(1.2);
    ASSERT_EQ(fsm.tick(input).state, ScanReplanState::REPLAN_TRAJ);
    input.nowS += 0.01;
    const double handoff = input.nowS + *input.handoffLeadTimeS;
    fixture.manager.setTimeSource([&] { return input.nowS + (missesDeadline ? 0.5 : 0.1); });
    output = fsm.tick(input);
    ASSERT_TRUE(fixture.manager.reboundDebug().success)
        << fixture.manager.reboundDebug().stage << ":" << fixture.manager.reboundDebug().reason;
    EXPECT_EQ(active.traj_id_, previous.traj_id_);
    EXPECT_DOUBLE_EQ(active.start_time_, previous.start_time_);
    if (missesDeadline) {
      EXPECT_TRUE(output.handoffMissed);
      EXPECT_FALSE(output.handoffPending);
      EXPECT_FALSE(output.trajectory);
      EXPECT_LE((active.position_traj_.getControlPoint() - controls).norm(), 1e-12);
      continue;
    }
    ASSERT_TRUE(output.handoffPending);
    ASSERT_TRUE(output.trajectory);
    const auto offer = output.handoff;
    ASSERT_TRUE(offer);
    fixture.manager.setTimeSource([&] { return input.nowS; });
    if (handoffCase == 2) {
      input.executionFrozen = true;
      input.nowS += 0.01;
      input.odometry->position = {0.25, 0.0, 0.5};
      input.odometry->velocity.setZero();
      output = fsm.tick(input);
      EXPECT_FALSE(output.handoffPending);
      EXPECT_EQ(offer->state.load(), TrajectoryHandoff::State::Withdrawn);
      ASSERT_TRUE(output.trajectory);
      EXPECT_LE((active.position_traj_.evaluateDeBoorT(0.0) -
                 input.odometry->position).norm(), 1e-3);
      continue;
    }
    if (handoffCase == 3) {
      // An obstacle observed after optimization must invalidate the pending
      // geometry before it can become the executing trajectory.
      fixture.request.bitmap.occupyInflated({1.8, 0.0, 0.5}, 0.15, 0.1, 0.1);
      fixture.request.refreshCollision(2);
      fixture.grid = nav_kernel::local::scan::Grid(scanParams(), fixture.request.request);
      input.nowS = handoff;
      output = fsm.checkFutureCollision(input);
      EXPECT_FALSE(output.trajectory);
      EXPECT_EQ(active.traj_id_, previous.traj_id_);
      EXPECT_EQ(output.state, ScanReplanState::REPLAN_TRAJ);
      EXPECT_FALSE(offer->receive());
      continue;
    }
    if (handoffCase == 4) {
      input.nowS = handoff + 0.1;
      output = fsm.tick(input);
      EXPECT_TRUE(output.handoffMissed);
      EXPECT_FALSE(output.trajectory);
      EXPECT_EQ(active.traj_id_, previous.traj_id_);
      EXPECT_FALSE(offer->receive());
      continue;
    }
    if (handoffCase == 5) {
      // The collision callback starts in time, but its geometry recheck runs
      // past the common reception deadline. It may not republish that offer.
      input.nowS = handoff - 0.01;
      double liveTime = input.nowS;
      fixture.manager.setTimeSource([&] {
        const double now = liveTime;
        liveTime += 0.04;
        return now;
      });
      output = fsm.checkFutureCollision(input);
      EXPECT_TRUE(output.handoffMissed);
      EXPECT_FALSE(output.trajectory);
      EXPECT_EQ(active.traj_id_, previous.traj_id_);
      EXPECT_FALSE(offer->receive());
      continue;
    }
    input.nowS = handoff - 0.01;
    EXPECT_TRUE(fsm.tick(input).handoffPending);
    EXPECT_EQ(active.traj_id_, previous.traj_id_);
    input.nowS = handoff + 0.01;
    fixture.manager.setTimeSource([&] { return input.nowS; });
    // A normally ordered collision timer may run just after handoff, before
    // the FSM observes reception. Only the remaining candidate has nonnegative
    // prediction times, and the empty map must keep the offer valid.
    output = fsm.checkFutureCollision(input);
    ASSERT_TRUE(output.handoff);
    EXPECT_EQ(offer->state.load(), TrajectoryHandoff::State::Offered);
    EXPECT_EQ(active.traj_id_, previous.traj_id_);
    ASSERT_TRUE(offer->receive());
    if (handoffCase == 6) input.nowS = handoff + 0.1;
    output = fsm.tick(input);
    ASSERT_TRUE(output.trajectory);
    EXPECT_GT(active.traj_id_, previous.traj_id_);
    EXPECT_DOUBLE_EQ(active.start_time_, handoff);
    const double elapsed = handoff - previous.start_time_;
    EXPECT_LE((active.position_traj_.evaluateDeBoorT(0.0) -
               previous.position_traj_.evaluateDeBoorT(elapsed)).norm(), 1e-9);
    EXPECT_LE((active.velocity_traj_.evaluateDeBoorT(0.0) -
               previous.velocity_traj_.evaluateDeBoorT(elapsed)).norm(), 1e-9);
    EXPECT_LE((active.acceleration_traj_.evaluateDeBoorT(0.0) -
               previous.acceleration_traj_.evaluateDeBoorT(elapsed)).norm(), 1e-9);
    // It is a new curve, not a replay or a relabeling of the old reference.
    const double future = std::min(1.0, active.duration_);
    EXPECT_GT((active.acceleration_traj_.evaluateDeBoorT(future) -
               previous.acceleration_traj_.evaluateDeBoorT(elapsed + future)).norm(), 1e-4);
  }
}

TEST(ScanReplanFsm, ReferenceReplanUsesMeasuredState) {
  using namespace nav_kernel::local::scan::upstream;
  for (const double observed_x : {0.0, 0.1}) {
    SCOPED_TRACE(observed_x);
    ScanAttemptFixture fixture;
    ScanReplanParams params;
    params.navigationMode = ScanNavigationMode::REFERENCE_PATH;
    params.planningHorizon = 3.5;
    params.noReplanThreshold = 0.1;
    params.replanThreshold = 0.2;
    PlannerFsm fsm(fixture.manager, params);
    FsmInput input;
    input.nowS = 1.0;
    input.odometry = FsmOdometry{};
    input.odometry->position = {0.0, 0.0, 0.5};
    input.referencePath = std::vector<Eigen::Vector3d>{
        {0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}};
    ASSERT_TRUE(fsm.tick(input).targetAccepted);
    input.referencePath.reset();
    FsmOutput output;
    for (int tick = 0; tick < 8 && !output.trajectory; ++tick) {
      input.nowS += PlannerFsm::kTickPeriodS;
      output = fsm.tick(input);
    }
    ASSERT_TRUE(output.trajectory);
    ASSERT_EQ(fsm.state(), ScanReplanState::EXEC_TRAJ);
    const auto previous_id = fixture.manager.local_data_.traj_id_;
    input.odometry->position.x() = observed_x;
    input.odometry->velocity = {0.05, 0.0, 0.0};
    // Select the same physical tracking error independently of the seed's
    // acceleration timing, then verify the replan starts from odometry.
    for (double time = 0.0; time < fixture.manager.local_data_.duration_; time += .01) {
      input.nowS = fixture.manager.local_data_.start_time_ + time;
      if ((fixture.manager.local_data_.position_traj_.evaluateDeBoorT(time) -
           input.odometry->position).norm() > .3)
        break;
    }
    const double elapsed = input.nowS - fixture.manager.local_data_.start_time_;
    const auto nominal = fixture.manager.local_data_.position_traj_.evaluateDeBoorT(elapsed);
    ASSERT_GT((nominal - input.odometry->position).norm(), 0.2);
    ASSERT_EQ(fsm.tick(input).state, ScanReplanState::REPLAN_TRAJ);
    input.nowS += PlannerFsm::kTickPeriodS;
    output = fsm.tick(input);
    ASSERT_TRUE(output.trajectory) << fixture.manager.reboundDebug().reason;
    ASSERT_GT(fixture.manager.local_data_.traj_id_, previous_id);
    EXPECT_LE((fixture.manager.reboundDebug().startPosition -
               input.odometry->position).norm(), 1e-9);
    // The upstream least-squares fit has sub-millimetre boundary residuals.
    EXPECT_LE((fixture.manager.local_data_.position_traj_.evaluateDeBoorT(0.0) -
               input.odometry->position).norm(), 1e-3);
    EXPECT_LE((fixture.manager.local_data_.velocity_traj_.evaluateDeBoorT(0.0) -
               input.odometry->velocity).norm(), 1e-3);
  }
}

TEST(ScanReplanFsm, HeldMotionIntentReplanUsesMeasuredStateWhenTrackingLags) {
  using namespace nav_kernel::local::scan::upstream;
  for (const double observed_x : {0.0, 0.1}) {
    SCOPED_TRACE(observed_x);
    ScanAttemptFixture fixture(true);
    ScanReplanParams params;
    params.navigationMode = ScanNavigationMode::REFERENCE_PATH;
    params.planningHorizon = 3.5;
    params.noReplanThreshold = 0.1;
    params.replanThreshold = 0.2;
    PlannerFsm fsm(fixture.manager, params);
    FsmInput input;
    input.nowS = 1.0;
    input.odometry = FsmOdometry{};
    input.odometry->position = {0.0, 0.0, 0.5};
    input.referencePath = std::vector<Eigen::Vector3d>{
        {0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}};
    ASSERT_TRUE(fsm.tick(input).targetAccepted);
    input.referencePath.reset();
    FsmOutput output;
    for (int tick = 0; tick < 8 && !output.trajectory; ++tick) {
      input.nowS += PlannerFsm::kTickPeriodS;
      output = fsm.tick(input);
    }
    ASSERT_TRUE(output.trajectory);
    ASSERT_EQ(fsm.state(), ScanReplanState::EXEC_TRAJ);
    const auto previous_id = fixture.manager.local_data_.traj_id_;
    input.odometry->position.x() = observed_x;
    input.odometry->velocity = {0.05, 0.0, 0.0};
    // The operator keeps the same intent while physical motion lags the clock.
    for (double time = 0.0; time < fixture.manager.local_data_.duration_; time += .01) {
      input.nowS = fixture.manager.local_data_.start_time_ + time;
      if ((fixture.manager.local_data_.position_traj_.evaluateDeBoorT(time) -
           input.odometry->position).norm() > .3)
        break;
    }
    const double elapsed = input.nowS - fixture.manager.local_data_.start_time_;
    ASSERT_GT((fixture.manager.local_data_.position_traj_.evaluateDeBoorT(elapsed) -
               input.odometry->position).norm(), .3);
    ASSERT_EQ(fsm.tick(input).state, ScanReplanState::REPLAN_TRAJ);
    input.nowS += PlannerFsm::kTickPeriodS;
    output = fsm.tick(input);
    ASSERT_TRUE(output.trajectory) << fixture.manager.reboundDebug().reason;
    ASSERT_GT(fixture.manager.local_data_.traj_id_, previous_id);
    EXPECT_LE((fixture.manager.reboundDebug().startPosition -
               input.odometry->position).norm(), 1e-9);
    EXPECT_LE((fixture.manager.local_data_.position_traj_.evaluateDeBoorT(0.0) -
               input.odometry->position).norm(), 1e-3);
    EXPECT_LE((fixture.manager.local_data_.velocity_traj_.evaluateDeBoorT(0.0) -
               input.odometry->velocity).norm(), 1e-3);
  }
}

TEST(ScanController, ForwardClockGapsAdvanceExecutionTime) {
  using namespace nav_kernel::local::scan::upstream;
  ScanAttemptFixture fixture;
  ASSERT_TRUE(fixture.replan());
  const auto &spline = fixture.manager.local_data_.position_traj_;
  const BsplineTrajectory trajectory{spline.getControlPoint(), spline.getKnot(), 3, 1, 10.0};
  for (const double gap : {0.3, 0.5}) {
    SCOPED_TRACE(gap);
    ClosedLoopController controller;
    ASSERT_TRUE(controller.setTrajectory(trajectory, 10.0));
    ClosedLoopControllerState state;
    state.position = spline.evaluateDeBoorT(0.0);
    state.nowS = 10.05;
    state.desiredHeading = 0.0;
    const auto moving = controller.step(state, ClosedLoopControllerParams{});
    state.nowS += gap;
    const auto delayed = controller.step(state, ClosedLoopControllerParams{});

    EXPECT_TRUE(controller.hasTrajectory());
    EXPECT_EQ(controller.trajectoryId(), trajectory.trajectoryId);
    EXPECT_FALSE(delayed.executionFrozen);
    EXPECT_NEAR(delayed.executionTimeS - moving.executionTimeS, gap, 1e-12);
    EXPECT_GT(delayed.command.vx, 0.0);
  }
}

TEST(ScanReplanFsm, ReferenceCallbackDoesNotAdvanceFrozenTrajectoryTime) {
  using nav_kernel::local::scan::upstream::FsmInput;
  using nav_kernel::local::scan::upstream::FsmOdometry;
  using nav_kernel::local::scan::upstream::SCANPlannerManager;
  using nav_kernel::local::scan::upstream::PlannerFsm;
  using nav_kernel::local::scan::upstream::ScanNavigationMode;
  using nav_kernel::local::scan::upstream::ScanReplanParams;

  SCANPlannerManager manager;
  manager.pp_.max_vel_ = 0.75;
  ScanReplanParams params;
  params.navigationMode = ScanNavigationMode::REFERENCE_PATH;
  params.bodyHeight = 0.4;
  PlannerFsm fsm(manager, params);

  FsmInput timer;
  timer.nowS = 9.95;
  timer.executionFrozen = true;
  timer.odometry = FsmOdometry{};
  (void)fsm.tick(timer);

  manager.local_data_.start_time_ = 5.0;
  FsmInput pathCallback;
  pathCallback.nowS = 10.0;
  pathCallback.executionFrozen = true;
  pathCallback.odometry = FsmOdometry{};
  pathCallback.referencePath = std::vector<Eigen::Vector3d>{
      {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  const auto output = fsm.tick(pathCallback);

  ASSERT_TRUE(output.targetAccepted);
  EXPECT_DOUBLE_EQ(manager.local_data_.start_time_, 5.0);
}

TEST(ScanReplanFsm, ExplicitFreezePreservesProgressAcrossLongTimerGaps) {
  using namespace nav_kernel::local::scan::upstream;
  for (const double gap : {0.1, 0.3, 0.5}) {
    SCOPED_TRACE(gap);
    SCANPlannerManager manager;
    PlannerFsm fsm(manager, ScanReplanParams{});
    FsmInput input;
    input.nowS = 10.0;
    input.odometry = FsmOdometry{};
    input.executionFrozen = true;
    (void)fsm.tick(input);
    manager.local_data_.start_time_ = 5.0;
    manager.local_data_.traj_id_ = 1;
    input.nowS += gap;
    (void)fsm.tick(input);
    EXPECT_NEAR(input.nowS - manager.local_data_.start_time_, 5.0, 1e-12);
    manager.local_data_.start_time_ = 0.0;
    input.nowS += gap;
    (void)fsm.tick(input);
    EXPECT_NEAR(manager.local_data_.start_time_, gap, 1e-12);
  }
}

TEST(ScanReplanFsm, FirstFrozenAcknowledgementPreservesItsReceptionInterval) {
  using namespace nav_kernel::local::scan::upstream;
  SCANPlannerManager manager;
  PlannerFsm fsm(manager, ScanReplanParams{});
  FsmInput input;
  input.nowS = 9.99;
  input.executionFrozen = true;
  ASSERT_EQ(fsm.tick(input).state, ScanReplanState::INIT);

  manager.local_data_.traj_id_ = 1;
  input.executingTrajectoryId = 1;
  input.executionStartTimeS = 10.0;
  input.nowS = 10.01;
  ASSERT_EQ(fsm.tick(input).state, ScanReplanState::INIT);
  EXPECT_NEAR(manager.local_data_.start_time_, 10.01, 1e-12);

  input.nowS = 10.02;
  (void)fsm.tick(input);
  EXPECT_NEAR(manager.local_data_.start_time_, 10.02, 1e-12);
  input.nowS = 10.03;
  input.executionFrozen = false;
  (void)fsm.tick(input);
  EXPECT_NEAR(input.nowS - manager.local_data_.start_time_, .01, 1e-12);
}

TEST(ScanReplanFsm, FirstFrozenAcknowledgementDoesNotRefreezeSettledMotionTime) {
  using namespace nav_kernel::local::scan::upstream;
  SCANPlannerManager manager;
  PlannerFsm fsm(manager, ScanReplanParams{});
  FsmInput input;
  input.nowS = 9.99;
  input.executionFrozen = true;
  ASSERT_EQ(fsm.tick(input).state, ScanReplanState::INIT);

  manager.local_data_.traj_id_ = 1;
  input.nowS = 10.20;
  input.executionFrozen = false;
  (void)fsm.tick(input);
  input.executingTrajectoryId = 1;
  input.executionStartTimeS = 10.0;
  input.nowS = 10.21;
  input.executionFrozen = true;
  ASSERT_EQ(fsm.tick(input).state, ScanReplanState::INIT);
  EXPECT_NEAR(manager.local_data_.start_time_, 10.01, 1e-12);
}

TEST(ScanReplanFsm, AuthoritativeExecutionPhaseCorrectsMissedFreezeWithoutAccumulatingAgain) {
  using namespace nav_kernel::local::scan::upstream;
  SCANPlannerManager manager;
  PlannerFsm fsm(manager, ScanReplanParams{});
  FsmInput input;
  input.nowS = 9.99;
  input.executionFrozen = false;
  (void)fsm.tick(input);
  manager.local_data_.traj_id_ = 1;
  manager.local_data_.duration_ = 5.0;
  input.executingTrajectoryId = 1;
  input.executionStartTimeS = 10.0;
  input.nowS = 10.10;
  input.executionTimeS = .03;
  (void)fsm.tick(input);
  EXPECT_NEAR(manager.local_data_.start_time_, 10.07, 1e-12);

  input.nowS = 10.20;
  input.executionFrozen = true;
  input.executionTimeS = .04;
  (void)fsm.tick(input);
  EXPECT_NEAR(manager.local_data_.start_time_, 10.16, 1e-12);
  input.nowS = 10.21;
  (void)fsm.tick(input);
  EXPECT_NEAR(manager.local_data_.start_time_, 10.17, 1e-12);

  input.nowS = 10.22;
  input.executionFrozen = false;
  input.executionTimeS = .05;
  (void)fsm.tick(input);
  EXPECT_NEAR(manager.local_data_.start_time_, 10.17, 1e-12);
  input.nowS = 10.23;
  input.executingTrajectoryId = 2;
  input.executionTimeS = 3.0;
  (void)fsm.tick(input);
  EXPECT_NEAR(manager.local_data_.start_time_, 10.17, 1e-12);
}

TEST(ScanBackend, LowerLimitKeepsAnAlreadyFeasibleSplineWithoutChangingItsTiming) {
  RequestFixture fixture({{0, 0, .5}, {.2, 0, .5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += .01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const SplineTarget original = std::get<SplineTarget>(plan.target());
  double bound = 0;
  for (std::size_t i = 0; i + 1 < original.controls.size(); ++i) {
    const auto& from = original.controls[i];
    const auto& to = original.controls[i + 1];
    bound = std::max(bound, original.order * nav_kernel::distance3D(from, to) /
        (original.knots[i + original.order + 1] - original.knots[i + 1]));
  }
  ASSERT_LT(bound, original.maxLinearSpeedMps);
  fixture.request.maxLinearSpeedMps = .5*(bound + original.maxLinearSpeedMps);
  const nav_kernel::SplineView view(original);
  const double elapsed = .25*view.duration();
  fixture.request.clock.timestampS = original.startTimeS + elapsed;
  fixture.request.robot.pose.position = view.position(elapsed);
  fixture.request.robot.kinematics.valid = true;
  fixture.request.robot.kinematics.linearVelocity = view.velocity(elapsed);
  plan = backend.tick(fixture.request);
  ASSERT_TRUE(plan.ready());
  const auto& retained = std::get<SplineTarget>(plan.target());
  EXPECT_EQ(retained.trajectoryId, original.trajectoryId);
  EXPECT_EQ(retained.knots, original.knots);
  EXPECT_DOUBLE_EQ(retained.startTimeS, original.startTimeS);
  EXPECT_DOUBLE_EQ(retained.maxLinearSpeedMps, original.maxLinearSpeedMps);
  fixture.request.robot.kinematics.linearVelocity = {.99*fixture.request.maxLinearSpeedMps, 0, .3};
  fixture.request.clock.timestampS += .01;
  plan = backend.tick(fixture.request);
  ASSERT_TRUE(plan.ready()) << "Vertical balancing motion invalidated a planar speed limit";
  EXPECT_EQ(std::get<SplineTarget>(plan.target()).trajectoryId, original.trajectoryId);
  fixture.request.robot.kinematics.linearVelocity = {fixture.request.maxLinearSpeedMps + .1, 0, 0};
  fixture.request.clock.timestampS += .01;
  plan = backend.tick(fixture.request);
  ASSERT_TRUE(plan.ready());
  EXPECT_GT(std::get<SplineTarget>(plan.target()).speedTransitionEndS, 0.0);
}

TEST(ScanSpeedTransition, CubicBridgePreservesBoundaryAndBoundsAccelerationAndJerk) {
  ScanAttemptFixture fixture;
  fixture.manager.pp_.max_vel_ = .2;
  const Eigen::Vector3d position{0,0,.5}, velocity{.6,.2,.02}, acceleration{.1,.05,0};
  double transition = 0;
  ASSERT_TRUE(fixture.manager.decelerate(position,velocity,acceleration,.2,1,transition));
  const auto &curve = fixture.manager.local_data_.position_traj_;
  const auto derivative = curve.getDerivative();
  const auto second = derivative.getDerivative();
  const auto third = second.getDerivative();
  EXPECT_LE((curve.evaluateDeBoorT(0)-position).norm(),1e-12);
  EXPECT_LE((derivative.evaluateDeBoorT(0)-velocity).norm(),1e-12);
  EXPECT_LE((second.evaluateDeBoorT(0)-acceleration).norm(),1e-12);
  EXPECT_NEAR(derivative.evaluateDeBoorT(transition).head<2>().norm(),.2,1e-12);
  EXPECT_NEAR(derivative.evaluateDeBoorT(transition).z(),0,1e-12);
  EXPECT_LE(second.evaluateDeBoorT(transition).norm(),1e-12);
  EXPECT_GT(curve.getTimeSum(),transition);
  for (double time=0; time<curve.getTimeSum(); time+=.005) {
    EXPECT_LE(second.evaluateDeBoorT(time).norm(),fixture.manager.pp_.max_acc_+1e-10);
    EXPECT_LE(third.evaluateDeBoorT(time).norm(),fixture.manager.pp_.max_jerk_+1e-10);
    if (time>=transition)
      EXPECT_LE(derivative.evaluateDeBoorT(time).head<2>().norm(),.2+1e-10);
  }
}

TEST(ScanSpeedTransition, RejectedBridgeDoesNotReplaceTheExecutingTrajectory) {
  ScanAttemptFixture fixture;
  fixture.manager.pp_.max_vel_ = .2;
  fixture.manager.local_data_.traj_id_=17;
  fixture.request.bitmap.occupy({.55,.05,.55});
  double transition=0;
  EXPECT_FALSE(fixture.manager.decelerate({0,0,.5},{.6,0,0},
      Eigen::Vector3d::Zero(),.2,1,transition));
  EXPECT_EQ(fixture.manager.local_data_.traj_id_,17);
  EXPECT_EQ(transition,0);
}

TEST(ScanSpeedTransition, FixedHeadingTransitionEndsInsideThePhysicalLateralLimit) {
  ScanAttemptFixture fixture;
  fixture.manager.pp_.max_vel_=.4;
  fixture.manager.pp_.max_body_vx_=.6;
  fixture.manager.pp_.max_body_vy_=.25;
  fixture.manager.pp_.body_yaw_=0;
  double transition=0;
  ASSERT_TRUE(fixture.manager.decelerate({0,0,.5},{0,.5,0},
      Eigen::Vector3d::Zero(),.2,1,transition));
  const auto derivative=fixture.manager.local_data_.velocity_traj_;
  const Eigen::Vector3d finalVelocity=derivative.evaluateDeBoorT(transition);
  EXPECT_NEAR(finalVelocity.x(),0,1e-12);
  EXPECT_NEAR(finalVelocity.y(),.25,1e-12);
  EXPECT_LE(fixture.manager.pp_.speedRatio(finalVelocity),1+1e-10);
}

TEST(ScanSpeedTransition, ForecastRemainsAvailableToFinalMotionChecks) {
  for (bool crossesAtArrival : {false,true}) {
    SCOPED_TRACE(crossesAtArrival);
    ScanAttemptFixture fixture;
    fixture.manager.pp_.max_vel_=.2;
    double transition=0;
    ASSERT_TRUE(fixture.manager.decelerate({0,0,.5},{.6,0,0},
        Eigen::Vector3d::Zero(),.2,1,transition));
    const auto clearCurve=fixture.manager.local_data_.position_traj_;
    const nav_kernel::PredictedObstacle crossing = crossesAtArrival
        ? nav_kernel::PredictedObstacle{{.65,-1.7,.5},{.65,.3,.5},.05,.1,1}
        : nav_kernel::PredictedObstacle{{.65,-.2,.5},{.65,3.8,.5},.05,.1,1};
    fixture.request.request.environment.predictions={&crossing,1,3,1,2};
    const nav_kernel::local::scan::Grid grid(scanParams(),fixture.request.request);
    fixture.manager.grid_map_->setGrid(&grid);
    // Both people cross the same x=.65 line. The first is already clear
    // before arrival; the second meets the decelerating robot at t=1.7 s.
    const Eigen::Vector3d meetingPoint=clearCurve.evaluateDeBoorT(1.7);
    const Eigen::Vector3d meetingDerivative=clearCurve.getDerivative().evaluateDeBoorT(1.7);
    const Vec3 meeting{meetingPoint.x(),meetingPoint.y(),meetingPoint.z()};
    const Vec3 meetingVelocity{meetingDerivative.x(),meetingDerivative.y(),meetingDerivative.z()};
    EXPECT_EQ(grid.trajectoryOccupancy(meeting,meetingVelocity,meeting,meetingVelocity,
                                      1.7,1.7),crossesAtArrival ? 1 : 0);
    const bool accepted=fixture.manager.decelerate({0,0,.5},{.6,0,0},
        Eigen::Vector3d::Zero(),.2,1,transition);
    EXPECT_TRUE(accepted);
  }
}

TEST(ScanBackend, MovingSpeedReductionPublishesCheckedBridgeAndRepeatedReductionsRemainContinuous) {
  RequestFixture fixture({{0,0,.5},{4,0,.5}});
  fixture.request.clock.mode=nav_kernel::PlanClockMode::External;
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for(int tick=0;tick<20 && !plan.ready();++tick) {
    fixture.request.clock.timestampS+=.01;
    plan=backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  std::int64_t previousId=std::get<SplineTarget>(plan.target()).trajectoryId;
  fixture.request.robot.kinematics.valid=true;
  fixture.request.robot.kinematics.linearVelocity={.6,0,.3};
  for (double cap : {.3,.2,.15}) {
    SCOPED_TRACE(cap);
    const Vec3 expectedPosition=fixture.request.robot.pose.position;
    const Vec3 expectedVelocity=fixture.request.robot.kinematics.linearVelocity;
    fixture.request.maxLinearSpeedMps=cap;
    plan=backend.tick(fixture.request);
    ASSERT_TRUE(plan.ready())<<backend.debugSnapshot().searchReason;
    const SplineTarget target=std::get<SplineTarget>(plan.target());
    ASSERT_GT(target.speedTransitionEndS,0);
    ASSERT_GT(target.trajectoryId,previousId);
    EXPECT_DOUBLE_EQ(target.maxLinearSpeedMps,cap);
    const nav_kernel::SplineView curve(target);
    EXPECT_NEAR(curve.position(0).x,expectedPosition.x,1e-10);
    EXPECT_NEAR(curve.position(0).y,expectedPosition.y,1e-10);
    EXPECT_NEAR(curve.velocity(0).x,expectedVelocity.x,1e-10);
    EXPECT_NEAR(curve.velocity(0).y,expectedVelocity.y,1e-10);
    EXPECT_NEAR(curve.velocity(0).z,0,1e-10)<<"Base heave must not launch a vertical bridge";
    EXPECT_NEAR(std::hypot(curve.velocity(target.speedTransitionEndS).x,
                          curve.velocity(target.speedTransitionEndS).y),cap,1e-10);
    const double elapsed=.1*target.speedTransitionEndS;
    fixture.request.robot.pose.position=curve.position(elapsed);
    fixture.request.robot.kinematics.linearVelocity=curve.velocity(elapsed);
    fixture.request.clock.timestampS=target.startTimeS+elapsed;
    previousId=target.trajectoryId;
  }
}

TEST(ScanBackend, BlockedSpeedBridgeStopsInsteadOfPublishingUncheckedOrOldGeometry) {
  RequestFixture fixture({{0,0,.5},{4,0,.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for(int tick=0;tick<20 && !plan.ready();++tick) {
    fixture.request.clock.timestampS+=.01;
    plan=backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto oldId = std::get<SplineTarget>(plan.target()).trajectoryId;
  fixture.request.robot.kinematics.valid=true;
  fixture.request.robot.kinematics.linearVelocity={.6,0,0};
  fixture.request.maxLinearSpeedMps=.2;
  fixture.bitmap.occupy({.55,.05,.55});
  fixture.refreshCollision(2);
  plan=backend.tick(fixture.request);
  EXPECT_EQ(plan.status(),nav_kernel::LocalPlanStatus::NearFieldStop);
  EXPECT_EQ(backend.debugSnapshot().searchReason,"scan_speed_transition_blocked");
  const auto replacement = backend.tick(fixture.request);
  if (replacement.ready()) {
    const auto target = std::get<SplineTarget>(replacement.target());
    EXPECT_GT(target.trajectoryId, oldId);
    const nav_kernel::SplineView curve(target);
    const nav_kernel::local::scan::Grid grid(scanParams(), fixture.request);
    EXPECT_LT(nav_kernel::distance3D(curve.position(0), fixture.request.robot.pose.position), .01);
    EXPECT_LT(nav_kernel::distance3D(curve.velocity(curve.duration()), {}), .01);
    for (double t = 0; t < curve.duration(); t += .005) {
      const auto position = curve.position(t), next = curve.position(t + .005);
      EXPECT_EQ(grid.cylinderOccupancy(position,
          std::atan2(next.y-position.y, next.x-position.x)), 0);
      EXPECT_LE(nav_kernel::distance3D(curve.velocity(t), {}), .601);
    }
  }
}

TEST(ScanBackend, LowerCapKeepsMovingAtAnAlreadyLegalObservedSpeedWhilePreparingReplacement) {
  RequestFixture fixture({{0,0,.5},{3,0,.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for(int tick=0;tick<20 && !plan.ready();++tick) {
    fixture.request.clock.timestampS+=.01;
    plan=backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto previousId=std::get<SplineTarget>(plan.target()).trajectoryId;
  fixture.request.robot.kinematics.valid=true;
  fixture.request.robot.kinematics.linearVelocity={.1,0,0};
  fixture.request.maxLinearSpeedMps=.2;
  plan=backend.tick(fixture.request);
  ASSERT_TRUE(plan.ready())<<backend.debugSnapshot().searchReason;
  const SplineTarget target=std::get<SplineTarget>(plan.target());
  ASSERT_GT(target.trajectoryId,previousId);
  EXPECT_GT(target.speedTransitionEndS,0);
  const nav_kernel::SplineView curve(target);
  EXPECT_NEAR(curve.position(0).x,fixture.request.robot.pose.position.x,1e-10);
  EXPECT_NEAR(curve.velocity(0).x,.1,1e-10);
  EXPECT_NEAR(curve.velocity(target.speedTransitionEndS).x,.1,1e-10);
  EXPECT_GT(curve.velocity(curve.duration()).x,0);
}

TEST(ScanSpeedTransition, NormalScanReplacementStartsAfterDecelerationWithContinuousBoundary) {
  using namespace nav_kernel::local::scan::upstream;
  ScanAttemptFixture fixture;
  ScanReplanParams params;
  params.navigationMode=ScanNavigationMode::REFERENCE_PATH;
  params.planningHorizon=3.5;
  params.noReplanThreshold=.1;
  params.replanThreshold=.2;
  PlannerFsm fsm(fixture.manager,params);
  FsmInput input;
  input.nowS=1;
  input.odometry=FsmOdometry{};
  input.odometry->position={0,0,.5};
  input.referencePath=std::vector<Eigen::Vector3d>{{0,0,.5},{2,0,.5}};
  ASSERT_TRUE(fsm.tick(input).targetAccepted);
  input.referencePath.reset();
  FsmOutput output;
  for(int tick=0;tick<8 && !output.trajectory;++tick) {
    input.nowS+=.01;
    output=fsm.tick(input);
  }
  ASSERT_TRUE(output.trajectory);
  fixture.manager.pp_.max_vel_=.2;
  double transition=0;
  input.odometry->velocity={.5,0,0};
  ASSERT_TRUE(fixture.manager.decelerate(input.odometry->position,input.odometry->velocity,
      Eigen::Vector3d::Zero(),.4,input.nowS,transition));
  const LocalTrajData bridge=fixture.manager.local_data_;
  fsm.adoptSpeedTransition(transition);
  input.executingTrajectoryId=bridge.traj_id_;
  input.executionStartTimeS=bridge.start_time_;
  input.executionFrozen=false;
  input.handoffLeadTimeS=.1;
  output=fsm.tick(input);
  ASSERT_TRUE(output.trajectory)<<fixture.manager.reboundDebug().reason;
  ASSERT_TRUE(output.handoffPending)<<fixture.manager.reboundDebug().reason;
  ASSERT_TRUE(output.handoff);
  const auto &candidate=*output.trajectory;
  EXPECT_GE(candidate.startTimeS,bridge.start_time_+transition-1e-10);
  UniformBspline replacement(candidate.positionPoints,3,.1);
  replacement.setKnot(candidate.knots);
  const double boundary=candidate.startTimeS-bridge.start_time_;
  EXPECT_LE((replacement.evaluateDeBoorT(0)-bridge.position_traj_.evaluateDeBoorT(boundary)).norm(),1e-8);
  EXPECT_LE((replacement.getDerivative().evaluateDeBoorT(0)-
      bridge.velocity_traj_.evaluateDeBoorT(boundary)).norm(),1e-8);
  EXPECT_LE(replacement.getDerivative().evaluateDeBoorT(0).norm(),.2+1e-8);
  EXPECT_EQ(fixture.manager.local_data_.traj_id_,bridge.traj_id_)
      <<"Keep the checked bridge active until physical receipt";
  ASSERT_TRUE(output.handoff->receive());
  input.nowS=candidate.startTimeS;
  input.odometry->position=bridge.position_traj_.evaluateDeBoorT(boundary);
  input.odometry->velocity=bridge.velocity_traj_.evaluateDeBoorT(boundary);
  output=fsm.tick(input);
  ASSERT_TRUE(output.trajectory);
  EXPECT_GT(fixture.manager.local_data_.traj_id_,bridge.traj_id_);
}

TEST(ScanSpeedTransition, DelayedReceptionKeepsContinuationPending) {
  using namespace nav_kernel::local::scan::upstream;
  ScanAttemptFixture fixture;
  ScanReplanParams params;
  params.navigationMode=ScanNavigationMode::REFERENCE_PATH;
  params.planningHorizon=3.5;
  params.noReplanThreshold=.1;
  params.replanThreshold=.2;
  params.maxReplanFailCount=2;
  PlannerFsm fsm(fixture.manager,params);
  FsmInput input;
  input.nowS=1;
  input.odometry=FsmOdometry{};
  input.odometry->position={0,0,.5};
  input.referencePath=std::vector<Eigen::Vector3d>{{0,0,.5},{2,0,.5}};
  ASSERT_TRUE(fsm.tick(input).targetAccepted);
  input.referencePath.reset();
  FsmOutput output;
  for(int tick=0;tick<8 && !output.trajectory;++tick) {
    input.nowS+=.01;
    output=fsm.tick(input);
  }
  ASSERT_TRUE(output.trajectory);
  const LocalTrajData original=fixture.manager.local_data_;
  fixture.manager.pp_.max_vel_=.2;
  double transition=0;
  input.odometry->velocity={.5,0,0};
  ASSERT_TRUE(fixture.manager.decelerate(input.odometry->position,input.odometry->velocity,
      Eigen::Vector3d::Zero(),.4,input.nowS,transition));
  const LocalTrajData bridge=fixture.manager.local_data_;
  const int attempt=fixture.manager.reboundDebug().attemptId;
  fsm.adoptSpeedTransition(transition);
  input.executingTrajectoryId=original.traj_id_;
  input.executionStartTimeS=original.start_time_;
  input.executionFrozen=false;
  for(int tick=0;tick<4;++tick) {
    input.nowS=bridge.start_time_+tick*.01;
    output=fsm.tick(input);
    ASSERT_EQ(output.state,ScanReplanState::REPLAN_TRAJ);
    EXPECT_FALSE(output.trajectory);
    EXPECT_EQ(fixture.manager.reboundDebug().attemptId,attempt);
    EXPECT_EQ(fixture.manager.local_data_.traj_id_,bridge.traj_id_);
  }

  input.nowS=bridge.start_time_+.04;
  input.executingTrajectoryId=bridge.traj_id_;
  input.executionStartTimeS=bridge.start_time_;
  input.handoffLeadTimeS=.1;
  input.odometry->position=bridge.position_traj_.evaluateDeBoorT(.04);
  input.odometry->velocity=bridge.velocity_traj_.evaluateDeBoorT(.04);
  constexpr double solveDelay=.05;
  fixture.manager.setTimeSource([&] {return input.nowS+solveDelay;});
  output=fsm.tick(input);
  ASSERT_TRUE(output.handoffPending)<<fixture.manager.reboundDebug().reason;
  ASSERT_TRUE(output.trajectory);
  ASSERT_TRUE(output.handoff);
  const auto &candidate=*output.trajectory;
  EXPECT_GE(candidate.startTimeS,bridge.start_time_+transition-1e-10);
  EXPECT_LT(candidate.startTimeS,bridge.start_time_+bridge.duration_);
  EXPECT_LT(input.nowS+solveDelay+2*PlannerFsm::kTickPeriodS,candidate.startTimeS);
  UniformBspline replacement(candidate.positionPoints,candidate.order,.1);
  replacement.setKnot(candidate.knots);
  const double boundary=candidate.startTimeS-bridge.start_time_;
  EXPECT_LE((replacement.evaluateDeBoorT(0)-
      bridge.position_traj_.evaluateDeBoorT(boundary)).norm(),1e-8);
  EXPECT_LE((replacement.getDerivative().evaluateDeBoorT(0)-
      bridge.velocity_traj_.evaluateDeBoorT(boundary)).norm(),1e-8);
  EXPECT_LE((replacement.getDerivative().getDerivative().evaluateDeBoorT(0)-
      bridge.acceleration_traj_.evaluateDeBoorT(boundary)).norm(),1e-8);
  EXPECT_EQ(fixture.manager.local_data_.traj_id_,bridge.traj_id_);
}

TEST(ScanSpeedTransition, RejectedBridgeWithdrawsTheUnreceivedFasterOffer) {
  using namespace nav_kernel::local::scan::upstream;
  ScanAttemptFixture fixture;
  ScanReplanParams params;
  params.navigationMode=ScanNavigationMode::REFERENCE_PATH;
  params.planningHorizon=3.5;
  PlannerFsm fsm(fixture.manager,params);
  FsmInput input;
  input.nowS=1;
  input.odometry=FsmOdometry{};
  input.odometry->position={0,0,.5};
  input.referencePath=std::vector<Eigen::Vector3d>{{0,0,.5},{2,0,.5}};
  ASSERT_TRUE(fsm.tick(input).targetAccepted);
  input.referencePath.reset();
  FsmOutput output;
  for(int tick=0;tick<8 && !output.trajectory;++tick) {
    input.nowS+=.01;
    output=fsm.tick(input);
  }
  ASSERT_TRUE(output.trajectory);
  fixture.manager.pp_.max_vel_=.2;
  input.odometry->velocity={.5,0,0};
  double transition=0;
  ASSERT_TRUE(fixture.manager.decelerate(input.odometry->position,
      input.odometry->velocity,Eigen::Vector3d::Zero(),.4,input.nowS,transition));
  fsm.adoptSpeedTransition(transition);
  input.executingTrajectoryId=fixture.manager.local_data_.traj_id_;
  input.executionStartTimeS=fixture.manager.local_data_.start_time_;
  input.executionFrozen=false;
  input.handoffLeadTimeS=.1;
  output=fsm.tick(input);
  ASSERT_TRUE(output.handoffPending);
  const auto offer=output.handoff;
  const auto offerId=output.trajectory->trajectoryId;
  fixture.manager.pp_.max_vel_=.1;
  fixture.request.bitmap.occupy({.55,.05,.55});
  EXPECT_FALSE(fsm.startSpeedTransition(*input.odometry,input.nowS,.185,output));
  EXPECT_FALSE(output.trajectory);
  EXPECT_EQ(output.state,ScanReplanState::GEN_NEW_TRAJ);
  EXPECT_FALSE(offer->receive());
  EXPECT_GE(fixture.manager.local_data_.traj_id_,offerId);
  input.odometry->velocity.setZero();
  input.executionFrozen=true;
  for(int tick=0;tick<3;++tick) {
    input.nowS+=.01;
    output=fsm.tick(input);
    EXPECT_FALSE(output.handoffPending);
    if(output.trajectory) EXPECT_GT(output.trajectory->trajectoryId,offerId);
  }
}

TEST(ScanBackend, StationarySpeedReductionReplansWithoutRestartingIds) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const SplineTarget original = std::get<SplineTarget>(plan.target());
  EXPECT_DOUBLE_EQ(original.maxLinearSpeedMps, scanParams().scan.maxVelocity);

  fixture.request.maxLinearSpeedMps = 0.2;
  fixture.request.clock.timestampS += 0.01;
  plan = backend.tick(fixture.request);
  EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Pending);

  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const SplineTarget slower = std::get<SplineTarget>(plan.target());
  EXPECT_GT(slower.trajectoryId, original.trajectoryId);
  EXPECT_DOUBLE_EQ(slower.maxLinearSpeedMps, 0.2);
  EXPECT_GT(nav_kernel::SplineView(slower).duration(),
            nav_kernel::SplineView(original).duration());
}

TEST(ScanBackend, ContinuousSpeedUpdatesDoNotStarveFsmTimers) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  std::int64_t lastId = 0;
  int publications = 0;
  for (int tick = 0; tick < 16; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    fixture.request.maxLinearSpeedMps = 0.2 + tick * 0.01;
    const LocalPlan plan = backend.tick(fixture.request);
    if (!plan.ready()) continue;
    const auto &spline = std::get<SplineTarget>(plan.target());
    if (spline.trajectoryId == lastId) continue;
    EXPECT_GT(spline.trajectoryId, lastId);
    EXPECT_DOUBLE_EQ(spline.maxLinearSpeedMps, fixture.request.maxLinearSpeedMps);
    lastId = spline.trajectoryId;
    ++publications;
  }
  EXPECT_GE(publications, 3);
}

TEST(ScanBackend, SpeedIncreaseRetainsCurrentTrajectoryUntilReplacement) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  fixture.request.maxLinearSpeedMps = 0.2;
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto oldId = std::get<SplineTarget>(plan.target()).trajectoryId;
  fixture.request.maxLinearSpeedMps = 0.5;
  fixture.request.clock.timestampS += 0.01;
  plan = backend.tick(fixture.request);
  ASSERT_TRUE(plan.ready());
  EXPECT_EQ(std::get<SplineTarget>(plan.target()).trajectoryId, oldId);
  EXPECT_DOUBLE_EQ(std::get<SplineTarget>(plan.target()).maxLinearSpeedMps, 0.2);
  for (int tick = 0; tick < 20 &&
                     std::get<SplineTarget>(plan.target()).trajectoryId == oldId; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    ASSERT_TRUE(plan.ready());
  }
  EXPECT_GT(std::get<SplineTarget>(plan.target()).trajectoryId, oldId);
  EXPECT_DOUBLE_EQ(std::get<SplineTarget>(plan.target()).maxLinearSpeedMps, 0.5);
}

TEST(ScanBackend, SpeedReductionStartsFromStoppedOdometryInsteadOfOldSplineVelocity) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const SplineTarget original = std::get<SplineTarget>(plan.target());
  const nav_kernel::SplineView originalView(original);
  const double movingTime = 0.4 * originalView.duration();
  const Vec3 oldVelocity = originalView.velocity(movingTime);
  ASSERT_GT(std::hypot(oldVelocity.x, oldVelocity.y), 0.2);
  fixture.request.robot.pose.position = originalView.position(movingTime);
  fixture.request.robot.kinematics.valid = true;
  fixture.request.robot.kinematics.linearVelocity = {};
  fixture.request.maxLinearSpeedMps = 0.1;
  fixture.request.clock.executionFrozen = true;
  fixture.request.clock.timestampS = original.startTimeS + movingTime;

  // The collision timer may receive the lower limit before the FSM timer.
  plan = backend.checkCollision(fixture.request);
  EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Pending);
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto &slower = std::get<SplineTarget>(plan.target());
  ASSERT_GT(slower.trajectoryId, original.trajectoryId);
  EXPECT_DOUBLE_EQ(slower.maxLinearSpeedMps, 0.1);
  const nav_kernel::SplineView slowerView(slower);
  const Vec3 start = slowerView.position(0.0);
  const Vec3 velocity = slowerView.velocity(0.0);
  EXPECT_NEAR(start.x, fixture.request.robot.pose.position.x, 0.02);
  EXPECT_NEAR(start.y, fixture.request.robot.pose.position.y, 0.02);
  EXPECT_NEAR(velocity.x, 0.0, 0.01);
  EXPECT_NEAR(velocity.y, 0.0, 0.01);
}

TEST(ScanBackend, ContinuousSpeedReductionsDoNotRestartPendingGeneration) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  std::int64_t lastId = std::get<SplineTarget>(plan.target()).trajectoryId;
  int publications = 0;
  for (int tick = 0; tick < 16; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    fixture.request.maxLinearSpeedMps = 0.6 - tick * 0.01;
    plan = backend.tick(fixture.request);
    if (!plan.ready()) continue;
    const auto &spline = std::get<SplineTarget>(plan.target());
    if (spline.trajectoryId == lastId) continue;
    EXPECT_GT(spline.trajectoryId, lastId);
    EXPECT_DOUBLE_EQ(spline.maxLinearSpeedMps, fixture.request.maxLinearSpeedMps);
    lastId = spline.trajectoryId;
    ++publications;
  }
  EXPECT_GE(publications, 3);
}

TEST(ScanBackend, RestoringSpeedCannotReviveInvalidatedTrajectory) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  fixture.request.maxLinearSpeedMps = 0.5;
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto oldId = std::get<SplineTarget>(plan.target()).trajectoryId;

  fixture.request.maxLinearSpeedMps = 0.1;
  fixture.request.clock.timestampS += 0.01;
  EXPECT_EQ(backend.tick(fixture.request).status(), nav_kernel::LocalPlanStatus::Pending);
  fixture.request.maxLinearSpeedMps = 0.5;
  fixture.request.clock.timestampS += 0.01;
  EXPECT_EQ(backend.checkCollision(fixture.request).status(), nav_kernel::LocalPlanStatus::Pending);
  for (int tick = 0; tick < 20; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    if (plan.ready()) break;
    EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Pending);
  }
  ASSERT_TRUE(plan.ready());
  EXPECT_GT(std::get<SplineTarget>(plan.target()).trajectoryId, oldId);
  EXPECT_DOUBLE_EQ(std::get<SplineTarget>(plan.target()).maxLinearSpeedMps, 0.5);
}

TEST(ScanBackend, BlockedReferenceDuringSpeedReductionCannotReviveOldSpline) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  for (int x = -1; x <= 21; ++x)
    fixture.bitmap.occupyInflated({x * 0.1F, 0.0F, 0.5F}, 0.15, 0.1, 0.1);
  fixture.refreshCollision(2);
  fixture.request.maxLinearSpeedMps = 0.1;
  fixture.request.clock.timestampS += 0.01;
  EXPECT_EQ(backend.tick(fixture.request).status(), nav_kernel::LocalPlanStatus::Pending);
  fixture.request.maxLinearSpeedMps = 0.5;
  for (int tick = 0; tick < 10; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    EXPECT_FALSE(plan.ready());
  }
  EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Blocked);
}

TEST(ScanBackend, PreservesRequestedSpeedBelowConfigurationFloor) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {1.0, 0.0, 0.5}});
  fixture.request.maxLinearSpeedMps = 0.03;
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  EXPECT_DOUBLE_EQ(std::get<SplineTarget>(plan.target()).maxLinearSpeedMps, 0.03);
}

TEST(ScanBackend, FinishedShortHorizonContinuesWithoutAnotherGoal) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {3.0, 0.0, 0.5}});
  auto params = scanParams();
  params.scan.planningHorizon = 0.5;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto first = std::get<SplineTarget>(plan.target());
  const nav_kernel::SplineView firstView(first);
  const auto endpoint = firstView.position(firstView.duration());
  ASSERT_LT(endpoint.x, 1.0);
  fixture.request.robot.pose.position = endpoint;
  fixture.request.clock.timestampS = first.startTimeS + firstView.duration() + 0.03;
  for (int tick = 0; tick < 20; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    if (plan.ready() && std::get<SplineTarget>(plan.target()).trajectoryId > first.trajectoryId)
      break;
  }
  ASSERT_TRUE(plan.ready());
  const auto next = std::get<SplineTarget>(plan.target());
  EXPECT_GT(next.trajectoryId, first.trajectoryId);
  const nav_kernel::SplineView nextView(next);
  EXPECT_GT(nextView.position(nextView.duration()).x, endpoint.x + 0.1);
}

TEST(ScanBackend, ZeroMotionIntentCannotRepublishThePreviousSpline) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {3.0, 0.0, 0.5}});
  fixture.request.objective = nav_kernel::MotionIntentTarget{
      {0.0, 0.5, 3.0, 90.0}, {fixture.route.data(), 2, 1}};
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  std::get<nav_kernel::MotionIntentTarget>(fixture.request.objective)
      .intent.speedNormalized = 0.0;
  for (int tick = 0; tick < 20; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    EXPECT_FALSE(plan.ready());
    EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::NoPath);
    EXPECT_EQ(backend.debugSnapshot().searchReason, "intent_idle");
  }
}

TEST(ScanBackend, HeldMotionIntentReplansBeforeShortSegmentStops) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {3.0, 0.0, 0.5}});
  // Keep planning completion at the input time to compare exact boundaries.
  fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
  fixture.request.objective = nav_kernel::MotionIntentTarget{
      {0.0, 0.5, 3.0, 90.0}, {fixture.route.data(), 2, 1}};
  fixture.request.maxLinearSpeedMps = 0.5;
  auto params = scanParams();
  params.scan.planningHorizon = 0.5;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
  const auto first = std::get<SplineTarget>(plan.target());
  const nav_kernel::SplineView firstView(first);
  const auto start = firstView.position(0.0);
  const auto endpoint = firstView.position(firstView.duration());
  const double segmentLength = nav_kernel::distance3D(start, endpoint);
  ASSERT_LT(segmentLength, params.scan.replanDistance);

  double continuationTime = 0.0;
  const double continuationDistance =
      std::min(params.scan.replanDistance,
               std::max(params.scan.noReplanDistance, 0.5 * segmentLength));
  for (int sample = 1; sample < 1000; ++sample) {
    const double time = firstView.duration() * sample / 1000.0;
    if (nav_kernel::distance3D(start, firstView.position(time)) >
        continuationDistance + 1e-3) {
      continuationTime = time;
      break;
    }
  }
  ASSERT_GT(continuationTime, 0.0);
  ASSERT_LT(continuationTime, firstView.duration() - 0.02);
  fixture.request.robot.pose.position = firstView.position(continuationTime);
  fixture.request.robot.kinematics.valid = true;
  fixture.request.robot.kinematics.linearVelocity = firstView.velocity(continuationTime);
  fixture.request.clock.timestampS = first.startTimeS + continuationTime;

  LocalPlan replacement;
  for (int tick = 0; tick < 20; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    replacement = backend.tick(fixture.request);
    if (replacement.ready() &&
        std::get<SplineTarget>(replacement.target()).trajectoryId > first.trajectoryId)
      break;
  }
  ASSERT_TRUE(replacement.ready()) << backend.debugSnapshot().searchReason;
  const auto continued = std::get<SplineTarget>(replacement.target());
  ASSERT_GT(continued.trajectoryId, first.trajectoryId);
  ASSERT_LT(continued.startTimeS, first.startTimeS + firstView.duration());

  const nav_kernel::SplineView continuedView(continued);
  // Continue before the horizon ends without substituting the old spline's
  // clock-driven state for the robot's observed position and velocity.
  EXPECT_LT(nav_kernel::distance3D(continuedView.position(0.0),
                                   fixture.request.robot.pose.position), 1e-3);
  EXPECT_LT(nav_kernel::distance3D(continuedView.velocity(0.0),
                                   fixture.request.robot.kinematics.linearVelocity), 1e-3);
  const auto initialVelocity = continuedView.velocity(0.0);
  EXPECT_GT(std::hypot(initialVelocity.x, initialVelocity.y), 0.05);
}

TEST(ScanBackend, HeldMotionIntentRetainsShortSideTargetUntilMeasuredArrival) {
  for (const bool obstructTail : {false, true}) {
    SCOPED_TRACE(obstructTail);
    RequestFixture fixture({{0.025, 1.025, 0.525}, {3.525, 1.025, 0.525}}, 0.05);
    const auto setPocket = [&](bool oppositeOpen) {
      fixture.bitmap.clear();
      for (int x = -99; x < 99; ++x) {
        fixture.bitmap.occupy({x * 0.05 + 0.025, 0.675, 0.525});
        if (!oppositeOpen)
          fixture.bitmap.occupy({x * 0.05 + 0.025, 1.075, 0.525});
      }
      for (int y = -99; y < 99; ++y)
        fixture.bitmap.occupy({0.075, y * 0.05 + 0.025, 0.525});
      for (int x = 2; x < 75; ++x)
        fixture.bitmap.occupy({x * 0.05 + 0.025, 1.025, 0.525});
    };
    setPocket(false);
    fixture.refreshCollision(2);
    fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
    fixture.request.objective = nav_kernel::MotionIntentTarget{
        {0.0, 1.0, 3.5, 90.0}, {fixture.route.data(), 2, 1}};
    fixture.request.maxLinearSpeedMps = 0.5;
    auto params = scanParams();
    params.scan.voxelResolution = 0.05;
    nav_kernel::local::scan::Backend backend(params);
    LocalPlan plan;
    for (int tick = 0; tick < 100 && !plan.ready(); ++tick) {
      fixture.request.clock.timestampS += 0.01;
      plan = backend.tick(fixture.request);
    }
    ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
    const auto first = std::get<SplineTarget>(plan.target());
    const nav_kernel::SplineView firstView(first);
    const auto start = firstView.position(0.0);
    const auto endpoint = firstView.position(firstView.duration());
    ASSERT_NEAR(endpoint.x, start.x, 1e-6);
    ASSERT_NEAR(start.y - endpoint.y, 0.3, 1e-6);

    // The opposite side becomes free while W and its reference stay unchanged.
    setPocket(true);
    fixture.refreshCollision(3);
    double pastHalfTime = 0.0;
    for (int sample = 1; sample < 1000; ++sample) {
      const double time = firstView.duration() * sample / 1000.0;
      if (start.y - firstView.position(time).y > 0.16) {
        pastHalfTime = time;
        break;
      }
    }
    ASSERT_GT(pastHalfTime, 0.0);
    const auto plannedPosition = firstView.position(pastHalfTime);
    fixture.request.robot.pose.position = plannedPosition;
    fixture.request.robot.pose.position.y += 0.13;
    fixture.request.robot.kinematics.valid = true;
    fixture.request.robot.kinematics.linearVelocity = {0.0, -0.1, 0.0};
    ASSERT_LT(nav_kernel::distance3D(plannedPosition, endpoint), 0.2);
    ASSERT_GT(nav_kernel::distance3D(fixture.request.robot.pose.position, endpoint), 0.2);
    fixture.request.clock.timestampS = first.startTimeS + pastHalfTime;
    for (int tick = 0; tick < 3; ++tick) {
      fixture.request.clock.timestampS += 0.01;
      plan = backend.tick(fixture.request);
      ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
      EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_execute_trajectory");
      EXPECT_EQ(std::get<SplineTarget>(plan.target()).trajectoryId, first.trajectoryId);
      const nav_kernel::SplineView retained(std::get<SplineTarget>(plan.target()));
      EXPECT_LT(nav_kernel::distance3D(retained.position(retained.duration()), endpoint), 1e-6);
    }

    if (obstructTail) {
      // Deferring the progress timer must not suppress collision-timer replans.
      for (int x = -99; x < 99; ++x)
        fixture.bitmap.occupy({x * 0.05 + 0.025, 0.825, 0.525});
      fixture.refreshCollision(4);
      fixture.request.clock.timestampS += 0.01;
      const nav_kernel::local::scan::Grid blockedGrid(params, fixture.request);
      ASSERT_EQ(blockedGrid.segmentInflatedOccupancy(
                    firstView.position(fixture.request.clock.timestampS - first.startTimeS),
                    0.0, endpoint, 0.0), 1);
      const auto priorAttemptId = backend.debugSnapshot().scanAttempt.attemptId;
      plan = backend.checkCollision(fixture.request);
      const auto collisionDebug = backend.debugSnapshot();
      EXPECT_TRUE(collisionDebug.scanAttempt.attemptId > priorAttemptId ||
                  (plan.status() == nav_kernel::LocalPlanStatus::NearFieldStop &&
                   collisionDebug.searchReason == "scan_emergency_stop") ||
                  (plan.status() == nav_kernel::LocalPlanStatus::Blocked &&
                   collisionDebug.searchReason == "scan_local_target_blocked"))
          << collisionDebug.searchReason;
      if (plan.ready()) {
        ASSERT_NE(std::get<SplineTarget>(plan.target()).trajectoryId, first.trajectoryId);
        const nav_kernel::SplineView replacement(std::get<SplineTarget>(plan.target()));
        for (double t = 0.0; t < replacement.duration(); t += 0.01) {
          ASSERT_EQ(blockedGrid.segmentInflatedOccupancy(replacement.position(t), 0.0,
              replacement.position(std::min(t + 0.01, replacement.duration())), 0.0), 0);
        }
      }
      continue;
    }

    // Nominal completion is still 15 cm short in measured motion. Continue the
    // retained endpoint rather than treating the new-candidate 20 cm rule as arrival.
    fixture.request.robot.pose.position = endpoint;
    fixture.request.robot.pose.position.y += 0.15;
    fixture.request.clock.timestampS = first.startTimeS + firstView.duration() + 0.03;
    for (int tick = 0; tick < 30; ++tick) {
      fixture.request.clock.timestampS += 0.01;
      plan = backend.tick(fixture.request);
      if (plan.ready() && std::get<SplineTarget>(plan.target()).trajectoryId > first.trajectoryId)
        break;
    }
    ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
    const auto continuation = std::get<SplineTarget>(plan.target());
    ASSERT_GT(continuation.trajectoryId, first.trajectoryId);
    const nav_kernel::SplineView continuedView(continuation);
    EXPECT_LT(nav_kernel::distance3D(continuedView.position(0.0),
                                    fixture.request.robot.pose.position), 1e-6);
    EXPECT_LT(nav_kernel::distance3D(continuedView.velocity(0.0),
                                    fixture.request.robot.kinematics.linearVelocity), 1e-6);
    EXPECT_LT(nav_kernel::distance3D(continuedView.position(continuedView.duration()), endpoint), 1e-6);

    fixture.request.robot.pose.position = endpoint;
    fixture.request.robot.kinematics.linearVelocity = {};
    fixture.request.clock.timestampS = continuation.startTimeS + continuedView.duration() + 0.03;
    for (int tick = 0; tick < 30; ++tick) {
      fixture.request.clock.timestampS += 0.01;
      plan = backend.tick(fixture.request);
      if (plan.ready() && std::get<SplineTarget>(plan.target()).trajectoryId > continuation.trajectoryId)
        break;
    }
    ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason;
    ASSERT_GT(std::get<SplineTarget>(plan.target()).trajectoryId, continuation.trajectoryId);
    const nav_kernel::SplineView afterArrival(std::get<SplineTarget>(plan.target()));
    EXPECT_GT(afterArrival.position(afterArrival.duration()).y, endpoint.y + 0.2);
  }
}

TEST(ScanBackend, EndTimeAloneDoesNotFinishAnUnreachedReference) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto first = std::get<SplineTarget>(plan.target());
  fixture.request.robot.pose.position.x = 0.2;
  fixture.request.clock.timestampS = first.startTimeS + nav_kernel::SplineView(first).duration() + 0.3;
  for (int tick = 0; tick < 20; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
    if (plan.ready() && std::get<SplineTarget>(plan.target()).trajectoryId > first.trajectoryId)
      break;
  }
  ASSERT_TRUE(plan.ready());
  EXPECT_GT(std::get<SplineTarget>(plan.target()).trajectoryId, first.trajectoryId);
}

TEST(ScanBackend, ReachedReferenceWaitsWithoutStartingAnotherTrajectory) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto first = std::get<SplineTarget>(plan.target());
  fixture.request.robot.pose.position = fixture.route.back();
  fixture.request.clock.timestampS = first.startTimeS + nav_kernel::SplineView(first).duration() + 0.3;
  for (int tick = 0; tick < 20; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  EXPECT_EQ(std::get<SplineTarget>(plan.target()).trajectoryId, first.trajectoryId);
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_wait_target");
}

TEST(ScanBackend, FreshExecutionFeedbackFinishesReferenceDespiteOlderRequestClock) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += .01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto first = std::get<SplineTarget>(plan.target());
  const double duration = nav_kernel::SplineView(first).duration();
  fixture.request.robot.pose.position = fixture.route.back();
  fixture.request.clock.timestampS = first.startTimeS + .5 * duration;
  const nav_kernel::local::scan::ExecutionFeedback execution{
      first.startTimeS + duration + .3, first.trajectoryId, first.startTimeS, false};
  backend.setExecutionFeedback([execution] { return execution; });

  plan = backend.tick(fixture.request);
  ASSERT_TRUE(plan.ready());
  EXPECT_EQ(std::get<SplineTarget>(plan.target()).trajectoryId, first.trajectoryId);
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_wait_target");
}

TEST(ScanBackend, ExecutionProgressOverridesElapsedWallTimeAndKeepsFrozenPhase) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += .01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto first = std::get<SplineTarget>(plan.target());
  const nav_kernel::SplineView view(first);
  nav_kernel::local::scan::ExecutionFeedback execution{
      first.startTimeS + view.duration() + .3, first.trajectoryId, first.startTimeS, false};
  execution.progress = nav_kernel::LocalExecutionProgress{
      first.trajectoryId, execution.nowS - .05, 0.0};
  fixture.request.robot.pose.position = view.position(.05);
  backend.setExecutionFeedback([&execution] { return execution; });
  plan = backend.tick(fixture.request);
  ASSERT_TRUE(plan.ready());
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_execute_trajectory");

  execution.nowS += .01;
  execution.frozen = true;
  execution.progress = nav_kernel::LocalExecutionProgress{
      first.trajectoryId, execution.nowS - .5, 0.0};
  fixture.request.robot.pose.position = view.position(0.0);
  plan = backend.tick(fixture.request);
  ASSERT_TRUE(plan.ready());
  EXPECT_EQ(std::get<SplineTarget>(plan.target()).trajectoryId, first.trajectoryId);
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_execute_trajectory");

  execution.nowS += .01;
  execution.frozen = false;
  execution.progress = nav_kernel::LocalExecutionProgress{
      first.trajectoryId + 1, execution.nowS, view.duration()};
  fixture.request.robot.pose.position = view.position(.01);
  plan = backend.tick(fixture.request);
  ASSERT_TRUE(plan.ready());
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_execute_trajectory");

  execution.nowS += .01;
  execution.frozen = true;
  execution.progress = nav_kernel::LocalExecutionProgress{
      first.trajectoryId, execution.nowS, view.duration()};
  fixture.request.robot.pose.position = fixture.route.back();
  plan = backend.tick(fixture.request);
  ASSERT_TRUE(plan.ready());
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_wait_target");
}

TEST(ScanBackend, FailedContinuationDoesNotKeepFinishedSplineReady) {
  RequestFixture fixture({{0,0,.5},{5,0,.5}});
  auto params = scanParams();
  params.scan.planningHorizon = 1.5;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick=0; tick<20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += .01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const auto first = std::get<SplineTarget>(plan.target());
  const nav_kernel::SplineView view(first);
  fixture.request.robot.pose.position = view.position(view.duration());
  const double barrier = fixture.request.robot.pose.position.x + .5;
  for (int y=-49; y<49; ++y)
    fixture.bitmap.occupyInflated({static_cast<float>(barrier),y*.1F,.5F},.15,.2,.2);
  fixture.refreshCollision(2);
  fixture.request.clock.timestampS = first.startTimeS + view.duration() + .1;
  for (int tick=0; tick<12; ++tick) {
    fixture.request.clock.timestampS += .01;
    plan = backend.tick(fixture.request);
  }
  EXPECT_FALSE(plan.ready()) << backend.debugSnapshot().searchReason;
  EXPECT_EQ(plan.status(),nav_kernel::LocalPlanStatus::Blocked);
  fixture.bitmap.clear();
  fixture.refreshCollision(3);
  for (int tick=0; tick<20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += .01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  EXPECT_GT(std::get<SplineTarget>(plan.target()).trajectoryId,first.trajectoryId);
}

TEST(ScanBackend, BlockedReferenceDoesNotReplaceFsmTimerCallbacks) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  const SplineTarget initial = std::get<SplineTarget>(plan.target());
  for (int x = -1; x <= 21; ++x)
    fixture.bitmap.occupyInflated({x * 0.1F, 0.0F, 0.5F}, 0.15, 0.1, 0.1);
  fixture.refreshCollision(2);
  fixture.request.objective = RouteTarget{{fixture.route.data(), 2, 2}};
  fixture.request.clock.timestampS += 0.01;
  plan = backend.tick(fixture.request);
  ASSERT_TRUE(plan.ready());
  EXPECT_EQ(std::get<SplineTarget>(plan.target()).trajectoryId, initial.trajectoryId);

  // The subscriber event must not suppress the next timer callback. That
  // callback invalidates the old spline when no usable target remains.
  fixture.request.clock.timestampS =
      initial.startTimeS + nav_kernel::SplineView(initial).duration() + 0.3;
  plan = backend.tick(fixture.request);
  EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Blocked);
  EXPECT_EQ(backend.debugSnapshot().searchReason, "scan_local_target_blocked");

  fixture.bitmap.clear();
  fixture.refreshCollision(3);
  fixture.request.objective = RouteTarget{{fixture.route.data(), 2, 3}};
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready());
  EXPECT_GT(std::get<SplineTarget>(plan.target()).trajectoryId, initial.trajectoryId);
}

TEST(ScanBackend, BlockedReferenceRecoversOnMapUpdateWithoutAnotherClick) {
  RequestFixture fixture({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}});
  for (int x = -1; x <= 21; ++x)
    fixture.bitmap.occupyInflated({x * 0.1F, 0.0F, 0.5F}, 0.15, 0.1, 0.1);
  fixture.refreshCollision(2);
  nav_kernel::local::scan::Backend backend(scanParams());
  LocalPlan plan;
  for (int tick = 0; tick < 10; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  EXPECT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Blocked);

  fixture.bitmap.clear();
  fixture.refreshCollision(3);
  for (int tick = 0; tick < 20 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    plan = backend.tick(fixture.request);
  }
  EXPECT_TRUE(plan.ready());
}

TEST(ScanGrid, BoundaryDepartureLeavesOnlyInitialCellAndChecksBothCylinders) {
  RequestFixture f({{.025, .025, .525}, {.375, .025, .525}}, .05);
  auto params = scanParams();
  params.scan.voxelResolution = .05;
  params.scan.cylinderOffset = .25;
  f.bitmap.occupy({.275, .025, .525});
  f.refreshCollision(2);
  nav_kernel::local::scan::Grid grid(params, f.request);
  const auto pose = f.request.robot.pose;
  EXPECT_FALSE(grid.obstacleFree(pose.position, pose.yaw));
  EXPECT_TRUE(grid.boundaryDepartureFree(pose, {.025, -.325, .525}));
  EXPECT_FALSE(grid.boundaryDepartureFree(pose, {.025, .026, .525}));
  EXPECT_FALSE(grid.boundaryDepartureFree(pose, {.025, -.325, .725}));

  // A second occupied cell along the exit is forbidden, even if the endpoint is free.
  f.bitmap.occupy({.275, -.025, .525});
  EXPECT_FALSE(grid.boundaryDepartureFree(pose, {.025, -.325, .525}));
  f.bitmap.clear();
  f.bitmap.occupy({.275, .025, .525});
  f.bitmap.occupy({-.225, -.125, .525});
  EXPECT_FALSE(grid.boundaryDepartureFree(pose, {.025, -.325, .525}));
}

TEST(ScanGrid, BoundaryDepartureRejectsActualMotionIntoAnObstacle) {
  RequestFixture f({{.025, .025, .525}, {.025, -.325, .525}}, .05);
  auto params = scanParams();
  params.scan.voxelResolution = .05;
  params.scan.cylinderOffset = .25;
  f.bitmap.occupy({.275, .025, .525});
  f.bitmap.occupy({.325, .025, .525});
  f.refreshCollision(2);
  nav_kernel::local::scan::Grid grid(params, f.request);
  auto pose = f.request.robot.pose;
  pose.yaw = 0;
  const nav_kernel::Twist command{0, -.15, 0};
  EXPECT_TRUE(grid.boundaryDepartureMotionFree(pose, command, {}, .15, .35, .5));
  EXPECT_TRUE(grid.boundaryDepartureMotionFree(pose, command, {0, -.1, 0}, .15, .35, .5));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(pose, command, {.1, 0, 0}, .15, .35, .5));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(pose, command, {0, -.2, 0}, .15, .35, .5));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(pose, {0, -.3, 0}, {}, .15, .35, .5));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(pose, {0, -.15, .1}, {}, .15, .35, .5));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(pose, command, {0, 0, .1}, .15, .35, .5));
  // A larger configured command still needs a complete exit and braking room.
  EXPECT_TRUE(grid.boundaryDepartureMotionFree(pose, {0, -.3, 0}, {0, -.25, 0}, .3, .35, .5));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(pose, {0, -.3, 0}, {}, .3, .35, .1));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(pose, {0, -.3, 0}, {}, .3, 1.0, .5));
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(pose, {0, -.3, 0}, {.1, 0, 0}, .3, .35, .5));
  f.bitmap.occupy({.275, -.025, .525});
  EXPECT_FALSE(grid.boundaryDepartureMotionFree(pose, {0, -.3, 0}, {}, .3, .35, .5));
}

TEST(ScanCollisionEvidence, UsesMeasuredVoxelsAndClearsOnReset) {
  for (bool measured : {false, true}) {
    SCOPED_TRACE(measured);
    RequestFixture fixture({{0, 0, .5}, {2, 0, .5}});
    for (double x = -.25; x < .3; x += .1)
      for (double y = -.25; y < .3; y += .1)
        for (double z = .25; z < .8; z += .1)
          fixture.bitmap.occupy({x, y, z});
    fixture.refreshCollision(2);
    auto &collision = fixture.request.environment.collision;
    CollisionBitmap observed({-5,-5,-1}, {5,5,2}, .1);
    if (measured) observed.occupy({.05,.05,.55});
    const auto bits = observed.view();
    collision.measuredOccupiedStorage = std::make_shared<const std::vector<std::uint8_t>>(
        bits.inflatedBits, bits.inflatedBits + bits.inflatedBytes);
    nav_kernel::local::scan::Backend backend(scanParams());
    for (int i = 0; i < 20 && !backend.debugSnapshot().collisionEvidence; ++i) {
      fixture.request.clock.timestampS += .01;
      (void)backend.tick(fixture.request);
    }
    const auto evidence = backend.debugSnapshot().collisionEvidence;
    ASSERT_TRUE(evidence);
    ASSERT_TRUE(evidence->measuredPoints);
    EXPECT_EQ(evidence->measuredPoints->size(), measured ? 1U : 0U);
    EXPECT_EQ(evidence->collisionGeneration, 2U);
    EXPECT_EQ(evidence->collisionObservationSequence, 2U);
    if (measured) {
      EXPECT_NEAR(evidence->measuredPoints->front().x, .05, 1e-6);
      EXPECT_NEAR(evidence->measuredPoints->front().z, .55, 1e-6);
    }
    backend.reset();
    EXPECT_FALSE(backend.debugSnapshot().collisionEvidence);
  }
}

TEST(ScanCollisionEvidence, EvidenceUsesCylinderBoundsIndependentOfBodyParts) {
  namespace collision_geometry = lingtu::nav::collision;
  for (bool observed : {false, true}) {
    SCOPED_TRACE(observed);
    RequestFixture fixture({{0,0,.5},{2,0,.5}}, .05);
    auto body = std::make_shared<collision_geometry::Body>();
    body->margin_m = 0;
    collision_geometry::Part base;
    base.link = "base";
    base.center.z = .07;
    base.half_size = {.48,.255,.17};
    body->parts.push_back(base);
    collision_geometry::Part raised;
    raised.link = "raised";
    raised.center.z = .65;
    raised.half_size = {.05,.05,.05};
    body->parts.push_back(raised);
    if (observed) {
      auto configured = std::make_shared<collision_geometry::Body>();
      configured->margin_m = 0;
      base.center.z = 0;
      base.half_size = {.1,.1,.1};
      configured->parts.push_back(base);
      fixture.request.body = configured;
      fixture.request.observedBody = body;
    } else {
      fixture.request.body = body;
    }
    fixture.request.bodyHeading = 0;
    const Vec3 corner{.475,.225,.525};
    const Vec3 upper{.025,.025,1.175};
    const Vec3 front{.175,.025,.525};
    fixture.bitmap.occupy(front);
    fixture.bitmap.occupy(corner);
    fixture.bitmap.occupy(upper);
    fixture.refreshCollision(2);
    auto &collision = fixture.request.environment.collision;
    collision.measuredOccupiedStorage =
        std::make_shared<const std::vector<std::uint8_t>>(
            collision.inflatedBits, collision.inflatedBits + collision.inflatedBytes);
    collision.knownFreeStorage =
        std::make_shared<const std::vector<std::uint8_t>>(collision.inflatedBytes,255U);
    auto params = scanParams();
    params.scan.voxelResolution = .05;
    params.scan.cylinderRadius = .25;
    params.scan.cylinderOffset = .19;
    const nav_kernel::local::scan::Grid grid(params,fixture.request);
    ASSERT_TRUE(grid.valid()) << grid.reason();
    ASSERT_EQ(grid.inflatedOccupancy(fixture.route.front(),0),1);
    nav_kernel::local::scan::Backend backend(params);
    for (int tick = 0; tick < 20 && !backend.debugSnapshot().collisionEvidence; ++tick) {
      fixture.request.clock.timestampS += .01;
      (void)backend.tick(fixture.request);
    }
    const auto evidence = backend.debugSnapshot().collisionEvidence;
    ASSERT_TRUE(evidence) << backend.debugSnapshot().searchReason << ':'
                          << backend.debugSnapshot().scanAttempt.reason;
    ASSERT_TRUE(evidence->measuredPoints);
    ASSERT_LE(nav_kernel::distance3D(evidence->rejectedPosition,fixture.route.front()),1e-8);
    const auto contains = [&](const Vec3 &expected) {
      return std::any_of(evidence->measuredPoints->begin(),evidence->measuredPoints->end(),
          [&](const Vec3 &point) { return nav_kernel::distance3D(point,expected)<1e-8; });
    };
    EXPECT_TRUE(contains(front));
    EXPECT_FALSE(contains(corner));
    EXPECT_FALSE(contains(upper));
  }
}

TEST(ScanBackend, RouteGuideKeepsCornerExitBrakeExecutable) {
  namespace collision = lingtu::nav::collision;
  // Translate the first wall of the native 30 m detour by -10 m. The guide
  // stays above it before turning down; the observed moving boundary comes
  // from the last unblocked handoff, before the benchmark lost tracking.
  RequestFixture fixture({{-2,1.4,.35},{.8,1.4,.35},{.9,1.4,.35},
                          {2.3,0,.35},{4,0,.35}});
  fixture.request.robot.pose = {{.325167823023051,1.2190217216502166,.35},0};
  fixture.request.robot.kinematics.valid = true;
  fixture.request.robot.kinematics.linearVelocity = {.36381182176712357,.06597119654211449,0};
  fixture.request.bodyHeading = 0;
  fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
  fixture.request.maxLinearSpeedMps = .6;
  fixture.request.supportHeightM = .35;
  fixture.request.supportHeightToleranceM = .01;
  auto body = std::make_shared<collision::Body>();
  body->margin_m = 0;
  collision::Part part;
  part.center = {0,0,.07};
  part.half_size = {.48,.255,.17};
  body->parts.push_back(part);
  fixture.request.body = body;
  for (int y = -50; y < 50; ++y)
    for (int x = -50; x < 50; ++x)
      fixture.bitmap.occupy({(x+.5)*.1,(y+.5)*.1,-.05});
  for (int z = 0; z < 14; ++z)
    for (int y = -40; y < 8; ++y)
      for (int x = 0; x < 6; ++x)
        fixture.bitmap.occupy({(x+.5)*.1,(y+.5)*.1,(z+.5)*.1});
  fixture.refreshCollision(2);
  const auto &map = fixture.request.environment.collision;
  fixture.request.environment.collision.measuredOccupiedStorage =
      std::make_shared<const std::vector<std::uint8_t>>(
          map.inflatedBits,map.inflatedBits+map.inflatedBytes);
  fixture.request.environment.collision.knownFreeStorage =
      std::make_shared<const std::vector<std::uint8_t>>(map.inflatedBytes,255U);
  auto params = scanParams();
  params.scan.maxVelocity = .6;
  params.scan.maxAcceleration = 2;
  params.scan.maxBodyVx = .6;
  params.scan.maxBodyVy = .35;
  nav_kernel::local::scan::Backend backend(params);
  LocalPlan plan;
  for (int tick = 0; tick < 30 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += .01;
    plan = backend.tick(fixture.request);
  }
  ASSERT_TRUE(plan.ready()) << backend.debugSnapshot().searchReason << ':'
                           << backend.debugSnapshot().scanAttempt.reason;
  const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
  ASSERT_TRUE(spline.valid());
  EXPECT_LE(nav_kernel::distance3D(spline.position(0),fixture.request.robot.pose.position),1e-8);
  EXPECT_LE(nav_kernel::distance3D(spline.velocity(0),
                                  fixture.request.robot.kinematics.linearVelocity),1e-8);
  const nav_kernel::local::scan::Grid grid(params,fixture.request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  int blockedBrakingStates = 0;
  double firstBlockedTime = -1;
  nav_kernel::Vec3 firstBlockedPosition{};
  for (double t = 0; t < spline.duration(); t += .02) {
    const auto position = spline.position(t);
    if (position.x > 1.4) break;
    const auto velocity = spline.velocity(t);
    ASSERT_EQ(grid.inflatedOccupancy(position,0),0);
    if (grid.brakingOccupancy({position,0},{velocity.x,velocity.y,0},.35,.5,3) != 0) {
      if (blockedBrakingStates == 0) {
        firstBlockedTime = t;
        firstBlockedPosition = position;
      }
      ++blockedBrakingStates;
    }
  }
  EXPECT_EQ(blockedBrakingStates,0)
      << "first blocked brake at t=" << firstBlockedTime << " position="
      << firstBlockedPosition.x << ',' << firstBlockedPosition.y;
}

TEST(ScanReplanFsm, SimilarSafeReplacementKeepsExecutingTrajectory) {
  using namespace nav_kernel::local::scan::upstream;
  ScanAttemptFixture fixture;
  ScanReplanParams params;
  params.navigationMode = ScanNavigationMode::REFERENCE_PATH;
  params.planningHorizon = 3.5;
  params.bodyHeight = 0;
  params.noReplanThreshold = .1;
  params.replanThreshold = .1;
  PlannerFsm fsm(fixture.manager, params);
  FsmInput input;
  input.nowS = 1;
  input.odometry = FsmOdometry{};
  input.odometry->position = {0,0,.5};
  input.referencePath = std::vector<Eigen::Vector3d>{{0,0,.5},{2,0,.5}};
  ASSERT_TRUE(fsm.tick(input).targetAccepted);
  input.referencePath.reset();
  FsmOutput output;
  for (int tick=0;tick<8 && !output.trajectory;++tick) {
    input.nowS += PlannerFsm::kTickPeriodS;
    output = fsm.tick(input);
  }
  ASSERT_TRUE(output.trajectory);
  const auto previous = fixture.manager.local_data_;
  input.nowS = previous.start_time_ + .2 * previous.duration_;
  input.executingTrajectoryId = previous.traj_id_;
  input.executionStartTimeS = previous.start_time_;
  input.handoffLeadTimeS = .2;
  const double elapsed = input.nowS - previous.start_time_;
  input.odometry->position = previous.position_traj_.evaluateDeBoorT(elapsed);
  input.odometry->velocity = previous.velocity_traj_.evaluateDeBoorT(elapsed);
  ASSERT_EQ(fsm.tick(input).state, ScanReplanState::REPLAN_TRAJ);
  input.nowS += .01;
  output = fsm.tick(input);
  ASSERT_TRUE(fixture.manager.reboundDebug().success)
      << fixture.manager.reboundDebug().reason;
  std::cout << "equivalent replan: offered=" << output.handoffPending
            << ", executing=" << previous.traj_id_ << ", proposed="
            << (output.trajectory ? output.trajectory->trajectoryId : previous.traj_id_) << '\n';
  if (output.trajectory) {
    const auto &candidate = *output.trajectory;
    const double interval = candidate.knots(4) - candidate.knots(3);
    const UniformBspline curve(candidate.positionPoints,3,interval);
    EXPECT_GE(curve.getTimeSum(), .9 * (previous.duration_ -
        (input.nowS + *input.handoffLeadTimeS - previous.start_time_)));
  }
  EXPECT_FALSE(output.handoffPending);
  EXPECT_FALSE(output.trajectory);
  EXPECT_EQ(fixture.manager.local_data_.traj_id_, previous.traj_id_);
  EXPECT_DOUBLE_EQ(fixture.manager.local_data_.start_time_, previous.start_time_);
  EXPECT_LT((fixture.manager.local_data_.position_traj_.getControlPoint() -
             previous.position_traj_.getControlPoint()).norm(), 1e-12);
  // Keeping a safe tail must not make the timer optimize it again next tick.
  input.nowS += .01;
  EXPECT_EQ(fsm.tick(input).state, ScanReplanState::EXEC_TRAJ);
}

TEST(ScanReplanFsm, ShortRoutePreparesContinuationBeforeItsStoppingTail) {
  using namespace nav_kernel::local::scan::upstream;
  for(const double solveDelay : {.05,.10}) {
    SCOPED_TRACE(solveDelay);
    ScanAttemptFixture fixture;
    fixture.manager.pp_.preserve_boundary_=true;
    ScanReplanParams params;
    params.navigationMode=ScanNavigationMode::REFERENCE_PATH;
    params.planningHorizon=.5;
    params.noReplanThreshold=.1;
    params.replanThreshold=1;
    PlannerFsm fsm(fixture.manager,params);
    FsmInput input;
    input.nowS=1;
    input.odometry=FsmOdometry{};
    input.odometry->position={0,0,.5};
    input.referencePath=std::vector<Eigen::Vector3d>{{0,0,.5},{3,0,.5}};
    ASSERT_TRUE(fsm.tick(input).targetAccepted);
    input.referencePath.reset();
    FsmOutput output;
    for(int tick=0;tick<8 && !output.trajectory;++tick) {
      input.nowS+=.01;
      output=fsm.tick(input);
    }
    ASSERT_TRUE(output.trajectory)<<fixture.manager.reboundDebug().reason;
    const auto previous=fixture.manager.local_data_;
    const double lead=1.5*solveDelay+2*PlannerFsm::kTickPeriodS;
    const double triggerTime=previous.duration_-2*lead;
    ASSERT_GT(triggerTime,0);
    input.nowS=previous.start_time_+triggerTime;
    input.executingTrajectoryId=previous.traj_id_;
    input.executionStartTimeS=previous.start_time_;
    input.handoffLeadTimeS=lead;
    input.odometry->position=previous.position_traj_.evaluateDeBoorT(triggerTime);
    input.odometry->velocity=previous.velocity_traj_.evaluateDeBoorT(triggerTime);
    ASSERT_LT((input.odometry->position-previous.start_pos_).norm(),params.replanThreshold);
    ASSERT_EQ(fsm.tick(input).state,ScanReplanState::REPLAN_TRAJ);
    input.nowS+=.01;
    const double boundary=input.nowS+lead-previous.start_time_;
    input.odometry->position=previous.position_traj_.evaluateDeBoorT(triggerTime+.01);
    input.odometry->velocity=previous.velocity_traj_.evaluateDeBoorT(triggerTime+.01);
    fixture.manager.setTimeSource([&] {return input.nowS+solveDelay;});
    output=fsm.tick(input);
    ASSERT_TRUE(output.handoffPending)<<fixture.manager.reboundDebug().reason;
    ASSERT_TRUE(output.trajectory);
    const auto& offered=*output.trajectory;
    EXPECT_LT(offered.startTimeS,previous.start_time_+previous.duration_);
    const double interval=offered.knots(4)-offered.knots(3);
    UniformBspline next(offered.positionPoints,offered.order,interval);
    next.setKnot(offered.knots);
    EXPECT_LT((next.evaluateDeBoorT(0)-previous.position_traj_.evaluateDeBoorT(boundary)).norm(),1e-6);
    EXPECT_LT((next.getDerivative().evaluateDeBoorT(0)-previous.velocity_traj_.evaluateDeBoorT(boundary)).norm(),1e-6);
    EXPECT_LT((next.getDerivative().getDerivative().evaluateDeBoorT(0)-
               previous.acceleration_traj_.evaluateDeBoorT(boundary)).norm(),1e-6);
    EXPECT_GT(next.evaluateDeBoorT(next.getTimeSum()).x(),
              previous.position_traj_.evaluateDeBoorT(previous.duration_).x()+.1);
    RecordProperty("solve_delay_ms",solveDelay*1000);
    RecordProperty("remaining_at_request_s",2*lead);
    RecordProperty("handoff_before_old_end_s",previous.duration_-boundary);
    std::cout<<"CONTINUATION solve_ms="<<solveDelay*1000
             <<" remaining_at_request_s="<<2*lead
             <<" handoff_before_old_end_s="<<previous.duration_-boundary<<'\n';
  }
}

TEST(ScanReplanFsm, TrackingLagReplansFromMeasuredMotionWithSolverDelay) {
  using namespace nav_kernel::local::scan::upstream;
  for(const double solveDelay : {.05,.10}) {
    SCOPED_TRACE(solveDelay);
    ScanAttemptFixture fixture;
    fixture.manager.pp_.preserve_boundary_=true;
    ScanReplanParams params;
    params.navigationMode=ScanNavigationMode::REFERENCE_PATH;
    params.planningHorizon=3.5;
    params.noReplanThreshold=.1;
    params.replanThreshold=1;
    PlannerFsm fsm(fixture.manager,params);
    FsmInput input;
    input.nowS=1;
    input.odometry=FsmOdometry{};
    input.odometry->position={0,0,.5};
    input.referencePath=std::vector<Eigen::Vector3d>{{0,0,.5},{3,0,.5}};
    ASSERT_TRUE(fsm.tick(input).targetAccepted);
    input.referencePath.reset();
    FsmOutput output;
    for(int tick=0;tick<8 && !output.trajectory;++tick) {
      input.nowS+=.01;
      output=fsm.tick(input);
    }
    ASSERT_TRUE(output.trajectory);
    const auto previous=fixture.manager.local_data_;
    const double elapsed=.5*previous.duration_;
    input.nowS=previous.start_time_+elapsed;
    input.executingTrajectoryId=previous.traj_id_;
    input.executionStartTimeS=previous.start_time_;
    input.handoffLeadTimeS=.3;
    input.odometry->position=previous.position_traj_.evaluateDeBoorT(elapsed)-Eigen::Vector3d(.3,0,0);
    input.odometry->velocity={.1,0,0};
    ASSERT_EQ(fsm.tick(input).state,ScanReplanState::REPLAN_TRAJ);
    input.nowS+=.01;
    input.odometry->position.x()+=.001;
    const Eigen::Vector3d observed=input.odometry->position;
    fixture.manager.setTimeSource([&] {return input.nowS+solveDelay;});
    output=fsm.tick(input);
    ASSERT_TRUE(output.trajectory)<<fixture.manager.reboundDebug().reason;
    EXPECT_FALSE(output.handoffPending);
    const auto& next=fixture.manager.local_data_;
    EXPECT_LT((next.position_traj_.evaluateDeBoorT(0)-observed).norm(),1e-6);
    EXPECT_LT((next.velocity_traj_.evaluateDeBoorT(0)-input.odometry->velocity).norm(),1e-6);
    const Eigen::Vector3d measuredAtCompletion=observed+solveDelay*input.odometry->velocity;
    EXPECT_LT((next.position_traj_.evaluateDeBoorT(solveDelay)-measuredAtCompletion).norm(),
              fixture.manager.pp_.ctrl_pt_dist);
    const auto target=[](const LocalTrajData& trajectory) {
      SplineTarget result;
      const auto controls=trajectory.position_traj_.getControlPoint();
      result.controls.reserve(controls.cols());
      for(Eigen::Index i=0;i<controls.cols();++i) {
        const auto point=controls.col(i);
        result.controls.push_back({point.x(),point.y(),point.z()});
      }
      const auto knots=trajectory.position_traj_.getKnot();
      result.knots.assign(knots.data(),knots.data()+knots.size());
      result.order=3;
      result.trajectoryId=trajectory.traj_id_;
      result.startTimeS=trajectory.start_time_;
      result.maxLinearSpeedMps=.5;
      return result;
    };
    nav_kernel::Follower follower;
    nav_kernel::FollowerState state;
    state.params.maxSpeed=.5;
    state.params.maxAccel=2;
    state.params.nominalDt=.02;
    state.params.spline.maxVx=state.params.spline.maxVy=.5;
    state.desiredHeading=0;
    const auto oldTarget=LocalPlan::spline(target(previous));
    const double observedTime=input.nowS-previous.start_time_;
    const Eigen::Vector3d lagAtRequest=
        previous.position_traj_.evaluateDeBoorT(observedTime)-observed;
    nav_kernel::Twist command;
    // Warm the executing reference clock before injecting tracking lag.
    // A newly constructed follower deliberately starts its first curve at zero.
    for(double t=0;t<observedTime;t+=.02) {
      state.currentTime=previous.start_time_+t;
      const Eigen::Vector3d measured=previous.position_traj_.evaluateDeBoorT(t)-
          (t/observedTime)*lagAtRequest;
      state.vehicleRelative={measured.x(),measured.y(),measured.z()};
      command=follower.follow(oldTarget,state).cmd;
    }
    state.currentTime=input.nowS;
    state.vehicleRelative={observed.x(),observed.y(),observed.z()};
    command=follower.follow(oldTarget,state).cmd;
    for(double t=.02;t<=solveDelay+.001;t+=.02) {
      state.currentTime=input.nowS+t;
      state.vehicleRelative.x=observed.x()+.1*t;
      command=follower.follow(oldTarget,state).cmd;
    }
    state.currentTime+=.02;
    state.vehicleRelative.x=observed.x()+.1*(state.currentTime-input.nowS);
    const auto replaced=follower.follow(LocalPlan::spline(target(next)),state);
    const double delta=std::hypot(replaced.cmd.vx-command.vx,replaced.cmd.vy-command.vy);
    EXPECT_LE(delta,state.params.maxAccel*.02+1e-8);
    EXPECT_GT(replaced.cmd.vx,0);
    EXPECT_FALSE(replaced.executionFrozen);
    std::cout<<"TRACKING_LAG solve_ms="<<solveDelay*1000
             <<" old_error_m="<<.3
             <<" new_start_error_m="<<(next.position_traj_.evaluateDeBoorT(0)-observed).norm()
             <<" error_at_completion_m="<<(next.position_traj_.evaluateDeBoorT(solveDelay)-measuredAtCompletion).norm()
             <<" command_change_mps="<<delta<<'\n';
  }
}

TEST(ScanReplanFsm, NewReferenceStillReplacesRetainedTrajectory) {
  using namespace nav_kernel::local::scan::upstream;
  ScanAttemptFixture fixture;
  ScanReplanParams params;
  params.navigationMode = ScanNavigationMode::REFERENCE_PATH;
  params.planningHorizon = 3.5;
  params.bodyHeight = 0;
  params.noReplanThreshold = .1;
  params.replanThreshold = .2;
  PlannerFsm fsm(fixture.manager, params);
  FsmInput input;
  input.nowS = 1;
  input.odometry = FsmOdometry{};
  input.odometry->position = {0,0,.5};
  input.referencePath = std::vector<Eigen::Vector3d>{{0,0,.5},{2,0,.5}};
  ASSERT_TRUE(fsm.tick(input).targetAccepted);
  input.referencePath.reset();
  FsmOutput output;
  for (int tick=0;tick<8 && !output.trajectory;++tick) {
    input.nowS += .01;
    output = fsm.tick(input);
  }
  ASSERT_TRUE(output.trajectory);
  const auto previous = fixture.manager.local_data_;
  input.nowS = previous.start_time_ + .2 * previous.duration_;
  const double elapsed = input.nowS - previous.start_time_;
  input.odometry->position = previous.position_traj_.evaluateDeBoorT(elapsed);
  input.odometry->velocity = previous.velocity_traj_.evaluateDeBoorT(elapsed);
  input.executingTrajectoryId = previous.traj_id_;
  input.executionStartTimeS = previous.start_time_;
  input.handoffLeadTimeS = .2;
  input.referencePath = std::vector<Eigen::Vector3d>{input.odometry->position,{2,0,.5}};
  ASSERT_TRUE(fsm.tick(input).targetAccepted);
  input.referencePath.reset();
  for (int tick=0;tick<8 && !output.handoffPending;++tick) {
    input.nowS += .01;
    output = fsm.tick(input);
  }
  ASSERT_TRUE(output.handoffPending);
  ASSERT_TRUE(output.trajectory);
  EXPECT_GT(output.trajectory->trajectoryId, previous.traj_id_);
}

TEST(ScanReplanFsm, FailedReplacementKeepsSafeExecutingTrajectory) {
  using namespace nav_kernel::local::scan::upstream;
  ScanAttemptFixture fixture;
  ScanReplanParams params;
  params.navigationMode = ScanNavigationMode::REFERENCE_PATH;
  params.planningHorizon = 3.5;
  params.bodyHeight = 0;
  params.noReplanThreshold = .1;
  params.replanThreshold = .2;
  PlannerFsm fsm(fixture.manager, params);
  FsmInput input;
  input.nowS = 1;
  input.odometry = FsmOdometry{};
  input.odometry->position = {0,0,.5};
  input.referencePath = std::vector<Eigen::Vector3d>{{0,0,.5},{2,0,.5}};
  ASSERT_TRUE(fsm.tick(input).targetAccepted);
  input.referencePath.reset();
  FsmOutput output;
  for (int tick=0;tick<8 && !output.trajectory;++tick) {
    input.nowS += PlannerFsm::kTickPeriodS;
    output = fsm.tick(input);
  }
  ASSERT_TRUE(output.trajectory);
  const auto previous = fixture.manager.local_data_;
  input.nowS = previous.start_time_ + .4 * previous.duration_;
  input.executingTrajectoryId = previous.traj_id_;
  input.executionStartTimeS = previous.start_time_;
  input.handoffLeadTimeS = .2;
  const double elapsed = input.nowS - previous.start_time_;
  input.odometry->position = previous.position_traj_.evaluateDeBoorT(elapsed);
  input.odometry->velocity = previous.velocity_traj_.evaluateDeBoorT(elapsed);
  ASSERT_EQ(fsm.tick(input).state, ScanReplanState::REPLAN_TRAJ);
  input.nowS += .01;
  output = fsm.tick(input);
  ASSERT_FALSE(fixture.manager.reboundDebug().success)
      << fixture.manager.reboundDebug().reason;
  EXPECT_EQ(output.state, ScanReplanState::EXEC_TRAJ);
  EXPECT_FALSE(output.handoffPending);
  EXPECT_FALSE(output.trajectory);
  EXPECT_EQ(fixture.manager.local_data_.traj_id_, previous.traj_id_);
  EXPECT_DOUBLE_EQ(fixture.manager.local_data_.start_time_, previous.start_time_);
  EXPECT_LT((fixture.manager.local_data_.position_traj_.getControlPoint() -
             previous.position_traj_.getControlPoint()).norm(), 1e-12);
  // Keeping a safe tail must not make the timer optimize it again next tick.
  input.nowS += .01;
  EXPECT_EQ(fsm.tick(input).state, ScanReplanState::EXEC_TRAJ);
}

TEST(ScanReplanFsm, NewObstacleInvalidatesEquivalentOldTail) {
  using namespace nav_kernel::local::scan::upstream;
  ScanAttemptFixture fixture;
  ScanReplanParams params;
  params.navigationMode = ScanNavigationMode::REFERENCE_PATH;
  params.planningHorizon = 3.5;
  params.noReplanThreshold = .1;
  params.replanThreshold = .2;
  PlannerFsm fsm(fixture.manager, params);
  FsmInput input;
  input.nowS = 1;
  input.odometry = FsmOdometry{};
  input.odometry->position = {0,0,.5};
  input.referencePath = std::vector<Eigen::Vector3d>{{0,0,.5},{2,0,.5}};
  ASSERT_TRUE(fsm.tick(input).targetAccepted);
  input.referencePath.reset();
  FsmOutput output;
  for (int tick=0;tick<8 && !output.trajectory;++tick) {
    input.nowS += .01;
    output = fsm.tick(input);
  }
  ASSERT_TRUE(output.trajectory);
  const auto previous = fixture.manager.local_data_;
  input.nowS = previous.start_time_ + .4 * previous.duration_;
  const double elapsed = input.nowS - previous.start_time_;
  input.odometry->position = previous.position_traj_.evaluateDeBoorT(elapsed);
  input.odometry->velocity = previous.velocity_traj_.evaluateDeBoorT(elapsed);
  input.executingTrajectoryId = previous.traj_id_;
  input.executionStartTimeS = previous.start_time_;
  input.handoffLeadTimeS = .2;
  const Eigen::Vector3d obstacle = previous.position_traj_.evaluateDeBoorT(.85 * previous.duration_);
  fixture.request.bitmap.occupyInflated({static_cast<float>(obstacle.x()),
      static_cast<float>(obstacle.y()),static_cast<float>(obstacle.z())}, .1,.1,.1);
  fixture.request.refreshCollision(2);
  fixture.grid = nav_kernel::local::scan::Grid(scanParams(), fixture.request.request);
  ASSERT_EQ(fsm.tick(input).state, ScanReplanState::REPLAN_TRAJ);
  input.nowS += .01;
  output = fsm.tick(input);
  // A new safe candidate may be offered, but an occupied old tail may not be
  // accepted as the successful outcome of a failed replacement attempt.
  EXPECT_TRUE(output.handoffPending || output.state != ScanReplanState::EXEC_TRAJ);
  if (output.handoffPending) {
    ASSERT_TRUE(output.trajectory);
    EXPECT_GT(output.trajectory->trajectoryId, previous.traj_id_);
  }
}

TEST(ScanTask, NewRecoveryGoalDoesNotInheritPreviousFailure) {
  RequestFixture fixture({{0,0,.5}, {1.6,0,.5}});
  for (int y=-50; y<50; ++y)
    for (int z=-10; z<20; ++z)
      fixture.bitmap.occupy({.85,(y+.5)*.1,(z+.5)*.1});
  fixture.refreshCollision(2);
  fixture.request.bodyHeading = 0;
  auto body = std::make_shared<lingtu::nav::collision::Body>();
  body->margin_m = 0;
  lingtu::nav::collision::Part part;
  part.half_size = {.3,.15,.1};
  body->parts.push_back(part);
  fixture.request.body = body;
  auto &collision = fixture.request.environment.collision;
  collision.measuredOccupiedStorage = std::make_shared<const std::vector<std::uint8_t>>(
      collision.inflatedBits, collision.inflatedBits + collision.inflatedBytes);
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  nav_kernel::LocalPlan plan;
  nav_kernel::local::scan::Update update;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  do {
    fixture.request.clock.timestampS += .005;
    update = task.update(fixture.request);
    plan = update.plan;
    if (plan.status() == nav_kernel::LocalPlanStatus::Blocked ||
        plan.status() == nav_kernel::LocalPlanStatus::NoPath) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  } while (std::chrono::steady_clock::now() < deadline);
  ASSERT_TRUE(plan.status() == nav_kernel::LocalPlanStatus::Blocked ||
              plan.status() == nav_kernel::LocalPlanStatus::NoPath);

  fixture.route.back() = {-1.2,0,.5};
  fixture.request.objective = nav_kernel::RouteTarget{{fixture.route.data(), 2, 2}};
  plan = task.update(fixture.request).plan;
  EXPECT_TRUE(plan.status() == nav_kernel::LocalPlanStatus::Pending || plan.ready());
  const auto recoveryDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!plan.ready() && std::chrono::steady_clock::now() < recoveryDeadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    fixture.request.clock.timestampS += .005;
    update = task.update(fixture.request);
    plan = update.plan;
  }
  EXPECT_TRUE(plan.ready()) << update.debug.searchReason << ':' << update.debug.scanAttempt.reason;
  if (plan.ready()) {
    const nav_kernel::SplineView spline(std::get<SplineTarget>(plan.target()));
    EXPECT_NEAR(spline.position(spline.duration()).x, -1.2, .01);
    EXPECT_NEAR(spline.velocity(spline.duration()).x, 0.0, .01);
  }
}

namespace {

bool hasStrictPlanarSelfIntersection(const std::vector<Eigen::Vector3d> &points) {
  const auto cross = [](const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                        const Eigen::Vector3d &c) {
    return (b.x() - a.x()) * (c.y() - a.y()) -
           (b.y() - a.y()) * (c.x() - a.x());
  };
  for (std::size_t i = 1; i < points.size(); ++i) {
    for (std::size_t j = i + 2; j < points.size(); ++j) {
      const auto &a = points[i - 1], &b = points[i];
      const auto &c = points[j - 1], &d = points[j];
      if (cross(a, b, c) * cross(a, b, d) < 0 &&
          cross(c, d, a) * cross(c, d, b) < 0) return true;
    }
  }
  return false;
}

}  // namespace

TEST(ScanRouteContinuity, RecordedMovingRetimeKeepsFlatCurveSimple) {
  using namespace nav_kernel::local::scan::upstream;
  ASSERT_TRUE(hasStrictPlanarSelfIntersection({{0,0,0},{1,1,0},{0,1,0},{1,0,0}}));
  ASSERT_FALSE(hasStrictPlanarSelfIntersection({{0,0,0},{1,0,0},{1,1,0},{2,1,0}}));
  const auto directory = std::filesystem::path(__FILE__).parent_path() / "fixtures";
  // Original begin inputs from tora-curve-upstream-20261106/stages/stages.jsonl.
  std::ifstream routeFile(directory / "tora_retime.route");
  ASSERT_TRUE(routeFile.is_open());
  std::vector<Eigen::Vector3d> route;
  Eigen::Vector3d point;
  while (routeFile >> point.x() >> point.y() >> point.z()) route.push_back(point);
  ASSERT_GE(route.size(), 2U);
  std::ifstream attempts(directory / "tora_retime.attempts");
  ASSERT_TRUE(attempts.is_open());
  for (const int expectedAttempt : {22, 23, 50}) {
    int attempt;
    double now;
    ASSERT_TRUE(attempts >> attempt >> now);
    SCOPED_TRACE(attempt);
    ASSERT_EQ(attempt, expectedAttempt);
    Eigen::Vector3d start, initialVelocity, initialAcceleration, target, targetVelocity;
    for (auto *value : {&start, &initialVelocity, &initialAcceleration, &target,
                        &targetVelocity}) {
      ASSERT_TRUE(attempts >> value->x() >> value->y() >> value->z());
    }
    std::pair<double, double> referenceWindow;
    ASSERT_TRUE(attempts >> referenceWindow.first >> referenceWindow.second);

    auto params = scanParams();
    params.checkObstacle = false;
    const nav_kernel::local::scan::Grid grid(params, LocalPlanRequest{});
    ASSERT_TRUE(grid.valid()) << grid.reason();
    PlanParameters plan;
    plan.max_vel_ = std::sqrt(2.0);
    plan.max_acc_ = .5;
    plan.max_jerk_ = 4;
    plan.ctrl_pt_dist = .2;
    plan.preserve_boundary_ = true;
    // The capture omitted body yaw, so use explicit fixed body-axis bounds.
    plan.max_body_vx_ = 1;
    plan.max_body_vy_ = 1;
    plan.body_yaw_ = 0;
    BsplineOptimizerParams optimizer;
    optimizer.lambda_smooth = 1;
    optimizer.lambda_collision = 1;
    optimizer.lambda_feasibility = .1;
    optimizer.lambda_fitness = 1;
    optimizer.dist0 = .2;
    optimizer.max_vel = plan.max_vel_;
    optimizer.max_acc = plan.max_acc_;
    SCANPlannerManager manager;
    manager.initPlanModules(plan, optimizer, std::make_shared<GridMap>(grid));
    const auto zero = Eigen::Vector3d::Zero().eval();
    const std::vector<Eigen::Vector3d> waypoints(route.begin() + 1, route.end());
    ASSERT_TRUE(manager.planGlobalTrajWaypoints(route.front(), zero, zero,
        waypoints, zero, zero, now));
    ASSERT_TRUE(manager.reboundReplan(start, initialVelocity, initialAcceleration,
        target, targetVelocity, true, false, now, referenceWindow))
        << manager.reboundDebug().stage << ':' << manager.reboundDebug().reason;
    const auto &position = manager.local_data_.position_traj_;
    const auto velocity = position.getDerivative();
    const auto acceleration = velocity.getDerivative();
    const double duration = position.getTimeSum();
    EXPECT_LT((position.evaluateDeBoorT(0) - start).norm(), 1e-9);
    EXPECT_LT((velocity.evaluateDeBoorT(0) - initialVelocity).norm(), 1e-9);
    EXPECT_LT((acceleration.evaluateDeBoorT(0) - initialAcceleration).norm(), 1e-9);
    EXPECT_LT((position.evaluateDeBoorT(duration) - target).norm(), 1e-9);
    EXPECT_LT((velocity.evaluateDeBoorT(duration) - targetVelocity).norm(), 1e-9);
    EXPECT_LT(acceleration.evaluateDeBoorT(duration).norm(), 1e-9);
    const Eigen::Vector3d chord = (target - start).normalized();
    std::vector<Eigen::Vector3d> samples;
    const int intervals = static_cast<int>(std::ceil(duration / .01));
    double maximumProgress = 0, maximumBacktrack = 0;
    for (int i = 0; i <= intervals; ++i) {
      const double t = duration * i / intervals;
      samples.push_back(position.evaluateDeBoorT(t));
      EXPECT_LE(plan.speedRatio(velocity.evaluateDeBoorT(t)), 1 + 1e-8) << t;
      EXPECT_LE(acceleration.evaluateDeBoorT(t).norm(), plan.max_acc_ + 1e-8) << t;
      const double progress = (samples.back() - start).dot(chord);
      maximumProgress = std::max(maximumProgress, progress);
      maximumBacktrack = std::max(maximumBacktrack, maximumProgress - progress);
    }
    EXPECT_FALSE(hasStrictPlanarSelfIntersection(samples));
    // These forward flat bends have no reversing objective.
    EXPECT_LE(maximumBacktrack, .02);
  }
}
