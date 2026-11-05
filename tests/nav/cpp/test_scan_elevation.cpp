#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "collision_bitmap.hpp"
#include "collision/body.hpp"
#include "collision/terrain.hpp"
#include "planning/local/scan/grid.hpp"
#include "planning/local/scan/upstream/plan_manage/planner_fsm.h"

namespace {
using namespace nav_kernel::local::scan::upstream;

struct ElevationFixture {
  nav_kernel::LocalPlannerParams params;
  nav_kernel::LocalPlanRequest request;
  std::unique_ptr<nav_kernel::local::scan::Grid> grid;
  SCANPlannerManager manager;

  ElevationFixture() {
    params.checkObstacle=false;
    params.scan.voxelResolution=.1;
    grid=std::make_unique<nav_kernel::local::scan::Grid>(params,request);
    PlanParameters plan;
    plan.max_vel_=.6;
    plan.max_acc_=2;
    plan.max_jerk_=4;
    plan.ctrl_pt_dist=.2;
    plan.planning_horizon_=8;
    BsplineOptimizerParams optimizer;
    optimizer.lambda_smooth=1;
    optimizer.lambda_collision=.5;
    optimizer.lambda_feasibility=.1;
    optimizer.lambda_fitness=1;
    optimizer.dist0=.2;
    optimizer.max_vel=plan.max_vel_;
    optimizer.max_acc=plan.max_acc_;
    manager.initPlanModules(plan,optimizer,std::make_shared<GridMap>(*grid));
  }

  bool plan(const std::vector<Eigen::Vector3d> &route,
            Eigen::Vector3d initialVelocity=Eigen::Vector3d::Zero(),
            std::optional<Eigen::Vector3d> observedPosition=std::nullopt) {
    ScanReplanParams settings;
    settings.navigationMode=ScanNavigationMode::REFERENCE_PATH;
    settings.planningHorizon=8;
    settings.noReplanThreshold=.1;
    settings.replanThreshold=.25;
    PlannerFsm fsm(manager,settings);
    FsmInput input;
    input.nowS=1;
    FsmOdometry odometry;
    odometry.position=observedPosition.value_or(route.front());
    odometry.velocity=initialVelocity;
    input.odometry=odometry;
    input.referencePath=route;
    if(!fsm.tick(input).targetAccepted) return false;
    input.referencePath.reset();
    for(int tick=0;tick<8;++tick) {
      input.nowS+=.01;
      const auto output=fsm.tick(input);
      if(output.trajectory && output.state==ScanReplanState::EXEC_TRAJ) return true;
    }
    return false;
  }
};

TEST(ScanElevation, HeightOffsetOnlyChangesRouteEntry) {
  ElevationFixture fixture;
  fixture.manager.pp_.preserve_boundary_=true;
  ASSERT_TRUE(fixture.plan({{0,0,.5},{1,0,.5},{2,0,1},{2.5,0,1},{4,0,1}},
      Eigen::Vector3d::Zero(),Eigen::Vector3d{1,0,.4})) << fixture.manager.reboundDebug().reason;
  const auto &curve=fixture.manager.local_data_.position_traj_;
  double nearest=1e9, height=0;
  for(int i=0;i<=400;++i) {
    const auto point=curve.evaluateDeBoorT(curve.getTimeSum()*i/400);
    if(std::abs(point.x()-2.5)<nearest) {
      nearest=std::abs(point.x()-2.5);
      height=point.z();
    }
  }
  EXPECT_NEAR(curve.evaluateDeBoorT(0).z(),.4,1e-9);
  EXPECT_GT(height,.97);
}

TEST(ScanElevation, ReturnedLocalSplineRetainsCrestBetweenEqualHeightEndpoints) {
  ElevationFixture fixture;
  ASSERT_TRUE(fixture.plan({{0,0,.5},{1,0,.5},{1.8,0,1.1},
                            {2.2,0,1.1},{3,0,.5},{4,0,.5}}))
      << fixture.manager.reboundDebug().reason;
  const auto &spline=fixture.manager.local_data_.position_traj_;
  double peak=.5;
  for(int sample=0;sample<=200;++sample)
    peak=std::max(peak,spline.evaluateDeBoorT(spline.getTimeSum()*sample/200)(2));
  EXPECT_GT(peak,1.0);
  RecordProperty("peak_height_m",peak);
  EXPECT_NEAR(spline.evaluateDeBoorT(0)(2),.5,.02);
  EXPECT_NEAR(spline.evaluateDeBoorT(spline.getTimeSum())(2),.5,.02);
}

TEST(ScanElevation, CurvedHeightKeepsTheMovingPhysicalBoundary) {
  ElevationFixture fixture;
  fixture.manager.pp_.preserve_boundary_=true;
  const Eigen::Vector3d start{0,0,.5},goal{4,0,.5},velocity{.15,0,0};
  ASSERT_TRUE(fixture.plan({start,{1,0,.5},{1.8,0,1.1},
                            {2.2,0,1.1},{3,0,.5},goal},velocity))
      << fixture.manager.reboundDebug().reason;
  const auto &spline=fixture.manager.local_data_.position_traj_;
  const auto first=spline.getDerivative();
  const auto second=first.getDerivative();
  EXPECT_LT((spline.evaluateDeBoorT(0)-start).norm(),1e-9);
  EXPECT_LT((first.evaluateDeBoorT(0)-velocity).norm(),1e-9);
  EXPECT_LT(second.evaluateDeBoorT(0).norm(),1e-9);
  EXPECT_LT((spline.evaluateDeBoorT(spline.getTimeSum())-goal).norm(),1e-9);
  EXPECT_LT(first.evaluateDeBoorT(spline.getTimeSum()).norm(),1e-9);
}

TEST(ScanElevation, OrdinaryRouteRetainsSShapedBendsWithoutBoundaryMode) {
  for (const bool fixedHeading : {false, true}) {
    SCOPED_TRACE(fixedHeading ? "fixed heading" : "tangent heading");
    ElevationFixture fixture;
    if (fixedHeading) {
      fixture.request.bodyHeading=0;
      fixture.manager.pp_.body_yaw_=0;
    }
    std::vector<Eigen::Vector3d> reference;
    for (int sample=0;sample<=32;++sample) {
      const double x=sample/8.0;
      reference.push_back({x,.6*std::sin(M_PI*x/2),.5});
    }
    ASSERT_FALSE(fixture.manager.pp_.preserve_boundary_);
    ASSERT_FALSE(fixture.manager.pp_.motion_intent_);
    ASSERT_TRUE(fixture.plan(reference))<<fixture.manager.reboundDebug().reason;
    const auto& spline=fixture.manager.local_data_.position_traj_;
    for (const int bend : {8,16,24}) {
      double distance=10;
      for (int sample=0;sample<=400;++sample)
        distance=std::min(distance,(spline.evaluateDeBoorT(
            spline.getTimeSum()*sample/400)-reference[bend]).norm());
      EXPECT_LT(distance,.15)<<"missing bend "<<reference[bend].transpose();
    }
    EXPECT_LT((spline.evaluateDeBoorT(0)-reference.front()).norm(),.02);
    EXPECT_LT((spline.evaluateDeBoorT(spline.getTimeSum())-reference.back()).norm(),.02);
  }
}

TEST(ScanElevation, ReferenceHeightDoesNotBypassAnObservedObstacle) {
  ElevationFixture fixture;
  lingtu::nav::tests::CollisionBitmap bitmap({-.5,-.5,-1},{4.5,.5,2},.1);
  // Every XY column contains an occupied layer that the crest must cross.
  for(double x=-.45;x<4.5;x+=.1)
    for(double y=-.45;y<.5;y+=.1) bitmap.occupy({x,y,.75});
  fixture.params.checkObstacle=true;
  fixture.request.environment.collision=bitmap.view();
  fixture.grid=std::make_unique<nav_kernel::local::scan::Grid>(fixture.params,fixture.request);
  fixture.manager.grid_map_->setGrid(fixture.grid.get());
  EXPECT_FALSE(fixture.plan({{0,0,.5},{1,0,.5},{1.8,0,1.1},
                             {2.2,0,1.1},{3,0,.5},{4,0,.5}}));
  EXPECT_FALSE(fixture.manager.reboundDebug().success);
}

TEST(ScanElevation, CylinderReferenceCanInitializeInsideAnSShapedChannel) {
  ElevationFixture fixture;
  lingtu::nav::tests::CollisionBitmap bitmap({-2,-1,-1},{2,2,2},.1);
  std::vector<float> hits;
  for(double y=-.95;y<2;y+=.1) for(double x=-1.95;x<2;x+=.1)
    if(y<-.4 || y>=1.6 || (x<-.4 && y>=.4) || (x>=.4 && y<.8))
      for(double z=.05;z<1;z+=.1)
        hits.insert(hits.end(), {static_cast<float>(x),static_cast<float>(y),static_cast<float>(z)});
  bitmap.occupyInflated(hits,.15,.1,.1);
  fixture.params.checkObstacle=true;
  fixture.request.environment.collision=bitmap.view();
  fixture.request.robot.pose.position={-1.1,0,.5};
  fixture.grid=std::make_unique<nav_kernel::local::scan::Grid>(fixture.params,fixture.request);
  ASSERT_TRUE(fixture.grid->valid())<<fixture.grid->reason();
  fixture.manager.grid_map_->setGrid(fixture.grid.get());
  fixture.manager.pp_.preserve_boundary_=true;
  ASSERT_TRUE(fixture.plan({{-1.1,0,.5},{-.4,0,.5},{-.2,.1,.5},{0,.35,.5},
      {0,.8,.5},{.1,1.05,.5},{.4,1.2,.5},{1.1,1.2,.5}}))
      <<fixture.manager.reboundDebug().stage<<':'<<fixture.manager.reboundDebug().reason;
  const auto& spline=fixture.manager.local_data_.position_traj_;
  EXPECT_LT((spline.evaluateDeBoorT(0)-Eigen::Vector3d{-1.1,0,.5}).norm(),1e-9);
  EXPECT_LT((spline.evaluateDeBoorT(spline.getTimeSum())-Eigen::Vector3d{1.1,1.2,.5}).norm(),1e-9);
  for(double t=0;t<spline.getTimeSum();t+=.02) {
    const double next=std::min(t+.02,spline.getTimeSum());
    const auto a=spline.evaluateDeBoorT(t),b=spline.evaluateDeBoorT(next);
    EXPECT_EQ(fixture.manager.grid_map_->getInflateOccupancy(
        a, std::atan2(b.y()-a.y(),b.x()-a.x())),0);
  }
}

TEST(ScanElevation, CheckedCurveSeedDoesNotFollowThePolynomialReversePrefix) {
  ElevationFixture fixture;
  fixture.manager.pp_.preserve_boundary_=true;
  const Eigen::Vector3d start{.791336,-.002926,.5},goal{1.6,.8,.5};
  std::vector<Eigen::Vector3d> reference{start,{.8,.4,.5}};
  for(int sample=1;sample<=16;++sample) {
    const double angle=M_PI-M_PI/2*sample/16;
    reference.push_back({1.2+.4*std::cos(angle),.4+.4*std::sin(angle),.5});
  }
  reference.push_back(goal);
  ASSERT_TRUE(fixture.plan(reference))<<fixture.manager.reboundDebug().reason;
  const auto& spline=fixture.manager.local_data_.position_traj_;
  const auto velocity=spline.getDerivative();
  const auto forward=(reference[1]-start).normalized();
  for(int sample=0;sample<=40;++sample) {
    const double time=spline.getTimeSum()*.1*sample/40;
    EXPECT_GE((spline.evaluateDeBoorT(time)-start).dot(forward),-1e-5);
    EXPECT_GE(velocity.evaluateDeBoorT(time).dot(forward),-1e-5);
  }
  EXPECT_LT((spline.evaluateDeBoorT(0)-start).norm(),1e-9);
  EXPECT_LT((spline.evaluateDeBoorT(spline.getTimeSum())-goal).norm(),1e-9);
}

TEST(ScanElevation, OptimizedRouteRetainsHeightWithoutLocalFloorHits) {
  ElevationFixture fixture;
  fixture.request.supportHeightM = .5;
  fixture.request.supportHeightToleranceM = .05;
  *fixture.grid = nav_kernel::local::scan::Grid(fixture.params, fixture.request);
  ASSERT_TRUE(fixture.plan({{0,0,.5},{2,0,.5}})) << fixture.manager.reboundDebug().reason;
  const auto &curve = fixture.manager.local_data_.position_traj_;
  for (int i=0; i<=40; ++i)
    EXPECT_NEAR(curve.evaluateDeBoorT(curve.getTimeSum()*i/40).z(), .5, .02);
}

struct FloorGapFixture : ElevationFixture {
  lingtu::nav::tests::CollisionBitmap floor{{-1,-1,-1},{2,1,2},.1};

  FloorGapFixture() {
    params.checkObstacle = true;
    auto body = std::make_shared<lingtu::nav::collision::Body>();
    body->margin_m = 0;
    lingtu::nav::collision::Part part;
    part.half_size = {.1,.1,.1};
    body->parts.push_back(part);
    request.body = body;
    request.robot.pose = {{0,0,.35},0};
    request.supportHeightM = .35;
    request.supportHeightToleranceM = .05;
    for (int y=-10; y<10; ++y) for (int x=-10; x<20; ++x) {
      const double px = (x+.5)*.1;
      if (px < .3 || px > .7) floor.occupy({px,(y+.5)*.1,-.05});
    }
    snapshot();
  }

  void snapshot() {
    request.environment.collision = floor.view();
    const auto view = floor.view();
    request.environment.collision.measuredOccupiedStorage =
        std::make_shared<const std::vector<std::uint8_t>>(
            view.inflatedBits, view.inflatedBits + view.inflatedBytes);
    *grid = nav_kernel::local::scan::Grid(params,request);
  }
};

TEST(ScanElevation, OptimizedRouteCanCrossMissingFloorEvidence) {
  FloorGapFixture fixture;
  ASSERT_TRUE(fixture.plan({{0,0,.35},{1,0,.35}}))
      << fixture.manager.reboundDebug().reason;
  const auto &curve = fixture.manager.local_data_.position_traj_;
  const auto derivative = curve.getDerivative();
  bool crossed_gap = false;
  for (int i=0; i<80; ++i) {
    const double a=curve.getTimeSum()*i/80, b=curve.getTimeSum()*(i+1)/80;
    const auto start=curve.evaluateDeBoorT(a), end=curve.evaluateDeBoorT(b);
    const auto velocity=derivative.evaluateDeBoorT(a), endVelocity=derivative.evaluateDeBoorT(b);
    crossed_gap |= start.x() > .3 && start.x() < .7;
    EXPECT_NEAR(start.z(),.35,.02);
    EXPECT_EQ(fixture.grid->trajectoryOccupancy({start.x(),start.y(),start.z()},
        {velocity.x(),velocity.y(),velocity.z()}, {end.x(),end.y(),end.z()},
        {endVelocity.x(),endVelocity.y(),endVelocity.z()},a,b),0);
  }
  EXPECT_TRUE(crossed_gap);
}

TEST(ScanElevation, MeasuredObstacleStillBlocksTheSegmentAcrossMissingFloorEvidence) {
  FloorGapFixture fixture;
  ASSERT_EQ(fixture.grid->trajectoryOccupancy({.1,0,.35},{1,0,0},{.9,0,.35},{1,0,0},0,1),0);
  fixture.floor.occupy({.55,.05,.35});
  fixture.snapshot();
  EXPECT_EQ(fixture.grid->trajectoryOccupancy({.1,0,.35},{1,0,0},{.9,0,.35},{1,0,0},0,1),1);
}
}  // namespace
