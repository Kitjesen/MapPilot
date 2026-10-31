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
            Eigen::Vector3d initialVelocity=Eigen::Vector3d::Zero()) {
    ScanReplanParams settings;
    settings.navigationMode=ScanNavigationMode::REFERENCE_PATH;
    settings.planningHorizon=8;
    settings.noReplanThreshold=.1;
    settings.replanThreshold=.25;
    PlannerFsm fsm(manager,settings);
    FsmInput input;
    input.nowS=1;
    FsmOdometry odometry;
    odometry.position=route.front();
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

TEST(ScanElevation, CheckedTangentReferenceCanInitializeInsideAnSShapedChannel) {
  ElevationFixture fixture;
  lingtu::nav::tests::CollisionBitmap bitmap({-2,-1,-1},{2,2,2},.1);
  for(double y=-.95;y<2;y+=.1) for(double x=-1.95;x<2;x+=.1)
    if(y<-.4 || y>=1.6 || (x<-.4 && y>=.4) || (x>=.4 && y<.8))
      for(double z=.05;z<1;z+=.1) bitmap.occupy({x,y,z});
  fixture.params.checkObstacle=true;
  fixture.request.environment.collision=bitmap.view();
  const auto& view=fixture.request.environment.collision;
  fixture.request.environment.collision.measuredOccupiedStorage=
      std::make_shared<const std::vector<std::uint8_t>>(view.inflatedBits,view.inflatedBits+view.inflatedBytes);
  fixture.request.environment.collision.knownFreeStorage=
      std::make_shared<const std::vector<std::uint8_t>>(view.inflatedBytes,255);
  auto body=std::make_shared<lingtu::nav::collision::Body>();body->margin_m=0;
  lingtu::nav::collision::Part box;box.half_size={.3,.15,.1};body->parts.push_back(box);
  fixture.request.body=body;fixture.request.robot.pose.position={-1.1,0,.5};
  fixture.grid=std::make_unique<nav_kernel::local::scan::Grid>(fixture.params,fixture.request);
  ASSERT_TRUE(fixture.grid->valid())<<fixture.grid->reason();
  fixture.manager.grid_map_->setGrid(fixture.grid.get());
  fixture.manager.pp_.preserve_boundary_=true;
  ASSERT_TRUE(fixture.plan({{-1.1,0,.5},{-.4,0,.5},{-.2,.1,.5},{0,.35,.5},
      {0,.8,.5},{.1,1.05,.5},{.4,1.2,.5},{1.1,1.2,.5}}))
      <<fixture.manager.reboundDebug().stage<<':'<<fixture.manager.reboundDebug().reason;
  const auto& spline=fixture.manager.local_data_.position_traj_;
  const auto velocity=spline.getDerivative();
  EXPECT_LT((spline.evaluateDeBoorT(0)-Eigen::Vector3d{-1.1,0,.5}).norm(),1e-9);
  EXPECT_LT((spline.evaluateDeBoorT(spline.getTimeSum())-Eigen::Vector3d{1.1,1.2,.5}).norm(),1e-9);
  for(double t=0;t<spline.getTimeSum();t+=.02) {
    const double next=std::min(t+.02,spline.getTimeSum());
    const auto a=spline.evaluateDeBoorT(t),b=spline.evaluateDeBoorT(next);
    const auto va=velocity.evaluateDeBoorT(t),vb=velocity.evaluateDeBoorT(next);
    EXPECT_EQ(fixture.grid->trajectoryOccupancy({a.x(),a.y(),a.z()},{va.x(),va.y(),va.z()},
        {b.x(),b.y(),b.z()},{vb.x(),vb.y(),vb.z()},t,next),0);
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

TEST(ScanElevation, SharedReconstructedSupportChecksTheOptimizedRouteWithoutLocalFloorHits) {
  ElevationFixture fixture;
  const std::vector<nav_kernel::Vec3> reference{{0,0,.5},{2,0,.5}};
  fixture.request.reference = {reference.data(), static_cast<int>(reference.size()), 1};
  fixture.request.supportHeightM = .5;
  fixture.request.supportHeightToleranceM = .05;
  fixture.request.supportSurface = [](const lingtu::nav::collision::Vec3 &ground,
                                      double tolerance) -> std::optional<double> {
    return std::abs(ground.z) <= tolerance ? std::optional<double>{0.0} : std::nullopt;
  };
  *fixture.grid = nav_kernel::local::scan::Grid(fixture.params, fixture.request);
  ASSERT_TRUE(fixture.plan({{0,0,.5},{2,0,.5}})) << fixture.manager.reboundDebug().reason;
  const auto &curve = fixture.manager.local_data_.position_traj_;
  const auto derivative = curve.getDerivative();
  for (int i=0; i<40; ++i) {
    const double a = curve.getTimeSum()*i/40, b = curve.getTimeSum()*(i+1)/40;
    const auto start=curve.evaluateDeBoorT(a), end=curve.evaluateDeBoorT(b);
    const auto velocity=derivative.evaluateDeBoorT(a), endVelocity=derivative.evaluateDeBoorT(b);
    EXPECT_EQ(fixture.grid->trajectoryOccupancy({start.x(),start.y(),start.z()},
        {velocity.x(),velocity.y(),velocity.z()}, {end.x(),end.y(),end.z()},
        {endVelocity.x(),endVelocity.y(),endVelocity.z()},a,b),0);
  }
}

TEST(ScanElevation, SharedSupportRejectsARealGapBetweenSupportedSegmentEndpoints) {
  nav_kernel::LocalPlannerParams params;
  params.checkObstacle=false;
  params.scan.voxelResolution=.1;
  const std::vector<nav_kernel::Vec3> reference{{0,0,.35},{1,0,.35}};
  nav_kernel::LocalPlanRequest request;
  request.reference={reference.data(),static_cast<int>(reference.size()),1};
  request.supportHeightM=.35;
  request.supportHeightToleranceM=.05;
  request.supportSurface=[](const lingtu::nav::collision::Vec3 &ground,
                            double tolerance) -> std::optional<double> {
    if (ground.x > .3 && ground.x < .7) return std::nullopt;
    return std::abs(ground.z)<=tolerance ? std::optional<double>{0.0} : std::nullopt;
  };
  nav_kernel::local::scan::Grid grid(params,request);
  ASSERT_TRUE(grid.valid());
  EXPECT_EQ(grid.inflatedOccupancy({.5,0,.35},0),0) << "support must not alter obstacle evidence";
  EXPECT_EQ(grid.trajectoryOccupancy({.1,0,.35},{1,0,0},{.9,0,.35},{1,0,0},0,1),1);

  ElevationFixture fixture;
  fixture.request=request;
  *fixture.grid=nav_kernel::local::scan::Grid(fixture.params,fixture.request);
  EXPECT_FALSE(fixture.plan({{0,0,.35},{1,0,.35}}))
      << "SCAN returned an optimized trajectory across the missing support";
}

TEST(ScanElevation, SharedSupportUsesReferenceLayerAndAllowsBaseHeave) {
  nav_kernel::LocalPlannerParams params;
  params.checkObstacle=false;
  params.scan.voxelResolution=.1;
  const std::vector<nav_kernel::Vec3> reference{{0,0,2.35},{1,0,2.35}};
  nav_kernel::LocalPlanRequest request;
  request.reference={reference.data(),static_cast<int>(reference.size()),1};
  request.supportHeightM=.35;
  request.supportHeightToleranceM=.05;
  double selected=-1;
  request.supportSurface=[&](const lingtu::nav::collision::Vec3 &ground,
                             double tolerance) -> std::optional<double> {
    for (double height : {0.0,2.0}) if (std::abs(ground.z-height)<=tolerance) {
      selected=height;
      return height;
    }
    return std::nullopt;
  };
  request.robot.pose.position={0,0,2.45};
  nav_kernel::local::scan::Grid grid(params,request);
  EXPECT_EQ(grid.trajectoryOccupancy({0,0,2.45},{1,0,0},{1,0,2.35},{1,0,0},0,1),0);
  EXPECT_DOUBLE_EQ(selected,2.0);
  request.supportSurface=[](const lingtu::nav::collision::Vec3 &ground,
                            double tolerance) -> std::optional<double> {
    return std::abs(ground.z)<=tolerance ? std::optional<double>{0.0} : std::nullopt;
  };
  EXPECT_EQ(nav_kernel::local::scan::Grid(params,request).trajectoryOccupancy(
      {0,0,2.45},{1,0,0},{1,0,2.35},{1,0,0},0,1),1) << "the lower floor cannot replace the missing upper floor";
}

TEST(ScanElevation, SharedSupportTransformsOdomReferenceIntoMapCoordinates) {
  nav_kernel::LocalPlannerParams params;
  params.checkObstacle=false;
  params.scan.voxelResolution=.1;
  const std::vector<nav_kernel::Vec3> reference{{0,0,.35},{1,0,.35}};
  nav_kernel::LocalPlanRequest request;
  request.reference={reference.data(),static_cast<int>(reference.size()),1};
  request.supportHeightM=.35;
  request.supportHeightToleranceM=.05;
  request.environment.collision.gridFromPlanningTranslation={10,20,2};
  request.environment.collision.gridFromPlanningYaw=M_PI/2;
  request.supportSurface=[](const lingtu::nav::collision::Vec3 &ground,
                            double tolerance) -> std::optional<double> {
    if (std::abs(ground.x-10)>1e-6 || ground.y<20 || ground.y>21) return std::nullopt;
    return std::abs(ground.z-2)<=tolerance ? std::optional<double>{2.0} : std::nullopt;
  };
  nav_kernel::local::scan::Grid grid(params,request);
  EXPECT_EQ(grid.trajectoryOccupancy({0,0,.35},{1,0,0},{1,0,.35},{1,0,0},0,1),0);
}
}  // namespace
