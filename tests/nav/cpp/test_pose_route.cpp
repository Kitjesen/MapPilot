#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <thread>

#include "collision_bitmap.hpp"
#include "navigation/executor.hpp"
#include "navigation/pose_route.hpp"

namespace {
using namespace lingtu::nav::navigation;

Route poseRoute() {
  Route route;
  auto body = std::make_shared<lingtu::nav::collision::Body>();
  lingtu::nav::collision::Part part;
  part.rotation = {1,0,0,0,1,0,0,0,1};
  part.half_size = {0.2,0.1,0.1};
  body->parts.push_back(part);
  route.body = body;
  route.posePoints = {{{0,0,0},0}, {{1,0,0},0}, {{1,0,0},M_PI/2}, {{1,1,0},M_PI/2}};
  return route;
}

// Sideways travel after this requested turn cannot use forward tangent following.
Route stoppedPoseRoute() {
  auto route=poseRoute();
  route.posePoints.back().position={2,0,0};
  return route;
}

Executor makeExecutor(ExecutorConfig config = {}) {
  config.planning_frame = PlanningFrame::Map;
  nav_kernel::LocalPlannerParams params;
  params.backend = nav_kernel::LocalPlannerBackend::Scan;
  params.checkObstacle = true;
  params.scan.voxelResolution = 0.1;
  nav_kernel::LocalPlanner planner(params);
  EXPECT_TRUE(planner.configure(""));
  return Executor(config, std::move(planner));
}

ExecutionInput observation(nav_kernel::Pose pose, double time = 1.0) {
  static lingtu::nav::tests::CollisionBitmap bitmap;
  ExecutionInput input;
  input.mapBody = input.odomBody = pose;
  input.timestampS = time;
  input.observation.frame_epoch = 1;
  input.observation.body_velocity_valid = true;
  input.observation.odom_stamp_s = time;
  input.observation.clock_mode = nav_kernel::PlanClockMode::External;
  input.observation.collision = bitmap.view(time);
  const auto bytes = input.observation.collision.inflatedBytes;
  input.observation.collision.measuredOccupiedStorage =
      std::make_shared<std::vector<std::uint8_t>>(bytes, 0);
  input.observation.collision.knownFreeStorage =
      std::make_shared<std::vector<std::uint8_t>>(bytes, 255);
  return input;
}

TEST(PoseRoute, RetainsTurnAndMergesOnlySameYawMoves) {
  const auto stages = compilePoseRoute(poseRoute().posePoints);
  ASSERT_EQ(stages.size(), 4U);
  EXPECT_TRUE(stages[0].rotation);
  EXPECT_FALSE(stages[1].rotation);
  EXPECT_TRUE(stages[2].rotation);
  EXPECT_FALSE(stages[3].rotation);
  EXPECT_EQ(stages[1].points.size(), 2U);
  EXPECT_DOUBLE_EQ(stages[2].points.front().x, 1.0);
}

TEST(PoseRoute, FinalTurnUsesRequestedArrivalRadius) {
  auto route = poseRoute();
  route.posePoints = {{{0, 0, 0}, 0}, {{1, 0, 0}, 0}, {{1, 0, 0}, .2}};
  route.finalYaw = .2;
  route.goalToleranceM = .15;
  auto executor = makeExecutor();
  executor.setRoute(route);
  const auto initial=executor.tick(observation({{0, 0, 0}, 0}, 1.0));
  ASSERT_EQ(initial.pose_stage_index,1U)<<initial.reason;
  // Exercise arrival radius, independently of the execution-gap hold policy.
  auto input = observation({{.98, 0, 0}, .2}, 1.01);
  auto output = executor.tick(input);
  EXPECT_EQ(output.reason, "settling_at_goal");
  EXPECT_DOUBLE_EQ(output.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.cmd_vel.vy, 0.0);
  for (int i = 1; i <= 6; ++i) {
    input.timestampS = input.observation.odom_stamp_s = 1.01 + i * .1;
    output = executor.tick(input);
  }
  EXPECT_TRUE(output.goal_reached) << output.reason;
}

TEST(PoseRoute, FinalTurnStillRequiresHeightAndYaw) {
  auto route = poseRoute();
  route.posePoints = {{{0, 0, 0}, 0}, {{1, 0, 0}, 0}, {{1, 0, 0}, .2}};
  route.finalYaw = .2;
  route.goalToleranceM = .15;
  auto executor = makeExecutor();
  executor.setRoute(route);
  executor.tick(observation({{0, 0, 0}, 0}, 1.0));
  const auto wrong_floor = executor.tick(observation({{.98, 0, .5}, .2}, 2.0));
  EXPECT_FALSE(wrong_floor.goal_reached);
  EXPECT_EQ(wrong_floor.pose_stage_index, 1U);
  auto moving = observation({{.98, 0, 0}, 0}, 2.05);
  moving.observation.body_linear_velocity.x = .2;
  const auto stopping = executor.tick(moving);
  EXPECT_EQ(stopping.reason, "settling_before_goal_yaw");
  EXPECT_DOUBLE_EQ(stopping.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(stopping.cmd_vel.wz, 0.0);
  const auto turning = executor.tick(observation({{.98, 0, 0}, 0}, 2.1));
  EXPECT_FALSE(turning.goal_reached);
  EXPECT_EQ(turning.reason, "aligning_goal_yaw");
  EXPECT_GT(turning.cmd_vel.wz, 0.0);
}

TEST(PoseRoute, PositionOnlyGoalDoesNotInheritThePathsTerminalHeading) {
  for (bool requestedYaw : {false, true}) {
    SCOPED_TRACE(requestedYaw);
    auto route = poseRoute();
    route.posePoints = {{{0, 0, 0}, 0}, {{1, 0, 0}, 0}};
    if (requestedYaw) route.finalYaw = 0;
    route.goalToleranceM = .15;
    auto executor = makeExecutor();
    executor.setRoute(route);
    executor.tick(observation({{0, 0, 0}, 0}, 1));
    ExecutionOutput output;
    for (int i = 0; i <= 30; ++i) {
      output = executor.tick(observation({{.98, 0, 0}, .5}, 1.02 + i * .02));
      if (!requestedYaw) EXPECT_DOUBLE_EQ(output.cmd_vel.wz, 0) << output.reason;
    }
    if (requestedYaw) {
      EXPECT_FALSE(output.goal_reached);
      EXPECT_EQ(output.reason, "aligning_goal_yaw");
      EXPECT_LT(output.cmd_vel.wz, 0);
    } else {
      EXPECT_TRUE(output.goal_reached) << output.reason;
    }
  }
}

TEST(PoseRoute, RejectsIndependentYawThatDoesNotFollowTheForwardReference) {
  auto route = poseRoute();
  route.posePoints = {{{0,0,0},0}, {{1,0,0},M_PI/2}};
  EXPECT_THROW(compilePoseRoute(route.posePoints), std::invalid_argument);
}

Route sampledTurnRoute() {
  auto route=poseRoute();
  route.posePoints.clear();
  for(int sample=0;sample<=40;++sample) {
    const double yaw=M_PI*.5*sample/40;
    route.posePoints.push_back({{std::sin(yaw),1-std::cos(yaw),.1*yaw},yaw});
  }
  route.finalYaw=M_PI/2;
  route.goalToleranceM=.025;
  route.yawToleranceRad=.05;
  return route;
}

TEST(PoseRoute, CoupledForwardPosesKeepOneThreeDimensionalMovingReference) {
  const auto route=sampledTurnRoute();
  const auto stages=compilePoseRoute(route.posePoints);
  ASSERT_EQ(stages.size(),2U);
  EXPECT_TRUE(stages[0].rotation);
  EXPECT_TRUE(stages[1].tangentHeading);
  EXPECT_FALSE(stages[1].rotation);
  ASSERT_EQ(stages[1].points.size(),route.posePoints.size());
  for(std::size_t i=0;i<route.posePoints.size();++i)
    EXPECT_DOUBLE_EQ(nav_kernel::distance3D(stages[1].points[i],route.posePoints[i].position),0);
  EXPECT_DOUBLE_EQ(stages[1].yaw,*route.finalYaw);
  auto crab=route.posePoints;
  for(auto& point:crab) point.yaw+=.3;
  EXPECT_THROW(compilePoseRoute(crab),std::invalid_argument);
  for(auto& point:crab) point.yaw+=M_PI-.3;
  EXPECT_THROW(compilePoseRoute(crab),std::invalid_argument);
}

TEST(PoseRoute, ContinuousUnroundedCornerRetainsOriginalHeightAndExplicitTerminalTurn) {
  auto route=poseRoute();
  route.posePoints[1].position.z=route.posePoints[2].position.z=.1;
  route.posePoints[3].position.z=.2;
  route.posePoints.push_back({route.posePoints.back().position,M_PI/2+.3});
  const auto stages=compilePoseRoute(route.posePoints);
  ASSERT_FALSE(poseRouteBlend(stages,1,.05));
  const auto continuous=poseRouteContinuous(stages,1,.05);
  ASSERT_TRUE(continuous);
  EXPECT_TRUE(continuous->stage.tangentHeading);
  EXPECT_EQ(continuous->lastStageIndex,3U);
  ASSERT_EQ(continuous->stage.points.size(),3U);
  EXPECT_DOUBLE_EQ(continuous->stage.points[1].z,.1);
  EXPECT_DOUBLE_EQ(continuous->stage.points[2].z,.2);
  EXPECT_TRUE(stages.back().rotation);
  EXPECT_DOUBLE_EQ(stages.back().yaw,M_PI/2+.3);
  EXPECT_FALSE(poseRouteContinuous(compilePoseRoute(stoppedPoseRoute().posePoints),1,.05));
}

Route wideCornerRoute() {
  auto route=poseRoute();
  route.posePoints={{{0,0,0},0},{{3,0,0},0},{{3,0,0},M_PI/2},{{3,3,0},M_PI/2}};
  route.goalToleranceM=.025;
  route.yawToleranceRad=.05;
  route.finalYaw=M_PI/2;
  route.supportHeightM=.5;
  route.supportHeightToleranceM=.01;
  return route;
}

ExecutionInput wideObservation(nav_kernel::Pose pose, double time=1) {
  auto input=observation(pose,time);
  static const auto floor=[] {
    lingtu::nav::tests::CollisionBitmap bitmap;
    for(int y=0;y<100;++y) for(int x=0;x<100;++x)
      bitmap.occupy({-4.95+x*.1,-4.95+y*.1,-.55});
    return bitmap;
  }();
  const auto view=floor.view();
  input.observation.collision.measuredOccupiedStorage=
      std::make_shared<const std::vector<std::uint8_t>>(view.inflatedBits,view.inflatedBits+view.inflatedBytes);
  return input;
}

TEST(PoseRoute, WideTurnCandidateUsesRoundedSeedAndRetainsTerminalPose) {
  const auto stages=compilePoseRoute(wideCornerRoute().posePoints);
  const auto blend=poseRouteBlend(stages,1,.05);
  ASSERT_TRUE(blend);
  ASSERT_GT(blend->stage.points.size(),4U);
  EXPECT_DOUBLE_EQ(blend->stage.points.back().x,3);
  EXPECT_DOUBLE_EQ(blend->stage.points.back().y,3);
  EXPECT_DOUBLE_EQ(blend->stage.yaw,M_PI/2);
  EXPECT_DOUBLE_EQ(blend->radiusM,1);
  EXPECT_DOUBLE_EQ(blend->stage.points[1].x,2);
  EXPECT_DOUBLE_EQ(blend->stage.points[1].y,0);
  EXPECT_TRUE(poseRouteBlend(compilePoseRoute(poseRoute().posePoints),1,.05));
  auto elevated=wideCornerRoute();elevated.posePoints.back().position.z=.1;
  EXPECT_FALSE(poseRouteBlend(compilePoseRoute(elevated.posePoints),1,.05));
  auto bins=wideCornerRoute();
  bins.posePoints={{{0,0,0},0},{{3,0,0},0},{{3,0,0},M_PI/8},
      {{3,0,0},M_PI/4},{{3,0,0},3*M_PI/8},{{3,0,0},M_PI/2},{{3,3,0},M_PI/2}};
  const auto binnedBlend=poseRouteBlend(compilePoseRoute(bins.posePoints),1,.05);
  ASSERT_TRUE(binnedBlend);
  EXPECT_EQ(binnedBlend->lastStageIndex,6U);
  EXPECT_DOUBLE_EQ(binnedBlend->radiusM,1);
  bins.posePoints[3].yaw=-M_PI/4;
  EXPECT_FALSE(poseRouteBlend(compilePoseRoute(bins.posePoints),1,.05));
  auto crab=wideCornerRoute();
  for(auto& p:crab.posePoints) p.yaw+=.3;
  EXPECT_FALSE(poseRouteBlend(compilePoseRoute(crab.posePoints),1,.05));
}

TEST(PoseRoute, MeasuredForwardYawUsesTheTasksExistingTolerance) {
  constexpr double measuredYaw=.001213979959;
  auto route=wideCornerRoute();
  route.yawToleranceRad=3*M_PI/180;
  route.posePoints[0].yaw=route.posePoints[1].yaw=measuredYaw;
  const auto stages=compilePoseRoute(route.posePoints);
  EXPECT_TRUE(poseRouteBlend(stages,1,*route.yawToleranceRad));
  EXPECT_FALSE(poseRouteBlend(stages,1,measuredYaw*.5));
  auto executor=makeExecutor();executor.setRoute(route);
  auto input=wideObservation({{0,0,0},measuredYaw});
  EXPECT_EQ(executor.tick(input).pose_stage_count,2U);
  route.posePoints[0].yaw=route.posePoints[1].yaw=.3;
  EXPECT_FALSE(poseRouteBlend(compilePoseRoute(route.posePoints),1,*route.yawToleranceRad));
}

TEST(PoseRoute, ShortSRetainsBothCurvaturesAndTheRequestedTerminalPose) {
  const std::vector<PoseRoutePoint> points={{{0,0,0},0},{{.8,0,0},0},
      {{.8,0,0},M_PI/2},{{.8,.8,0},M_PI/2},{{.8,.8,0},0},{{1.6,.8,0},0}};
  const auto blend=poseRouteBlend(compilePoseRoute(points),1,.05);
  ASSERT_TRUE(blend);
  EXPECT_EQ(blend->lastStageIndex,5U);
  EXPECT_NEAR(nav_kernel::distance3D(blend->stage.points.front(),points.front().position),0,1e-12);
  EXPECT_NEAR(nav_kernel::distance3D(blend->stage.points.back(),points.back().position),0,1e-12);
  EXPECT_DOUBLE_EQ(blend->stage.yaw,0);
  bool left=false,right=false;
  for(std::size_t i=2;i<blend->stage.points.size();++i) {
    const auto& a=blend->stage.points[i-2];
    const auto& b=blend->stage.points[i-1];
    const auto& c=blend->stage.points[i];
    const double cross=(b.x-a.x)*(c.y-b.y)-(b.y-a.y)*(c.x-b.x);
    left=left || cross>1e-8;right=right || cross< -1e-8;
  }
  EXPECT_TRUE(left);EXPECT_TRUE(right);
}

TEST(PoseRoute, GridStaircaseCanSeedACurveWithoutAdmittingLongCrabbing) {
  const std::vector<PoseRoutePoint> points={{{-1.1,0,0},0},{{-.25,0,0},0},
      {{-.25,0,0},M_PI/4},{{-.2,.05,0},M_PI/4},{{-.15,.05,0},M_PI/4},
      {{-.1,.1,0},M_PI/4},{{-.05,.15,0},M_PI/4},{{0,.2,0},M_PI/4},
      {{0,.2,0},M_PI/2},{{0,.95,0},M_PI/2},{{0,.95,0},M_PI/4},
      {{.05,1,0},M_PI/4},{{.05,1.05,0},M_PI/4},{{.1,1.1,0},M_PI/4},
      {{.15,1.15,0},M_PI/4},{{.2,1.15,0},M_PI/4},{{.25,1.2,0},M_PI/4},
      {{.25,1.2,0},0},{{1.1,1.2,0},0}};
  const auto stages=compilePoseRoute(points);
  EXPECT_FALSE(poseRouteBlend(stages,1,.05));
  const auto blend=poseRouteBlend(stages,1,.05,.05);
  ASSERT_TRUE(blend);
  EXPECT_EQ(blend->lastStageIndex,stages.size()-1);
  EXPECT_GT(blend->stage.points.size(),4U);
  EXPECT_GT(blend->radiusM,0);
  EXPECT_NEAR(nav_kernel::distance3D(blend->stage.points.back(),points.back().position),0,1e-12);
  auto crab=wideCornerRoute();
  for(auto& point:crab.posePoints) point.yaw+=.3;
  EXPECT_FALSE(poseRouteBlend(compilePoseRoute(crab.posePoints),1,.05,.05));
}

TEST(PoseRoute, ShortSMultipartBodyUsesOneContinuousStage) {
  auto route=wideCornerRoute();
  route.posePoints={{{0,0,0},0},{{.8,0,0},0},{{.8,0,0},M_PI/2},
      {{.8,.8,0},M_PI/2},{{.8,.8,0},0},{{1.6,.8,0},0}};
  route.finalYaw=0;
  auto body=std::make_shared<lingtu::nav::collision::Body>(*route.body);
  auto sensor=body->parts.front();sensor.link="front_sensor";
  sensor.center={.4,0,.15};sensor.half_size={.04,.04,.04};
  body->parts.push_back(sensor);route.body=body;
  auto executor=makeExecutor();executor.setRoute(route);
  const auto output=executor.tick(wideObservation({{0,0,0},0}));
  EXPECT_EQ(output.pose_stage_index,1U);
  EXPECT_EQ(output.pose_stage_count,2U);
  EXPECT_FALSE(output.goal_reached);
}

TEST(PoseRoute, ShortSRetainsASeparateRequestedTerminalYaw) {
  auto route=wideCornerRoute();
  route.posePoints={{{0,0,0},0},{{.8,0,0},0},{{.8,0,0},M_PI/2},
      {{.8,.8,0},M_PI/2},{{.8,.8,0},0},{{1.6,.8,0},0},{{1.6,.8,0},.2}};
  route.finalYaw=.2;
  auto executor=makeExecutor();executor.setRoute(route);
  EXPECT_EQ(executor.tick(wideObservation({{0,0,0},0})).pose_stage_count,3U);
  const auto output=executor.tick(wideObservation({{1.6,.8,0},0},2));
  EXPECT_EQ(output.pose_stage_count,3U);
  EXPECT_EQ(output.pose_stage_index,2U);
  EXPECT_EQ(output.reason,"aligning_goal_yaw");
  EXPECT_FALSE(output.goal_reached);
  EXPECT_DOUBLE_EQ(output.cmd_vel.vx,0);
  EXPECT_GT(output.cmd_vel.wz,0);
}

TEST(PoseRoute, FailedBlendedSplineRestoresTheOriginalCheckedStages) {
  auto executor=makeExecutor();executor.setRoute(wideCornerRoute());
  EXPECT_EQ(executor.tick(wideObservation({{0,0,0},0})).pose_stage_count,2U);
  auto input=observation({{0,0,0},0},2);
  input.observation.collision.measuredOccupiedStorage=
      std::make_shared<const std::vector<std::uint8_t>>(
          input.observation.collision.inflatedBytes,255);
  input.observation.collision.generation=2;
  input.observation.cloud_generation=2;
  auto output=executor.tick(input);
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(output.pose_stage_count==2 && std::chrono::steady_clock::now()<deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    output=executor.tick(input);
  }
  EXPECT_EQ(output.pose_stage_count,4U);
  EXPECT_EQ(output.reason,"pose_blend_replanning");
  EXPECT_FALSE(output.goal_reached);
  EXPECT_DOUBLE_EQ(output.cmd_vel.vx,0);
  EXPECT_DOUBLE_EQ(output.cmd_vel.vy,0);
  EXPECT_DOUBLE_EQ(output.cmd_vel.wz,0);
}

TEST(PoseRoute, MidBlendFailureKeepsItsCurrentReferenceInsteadOfAnOldTurnCentre) {
  auto executor=makeExecutor();executor.setRoute(wideCornerRoute());
  EXPECT_EQ(executor.tick(wideObservation({{0,0,0},0})).pose_stage_count,2U);
  auto input=observation({{2.7,.3,0},M_PI/4},2);
  input.observation.collision.measuredOccupiedStorage=
      std::make_shared<const std::vector<std::uint8_t>>(
          input.observation.collision.inflatedBytes,255);
  input.observation.collision.generation=2;input.observation.cloud_generation=2;
  auto output=executor.tick(input);
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(output.reason!="local_blocked" && output.reason!="local_no_path" &&
        std::chrono::steady_clock::now()<deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));output=executor.tick(input);
  }
  EXPECT_EQ(output.pose_stage_count,2U);
  EXPECT_EQ(output.pose_stage_index,1U);
  EXPECT_TRUE(output.reason=="local_blocked" || output.reason=="local_no_path")<<output.reason;
  EXPECT_FALSE(output.goal_reached);
  EXPECT_DOUBLE_EQ(output.cmd_vel.vx,0);EXPECT_DOUBLE_EQ(output.cmd_vel.vy,0);
}

TEST(PoseRoute, ActualTangentBodyDoesNotRequireAllYawClearanceOnStraightApproach) {
  auto executor=makeExecutor();executor.setRoute(wideCornerRoute());
  auto input=wideObservation({{0,0,0},0});
  lingtu::nav::tests::CollisionBitmap obstacle;
  obstacle.occupy({.15,.25,0});
  const auto view=obstacle.view();
  auto occupied=std::make_shared<std::vector<std::uint8_t>>(
      *input.observation.collision.measuredOccupiedStorage);
  for(std::size_t i=0;i<view.inflatedBytes;++i) (*occupied)[i]|=view.inflatedBits[i];
  input.observation.collision.measuredOccupiedStorage=occupied;
  EXPECT_EQ(executor.tick(input).pose_stage_count,2U);
}

TEST(PoseRoute, AttachmentObstacleDoesNotPreemptScanReferencePlanning) {
  auto route=wideCornerRoute();
  route.posePoints=poseRoute().posePoints;
  auto body=std::make_shared<lingtu::nav::collision::Body>(*route.body);
  auto sensor=body->parts.front();sensor.link="front_sensor";
  sensor.center={.6,0,.15};sensor.half_size={.04,.04,.04};
  body->parts.push_back(sensor);route.body=body;
  auto input=wideObservation({{0,0,0},0});
  lingtu::nav::tests::CollisionBitmap obstacle;
  obstacle.occupy({1.275,.575,.15});
  const auto view=obstacle.view();
  auto occupied=std::make_shared<std::vector<std::uint8_t>>(
      *input.observation.collision.measuredOccupiedStorage);
  for(std::size_t i=0;i<view.inflatedBytes;++i) (*occupied)[i]|=view.inflatedBits[i];
  input.observation.collision.measuredOccupiedStorage=occupied;
  auto executor=makeExecutor();executor.setRoute(route);
  const auto output=executor.tick(input);
  EXPECT_EQ(output.pose_stage_count,2U);
  EXPECT_EQ(output.pose_stage_index,1U);
  EXPECT_FALSE(output.goal_reached);
}

TEST(PoseRoute, WideClearCornerActivatesOneContinuousSplineStage) {
  for(auto frame:{PlanningFrame::Map,PlanningFrame::Odom}) {
    ExecutorConfig config;config.planning_frame=frame;
    nav_kernel::LocalPlannerParams params;params.backend=nav_kernel::LocalPlannerBackend::Scan;
    params.scan.voxelResolution=.1;
    nav_kernel::LocalPlanner planner(params);ASSERT_TRUE(planner.configure(""));
    Executor executor(config,std::move(planner));
    const auto route=wideCornerRoute();executor.setRoute(route);
    auto input=wideObservation({{0,0,0},0});
    input.mapFromOdom={{1,-1,0},.7};
    input.odomBody.position=input.mapFromOdom.odomPointFromMap(input.mapBody.position);
    input.odomBody.yaw=-.7;
    const auto start=executor.tick(input);
    EXPECT_EQ(start.pose_stage_index,1U);
    EXPECT_EQ(start.pose_stage_count,2U);
    EXPECT_FALSE(start.goal_reached);
    input.mapBody={{3,3,0},M_PI/2-.06};
    input.odomBody.position=input.mapFromOdom.odomPointFromMap(input.mapBody.position);
    input.odomBody.yaw=input.mapBody.yaw-.7;
    input.timestampS+=.02;input.observation.odom_stamp_s=input.timestampS;
    const auto terminal=executor.tick(input);
    EXPECT_FALSE(terminal.goal_reached);
    EXPECT_EQ(terminal.reason,"aligning_goal_yaw");
    EXPECT_DOUBLE_EQ(terminal.cmd_vel.wz,.2);
  }
}

TEST(PoseRoute, FutureObstacleAndUnknownSpaceDoNotRejectTheReferenceBeforeScan) {
  for(bool unknown:{false,true}) {
    auto executor=makeExecutor();executor.setRoute(wideCornerRoute());
    auto input=wideObservation({{0,0,0},0});
    lingtu::nav::tests::CollisionBitmap bitmap;
    bitmap.occupy({2.7,.3,0});
    const auto view=bitmap.view();
    if (unknown) {
      input.observation.collision.knownFreeStorage=
          std::make_shared<std::vector<std::uint8_t>>(view.inflatedBytes,0);
    } else {
      auto occupied=std::make_shared<std::vector<std::uint8_t>>(
          *input.observation.collision.measuredOccupiedStorage);
      for(std::size_t i=0;i<view.inflatedBytes;++i) (*occupied)[i]|=view.inflatedBits[i];
      input.observation.collision.measuredOccupiedStorage=occupied;
    }
    const auto output=executor.tick(input);
    EXPECT_EQ(output.pose_stage_count,2U);
    EXPECT_EQ(output.pose_stage_index,1U);
    EXPECT_FALSE(output.goal_reached);
  }
}

TEST(PoseRoute, WideTurnCommandsTranslationAndBoundedYawTogether) {
  auto executor=makeExecutor();executor.setRoute(wideCornerRoute());
  auto input=wideObservation({{0,0,0},0});
  ASSERT_EQ(executor.tick(input).pose_stage_count,2U);
  // Re-anchor a paused run at the turn entrance, so the asynchronous spline
  // starts from the observed pose instead of fabricating travelled distance.
  executor.suspendAutonomy();
  input=wideObservation({{2+std::sqrt(.5),1-std::sqrt(.5),0},M_PI/4},2);
  auto output=executor.tick(input);
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(!output.path_found && std::chrono::steady_clock::now()<deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    output=executor.tick(input);
  }
  ASSERT_TRUE(output.path_found) << output.reason << ':' << output.local_planner_debug.searchReason;
  for(int tick=0;tick<20 && output.cmd_vel.wz<=0;++tick) {
    input.timestampS+=.02;input.observation.odom_stamp_s=input.timestampS;
    output=executor.tick(input);
  }
  EXPECT_GT(std::hypot(output.cmd_vel.vx,output.cmd_vel.vy),0);
  EXPECT_GT(output.cmd_vel.wz,0);
  EXPECT_LE(output.cmd_vel.wz,ExecutorConfig{}.follower.spline.maxYawRateRadS);
  EXPECT_FALSE(output.trajectory_frozen);
  EXPECT_FALSE(output.goal_reached);
}

TEST(PoseRoute, CoupledReferenceCommandsForwardAndYawTogetherAndRetainsTerminalYaw) {
  auto executor=makeExecutor();
  auto route=sampledTurnRoute();
  executor.setRoute(route);
  ASSERT_EQ(executor.tick(observation({route.posePoints.front().position,0})).pose_stage_count,2U);
  executor.suspendAutonomy();
  const auto& middle=route.posePoints[20];
  auto input=observation({middle.position,middle.yaw},2);
  auto output=executor.tick(input);
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(!output.path_found && std::chrono::steady_clock::now()<deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));output=executor.tick(input);
  }
  ASSERT_TRUE(output.path_found)<<output.reason<<':'<<output.local_planner_debug.searchReason;
  for(int tick=0;tick<20 && output.cmd_vel.wz<=0;++tick) {
    input.timestampS+=.02;input.observation.odom_stamp_s=input.timestampS;
    output=executor.tick(input);
  }
  EXPECT_EQ(output.pose_stage_index,1U);
  EXPECT_GT(output.cmd_vel.vx,0);
  EXPECT_GT(output.cmd_vel.wz,0);
  EXPECT_FALSE(output.trajectory_frozen);
  input.mapBody=input.odomBody={route.posePoints.back().position,M_PI/2-.06};
  input.timestampS+=.02;input.observation.odom_stamp_s=input.timestampS;
  output=executor.tick(input);
  EXPECT_FALSE(output.goal_reached);
  EXPECT_EQ(output.reason,"aligning_goal_yaw");
  EXPECT_GT(output.cmd_vel.wz,0);
}

TEST(PoseRoute, UnroundedIntermediateCornerIsNotAStopOrAGoal) {
  auto executor=makeExecutor();executor.setRoute(poseRoute());
  auto input=observation({{0,0,0},0});
  const auto start=executor.tick(input);
  ASSERT_EQ(start.pose_stage_count,2U);
  input.mapBody=input.odomBody={{1,0,0},0};
  input.observation.body_linear_velocity.x=.2;
  input.timestampS+=.02;input.observation.odom_stamp_s=input.timestampS;
  const auto corner=executor.tick(input);
  EXPECT_EQ(corner.pose_stage_index,1U);
  EXPECT_EQ(corner.route_generation,start.route_generation);
  EXPECT_FALSE(corner.goal_reached);
  EXPECT_NE(corner.reason,"aligning_goal_yaw");
  EXPECT_NE(corner.reason,"settling_at_goal");
}

TEST(PoseRoute, CoupledReferenceStillRejectsMeasuredTorsoObstacle) {
  auto executor=makeExecutor();auto route=sampledTurnRoute();executor.setRoute(route);
  auto input=observation({route.posePoints.front().position,0});
  lingtu::nav::tests::CollisionBitmap obstacle;
  obstacle.occupy({.15,.05,.05});
  const auto view=obstacle.view();
  input.observation.collision.measuredOccupiedStorage=
      std::make_shared<const std::vector<std::uint8_t>>(view.inflatedBits,
                                                      view.inflatedBits+view.inflatedBytes);
  auto output=executor.tick(input);
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(output.local_planner_debug.searchReason.empty() &&
        std::chrono::steady_clock::now()<deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));output=executor.tick(input);
  }
  EXPECT_FALSE(output.goal_reached);
  EXPECT_FALSE(output.path_found)<<output.reason;
  EXPECT_DOUBLE_EQ(output.cmd_vel.vx,0);
  EXPECT_DOUBLE_EQ(output.cmd_vel.vy,0);
  EXPECT_DOUBLE_EQ(output.cmd_vel.wz,0);
}

TEST(PoseRoute, WideTurnRetainsNormalHeadingAlignmentGate) {
  ExecutorConfig config;
  config.follower.spline.maxYawRateRadS=.9;
  config.goal_yaw_max_rate=.2;
  auto executor=makeExecutor(config);executor.setRoute(wideCornerRoute());
  auto input=wideObservation({{0,0,0},0});
  ASSERT_EQ(executor.tick(input).pose_stage_count,2U);
  executor.suspendAutonomy();
  input=wideObservation({{2+std::sqrt(.5),1-std::sqrt(.5),0},-3*M_PI/4},2);
  auto output=executor.tick(input);
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(!output.path_found && std::chrono::steady_clock::now()<deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    output=executor.tick(input);
  }
  ASSERT_TRUE(output.path_found) << output.reason << ':' << output.local_planner_debug.searchReason;
  for(int tick=0;tick<20 && !output.trajectory_frozen;++tick) {
    input.timestampS+=.02;input.observation.odom_stamp_s=input.timestampS;
    output=executor.tick(input);
  }
  EXPECT_DOUBLE_EQ(output.cmd_vel.vx,0);
  EXPECT_DOUBLE_EQ(output.cmd_vel.vy,0);
  EXPECT_GT(std::abs(output.cmd_vel.wz),0);
  EXPECT_GT(std::abs(output.cmd_vel.wz),config.goal_yaw_max_rate);
  EXPECT_LE(std::abs(output.cmd_vel.wz),config.follower.spline.maxYawRateRadS);
  EXPECT_TRUE(output.trajectory_frozen);
  input=wideObservation({{3,3,0},M_PI/2-.5},3);
  output=executor.tick(input);
  EXPECT_EQ(output.reason,"aligning_goal_yaw");
  EXPECT_DOUBLE_EQ(output.cmd_vel.wz,config.goal_yaw_max_rate);
}

TEST(PoseRoute, BlendedArrivalCorrectsMeasuredOvershootWithoutTurningBack) {
  auto route=wideCornerRoute();
  route.goalToleranceM=.01;
  auto executor=makeExecutor();executor.setRoute(route);
  auto input=wideObservation({{0,0,0},0});
  ASSERT_EQ(executor.tick(input).pose_stage_count,2U);
  executor.suspendAutonomy();
  input=wideObservation({{3,2.995,0},M_PI/2},2);
  input.observation.body_linear_velocity.x=.05;
  auto output=executor.tick(input);
  ASSERT_EQ(output.reason,"settling_at_goal");
  ASSERT_EQ(output.pose_stage_index,1U);
  // Recorded failure: settling drift crossed the unchanged 1cm stage radius.
  input=wideObservation({{3,3.013,0},M_PI/2},2.02);
  output=executor.tick(input);
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(!output.path_found && std::chrono::steady_clock::now()<deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));output=executor.tick(input);
  }
  ASSERT_TRUE(output.path_found) << output.reason << ':' << output.local_planner_debug.searchReason;
  input.timestampS+=.02;input.observation.odom_stamp_s=input.timestampS;
  output=executor.tick(input);
  EXPECT_EQ(output.pose_stage_index,1U);
  EXPECT_FALSE(output.goal_reached);
  EXPECT_FALSE(output.trajectory_frozen);
  EXPECT_LT(std::abs(output.cmd_vel.wz),.05);
  EXPECT_LT(output.cmd_vel.vx,0);
}

TEST(PoseRoute, MeasuredMillimetreProjectionAtStageStartCannotCreateAMetreDetour) {
  auto route=poseRoute();
  constexpr double yaw=.0012139799594337651;
  route.posePoints={{{8.000136774031164,5.999939126255716,.450763004875826},yaw},
      {{16.000136774031166,5.999939126255716,.450763004875826},yaw}};
  route.goalToleranceM=.025;route.yawToleranceRad=3*M_PI/180;
  auto executor=makeExecutor();executor.setRoute(route);
  auto input=observation({{8.00300876592,6.00119550357,.450763004875826},.00746815260078},47.96);
  lingtu::nav::tests::CollisionBitmap bitmap({-5,-5,-2},{25,15,2},.1);
  input.observation.collision=bitmap.view(47.96);
  const auto bytes=input.observation.collision.inflatedBytes;
  input.observation.collision.measuredOccupiedStorage=
      std::make_shared<const std::vector<std::uint8_t>>(bytes,0);
  input.observation.collision.knownFreeStorage=
      std::make_shared<const std::vector<std::uint8_t>>(bytes,255);
  auto output=executor.tick(input);
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(!output.path_found && std::chrono::steady_clock::now()<deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));output=executor.tick(input);
  }
  ASSERT_TRUE(output.path_found) << output.reason << ':' << output.local_planner_debug.searchReason;
  ASSERT_FALSE(output.local_path_map.empty());
  EXPECT_LE(nav_kernel::distance3D(output.local_path_map.front(),input.mapBody.position),1e-6);
  for(const auto& point:output.local_path_map) {
    EXPECT_GE(point.x,input.mapBody.position.x-.01);
    EXPECT_GE(point.y,route.posePoints.front().position.y-.01);
    EXPECT_LE(point.y,input.mapBody.position.y+.01);
  }
}

TEST(PoseRoute, WideTurnCrabHeadingsKeepTheOriginalCheckedStages) {
  auto route=wideCornerRoute();
  for(auto& point:route.posePoints) point.yaw+=.3;
  route.finalYaw=M_PI/2+.3;
  auto executor=makeExecutor();executor.setRoute(route);
  auto input=wideObservation({{0,0,0},.3});
  EXPECT_EQ(executor.tick(input).pose_stage_count,4U);
}

TEST(PoseRoute, WideTurnKeepsDeclaredStairStages) {
    auto route=wideCornerRoute();
      StairSegment segment;
      segment.flight.id="declared-distant-flight";
      segment.flight.origin={100,100,0};
      segment.flight.axis={1,0,0};
      segment.flight.lateralMin=-1;segment.flight.lateralMax=1;
      segment.flight.treads={{-2,0,-.5},{0,.3,-.35},{.3,3,-.2}};
      segment.bodyHeight=.5;
      segment.entry={100,100,0};segment.exit={100.3,100,.3};
      route.stairs.push_back(segment);
    auto executor=makeExecutor();executor.setRoute(route);
    EXPECT_EQ(executor.tick(observation({{0,0,0},0})).pose_stage_count,4U);
}

TEST(PoseRoute, WideTurnCancelAndFrameChangeClearItsScheduleButResumeRetainsTheGeometry) {
  auto executor=makeExecutor();const auto route=wideCornerRoute();
  executor.setRoute(route);auto input=wideObservation({{0,0,0},0});
  ASSERT_EQ(executor.tick(input).pose_stage_count,2U);
  input.mode=ExecutionMode::MotionIntent;input.motionIntent.vx=-.2;
  input.timestampS+=.02;
  executor.tick(input);
  input.mode=ExecutionMode::Route;input.timestampS+=.02;
  EXPECT_EQ(executor.tick(input).pose_stage_count,2U);
  executor.setRoute(route);input=wideObservation({{0,0,0},0});
  ASSERT_EQ(executor.tick(input).pose_stage_count,2U);
  executor.resumeRoute();input.timestampS+=.02;
  EXPECT_EQ(executor.tick(input).pose_stage_count,2U);
  executor.setRoute(route);input=wideObservation({{0,0,0},0});
  ASSERT_EQ(executor.tick(input).pose_stage_count,2U);
  input.observation.frame_epoch=2;input.timestampS+=.02;
  EXPECT_EQ(executor.tick(input).reason,"pose_route_frame_changed");
  EXPECT_FALSE(executor.hasRoute());
  executor.setRoute(route);input=wideObservation({{0,0,0},0});executor.tick(input);
  executor.clear();input.timestampS+=.02;
  const auto stopped=executor.tick(input);
  EXPECT_EQ(stopped.pose_stage_count,0U);
  EXPECT_DOUBLE_EQ(stopped.cmd_vel.vx,0);
  EXPECT_DOUBLE_EQ(stopped.cmd_vel.wz,0);
}

TEST(PoseRoute, ManualHeadingAndMidCurveResumeUseTheObservedPoseAndRetainedGoal) {
  for(bool steering:{false,true}) {
    auto executor=makeExecutor();executor.setRoute(wideCornerRoute());
    EXPECT_EQ(executor.tick(wideObservation({{0,0,0},0})).pose_stage_count,2U);
    auto input=wideObservation({{2+std::sqrt(.5),1-std::sqrt(.5),0},M_PI/4},2);
    if(steering) {
      input.mode=ExecutionMode::MotionIntent;input.motionIntent.vx=.2;
      auto manual=executor.tick(input);
      const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
      while(!manual.path_found && std::chrono::steady_clock::now()<deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));manual=executor.tick(input);
      }
      ASSERT_TRUE(manual.path_found)<<manual.reason<<':'<<manual.local_planner_debug.searchReason;
      input.timestampS+=.05;input.observation.odom_stamp_s=input.timestampS;
      manual=executor.tick(input);
      EXPECT_NEAR(manual.tracking.headingErrorRad,0,1e-9);
      EXPECT_NEAR(manual.cmd_vel.wz,0,1e-9);
      EXPECT_GT(manual.cmd_vel.vx,0);
      input.mode=ExecutionMode::Route;
    } else executor.resumeRoute();
    auto resumed=executor.tick(input);
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    while(!resumed.path_found && std::chrono::steady_clock::now()<deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));resumed=executor.tick(input);
    }
    ASSERT_TRUE(resumed.path_found)<<resumed.reason<<':'<<resumed.local_planner_debug.searchReason;
    EXPECT_EQ(resumed.pose_stage_count,2U);
    EXPECT_NEAR(nav_kernel::distance3D(resumed.target,{3,3,0}),0,1e-9);
    ASSERT_FALSE(resumed.local_path_map.empty());
    EXPECT_NEAR(nav_kernel::distance3D(resumed.local_path_map.front(),input.mapBody.position),0,1e-6);
    EXPECT_NEAR(resumed.tracking.headingErrorRad,0,1e-9);
    EXPECT_FALSE(resumed.goal_reached);
  }
}

TEST(PoseRoute, MissingArcSupportMetadataUsesTheUnchangedContinuousReference) {
  for(bool missing:{false,true}) {
    auto route=wideCornerRoute();
    if(missing) route.supportHeightM.reset();
    else route.supportHeightToleranceM=-.1;
    auto executor=makeExecutor();executor.setRoute(route);
    EXPECT_EQ(executor.tick(wideObservation({{0,0,0},0})).pose_stage_count,2U);
  }
}

TEST(PoseRoute, MeasuredYawMustCompleteBeforeNextMove) {
  auto executor = makeExecutor();
  auto route = poseRoute();
  route.posePoints = {{{0,0,0},M_PI/2}, {{0,1,0},M_PI/2}};
  executor.setRoute(route);
  auto input = observation({{0,0,0},0});
  for (int i = 0; i < 3; ++i) {
    input.timestampS = 1.0 + i * 0.01;
    input.observation.odom_stamp_s = input.timestampS;
    const auto output = executor.tick(input);
    EXPECT_FALSE(output.goal_reached);
    EXPECT_DOUBLE_EQ(output.cmd_vel.vx, 0.0);
    EXPECT_DOUBLE_EQ(output.cmd_vel.vy, 0.0);
    EXPECT_EQ(output.reason, "aligning_goal_yaw");
    EXPECT_GT(output.cmd_vel.wz, 0.0) << output.reason;
  }
}

TEST(PoseRoute, OldTaskAndFrameChangeCannotKeepPoseState) {
  auto executor = makeExecutor();
  executor.setRoute(poseRoute());
  auto input = observation({{0,0,0},0});
  executor.tick(input);
  input.observation.frame_epoch = 2;
  input.timestampS += 0.01;
  EXPECT_EQ(executor.tick(input).reason, "pose_route_frame_changed");
  EXPECT_FALSE(executor.hasRoute());
  Route old;
  old.points = {{0,0,0}, {2,0,0}};
  EXPECT_NO_THROW(executor.setRoute(old));
  EXPECT_TRUE(executor.hasRoute());
  executor.clear();
  EXPECT_FALSE(executor.hasRoute());
  input.timestampS += 0.01;
  const auto stopped = executor.tick(input);
  EXPECT_DOUBLE_EQ(stopped.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(stopped.cmd_vel.vy, 0.0);
  EXPECT_DOUBLE_EQ(stopped.cmd_vel.wz, 0.0);
  EXPECT_EQ(stopped.pose_stage_count, 0U);
}

TEST(PoseRoute, IntermediateTurnCannotReportTerminalSuccess) {
  auto executor = makeExecutor();
  executor.setRoute(stoppedPoseRoute());
  auto input = observation({{0,0,0},0});
  const auto start = executor.tick(input);
  EXPECT_EQ(start.pose_stage_index, 1U);
  EXPECT_FALSE(start.goal_reached);
  input.mapBody = input.odomBody = {{1,0,0},0};
  input.timestampS += 0.01;
  input.observation.odom_stamp_s = input.timestampS;
  const auto turn = executor.tick(input);
  EXPECT_EQ(turn.pose_stage_index, 2U);
  EXPECT_FALSE(turn.goal_reached);
  EXPECT_EQ(turn.reason, "aligning_goal_yaw");
  EXPECT_GT(turn.cmd_vel.wz, 0.0) << turn.reason;
  EXPECT_DOUBLE_EQ(turn.cmd_vel.vx, 0.0);
  input.timestampS += 0.01;
  input.observation.odom_stamp_s = input.timestampS;
  const auto unchanged = executor.tick(input);
  EXPECT_EQ(unchanged.pose_stage_index, 2U);
  EXPECT_EQ(unchanged.route_generation, turn.route_generation);
  EXPECT_GT(turn.route_generation, start.route_generation);
  EXPECT_FALSE(unchanged.goal_reached);
}

TEST(PoseRoute, MovingRobotCannotAdvanceToTurn) {
  auto executor = makeExecutor();
  executor.setRoute(stoppedPoseRoute());
  auto input = observation({{0,0,0},0});
  executor.tick(input);
  input.mapBody = input.odomBody = {{1,0,0},0};
  input.observation.body_linear_velocity.x = 0.2;
  input.timestampS += 0.01;
  const auto moving = executor.tick(input);
  EXPECT_EQ(moving.pose_stage_index, 1U);
  EXPECT_FALSE(moving.goal_reached);
  EXPECT_DOUBLE_EQ(moving.cmd_vel.vx, 0.0);
  EXPECT_DOUBLE_EQ(moving.cmd_vel.wz, 0.0);
}

TEST(PoseRoute, AlignedInitialTranslationDoesNotRequireAStop) {
  auto executor = makeExecutor();
  executor.setRoute(poseRoute());
  auto input = observation({{0,0,0},0});
  input.observation.body_linear_velocity.x = .2;
  auto output = executor.tick(input);
  ASSERT_EQ(output.pose_stage_index, 1U);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!output.path_found && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    output = executor.tick(input);
  }
  ASSERT_TRUE(output.path_found) << output.reason << ':' << output.local_planner_debug.searchReason;
  EXPECT_EQ(output.pose_stage_index, 1U);
  EXPECT_FALSE(output.trajectory_frozen);
  EXPECT_GT(output.cmd_vel.vx, .1);
}

TEST(PoseRoute, InitialMovingHandoffKeepsPoseAccuracyAndRealTurnStops) {
  auto executor = makeExecutor();
  executor.setRoute(poseRoute());
  auto input = observation({{.02,0,0},0});
  input.observation.body_linear_velocity.x = .2;
  EXPECT_EQ(executor.tick(input).pose_stage_index, 0U);

  executor.setRoute(poseRoute());
  input = observation({{0,0,0},.1});
  input.observation.body_linear_velocity.x = .2;
  EXPECT_EQ(executor.tick(input).pose_stage_index, 0U);

  auto route = poseRoute();
  route.posePoints = {{{0,0,0},0}, {{0,0,0},M_PI/2}, {{0,1,0},M_PI/2}};
  executor.setRoute(route);
  input = observation({{0,0,0},0});
  input.observation.body_linear_velocity.x = .2;
  EXPECT_EQ(executor.tick(input).pose_stage_index, 0U);
  input.timestampS += .02;
  input.observation.odom_stamp_s = input.timestampS;
  input.observation.body_linear_velocity.x = 0;
  EXPECT_EQ(executor.tick(input).pose_stage_index, 1U);
}

TEST(PoseRoute, InitialTurnUsesMeasuredPositionAfterCentimeterBodyDrift) {
  auto route = poseRoute();
  route.posePoints = {{{0,0,0},0}, {{0,0,0},M_PI/2}, {{0,1,0},M_PI/2}};
  auto executor = makeExecutor();
  executor.setRoute(route);
  auto input = observation({{0,0,0},0});
  ASSERT_EQ(executor.tick(input).pose_stage_index, 1U);
  input.mapBody = input.odomBody = {{.011,0,-.011},.5};
  input.observation.body_geometry = route.body;
  input.timestampS += .02;
  input.observation.odom_stamp_s = input.timestampS;
  const auto turning = executor.tick(input);
  EXPECT_EQ(turning.pose_stage_index, 1U);
  EXPECT_EQ(turning.reason, "aligning_goal_yaw");
  EXPECT_DOUBLE_EQ(turning.cmd_vel.vx, 0);
  EXPECT_DOUBLE_EQ(turning.cmd_vel.vy, 0);
  EXPECT_GT(turning.cmd_vel.wz, 0);

  input.mapBody = input.odomBody = {{.03,0,-.011},M_PI/2};
  input.timestampS += .02;
  input.observation.odom_stamp_s = input.timestampS;
  const auto translated = executor.tick(input);
  EXPECT_EQ(translated.pose_stage_index, 2U);
  EXPECT_FALSE(translated.goal_reached);
  EXPECT_NE(translated.reason, "aligning_goal_yaw");
}

TEST(PoseRoute, InitialTurnAtMeasuredPositionStillChecksTheWholeSweep) {
  auto route = poseRoute();
  route.posePoints = {{{0,0,0},0}, {{0,0,0},M_PI/2}, {{0,1,0},M_PI/2}};
  auto executor = makeExecutor();
  executor.setRoute(route);
  ASSERT_EQ(executor.tick(observation({{0,0,0},0})).pose_stage_index, 1U);
  auto input = observation({{.011,0,0},0}, 1.02);
  lingtu::nav::tests::CollisionBitmap obstacle({-1,-1,-1},{1,1,1},.025);
  obstacle.occupy({.14,.14,0});
  input.observation.collision = obstacle.view(1.02);
  const auto &view = input.observation.collision;
  input.observation.collision.measuredOccupiedStorage =
      std::make_shared<const std::vector<std::uint8_t>>(view.inflatedBits,
                                                       view.inflatedBits + view.inflatedBytes);
  input.observation.collision.knownFreeStorage =
      std::make_shared<const std::vector<std::uint8_t>>(view.inflatedBytes, 255);
  const auto blocked = executor.tick(input);
  EXPECT_EQ(blocked.pose_stage_index, 1U);
  EXPECT_EQ(blocked.reason, "pose_turn_blocked");
  EXPECT_FALSE(blocked.path_found);
  EXPECT_DOUBLE_EQ(blocked.cmd_vel.vx, 0);
  EXPECT_DOUBLE_EQ(blocked.cmd_vel.vy, 0);
  EXPECT_DOUBLE_EQ(blocked.cmd_vel.wz, 0);
}

TEST(PoseRoute, IntermediateTurnReconnectsFromMeasuredPosition) {
  auto executor = makeExecutor();
  executor.setRoute(stoppedPoseRoute());
  ASSERT_EQ(executor.tick(observation({{0,0,0},0})).pose_stage_index, 1U);
  ASSERT_EQ(executor.tick(observation({{1,0,0},0},1.02)).pose_stage_index, 2U);
  const auto drifted = executor.tick(observation({{1.011,0,0},M_PI/2},1.04));
  EXPECT_EQ(drifted.pose_stage_index, 3U);
  EXPECT_FALSE(drifted.goal_reached);
}

TEST(PoseRoute, InternalTurnUsesNavigationToleranceWithoutAnObservedBodySnapshot) {
  auto executor = makeExecutor();
  executor.setRoute(stoppedPoseRoute());
  ASSERT_EQ(executor.tick(observation({{0,0,0},0})).pose_stage_index, 1U);
  const auto turning = executor.tick(observation({{.96,0,.06},0},1.02));
  EXPECT_EQ(turning.pose_stage_index, 2U);
  EXPECT_EQ(turning.reason, "aligning_goal_yaw");
  EXPECT_GT(turning.cmd_vel.wz, 0.0);
  EXPECT_FALSE(turning.goal_reached);
}

TEST(PoseRoute, AlignedInternalTurnHandsMeasuredYawMotionToTheNextSpline) {
  auto executor = makeExecutor();
  executor.setRoute(stoppedPoseRoute());
  ASSERT_EQ(executor.tick(observation({{0,0,0},0})).pose_stage_index, 1U);
  ASSERT_EQ(executor.tick(observation({{1,0,0},0},1.02)).pose_stage_index, 2U);
  auto input = observation({{1.04,0,.06},M_PI/2-.06},1.04);
  input.observation.body_linear_velocity.x = .05;
  input.observation.body_yaw_rate = .18;
  const auto moving = executor.tick(input);
  EXPECT_EQ(moving.pose_stage_index, 3U);
  EXPECT_NE(moving.reason, "settling_before_pose_turn");
  EXPECT_FALSE(moving.goal_reached);
}

TEST(PoseRoute, InitialTurnDoesNotAbsorbManualRelocationOrHeightChange) {
  for (const nav_kernel::Vec3 position : {nav_kernel::Vec3{.151,0,0}, {0,0,.351}, {1,0,0}}) {
    SCOPED_TRACE(position.x);
    auto route = poseRoute();
    route.posePoints = {{{0,0,0},0}, {{0,0,0},M_PI/2}, {{0,1,0},M_PI/2}};
    ExecutorConfig config;
    config.goal_reached_m = .15;
    auto executor = makeExecutor(config);
    executor.setRoute(route);
    ASSERT_EQ(executor.tick(observation({{0,0,0},0})).pose_stage_index, 1U);
    executor.resumeRoute();
    auto input = observation({position,M_PI/2}, 1.02);
    input.observation.body_geometry = route.body;
    const auto moved = executor.tick(input);
    EXPECT_EQ(moved.pose_stage_index, 1U);
    EXPECT_NE(moved.reason, "aligning_goal_yaw");
    EXPECT_FALSE(moved.goal_reached);
  }
}

TEST(PoseRoute, InitialStanceDriftDoesNotDependOnMapCellBoundary) {
  for (const double drift : {.011, .0249, .0251, .031, .0499, .0501, .051}) {
    SCOPED_TRACE(drift);
    auto route = poseRoute();
    route.posePoints = {{{0,0,0},0}, {{0,0,0},M_PI/2}, {{0,1,0},M_PI/2}};
    ExecutorConfig config;
    config.goal_reached_m = .15;
    auto executor = makeExecutor(config);
    executor.setRoute(route);
    ASSERT_EQ(executor.tick(observation({{0,0,0},0})).pose_stage_index, 1U);
    auto input = observation({{drift,0,-.011},.5}, 1.02);
    input.observation.body_geometry = route.body;
    EXPECT_EQ(executor.tick(input).reason, "aligning_goal_yaw");
    input.mapBody.yaw = input.odomBody.yaw = M_PI/2;
    input.timestampS = input.observation.odom_stamp_s = 1.04;
    EXPECT_EQ(executor.tick(input).pose_stage_index, 2U);
  }
}

TEST(PoseRoute, InitialObservedStanceReusesAdmittedSupportHeightTolerance) {
  for (const double heave : {.021, .03, .08, .101}) {
    SCOPED_TRACE(heave);
    auto route = poseRoute();
    route.posePoints = {{{0,0,0},0}, {{0,0,0},M_PI/2}, {{0,1,0},M_PI/2}};
    route.supportHeightM = .435;
    route.supportHeightToleranceM = .1;
    auto executor = makeExecutor();
    executor.setRoute(route);
    ASSERT_EQ(executor.tick(observation({{0,0,0},0})).pose_stage_index, 1U);
    auto input = observation({{.011,0,heave},M_PI/2}, 1.02);
    input.observation.body_geometry = route.body;
    const auto output = executor.tick(input);
    EXPECT_EQ(output.pose_stage_index, heave <= .1 ? 2U : 1U);
    EXPECT_FALSE(output.goal_reached);
  }
}

TEST(PoseRoute, InitialRelocationOutsideArrivalRadiusUsesScanToReconnect) {
  auto route = poseRoute();
  route.posePoints = {{{0,0,0},0}, {{0,0,0},M_PI/2}, {{0,1,0},M_PI/2}};
  ExecutorConfig config;
  config.goal_reached_m = .15;
  auto executor = makeExecutor(config);
  executor.setRoute(route);
  ASSERT_EQ(executor.tick(observation({{0,0,0},0})).pose_stage_index, 1U);
  executor.resumeRoute();
  auto input = observation({{.151,0,0},M_PI/2}, 1.02);
  input.observation.body_geometry = route.body;
  bool moving = false;
  ExecutionOutput output;
  for (int tick = 0; tick < 500; ++tick) {
    output = executor.tick(input);
    moving = moving || std::hypot(output.cmd_vel.vx, output.cmd_vel.vy) > .001;
    if (output.pose_stage_index == 2U) break;
    const double yaw = input.mapBody.yaw;
    input.mapBody.position.x += .02*(std::cos(yaw)*output.cmd_vel.vx-std::sin(yaw)*output.cmd_vel.vy);
    input.mapBody.position.y += .02*(std::sin(yaw)*output.cmd_vel.vx+std::cos(yaw)*output.cmd_vel.vy);
    input.mapBody.yaw += .02*output.cmd_vel.wz;
    input.odomBody = input.mapBody;
    input.timestampS = input.observation.odom_stamp_s = 1.04 + .02*tick;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_TRUE(moving) << output.reason;
  EXPECT_EQ(output.pose_stage_index, 2U) << output.reason;
}

TEST(PoseRoute, FiniteVerticalMotionDoesNotBlockStageChanges) {
  for (const double vertical_speed : {-.3, .1, .3}) {
    SCOPED_TRACE(vertical_speed);
    auto executor = makeExecutor();
    executor.setRoute(stoppedPoseRoute());
    auto input = observation({{0,0,0},0});
    input.observation.body_linear_velocity.z = vertical_speed;
    ASSERT_EQ(executor.tick(input).pose_stage_index, 1U);

    input.mapBody = input.odomBody = {{1,0,0},0};
    input.timestampS += .02;
    input.observation.odom_stamp_s = input.timestampS;
    const auto turn = executor.tick(input);
    EXPECT_EQ(turn.pose_stage_index, 2U);
    EXPECT_EQ(turn.reason, "aligning_goal_yaw");
    EXPECT_GT(turn.cmd_vel.wz, 0);

    input.mapBody.yaw = input.odomBody.yaw = M_PI/2;
    input.timestampS += .02;
    input.observation.odom_stamp_s = input.timestampS;
    EXPECT_EQ(executor.tick(input).pose_stage_index, 3U);
  }
}

TEST(PoseRoute, VerticalMotionCannotHideHorizontalMotionOrUnreachedHeight) {
  for (int moving_axis = 0; moving_axis < 3; ++moving_axis) {
    SCOPED_TRACE(moving_axis);
    auto executor = makeExecutor();
    executor.setRoute(stoppedPoseRoute());
    auto input = observation({{0,0,0},0});
    ASSERT_EQ(executor.tick(input).pose_stage_index, 1U);
    input.mapBody = input.odomBody = {{1,0,0},0};
    input.observation.body_linear_velocity.z = .3;
    if (moving_axis == 0) input.observation.body_linear_velocity.x = .04;
    if (moving_axis == 1) input.observation.body_linear_velocity.y = .04;
    if (moving_axis == 2) input.observation.body_yaw_rate = .09;
    input.timestampS += .02;
    input.observation.odom_stamp_s = input.timestampS;
    EXPECT_EQ(executor.tick(input).pose_stage_index, 1U);

    input.observation.body_linear_velocity.x = input.observation.body_linear_velocity.y = 0;
    input.observation.body_yaw_rate = 0;
    input.mapBody.position.z = input.odomBody.position.z = .351;
    input.timestampS += .02;
    input.observation.odom_stamp_s = input.timestampS;
    EXPECT_EQ(executor.tick(input).pose_stage_index, 1U);
  }
}

TEST(PoseRoute, TwentyMillimeterResidualStartsValidatedTurn) {
  auto executor = makeExecutor();
  executor.setRoute(stoppedPoseRoute());
  auto input = observation({{0,0,0},0});
  executor.tick(input);
  input.mapBody = input.odomBody = {{0.98,0,0},0};
  input.timestampS += 0.01;
  const auto early = executor.tick(input);
  EXPECT_EQ(early.pose_stage_index, 2U);
  EXPECT_FALSE(early.goal_reached);
}

TEST(PoseRoute, MeasuredIntermediateTurnCannotInheritNominalClearance) {
  auto executor = makeExecutor();
  executor.setRoute(stoppedPoseRoute());
  executor.tick(observation({{0,0,0},0}));
  auto input = observation({{.98,0,0},0}, 1.02);
  lingtu::nav::tests::CollisionBitmap obstacle({-1,-1,-1},{3,2,1},.025);
  obstacle.occupy({1.12,.14,0});
  input.observation.collision = obstacle.view(1.02);
  const auto &view = input.observation.collision;
  input.observation.collision.measuredOccupiedStorage =
      std::make_shared<const std::vector<std::uint8_t>>(view.inflatedBits,
                                                       view.inflatedBits + view.inflatedBytes);
  input.observation.collision.knownFreeStorage =
      std::make_shared<const std::vector<std::uint8_t>>(view.inflatedBytes, 255);
  const auto blocked = executor.tick(input);
  EXPECT_EQ(blocked.pose_stage_index, 1U);
  EXPECT_FALSE(blocked.goal_reached);
}

TEST(PoseRoute, MeasuredIntermediateHandoffAcceptsExecutableHeadingResidual) {
  auto executor = makeExecutor();
  executor.setRoute(stoppedPoseRoute());
  executor.tick(observation({{0,0,0},0}));
  executor.tick(observation({{1,0,0},0},1.02));
  const auto next = executor.tick(observation({{1.014,.016,0},M_PI/2+.075},1.04));
  EXPECT_EQ(next.pose_stage_index, 3U);
  EXPECT_FALSE(next.goal_reached);
}

TEST(PoseRoute, MeasuredIntermediateHeaveUsesAdmittedSupportTolerance) {
  auto route = stoppedPoseRoute();
  route.supportHeightM = .435;
  route.supportHeightToleranceM = .1;
  auto executor = makeExecutor();
  executor.setRoute(route);
  executor.tick(observation({{0,0,0},0}));
  auto input = observation({{.9845,.001,-.021},0},1.02);
  input.observation.body_geometry = route.body;
  const auto turning = executor.tick(input);
  EXPECT_EQ(turning.pose_stage_index, 2U);
  EXPECT_EQ(turning.reason, "aligning_goal_yaw");
  EXPECT_GT(turning.cmd_vel.wz, 0);
  EXPECT_DOUBLE_EQ(turning.cmd_vel.vx, 0);
  EXPECT_FALSE(turning.goal_reached);
}

TEST(PoseRoute, MeasuredIntermediateHeaveStillChecksActualTurnCollision) {
  auto route = stoppedPoseRoute();
  route.supportHeightM = .435;
  route.supportHeightToleranceM = .1;
  auto executor = makeExecutor();
  executor.setRoute(route);
  executor.tick(observation({{0,0,0},0}));
  auto input = observation({{.9845,.001,-.021},0},1.02);
  input.observation.body_geometry = route.body;
  lingtu::nav::tests::CollisionBitmap obstacle({-1,-1,-1},{3,2,1},.025);
  obstacle.occupy({1.12,.14,-.025});
  input.observation.collision = obstacle.view(1.02);
  const auto &view = input.observation.collision;
  input.observation.collision.measuredOccupiedStorage =
      std::make_shared<const std::vector<std::uint8_t>>(view.inflatedBits,
                                                       view.inflatedBits + view.inflatedBytes);
  input.observation.collision.knownFreeStorage =
      std::make_shared<const std::vector<std::uint8_t>>(view.inflatedBytes, 255);
  const auto blocked = executor.tick(input);
  EXPECT_EQ(blocked.pose_stage_index, 1U);
  EXPECT_FALSE(blocked.goal_reached);
}

TEST(PoseRoute, MeasuredIntermediateArrivalStopsBeforeAdvancingTheTurn) {
  auto route = stoppedPoseRoute();
  route.supportHeightM = .435;
  route.supportHeightToleranceM = .1;
  auto executor = makeExecutor();
  executor.setRoute(route);
  executor.tick(observation({{0,0,0},0}));
  auto input = observation({{.9845,.001,-.021},0},1.02);
  input.observation.body_geometry = route.body;
  input.observation.body_linear_velocity.x = .065;
  const auto stopping = executor.tick(input);
  EXPECT_EQ(stopping.pose_stage_index, 1U);
  EXPECT_EQ(stopping.reason, "settling_before_pose_turn");
  EXPECT_DOUBLE_EQ(stopping.cmd_vel.vx, 0);
  EXPECT_DOUBLE_EQ(stopping.cmd_vel.vy, 0);
  EXPECT_DOUBLE_EQ(stopping.cmd_vel.wz, 0);
  input.observation.body_linear_velocity.x = 0;
  input.timestampS = input.observation.odom_stamp_s = 1.04;
  const auto turning = executor.tick(input);
  EXPECT_EQ(turning.pose_stage_index, 2U);
  EXPECT_EQ(turning.reason, "aligning_goal_yaw");
  EXPECT_GT(turning.cmd_vel.wz, 0);
}

TEST(PoseRoute, ObservedArticulationAllowsBoundedHeaveAndValidatedXYResidual) {
  auto executor = makeExecutor();
  auto route = stoppedPoseRoute();
  executor.setRoute(route);
  auto input = observation({{0,0,0},0});
  executor.tick(input);
  input.mapBody = input.odomBody = {{1,0,-0.015},0};
  input.timestampS += .02;
  input.observation.odom_stamp_s = input.timestampS;
  const auto fixed = executor.tick(input);
  EXPECT_EQ(fixed.pose_stage_index, 2U);
  input.observation.body_geometry = route.body;
  input.timestampS += .02;
  input.observation.odom_stamp_s = input.timestampS;
  const auto measured = executor.tick(input);
  EXPECT_EQ(measured.pose_stage_index, 2U);
  EXPECT_EQ(measured.reason, "aligning_goal_yaw");
  EXPECT_GT(measured.cmd_vel.wz, 0.0);

  executor.setRoute(route);
  input = observation({{0,0,0},0});
  executor.tick(input);
  input.observation.body_geometry = route.body;
  input.mapBody = input.odomBody = {{.98,0,-0.009},0};
  input.timestampS += .02;
  EXPECT_EQ(executor.tick(input).pose_stage_index, 2U);
  executor.setRoute(route);
  executor.tick(observation({{0,0,0},0}));
  input.mapBody = input.odomBody = {{1,0,-0.351},0};
  input.timestampS += .02;
  EXPECT_EQ(executor.tick(input).pose_stage_index, 1U);
}

TEST(PoseRoute, BalancingRatesNeedStableMeasuredPoseBeforeTerminalSuccess) {
  auto executor = makeExecutor();
  auto route = poseRoute();
  route.posePoints = {{{0,0,0},0}, {{1,0,0},0}};
  route.goalToleranceM = .025;
  executor.setRoute(route);
  auto input = observation({{0,0,0},0});
  executor.tick(input);
  input.observation.body_geometry = route.body;
  input.observation.body_yaw_rate = .20;
  input.observation.body_linear_velocity.z = .04;
  ExecutionOutput output;
  for(int i=0;i<=26;++i) {
    input.mapBody=input.odomBody={{1, .001*std::sin(i)}, .003*std::sin(i)};
    input.timestampS=2.+i*.02;
    input.observation.odom_stamp_s=input.timestampS;
    output=executor.tick(input);
    if(i<25) EXPECT_FALSE(output.goal_reached);
  }
  EXPECT_TRUE(output.goal_reached);
  EXPECT_DOUBLE_EQ(output.cmd_vel.vx,0);
  EXPECT_DOUBLE_EQ(output.cmd_vel.wz,0);

  executor.setRoute(route);
  input=observation({{0,0,0},0});executor.tick(input);
  input.observation.body_geometry=route.body;
  input.observation.body_linear_velocity.x=.1;
  for(int i=0;i<24;++i) {
    input.mapBody=input.odomBody={{.977+i*.002,0,0},0};
    input.timestampS=3.+i*.02;input.observation.odom_stamp_s=input.timestampS;
    EXPECT_FALSE(executor.tick(input).goal_reached);
  }
  executor.setRoute(route);
  input=observation({{0,0,0},0});executor.tick(input);
  input.observation.body_geometry=route.body;
  input.observation.body_linear_velocity.z=.1;
  for(int i=0;i<32;++i) {
    input.mapBody=input.odomBody={{1,0,-i*.002},0};
    input.timestampS=4.+i*.02;input.observation.odom_stamp_s=input.timestampS;
    EXPECT_FALSE(executor.tick(input).goal_reached);
  }
}

TEST(PoseRoute, ArticulatedYawSettlesThroughoutTheRequestedTolerance) {
  auto executor=makeExecutor();auto route=poseRoute();
  route.posePoints={{{0,0,0},0},{{1,0,0},0}};
  route.finalYaw=0;
  route.yawToleranceRad=.05;executor.setRoute(route);
  auto input=observation({{0,0,0},0});executor.tick(input);
  input.observation.body_geometry=route.body;
  input.mapBody=input.odomBody={{1,0,0},-.04};
  for (int i=0;i<=26;++i) {
    input.timestampS=2+i*.02;input.observation.odom_stamp_s=input.timestampS;
    const auto output=executor.tick(input);
    EXPECT_DOUBLE_EQ(output.cmd_vel.wz,0);
    EXPECT_EQ(output.reason,i<25 ? "settling_at_goal" : "goal_reached");
    EXPECT_EQ(output.goal_reached,i>=25);
  }
}

TEST(PoseRoute, LeavingRequestedYawToleranceRestartsArrivalDwell) {
  auto executor=makeExecutor();auto route=poseRoute();
  route.posePoints={{{0,0,0},0},{{1,0,0},0}};
  route.finalYaw=0;
  route.yawToleranceRad=.05;executor.setRoute(route);
  auto input=observation({{0,0,0},0});executor.tick(input);
  input.observation.body_geometry=route.body;
  input.mapBody=input.odomBody={{1,0,0},-.04};
  for (int i=0;i<20;++i) {
    input.timestampS=2+i*.02;input.observation.odom_stamp_s=input.timestampS;
    EXPECT_EQ(executor.tick(input).reason,"settling_at_goal");
  }
  input.mapBody=input.odomBody={{1,0,0},-.051};
  input.timestampS=2.4;input.observation.odom_stamp_s=2.4;
  const auto escaped=executor.tick(input);
  EXPECT_FALSE(escaped.goal_reached);
  EXPECT_EQ(escaped.reason,"aligning_goal_yaw");
  EXPECT_DOUBLE_EQ(escaped.cmd_vel.wz,.2);
  input.mapBody=input.odomBody={{1,0,0},-.04};
  for (int i=0;i<=26;++i) {
    input.timestampS=2.42+i*.02;input.observation.odom_stamp_s=input.timestampS;
    const auto output=executor.tick(input);
    EXPECT_DOUBLE_EQ(output.cmd_vel.wz,0);
    if (i<25) EXPECT_FALSE(output.goal_reached);
    if (i==26) EXPECT_TRUE(output.goal_reached);
  }
}

TEST(PoseRoute, MinimumGoalYawRateRetainsCollisionStop) {
  for (const double direction : {-1.0,1.0}) {
    auto executor=makeExecutor();auto route=poseRoute();
    route.posePoints={{{0,0,0},0},{{1,0,0},0}};
    route.finalYaw=0;
    route.yawToleranceRad=.05;executor.setRoute(route);
    auto input=observation({{0,0,0},0});executor.tick(input);
    input.observation.body_geometry=route.body;
    input.mapBody=input.odomBody={{1,0,0},direction*.06};
    input.timestampS=2;input.observation.odom_stamp_s=2;
    const auto aligning=executor.tick(input);
    EXPECT_EQ(aligning.reason,"aligning_goal_yaw");
    EXPECT_DOUBLE_EQ(aligning.cmd_vel.wz,-direction*.2);
    input.observation.collision.measuredOccupiedStorage=
        std::make_shared<std::vector<std::uint8_t>>(
            input.observation.collision.inflatedBytes,255);
    input.timestampS+=.02;input.observation.odom_stamp_s=input.timestampS;
    const auto blocked=executor.tick(input);
    EXPECT_EQ(blocked.reason,"pose_turn_blocked");
    EXPECT_TRUE(blocked.near_field_stop);
    EXPECT_FALSE(blocked.path_found);
    EXPECT_DOUBLE_EQ(blocked.cmd_vel.vx,0);
    EXPECT_DOUBLE_EQ(blocked.cmd_vel.vy,0);
    EXPECT_DOUBLE_EQ(blocked.cmd_vel.wz,0);
  }
}

TEST(PoseRoute, ZeroGeometryMarginRetainsPositiveStageArrivalTolerance) {
  auto executor = makeExecutor();
  auto route = stoppedPoseRoute();
  auto body = std::make_shared<lingtu::nav::collision::Body>(*route.body);
  body->margin_m = 0;
  route.body = body;
  executor.setRoute(route);
  auto input = observation({{0,0,0},0});
  EXPECT_EQ(executor.tick(input).pose_stage_index,1U);
  input.mapBody = input.odomBody = {{.995,0,-.005},0};
  input.timestampS += .02;
  input.observation.odom_stamp_s = input.timestampS;
  const auto turn = executor.tick(input);
  EXPECT_EQ(turn.pose_stage_index,2U);
  EXPECT_EQ(turn.reason,"aligning_goal_yaw");
  EXPECT_GT(turn.cmd_vel.wz,0);
  input.mapBody = input.odomBody = {{1,0,0},M_PI/2};
  input.timestampS += .02;
  input.observation.odom_stamp_s = input.timestampS;
  EXPECT_EQ(executor.tick(input).pose_stage_index,3U);
}

TEST(PoseRoute, StairProfilesRetainTheBodyAndMapGeometryInBothFramesAndDirections) {
  constexpr double axisYaw = .6;
  for (const auto frame : {PlanningFrame::Map,PlanningFrame::Odom}) {
    for (bool descending : {false,true}) {
      SCOPED_TRACE(static_cast<int>(frame));
      SCOPED_TRACE(descending);
      ExecutorConfig config;
      config.planning_frame = frame;
      auto body = std::make_shared<lingtu::nav::collision::Body>(*poseRoute().body);
      body->margin_m = 0;
      config.body = body;
      nav_kernel::LocalPlannerParams params;
      params.backend = nav_kernel::LocalPlannerBackend::Scan;
      params.scan.voxelResolution = .1;
      nav_kernel::LocalPlanner planner(params);
      ASSERT_TRUE(planner.configure(""));
      Executor executor(config,std::move(planner));
      StairFlight flight;
      flight.id = "pose-stairs";
      flight.axis = {std::cos(axisYaw),std::sin(axisYaw),0};
      flight.lateralMin = -1;
      flight.lateralMax = 1;
      flight.treads = {{-2,0,0},{0,.3,.15},{.3,.6,.3},
                        {.6,.9,.45},{.9,3,.6}};
      Route guide;
      guide.points = {{descending ? -.45 : -.4,0,.4},{-.01,0,.4},
          {.01,0,.55},{.29,0,.55},{.31,0,.7},{.59,0,.7},
          {.61,0,.85},{.89,0,.85},{.91,0,1},{descending ? 1.3 : 1.35,0,1}};
      for (auto &point : guide.points) {
        point.y = point.x*std::sin(axisYaw);
        point.x *= std::cos(axisYaw);
      }
      if (descending) std::reverse(guide.points.begin(),guide.points.end());
      const double heading = nav_kernel::normalizeAngle(axisYaw+(descending ? M_PI : 0));
      for (const auto &point : guide.points) guide.posePoints.push_back({point,0});
      guide.stairs = resolveStairSegments(guide.points,{flight},.4);
      ASSERT_EQ(guide.stairs.size(),1U);
      ASSERT_NO_THROW(executor.setRoute(guide));
      EXPECT_EQ(executor.body(),body);
      auto input = observation({guide.points.front(),heading});
      input.mapFromOdom = {{1,-2,.15},-.7};
      const auto update = [&](double now) {
        input.timestampS = now;
        input.observation = observation(input.mapBody,now).observation;
        input.odomBody.position = input.mapFromOdom.odomPointFromMap(input.mapBody.position);
        input.odomBody.yaw = nav_kernel::normalizeAngle(input.mapBody.yaw-input.mapFromOdom.yaw);
      };
      update(1);
      const auto requirement = descending ? StairProfile::Descent : StairProfile::Ascent;
      const auto switching = executor.tick(input);
      EXPECT_EQ(switching.stairs.request,requirement);
      EXPECT_TRUE(switching.stairs.hold);
      EXPECT_FALSE(switching.goal_reached);
      update(1.02);
      input.stairProfile = {requirement,true,false,"stairs-model","",1.02};
      auto moving = executor.tick(input);
      const auto deadline = std::chrono::steady_clock::now()+std::chrono::seconds(2);
      while (!moving.path_found && std::chrono::steady_clock::now()<deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        moving = executor.tick(input);
      }
      ASSERT_TRUE(moving.path_found) << moving.reason << ':' << moving.local_planner_debug.searchReason;
      EXPECT_EQ(moving.pose_stage_index,1U);
      EXPECT_EQ(moving.stairs.phase,StairPhase::Traversing);
      EXPECT_EQ(moving.stairs.request,requirement);
      EXPECT_NEAR(moving.stairs.bodyYaw.value(),heading,1e-9);
      EXPECT_LE(std::hypot(moving.cmd_vel.vx,moving.cmd_vel.vy),config.stairs.maxSpeed+1e-9);
      EXPECT_FALSE(moving.recovery_verified);
      input.mapBody = {guide.points.back(),heading};
      update(1.04);
      input.stairProfile.stampS = 1.04;
      const auto restoring = executor.tick(input);
      EXPECT_EQ(restoring.stairs.request,StairProfile::Restore);
      EXPECT_TRUE(restoring.stairs.hold);
      EXPECT_FALSE(restoring.goal_reached);
      update(1.06);
      input.stairProfile = {StairProfile::Restore,true,false,"ordinary-model","",1.06};
      const auto restored = executor.tick(input);
      EXPECT_EQ(restored.stairs.phase,StairPhase::Ordinary);
      EXPECT_EQ(restored.reason,"settling_at_goal");
      EXPECT_DOUBLE_EQ(restored.cmd_vel.wz,0);

      // Cancellation on the flight retains its profile, pose and body model.
      executor.setRoute(guide);
      input.mapBody = {guide.points.front(),heading};
      update(1.08);
      input.stairProfile = {};
      executor.tick(input);
      update(1.10);
      input.stairProfile = {requirement,true,false,"stairs-model","",1.10};
      executor.tick(input);
      input.mapBody = {{.45*std::cos(axisYaw),.45*std::sin(axisYaw),.7},heading};
      update(1.12);
      input.stairProfile.stampS = 1.12;
      executor.tick(input);
      executor.clear();
      update(1.14);
      input.stairProfile.stampS = 1.14;
      const auto cancelled = executor.tick(input);
      EXPECT_EQ(cancelled.stairs.reason,"stair_hold_on_flight");
      EXPECT_TRUE(cancelled.stairs.hold);
      EXPECT_EQ(executor.body(),body);
      EXPECT_DOUBLE_EQ(cancelled.cmd_vel.vx,0);
      EXPECT_DOUBLE_EQ(cancelled.cmd_vel.vy,0);
      EXPECT_DOUBLE_EQ(cancelled.cmd_vel.wz,0);
    }
  }
}

TEST(PoseRoute, StairApproachTurnKeepsTheBodyCollisionQuery) {
  auto executor = makeExecutor();
  StairFlight flight;
  flight.id = "non-bin-pose-stairs";
  flight.axis = {std::cos(.6),std::sin(.6),0};
  flight.lateralMin = -1;
  flight.lateralMax = 1;
  flight.treads = {{-2,0,0},{0,.3,.15},{.3,.6,.3},
                    {.6,.9,.45},{.9,3,.6}};
  Route guide;
  guide.body = poseRoute().body;
  guide.points = {{-.4,0,.4},{-.01,0,.4},{.01,0,.55},
      {.29,0,.55},{.31,0,.7},{.59,0,.7},{.61,0,.85},
      {.89,0,.85},{.91,0,1},{1.35,0,1}};
  for (auto &point : guide.points) {
    point.y = point.x*std::sin(.6);
    point.x *= std::cos(.6);
    guide.posePoints.push_back({point,0});
  }
  guide.stairs = resolveStairSegments(guide.points,{flight},.4);
  ASSERT_EQ(guide.stairs.size(),1U);
  executor.setRoute(guide);
  auto input = observation({guide.points.front(),0});
  const auto aligning = executor.tick(input);
  EXPECT_EQ(aligning.reason,"stair_aligning");
  EXPECT_GT(aligning.cmd_vel.wz,0);
  EXPECT_EQ(aligning.stairs.phase,StairPhase::Approach);
  EXPECT_EQ(aligning.stairs.request,StairProfile::None);
  EXPECT_DOUBLE_EQ(aligning.cmd_vel.vx,0);
  EXPECT_DOUBLE_EQ(aligning.cmd_vel.vy,0);
  input.mapBody.yaw = .47;
  input.odomBody = input.mapBody;
  input.timestampS = 1.01;
  input.observation.odom_stamp_s = 1.01;
  const auto nearAligned = executor.tick(input);
  EXPECT_EQ(nearAligned.reason,"stair_aligning");
  EXPECT_DOUBLE_EQ(nearAligned.cmd_vel.wz,.2);
  input.mapBody.yaw = .6;
  input.odomBody = input.mapBody;
  input.timestampS = 1.02;
  input.observation.odom_stamp_s = 1.02;
  EXPECT_EQ(executor.tick(input).stairs.request,StairProfile::Ascent);

  auto withoutEvidence = makeExecutor();
  withoutEvidence.setRoute(guide);
  auto missing = observation({guide.points.front(),0});
  missing.observation.collision.measuredOccupiedStorage.reset();
  const auto blocked = withoutEvidence.tick(missing);
  EXPECT_EQ(blocked.reason,"stair_turn_blocked");
  EXPECT_TRUE(blocked.near_field_stop);
  EXPECT_DOUBLE_EQ(blocked.cmd_vel.wz,0);
}

TEST(PoseRoute, OnFlightGoalRetainsExplicitRequestedYawButPositionOnlyGoalUsesFlightHeading) {
  for (bool requestedYaw : {false,true}) {
    SCOPED_TRACE(requestedYaw);
    auto executor = makeExecutor();
    StairFlight flight;
    flight.id = "on-flight-goal";
    flight.lateralMin = -1;
    flight.lateralMax = 1;
    flight.treads = {{-2,0,0},{0,.3,.15},{.3,.6,.3},
                      {.6,.9,.45},{.9,3,.6}};
    Route guide;
    guide.body = poseRoute().body;
    guide.points = {{-.4,0,.4},{-.01,0,.4},{.01,0,.55},
        {.29,0,.55},{.31,0,.7},{.45,0,.7}};
    for (const auto &point : guide.points) guide.posePoints.push_back({point,.6});
    if (requestedYaw) guide.finalYaw = .6;
    guide.stairs = resolveStairSegments(guide.points,{flight},.4);
    ASSERT_EQ(guide.stairs.size(),1U);
    executor.setRoute(guide);
    auto input = observation({guide.points.front(),0});
    ASSERT_EQ(executor.tick(input).stairs.request,StairProfile::Ascent);
    input.timestampS = 1.02;
    input.observation.odom_stamp_s = 1.02;
    input.stairProfile = {StairProfile::Ascent,true,false,"stairs-model","",1.02};
    executor.tick(input);
    input.mapBody = input.odomBody = {guide.points.back(),0};
    ExecutionOutput output;
    for (int tick = 0; tick <= 30; ++tick) {
      const double now = 1.04+tick*.02;
      input.timestampS = now;
      input.observation = observation(input.mapBody,now).observation;
      input.stairProfile.stampS = now;
      output = executor.tick(input);
      if (requestedYaw) {
        EXPECT_FALSE(output.goal_reached);
        EXPECT_EQ(output.reason,"stair_goal_heading_wait");
        EXPECT_DOUBLE_EQ(output.cmd_vel.vx,0);
        EXPECT_DOUBLE_EQ(output.cmd_vel.vy,0);
        EXPECT_DOUBLE_EQ(output.cmd_vel.wz,0);
      }
    }
    if (!requestedYaw) EXPECT_TRUE(output.goal_reached) << output.reason;
    EXPECT_EQ(output.stairs.phase,StairPhase::Traversing);
  }
}

}  // namespace
