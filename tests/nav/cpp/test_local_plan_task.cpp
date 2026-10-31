#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "collision_bitmap.hpp"
#include "collision/body.hpp"
#include "planning/local/scan/task.hpp"
#include "tracking/follower.hpp"
#include "trajectory/spline.hpp"

namespace {

nav_kernel::LocalPlannerParams scanParams() {
  nav_kernel::LocalPlannerParams params;
  params.backend = nav_kernel::LocalPlannerBackend::Scan;
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
  RequestFixture()
      : route{{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}},
        bitmap({-5.0, -5.0, -1.0}, {5.0, 5.0, 2.0}, 0.1) {
    request.robot.pose = {route.front(), 0.0};
    request.objective = nav_kernel::RouteTarget{
        {route.data(), static_cast<int>(route.size()), 1}};
    request.identity = {1, 1, 0};
    request.clock.timestampS = 1.0;
    request.environment.collision = bitmap.view(1.0, 1);
  }

  void setGeneration(std::uint64_t generation) {
    request.identity.obstacleGeneration = generation;
    request.environment.collision =
        bitmap.view(request.clock.timestampS, generation);
  }

  void setIntent(double directionBodyDeg, std::uint64_t generation = 1) {
    request.objective = nav_kernel::MotionIntentTarget{
        {directionBodyDeg, 1.0, 2.0, 90.0},
        {route.data(), static_cast<int>(route.size()), generation}};
  }

  std::vector<nav_kernel::Vec3> route;
  lingtu::nav::tests::CollisionBitmap bitmap;
  nav_kernel::LocalPlanRequest request;
};

nav_kernel::LocalPlan waitForPlan(nav_kernel::local::scan::Task &task,
                                  RequestFixture &fixture) {
  nav_kernel::LocalPlan plan;
  const double initialTime=fixture.request.clock.timestampS;
  for (int tick = 0; tick < 400 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS = initialTime + 0.01 * tick;
    fixture.setGeneration(1);
    plan = task.update(fixture.request).plan;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return plan;
}

}  // namespace

TEST(LocalPlanTask, RejectsInvalidRouteWithoutStartingWork) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  nav_kernel::LocalPlanRequest request;
  EXPECT_EQ(task.update(request).plan.status(),
            nav_kernel::LocalPlanStatus::InvalidInput);
}

TEST(LocalPlanTask, SupportHeightMetadataDoesNotRequireFullFloorObservation) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  auto body = std::make_shared<lingtu::nav::collision::Body>();
  body->margin_m = 0;
  lingtu::nav::collision::Part box;
  box.half_size = {.15, .15, .1};
  body->parts.push_back(box);
  fixture.request.body = body;
  fixture.request.supportHeightM = .5;
  fixture.request.supportHeightToleranceM = .01;
  nav_kernel::LocalPlan plan;
  for (int i = 0; i < 400 && !plan.ready(); ++i) {
    fixture.request.clock.timestampS = 1 + .01 * i;
    fixture.setGeneration(1);
    const auto raw = fixture.bitmap.view();
    fixture.request.environment.collision.measuredOccupiedStorage =
        std::make_shared<const std::vector<std::uint8_t>>(
            raw.inflatedBits, raw.inflatedBits + raw.inflatedBytes);
    fixture.request.environment.collision.knownFreeStorage =
        std::make_shared<const std::vector<std::uint8_t>>(raw.inflatedBytes, 0U);
    plan = task.update(fixture.request).plan;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_TRUE(plan.ready());
}

TEST(LocalPlanTask, SupportHeightValueChangesKeepTheExecutingSpline) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
  auto body = std::make_shared<lingtu::nav::collision::Body>();
  body->margin_m = 0;
  lingtu::nav::collision::Part box;
  box.half_size = {.15, .15, .1};
  body->parts.push_back(box);
  fixture.request.body = body;
  fixture.request.supportHeightM = .5;
  fixture.request.supportHeightToleranceM = .01;
  const auto bytes = fixture.request.environment.collision.inflatedBytes;
  fixture.request.environment.collision.measuredOccupiedStorage =
      std::make_shared<const std::vector<std::uint8_t>>(bytes, 0U);
  fixture.request.environment.collision.knownFreeStorage =
      std::make_shared<const std::vector<std::uint8_t>>(bytes, 0U);
  auto update = task.update(fixture.request);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!update.plan.ready() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    update = task.update(fixture.request);
  }
  ASSERT_TRUE(update.plan.ready()) << update.debug.searchReason;
  const auto id = std::get<nav_kernel::SplineTarget>(update.plan.target()).trajectoryId;

  fixture.request.supportHeightM = .6;
  fixture.request.supportHeightToleranceM = .02;
  for (int tick = 0; tick < 30; ++tick) {
    update = task.update(fixture.request);
    ASSERT_TRUE(update.plan.ready()) << "Height-only metadata discarded a valid spline";
    EXPECT_EQ(std::get<nav_kernel::SplineTarget>(update.plan.target()).trajectoryId, id);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  fixture.request.supportHeightM.reset();
  fixture.request.supportHeightToleranceM.reset();
  EXPECT_FALSE(task.update(fixture.request).plan.ready())
      << "Changing boundary initialization must still discard the old spline";
}

TEST(LocalPlanTask, BlockedTargetInvalidatesCachedSplineAndRecoversOnMapUpdate) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  nav_kernel::LocalPlan first = waitForPlan(task, fixture);
  ASSERT_TRUE(first.ready());
  const auto oldId = std::get<nav_kernel::SplineTarget>(first.target()).trajectoryId;
  for (int x = -1; x <= 21; ++x)
    fixture.bitmap.occupyInflated({x * 0.1F, 0.0F, 0.5F}, 0.15, 0.1, 0.1);
  fixture.request.objective = nav_kernel::RouteTarget{{fixture.route.data(), 2, 2}};
  nav_kernel::LocalPlan plan;
  for (int tick = 0; tick < 200; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    fixture.setGeneration(2);
    plan = task.update(fixture.request).plan;
    if (plan.status() == nav_kernel::LocalPlanStatus::Blocked) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_EQ(plan.status(), nav_kernel::LocalPlanStatus::Blocked);
  for (int tick = 0; tick < 5; ++tick) EXPECT_FALSE(task.update(fixture.request).plan.ready());
  fixture.bitmap.clear();
  for (int tick = 0; tick < 200 && !plan.ready(); ++tick) {
    fixture.request.clock.timestampS += 0.01;
    fixture.setGeneration(3);
    plan = task.update(fixture.request).plan;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(plan.ready());
  EXPECT_GT(std::get<nav_kernel::SplineTarget>(plan.target()).trajectoryId, oldId);
}

TEST(LocalPlanTask, RepeatedResetPreservesPendingCoreReset) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  task.reset();
  task.reset();
  RequestFixture fixture;
  const nav_kernel::LocalPlan first = waitForPlan(task, fixture);
  ASSERT_TRUE(first.ready());
  ASSERT_FALSE(first.previewPath().empty());
  EXPECT_GT(first.previewPath().back().x, std::abs(first.previewPath().back().y));

  task.reset();
  task.reset();
  // A reset owns the identity boundary, so the next route may reuse generation 1.
  fixture.route[1] = {0.0, 2.0, 0.5};
  const nav_kernel::LocalPlan restarted = waitForPlan(task, fixture);
  ASSERT_TRUE(restarted.ready());
  ASSERT_FALSE(restarted.previewPath().empty());
  EXPECT_GT(restarted.previewPath().back().y, std::abs(restarted.previewPath().back().x));
}

TEST(LocalPlanTask, RejectsNonFiniteReferenceSnapshot) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  fixture.route.back().x = std::numeric_limits<double>::quiet_NaN();
  auto result = task.update(fixture.request);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (result.plan.status() == nav_kernel::LocalPlanStatus::Pending &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    result = task.update(fixture.request);
  }
  EXPECT_EQ(result.plan.status(), nav_kernel::LocalPlanStatus::InvalidInput);
  EXPECT_FALSE(result.plan.ready());
}

TEST(LocalPlanTask, RunsFsmOnOwnedTimer) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;

  EXPECT_EQ(task.update(fixture.request).plan.status(),
            nav_kernel::LocalPlanStatus::Pending);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  EXPECT_TRUE(task.update(fixture.request).plan.ready());
}

TEST(LocalPlanTask, PendingTrajectoryExposesWorkerReason) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  for (double y = -4.95; y < 5.0; y += 0.1)
    for (double z = -0.95; z < 2.0; z += 0.1)
      fixture.bitmap.occupy({0.55, y, z});
  fixture.setGeneration(2);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  auto result = task.update(fixture.request);
  while (result.debug.searchReason.empty() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    result = task.update(fixture.request);
  }
  EXPECT_EQ(result.plan.status(), nav_kernel::LocalPlanStatus::Pending);
  EXPECT_FALSE(result.debug.searchReason.empty());
}

TEST(LocalPlanTask, SpeedOnlyUpdateReachesWorkerWithoutReplacingGuide) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  const nav_kernel::LocalPlan first = waitForPlan(task, fixture);
  ASSERT_TRUE(first.ready());
  const auto firstId = std::get<nav_kernel::SplineTarget>(first.target()).trajectoryId;

  fixture.request.maxLinearSpeedMps = 0.2;
  nav_kernel::LocalPlan plan;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  do {
    fixture.request.clock.timestampS += 0.005;
    plan = task.update(fixture.request).plan;
    if (plan.ready() &&
        std::get<nav_kernel::SplineTarget>(plan.target()).maxLinearSpeedMps == 0.2) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  } while (std::chrono::steady_clock::now() < deadline);

  ASSERT_TRUE(plan.ready());
  const auto &slower = std::get<nav_kernel::SplineTarget>(plan.target());
  EXPECT_DOUBLE_EQ(slower.maxLinearSpeedMps, 0.2);
  EXPECT_GT(slower.trajectoryId, firstId);
}

TEST(LocalPlanTask, MovingSpeedReductionReanchorsItsBridgeWhenTheReferenceIsFrozen) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
  const auto initial = waitForPlan(task, fixture);
  ASSERT_TRUE(initial.ready());
  const auto first = std::get<nav_kernel::SplineTarget>(initial.target());
  const nav_kernel::SplineView view(first);
  double movingTime = 0;
  double speed = 0;
  for (int sample = 1; sample < 100; ++sample) {
    const double time = view.duration() * sample / 100.0;
    const auto velocity = view.velocity(time);
    if (std::hypot(velocity.x, velocity.y) > speed) {
      movingTime = time;
      speed = std::hypot(velocity.x, velocity.y);
    }
  }
  ASSERT_GT(speed, .3);
  fixture.request.robot.pose.position = view.position(movingTime);
  fixture.request.robot.kinematics.valid = true;
  fixture.request.robot.kinematics.linearVelocity = view.velocity(movingTime);
  fixture.request.clock.timestampS = first.startTimeS + movingTime;
  fixture.request.clock.executionFrozen = true;
  fixture.request.maxLinearSpeedMps = .3;
  const auto routeGeneration = fixture.request.referenceRoute()->generation;

  auto update = task.update(fixture.request);
  ASSERT_TRUE(update.plan.ready());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    update = task.update(fixture.request);
    if (update.plan.ready() &&
        std::get<nav_kernel::SplineTarget>(update.plan.target()).trajectoryId > first.trajectoryId)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(update.plan.ready()) << update.debug.searchReason;
  const auto bridge = std::get<nav_kernel::SplineTarget>(update.plan.target());
  ASSERT_GT(bridge.trajectoryId, first.trajectoryId);
  EXPECT_DOUBLE_EQ(bridge.maxLinearSpeedMps, .3);
  EXPECT_GT(bridge.speedTransitionEndS, 0);
  EXPECT_DOUBLE_EQ(bridge.startTimeS, fixture.request.clock.timestampS);
  const nav_kernel::SplineView bridgeView(bridge);
  const auto position = bridgeView.position(0);
  const auto velocity = bridgeView.velocity(0);
  EXPECT_NEAR(position.x, fixture.request.robot.pose.position.x, 1e-6);
  EXPECT_NEAR(position.y, fixture.request.robot.pose.position.y, 1e-6);
  EXPECT_NEAR(velocity.x, fixture.request.robot.kinematics.linearVelocity.x, 1e-6);
  EXPECT_NEAR(velocity.y, fixture.request.robot.kinematics.linearVelocity.y, 1e-6);
  EXPECT_EQ(fixture.request.referenceRoute()->generation, routeGeneration);
}

TEST(LocalPlanTask, MovingReductionTracksTheBridgeAndItsNormalReplacement) {
  for(const auto speeds : {std::pair<double,double>{.5,.3}, {1.0,.6}}) {
  SCOPED_TRACE(speeds.first);
  auto params=scanParams();
  params.maxSpeed=speeds.first;
  params.autonomySpeed=speeds.first;
  params.scan.maxVelocity=speeds.first;
  nav_kernel::local::scan::Task task(params);
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  fixture.route.back().x=4.5;
  fixture.request.clock.mode=nav_kernel::PlanClockMode::External;
  fixture.request.maxLinearSpeedMps=speeds.first;
  fixture.request.robot.kinematics.valid=true;
  fixture.request.robot.kinematics.linearVelocity={speeds.first,0,0};
  const auto initial=waitForPlan(task,fixture);
  ASSERT_TRUE(initial.ready());
  nav_kernel::Follower follower;
  nav_kernel::FollowerState state;
  state.params.maxSpeed=speeds.first;
  state.params.maxAccel=.5;
  state.params.spline.maxVx=speeds.first;
  state.params.spline.maxVy=.35;
  state.measuredBodyTwistValid=true;
  state.measuredBodyTwist={speeds.first,0,0};
  state.vehicleRelative=fixture.request.robot.pose.position;
  state.currentTime=fixture.request.clock.timestampS;
  auto control=follower.follow(initial,state);
  nav_kernel::LocalPlan lastPlan=initial;
  bool reduced=false,sawBridge=false,sawReplacement=false;
  std::int64_t bridgeId=0,previousId=control.tracking.trajectoryId;
  double positionAtReduction=0,minSpeed=speeds.first,maxJump=0,maxSeamError=0;
  double reducedAt=0,cruiseSpeed=0;
  for(int tick=0;tick<400;++tick) {
    fixture.request.clock.timestampS+=.02;
    fixture.request.clock.executionFrozen=control.executionFrozen;
    fixture.request.robot.kinematics.linearVelocity={control.cmd.vx,control.cmd.vy,0};
    if(!reduced) cruiseSpeed=std::max(cruiseSpeed,control.cmd.vx);
    if(!reduced && control.cmd.vx>.99*speeds.first) {
      reduced=true;
      fixture.request.maxLinearSpeedMps=speeds.second;
      positionAtReduction=state.vehicleRelative.x;
      reducedAt=fixture.request.clock.timestampS;
    }
    fixture.setGeneration(1);
    const auto update=task.update(fixture.request);
    ASSERT_TRUE(update.plan.ready())<<update.debug.searchReason;
    lastPlan=update.plan;
    const auto &target=std::get<nav_kernel::SplineTarget>(update.plan.target());
    if(reduced && target.speedTransitionEndS>0) {
      sawBridge=true;
      bridgeId=target.trajectoryId;
    } else if(sawBridge && target.trajectoryId>bridgeId) {
      sawReplacement=true;
    }
    if(reduced && target.trajectoryId!=previousId) {
      const nav_kernel::SplineView view(target);
      const auto reference=view.position(std::max(0.0,
          fixture.request.clock.timestampS-target.startTimeS));
      maxSeamError=std::max(maxSeamError,nav_kernel::distance2D(reference,state.vehicleRelative));
    }
    state.currentTime=fixture.request.clock.timestampS;
    state.requestedSpeed=reduced ? speeds.second/speeds.first : 1;
    state.measuredBodyTwist=control.cmd;
    const auto next=follower.follow(update.plan,state);
    if(reduced) {
      minSpeed=std::min(minSpeed,std::hypot(next.cmd.vx,next.cmd.vy));
      maxJump=std::max(maxJump,std::hypot(next.cmd.vx-control.cmd.vx,next.cmd.vy-control.cmd.vy));
    }
    state.vehicleRelative.x+=next.cmd.vx*.02;
    state.vehicleRelative.y+=next.cmd.vy*.02;
    fixture.request.robot.pose.position=state.vehicleRelative;
    previousId=target.trajectoryId;
    control=next;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if(reduced && fixture.request.clock.timestampS-reducedAt>=4) break;
  }
  EXPECT_TRUE(reduced);
  EXPECT_GE(cruiseSpeed,.99*speeds.first);
  EXPECT_TRUE(sawBridge);
  EXPECT_TRUE(sawReplacement);
  EXPECT_GT(minSpeed,.1);
  EXPECT_LE(maxJump,.5*.02+1e-8);
  EXPECT_LT(maxSeamError,.08);
  EXPECT_LE(std::hypot(control.cmd.vx,control.cmd.vy),speeds.second+1e-8);
  EXPECT_GT(state.vehicleRelative.x-positionAtReduction,.4);
  EXPECT_EQ(fixture.request.referenceRoute()->generation,1U);
  const auto prefix=speeds.first<1 ? "half_speed_" : "one_mps_";
  RecordProperty(std::string(prefix)+"speed_before_reduction_mps",cruiseSpeed);
  RecordProperty(std::string(prefix)+"minimum_speed_after_reduction_mps",minSpeed);
  RecordProperty(std::string(prefix)+"maximum_command_delta_mps",maxJump);
  RecordProperty(std::string(prefix)+"maximum_handoff_position_error_m",maxSeamError);
  RecordProperty(std::string(prefix)+"distance_after_reduction_m",state.vehicleRelative.x-positionAtReduction);
  state.currentTime+=.02;
  state.safetyStop=2;
  const auto emergency=follower.follow(lastPlan,state);
  EXPECT_DOUBLE_EQ(emergency.cmd.vx,0);
  EXPECT_DOUBLE_EQ(emergency.cmd.vy,0);
  EXPECT_DOUBLE_EQ(emergency.cmd.wz,0);
  task.reset();
  follower.reset();
  state.safetyStop=0;
  state.currentTime+=.02;
  const auto cancelled=follower.follow(
      nav_kernel::LocalPlan::stopped(nav_kernel::LocalPlanStatus::Cancelled),state);
  EXPECT_DOUBLE_EQ(cancelled.cmd.vx,0);
  EXPECT_DOUBLE_EQ(cancelled.cmd.vy,0);
  }
}

TEST(LocalPlanTask, RepeatedLowerCapsKeepTheLatestInputWithoutReplacingTheRoute) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
  const auto initial = waitForPlan(task, fixture);
  ASSERT_TRUE(initial.ready());
  const auto firstId = std::get<nav_kernel::SplineTarget>(initial.target()).trajectoryId;
  const auto routeGeneration = fixture.request.referenceRoute()->generation;
  for (int step = 0; step < 20; ++step) {
    fixture.request.maxLinearSpeedMps = .49 - .01 * step;
    const auto update = task.update(fixture.request);
    EXPECT_TRUE(update.plan.ready());
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  nav_kernel::local::scan::Update update;
  do {
    update = task.update(fixture.request);
    if (update.plan.ready() &&
        std::get<nav_kernel::SplineTarget>(update.plan.target()).maxLinearSpeedMps ==
            fixture.request.maxLinearSpeedMps)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (std::chrono::steady_clock::now() < deadline);
  ASSERT_TRUE(update.plan.ready()) << update.debug.searchReason;
  const auto &slower = std::get<nav_kernel::SplineTarget>(update.plan.target());
  EXPECT_GT(slower.trajectoryId, firstId);
  EXPECT_DOUBLE_EQ(slower.maxLinearSpeedMps, fixture.request.maxLinearSpeedMps);
  EXPECT_EQ(fixture.request.referenceRoute()->generation, routeGeneration);
}

TEST(LocalPlanTask, PausesOwnedClockForTimerGeneratedReferences) {
  nav_kernel::LocalPlannerParams runningParams = scanParams();
  runningParams.scan.noReplanDistance = 0.02;
  runningParams.scan.replanDistance = 0.02;

  nav_kernel::local::scan::Task pausedTask(runningParams);
  nav_kernel::local::scan::Task runningTask(runningParams);
  ASSERT_TRUE(pausedTask.configure());
  ASSERT_TRUE(runningTask.configure());
  RequestFixture pausedFixture;
  RequestFixture runningFixture;

  const nav_kernel::LocalPlan pausedInitial =
      waitForPlan(pausedTask, pausedFixture);
  const auto pausedOrigin = std::chrono::steady_clock::now();
  const double pausedTimestamp = pausedFixture.request.clock.timestampS;
  pausedTask.pause();
  const nav_kernel::LocalPlan runningInitial =
      waitForPlan(runningTask, runningFixture);
  const auto runningOrigin = std::chrono::steady_clock::now();
  const double runningTimestamp = runningFixture.request.clock.timestampS;
  ASSERT_TRUE(pausedInitial.ready());
  ASSERT_TRUE(runningInitial.ready());
  const auto pausedInitialId =
      std::get<nav_kernel::SplineTarget>(pausedInitial.target()).trajectoryId;
  const auto updateClocks = [&] {
    const auto now = std::chrono::steady_clock::now();
    pausedFixture.request.clock.timestampS = pausedTimestamp +
        std::chrono::duration<double>(now - pausedOrigin).count();
    runningFixture.request.clock.timestampS = runningTimestamp +
        std::chrono::duration<double>(now - runningOrigin).count();
  };
  pausedFixture.request.clock.executionFrozen = true;
  updateClocks();
  const auto pausedInitialAttempt =
      pausedTask.update(pausedFixture.request).debug.scanAttempt.attemptId;
  const auto runningInitialAttempt =
      runningTask.update(runningFixture.request).debug.scanAttempt.attemptId;

  // Neither task receives a source update while timer callbacks advance.
  std::this_thread::sleep_for(std::chrono::seconds(2));

  updateClocks();
  const auto pausedAfter = pausedTask.update(pausedFixture.request);
  const auto runningAfter = runningTask.update(runningFixture.request);
  ASSERT_TRUE(pausedAfter.plan.ready());
  ASSERT_TRUE(runningAfter.plan.ready());
  EXPECT_EQ(std::get<nav_kernel::SplineTarget>(pausedAfter.plan.target()).trajectoryId,
            pausedInitialId);
  // Attempts prove that the autonomous timer advanced. Reception still needs
  // a control tick inside the current offer's handoff window.
  EXPECT_EQ(pausedAfter.debug.scanAttempt.attemptId, pausedInitialAttempt);
  EXPECT_GT(runningAfter.debug.scanAttempt.attemptId, runningInitialAttempt);
}

TEST(LocalPlanTask, KeepsOfficialWorldFrameSplineDuringMapRefresh) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  const nav_kernel::LocalPlan ready = waitForPlan(task, fixture);
  ASSERT_TRUE(ready.ready());
  const auto &first = std::get<nav_kernel::SplineTarget>(ready.target());
  ASSERT_FALSE(first.controls.empty());

  fixture.request.robot.pose.position.x = 0.2;
  fixture.request.clock.timestampS += 0.01;
  fixture.setGeneration(2);
  const nav_kernel::LocalPlan retained = task.update(fixture.request).plan;

  ASSERT_TRUE(retained.ready());
  const auto &second = std::get<nav_kernel::SplineTarget>(retained.target());
  EXPECT_EQ(second.trajectoryId, first.trajectoryId);
  EXPECT_DOUBLE_EQ(second.controls.front().x, first.controls.front().x);
  EXPECT_DOUBLE_EQ(second.controls.front().y, first.controls.front().y);
}

TEST(LocalPlanTask, FirstSplineEpochIsItsActualReceptionTime) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  const nav_kernel::LocalPlan ready = waitForPlan(task, fixture);
  ASSERT_TRUE(ready.ready());
  const auto &spline = std::get<nav_kernel::SplineTarget>(ready.target());

  EXPECT_DOUBLE_EQ(spline.startTimeS, fixture.request.clock.timestampS);
}

TEST(LocalPlanTask, LateHandoffReceptionKeepsTheExecutingSpline) {
  for (const bool queuedBeforeDelay : {false, true}) {
    SCOPED_TRACE(queuedBeforeDelay);
    auto params = scanParams();
    params.scan.maxVelocity = params.maxSpeed;
    params.scan.noReplanDistance = 0.02;
    params.scan.replanDistance = 0.02;
    nav_kernel::local::scan::Task task(params);
    ASSERT_TRUE(task.configure());
    RequestFixture fixture;
    fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
    const auto first = waitForPlan(task, fixture);
    ASSERT_TRUE(first.ready());
    const auto spline = std::get<nav_kernel::SplineTarget>(first.target());
    const nav_kernel::SplineView view(spline);
    const double elapsed = std::min(1.0, 0.4 * view.duration());
    fixture.request.clock.timestampS = spline.startTimeS + elapsed;
    fixture.request.robot.pose.position = view.position(elapsed);
    fixture.request.robot.kinematics.valid = true;
    fixture.request.robot.kinematics.linearVelocity = view.velocity(elapsed);
    // A new reference requires an offer; an equivalent periodic replan may keep its tail.
    fixture.request.objective = nav_kernel::RouteTarget{
        {fixture.route.data(), static_cast<int>(fixture.route.size()), 2}};
    task.update(fixture.request);
    // Hold physical time while the worker prepares its future offer. This
    // covers both a late first poll and a queued offer whose control tick stalls.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    if (queuedBeforeDelay) {
      const auto waiting = task.update(fixture.request).plan;
      ASSERT_TRUE(waiting.ready());
      EXPECT_EQ(std::get<nav_kernel::SplineTarget>(waiting.target()).trajectoryId,
                spline.trajectoryId);
    }
    fixture.request.clock.timestampS += 0.2;
    const auto lateUpdate = task.update(fixture.request);
    EXPECT_EQ(lateUpdate.debug.searchReason, "scan_handoff_pending");
    const auto &late = lateUpdate.plan;
    ASSERT_TRUE(late.ready());
    EXPECT_EQ(std::get<nav_kernel::SplineTarget>(late.target()).trajectoryId,
              spline.trajectoryId);
    // Let the worker observe withdrawal and retry. It must retain the same
    // executing ID until a fresh offer reaches a physical handoff tick.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const auto retained = task.update(fixture.request).plan;
    ASSERT_TRUE(retained.ready());
    EXPECT_EQ(std::get<nav_kernel::SplineTarget>(retained.target()).trajectoryId,
              spline.trajectoryId);
  }
}

TEST(LocalPlanTask, PausedOfferWithdrawalPublishesItsMeasuredReplacement) {
  auto params = scanParams();
  params.scan.maxVelocity = params.maxSpeed;
  params.scan.noReplanDistance = 0.02;
  params.scan.replanDistance = 0.02;
  nav_kernel::local::scan::Task task(params);
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
  const auto first = waitForPlan(task, fixture);
  ASSERT_TRUE(first.ready());
  const auto spline = std::get<nav_kernel::SplineTarget>(first.target());
  const nav_kernel::SplineView view(spline);
  const double elapsed = std::min(1.0, 0.4 * view.duration());
  fixture.request.clock.timestampS = spline.startTimeS + elapsed;
  fixture.request.robot.pose.position = view.position(elapsed);
  fixture.request.robot.kinematics.valid = true;
  fixture.request.robot.kinematics.linearVelocity = view.velocity(elapsed);
  fixture.request.objective = nav_kernel::RouteTarget{
      {fixture.route.data(), static_cast<int>(fixture.route.size()), 2}};
  task.update(fixture.request);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const auto queued = task.update(fixture.request);
  ASSERT_EQ(queued.debug.searchReason, "scan_handoff_pending");
  ASSERT_EQ(std::get<nav_kernel::SplineTarget>(queued.plan.target()).trajectoryId,
            spline.trajectoryId);
  // pause() reaches the worker without a control poll withdrawing the token.
  // The measured replacement may reuse the offered ID, but it is a distinct
  // publication and must reach Task instead of being deduplicated away.
  task.pause();
  fixture.request.clock.executionFrozen = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const auto measured = task.update(fixture.request).plan;
  ASSERT_TRUE(measured.ready());
  EXPECT_GT(std::get<nav_kernel::SplineTarget>(measured.target()).trajectoryId,
            spline.trajectoryId);
}

TEST(LocalPlanTask, ExternalClockDoesNotAdvanceDuringWallTimePause) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
  fixture.request.clock.timestampS = 0.0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  auto result = task.update(fixture.request);
  while (!result.plan.ready() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    result = task.update(fixture.request);
  }
  ASSERT_TRUE(result.plan.ready());
  const auto &first = std::get<nav_kernel::SplineTarget>(result.plan.target());
  EXPECT_DOUBLE_EQ(first.startTimeS, 0.0);
  const auto id = first.trajectoryId;

  // Timers still run while the physical world is held at the same instant.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  result = task.update(fixture.request);
  ASSERT_TRUE(result.plan.ready());
  EXPECT_DOUBLE_EQ(result.debug.timestampS, 0.0);
  EXPECT_EQ(std::get<nav_kernel::SplineTarget>(result.plan.target()).trajectoryId, id);

  fixture.request.clock.timestampS = 0.05;
  task.update(fixture.request);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  result = task.update(fixture.request);
  EXPECT_DOUBLE_EQ(result.debug.timestampS, 0.05);
}

TEST(LocalPlanTask, KeepsPublishedSplineUntilNewIntentIsPlanned) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  fixture.setIntent(0.0);
  ASSERT_TRUE(waitForPlan(task, fixture).ready());

  fixture.setIntent(90.0, 2);
  fixture.request.clock.timestampS += 0.01;
  EXPECT_TRUE(task.update(fixture.request).plan.ready());
}

TEST(LocalPlanTask, RouteGenerationOwnsReferenceReplacement) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  const nav_kernel::LocalPlan initial = waitForPlan(task, fixture);
  ASSERT_TRUE(initial.ready());
  ASSERT_FALSE(initial.previewPath().empty());
  EXPECT_GT(initial.previewPath().back().x,
            std::abs(initial.previewPath().back().y));

  fixture.route[1] = {0.0, 2.0, 0.5};
  nav_kernel::LocalPlan retained = initial;
  for (int tick = 0; tick < 60; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    retained = task.update(fixture.request).plan;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_TRUE(retained.ready());
  ASSERT_FALSE(retained.previewPath().empty());
  EXPECT_GT(retained.previewPath().back().x,
            std::abs(retained.previewPath().back().y));

  fixture.request.objective = nav_kernel::RouteTarget{
      {fixture.route.data(), static_cast<int>(fixture.route.size()), 2}};
  nav_kernel::LocalPlan replaced = retained;
  for (int tick = 0; tick < 200; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    replaced = task.update(fixture.request).plan;
    if (replaced.ready() && !replaced.previewPath().empty() &&
        replaced.previewPath().back().y >
            std::abs(replaced.previewPath().back().x)) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(replaced.ready());
  ASSERT_FALSE(replaced.previewPath().empty());
  EXPECT_GT(replaced.previewPath().back().y,
            std::abs(replaced.previewPath().back().x));
}

TEST(LocalPlanTask, BodyHeadingChangesDiscardCachedCollisionFootprint) {
  auto params = scanParams();
  params.scan.cylinderOffset = .25;
  nav_kernel::local::scan::Task task(params);
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  auto plan = waitForPlan(task, fixture);
  ASSERT_TRUE(plan.ready());
  auto previousId = std::get<nav_kernel::SplineTarget>(plan.target()).trajectoryId;
  for (const auto heading : {std::optional<double>{0.0}, std::optional<double>{std::acos(-1.0) / 2},
                             std::optional<double>{}}) {
    fixture.request.bodyHeading = heading;
    EXPECT_FALSE(task.update(fixture.request).plan.ready());
    plan = waitForPlan(task, fixture);
    ASSERT_TRUE(plan.ready());
    const auto id = std::get<nav_kernel::SplineTarget>(plan.target()).trajectoryId;
    EXPECT_GT(id, previousId);
    previousId = id;
  }
}

TEST(LocalPlanTask, BodyModelReplacementDiscardsCachedSplineWithoutNewRouteOrFrame) {
  namespace collision = lingtu::nav::collision;
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
  fixture.request.bodyHeading = 0.0;
  lingtu::nav::tests::CollisionBitmap measured({-5,-5,-1},{5,5,2},.1);
  for (double x = -4.95; x < 5.; x += .1)
    for (double z = -.95; z < 2.; z += .1) measured.occupy({x,.35,z});
  const auto raw = measured.view();
  fixture.request.environment.collision.measuredOccupiedStorage =
      std::make_shared<const std::vector<std::uint8_t>>(
          raw.inflatedBits,raw.inflatedBits + raw.inflatedBytes);
  fixture.request.environment.collision.knownFreeStorage =
      std::make_shared<const std::vector<std::uint8_t>>(raw.inflatedBytes,255U);
  auto small = std::make_shared<collision::Body>();
  small->margin_m = 0;
  collision::Part part;
  part.link = "base";
  part.half_size = {.15,.08,.08};
  small->parts.push_back(part);
  auto large = std::make_shared<collision::Body>(*small);
  large->parts.front().half_size.y = .5;
  fixture.request.body = small;
  const auto waitReady = [&]() {
    auto update = task.update(fixture.request);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!update.plan.ready() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      update = task.update(fixture.request);
    }
    return update;
  };
  const auto initial = waitReady();
  ASSERT_TRUE(initial.plan.ready()) << initial.debug.searchReason;
  const auto initialId = std::get<nav_kernel::SplineTarget>(initial.plan.target()).trajectoryId;

  // Body identity is the only changed input; both route and frame stay fixed.
  fixture.request.body = large;
  auto update = task.update(fixture.request);
  ASSERT_FALSE(update.plan.ready());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  bool workerResponded = false;
  do {
    ASSERT_FALSE(update.plan.ready()) << "Old spline escaped after body replacement";
    workerResponded = workerResponded || !update.debug.searchReason.empty();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    update = task.update(fixture.request);
  } while (std::chrono::steady_clock::now() < deadline);
  EXPECT_FALSE(update.plan.ready());
  EXPECT_TRUE(workerResponded);

  fixture.request.body = small;
  EXPECT_FALSE(task.update(fixture.request).plan.ready());
  const auto recovered = waitReady();
  ASSERT_TRUE(recovered.plan.ready()) << recovered.debug.searchReason;
  EXPECT_GT(std::get<nav_kernel::SplineTarget>(recovered.plan.target()).trajectoryId,initialId);
}

TEST(LocalPlanTask, ObservedBodySnapshotsUpdateCollisionWithoutReplacingGuideIdentity) {
  namespace collision = lingtu::nav::collision;
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  fixture.request.clock.mode = nav_kernel::PlanClockMode::External;
  fixture.request.bodyHeading = 0.0;
  lingtu::nav::tests::CollisionBitmap measured({-5,-5,-1},{5,5,2},.1);
  for (double x = -4.95; x < 5.; x += .1)
    for (double z = -.95; z < 2.; z += .1) measured.occupy({x,.35,z});
  const auto raw = measured.view();
  fixture.request.environment.collision.measuredOccupiedStorage =
      std::make_shared<const std::vector<std::uint8_t>>(
          raw.inflatedBits,raw.inflatedBits + raw.inflatedBytes);
  fixture.request.environment.collision.knownFreeStorage =
      std::make_shared<const std::vector<std::uint8_t>>(raw.inflatedBytes,255U);
  auto nominal = std::make_shared<collision::Body>();
  nominal->margin_m = 0;
  collision::Part part;
  part.link = "base";
  part.half_size = {.15,.08,.08};
  nominal->parts.push_back(part);
  fixture.request.body = nominal;
  const auto stableModel = fixture.request.body;
  const auto frameEpoch = fixture.request.identity.frameEpoch;
  const auto mapGeneration = fixture.request.environment.collision.generation;
  const auto referenceGeneration = fixture.request.referenceRoute()->generation;

  const auto updateSnapshot = [&](const collision::Body &shape) {
    fixture.request.observedBody = std::make_shared<const collision::Body>(shape);
    return task.update(fixture.request);
  };
  const auto waitReady = [&]() {
    auto update = updateSnapshot(*nominal);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!update.plan.ready() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      update = updateSnapshot(*nominal);
    }
    return update;
  };
  const auto initial = waitReady();
  ASSERT_TRUE(initial.plan.ready()) << initial.debug.searchReason;
  const auto initialId = std::get<nav_kernel::SplineTarget>(initial.plan.target()).trajectoryId;
  for (int frame = 0; frame < 25; ++frame) {
    const auto update = updateSnapshot(*nominal);
    ASSERT_TRUE(update.plan.ready()) << "Equivalent FK snapshot reset the active spline";
    EXPECT_EQ(std::get<nav_kernel::SplineTarget>(update.plan.target()).trajectoryId,initialId);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  // Only runtime articulation changes. Model identity, frame, map and guide
  // stay fixed, and every frame owns a fresh immutable snapshot.
  collision::Body widened = *nominal;
  widened.parts.front().half_size.y = .5;
  auto blocked = updateSnapshot(widened);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (blocked.plan.ready() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    blocked = updateSnapshot(widened);
  }
  ASSERT_FALSE(blocked.plan.ready())
      << "Worker retained Ready after the observed body reached the existing wall";
  for (int frame = 0; frame < 20; ++frame) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    EXPECT_FALSE(updateSnapshot(widened).plan.ready());
  }

  const auto recovered = waitReady();
  ASSERT_TRUE(recovered.plan.ready()) << recovered.debug.searchReason;
  EXPECT_EQ(fixture.request.body,stableModel);
  EXPECT_EQ(fixture.request.identity.frameEpoch,frameEpoch);
  EXPECT_EQ(fixture.request.environment.collision.generation,mapGeneration);
  EXPECT_EQ(fixture.request.referenceRoute()->generation,referenceGeneration);
}

TEST(LocalPlanTask, OwnsReferenceAcrossAsyncTicks) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  EXPECT_EQ(task.update(fixture.request).plan.status(),
            nav_kernel::LocalPlanStatus::Pending);

  // Destroy the caller's buffer while the worker still owns this generation.
  std::vector<nav_kernel::Vec3>{{0.0, 0.0, 0.5}, {0.0, 2.0, 0.5}}
      .swap(fixture.route);
  fixture.request.objective = nav_kernel::RouteTarget{
      {fixture.route.data(), static_cast<int>(fixture.route.size()), 1}};

  const nav_kernel::LocalPlan plan = waitForPlan(task, fixture);
  ASSERT_TRUE(plan.ready());
  ASSERT_FALSE(plan.previewPath().empty());
  EXPECT_GT(plan.previewPath().back().x,
            std::abs(plan.previewPath().back().y));
}

TEST(LocalPlanTask, UsesCompleteReferenceInsteadOfShortGuide) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  const std::vector<nav_kernel::Vec3> reference{
      {0.0, 0.0, 0.5}, {0.0, 1.0, 0.5}, {0.0, 2.0, 0.5}};
  fixture.request.reference = {
      reference.data(), static_cast<int>(reference.size()), 1};

  const nav_kernel::LocalPlan plan = waitForPlan(task, fixture);

  ASSERT_TRUE(plan.ready());
  ASSERT_FALSE(plan.previewPath().empty());
  EXPECT_GT(plan.previewPath().back().y,
            std::abs(plan.previewPath().back().x));
}

TEST(LocalPlanTask, DropsCompletionForSupersededReference) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  EXPECT_EQ(task.update(fixture.request).plan.status(),
            nav_kernel::LocalPlanStatus::Pending);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  fixture.route[1] = {0.0, 2.0, 0.5};
  fixture.request.objective = nav_kernel::RouteTarget{
      {fixture.route.data(), static_cast<int>(fixture.route.size()), 2}};

  nav_kernel::LocalPlan plan;
  for (int tick = 0; tick < 400; ++tick) {
    fixture.request.clock.timestampS = 1.1 + 0.01 * tick;
    fixture.setGeneration(1);
    plan = task.update(fixture.request).plan;
    if (plan.ready()) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  ASSERT_TRUE(plan.ready());
  ASSERT_FALSE(plan.previewPath().empty());
  EXPECT_GT(plan.previewPath().back().y,
            std::abs(plan.previewPath().back().x));
}

TEST(LocalPlanTask, ProcessesResetEpochOnCollisionTimer) {
  nav_kernel::local::scan::Task task(scanParams());
  ASSERT_TRUE(task.configure());
  RequestFixture fixture;
  const nav_kernel::LocalPlan ready = waitForPlan(task, fixture);
  ASSERT_TRUE(ready.ready());
  const auto firstId =
      std::get<nav_kernel::SplineTarget>(ready.target()).trajectoryId;

  fixture.bitmap.occupyInflated({0.5F, 0.0F, 0.5F}, 0.4, 0.2, 0.2);
  nav_kernel::LocalPlan updated;
  for (int tick = 0; tick < 400; ++tick) {
    fixture.request.clock.timestampS += 0.01;
    fixture.setGeneration(1);
    fixture.request.environment.collision.resetEpoch = 2;
    updated = task.update(fixture.request).plan;
    if (updated.ready() &&
        std::get<nav_kernel::SplineTarget>(updated.target()).trajectoryId >
            firstId) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  ASSERT_TRUE(updated.ready());
  EXPECT_GT(std::get<nav_kernel::SplineTarget>(updated.target()).trajectoryId,
            firstId);
}
