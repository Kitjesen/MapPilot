#include <algorithm>
#include <gtest/gtest.h>

#include "navigation/executor.hpp"
#include "navigation/stairs.hpp"
#include "endpoint/nav/app/stair_profile.hpp"

namespace {
using namespace lingtu::nav::navigation;

StairFlight flight() {
  StairFlight value;
  value.id = "flight-a";
  value.lateralMin = -0.6;
  value.lateralMax = 0.6;
  value.treads = {{-2.0, 0.0, 0.0}, {0.0, 0.3, 0.15},
                  {0.3, 0.6, 0.30}, {0.6, 0.9, 0.45}, {0.9, 3.0, 0.6}};
  return value;
}

std::vector<nav_kernel::Vec3> path() {
  return {{-1.5, 0, 0.4}, {-0.01, 0, 0.4}, {0.01, 0, 0.55},
          {0.29, 0, 0.55}, {0.31, 0, 0.7}, {0.59, 0, 0.7},
          {0.61, 0, 0.85}, {0.89, 0, 0.85}, {0.91, 0, 1.0}, {2, 0, 1.0}};
}

StairProfileFeedback ready(StairProfile profile, double stamp = 1.0) {
  return {profile, true, false, "read-back-profile", "", stamp};
}

StairTraversal session() {
  StairTraversal session;
  session.setSegments(resolveStairSegments(path(), {flight()}, 0.4));
  return session;
}

nav_kernel::Pose pose(double x, double z, double yaw = 0.0) {
  nav_kernel::Pose result;
  result.position = {x, 0, z};
  result.yaw = yaw;
  return result;
}

TEST(StairPath, MatchesObservedTreadsInBothDirections) {
  auto route = path();
  auto up = resolveStairSegments(route, {flight()}, 0.4);
  ASSERT_EQ(up.size(), 1u);
  EXPECT_TRUE(up.front().ascending);
  EXPECT_LT(up.front().pathBegin, up.front().pathEnd);
  std::reverse(route.begin(), route.end());
  const auto down = resolveStairSegments(route, {flight()}, 0.4);
  ASSERT_EQ(down.size(), 1u);
  EXPECT_FALSE(down.front().ascending);
}

TEST(StairPath, MatchesConnectedObservedLandingGap) {
  auto observed = flight();
  // Recognition at 5 cm accepts this measured gap (2.5 sampling cells).
  observed.treads.front().end = -0.12;
  auto route = path();
  const auto up = resolveStairSegments(route, {observed}, 0.4);
  ASSERT_EQ(up.size(), 1u);
  EXPECT_TRUE(up.front().ascending);
  std::reverse(route.begin(), route.end());
  ASSERT_EQ(resolveStairSegments(route, {observed}, 0.4).size(), 1u);
}

TEST(StairPath, MatchesNativeMujocoVoxelRouteAgainstObservedSurfaces) {
  // native-sequence-v2 / showcase_rays_v4: preserve every elevation change
  // from the real OctoPlanner route, omitting only collinear 5 cm samples.
  StairFlight observed;
  observed.id = "showcase";
  observed.origin = {-0.800000012, 0.0, 0.03};
  observed.lateralMin = -1.2983;
  observed.lateralMax = 1.2979;
  observed.treads = {
      {-0.599675119, 0.699830830, 0.029599514},
      {0.819999874, 1.180000305, 0.059984608},
      {1.179999828, 1.540000439, 0.119840763},
      {1.539999723, 1.900000334, 0.180625304},
      {1.899999857, 2.260000229, 0.239122043},
      {2.259999752, 2.620000601, 0.299916431},
      {2.619999647, 3.800000429, 0.360139401},
      {3.799999475, 4.160000801, 0.419154684},
      {4.159999371, 4.520000458, 0.479942113},
      {4.519999504, 4.880000591, 0.540047681},
      {4.879999638, 5.240000725, 0.599723949},
      {5.239999771, 5.600000381, 0.659718476},
      {5.599999428, 7.399969578, 0.719917315},
  };
  std::vector<nav_kernel::Vec3> route{
      {-0.999052, -0.000723, 0.471351}, {0.700948, -0.000723, 0.471351},
      {0.750948, -0.000723, 0.521351}, {1.050948, -0.000723, 0.521351},
      {1.100948, -0.000723, 0.571351}, {1.400948, -0.000723, 0.571351},
      {1.450948, -0.000723, 0.621351}, {1.500948, -0.000723, 0.671351},
      {1.800948, -0.000723, 0.671351}, {1.850948, -0.000723, 0.721351},
      {2.950948, -0.000723, 0.721351}, {3.000948, -0.000723, 0.771351},
      {3.350948, -0.000723, 0.771351}, {3.400948, -0.000723, 0.821351},
      {3.700948, -0.000723, 0.821351}, {3.750948, -0.000723, 0.871351},
      {4.000948, -0.000723, 0.871351}, {4.050948, -0.000723, 0.921351},
      {4.100948, -0.000723, 0.971351}, {4.400948, -0.000723, 0.971351},
      {4.450948, -0.000723, 1.021351}, {4.750948, -0.000723, 1.021351},
      {4.800948, -0.000723, 1.071351}, {5.800948, -0.000723, 1.071351},
      {5.850948, -0.000723, 1.121351}, {5.900948, -0.000723, 1.171351},
  };
  const double tolerance = StairTraversalConfig{}.heightTolerance;
  const auto up = resolveStairSegments(route, {observed}, 0.435, tolerance);
  ASSERT_EQ(up.size(), 1u);
  EXPECT_TRUE(up.front().ascending);
  StairTraversalConfig thunder;
  thunder.bodyLength = 1.0;
  thunder.bodyWidth = 0.6;
  StairTraversal stairs(thunder);
  stairs.setSegments(up);
  const auto approach = stairs.step(pose(-.999052, .471351), {}, true, 1, {});
  ASSERT_TRUE(approach.approachTarget);
  EXPECT_FALSE(approach.hold);
  EXPECT_EQ(approach.phase, StairPhase::Approach);
  EXPECT_LT(approach.approachTarget->x, -.4);
  EXPECT_EQ(stairs.step(pose(approach.approachTarget->x, approach.approachTarget->z),
                       {}, true, 1, {}).request, StairProfile::Ascent);
  auto below = route;
  for (auto& point : below) point.z = .471351;
  EXPECT_TRUE(resolveStairSegments(below, {observed}, .435, tolerance).empty());
  auto other_floor = route;
  for (auto& point : other_floor) point.z += 3.0;
  EXPECT_TRUE(resolveStairSegments(other_floor, {observed}, .435, tolerance).empty());
  std::reverse(route.begin(), route.end());
  const auto down = resolveStairSegments(route, {observed}, .435, tolerance);
  ASSERT_EQ(down.size(), 1u);
  EXPECT_FALSE(down.front().ascending);
}

TEST(StairPath, RejectsBesideBelowAndWrongLevelWithSameXY) {
  auto beside = path();
  for (auto& p : beside) p.y = 0.9;
  EXPECT_TRUE(resolveStairSegments(beside, {flight()}, 0.4).empty());
  auto below = path();
  for (auto& p : below) p.z = 0.4;
  EXPECT_TRUE(resolveStairSegments(below, {flight()}, 0.4).empty());
  auto other_floor = path();
  for (auto& p : other_floor) p.z += 3.0;
  EXPECT_TRUE(resolveStairSegments(other_floor, {flight()}, 0.4).empty());
}

TEST(StairPath, DoesNotAttachToEndpointsWhoseMiddleLeavesFlight) {
  auto route = path();
  route[4].y = 1.0;
  EXPECT_TRUE(resolveStairSegments(route, {flight()}, 0.4).empty());
}

TEST(StairPath, ResolvesMultipleFlightsInRouteOrder) {
  auto second = flight();
  second.id = "flight-b";
  second.origin.x = 4;
  for (auto& tread : second.treads) tread.height += 0.6;
  auto route = path();
  auto extension = path();
  for (auto& point : extension) { point.x += 4; point.z += 0.6; }
  route.insert(route.end(), extension.begin(), extension.end());
  const auto result = resolveStairSegments(route, {second, flight()}, 0.4);
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0].flight.id, "flight-a");
  EXPECT_EQ(result[1].flight.id, "flight-b");
  EXPECT_LT(result[0].pathEnd, result[1].pathBegin);
}

TEST(StairPath, RetainsRequirementWhenGoalOrReplanStartsOnFlight) {
  auto route = path();
  route.resize(5);
  ASSERT_EQ(resolveStairSegments(route, {flight()}, 0.4).size(), 1u);
  route = path();
  route.erase(route.begin(), route.begin() + 4);
  const auto segments = resolveStairSegments(route, {flight()}, 0.4);
  ASSERT_EQ(segments.size(), 1u);
  StairTraversal stairs;
  stairs.setSegments(segments);
  const auto output = stairs.step(pose(0.31, 0.7), {}, true, 1, {});
  EXPECT_TRUE(output.hold);
  EXPECT_EQ(output.request, StairProfile::None);
  EXPECT_EQ(output.reason, "stair_entry_required");
}

TEST(StairTraversal, StopsWholeFootprintOnLandingBeforeRequest) {
  auto stairs = session();
  auto output = stairs.step(pose(-1, 0.4), {}, true, 1, {});
  ASSERT_TRUE(output.approachTarget);
  EXPECT_NEAR(output.approachTarget->x, -0.4, 1e-9);
  EXPECT_EQ(output.request, StairProfile::None);
  EXPECT_FALSE(output.hold);
  output = stairs.step(pose(-0.4, 0.4), {0.2, 0, 0}, true, 1, {});
  EXPECT_EQ(output.request, StairProfile::None);
  output = stairs.step(pose(-0.4, 0.4), {}, true, 1, {});
  EXPECT_EQ(output.request, StairProfile::Ascent);
  EXPECT_EQ(output.phase, StairPhase::Switching);
  EXPECT_TRUE(output.hold);
}

TEST(StairTraversal, RejectsLandingShorterThanPhysicalFootprintAndEntryMargin) {
  auto short_landing = flight();
  short_landing.treads.front().begin = -1.05;
  StairTraversalConfig thunder;
  thunder.bodyLength = 1.0;
  thunder.bodyWidth = 0.6;
  StairTraversal stairs(thunder);
  stairs.setSegments(resolveStairSegments(path(), {short_landing}, .4));
  const auto output = stairs.step(pose(-.6, .4), {}, true, 1, {});
  EXPECT_TRUE(output.hold);
  EXPECT_EQ(output.reason, "stair_landing_space");
  EXPECT_EQ(output.request, StairProfile::None);
}

TEST(StairTraversal, DiagonalLandingTargetStaysOnTheRouteAxis) {
  constexpr double diagonal = 0.7071067811865476;
  auto observed = flight();
  observed.axis = {diagonal, diagonal, 0};
  observed.origin = {2, 3, 0};
  auto route = path();
  for (auto& point : route) {
    const double x = point.x;
    point.x = observed.origin.x + x * diagonal;
    point.y = observed.origin.y + x * diagonal;
  }
  StairTraversal stairs;
  stairs.setSegments(resolveStairSegments(route, {observed}, .4));
  auto body = pose(2 - diagonal, .4, 0.7853981633974483);
  body.position.y = 3 - diagonal;
  auto output = stairs.step(body, {}, true, 1, {});
  ASSERT_TRUE(output.approachTarget);
  EXPECT_NEAR(output.approachTarget->x, 2 - .4 * diagonal, 1e-9);
  EXPECT_NEAR(output.approachTarget->y, 3 - .4 * diagonal, 1e-9);
  body.position = *output.approachTarget;
  EXPECT_EQ(stairs.step(body, {}, true, 1, {}).request, StairProfile::Ascent);
}

TEST(StairTraversal, FutureFlightDoesNotGateOrdinaryApproachBesideOrBelowIt) {
  auto stairs = session();
  auto beside = pose(0.5, 0.4);
  beside.position.y = 2;
  auto output = stairs.step(beside, {}, true, 1, {});
  EXPECT_FALSE(output.hold);
  EXPECT_FALSE(output.onFlight);
  EXPECT_FALSE(output.constrainRecovery);
  EXPECT_EQ(output.speedLimit, 0);
  EXPECT_EQ(output.request, StairProfile::None);
  auto below = pose(0.6, -0.6);
  output = stairs.step(below, {}, true, 1, {});
  EXPECT_FALSE(output.hold);
  EXPECT_FALSE(output.onFlight);
  EXPECT_EQ(output.speedLimit, 0);
  output = stairs.step(pose(-10, 0.4), {}, true, 1, {});
  EXPECT_FALSE(output.hold);
  EXPECT_EQ(output.speedLimit, 0);
}

TEST(StairTraversal, GroundedTransitionDoesNotDropSwitchRequest) {
  auto stairs = session();
  stairs.step(pose(-0.4, 0.4), {}, true, 1, {});
  const auto output = stairs.step(pose(-0.4, 0.1), {}, true, 1.1, {});
  EXPECT_EQ(output.request, StairProfile::Ascent);
  EXPECT_TRUE(output.hold);
  EXPECT_EQ(output.phase, StairPhase::Switching);
}

TEST(StairTraversal, RequiresFreshReadbackAndReadyNotAcknowledgment) {
  auto stairs = session();
  stairs.step(pose(-0.4, 0.4), {}, true, 1, {});
  auto acknowledgment = ready(StairProfile::Ascent);
  acknowledgment.activeProfile.clear();
  EXPECT_TRUE(stairs.step(pose(-0.4, 0.4), {}, true, 1, acknowledgment).hold);
  auto not_standing = ready(StairProfile::Ascent);
  not_standing.ready = false;
  EXPECT_TRUE(stairs.step(pose(-0.4, 0.4), {}, true, 1, not_standing).hold);
  auto output = stairs.step(pose(-0.4, 0.4), {}, true, 1, ready(StairProfile::Ascent));
  EXPECT_FALSE(output.hold);
  EXPECT_EQ(output.phase, StairPhase::Traversing);
  ASSERT_TRUE(output.bodyYaw);
  EXPECT_NEAR(*output.bodyYaw, 0, 1e-9);
  EXPECT_TRUE(stairs.step(pose(0.4, 0.7), {}, true, 3, ready(StairProfile::Ascent)).hold);
  EXPECT_FALSE(stairs.step(pose(0.4, 0.7), {}, true, 3, ready(StairProfile::Ascent, 3)).hold);
}

TEST(StairTraversal, DoesNotRestoreUntilRearFootprintClearsObservedLanding) {
  auto stairs = session();
  stairs.step(pose(-0.4, 0.4), {}, true, 1, {});
  stairs.step(pose(-0.4, 0.4), {}, true, 1, ready(StairProfile::Ascent));
  EXPECT_EQ(stairs.step(pose(1.1, 1), {}, true, 1, ready(StairProfile::Ascent)).request,
            StairProfile::Ascent);
  auto output = stairs.step(pose(1.35, 1), {}, true, 1, ready(StairProfile::Ascent));
  EXPECT_EQ(output.request, StairProfile::Restore);
  EXPECT_TRUE(output.hold);
  output = stairs.step(pose(1.35, 1), {}, true, 1, ready(StairProfile::Restore));
  EXPECT_EQ(output.phase, StairPhase::Ordinary);
  EXPECT_FALSE(output.bodyYaw);
  EXPECT_FALSE(stairs.pending());
}

TEST(StairTraversal, ExitGoalUsesObservedLandingWithinExistingGoalTolerance) {
  auto stairs = session();
  stairs.step(pose(-0.4, 0.4), {}, true, 1, {});
  stairs.step(pose(-0.4, 0.4), {}, true, 1, ready(StairProfile::Ascent));
  const auto outside = stairs.guideGoal({1.1, 0, 1}, 0.45, 0);
  EXPECT_FALSE(outside.arrivalAllowed);
  EXPECT_FALSE(outside.hold);
  ASSERT_TRUE(outside.target);
  EXPECT_NEAR(outside.target->x, 1.35, 1e-9);
  EXPECT_TRUE(stairs.guideGoal({1.1, 0, 1}, 0.10, 0).hold);
  EXPECT_TRUE(stairs.guideGoal({0.4, 0, 0.7}, 0.45, 0).arrivalAllowed);
  stairs.step(pose(1.35, 1), {}, true, 1, ready(StairProfile::Ascent));
  EXPECT_FALSE(stairs.guideGoal({1.1, 0, 1}, 0.45, 0).arrivalAllowed);
  stairs.step(pose(1.35, 1), {}, true, 1, ready(StairProfile::Restore));
  EXPECT_TRUE(stairs.guideGoal({1.1, 0, 1}, 0.45, 0).arrivalAllowed);
}

TEST(StairTraversal, ExitRestoreWaitsForMeasuredStopButRenewalSurvivesOdometryLoss) {
  auto stairs = session();
  stairs.step(pose(-0.4, 0.4), {}, true, 1, {});
  stairs.step(pose(-0.4, 0.4), {}, true, 1, ready(StairProfile::Ascent));
  auto output = stairs.step(pose(1.35, 1), {0.1, 0, 0}, true, 1,
                           ready(StairProfile::Ascent));
  EXPECT_TRUE(output.hold);
  EXPECT_EQ(output.reason, "stair_settling");
  EXPECT_EQ(output.request, StairProfile::Ascent);
  output = stairs.step(pose(1.35, 1), {0, 0, 0.2}, true, 1,
                       ready(StairProfile::Ascent));
  EXPECT_NE(output.request, StairProfile::Restore);
  output = stairs.step(pose(1.35, 1), {}, false, 1, ready(StairProfile::Ascent));
  EXPECT_NE(output.request, StairProfile::Restore);
  output = stairs.step(pose(1.35, 1), {}, true, 1, ready(StairProfile::Ascent));
  EXPECT_TRUE(output.hold);
  EXPECT_EQ(output.request, StairProfile::Restore);
  output = stairs.step(pose(1.35, 0.7), {}, false, 1, {});
  EXPECT_TRUE(output.hold);
  EXPECT_EQ(output.request, StairProfile::Restore);
  output = stairs.step(pose(1.35, 1), {}, false, 1, ready(StairProfile::Restore));
  EXPECT_EQ(output.phase, StairPhase::Ordinary);
  EXPECT_FALSE(stairs.pending());
}

TEST(StairTraversal, ExitGuideNeverExtendsOutsideObservedLanding) {
  auto narrow_landing = flight();
  narrow_landing.treads.back().end = 1.5;
  StairTraversal stairs;
  stairs.setSegments(resolveStairSegments(path(), {narrow_landing}, 0.4));
  stairs.step(pose(-0.4, 0.4), {}, true, 1, {});
  stairs.step(pose(-0.4, 0.4), {}, true, 1, ready(StairProfile::Ascent));
  EXPECT_TRUE(stairs.guideGoal({1.1, 0, 1}, 0.45, 0).hold);
}

TEST(StairTraversal, CancellationAndNewPathKeepProfileWhileOnFlight) {
  auto stairs = session();
  stairs.step(pose(-0.4, 0.4), {}, true, 1, {});
  stairs.step(pose(-0.4, 0.4), {}, true, 1, ready(StairProfile::Ascent));
  stairs.cancel();
  auto output = stairs.step(pose(0.4, 0.7), {}, true, 1, ready(StairProfile::Ascent));
  EXPECT_TRUE(output.hold);
  EXPECT_EQ(output.request, StairProfile::None);
  EXPECT_NE(output.request, StairProfile::Restore);
  stairs.setSegments({});
  output = stairs.step(pose(0.4, 0.7), {}, true, 1, ready(StairProfile::Ascent));
  EXPECT_EQ(output.request, StairProfile::None);
  EXPECT_TRUE(output.hold);
  output = stairs.step(pose(1.35, 1), {}, true, 1, ready(StairProfile::Ascent));
  EXPECT_EQ(output.request, StairProfile::Restore);
}

TEST(StairTraversal, ReplanInSameDirectionKeepsExistingFlightSession) {
  auto stairs = session();
  stairs.step(pose(-0.4, 0.4), {}, true, 1, {});
  stairs.step(pose(-0.4, 0.4), {}, true, 1, ready(StairProfile::Ascent));
  auto replanned = path();
  replanned.erase(replanned.begin(), replanned.begin() + 4);
  stairs.setSegments(resolveStairSegments(replanned, {flight()}, 0.4));
  const auto output = stairs.step(pose(0.31, 0.7), {}, true, 1, ready(StairProfile::Ascent));
  EXPECT_EQ(output.phase, StairPhase::Traversing);
  EXPECT_FALSE(output.hold);
}

TEST(StairTraversal, CancellationOnGroundedEntranceRestoresExactDriverBaseline) {
  auto stairs = session();
  stairs.step(pose(-0.4, 0.4), {}, true, 1, {});
  stairs.cancel();
  const auto output = stairs.step(pose(-0.4, 0.1), {}, true, 1, {});
  EXPECT_EQ(output.request, StairProfile::Restore);
  EXPECT_TRUE(output.hold);
}

TEST(StairTraversal, UnsupportedProfileOnlyHoldsAffectedStairRoute) {
  auto stairs = session();
  StairProfileFeedback unsupported;
  unsupported.unsupported = true;
  const auto output = stairs.step(pose(-0.4, 0.4), {}, true, 1, unsupported);
  EXPECT_TRUE(output.hold);
  EXPECT_EQ(output.reason, "stair_profile_unsupported");
  StairTraversal ordinary;
  EXPECT_FALSE(ordinary.step(pose(0, 0.4), {}, true, 1, unsupported).hold);
}

TEST(StairTraversal, RejectsLateralLocalDetourOffObservedTreads) {
  auto stairs = session();
  stairs.step(pose(-0.4, 0.4), {}, true, 1, {});
  stairs.step(pose(-0.4, 0.4), {}, true, 1, ready(StairProfile::Ascent));
  EXPECT_TRUE(stairs.localPathAllowed({{0.3, 0, 0.7}, {0.6, 0, 0.85}}, 0));
  EXPECT_FALSE(stairs.localPathAllowed({{0.3, 0, 0.7}, {0.6, 0.5, 0.85}}, 0));
  // A lateral step retains the flight heading; its tangent is not body yaw.
  EXPECT_TRUE(stairs.localPathAllowed({{0.3, 0.3, 0.7}, {0.31, 0.39, 0.7}}, 0));
}

TEST(StairTraversal, SideStepKeepsActualTurningFootprintInsideObservedWidth) {
  const double pi = std::acos(-1.0);
  for (bool ascending : {true, false}) {
    auto observed = flight();
    observed.lateralMin = -1.2983;
    observed.lateralMax = 1.2979;
    auto route = path();
    if (!ascending) std::reverse(route.begin(), route.end());
    StairTraversalConfig config;
    config.bodyLength = 1.0;
    config.bodyWidth = 0.6;
    StairTraversal stairs(config);
    stairs.setSegments(resolveStairSegments(route, {observed}, 0.4));
    const auto entrance = ascending ? pose(-0.6, 0.4) : pose(1.5, 1.0, pi);
    const auto profile = ascending ? StairProfile::Ascent : StairProfile::Descent;
    stairs.step(entrance, {}, true, 1, {});
    ASSERT_EQ(stairs.step(entrance, {}, true, 1, ready(profile)).phase,
              StairPhase::Traversing);
    // Real Thunder dimensions and measured flight width. The complete turn
    // from .12 rad fits (outer edge 1.2577); tangent-as-yaw would give 1.40.
    const double yaw = ascending ? 0.12 : -pi + 0.12;
    EXPECT_TRUE(stairs.localPathAllowed({{0.4, 0.90, 0.7}, {0.4, 0.80, 0.7}}, yaw));
    // The v12 rejected curve really leaves the observed flight, even aligned.
    EXPECT_FALSE(stairs.localPathAllowed({{0.4, 1.2934, 0.7}, {0.5, 1.20, 0.7}},
                                        ascending ? 0.0 : pi));
  }
}

TEST(StairTraversal, WidthIncludesInteriorMaximumOfAlignmentTurn) {
  auto observed = flight();
  observed.lateralMin = -0.55;
  observed.lateralMax = 0.55;
  StairTraversalConfig config;
  config.bodyLength = 1.0;
  config.bodyWidth = 0.6;
  StairTraversal stairs(config);
  stairs.setSegments(resolveStairSegments(path(), {observed}, 0.4));
  stairs.step(pose(-0.6, 0.4), {}, true, 1, {});
  ASSERT_EQ(stairs.step(pose(-0.6, 0.4), {}, true, 1, ready(StairProfile::Ascent)).phase,
            StairPhase::Traversing);
  const std::vector<nav_kernel::Vec3> local{{0.4, 0.01, 0.7}, {0.5, 0.02, 0.7}};
  EXPECT_TRUE(stairs.localPathAllowed(local, 0.0));
  // Both end orientations fit, but the intermediate 59-degree footprint has
  // a .5831 m half-width and cannot turn inside this 1.10 m flight.
  EXPECT_FALSE(stairs.localPathAllowed(local, std::acos(-1.0) / 2));
}

TEST(StairTraversal, DescendingHasOwnRequestAndRestoresOnLowerLanding) {
  auto route = path();
  std::reverse(route.begin(), route.end());
  StairTraversal stairs;
  stairs.setSegments(resolveStairSegments(route, {flight()}, 0.4));
  const double pi = 3.14159265358979323846;
  auto output = stairs.step(pose(1.3, 1, pi), {}, true, 1, {});
  EXPECT_EQ(output.request, StairProfile::Descent);
  output = stairs.step(pose(0.4, 0.7, pi), {}, true, 1, ready(StairProfile::Descent));
  EXPECT_EQ(output.phase, StairPhase::Traversing);
  ASSERT_TRUE(output.bodyYaw);
  EXPECT_NEAR(std::abs(*output.bodyYaw), pi, 1e-9);
  output = stairs.step(pose(-0.45, 0.4, pi), {}, true, 1, ready(StairProfile::Descent));
  EXPECT_EQ(output.request, StairProfile::Restore);
}

TEST(StairTraversal, ConnectedFlightsKeepProfileAndAlignAtNextLanding) {
  auto first = resolveStairSegments(path(), {flight()}, 0.4).front();
  auto second = first;
  second.flight.id = "flight-b";
  second.flight.origin.x = 3;
  for (auto& tread : second.flight.treads) tread.height += 0.6;
  second.entry.x += 3;
  second.entry.z += 0.6;
  second.exit.x += 3;
  second.exit.z += 0.6;
  StairTraversal stairs;
  stairs.setSegments({first, second});
  stairs.step(pose(-0.4, 0.4), {}, true, 1, {});
  stairs.step(pose(-0.4, 0.4), {}, true, 1, ready(StairProfile::Ascent));
  const auto output = stairs.step(pose(1.4, 1), {}, true, 1, ready(StairProfile::Ascent));
  EXPECT_EQ(output.flightId, "flight-b");
  EXPECT_EQ(output.phase, StairPhase::Approach);
  EXPECT_NE(output.request, StairProfile::Restore);
  ASSERT_TRUE(output.approachTarget);
  EXPECT_NEAR(output.approachTarget->x, 2.6, 1e-9);
}

TEST(StairExecutor, RequestsAtLandingAndKeepsRestorationAfterClear) {
  ExecutorConfig config;
  nav_kernel::LocalPlannerParams params;
  params.vehicleWidth = 0.4;
  Executor executor(config, nav_kernel::LocalPlanner(params));
  Route route;
  route.points = path();
  route.stairs = resolveStairSegments(route.points, {flight()}, 0.4);
  executor.setRoute(route);
  ExecutionInput input;
  input.mapBody = pose(-0.4, 0.4);
  input.odomBody = input.mapBody;
  input.timestampS = 1;
  input.observation.body_velocity_valid = true;
  auto output = executor.tick(input);
  EXPECT_EQ(output.stairs.request, StairProfile::Ascent);
  EXPECT_EQ(output.cmd_vel.vx, 0);
  executor.clear();
  EXPECT_TRUE(executor.hasStairTransition());
  output = executor.tick(input);
  EXPECT_EQ(output.stairs.request, StairProfile::Restore);
  EXPECT_FALSE(output.active);
}

TEST(StairExecutor, OutsideGoalCannotFinishUntilFootprintClearAndProfileRestored) {
  ExecutorConfig config;
  nav_kernel::LocalPlannerParams params;
  Executor executor(config, nav_kernel::LocalPlanner(params));
  Route route;
  route.points = path();
  route.points.back() = {1.1, 0, 1};
  route.goalToleranceM = 0.45;
  route.stairs = resolveStairSegments(route.points, {flight()}, 0.4);
  executor.setRoute(route);
  ExecutionInput input;
  input.mapBody = input.odomBody = pose(-0.4, 0.4);
  input.timestampS = 1;
  input.observation.body_velocity_valid = true;
  EXPECT_EQ(executor.tick(input).stairs.request, StairProfile::Ascent);
  input.stairProfile = ready(StairProfile::Ascent);
  input.mapBody = input.odomBody = pose(1.1, 1);
  input.observation.odom_stamp_s = 1;
  auto output = executor.tick(input);
  EXPECT_FALSE(output.goal_reached);
  EXPECT_NE(output.reason, "settling_at_goal");
  input.mapBody = input.odomBody = pose(1.35, 1);
  output = executor.tick(input);
  EXPECT_EQ(output.stairs.request, StairProfile::Restore);
  EXPECT_FALSE(output.goal_reached);
  input.stairProfile = ready(StairProfile::Restore);
  output = executor.tick(input);
  EXPECT_EQ(output.reason, "settling_at_goal");
  for (int i = 1; i <= 6; ++i) {
    input.timestampS = input.observation.odom_stamp_s = 1 + 0.1 * i;
    output = executor.tick(input);
  }
  EXPECT_TRUE(output.goal_reached);
  EXPECT_FALSE(executor.hasStairTransition());
}

TEST(StairExecutor, DeliberateOnFlightGoalRetainsStairProfile) {
  ExecutorConfig config;
  nav_kernel::LocalPlannerParams params;
  Executor executor(config, nav_kernel::LocalPlanner(params));
  Route route;
  route.points = path();
  route.points.resize(5);
  route.stairs = resolveStairSegments(route.points, {flight()}, 0.4);
  executor.setRoute(route);
  ExecutionInput input;
  input.mapBody = input.odomBody = pose(-0.4, 0.4);
  input.timestampS = 1;
  input.observation.body_velocity_valid = true;
  executor.tick(input);
  input.stairProfile = ready(StairProfile::Ascent);
  input.mapBody = input.odomBody = pose(0.31, 0.7);
  ExecutionOutput output;
  for (int i = 0; i <= 6; ++i) {
    input.timestampS = input.observation.odom_stamp_s = 1 + 0.1 * i;
    output = executor.tick(input);
  }
  EXPECT_TRUE(output.goal_reached);
  EXPECT_EQ(output.stairs.request, StairProfile::Ascent);
  EXPECT_TRUE(executor.hasStairTransition());
}

TEST(StairFeedback, RequiresReadyAndMatchingActualReadback) {
  using namespace lingtu::nav::endpoint;
  StairProfileState state{100, 10, "task", "stairs_up", "switching", "stairs-model", "ordinary", ""};
  EXPECT_FALSE(stairProfileFeedback(state, "task", 100, 10, 1).ready);
  state.phase = "ready";
  EXPECT_FALSE(stairProfileFeedback(state, "task", 100, 10, 1).ready);
  state.activeProfile = "stairs-model";
  EXPECT_TRUE(stairProfileFeedback(state, "task", 100, 10, 1).ready);
  state.requestedProfile.clear();
  state.activeProfile.clear();
  EXPECT_FALSE(stairProfileFeedback(state, "task", 100, 10, 1).ready);
}

TEST(StairFeedback, RejectsStaleSourceReceiptOrDifferentTask) {
  using namespace lingtu::nav::endpoint;
  StairProfileState state{100, 10, "task", "stairs_up", "ready", "stairs-model", "stairs-model", ""};
  EXPECT_TRUE(stairProfileFeedback(state, "task", 100, 10, 1).ready);
  EXPECT_FALSE(stairProfileFeedback(state, "other", 100, 10, 1).ready);
  EXPECT_FALSE(stairProfileFeedback(state, "task", 102, 10, 1).ready);
  EXPECT_FALSE(stairProfileFeedback(state, "task", 100, 12, 1).ready);
}

TEST(StairFeedback, ExpectedBodyTransitionDoesNotMaskMotorFault) {
  using namespace lingtu::nav::endpoint;
  StairProfileState state{100, 10, "task", "stairs_up", "grounding", "stairs-model", "ordinary", ""};
  EXPECT_TRUE(expectedProfileTransition(state, "task", "profile_transition", 100, 10));
  EXPECT_FALSE(expectedProfileTransition(state, "task", "motor_fault", 100, 10));
  EXPECT_FALSE(expectedProfileTransition(state, "other", "profile_transition", 100, 10));
  EXPECT_FALSE(expectedProfileTransition(state, "task", "profile_transition", 102, 10));
  state.phase = "unsupported";
  EXPECT_FALSE(expectedProfileTransition(state, "task", "profile_transition", 100, 10));
}

TEST(StairFeedback, PendingRequestAllowsDriverTransitionButNeverFakesReady) {
  using namespace lingtu::nav::endpoint;
  StairProfileState state;
  EXPECT_TRUE(expectedProfileTransition(state, "task", "profile_transition", 100, 10, 10));
  EXPECT_FALSE(expectedProfileTransition(state, "task", "motor_fault", 100, 10, 10));
  EXPECT_FALSE(expectedProfileTransition(state, "", "profile_transition", 100, 10, 10));
  EXPECT_FALSE(expectedProfileTransition(state, "task", "profile_transition", 100, 12, 10));
  EXPECT_FALSE(stairProfileFeedback(state, "task", 100, 10, 1).ready);
}
}  // namespace
