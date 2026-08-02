#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "plan/active_path_blockage_policy.hpp"
#include "plan/goal_plan_controller.hpp"

namespace {

using lingtu::nav::endpoint::ActivePathBlockageObservation;
using lingtu::nav::endpoint::ActivePathBlockagePolicy;
using lingtu::nav::endpoint::ActivePathBlockagePolicyConfig;
using lingtu::nav::endpoint::goalPlanAcceptsReplanTrigger;
using lingtu::nav::endpoint::GoalPlanSnapshot;
using lingtu::nav::endpoint::GoalReplanIdentity;
using lingtu::nav::endpoint::GoalReplanTriggerKind;
using lingtu::nav::plan::MapIdentity;

void require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "test_active_path_blockage_policy: FAIL: %s\n", message);
    std::exit(1);
  }
}

MapIdentity mapIdentity(std::string id = "map-a", std::int64_t version = 7,
                        std::string sha = "sha-a", std::string frame = "map") {
  return {std::move(id), version, std::move(sha), std::move(frame)};
}

GoalReplanIdentity goalIdentity(std::string task = "task-a", std::string request = "req-a",
                                std::uint64_t epoch = 11U, MapIdentity map = mapIdentity()) {
  return {std::move(task), std::move(request), epoch, std::move(map)};
}

ActivePathBlockagePolicyConfig config() {
  ActivePathBlockagePolicyConfig value;
  value.persistence_s = 1.0;
  value.minimum_fresh_observations = 3U;
  value.lookahead_m = 5.0;
  value.corridor_radius_m = 0.50;
  value.corridor_vertical_tolerance_m = 0.75;
  value.overlay_radius_m = 0.65;
  value.overlay_half_height_m = 1.25;
  value.max_regions = 8U;
  value.minimum_obstacle_points = 4U;
  return value;
}

std::vector<nav_kernel::Vec3> path() {
  return {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {4.0, 0.0, 0.0}, {6.0, 0.0, 0.0}, {8.0, 0.0, 0.0}};
}

std::vector<float> pointCloud(std::initializer_list<std::array<float, 4>> points) {
  std::vector<float> result;
  result.reserve(points.size() * 4U);
  for (const auto &point : points) {
    result.insert(result.end(), point.begin(), point.end());
  }
  return result;
}

void appendObstacleGroup(std::vector<float> &points, float x, float y = 0.0F, float z = 0.2F,
                         float height = 0.4F) {
  constexpr float kOffset = 0.08F;
  const auto group = pointCloud({{x - kOffset, y - kOffset, z, height},
                                 {x - kOffset, y + kOffset, z, height},
                                 {x + kOffset, y - kOffset, z, height},
                                 {x + kOffset, y + kOffset, z, height}});
  points.insert(points.end(), group.begin(), group.end());
}

std::vector<float> staticBlockage(float x = 2.0F) {
  std::vector<float> result;
  appendObstacleGroup(result, x);
  return result;
}

ActivePathBlockageObservation
observation(double now_s, std::uint64_t cloud_generation, std::uint64_t traversability_generation,
            const std::vector<nav_kernel::Vec3> &active_path,
            const std::vector<float> &live_obstacles, GoalReplanIdentity identity = goalIdentity(),
            std::uint64_t frame_epoch = 3U, nav_kernel::Vec3 robot = {0.0, 0.0, 0.0},
            bool external = true) {
  ActivePathBlockageObservation value;
  value.now_s = now_s;
  value.external_active_goal = external;
  value.goal = std::move(identity);
  value.frame_epoch = frame_epoch;
  value.robot_position = robot;
  value.active_global_path = &active_path;
  value.live_obstacles_xyzh = &live_obstacles;
  value.cloud_generation = cloud_generation;
  value.traversability_generation = traversability_generation;
  return value;
}

void testStaticLiveOccupancyTriggers() {
  ActivePathBlockagePolicy policy(config());
  const auto active_path = path();
  const auto blocked = staticBlockage();

  require(!policy.observe(observation(20.0, 10U, 20U, active_path, blocked)),
          "first static occupancy observation triggered");
  require(!policy.observe(observation(20.5, 11U, 21U, active_path, blocked)),
          "second static occupancy observation triggered early");
  const auto trigger = policy.observe(observation(21.0, 12U, 22U, active_path, blocked));
  require(trigger.has_value(), "persistent static live occupancy did not trigger");
  require(trigger->kind == GoalReplanTriggerKind::kPersistentPathObstruction,
          "static occupancy trigger kind changed");
  require(trigger->reason == "persistent_path_obstruction", "trigger reason changed");
  require(lingtu::nav::endpoint::sameGoalReplanIdentity(trigger->goal, goalIdentity()),
          "trigger did not retain active goal identity");
}

void testTransientAndSparseOccupancyDoNotTrigger() {
  const auto active_path = path();
  const auto blocked = staticBlockage();
  const std::vector<float> clear;

  ActivePathBlockagePolicy transient(config());
  require(!transient.observe(observation(10.0, 1U, 1U, active_path, blocked)),
          "first transient observation triggered");
  require(!transient.observe(observation(10.4, 2U, 2U, active_path, blocked)),
          "short transient obstruction triggered");
  require(!transient.observe(observation(10.6, 3U, 3U, active_path, clear)),
          "fresh clear corridor emitted a trigger");
  require(transient.snapshot().fresh_blocked_observations == 0U,
          "fresh clear corridor did not reset transient evidence");

  ActivePathBlockagePolicy sparse(config());
  const auto sparse_points =
      pointCloud({{1.9F, -0.1F, 0.2F, 0.4F}, {2.0F, 0.0F, 0.2F, 0.4F}, {2.1F, 0.1F, 0.2F, 0.4F}});
  for (std::uint64_t generation = 1U; generation <= 5U; ++generation) {
    require(!sparse.observe(observation(12.0 + static_cast<double>(generation), generation,
                                        generation, active_path, sparse_points)),
            "sparse corridor noise triggered");
  }
  require(sparse.snapshot().fresh_blocked_observations == 0U,
          "sparse corridor noise accumulated as blockage");
  require(sparse.snapshot().current_blocker_count == 3U, "sparse point count diagnostics changed");
}

void testOnlyForwardCorridorBlocks() {
  const auto active_path = path();
  std::vector<float> lateral;
  appendObstacleGroup(lateral, 2.0F, 1.2F);
  std::vector<float> beyond_lookahead;
  appendObstacleGroup(beyond_lookahead, 7.0F);

  ActivePathBlockagePolicy lateral_policy(config());
  for (std::uint64_t generation = 1U; generation <= 4U; ++generation) {
    require(!lateral_policy.observe(observation(30.0 + static_cast<double>(generation), generation,
                                                generation, active_path, lateral)),
            "live occupancy outside corridor triggered");
  }

  ActivePathBlockagePolicy distant_policy(config());
  for (std::uint64_t generation = 1U; generation <= 4U; ++generation) {
    require(!distant_policy.observe(observation(35.0 + static_cast<double>(generation), generation,
                                                generation, active_path, beyond_lookahead)),
            "live occupancy beyond lookahead triggered");
  }

  ActivePathBlockagePolicy behind_policy(config());
  std::vector<float> behind;
  appendObstacleGroup(behind, 1.0F);
  const nav_kernel::Vec3 robot_mid_path{4.0, 0.0, 0.0};
  for (std::uint64_t generation = 1U; generation <= 4U; ++generation) {
    require(!behind_policy.observe(observation(40.0 + static_cast<double>(generation), generation,
                                               generation, active_path, behind, goalIdentity(), 3U,
                                               robot_mid_path)),
            "live occupancy behind nearest path point triggered");
  }
}

void testIdentityAndFrameChangesResetEvidence() {
  const auto active_path = path();
  const auto blocked = staticBlockage();

  ActivePathBlockagePolicy goal_change(config());
  require(!goal_change.observe(observation(50.0, 1U, 1U, active_path, blocked)),
          "goal reset setup triggered");
  require(!goal_change.observe(observation(50.6, 2U, 2U, active_path, blocked)),
          "goal reset setup triggered early");
  require(!goal_change.observe(
              observation(51.2, 3U, 3U, active_path, blocked, goalIdentity("task-b", "req-b", 1U))),
          "new goal inherited prior evidence");
  require(goal_change.snapshot().fresh_blocked_observations == 1U,
          "new goal did not restart evidence count");

  ActivePathBlockagePolicy map_change(config());
  require(!map_change.observe(observation(52.0, 1U, 1U, active_path, blocked)),
          "map reset setup triggered");
  require(!map_change.observe(observation(52.6, 2U, 2U, active_path, blocked)),
          "map reset setup triggered early");
  require(!map_change.observe(
              observation(53.2, 3U, 3U, active_path, blocked,
                          goalIdentity("task-a", "req-a", 11U, mapIdentity("map-b", 8, "sha-b")))),
          "new map inherited prior evidence");
  require(map_change.snapshot().fresh_blocked_observations == 1U,
          "new map did not restart evidence count");

  ActivePathBlockagePolicy frame_change(config());
  require(!frame_change.observe(observation(54.0, 1U, 1U, active_path, blocked)),
          "frame reset setup triggered");
  require(!frame_change.observe(observation(54.6, 2U, 2U, active_path, blocked)),
          "frame reset setup triggered early");
  require(
      !frame_change.observe(observation(55.2, 3U, 3U, active_path, blocked, goalIdentity(), 4U)),
      "new frame epoch inherited prior evidence");
  require(frame_change.snapshot().fresh_blocked_observations == 1U,
          "new frame epoch did not restart evidence count");
}

void testStaleClearDoesNotEraseButFreshClearDoes() {
  ActivePathBlockagePolicy policy(config());
  const auto active_path = path();
  const auto blocked = staticBlockage();
  const std::vector<float> clear;

  require(!policy.observe(observation(60.0, 10U, 20U, active_path, blocked)),
          "clear freshness setup triggered");
  require(policy.snapshot().fresh_blocked_observations == 1U,
          "fresh blocked evidence was not recorded");

  require(!policy.observe(observation(60.2, 10U, 20U, active_path, clear)),
          "duplicate-generation clear triggered");
  require(policy.snapshot().fresh_blocked_observations == 1U,
          "duplicate-generation clear erased evidence");

  require(!policy.observe(observation(60.4, 11U, 20U, active_path, clear)),
          "partially fresh clear triggered");
  require(policy.snapshot().fresh_blocked_observations == 1U,
          "partially fresh clear erased evidence");

  require(!policy.observe(observation(60.6, 0U, 21U, active_path, clear)),
          "missing-generation clear triggered");
  require(policy.snapshot().fresh_blocked_observations == 1U,
          "missing-generation clear erased evidence");

  require(!policy.observe(observation(60.8, 11U, 21U, active_path, blocked)),
          "second fresh blocked observation triggered early");
  require(policy.snapshot().fresh_blocked_observations == 2U,
          "blocked evidence did not survive stale clear samples");

  require(!policy.observe(observation(61.0, 12U, 22U, active_path, clear)),
          "fresh clear observation triggered");
  require(policy.snapshot().fresh_blocked_observations == 0U,
          "fresh dual-generation clear did not erase evidence");
}

void testClockAndGenerationRollbackResetEvidence() {
  const auto active_path = path();
  const auto blocked = staticBlockage();

  ActivePathBlockagePolicy clock(config());
  require(!clock.observe(observation(70.0, 10U, 10U, active_path, blocked)),
          "clock rollback setup triggered");
  require(!clock.observe(observation(70.6, 11U, 11U, active_path, blocked)),
          "clock rollback setup triggered early");
  require(!clock.observe(observation(69.0, 12U, 12U, active_path, blocked)),
          "clock rollback triggered");
  require(clock.snapshot().fresh_blocked_observations == 0U,
          "clock rollback did not reset evidence");

  ActivePathBlockagePolicy generation(config());
  require(!generation.observe(observation(80.0, 10U, 20U, active_path, blocked)),
          "generation rollback setup triggered");
  require(!generation.observe(observation(80.6, 11U, 21U, active_path, blocked)),
          "generation rollback setup triggered early");
  require(!generation.observe(observation(81.2, 9U, 22U, active_path, blocked)),
          "generation rollback triggered");
  require(generation.snapshot().fresh_blocked_observations == 0U,
          "generation rollback did not reset evidence");
}

void testOverlayIsDeterministicDeduplicatedAndBounded() {
  auto value = config();
  value.max_regions = 3U;
  ActivePathBlockagePolicy policy(value);
  const auto active_path = path();
  std::vector<float> blocked;
  appendObstacleGroup(blocked, 4.0F);
  appendObstacleGroup(blocked, 1.0F);
  appendObstacleGroup(blocked, 3.0F);
  appendObstacleGroup(blocked, 2.0F);

  require(!policy.observe(observation(90.0, 101U, 201U, active_path, blocked)),
          "overlay setup triggered");
  require(!policy.observe(observation(90.5, 102U, 202U, active_path, blocked)),
          "overlay setup triggered early");
  const auto trigger = policy.observe(observation(91.0, 103U, 203U, active_path, blocked));
  require(trigger.has_value(), "overlay trigger missing");
  const auto &overlay = trigger->temporary_overlay;
  require(overlay.revision != 0U, "overlay revision is zero");
  require(overlay.frame_epoch == 3U, "overlay frame epoch changed");
  require(overlay.obstacle_generation == 103U, "overlay obstacle generation changed");
  require(overlay.traversability_generation == 203U, "overlay traversability generation changed");
  require(overlay.blocked_regions.size() == value.max_regions,
          "overlay spatial deduplication or region cap changed");
  require(overlay.blocked_regions[0].center.x < overlay.blocked_regions[1].center.x &&
              overlay.blocked_regions[1].center.x < overlay.blocked_regions[2].center.x,
          "overlay regions are not ordered by forward path distance");
  require(std::abs(overlay.blocked_regions.front().radius_xy_m - value.overlay_radius_m) < 1e-12,
          "overlay radius changed");
  require(std::abs(overlay.blocked_regions.front().min_z - (0.2 - value.overlay_half_height_m)) <
                  1e-6 &&
              std::abs(overlay.blocked_regions.front().max_z -
                       (0.2 + value.overlay_half_height_m)) < 1e-6,
          "overlay height bounds changed");

  require(!policy.observe(observation(92.0, 104U, 204U, active_path, blocked)),
          "same identity emitted a second trigger");
}

void testInactiveInvalidAndMalformedEvidenceReset() {
  ActivePathBlockagePolicy policy(config());
  const auto active_path = path();
  const std::vector<nav_kernel::Vec3> empty_path;
  const auto blocked = staticBlockage();

  require(!policy.observe(observation(100.0, 1U, 1U, active_path, blocked)),
          "invalid evidence setup triggered");
  require(!policy.observe(observation(100.6, 2U, 2U, empty_path, blocked)), "empty path triggered");
  require(policy.snapshot().fresh_blocked_observations == 0U, "empty path did not reset evidence");

  const std::vector<float> malformed{1.0F, 0.0F, 0.2F};
  require(!policy.observe(observation(101.0, 3U, 3U, active_path, malformed)),
          "malformed xyzh triggered");
  require(policy.snapshot().fresh_blocked_observations == 0U,
          "malformed xyzh did not reset evidence");

  require(!policy.observe(observation(101.5, 4U, 4U, active_path, blocked, goalIdentity(), 3U,
                                      {0.0, 0.0, 0.0}, false)),
          "inactive external goal triggered");
  require(!policy.snapshot().goal.has_value(), "inactive external goal retained binding");
}

void testInadmissibleGoalPlanStateDoesNotConsumeOneShotTrigger() {
  const auto active_path = path();
  const auto blocked = staticBlockage();
  struct GateCase {
    bool GoalPlanSnapshot::*blocked_state;
  };
  constexpr std::array<GateCase, 3> cases{{
      {&GoalPlanSnapshot::busy},
      {&GoalPlanSnapshot::pending_plan_queued},
      {&GoalPlanSnapshot::active_paused},
  }};

  for (const auto &gate_case : cases) {
    ActivePathBlockagePolicy policy(config());
    GoalPlanSnapshot goal_plan;
    auto observe = [&](double now_s, std::uint64_t generation) {
      return policy.observe(observation(now_s, generation, generation, active_path, blocked,
                                        goalIdentity(), 3U, {0.0, 0.0, 0.0},
                                        goalPlanAcceptsReplanTrigger(goal_plan)));
    };

    require(!observe(110.0, 1U), "goal-plan gate setup triggered");
    require(!observe(110.6, 2U), "goal-plan gate setup triggered early");

    goal_plan.*(gate_case.blocked_state) = true;
    require(!observe(111.2, 3U), "inadmissible goal plan emitted a persistent blockage trigger");
    require(!policy.snapshot().goal.has_value() &&
                policy.snapshot().fresh_blocked_observations == 0U &&
                !policy.snapshot().trigger_emitted,
            "inadmissible goal plan did not reset pending blockage evidence");

    goal_plan.*(gate_case.blocked_state) = false;
    require(!observe(112.0, 4U), "re-admitted blockage inherited prior evidence");
    require(!observe(112.5, 5U), "re-admitted blockage triggered before persistence threshold");
    require(observe(113.0, 6U).has_value(),
            "re-admitted blockage did not emit a fresh persistent trigger");
  }
}

void testConfigValidation() {
  auto expect_invalid = [](ActivePathBlockagePolicyConfig invalid, const char *message) {
    bool threw = false;
    try {
      ActivePathBlockagePolicy policy(invalid);
      (void)policy;
    } catch (const std::invalid_argument &) {
      threw = true;
    }
    require(threw, message);
  };

  auto value = config();
  value.persistence_s = 0.0;
  expect_invalid(value, "zero persistence was accepted");
  value = config();
  value.minimum_fresh_observations = 1U;
  expect_invalid(value, "single-observation trigger was accepted");
  value = config();
  value.overlay_radius_m = std::numeric_limits<double>::infinity();
  expect_invalid(value, "infinite overlay radius was accepted");
  value = config();
  value.overlay_half_height_m = 0.0;
  expect_invalid(value, "zero overlay height was accepted");
  value = config();
  value.max_regions = 65U;
  expect_invalid(value, "planner-incompatible region count was accepted");
  value = config();
  value.minimum_obstacle_points = 0U;
  expect_invalid(value, "zero minimum obstacle points was accepted");
}

}  // namespace

int main() {
  testStaticLiveOccupancyTriggers();
  testTransientAndSparseOccupancyDoNotTrigger();
  testOnlyForwardCorridorBlocks();
  testIdentityAndFrameChangesResetEvidence();
  testStaleClearDoesNotEraseButFreshClearDoes();
  testClockAndGenerationRollbackResetEvidence();
  testOverlayIsDeterministicDeduplicatedAndBounded();
  testInactiveInvalidAndMalformedEvidenceReset();
  testInadmissibleGoalPlanStateDoesNotConsumeOneShotTrigger();
  testConfigValidation();
  return 0;
}
