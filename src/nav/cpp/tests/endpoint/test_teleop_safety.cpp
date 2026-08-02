#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "motion/teleop_safety.hpp"

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void testPureTeleopNeedsNoPoseButStillLimitsVelocity() {
  lingtu::nav::endpoint::TeleopSafetyConfig cfg;
  cfg.check_obstacle = false;
  cfg.use_traversability_cost = false;
  cfg.max_speed_mps = 0.4;
  cfg.max_yaw_rate = 1.0;

  const auto decision = lingtu::nav::endpoint::arbitrateTeleopCommand(cfg, {1.0, 0.0, 2.0}, 0.0,
                                                                      std::nullopt, {}, {}, false);

  require(decision.should_publish, "pure teleop request must produce a decision");
  require(!decision.stopped, "pure teleop must not stop only because pose is absent");
  require(decision.limited, "pure teleop must keep velocity limits");
  require(std::abs(decision.cmd.vx - 0.4) < 1e-9, "linear speed must be clamped");
  require(std::abs(decision.cmd.wz - 1.0) < 1e-9, "yaw rate must be clamped");
}

void testTeleopAvoidFailsClosedWithoutPose() {
  lingtu::nav::endpoint::TeleopSafetyConfig cfg;
  cfg.check_obstacle = true;

  const auto decision = lingtu::nav::endpoint::arbitrateTeleopCommand(cfg, {0.2, 0.0, 0.0}, 0.0,
                                                                      std::nullopt, {}, {}, false);

  require(decision.stopped, "teleop_avoid must fail closed without pose");
  require(decision.reason == "no_pose", "missing pose reason must be explicit");
}

void testSubthresholdPureTranslationPublishesAnExplicitZero() {
  lingtu::nav::endpoint::TeleopSafetyConfig cfg;
  cfg.check_obstacle = false;
  cfg.use_traversability_cost = false;
  cfg.min_motion_speed_mps = 0.03;

  const auto decision = lingtu::nav::endpoint::arbitrateTeleopCommand(cfg, {0.02, 0.0, 0.0}, 0.0,
                                                                      std::nullopt, {}, {}, false);

  require(decision.should_publish, "subthreshold command must still publish a safe output");
  require(decision.stopped, "subthreshold pure translation must be reported as stopped");
  require(decision.limited, "deadband suppression must be reported as command limiting");
  require(decision.reason == "below_min_motion", "deadband reason must be explicit");
  require(lingtu::nav::endpoint::linearSpeed(decision.cmd) == 0.0 && decision.cmd.wz == 0.0,
          "subthreshold pure translation must publish zero, not the raw nonzero request");
}

void testSubthresholdTranslationCannotBypassForwardChecksDuringYaw() {
  lingtu::nav::endpoint::TeleopSafetyConfig cfg;
  cfg.check_obstacle = false;
  cfg.use_traversability_cost = false;
  cfg.min_motion_speed_mps = 0.03;

  const auto decision = lingtu::nav::endpoint::arbitrateTeleopCommand(cfg, {0.02, 0.0, 0.20}, 0.0,
                                                                      std::nullopt, {}, {}, false);

  require(!decision.stopped, "a valid yaw request must remain active");
  require(decision.limited, "the subthreshold translation must be reported as limited");
  require(decision.cmd.vx == 0.0 && decision.cmd.vy == 0.0,
          "translation deadband must be applied independently of yaw");
  require(decision.cmd.wz == 0.20, "translation deadband must preserve the yaw request");
}

void testAutonomyFinalSafetyStopsFollowerCommandAtNearObstacle() {
  lingtu::nav::endpoint::CommandSafetyConfig cfg;
  cfg.check_obstacle = true;
  cfg.use_traversability_cost = false;
  cfg.stop_distance_m = 0.55;
  const lingtu::nav::endpoint::TraversabilityGrid grid;
  const std::optional<nav_kernel::Pose> pose = nav_kernel::Pose{};
  const std::vector<float> obstacle_xyzh{0.30F, 0.0F, 0.30F, 0.30F};

  const auto decision = lingtu::nav::endpoint::evaluateCommandSafety(
      cfg, {0.30, 0.0, 0.0}, 0.0, pose, obstacle_xyzh, grid, false);

  require(decision.stopped, "final autonomy safety must stop at a near obstacle");
  require(decision.reason == "obstacle_stop", "final stop reason must be auditable");
  require(lingtu::nav::endpoint::linearSpeed(decision.cmd) == 0.0, "stopped output must be zero");
}

void testAutonomySafetyFollowsLocalPathInsteadOfFrozenTwistArc() {
  lingtu::nav::endpoint::CommandSafetyConfig cfg;
  cfg.check_obstacle = true;
  cfg.use_traversability_cost = false;
  const std::optional<nav_kernel::Pose> pose = nav_kernel::Pose{};
  const std::vector<float> obstacle_xyzh{0.35F, 0.80F, 0.30F, 0.30F};
  const std::vector<nav_kernel::Vec3> local_path_map{
      {0.0, 0.0, 0.0},
      {0.6, 0.0, 0.0},
      {1.2, 0.0, 0.0},
  };
  const nav_kernel::Twist follower_cmd{0.397787, 0.042020, 0.785398};

  const auto frozen_twist = lingtu::nav::endpoint::evaluateCommandSafety(
      cfg, follower_cmd, 0.0, pose, obstacle_xyzh, {}, false);
  const auto path_aware = lingtu::nav::endpoint::evaluateAutonomyPathSafety(
      cfg, follower_cmd, pose, local_path_map, obstacle_xyzh, {}, false);

  require(frozen_twist.stopped || frozen_twist.slowed,
          "the regression fixture must intersect the frozen-Twist sweep");
  require(!path_aware.stopped && !path_aware.slowed,
          "autonomy safety must not invent a collision away from the actual local path");
}

void testAutonomyPathSafetyStopsAtObstacleOnLocalPath() {
  lingtu::nav::endpoint::CommandSafetyConfig cfg;
  cfg.check_obstacle = true;
  cfg.use_traversability_cost = false;
  const std::optional<nav_kernel::Pose> pose = nav_kernel::Pose{};
  const std::vector<float> obstacle_xyzh{0.30F, 0.0F, 0.30F, 0.30F};
  const std::vector<nav_kernel::Vec3> local_path_map{
      {0.0, 0.0, 0.0},
      {0.6, 0.0, 0.0},
      {1.2, 0.0, 0.0},
  };

  const auto decision = lingtu::nav::endpoint::evaluateAutonomyPathSafety(
      cfg, {0.30, 0.0, 0.0}, pose, local_path_map, obstacle_xyzh, {}, false);

  require(decision.stopped, "autonomy path safety must stop for an obstacle on the local path");
  require(decision.reason == "obstacle_stop", "path stop reason must remain auditable");
}

void testStraightMotionPreservesObstacleDistanceBands() {
  lingtu::nav::endpoint::TeleopSafetyConfig cfg;
  cfg.check_obstacle = true;
  cfg.use_traversability_cost = false;
  const std::optional<nav_kernel::Pose> pose = nav_kernel::Pose{};
  const std::vector<float> obstacle_xyzh{0.90F, 0.0F, 0.30F, 0.30F};

  const auto decision = lingtu::nav::endpoint::arbitrateTeleopCommand(
      cfg, {0.30, 0.0, 0.0}, 0.0, pose, obstacle_xyzh, {}, false);

  require(decision.slowed && !decision.stopped,
          "straight motion must slow in the configured slow band");
  require(decision.reason == "obstacle_slow", "straight obstacle reason must remain explicit");
  require(std::abs(decision.obstacle_distance_m - 0.90) < 0.06,
          "straight obstacle distance must retain centerline semantics");
}

void testMixedTurnDetectsObstacleInsideSweptFootprint() {
  lingtu::nav::endpoint::TeleopSafetyConfig cfg;
  cfg.check_obstacle = true;
  cfg.use_traversability_cost = false;
  const std::optional<nav_kernel::Pose> pose = nav_kernel::Pose{};
  const std::vector<float> obstacle_xyzh{0.40F, 0.65F, 0.30F, 0.30F};

  const auto decision = lingtu::nav::endpoint::arbitrateTeleopCommand(
      cfg, {0.30, 0.0, 1.0}, 0.0, pose, obstacle_xyzh, {}, false);

  require(decision.slowed, "a left-turning command must see an obstacle swept by its left flank");
  require(!decision.stopped, "an obstacle beyond the stop band should slow the mixed turn");
  require(decision.reason == "obstacle_slow", "mixed-turn obstacle reason must be auditable");
  require(decision.obstacle_distance_m > cfg.stop_distance_m,
          "mixed-turn obstacle should remain in the slow band");
}

void testLateralMotionUsesLateralSweptFootprint() {
  lingtu::nav::endpoint::TeleopSafetyConfig cfg;
  cfg.check_obstacle = true;
  cfg.use_traversability_cost = false;
  const std::optional<nav_kernel::Pose> pose = nav_kernel::Pose{};
  const std::vector<float> obstacle_xyzh{0.0F, 0.90F, 0.30F, 0.30F};

  const auto decision = lingtu::nav::endpoint::arbitrateTeleopCommand(
      cfg, {0.0, 0.30, 0.0}, 0.0, pose, obstacle_xyzh, {}, false);

  require(decision.slowed && !decision.stopped, "left strafe must slow for a left-side obstacle");
  require(decision.reason == "obstacle_slow", "lateral obstacle reason must be auditable");
  require(std::abs(decision.obstacle_distance_m - 0.90) < 0.06,
          "lateral path distance must remain meaningful");
}

void testPureTurnChecksRotatingFootprintCorners() {
  lingtu::nav::endpoint::TeleopSafetyConfig cfg;
  cfg.check_obstacle = true;
  cfg.use_traversability_cost = false;
  const std::optional<nav_kernel::Pose> pose = nav_kernel::Pose{};
  const std::vector<float> obstacle_xyzh{0.55F, 0.55F, 0.30F, 0.30F};

  const auto decision = lingtu::nav::endpoint::arbitrateTeleopCommand(
      cfg, {0.0, 0.0, 0.8}, 0.0, pose, obstacle_xyzh, {}, false);

  require(decision.stopped, "pure yaw must stop when a footprint corner sweeps an obstacle");
  require(decision.reason == "yaw_obstacle", "pure-yaw obstacle reason must remain explicit");
  require(decision.cmd.wz == 0.0, "blocked pure yaw must publish zero angular velocity");
}

lingtu::nav::endpoint::TraversabilityGrid constantGrid(float cost) {
  lingtu::nav::endpoint::TraversabilityGrid grid;
  grid.rows = 20;
  grid.cols = 20;
  grid.resolution = 0.20;
  grid.origin_x = -1.0;
  grid.origin_y = -1.0;
  grid.values.assign(static_cast<std::size_t>(grid.rows * grid.cols), cost);
  return grid;
}

lingtu::nav::endpoint::TraversabilityGrid gridWithCostAt(double map_x, double map_y, float cost) {
  lingtu::nav::endpoint::TraversabilityGrid grid;
  grid.rows = 80;
  grid.cols = 80;
  grid.resolution = 0.05;
  grid.origin_x = -1.0;
  grid.origin_y = -1.0;
  grid.values.assign(static_cast<std::size_t>(grid.rows * grid.cols), 0.0F);
  const int col = static_cast<int>(std::floor((map_x - grid.origin_x) / grid.resolution));
  const int row = static_cast<int>(std::floor((map_y - grid.origin_y) / grid.resolution));
  grid.values[static_cast<std::size_t>(row * grid.cols + col)] = cost;
  return grid;
}

void testPureTurnStopsForHardTerrainSweptByFootprintCorner() {
  lingtu::nav::endpoint::TeleopSafetyConfig cfg;
  cfg.check_obstacle = false;
  cfg.use_traversability_cost = true;
  const std::optional<nav_kernel::Pose> pose = nav_kernel::Pose{};

  const auto decision = lingtu::nav::endpoint::arbitrateTeleopCommand(
      cfg, {0.0, 0.0, 0.8}, 0.0, pose, {}, gridWithCostAt(0.15, 0.72, 100.0F), true);

  require(decision.stopped,
          "pure yaw must stop for hard terrain swept by an intermediate footprint corner");
  require(decision.reason == "terrain_stop", "hard yaw terrain reason must be stable");
  require(decision.cmd.wz == 0.0, "hard yaw terrain must publish zero angular velocity");
  require(decision.traversability_cost == 100.0, "blocked yaw terrain cost must be auditable");
}

void testPureTurnFailsClosedForUnusableTerrainEvidence() {
  lingtu::nav::endpoint::TeleopSafetyConfig cfg;
  cfg.check_obstacle = false;
  cfg.use_traversability_cost = true;
  const std::optional<nav_kernel::Pose> pose = nav_kernel::Pose{};
  const nav_kernel::Twist yaw_request{0.0, 0.0, 0.8};

  const auto unavailable =
      lingtu::nav::endpoint::arbitrateTeleopCommand(cfg, yaw_request, 0.0, pose, {}, {}, true);
  require(unavailable.stopped, "pure yaw must stop when terrain grid is unavailable");
  require(unavailable.reason == "terrain_unavailable",
          "unavailable yaw terrain reason must be stable");

  lingtu::nav::endpoint::TraversabilityGrid undersized;
  undersized.rows = 4;
  undersized.cols = 4;
  undersized.resolution = 0.20;
  undersized.origin_x = -0.40;
  undersized.origin_y = -0.40;
  undersized.values.assign(16, 0.0F);
  const auto out_of_bounds = lingtu::nav::endpoint::arbitrateTeleopCommand(
      cfg, yaw_request, 0.0, pose, {}, undersized, true);
  require(out_of_bounds.stopped, "pure yaw must stop when its footprint leaves terrain coverage");
  require(out_of_bounds.reason == "terrain_out_of_bounds",
          "out-of-bounds yaw terrain reason must be stable");

  auto malformed = constantGrid(0.0F);
  malformed.values.pop_back();
  const auto invalid_shape = lingtu::nav::endpoint::arbitrateTeleopCommand(
      cfg, yaw_request, 0.0, pose, {}, malformed, true);
  require(invalid_shape.stopped, "pure yaw must stop for inconsistent terrain dimensions");
  require(invalid_shape.reason == "terrain_invalid", "malformed yaw terrain reason must be stable");

  const auto non_finite = lingtu::nav::endpoint::arbitrateTeleopCommand(
      cfg, yaw_request, 0.0, pose, {}, constantGrid(std::numeric_limits<float>::quiet_NaN()), true);
  require(non_finite.stopped, "pure yaw must stop for non-finite terrain cost");
  require(non_finite.reason == "terrain_invalid", "non-finite yaw terrain reason must be stable");

  const auto invalid_cost = lingtu::nav::endpoint::arbitrateTeleopCommand(
      cfg, yaw_request, 0.0, pose, {}, constantGrid(-1.0F), true);
  require(invalid_cost.stopped, "pure yaw must stop for terrain cost outside [0, 100]");
  require(invalid_cost.reason == "terrain_invalid",
          "out-of-range yaw terrain reason must be stable");

  const auto safe = lingtu::nav::endpoint::arbitrateTeleopCommand(cfg, yaw_request, 0.0, pose, {},
                                                                  constantGrid(0.0F), true);
  require(!safe.stopped, "fully covered free terrain must allow pure yaw");
  require(safe.reason == "accepted", "safe pure-yaw terrain reason must remain accepted");
  require(safe.cmd.wz == yaw_request.wz, "safe pure yaw must preserve angular velocity");
}

void testTranslationFailsClosedWhenTerrainLookupLeavesCoverage() {
  lingtu::nav::endpoint::TeleopSafetyConfig cfg;
  cfg.check_obstacle = false;
  cfg.use_traversability_cost = true;
  const std::optional<nav_kernel::Pose> pose = nav_kernel::Pose{};
  lingtu::nav::endpoint::TraversabilityGrid grid;
  grid.rows = 4;
  grid.cols = 4;
  grid.resolution = 0.20;
  grid.origin_x = -0.40;
  grid.origin_y = -0.40;
  grid.values.assign(16, 0.0F);

  const auto decision = lingtu::nav::endpoint::arbitrateTeleopCommand(cfg, {0.30, 0.0, 0.0}, 0.0,
                                                                      pose, {}, grid, true);

  require(decision.stopped, "translation must stop when a terrain lookup leaves coverage");
  require(decision.reason == "terrain_out_of_bounds",
          "translation lookup failure reason must remain explicit");
  require(lingtu::nav::endpoint::linearSpeed(decision.cmd) == 0.0,
          "failed translation terrain lookup must publish zero");
}

void testTeleopAvoidSlowsForSoftTerrainCost() {
  lingtu::nav::endpoint::TeleopSafetyConfig cfg;
  cfg.check_obstacle = true;
  cfg.use_traversability_cost = true;
  cfg.linear_slow_scale = 0.35;
  const std::optional<nav_kernel::Pose> pose = nav_kernel::Pose{};

  const auto decision = lingtu::nav::endpoint::arbitrateTeleopCommand(
      cfg, {0.30, 0.0, 0.0}, 0.0, pose, {}, constantGrid(50.0F), true);

  require(decision.slowed && !decision.stopped, "soft terrain must slow without stopping");
  require(decision.reason == "terrain_slow", "soft terrain reason must be explicit");
  require(std::abs(decision.cmd.vx - 0.105) < 1e-9,
          "soft terrain must apply the configured linear scale");
  require(decision.traversability_cost == 50.0, "sampled terrain cost must be auditable");
}

void testTeleopAvoidStopsForHardTerrainCost() {
  lingtu::nav::endpoint::TeleopSafetyConfig cfg;
  cfg.check_obstacle = true;
  cfg.use_traversability_cost = true;
  const std::optional<nav_kernel::Pose> pose = nav_kernel::Pose{};

  const auto decision = lingtu::nav::endpoint::arbitrateTeleopCommand(
      cfg, {0.30, 0.0, 0.0}, 0.0, pose, {}, constantGrid(100.0F), true);

  require(decision.stopped, "hard terrain must stop teleoperation");
  require(decision.reason == "terrain_stop", "hard terrain reason must be explicit");
  require(lingtu::nav::endpoint::linearSpeed(decision.cmd) == 0.0,
          "hard terrain output must be zero");
}

void testHardTerrainOutsideStopBandSlowsBeforeStopping() {
  lingtu::nav::endpoint::TeleopSafetyConfig cfg;
  cfg.check_obstacle = true;
  cfg.use_traversability_cost = true;
  cfg.stop_distance_m = 0.55;
  cfg.slow_distance_m = 1.20;
  cfg.linear_slow_scale = 0.35;
  const std::optional<nav_kernel::Pose> pose = nav_kernel::Pose{};

  const auto decision = lingtu::nav::endpoint::arbitrateTeleopCommand(
      cfg, {0.30, 0.0, 0.0}, 0.0, pose, {}, gridWithCostAt(0.75, 0.0, 100.0F), true);

  require(decision.slowed && !decision.stopped,
          "hard terrain in the outer warning band must slow before entering the stop band");
  require(decision.reason == "terrain_slow", "outer hard terrain must report terrain slow");
  require(std::abs(decision.cmd.vx - 0.105) < 1e-9,
          "outer hard terrain must apply the configured slow scale");
}

void testMixedTurnSamplesTerrainAlongArc() {
  lingtu::nav::endpoint::TeleopSafetyConfig cfg;
  cfg.check_obstacle = true;
  cfg.use_traversability_cost = true;
  const std::optional<nav_kernel::Pose> pose = nav_kernel::Pose{};

  const auto decision = lingtu::nav::endpoint::arbitrateTeleopCommand(
      cfg, {0.30, 0.0, 1.0}, 0.0, pose, {}, gridWithCostAt(0.29, 0.23, 100.0F), true);

  require(decision.stopped, "mixed turn must stop for hard terrain on its curved path");
  require(decision.reason == "terrain_stop", "curved terrain stop reason must be explicit");
  require(decision.traversability_cost == 100.0, "curved-path terrain cost must be auditable");
}

void testTeleopAvoidFailsClosedForStaleTerrainCost() {
  lingtu::nav::endpoint::TeleopSafetyConfig cfg;
  cfg.check_obstacle = true;
  cfg.use_traversability_cost = true;
  const std::optional<nav_kernel::Pose> pose = nav_kernel::Pose{};

  const auto decision = lingtu::nav::endpoint::arbitrateTeleopCommand(
      cfg, {0.30, 0.0, 0.0}, 0.0, pose, {}, constantGrid(0.0F), false);

  require(decision.stopped, "stale required terrain must fail closed inside command safety");
  require(decision.reason == "terrain_stale", "stale terrain reason must be explicit");
  require(lingtu::nav::endpoint::linearSpeed(decision.cmd) == 0.0,
          "stale terrain output must be zero");
}

void testSafetySlowCannotReintroduceSubthresholdTranslation() {
  lingtu::nav::endpoint::TeleopSafetyConfig cfg;
  cfg.check_obstacle = true;
  cfg.use_traversability_cost = true;
  cfg.linear_slow_scale = 0.35;
  cfg.min_motion_speed_mps = 0.03;
  const std::optional<nav_kernel::Pose> pose = nav_kernel::Pose{};

  const auto decision = lingtu::nav::endpoint::arbitrateTeleopCommand(
      cfg, {0.08, 0.0, 0.0}, 0.0, pose, {}, constantGrid(50.0F), true);

  require(decision.stopped, "a safety-scaled subthreshold translation must stop");
  require(!decision.slowed, "a zeroed safety output must not still be reported as moving slowly");
  require(decision.reason == "terrain_slow_below_min_motion",
          "post-safety deadband reason must preserve the terrain source");
  require(lingtu::nav::endpoint::linearSpeed(decision.cmd) == 0.0,
          "safety scaling must not leak a subthreshold nonzero translation");
}

}  // namespace

int main() {
  testPureTeleopNeedsNoPoseButStillLimitsVelocity();
  testTeleopAvoidFailsClosedWithoutPose();
  testSubthresholdPureTranslationPublishesAnExplicitZero();
  testSubthresholdTranslationCannotBypassForwardChecksDuringYaw();
  testAutonomyFinalSafetyStopsFollowerCommandAtNearObstacle();
  testAutonomySafetyFollowsLocalPathInsteadOfFrozenTwistArc();
  testAutonomyPathSafetyStopsAtObstacleOnLocalPath();
  testStraightMotionPreservesObstacleDistanceBands();
  testMixedTurnDetectsObstacleInsideSweptFootprint();
  testLateralMotionUsesLateralSweptFootprint();
  testPureTurnChecksRotatingFootprintCorners();
  testPureTurnStopsForHardTerrainSweptByFootprintCorner();
  testPureTurnFailsClosedForUnusableTerrainEvidence();
  testTranslationFailsClosedWhenTerrainLookupLeavesCoverage();
  testTeleopAvoidSlowsForSoftTerrainCost();
  testTeleopAvoidStopsForHardTerrainCost();
  testHardTerrainOutsideStopBandSlowsBeforeStopping();
  testMixedTurnSamplesTerrainAlongArc();
  testTeleopAvoidFailsClosedForStaleTerrainCost();
  testSafetySlowCannotReintroduceSubthresholdTranslation();
  std::cout << "test_teleop_safety passed\n";
  return 0;
}
