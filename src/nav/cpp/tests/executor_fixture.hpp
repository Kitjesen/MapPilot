#pragma once

#include "navigation/executor.hpp"

namespace lingtu::nav::test {

// Test-only call builders keep behavior-focused cases compact while exercising
// Executor's single Route value and ExecutionInput API.
class ExecutorHarness : public navigation::Executor {
 public:
  using navigation::Executor::Executor;
  using navigation::Executor::setRoute;
  using navigation::Executor::tick;

  void setRoute(const std::vector<nav_kernel::Vec3> &points,
                std::optional<double> final_yaw = std::nullopt,
                std::optional<double> goal_tolerance_m = std::nullopt,
                std::optional<double> yaw_tolerance_rad = std::nullopt) {
    navigation::Executor::setRoute(
        navigation::Route{points, final_yaw, goal_tolerance_m, yaw_tolerance_rad});
  }

  void clearRoute() { clear(); }

  navigation::ExecutionOutput tick(
      const nav_kernel::Pose &body, const float *obstacles, int obstacle_count,
      double timestamp_s, navigation::TraversabilityGridView traversability = {},
      navigation::ExecutionObservation observation = {}) {
    return navigation::Executor::tick(navigation::ExecutionInput{
        navigation::ExecutionMode::Route, body, body, {}, obstacles, obstacle_count,
        timestamp_s, traversability, observation, {}});
  }

  navigation::ExecutionOutput tickOdom(
      const nav_kernel::Pose &map_body, const nav_kernel::Pose &odom_body,
      const navigation::MapFromOdomTransform &map_from_odom,
      const float *obstacles, int obstacle_count, double timestamp_s,
      navigation::TraversabilityGridView traversability = {},
      navigation::ExecutionObservation observation = {}) {
    return navigation::Executor::tick(navigation::ExecutionInput{
        navigation::ExecutionMode::Route, map_body, odom_body, map_from_odom,
        obstacles, obstacle_count, timestamp_s, traversability, observation, {}});
  }

  navigation::ExecutionOutput tickIntent(
      const nav_kernel::Pose &body, const nav_kernel::Twist &intent,
      const float *obstacles, int obstacle_count, double timestamp_s,
      navigation::TraversabilityGridView traversability = {},
      navigation::ExecutionObservation observation = {}) {
    return navigation::Executor::tick(navigation::ExecutionInput{
        navigation::ExecutionMode::MotionIntent, body, body, {}, obstacles,
        obstacle_count, timestamp_s, traversability, observation, intent});
  }
};

}  // namespace lingtu::nav::test
