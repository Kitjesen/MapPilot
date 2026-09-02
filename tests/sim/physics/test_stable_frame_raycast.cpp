#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "lingtu/sim/mujoco_runtime.hpp"

namespace {

void require(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

lingtu::sim::MujocoRuntime composed_runtime(std::uint64_t model_generation) {
  lingtu::sim::PhysicsScenePlan plan;
  plan.session_id = "stable-frame-raycast-test";
  plan.model_generation = model_generation;
  plan.world_model_path = LINGTU_MUJOCO_SCENE_WORLD;
  plan.robots = {
      {"robot", LINGTU_MUJOCO_STABLE_FRAME_RAYCAST, "rig_root",
       {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}},
  };
  return lingtu::sim::MujocoRuntime::from_plan(std::move(plan));
}

lingtu::sim::MujocoRuntime composed_two_robot_runtime(
    std::uint64_t model_generation) {
  lingtu::sim::PhysicsScenePlan plan;
  plan.session_id = "stable-frame-multi-robot-raycast-test";
  plan.model_generation = model_generation;
  plan.world_model_path = LINGTU_MUJOCO_SCENE_WORLD;
  plan.robots = {
      {"sensor_robot", LINGTU_MUJOCO_STABLE_FRAME_RAYCAST, "rig_root",
       {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}},
      {"target_robot", LINGTU_MUJOCO_STABLE_FRAME_RAYCAST, "rig_root",
       {{2.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}},
  };
  return lingtu::sim::MujocoRuntime::from_plan(std::move(plan));
}

void require_invalid_argument(const lingtu::sim::MujocoRuntime &runtime,
                              const lingtu::sim::MujocoRaycastRequest &request,
                              const std::string &message) {
  bool rejected = false;
  try {
    static_cast<void>(runtime.raycast(request));
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, message);
}

void bind_active_stamp(const lingtu::sim::MujocoRuntime &runtime,
                       lingtu::sim::MujocoRaycastRequest &request) {
  const auto &snapshot = runtime.snapshot();
  request.session_id = snapshot.session_id;
  request.model_generation = snapshot.model_generation;
  request.reset_generation = snapshot.reset_generation;
  request.sequence = snapshot.sequence;
  request.sim_time_ns = snapshot.sim_time_ns;
}

void test_site_frame_uses_current_world_transform() {
  lingtu::sim::MujocoRuntime runtime({LINGTU_MUJOCO_STABLE_FRAME_RAYCAST});

  lingtu::sim::MujocoRaycastRequest request;
  request.sensor_frame_id = "site_sensor_frame";
  bind_active_stamp(runtime, request);
  request.rays = {{{0.0, 0.0, -1.0}, 0}};

  const auto frame = runtime.raycast(request);
  require(frame.model_generation == 0,
          "raycast must report the ModelDescriptor generation it used");
  require(frame.hits.size() == 1,
          "the site-frame downward ray must hit the floor once");
  require(std::abs(frame.hits.front().xyz_sensor[0]) < 1e-6F &&
              std::abs(frame.hits.front().xyz_sensor[1]) < 1e-6F &&
              std::abs(frame.hits.front().xyz_sensor[2] + 1.0F) < 1e-6F,
          "raycast must use the site's current world origin");
}

void test_body_frame_uses_current_world_transform() {
  lingtu::sim::MujocoRuntime runtime({LINGTU_MUJOCO_STABLE_FRAME_RAYCAST});

  lingtu::sim::MujocoRaycastRequest request;
  request.sensor_frame_id = "body_sensor_frame";
  bind_active_stamp(runtime, request);
  request.rays = {{{1.0, 0.0, 0.0}, 0}};

  const auto frame = runtime.raycast(request);
  require(frame.hits.size() == 1,
          "the rotated body-frame ray must hit the floor once");
  require(std::abs(frame.hits.front().xyz_sensor[0] - 1.0F) < 1e-6F &&
              std::abs(frame.hits.front().xyz_sensor[1]) < 1e-6F &&
              std::abs(frame.hits.front().xyz_sensor[2]) < 1e-6F,
          "raycast must use the body's current world origin and rotation");
}

void test_composed_model_accepts_only_exact_stable_frame_ids() {
  auto runtime = composed_runtime(7);

  lingtu::sim::MujocoRaycastRequest request;
  request.sensor_frame_id = "robot/site_sensor_frame";
  bind_active_stamp(runtime, request);
  request.rays = {{{0.0, 0.0, -1.0}, 0}};

  const auto frame = runtime.raycast(request);
  require(frame.model_generation == 7 && frame.hits.size() == 1,
          "a current exact stable site frame ID must resolve");

  request.sensor_frame_id = "robot__site_sensor_frame";
  require_invalid_argument(
      runtime, request,
      "a compiled MuJoCo local name must not bypass stable frame identity");

  request.sensor_frame_id = "missing/frame";
  require_invalid_argument(runtime, request,
                           "a missing stable frame ID must fail closed");
}

void test_stale_model_generation_fails_closed() {
  auto runtime = composed_runtime(11);

  lingtu::sim::MujocoRaycastRequest request;
  request.sensor_frame_id = "robot/site_sensor_frame";
  bind_active_stamp(runtime, request);
  request.model_generation = 10;
  request.rays = {{{0.0, 0.0, -1.0}, 0}};

  require_invalid_argument(
      runtime, request,
      "a stale model generation must fail closed before raycast");
}

void test_raycast_skips_only_the_sensor_robot_instance() {
  auto runtime = composed_two_robot_runtime(13);

  lingtu::sim::MujocoRaycastRequest request;
  request.sensor_frame_id = "sensor_robot/site_sensor_frame";
  bind_active_stamp(runtime, request);
  request.rays = {{{1.0, 0.0, 0.0}, 0}};

  const auto frame = runtime.raycast(request);
  require(frame.hits.size() == 1,
          "raycast must skip the sensor robot and retain another robot hit");
  require(frame.hits.front().entity_id == "target_robot" &&
              frame.hits.front().body_stable_id == "target_robot/self_occluder",
          "raycast must report the hit body stable identity; got entity=" +
              frame.hits.front().entity_id + " body=" +
              frame.hits.front().body_stable_id);
  require(std::abs(frame.hits.front().xyz_sensor[0] - 1.9F) < 1e-4F,
          "raycast must report the nearest non-self robot surface");
  require(std::abs(frame.hits.front().distance_m - 1.9) < 1e-4 &&
              std::abs(frame.hits.front().origin_world_m[0]) < 1e-6 &&
              std::abs(frame.hits.front().direction_world[0] - 1.0) < 1e-6 &&
              std::abs(frame.hits.front().position_world_m[0] - 1.9) < 1e-4,
          "raycast must report finite world-space query and hit geometry");
}

void test_ambiguous_body_and_site_frame_fails_closed() {
  lingtu::sim::MujocoRuntime runtime({LINGTU_MUJOCO_STABLE_FRAME_RAYCAST});

  lingtu::sim::MujocoRaycastRequest request;
  request.sensor_frame_id = "ambiguous_sensor_frame";
  bind_active_stamp(runtime, request);
  request.rays = {{{0.0, 0.0, -1.0}, 0}};

  require_invalid_argument(
      runtime, request,
      "a frame ID shared by a body and site must fail closed as ambiguous");
}
}  // namespace

int main() {
  try {
    test_site_frame_uses_current_world_transform();
    test_body_frame_uses_current_world_transform();
    test_composed_model_accepts_only_exact_stable_frame_ids();
    test_stale_model_generation_fails_closed();
    test_raycast_skips_only_the_sensor_robot_instance();
    test_ambiguous_body_and_site_frame_fails_closed();
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return EXIT_FAILURE;
  }

  std::cout << "PASS: stable-frame raycast\n";
  return EXIT_SUCCESS;
}
