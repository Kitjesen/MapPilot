#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include "lingtu/sim/physics_scene_composer.hpp"
#include "lingtu/sim/mujoco_runtime.hpp"

namespace {

void require(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void test_load_and_advance() {
  lingtu::sim::MujocoRuntime runtime({LINGTU_MUJOCO_TEST_MODEL});

  const auto &initial = runtime.snapshot();
  require(initial.sim_time_ns == 0, "initial simulation time must be zero");
  require(initial.model_generation == 0, "initial model generation must be zero");
  require(initial.bodies.size() == 1, "world body must not be exposed");
  require(initial.bodies.front().name == "test_body", "body name must be stable");
  const double initial_z = initial.bodies.front().position_m[2];

  const auto &advanced = runtime.advance(5);
  require(advanced.sequence == 1, "one published advance must increment sequence");
  require(advanced.physics_step == 5, "physics step count must include all substeps");
  require(advanced.sim_time_ns == 10'000'000, "five 2 ms steps must reach 10 ms");
  require(advanced.bodies.front().position_m[2] < initial_z,
          "gravity must change the physical body state");
  require(advanced.bodies.front().linear_velocity_mps[2] < 0.0,
          "published body state must include MuJoCo world-frame linear velocity");
  require(std::all_of(
              advanced.bodies.front().angular_velocity_rps.begin(),
              advanced.bodies.front().angular_velocity_rps.end(),
              [](double value) { return std::isfinite(value); }),
          "published body angular velocity must be finite");
}

void test_pause_preserves_the_published_state() {
  lingtu::sim::MujocoRuntime runtime({LINGTU_MUJOCO_TEST_MODEL});
  const auto &before_pause = runtime.advance(2);
  const auto sequence = before_pause.sequence;
  const auto physics_step = before_pause.physics_step;
  const auto sim_time_ns = before_pause.sim_time_ns;
  const auto position = before_pause.bodies.front().position_m;

  runtime.set_paused(true);
  require(runtime.paused(), "runtime must report its paused state");
  const auto &while_paused = runtime.advance(20);

  require(while_paused.sequence == sequence, "paused advance must not publish a new sequence");
  require(while_paused.physics_step == physics_step,
          "paused advance must not consume physics steps");
  require(while_paused.sim_time_ns == sim_time_ns,
          "paused advance must not change simulation time");
  require(while_paused.bodies.front().position_m == position,
          "paused advance must not change body state");

  runtime.set_paused(false);
  require(!runtime.paused(), "runtime must resume when pause is cleared");
  require(runtime.advance().physics_step == physics_step + 1,
          "resumed runtime must continue from the paused state");
}

void test_reset_starts_a_new_generation() {
  lingtu::sim::MujocoRuntime runtime({LINGTU_MUJOCO_TEST_MODEL});
  const double initial_z = runtime.snapshot().bodies.front().position_m[2];
  runtime.advance(10);

  const auto &reset = runtime.reset();
  require(reset.reset_generation == 1, "reset must increment the reset generation");
  require(reset.sequence == 0, "reset must restart the publication sequence");
  require(reset.physics_step == 0, "reset must restart the physics step count");
  require(reset.sim_time_ns == 0, "reset must restore simulation time to zero");
  require(std::abs(reset.bodies.front().position_m[2] - initial_z) < 1e-12,
          "reset must restore the model reference pose");
}

void test_mid360_raycast_uses_current_mujoco_model_and_data() {
  lingtu::sim::MujocoRuntime runtime({LINGTU_MUJOCO_MID360_RAYCAST});

  lingtu::sim::MujocoRaycastRequest request;
  request.sensor_frame_id = "lidar_site";
  request.session_id = runtime.snapshot().session_id;
  request.model_generation = runtime.snapshot().model_generation;
  request.reset_generation = runtime.snapshot().reset_generation;
  request.sequence = runtime.snapshot().sequence;
  request.sim_time_ns = runtime.snapshot().sim_time_ns;
  request.range_min_m = 0.1;
  request.range_max_m = 40.0;
  request.reflectivity_proxy = 15;
  request.unknown_line = 0;
  request.rays = {
      {{0.0, 0.0, -1.0}, 0},
      {{1.0, 0.0, 0.0}, 2'000'000},
  };

  const auto frame = runtime.raycast(request);
  require(frame.sim_time_ns == runtime.snapshot().sim_time_ns,
          "raycast frame must use the authoritative MuJoCo simulation clock");
  require(frame.hits.size() == 1,
          "only the downward ray should hit the real MuJoCo floor geometry");
  require(std::abs(frame.hits.front().xyz_sensor[0]) < 1e-6F &&
              std::abs(frame.hits.front().xyz_sensor[1]) < 1e-6F &&
              std::abs(frame.hits.front().xyz_sensor[2] + 1.0F) < 1e-6F,
          "raycast point must be expressed in the sensor frame");
  require(frame.hits.front().offset_time_ns == 0,
          "raycast point must preserve the scheduled point offset");
  require(frame.hits.front().reflectivity == 15 && frame.hits.front().tag == 0 &&
              frame.hits.front().line == 0,
          "raycast metadata must use the documented conservative proxy fields");
}

void test_thunder_model_snapshot_contract() {
  lingtu::sim::MujocoRuntime runtime({LINGTU_MUJOCO_THUNDER_MODEL});

  const auto &initial = runtime.snapshot();
  require(initial.bodies.size() == 21, "ThunderV4 must expose every non-world body");

  std::set<std::string> body_names;
  for (std::size_t index = 0; index < initial.bodies.size(); ++index) {
    const auto &pose = initial.bodies[index];
    require(pose.body_id == static_cast<std::int32_t>(index + 1),
            "ThunderV4 body IDs must preserve MuJoCo ordering");
    require(!pose.name.empty(), "every ThunderV4 body must have a stable name");
    require(body_names.insert(pose.name).second, "ThunderV4 body names must be unique");

    double quaternion_norm_squared = 0.0;
    for (const double value : pose.position_m) {
      require(std::isfinite(value), "every ThunderV4 body position must be finite");
    }
    for (const double value : pose.quaternion_wxyz) {
      require(std::isfinite(value), "every ThunderV4 body quaternion must be finite");
      quaternion_norm_squared += value * value;
    }
    for (const double value : pose.linear_velocity_mps) {
      require(std::isfinite(value), "every ThunderV4 body linear velocity must be finite");
    }
    for (const double value : pose.angular_velocity_rps) {
      require(std::isfinite(value), "every ThunderV4 body angular velocity must be finite");
    }
    require(std::abs(quaternion_norm_squared - 1.0) < 1e-9,
            "every ThunderV4 body quaternion must be normalized");
  }

  const auto &advanced = runtime.advance(10);
  require(advanced.physics_step == 10, "ThunderV4 must advance with the fixed MuJoCo step");
  const auto sim_time_ns = advanced.sim_time_ns;

  runtime.set_paused(true);
  require(runtime.advance(10).sim_time_ns == sim_time_ns,
          "paused ThunderV4 must preserve simulation time");
  runtime.set_paused(false);
  require(runtime.advance().physics_step == 11, "ThunderV4 must resume after pause");

  const auto &reset = runtime.reset();
  require(reset.reset_generation == 1, "ThunderV4 reset must start a new generation");
  require(reset.physics_step == 0 && reset.sim_time_ns == 0,
          "ThunderV4 reset must restore the clock origin");
}

void test_named_keyframe_initializes_thunder_nominal_stand() {
  lingtu::sim::MujocoRuntime runtime(
      {LINGTU_MUJOCO_THUNDER_MODEL, "v4_nominal_stand"});

  const auto &initial = runtime.snapshot();
  const auto base = std::find_if(
      initial.bodies.begin(), initial.bodies.end(),
      [](const lingtu::sim::BodyPose &body) { return body.name == "base_link"; });

  require(base != initial.bodies.end(), "ThunderV4 nominal snapshot must expose base_link");
  require(std::abs(base->position_m[2] - 0.6) < 1e-12,
          "named keyframe must initialize the ThunderV4 base at its nominal stand height");

  runtime.advance(10);
  const auto &reset = runtime.reset();
  const auto reset_base = std::find_if(
      reset.bodies.begin(), reset.bodies.end(),
      [](const lingtu::sim::BodyPose &body) { return body.name == "base_link"; });
  require(reset_base != reset.bodies.end(), "reset nominal snapshot must expose base_link");
  require(std::abs(reset_base->position_m[2] - 0.6) < 1e-12,
          "reset must restore the configured named keyframe");
}

void test_scene_composer_uses_one_model_for_multiple_instances() {
  lingtu::sim::PhysicsScenePlan plan;
  plan.session_id = "11111111111111111111111111111111";
  plan.model_generation = 7;
  plan.world_model_path = LINGTU_MUJOCO_SCENE_WORLD;
  plan.robots = {
      {"one", LINGTU_MUJOCO_SCENE_MODEL, "test_body", {{0.0, 0.0, 1.0}, {1.0, 0.0, 0.0, 0.0}}},
      {"two", LINGTU_MUJOCO_SCENE_MODEL, "test_body", {{1.0, 0.0, 1.0}, {1.0, 0.0, 0.0, 0.0}}},
  };

  lingtu::sim::PhysicsSceneComposer composer(std::move(plan));
  require(composer.model() != nullptr, "scene composer must produce a model");
  require(composer.model()->nbody == 3, "two robot instances must share one MuJoCo model");

  const auto &descriptor = composer.descriptor();
  require(descriptor.session_id == "11111111111111111111111111111111",
          "descriptor must retain product session id");
  require(descriptor.model_generation == 7, "descriptor must retain model generation");
  require(descriptor.instances.size() == 2, "descriptor must expose both robot instances");
  require(descriptor.instances[0].root_body_index != descriptor.instances[1].root_body_index,
          "robot instance roots must have distinct dense indices");
  require(mj_name2id(composer.model(), mjOBJ_BODY, "one__test_body") >= 0,
          "first robot namespace must be present");
  require(mj_name2id(composer.model(), mjOBJ_BODY, "two__test_body") >= 0,
          "second robot namespace must be present");
  const auto one_clock = std::find_if(
      descriptor.sensors.begin(), descriptor.sensors.end(),
      [](const lingtu::sim::SensorDescriptor &sensor) {
        return sensor.stable_id == "one/sim_clock";
      });
  const auto two_clock = std::find_if(
      descriptor.sensors.begin(), descriptor.sensors.end(),
      [](const lingtu::sim::SensorDescriptor &sensor) {
        return sensor.stable_id == "two/sim_clock";
      });
  require(one_clock != descriptor.sensors.end() &&
              two_clock != descriptor.sensors.end(),
          "objectless MuJoCo sensors must remain namespaced per instance");
  require(one_clock->source_stable_id == one_clock->stable_id &&
              two_clock->source_stable_id == two_clock->stable_id,
          "objectless sensors must use their own stable identity as source");
}

void test_scene_composer_attaches_payload_to_its_robot_parent_body() {
  lingtu::sim::PhysicsScenePlan plan;
  plan.session_id = "payload-session";
  plan.model_generation = 8;
  plan.world_model_path = LINGTU_MUJOCO_SCENE_WORLD;
  plan.robots = {
      {"carrier", LINGTU_MUJOCO_SCENE_MODEL, "test_body",
       {{2.0, 0.0, 1.0}, {1.0, 0.0, 0.0, 0.0}}},
  };

  lingtu::sim::PhysicsPayloadSpec payload;
  payload.instance_id = "payload_01";
  payload.namespace_name = "payload_01";
  payload.robot_instance_id = "carrier";
  payload.package = {"fictional_rws_01", "1.0.0", "payload",
                     "sim/packages/payloads/fictional_rws_01/1.0.0/payload.package.yaml"};
  payload.parent_frame = "test_body";
  payload.parent_body = "test_body";
  payload.mount_transform = {{0.0, 0.0, 0.25}, {1.0, 0.0, 0.0, 0.0}};
  payload.model = {LINGTU_MUJOCO_ACTUATED_MODEL, "test_body"};
  payload.authority = "mujoco";
  payload.collision_representation = "primitive_proxy";
  payload.frames = {{"test_body", "payload_root", ""}};
  plan.payloads.push_back(std::move(payload));

  lingtu::sim::PhysicsSceneComposer composer(std::move(plan));
  const int robot_body =
      mj_name2id(composer.model(), mjOBJ_BODY, "carrier__test_body");
  const int payload_body =
      mj_name2id(composer.model(), mjOBJ_BODY, "payload_01__test_body");
  require(robot_body >= 0 && payload_body >= 0,
          "robot and payload roots must compile into one MuJoCo model");
  require(composer.model()->body_parentid[payload_body] == robot_body,
          "payload root must attach beneath its robot's namespaced parent body");
  require(std::abs(composer.model()->body_pos[3 * payload_body + 2] - 0.75) <
              1e-12,
          "payload mount transform must compose with its authored root pose relative to the parent body");

  const auto &descriptor = composer.descriptor();
  require(descriptor.payloads.size() == 1 &&
              descriptor.payloads[0].instance_id == "payload_01" &&
              descriptor.payloads[0].robot_instance_id == "carrier" &&
              descriptor.payloads[0].root_body_index == payload_body,
          "descriptor must identify the attached payload root and owning robot");
  const auto payload_joint = std::find_if(
      descriptor.joints.begin(), descriptor.joints.end(),
      [](const lingtu::sim::JointDescriptor &joint) {
        return joint.stable_id == "payload_01/hinge_joint";
      });
  require(payload_joint != descriptor.joints.end(),
          "payload subtree stable IDs must use payload_id/local_name");
}

void test_scene_composer_rejects_robot_owned_global_options_before_attach() {
  lingtu::sim::PhysicsScenePlan plan;
  plan.session_id =
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
  plan.model_generation = 1;
  plan.world_model_path = LINGTU_MUJOCO_OPEN_FIELD;
  plan.global_policy = {
      0.002,
      mjINT_RK4,
      mjSOL_NEWTON,
      100,
      {0.0, 0.0, -9.81},
  };
  plan.robots = {
      {"cart_01", LINGTU_MUJOCO_ROBOT_GLOBAL_OPTION, "base_link",
       {{2.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}},
  };

  try {
    lingtu::sim::PhysicsSceneComposer composer(std::move(plan));
    (void)composer;
  } catch (const std::runtime_error &error) {
    const std::string message = error.what();
    require(message.find("robot cart_01") != std::string::npos,
            "global-option diagnostic must identify the robot instance");
    for (const std::string field : {"timestep", "iterations", "solver", "gravity"}) {
      require(message.find(field) != std::string::npos,
              "global-option diagnostic must identify field " + field);
    }
    return;
  }
  throw std::runtime_error(
      "robot-owned MuJoCo global options must fail before attachment");
}

void test_runtime_advances_one_composed_session() {
  lingtu::sim::PhysicsScenePlan plan;
  plan.session_id =
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
  plan.model_generation = 7;
  plan.world_model_path = LINGTU_MUJOCO_SCENE_WORLD;
  plan.robots = {
      {"one", LINGTU_MUJOCO_SCENE_MODEL, "test_body", {{0.0, 0.0, 1.0}, {1.0, 0.0, 0.0, 0.0}}},
      {"two", LINGTU_MUJOCO_SCENE_MODEL, "test_body", {{1.0, 0.0, 1.0}, {1.0, 0.0, 0.0, 0.0}}},
  };

  auto runtime = lingtu::sim::MujocoRuntime::from_plan(std::move(plan));
  const auto &initial = runtime.snapshot();

  require(initial.session_id ==
              "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
          "session runtime snapshot must retain the resolved session id");
  require(initial.model_generation == 7,
          "session runtime snapshot must retain the composed model generation");
  require(initial.bodies.size() == 2,
          "session runtime must expose both robot bodies from one model");

  const auto one = std::find_if(
      initial.bodies.begin(), initial.bodies.end(),
      [](const lingtu::sim::BodyPose &body) { return body.stable_id == "one/test_body"; });
  const auto two = std::find_if(
      initial.bodies.begin(), initial.bodies.end(),
      [](const lingtu::sim::BodyPose &body) { return body.stable_id == "two/test_body"; });
  require(one != initial.bodies.end() && two != initial.bodies.end(),
          "session runtime must publish stable IDs rather than compiled MuJoCo names");
  require(one->instance_id == "one" && one->frame_id == "test_body",
          "first body identity must separate instance and frame");
  require(two->instance_id == "two" && two->frame_id == "test_body",
          "second body identity must separate instance and frame");

  const double initial_one_z = one->position_m[2];
  const double initial_two_z = two->position_m[2];
  const auto &advanced = runtime.advance(5);
  require(advanced.physics_step == 5 && advanced.sim_time_ns == 10'000'000,
          "composed session must advance with one shared physics clock");

  const auto advanced_one = std::find_if(
      advanced.bodies.begin(), advanced.bodies.end(),
      [](const lingtu::sim::BodyPose &body) { return body.stable_id == "one/test_body"; });
  const auto advanced_two = std::find_if(
      advanced.bodies.begin(), advanced.bodies.end(),
      [](const lingtu::sim::BodyPose &body) { return body.stable_id == "two/test_body"; });
  require(advanced_one->position_m[2] < initial_one_z &&
              advanced_two->position_m[2] < initial_two_z,
          "both robot instances must be advanced by the same mjData");
}

void test_scenario_kinematic_proxy_is_composed_and_applied_atomically() {
  constexpr const char *session_id = "scenario-session";
  lingtu::sim::PhysicsScenePlan plan;
  plan.session_id = session_id;
  plan.model_generation = 9;
  plan.world_model_path = LINGTU_MUJOCO_SCENE_WORLD;
  plan.robots = {
      {"robot_01", LINGTU_MUJOCO_SCENE_MODEL, "test_body",
       {{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}},
  };
  plan.kinematic_entities = {
      {"pedestrian_01", LINGTU_MUJOCO_KINEMATIC_PROXY, "proxy_root",
       {{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}},
  };

  auto runtime = lingtu::sim::MujocoRuntime::from_plan(std::move(plan));
  const auto initial_body = std::find_if(
      runtime.snapshot().bodies.begin(), runtime.snapshot().bodies.end(),
      [](const lingtu::sim::BodyPose &body) {
        return body.stable_id == "pedestrian_01/proxy_root";
      });
  require(initial_body != runtime.snapshot().bodies.end(),
          "kinematic proxy must be exposed through the shared ModelDescriptor");
  require(std::abs(initial_body->position_m[0] - 1.0) < 1e-12,
          "kinematic proxy must start at its compiled initial transform");
  const auto initial_robot = std::find_if(
      runtime.snapshot().bodies.begin(), runtime.snapshot().bodies.end(),
      [](const lingtu::sim::BodyPose &body) {
        return body.stable_id == "robot_01/test_body";
      });
  require(initial_robot != runtime.snapshot().bodies.end(),
          "composed robot body must be present beside the kinematic proxy");
  const auto robot_position = initial_robot->position_m;
  const auto robot_quaternion = initial_robot->quaternion_wxyz;

  lingtu::sim::KinematicPoseBatch command;
  command.session_id = session_id;
  command.model_generation = 9;
  command.reset_generation = 0;
  command.sequence = 0;
  command.sim_time_ns = 0;
  command.entities = {
      {"pedestrian_01", {2.0, 0.5, 0.0}, {1.0, 0.0, 0.0, 0.0}},
  };
  command.entities[0].body_stable_id = "pedestrian_01/proxy_root";
  command.model_generation = 8;
  require(runtime.apply_kinematic_poses(command) ==
              lingtu::sim::KinematicPoseResult::rejected_stale_model_generation,
          "an old model generation must fail closed");
  command.model_generation = 10;
  require(runtime.apply_kinematic_poses(command) ==
              lingtu::sim::KinematicPoseResult::rejected_future_model_generation,
          "a future model generation must fail closed");
  command.model_generation = 9;
  command.reset_generation = 1;
  require(runtime.apply_kinematic_poses(command) ==
              lingtu::sim::KinematicPoseResult::rejected_future_reset_generation,
          "a future reset generation must fail closed");
  command.reset_generation = 0;
  command.sim_time_ns = 1;
  require(runtime.apply_kinematic_poses(command) ==
              lingtu::sim::KinematicPoseResult::rejected_time,
          "a scenario timestamp outside the MuJoCo clock must fail closed");
  command.sim_time_ns = 0;
  require(runtime.apply_kinematic_poses(command) ==
              lingtu::sim::KinematicPoseResult::applied,
          "generation-stamped scenario pose must be accepted");
  const auto moved_body = std::find_if(
      runtime.snapshot().bodies.begin(), runtime.snapshot().bodies.end(),
      [](const lingtu::sim::BodyPose &body) {
        return body.stable_id == "pedestrian_01/proxy_root";
      });
  require(moved_body != runtime.snapshot().bodies.end() &&
              std::abs(moved_body->position_m[0] - 2.0) < 1e-12 &&
              std::abs(moved_body->position_m[1] - 0.5) < 1e-12,
          "accepted scenario pose must immediately update MuJoCo truth");
  const auto unchanged_robot = std::find_if(
      runtime.snapshot().bodies.begin(), runtime.snapshot().bodies.end(),
      [](const lingtu::sim::BodyPose &body) {
        return body.stable_id == "robot_01/test_body";
      });
  require(unchanged_robot != runtime.snapshot().bodies.end() &&
              unchanged_robot->position_m == robot_position &&
              unchanged_robot->quaternion_wxyz == robot_quaternion,
          "scenario kinematic updates must not overwrite MuJoCo-authority robot state");

  command.entities[0].position_m = {3.0, 0.0, 0.0};
  require(runtime.apply_kinematic_poses(command) ==
              lingtu::sim::KinematicPoseResult::rejected_sequence,
          "a duplicate scenario sequence must not overwrite an accepted pose");

  command.session_id = "foreign-session";
  command.entities[0].position_m = {7.0, 0.0, 0.0};
  require(runtime.apply_kinematic_poses(command) ==
              lingtu::sim::KinematicPoseResult::rejected_session,
          "foreign-session scenario state must fail closed");
  const auto rejected_body = std::find_if(
      runtime.snapshot().bodies.begin(), runtime.snapshot().bodies.end(),
      [](const lingtu::sim::BodyPose &body) {
        return body.stable_id == "pedestrian_01/proxy_root";
      });
  require(rejected_body != runtime.snapshot().bodies.end() &&
              std::abs(rejected_body->position_m[0] - 2.0) < 1e-12,
          "a rejected batch must not partially mutate MuJoCo state");

  command.session_id = session_id;
  const auto &advanced = runtime.advance();
  command.sequence = advanced.sequence;
  command.sim_time_ns = advanced.sim_time_ns;
  command.entities[0].body_stable_id = "pedestrian_01/unknown_body";
  require(runtime.apply_kinematic_poses(command) ==
              lingtu::sim::KinematicPoseResult::rejected_entity_set,
          "an unknown body stable ID must fail closed");

  const auto &reset = runtime.reset();
  const auto reset_body = std::find_if(
      reset.bodies.begin(), reset.bodies.end(),
      [](const lingtu::sim::BodyPose &body) {
        return body.stable_id == "pedestrian_01/proxy_root";
      });
  require(reset_body != reset.bodies.end() &&
              std::abs(reset_body->position_m[0] - 1.0) < 1e-12,
          "reset must restore the compiled kinematic initial transform");

  command.entities[0].body_stable_id = "pedestrian_01/proxy_root";
  command.entities[0].position_m = {-0.75, 0.0, 0.0};
  command.sequence = 0;
  command.sim_time_ns = 0;
  command.reset_generation = 0;
  require(runtime.apply_kinematic_poses(command) ==
              lingtu::sim::KinematicPoseResult::rejected_stale_reset_generation,
          "a pre-reset scenario batch must not cross the reset boundary");
  command.reset_generation = 1;
  require(runtime.apply_kinematic_poses(command) ==
              lingtu::sim::KinematicPoseResult::applied,
          "the first sequence of the new reset generation must be accepted");
  const auto &contact_step = runtime.advance(10);
  const auto contacted_robot = std::find_if(
      contact_step.bodies.begin(), contact_step.bodies.end(),
      [](const lingtu::sim::BodyPose &body) {
        return body.stable_id == "robot_01/test_body";
      });
  require(contacted_robot != contact_step.bodies.end() &&
              contacted_robot->position_m[0] < -1.0001,
          "a MuJoCo-authority robot must physically respond to the kinematic proxy contact");
}

void test_catalog_world_and_robot_packages_compile_as_one_runtime() {
  lingtu::sim::PhysicsScenePlan plan;
  plan.session_id =
      "abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789";
  plan.model_generation = 1;
  plan.world_model_path = LINGTU_MUJOCO_OPEN_FIELD;
  plan.robots = {
      {"thunder_01", LINGTU_MUJOCO_THUNDER_MODEL, "base_link",
       {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}},
      {"cart_01", LINGTU_MUJOCO_OMNI_MODEL, "base_link",
       {{2.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}},
  };

  auto runtime = lingtu::sim::MujocoRuntime::from_plan(std::move(plan));
  const auto &snapshot = runtime.snapshot();
  require(!snapshot.bodies.empty(), "catalog packages must produce runtime bodies");
  require(std::all_of(
              snapshot.bodies.begin(), snapshot.bodies.end(),
              [](const lingtu::sim::BodyPose &body) { return !body.instance_id.empty(); }),
          "world package must not inject an un-namespaced placeholder robot");
  require(std::any_of(
              snapshot.bodies.begin(), snapshot.bodies.end(),
              [](const lingtu::sim::BodyPose &body) {
                return body.stable_id == "thunder_01/base_link";
              }),
          "catalog runtime must contain the Thunder instance root");
  require(std::any_of(
              snapshot.bodies.begin(), snapshot.bodies.end(),
              [](const lingtu::sim::BodyPose &body) {
                return body.stable_id == "cart_01/base_link";
              }),
          "catalog runtime must contain the OmniCart instance root");
}

void test_composed_snapshot_excludes_static_world_bodies() {
  lingtu::sim::PhysicsScenePlan plan;
  plan.session_id =
      "static-world-0123456789abcdef0123456789abcdef0123456789abcdef";
  plan.model_generation = 7;
  plan.world_model_path = LINGTU_MUJOCO_STATIC_BODY_WORLD;
  plan.robots = {
      {"robot_01", LINGTU_MUJOCO_SCENE_MODEL, "test_body",
       {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}},
  };

  lingtu::sim::PhysicsSceneComposer composer(plan);
  require(std::any_of(
              composer.descriptor().bodies.begin(),
              composer.descriptor().bodies.end(),
              [](const lingtu::sim::BodyDescriptor &body) {
                return body.stable_id == "static_prop";
              }),
          "static world bodies must remain in the session physics descriptor");

  auto runtime = lingtu::sim::MujocoRuntime::from_plan(std::move(plan));
  const auto &snapshot = runtime.snapshot();
  require(snapshot.bodies.size() == 1,
          "the dynamic snapshot must expose only robot and scenario bodies");
  require(snapshot.bodies.front().stable_id == "robot_01/test_body" &&
              snapshot.bodies.front().instance_id == "robot_01" &&
              snapshot.bodies.front().frame_id == "test_body",
          "the composed robot body must retain its stable runtime identity");
  require(std::none_of(
              snapshot.bodies.begin(), snapshot.bodies.end(),
              [](const lingtu::sim::BodyPose &body) {
                return body.stable_id == "static_prop";
              }),
          "static world bodies must not be published as dynamic visual entities");
}

void test_composed_thunder_sensor_snapshot_uses_stable_sources() {
  lingtu::sim::PhysicsScenePlan plan;
  plan.session_id =
      "sensor-session-0123456789abcdef0123456789abcdef0123456789abcdef";
  plan.model_generation = 42;
  plan.world_model_path = LINGTU_MUJOCO_OPEN_FIELD;
  plan.robots = {
      {"thunder_01", LINGTU_MUJOCO_THUNDER_MODEL, "base_link",
       {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}},
  };

  lingtu::sim::PhysicsSceneComposer composer(plan);
  const auto &descriptors = composer.descriptor().sensors;
  const auto descriptor_for = [&descriptors](std::int32_t sensor_type) {
    return std::find_if(
        descriptors.begin(), descriptors.end(),
        [sensor_type](const lingtu::sim::SensorDescriptor &descriptor) {
          return descriptor.sensor_type == sensor_type &&
                 descriptor.source_stable_id == "thunder_01/imu";
        });
  };
  const auto framequat = descriptor_for(mjSENS_FRAMEQUAT);
  const auto gyro = descriptor_for(mjSENS_GYRO);
  const auto accelerometer = descriptor_for(mjSENS_ACCELEROMETER);
  require(framequat != descriptors.end() && framequat->width == 4,
          "Thunder composed IMU framequat must be four-dimensional");
  require(gyro != descriptors.end() && gyro->width == 3,
          "Thunder composed IMU gyro must be three-dimensional");
  require(accelerometer != descriptors.end() && accelerometer->width == 3,
          "Thunder composed IMU accelerometer must be three-dimensional");
  require(framequat->object_type == mjOBJ_SITE &&
              framequat->object_index >= 0 &&
              framequat->source_stable_id == "thunder_01/imu",
          "IMU sensors must identify their composed site source by stable ID");

  auto runtime = lingtu::sim::MujocoRuntime::from_plan(std::move(plan));
  const auto &initial = runtime.snapshot();
  const auto snapshot_for = [&initial](const std::string &type) {
    return std::find_if(
        initial.sensors.begin(), initial.sensors.end(),
        [&type](const lingtu::sim::SensorState &sensor) {
          return sensor.sensor_type == type &&
                 sensor.source_stable_id == "thunder_01/imu";
        });
  };
  const auto initial_framequat = snapshot_for("framequat");
  const auto initial_gyro = snapshot_for("gyro");
  const auto initial_accelerometer = snapshot_for("accelerometer");
  require(initial_framequat != initial.sensors.end() &&
              initial_gyro != initial.sensors.end() &&
              initial_accelerometer != initial.sensors.end(),
          "Thunder composed snapshot must expose all IMU source streams");
  for (const auto sensor : {initial_framequat, initial_gyro,
                            initial_accelerometer}) {
    require(sensor->instance_id == "thunder_01" && sensor->frame_id == "imu",
            "sensor identity must separate instance namespace and local frame");
    require(std::all_of(
                sensor->values.begin(), sensor->values.end(),
                [](double value) { return std::isfinite(value); }),
            "initial Thunder IMU sensor values must be finite");
  }

  const auto stable_id = initial_framequat->stable_id;
  const auto source_stable_id = initial_framequat->source_stable_id;
  const auto sensor_count = initial.sensors.size();
  runtime.advance(3);
  const auto &reset = runtime.reset();
  require(reset.model_generation == 42 && reset.reset_generation == 1,
          "reset must preserve model generation and increment reset generation");
  require(reset.sensors.size() == sensor_count,
          "reset must preserve the composed sensor descriptor count");
  const auto reset_framequat = std::find_if(
      reset.sensors.begin(), reset.sensors.end(),
      [](const lingtu::sim::SensorState &sensor) {
        return sensor.sensor_type == "framequat" &&
               sensor.source_stable_id == "thunder_01/imu";
      });
  require(reset_framequat != reset.sensors.end() &&
              reset_framequat->stable_id == stable_id &&
              reset_framequat->source_stable_id == source_stable_id &&
              reset_framequat->instance_id == "thunder_01" &&
              reset_framequat->frame_id == "imu" &&
              reset_framequat->values.size() == 4,
          "reset must not rebuild or corrupt stable sensor descriptors");
}

void test_composed_runtime_applies_package_keyframe_and_spawn() {
  lingtu::sim::PhysicsScenePlan plan;
  plan.session_id =
      "123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0";
  plan.model_generation = 3;
  plan.world_model_path = LINGTU_MUJOCO_OPEN_FIELD;
  plan.robots = {
      {"thunder_01", LINGTU_MUJOCO_THUNDER_MODEL, "base_link",
       {{2.0, 3.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}, "v4_nominal_stand"},
  };

  auto runtime = lingtu::sim::MujocoRuntime::from_plan(std::move(plan));
  const auto find_base = [](const lingtu::sim::SimulationSnapshot &snapshot) {
    return std::find_if(
        snapshot.bodies.begin(), snapshot.bodies.end(),
        [](const lingtu::sim::BodyPose &body) {
          return body.stable_id == "thunder_01/base_link";
        });
  };
  const auto &initial = runtime.snapshot();
  const auto base = find_base(initial);
  require(base != initial.bodies.end(),
          "composed nominal state must expose the robot root body");
  require(std::abs(base->position_m[0] - 2.0) < 1e-12 &&
              std::abs(base->position_m[1] - 3.0) < 1e-12 &&
              std::abs(base->position_m[2] - 0.6) < 1e-12,
          "per-instance keyframe must compose with the session spawn transform");

  runtime.advance(10);
  const auto &reset = runtime.reset();
  const auto reset_base = find_base(reset);
  require(reset_base != reset.bodies.end() &&
              std::abs(reset_base->position_m[0] - 2.0) < 1e-12 &&
              std::abs(reset_base->position_m[1] - 3.0) < 1e-12 &&
              std::abs(reset_base->position_m[2] - 0.6) < 1e-12,
          "reset must restore the composed package keyframe and spawn");
}

lingtu::sim::CommandEnvelope actuator_command(
    const std::string &source_id,
    const std::string &instance_id,
    std::uint64_t sequence,
    lingtu::sim::GenerationStamp generation,
    std::uint64_t apply_time_ns,
    double value);

lingtu::sim::CommandEnvelope actuator_command(
    std::uint64_t sequence,
    lingtu::sim::GenerationStamp generation,
    std::uint64_t apply_time_ns,
    double value) {
  return actuator_command("test_controller", "robot", sequence, generation,
                          apply_time_ns, value);
}

lingtu::sim::CommandEnvelope actuator_command(
    const std::string &source_id,
    const std::string &instance_id,
    std::uint64_t sequence,
    lingtu::sim::GenerationStamp generation,
    std::uint64_t apply_time_ns,
    double value) {
  lingtu::sim::CommandEnvelope command;
  require(command.session_id.assign("actuator-session"),
          "session id must fit");
  require(command.source_id.assign(source_id), "controller ID must fit");
  require(command.instance_id.assign(instance_id), "instance ID must fit");
  require(command.type.assign("joint_torque"), "command type must fit");
  command.generation = generation;
  command.sequence = sequence;
  command.apply_time_ns = apply_time_ns;

  lingtu::sim::ActuatorCommandPayload payload;
  payload.value_count = 1;
  payload.values[0] = value;
  require(command.set_payload(payload), "actuator payload must fit the command envelope");
  return command;
}

void test_multi_robot_actuator_bindings_are_source_and_instance_isolated() {
  lingtu::sim::PhysicsScenePlan plan;
  plan.session_id = "actuator-session";
  plan.model_generation = 9;
  plan.world_model_path = LINGTU_MUJOCO_SCENE_WORLD;
  plan.robots = {
      {"robot_a", LINGTU_MUJOCO_ACTUATED_MODEL, "test_body",
       {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}},
      {"robot_b", LINGTU_MUJOCO_ACTUATED_MODEL, "test_body",
       {{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}},
  };

  auto runtime = lingtu::sim::MujocoRuntime::from_plan(std::move(plan));
  runtime.bind_actuators({
      "controller_a",
      "robot_a",
      "joint_torque",
      20'000'000,
      {"hinge_joint"},
  });
  runtime.bind_actuators({
      "controller_b",
      "robot_b",
      "joint_torque",
      20'000'000,
      {"hinge_joint"},
  });

  const auto find_actuator =
      [](const lingtu::sim::SimulationSnapshot &snapshot,
         const std::string &stable_id) {
        return std::find_if(
            snapshot.actuators.begin(), snapshot.actuators.end(),
            [&stable_id](const lingtu::sim::ActuatorState &actuator) {
              return actuator.stable_id == stable_id;
            });
      };

  require(runtime.apply_command(
              actuator_command("controller_a", "robot_a", 1, {9, 0}, 0, 1.0)) ==
              lingtu::sim::ActuatorCommandResult::applied,
          "controller A must apply to its own robot instance");
  const auto &after_a = runtime.snapshot();
  const auto actuator_a = find_actuator(after_a, "robot_a/hinge_joint");
  const auto actuator_b = find_actuator(after_a, "robot_b/hinge_joint");
  require(actuator_a != after_a.actuators.end() &&
              actuator_b != after_a.actuators.end(),
          "both robot actuator states must be published by stable ID");
  require(std::abs(actuator_a->control - 1.0) < 1e-12,
          "command A must drive actuator A");
  require(std::abs(actuator_b->control) < 1e-12,
          "command A must not drive actuator B");

  require(runtime.apply_command(
              actuator_command("controller_b", "robot_b", 1, {9, 0}, 0, -1.0)) ==
              lingtu::sim::ActuatorCommandResult::applied,
          "controller B must apply to its own robot instance");
  const auto &after_b = runtime.snapshot();
  require(std::abs(find_actuator(after_b, "robot_a/hinge_joint")->control - 1.0) <
              1e-12,
          "command B must not overwrite actuator A");
  require(std::abs(find_actuator(after_b, "robot_b/hinge_joint")->control + 1.0) <
              1e-12,
          "command B must drive actuator B independently");

  require(runtime.apply_command(
              actuator_command("controller_a", "robot_b", 2, {9, 0}, 0, 0.5)) ==
              lingtu::sim::ActuatorCommandResult::rejected_instance,
          "a source command naming the wrong instance must be rejected");
  const auto &after_wrong_instance = runtime.snapshot();
  require(std::abs(find_actuator(after_wrong_instance, "robot_a/hinge_joint")->control) <
              1e-12,
          "wrong-instance rejection may fail closed only for source A");
  require(std::abs(find_actuator(after_wrong_instance, "robot_b/hinge_joint")->control +
                   1.0) < 1e-12,
          "wrong-instance rejection must not move source B binding");

  require(runtime.apply_command(
              actuator_command("controller_b", "robot_b", 2, {8, 0}, 0, 0.5)) ==
              lingtu::sim::ActuatorCommandResult::rejected_stale_model_generation,
          "a stale model generation for source B must be rejected");
  const auto &after_wrong_generation = runtime.snapshot();
  require(std::abs(find_actuator(after_wrong_generation, "robot_a/hinge_joint")->control) <
              1e-12,
          "stale generation rejection for source B must not move source A binding");
  require(std::abs(find_actuator(after_wrong_generation, "robot_b/hinge_joint")->control) <
              1e-12,
          "stale generation rejection must fail closed only for source B");
}

void test_actuator_binding_drives_physics_and_fails_closed() {
  lingtu::sim::PhysicsScenePlan plan;
  plan.session_id = "actuator-session";
  plan.model_generation = 5;
  plan.world_model_path = LINGTU_MUJOCO_SCENE_WORLD;
  plan.robots = {
      {"robot", LINGTU_MUJOCO_ACTUATED_MODEL, "test_body",
       {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}},
  };

  auto runtime = lingtu::sim::MujocoRuntime::from_plan(std::move(plan));
  runtime.bind_actuators({
      "test_controller",
      "robot",
      "joint_torque",
      10'000'000,
      {"hinge_joint"},
  });

  const auto find_actuator = [](const lingtu::sim::SimulationSnapshot &snapshot) {
    return std::find_if(
        snapshot.actuators.begin(), snapshot.actuators.end(),
        [](const lingtu::sim::ActuatorState &actuator) {
          return actuator.stable_id == "robot/hinge_joint";
        });
  };
  const auto find_joint = [](const lingtu::sim::SimulationSnapshot &snapshot) {
    return std::find_if(
        snapshot.joints.begin(), snapshot.joints.end(),
        [](const lingtu::sim::JointState &joint) {
          return joint.stable_id == "robot/hinge_joint";
        });
  };

  const auto &initial = runtime.snapshot();
  const auto initial_joint = find_joint(initial);
  require(initial_joint != initial.joints.end(), "joint state must be published by stable ID");
  require(initial_joint->position_rad.size() == 1 &&
              initial_joint->velocity_rps.size() == 1,
          "hinge state must expose one qpos and one qvel value");

  require(runtime.apply_command(actuator_command(1, {5, 0}, 0, 1.0)) ==
              lingtu::sim::ActuatorCommandResult::applied,
          "a current, complete actuator command must apply atomically");
  const auto applied_actuator = find_actuator(runtime.snapshot());
  require(applied_actuator != runtime.snapshot().actuators.end() &&
              std::abs(applied_actuator->control - 1.0) < 1e-12,
          "applied command must be visible in actuator state");

  const auto &advanced = runtime.advance(1);
  const auto advanced_joint = find_joint(advanced);
  require(advanced_joint != advanced.joints.end() &&
              std::abs(advanced_joint->velocity_rps[0]) > 1e-9,
          "a non-zero actuator command must change MuJoCo joint velocity");

  auto wrong_session = actuator_command(2, {5, 0}, advanced.sim_time_ns, 1.0);
  require(wrong_session.session_id.assign("foreign-session"),
          "alternate session id must fit");
  require(runtime.apply_command(wrong_session) ==
              lingtu::sim::ActuatorCommandResult::rejected_session,
          "a command from another session must be rejected");
  require(std::abs(find_actuator(runtime.snapshot())->control) < 1e-12,
          "a wrong-session command must fail closed");

  require(runtime.apply_command(actuator_command(3, {5, 0}, advanced.sim_time_ns, 3.0)) ==
              lingtu::sim::ActuatorCommandResult::rejected_out_of_range,
          "an out-of-range command must be rejected");
  require(std::abs(find_actuator(runtime.snapshot())->control) < 1e-12,
          "a rejected command must zero every actuator owned by its binding");

  require(runtime.apply_command(actuator_command(4, {4, 0}, advanced.sim_time_ns, 1.0)) ==
              lingtu::sim::ActuatorCommandResult::rejected_stale_model_generation,
          "an old model generation must be rejected");

  require(runtime.apply_command(actuator_command(5, {5, 0}, advanced.sim_time_ns, 1.0)) ==
              lingtu::sim::ActuatorCommandResult::applied,
          "a new valid command must recover after a fail-closed rejection");
  const auto &expired = runtime.advance(6);
  require(expired.sim_time_ns >= 12'000'000,
          "the stale-control test must advance beyond its timeout");
  require(std::abs(find_actuator(expired)->control) < 1e-12,
          "a missing command refresh must expire to zero in simulation time");

  require(runtime.apply_command(
              actuator_command(6, {5, 0}, expired.sim_time_ns, 1.0)) ==
              lingtu::sim::ActuatorCommandResult::applied,
          "a current command must apply before pause");
  runtime.set_paused(true);
  require(std::abs(find_actuator(runtime.snapshot())->control) < 1e-12,
          "pause must immediately clear owned actuator controls");
  require(runtime.apply_command(
              actuator_command(7, {5, 0}, expired.sim_time_ns, 1.0)) ==
              lingtu::sim::ActuatorCommandResult::rejected_paused,
          "paused Physics Runtime must reject actuator commands");
  runtime.set_paused(false);
  require(runtime.apply_command(
              actuator_command(8, {5, 0}, expired.sim_time_ns, 1.0)) ==
              lingtu::sim::ActuatorCommandResult::applied,
          "resume must require a newer explicit command");

  const auto &reset = runtime.reset();
  require(reset.reset_generation == 1, "reset must advance the command generation");
  require(std::abs(find_actuator(reset)->control) < 1e-12,
          "reset must force every bound actuator to its torque-safe zero");
  require(runtime.apply_command(actuator_command(1, {5, 0}, 0, 1.0)) ==
              lingtu::sim::ActuatorCommandResult::rejected_stale_reset_generation,
          "commands from the previous reset must not reactivate controls");
}

}  // namespace

int main() {
  try {
    test_load_and_advance();
    test_pause_preserves_the_published_state();
    test_reset_starts_a_new_generation();
    test_mid360_raycast_uses_current_mujoco_model_and_data();
    test_thunder_model_snapshot_contract();
    test_named_keyframe_initializes_thunder_nominal_stand();
    test_scene_composer_rejects_robot_owned_global_options_before_attach();
    test_scene_composer_uses_one_model_for_multiple_instances();
    test_scene_composer_attaches_payload_to_its_robot_parent_body();
    test_runtime_advances_one_composed_session();
    test_scenario_kinematic_proxy_is_composed_and_applied_atomically();
    test_catalog_world_and_robot_packages_compile_as_one_runtime();
    test_composed_snapshot_excludes_static_world_bodies();
    test_composed_thunder_sensor_snapshot_uses_stable_sources();
    test_composed_runtime_applies_package_keyframe_and_spawn();
    test_multi_robot_actuator_bindings_are_source_and_instance_isolated();
    test_actuator_binding_drives_physics_and_fails_closed();
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return EXIT_FAILURE;
  }

  std::cout << "PASS: load, advance, pause, and reset\n";
  return EXIT_SUCCESS;
}
