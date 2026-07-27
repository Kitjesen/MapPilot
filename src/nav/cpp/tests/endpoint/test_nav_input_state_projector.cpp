#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "input/nav_input_state_projector.hpp"
namespace {

using lingtu::nav::endpoint::EndpointState;
using lingtu::nav::endpoint::InputGate;
using lingtu::nav::endpoint::InputGateConfig;
using lingtu::nav::endpoint::LiveObstacleLayer;
using lingtu::nav::endpoint::NavInputStateProjector;
using lingtu::nav::endpoint::NavInputStateProjectorActions;
using lingtu::nav::endpoint::NavInputStateProjectorConfig;
using lingtu::nav::endpoint::PlannerInputClearSource;
using lingtu::nav::endpoint::SteadyClock;
using lingtu::nav::endpoint::TimingDiagnostics;
using lingtu::nav::endpoint::TransformBuffer;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

lingtu_dds_TFMessage tfMessage(lingtu_dds_TransformStamped &transform, double stamp_s,
                               double x = 0.0) {
  static char map_frame[] = "map";
  static char odom_frame[] = "odom";
  transform = {};
  transform.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  transform.header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_s - static_cast<double>(transform.header.stamp.sec)) * 1e9);
  transform.header.frame_id = map_frame;
  transform.child_frame_id = odom_frame;
  transform.transform.translation.x = x;
  transform.transform.rotation.w = 1.0;

  lingtu_dds_TFMessage message{};
  message.transforms._length = 1;
  message.transforms._maximum = 1;
  message.transforms._buffer = &transform;
  return message;
}
lingtu_dds_Odometry odometryMessage(double stamp_s, const char *frame_id, double x = 0.0) {
  lingtu_dds_Odometry message{};
  message.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  message.header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_s - static_cast<double>(message.header.stamp.sec)) * 1e9);
  message.header.frame_id = const_cast<char *>(frame_id);
  message.child_frame_id = const_cast<char *>("body");
  message.pose.pose.position.x = x;
  message.pose.pose.orientation.w = 1.0;
  return message;
}

SteadyClock::time_point steadyTime(double seconds) {
  return SteadyClock::time_point{
      std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<double>(seconds))};
}

lingtu_dds_DriverControlState driverControlMessage(double stamp_s) {
  lingtu_dds_DriverControlState message{};
  message.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  message.header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_s - static_cast<double>(message.header.stamp.sec)) * 1e9);
  message.fsm = const_cast<char *>("standing");
  message.owner = const_cast<char *>("grpc");
  message.owner_id = const_cast<char *>("lingtu-driver");
  message.reason = const_cast<char *>("");
  message.accepted_producer_boot_id = const_cast<char *>("");
  return message;
}
struct PointCloudFixture {
  lingtu_dds_PointField fields[3]{};
  std::vector<std::uint8_t> data = std::vector<std::uint8_t>(12);
  lingtu_dds_PointCloud2 message{};

  PointCloudFixture(double stamp_s, const char *frame_id, float x, float y, float z) {
    const char *names[3] = {"x", "y", "z"};
    for (std::size_t index = 0; index < 3; ++index) {
      fields[index].name = const_cast<char *>(names[index]);
      fields[index].offset = static_cast<std::uint32_t>(index * 4);
      fields[index].datatype = 7;
      fields[index].count = 1;
    }
    message.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
    message.header.stamp.nanosec =
        static_cast<std::uint32_t>((stamp_s - static_cast<double>(message.header.stamp.sec)) * 1e9);
    message.header.frame_id = const_cast<char *>(frame_id);
    message.height = 1;
    message.width = 1;
    message.fields._length = 3;
    message.fields._maximum = 3;
    message.fields._buffer = fields;
    message.point_step = 12;
    message.row_step = 12;
    message.data._length = static_cast<std::uint32_t>(data.size());
    message.data._maximum = message.data._length;
    message.data._buffer = data.data();
    setPoint(x, y, z);
  }

  void setPoint(float x, float y, float z) {
    const float xyz[3] = {x, y, z};
    std::memcpy(data.data(), xyz, sizeof(xyz));
  }
};
struct TraversabilityFixture {
  std::vector<std::uint8_t> data{1, 2, 3, 4};
  lingtu_dds_OccupancyGrid message{};

  TraversabilityFixture(double stamp_s, const char *frame_id) {
    message.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
    message.header.stamp.nanosec =
        static_cast<std::uint32_t>((stamp_s - static_cast<double>(message.header.stamp.sec)) * 1e9);
    message.header.frame_id = const_cast<char *>(frame_id);
    message.info.width = 2;
    message.info.height = 2;
    message.info.resolution = 0.25f;
    message.info.origin.orientation.w = 1.0;
    message.data._length = static_cast<std::uint32_t>(data.size());
    message.data._maximum = message.data._length;
    message.data._buffer = data.data();
  }
};

void testTfProjectionPreservesCountersGenerationsAndReceiveClock() {
  EndpointState state;
  InputGate input_gate;
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  LiveObstacleLayer live_obstacles;
  NavInputStateProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   NavInputStateProjectorConfig{}, NavInputStateProjectorActions{});

  const lingtu_dds_TFMessage invalid{};
  projector.projectTf(invalid, 99.0);
  require(state.tf_count == 1, "invalid TF must still increment the receive counter");
  require(state.tf_generation == 0, "invalid TF must not advance generation");

  lingtu_dds_TransformStamped transform{};
  auto accepted = tfMessage(transform, 10.0);
  projector.projectTf(accepted, 100.0);
  require(state.tf_count == 2, "accepted TF must increment the receive counter");
  require(state.tf_generation == 1, "accepted TF must advance generation");
  require(std::abs(state.last_tf_s - 10.0) < 1e-9, "TF source clock must be retained");
  require(std::abs(state.last_tf_receive_s - 100.0) < 1e-9,
          "TF freshness clock must use explicit steady receipt time");

  auto rejected = tfMessage(transform, 9.9);
  projector.projectTf(rejected, 101.0);
  require(state.tf_count == 3, "rejected TF must still increment the receive counter");
  require(state.tf_generation == 1, "rejected TF must not advance generation");
  require(state.frames.last_error == "map_odom_tf_out_of_order",
          "TF rejection reason must remain exact");
  auto rebased = tfMessage(transform, 9.0);
  projector.projectTf(rebased, 102.0);
  require(state.tf_count == 4, "TF clock rebase must still increment the receive counter");
  require(state.tf_generation == 2, "TF clock rebase must accept the triggering sample");
  require(state.frame_epoch == 1, "TF clock rebase must start a new input epoch");
  require(state.frames.clock_rebases == 1, "TF clock rebase must increment diagnostics");
  require(state.map_odom_epoch_start_s == 9.0,
          "TF clock rebase must seed the new epoch with the triggering source clock");
}

void testEpochResetClearsInputsBeforeSynchronousEffectsAndAcceptsTriggeringTf() {
  EndpointState state;
  InputGate input_gate;
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  LiveObstacleLayer live_obstacles;
  bool callback_called = false;
  bool callback_clear_motion = false;
  NavInputStateProjectorActions actions;
  actions.on_epoch_reset = [&](double epoch_start_s, const std::string &reason, bool clear_motion) {
    callback_called = true;
    callback_clear_motion = clear_motion;
    require(std::abs(epoch_start_s - 11.0) < 1e-9, "epoch callback must carry source stamp");
    require(reason == "map_frame_jump", "epoch callback must preserve reset reason");
    require(state.frame_epoch == 1, "input epoch must increment before external effects");
    require(state.tf_generation == 1, "triggering TF generation must advance after effects");
    require(!state.map_odom_tf, "old map transform must clear before external effects");
    require(!state.odom_body, "old odometry must clear before external effects");
    require(state.terrain_xyzh.empty(), "terrain must clear before external effects");
    require(state.last_cloud_s == 0.0, "cloud freshness must clear before external effects");
    require(!map_odom_buffer.sample(10.0, 1.0),
            "transform history must clear before external effects");
  };
  NavInputStateProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   NavInputStateProjectorConfig{}, std::move(actions));

  lingtu_dds_TransformStamped transform{};
  auto initial = tfMessage(transform, 10.0, 0.0);
  projector.projectTf(initial, 100.0);
  state.odom_body = nav_kernel::Pose{};
  state.terrain_xyzh = {1.0f, 2.0f, 3.0f, 4.0f};
  state.last_cloud_s = 10.0;
  state.last_cloud_receive_s = 100.0;
  state.cloud_generation = 7;

  auto jumped = tfMessage(transform, 11.0, 0.6);
  projector.projectTf(jumped, 101.0);

  require(callback_called, "epoch effects must run synchronously");
  require(callback_clear_motion, "map-frame jump must request motion clearing");
  require(state.tf_generation == 2, "triggering TF must advance after epoch effects");
  require(state.map_odom_tf.has_value(), "triggering TF must seed the new epoch");
  require(std::abs(state.map_odom_tf->translation.x - 0.6) < 1e-9,
          "new epoch must retain the triggering transform");
  require(state.map_odom_epoch_start_s == 11.0,
          "new epoch boundary must use the triggering source stamp");
}

void testOdometryAcceptsMapFrameAndRequiresTimeAlignedTfForOdomFrame() {
  EndpointState map_state;
  InputGate map_gate;
  TransformBuffer map_pose_buffer;
  TransformBuffer map_tf_buffer;
  LiveObstacleLayer map_obstacles;
  NavInputStateProjector map_projector(map_state, map_gate, map_pose_buffer, map_tf_buffer,
                                       map_obstacles, NavInputStateProjectorConfig{},
                                       NavInputStateProjectorActions{});

  auto map_odom = odometryMessage(10.0, "map", 1.25);
  map_projector.projectOdometry(map_odom, 100.0);
  require(map_state.odom_count == 1, "map-frame odometry must be accepted without TF");
  require(map_state.odom_generation == 1, "accepted map odometry must advance generation");
  require(!map_state.odom_requires_tf, "map-frame odometry must bypass the TF gate");
  require(map_state.map_body && std::abs(map_state.map_body->position.x - 1.25) < 1e-9,
          "map-frame odometry must directly project the map pose");
  require(std::abs(map_state.last_odom_receive_s - 100.0) < 1e-9,
          "odometry freshness must use explicit steady receipt time");

  EndpointState odom_state;
  InputGate odom_gate;
  TransformBuffer odom_pose_buffer;
  TransformBuffer odom_tf_buffer;
  LiveObstacleLayer odom_obstacles;
  NavInputStateProjector odom_projector(odom_state, odom_gate, odom_pose_buffer, odom_tf_buffer,
                                        odom_obstacles, NavInputStateProjectorConfig{},
                                        NavInputStateProjectorActions{});

  auto odom_frame = odometryMessage(10.0, "odom", 1.0);
  odom_projector.projectOdometry(odom_frame, 100.0);
  require(odom_state.odom_count == 0, "odom-frame sample without TF must be rejected");
  require(odom_state.odom_generation == 0, "rejected odometry must not advance generation");
  require(odom_state.frames.odom_rejected == 1, "TF-gap rejection must be counted");
  require(odom_state.frames.last_error == "odom_tf_gap_exceeded",
          "odom-frame rejection must report the exact TF-gap reason");

  lingtu_dds_TransformStamped transform{};
  auto map_odom_tf = tfMessage(transform, 10.0, 2.0);
  odom_projector.projectTf(map_odom_tf, 100.1);
  odom_projector.projectOdometry(odom_frame, 100.2);
  require(odom_state.odom_count == 1, "time-aligned TF must admit odom-frame odometry");
  require(odom_state.odom_generation == 1, "accepted odom-frame sample must advance generation");
  require(odom_state.odom_requires_tf, "odom-frame sample must keep TF required");
  require(odom_state.map_body && std::abs(odom_state.map_body->position.x - 3.0) < 1e-9,
          "odom-frame pose must compose through the source-time map transform");
}

void testDriverProjectionCopiesLoansUsesSteadyFreshnessAndPreservesBlockerPrecedence() {
  EndpointState state;
  InputGateConfig gate_config;
  gate_config.recovery_frames = 1;
  gate_config.require_odom = false;
  gate_config.require_cloud = false;
  gate_config.require_driver_control = true;
  InputGate input_gate(gate_config);
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  LiveObstacleLayer live_obstacles;
  NavInputStateProjectorConfig projector_config;
  projector_config.driver_control_max_age_s = 0.35;
  NavInputStateProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   projector_config, NavInputStateProjectorActions{});

  require(projector.driverControlBlocker(steadyTime(1000.0)) == "driver_control_missing",
          "missing driver control must have highest blocker precedence");

  auto disconnected = driverControlMessage(10.0);
  disconnected.ready = true;
  disconnected.motors_enabled = true;
  disconnected.lease_valid = true;
  projector.projectDriverControl(disconnected, steadyTime(1000.0));
  require(state.driver_control_generation == 1, "accepted driver state must advance generation");
  require(state.driver_control_reason == "disconnected",
          "fallback reason precedence must start disconnected");
  require(projector.driverControlBlocker(steadyTime(1000.1)) == "driver_control_disconnected",
          "fresh not-ready driver state must expose its detailed blocker");

  char producer_reason[] = "producer_fault";
  char producer_boot[] = "boot-a";
  auto rejected = driverControlMessage(10.05);
  rejected.connected = true;
  rejected.ready = false;
  rejected.motors_enabled = true;
  rejected.lease_valid = true;
  rejected.reason = producer_reason;
  rejected.accepted_producer_boot_id = producer_boot;
  projector.projectDriverControl(rejected, steadyTime(1000.1));
  producer_reason[0] = 'X';
  producer_boot[0] = 'X';
  require(state.driver_control_reason == "producer_fault",
          "projector must own copied driver reason after DDS loan release");
  require(state.driver_accepted_producer_boot_id == "boot-a",
          "projector must own copied driver acknowledgement identity");

  auto ready = driverControlMessage(10.1);
  ready.connected = true;
  ready.ready = true;
  ready.motors_enabled = true;
  ready.lease_valid = true;
  projector.projectDriverControl(ready, steadyTime(1000.2));
  require(state.driver_control_ready, "fully ready driver state must admit motion");
  require(projector.driverControlBlocker(steadyTime(1000.3)).empty(),
          "fresh ready driver state must not block motion");
  require(std::abs(projector.driverControlReceiveAge(steadyTime(1000.3)) - 0.1) < 1e-9,
          "driver freshness must use the injected steady clock");

  const auto gate_state = projector.evaluateInputGate(1000.3, steadyTime(1000.3));
  require(gate_state.ready, "steady receipt must keep gate ready despite producer clock offset");
  require(std::abs(gate_state.driver_control_age_s - 0.1) < 1e-9,
          "InputSnapshot driver age must stay in the receiver steady-clock domain");

  const auto clock_rebases_before = state.frames.clock_rebases;
  auto rebased = ready;
  rebased.header.stamp.sec = 9;
  rebased.header.stamp.nanosec = 0;
  rebased.ready = false;
  rebased.reason = const_cast<char *>("lease_lost");
  projector.projectDriverControl(rebased, steadyTime(1000.4));
  require(state.driver_control_generation == 4, "driver clock rebase must be accepted");
  require(state.frames.clock_rebases == clock_rebases_before,
          "driver clock rebase must remain silent in frame diagnostics");
  require(projector.driverControlBlocker(steadyTime(1001.0)) == "driver_control_stale",
          "driver staleness must precede readiness details");
}

void testCloudTerrainAndSnapshotProjectionOwnDdsDataAndExactClocks() {
  EndpointState state;
  InputGate input_gate;
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  LiveObstacleLayer live_obstacles({
      0.10,
      1.0,
      0.0,
      4.0,
      0.0,
      100,
      1,
      false,
  });
  NavInputStateProjectorConfig config;
  config.sensor_offset = {0.2, 0.0, 0.0};
  config.max_obstacle_points = 100;
  NavInputStateProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   config, NavInputStateProjectorActions{});

  auto odometry = odometryMessage(10.0, "map", 0.0);
  projector.projectOdometry(odometry, 99.0);
  PointCloudFixture cloud(10.0, "map", 1.0f, 0.0f, 0.2f);
  TimingDiagnostics timing;
  projector.projectCloud(cloud.message, 100.0, 1000.0, timing);

  require(state.cloud_count == 1, "every cloud callback must increment cloud count");
  require(state.cloud_generation == 1, "accepted nonempty cloud must advance generation");
  require(std::abs(state.last_cloud_s - 10.0) < 1e-9, "cloud source stamp must be retained");
  require(std::abs(state.last_cloud_receive_s - 100.0) < 1e-9,
          "cloud freshness must use explicit steady receipt time");
  require(std::abs(state.cloud_sync.last_stamp_age_s - 990.0) < 1e-9,
          "cloud wall/source age must remain diagnostic-only");
  require(state.obstacle_xyzh.empty(),
          "cloud ingestion must defer planner snapshot materialization");
  require(state.obstacle_snapshot_dirty, "accepted scan must mark snapshot dirty");
  require(state.last_sensor_origin.valid && std::abs(state.last_sensor_origin.x - 0.2) < 1e-9,
          "sensor origin must rotate and translate the configured body offset");

  cloud.setPoint(99.0f, 0.0f, 0.2f);
  require(projector.materializeLiveObstacleSnapshot(timing),
          "dirty live obstacle layer must materialize once");
  require(!state.obstacle_xyzh.empty(), "materialized live obstacle snapshot must be available");
  require(std::abs(state.obstacle_xyzh.front() - 1.0f) < 0.11f,
          "materialized obstacles must not retain the DDS loan buffer");
  require(!state.obstacle_snapshot_dirty, "materialization must clear the dirty flag");
  require(!projector.materializeLiveObstacleSnapshot(timing),
          "clean live obstacle layer must not rematerialize");

  const lingtu_dds_PointCloud2 invalid{};
  projector.projectCloud(invalid, 101.0, 1001.0, timing);
  require(state.cloud_count == 2, "invalid cloud must still increment callback count");
  require(state.cloud_generation == 1, "invalid cloud must not advance generation");
  require(state.cloud_sync.stamp_rejected == 1, "invalid cloud stamp must be counted");

  PointCloudFixture terrain(10.5, "map", 2.0f, 0.0f, 0.3f);
  projector.projectTerrainMap(terrain.message, 101.5, timing);
  require(state.terrain_map_count == 1, "nonempty terrain map must increment accepted count");
  require(state.terrain_xyzh.size() == 4, "terrain map must own converted XYZH");
  terrain.setPoint(88.0f, 0.0f, 0.3f);
  require(std::abs(state.terrain_xyzh.front() - 2.0f) < 1e-6f,
          "terrain projection must not retain DDS data");

  PointCloudFixture terrain_ext(10.6, "map", 3.0f, 0.0f, 0.4f);
  projector.projectTerrainMapExt(terrain_ext.message, 101.6, timing);
  require(state.terrain_map_ext_count == 1, "terrain-ext must track its own accepted count");
  require(std::abs(state.last_terrain_ext_receive_s - 101.6) < 1e-9,
          "terrain-ext freshness must use its own steady receipt clock");
}

void testPlannerClearingUsesDistinctSynchronousReasonsWithoutResettingCloudEpoch() {
  EndpointState state;
  InputGate input_gate;
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  LiveObstacleLayer live_obstacles;
  std::vector<std::string> invalidation_reasons;
  NavInputStateProjectorActions actions;
  actions.on_rolling_snapshot_invalidated = [&](const std::string &reason) {
    require(state.terrain_xyzh.empty(), "planner inputs must clear before rolling invalidation");
    require(state.obstacle_xyzh.empty(), "obstacles must clear before rolling invalidation");
    invalidation_reasons.push_back(reason);
    state.frames.last_error = reason;
  };
  NavInputStateProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   NavInputStateProjectorConfig{}, std::move(actions));

  state.terrain_xyzh = {1.0f, 0.0f, 0.0f, 0.0f};
  state.terrain_ext_xyzh = state.terrain_xyzh;
  state.planner_terrain_xyzh = state.terrain_xyzh;
  state.obstacle_xyzh = state.terrain_xyzh;
  state.last_terrain_map_s = 10.0;
  state.last_traversability_s = 10.0;
  state.last_cloud_s = 10.0;
  state.last_cloud_receive_s = 100.0;
  state.cloud_generation = 7;
  state.last_sensor_origin = {1.0, 2.0, 3.0, true};
  state.obstacle_snapshot_dirty = true;
  live_obstacles.update(state.obstacle_xyzh, 10.0);

  const lingtu_dds_Bool no_clear{};
  projector.clearPlannerInputs(no_clear, PlannerInputClearSource::kMap);
  require(state.map_clearing_count == 0, "false map clear must be a no-op");
  require(!state.terrain_xyzh.empty(), "false map clear must preserve planner inputs");

  lingtu_dds_Bool clear{};
  clear.data = true;
  projector.clearPlannerInputs(clear, PlannerInputClearSource::kMap);
  require(state.map_clearing_count == 1, "map clear must increment only map counter");
  require(state.cloud_clearing_count == 0, "map clear must not increment cloud counter");
  require(invalidation_reasons.size() == 1 &&
              invalidation_reasons.back() == "execution_grid_map_cleared",
          "map clear must synchronously invalidate the matching rolling input");
  require(state.frames.last_error == "execution_grid_map_cleared",
          "rolling callback must be able to publish the clearing reason");
  require(state.last_cloud_s == 10.0, "planner clear must preserve cloud source epoch");
  require(state.cloud_generation == 7, "planner clear must preserve cloud generation");
  require(state.last_sensor_origin.valid, "planner clear must preserve sensor origin");

  state.terrain_xyzh = {2.0f, 0.0f, 0.0f, 0.0f};
  state.obstacle_xyzh = state.terrain_xyzh;
  projector.clearPlannerInputs(clear, PlannerInputClearSource::kCloud);
  require(state.cloud_clearing_count == 1, "cloud clear must increment cloud counter");
  require(invalidation_reasons.size() == 2 &&
              invalidation_reasons.back() == "execution_grid_cloud_cleared",
          "cloud clear must use its distinct synchronous invalidation reason");
}
void testTraversabilityAndLocalizationProjectionOwnPayloadsAndAdvanceExactGenerations() {
  EndpointState state;
  InputGate input_gate;
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  LiveObstacleLayer live_obstacles;
  NavInputStateProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   NavInputStateProjectorConfig{}, NavInputStateProjectorActions{});

  TraversabilityFixture grid(20.0, "map");
  projector.projectTraversability(grid.message, 200.0);
  require(state.traversability_count == 1, "accepted grid must increment accepted count");
  require(state.traversability_generation == 1, "accepted grid must advance generation");
  require(state.traversability_grid.generation == 1,
          "decoded grid generation must match projector generation");
  require(std::abs(state.last_traversability_receive_s - 200.0) < 1e-9,
          "grid freshness must use explicit steady receipt time");
  grid.data[0] = 99;
  require(state.traversability_grid.values.front() == 1.0f,
          "traversability projection must not retain DDS payload");

  TraversabilityFixture bad_grid(20.1, "odom");
  projector.projectTraversability(bad_grid.message, 200.1);
  require(state.traversability_count == 1, "rejected grid must not increment accepted count");
  require(state.traversability_generation == 1, "rejected grid must not advance generation");
  require(state.frames.grid_rejected == 1, "rejected grid must increment rejection count");
  require(state.frames.last_error == "grid_frame_unsupported",
          "grid decoder reason must remain exact");

  char health_json[] = R"({"state":"TRACKING","reason":"nominal","ts":30.0})";
  lingtu_dds_Text health{};
  health.data = health_json;
  projector.projectLocalizationHealth(health, 300.0);
  require(state.localization_health_generation == 1, "accepted health must advance generation");
  require(state.localization_health.healthy, "TRACKING health must be projected healthy");
  require(state.localization_health.state == "TRACKING", "health state must be preserved");
  require(std::abs(state.localization_health_receive_s - 300.0) < 1e-9,
          "health freshness must use explicit steady receipt time");
  health_json[10] = 'X';
  require(state.localization_health.state == "TRACKING",
          "localization projection must not retain DDS text loan");

  char invalid_json[] = R"({"ts":30.1})";
  health.data = invalid_json;
  projector.projectLocalizationHealth(health, 300.1);
  require(state.localization_health_generation == 1,
          "invalid health payload must not advance generation");
  require(state.frames.last_error == "localization_health_state_missing",
          "health decoder error must remain exact");

  const auto rebases_before = state.frames.clock_rebases;
  char rebased_json[] = R"({"state":"LOCKED","ts":20.0})";
  health.data = rebased_json;
  projector.projectLocalizationHealth(health, 300.2);
  require(state.localization_health_generation == 2,
          "large health clock rollback must be accepted");
  require(state.frames.clock_rebases == rebases_before + 1,
          "accepted health clock rollback must increment rebase diagnostics");
}

void testEpochRecoveryBaselinesBeforeTriggeringTfGeneration() {
  EndpointState state;
  InputGateConfig gate_config;
  gate_config.recovery_frames = 1;
  gate_config.require_cloud = false;
  gate_config.odom_max_age_s = 0.5;
  gate_config.tf_max_age_s = 0.5;
  InputGate input_gate(gate_config);
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  LiveObstacleLayer live_obstacles;
  NavInputStateProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   NavInputStateProjectorConfig{}, NavInputStateProjectorActions{});

  lingtu_dds_TransformStamped transform{};
  auto initial_tf = tfMessage(transform, 10.0, 0.0);
  auto initial_odom = odometryMessage(10.0, "odom", 0.0);
  projector.projectTf(initial_tf, 100.0);
  projector.projectOdometry(initial_odom, 100.0);
  require(projector.evaluateInputGate(100.05, steadyTime(100.05)).ready,
          "initial complete generation set must open a one-frame gate");

  auto jumped_tf = tfMessage(transform, 11.0, 0.6);
  projector.projectTf(jumped_tf, 101.0);
  require(!state.odom_body && state.last_odom_s == 0.0,
          "epoch reset must immediately remove old odometry");

  auto recovered_odom = odometryMessage(11.0, "odom", 0.0);
  projector.projectOdometry(recovered_odom, 101.05);
  auto gate_state = projector.evaluateInputGate(101.1, steadyTime(101.1));
  require(gate_state.ready,
          "triggering TF generation must count toward recovery because baseline precedes it");
}

}  // namespace

int main() {
  testTfProjectionPreservesCountersGenerationsAndReceiveClock();
  testEpochResetClearsInputsBeforeSynchronousEffectsAndAcceptsTriggeringTf();
  testOdometryAcceptsMapFrameAndRequiresTimeAlignedTfForOdomFrame();
  testDriverProjectionCopiesLoansUsesSteadyFreshnessAndPreservesBlockerPrecedence();
  testCloudTerrainAndSnapshotProjectionOwnDdsDataAndExactClocks();
  testTraversabilityAndLocalizationProjectionOwnPayloadsAndAdvanceExactGenerations();
  testPlannerClearingUsesDistinctSynchronousReasonsWithoutResettingCloudEpoch();
  testEpochRecoveryBaselinesBeforeTriggeringTfGeneration();
  std::cout << "test_nav_input_state_projector passed\n";
  return 0;
}
