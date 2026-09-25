#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <future>
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "planning/local/planner.hpp"
#include "safety/command.hpp"
#include "status/control_loop_health.hpp"
#include "status/nav_status_publisher.hpp"
#include "../collision_bitmap.hpp"

namespace {

using lingtu::nav::endpoint::CloudSyncDiagnostics;
using lingtu::nav::endpoint::CommandDiagnostics;
using lingtu::nav::endpoint::ControlLoopHealthSnapshot;
using lingtu::nav::endpoint::DynamicCluster;
using lingtu::nav::endpoint::FrameDiagnostics;
using lingtu::nav::endpoint::InputGateState;
using lingtu::nav::endpoint::LocalDiagnostics;
using lingtu::nav::endpoint::MotionLayerStats;
using lingtu::nav::endpoint::MotionStopEvidenceTracker;
using lingtu::nav::endpoint::NavStatusPublisher;
using lingtu::nav::endpoint::NavStatusPublisherActions;
using lingtu::nav::endpoint::OperatorMotionTransportDiagnostics;
using lingtu::nav::endpoint::PlanDiagnostics;
using lingtu::nav::endpoint::SensorOrigin;
using lingtu::nav::endpoint::StatusMotionLayerSample;
using lingtu::nav::endpoint::StatusPlannerSample;
using lingtu::nav::endpoint::StatusRuntimeState;
using lingtu::nav::endpoint::StatusWriterConfig;
using lingtu::nav::endpoint::StatusFarInputSample;
using lingtu::nav::endpoint::StopConfirmationConfig;
using lingtu::nav::endpoint::StopConfirmationDiagnostics;
using lingtu::nav::endpoint::StopConfirmationState;
using lingtu::nav::endpoint::TeleopDiagnostics;
using lingtu::nav::endpoint::TimingDiagnostics;
using lingtu::nav::endpoint::TraversabilityGrid;

void require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

bool contains(const std::string &value, const std::string &expected) {
  return value.find(expected) != std::string::npos;
}

struct Fixture {
  double steady_s{10.0};
  double wall_s{1234.5};
  double sampled_wall_s{-1.0};
  int planner_calls{0};
  int motion_calls{0};
  int log_calls{0};
  std::vector<std::string> writes;

  InputGateState input_gate;
  CloudSyncDiagnostics cloud_sync;
  FrameDiagnostics frames;
  OperatorMotionTransportDiagnostics operator_motion_transport;
  SensorOrigin sensor_origin;
  PlanDiagnostics plan;
  LocalDiagnostics local;
  TeleopDiagnostics teleop;
  std::vector<nav_kernel::Vec3> global_path;
  std::vector<nav_kernel::Vec3> local_path;
  nav_kernel::LocalPlannerDebugSnapshot local_debug;
  TraversabilityGrid traversability;
  lingtu::nav::tests::CollisionBitmap collision_bitmap{
      {-1.0, -1.0, -0.5}, {2.0, 1.0, 1.5}, 0.1};
  std::vector<float> obstacles{
      0.1F, 0.0F, 0.0F, -0.3F,
      1.0F, 2.0F, 3.0F, 0.5F,
  };
  std::vector<DynamicCluster> clusters;
  MotionLayerStats motion_stats;
  ControlLoopHealthSnapshot loop_health;
  StatusRuntimeState state;
  CommandDiagnostics commands;

  Fixture() {
    input_gate.ready = true;
    input_gate.reason = "ready";
    input_gate.driver_control_age_s = 0.1;
    input_gate.driver_control_ready = true;
    input_gate.driver_control_reason = "ready";
    local.cmd_vel.vx = 0.31;
    local.recovery_state = 2;
    local.recovery_action = 1;
    local.recovery_attempt = 2;
    local.recovery_candidate_count = 7;
    local.recovery_rotation_target_rad = -0.6;
    local.recovery_verified = true;
    local.recovery_progress = 0.625;
    local.recovery_reason = "recovery_translation_active";
    local.recovery_exhausted = true;
    teleop.manual_mode = true;
    teleop.output.vx = 0.77;
    motion_stats.dynamic_cells = 6;
    motion_stats.predicted_points = 40;
    motion_stats.prediction_clusters = 2;
    motion_stats.prediction_horizon_s = 1.0;
    loop_health.ready = true;
    loop_health.healthy = false;
    loop_health.reason = "deadline_miss_ratio_high";
    loop_health.period_ms = 50.0;
    loop_health.window_samples = 120;
    loop_health.total_samples = 140;
    loop_health.loop_ms = {49.75, 50.0, 52.5, 55.0, 60.0};
    loop_health.work_ms = {10.25, 9.0, 18.5, 22.0, 24.5};
    loop_health.overrun_ms = {0.75, 0.0, 2.5, 4.0, 5.5};
    loop_health.deadline_misses = 8;
    loop_health.deadline_miss_ratio = 0.066667;
    loop_health.current_miss_streak = 1;
    loop_health.max_miss_streak = 4;
    loop_health.p95_utilization = 0.84;
    loop_health.max_utilization = 0.98;
    loop_health.history.first_sequence = 139;
    loop_health.history.overruns_ms = {5.5, 0.0};
    loop_health.history.overruns = {{139, 55.5, {1.0, 2.0, 0.0, 50.0, 1.0}}};

    state.has_odom = true;
    state.has_map_odom_tf = true;
    state.path_active = true;
    state.counters.odom = 42;
    state.counters.goals = 5;
    state.counters.clouds = 9;
    state.counters.traversability = 7;
    state.counters.outputs = 11;
    state.counters.cmd_vel = 10;
    state.final_output = {"endpoint-a:boot", 17U};
    state.driver_control = {true, true, "ready", true, "endpoint-a:boot", 17U};
    state.input_gate = &input_gate;
    operator_motion_transport.ack_sent = 4;
    operator_motion_transport.ack_publish_failed = 1;
    operator_motion_transport.status_sent = 8;
    operator_motion_transport.status_publish_failed = 2;
    operator_motion_transport.last_ack.observed = true;
    operator_motion_transport.last_ack.published = true;
    operator_motion_transport.last_ack.source_id = "gateway";
    operator_motion_transport.last_ack.source_epoch = 9;
    operator_motion_transport.last_ack.source_sequence = 12;
    operator_motion_transport.last_ack.request_id = "claim-12";
    operator_motion_transport.last_ack.action = 0;
    operator_motion_transport.last_ack.accepted = true;
    operator_motion_transport.last_ack.reason = "claimed";
    operator_motion_transport.last_ack.accepted_sequence = 12;
    operator_motion_transport.status.observed = true;
    operator_motion_transport.status.published = true;
    operator_motion_transport.status.active_source_id = "gateway";
    operator_motion_transport.status.active_source_epoch = 9;
    operator_motion_transport.status.has_active_authority = true;
    operator_motion_transport.status.has_active_sample = true;
    operator_motion_transport.status.last_sample_sequence = 13;
    operator_motion_transport.status.admitted_sequence = 13;
    operator_motion_transport.status.final_output_sequence = 17;
    operator_motion_transport.status.authority_reason = "sample_admitted";
    operator_motion_transport.status.input_gate_reason = "ready";
    operator_motion_transport.status.teleop_output.vx = 0.25;
    state.cloud_sync = &cloud_sync;
    state.frames = &frames;
    state.last_sensor_origin = &sensor_origin;
    state.operator_motion_transport = &operator_motion_transport;
    state.plan = &plan;
    state.local = &local;
    state.teleop = &teleop;
    state.global_path = &global_path;
    state.local_path = &local_path;
    state.local_planner_debug = &local_debug;
    state.local_map_traversability = &traversability;
    collision_bitmap.occupy({0.5, 0.0, 0.4});
    state.local_collision_map = collision_bitmap.view(1234.0, 3U);
    state.local_collision_map.observationSequence = 2U;

    commands.received = 3;
    commands.ack_sent = 2;
    commands.ack_publish_failed = 1;
  }

  StatusWriterConfig config(std::string path = "ignored.json") const {
    StatusWriterConfig result;
    result.status_file = std::move(path);
    result.control_mode = "autonomy";
    result.local_planner = "cmu";
    result.product = "nav";
    result.product_session_id = "product-session-test";
    result.check_obstacle = true;
    result.use_traversability_cost = true;
    result.driver_control_max_age_s = 0.35;
    result.input_require_driver_control = true;
    result.local_map_debug_point_limit = 4;
    result.stop_confirmation_timeout_s = 4.0;
    result.stop_confirmation_evidence = "driver_ack_and_odometry";
    return result;
  }

  NavStatusPublisherActions actions() {
    NavStatusPublisherActions result;
    result.steady_now_s = [this]() { return steady_s; };
    result.wall_now_s = [this]() { return wall_s; };
    result.sample_planner = [this](double, TimingDiagnostics &timing) {
      ++planner_calls;
      timing.obstacle_merge_ms += 3.0;
      return StatusPlannerSample{true, true, false, obstacles.size() / 4, &obstacles};
    };
    result.sample_motion_layer = [this](double stamp_s) {
      ++motion_calls;
      sampled_wall_s = stamp_s;
      return StatusMotionLayerSample{17, motion_stats, &clusters};
    };
    result.sample_loop_health = [this]() { return loop_health; };
    result.sample_far_input = []() {
      return StatusFarInputSample{false, true, "not_required", "", 0};
    };
    result.log = [this](const std::string &) { ++log_calls; };
    return result;
  }

  auto sink() {
    return [this](const std::filesystem::path &, const std::string &snapshot) {
      writes.push_back(snapshot);
      return true;
    };
  }
};

void testQueuedSnapshotOwnsCollisionBytesAfterInputChanges() {
  Fixture fixture;
  std::promise<void> sink_started;
  std::promise<void> release_sink;
  auto started = sink_started.get_future();
  auto released = release_sink.get_future();
  NavStatusPublisher publisher(fixture.config(), 2.0, fixture.actions(),
      [&](const std::filesystem::path &, const std::string &snapshot) {
        if (fixture.writes.empty()) {
          sink_started.set_value();
          released.wait();
        }
        fixture.writes.push_back(snapshot);
        return true;
      });
  TimingDiagnostics previous, current;
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "first snapshot must reach the writer");
  require(started.wait_for(std::chrono::seconds(5)) == std::future_status::ready,
          "snapshot writer must start before queuing its replacement");
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "second snapshot must queue while the writer is busy");
  fixture.collision_bitmap.clear();
  release_sink.set_value();
  publisher.flush();
  require(fixture.writes.size() == 2U, "both snapshots must finish");
  require(contains(fixture.writes.back(), "\"occupied_points_returned\": 1"),
          "queued sampling must retain the captured collision bitmap");
}

void testCadenceImmediateDisabledAndEmptyFileLog() {
  Fixture fixture;
  NavStatusPublisher publisher(fixture.config(), 2.0, fixture.actions(), fixture.sink());
  TimingDiagnostics previous;
  TimingDiagnostics current;

  require(!publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "publisher must wait for the first steady-clock deadline");
  fixture.steady_s = 12.0;
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "publisher must run at the steady-clock deadline");
  publisher.flush();
  require(fixture.planner_calls == 1, "due publish must sample planner once");
  require(fixture.motion_calls == 1, "due publish must sample motion once");
  require(std::abs(fixture.sampled_wall_s - fixture.wall_s) < 1e-9,
          "motion clusters must use the wall-clock status stamp");
  require(fixture.writes.size() == 1, "due publish must submit one snapshot");

  require(!publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "deadline must be scheduled before status work");
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "requestImmediate must bypass the pending deadline");
  publisher.flush();

  Fixture disabled;
  NavStatusPublisher disabled_publisher(disabled.config(), 0.0, disabled.actions(),
                                        disabled.sink());
  disabled_publisher.requestImmediate();
  require(!disabled_publisher.publishIfDue(disabled.state, disabled.commands, previous, current),
          "non-positive intervals must remain disabled");
  require(disabled.planner_calls == 0, "disabled publisher must do no work");

  Fixture log_only;
  NavStatusPublisher log_only_publisher(log_only.config(""), 1.0, log_only.actions(),
                                        log_only.sink());
  log_only.steady_s = 11.0;
  require(log_only_publisher.publishIfDue(log_only.state, log_only.commands, previous, current),
          "empty status file must not disable periodic status");
  log_only_publisher.flush();
  require(log_only.log_calls == 1, "empty status file must still emit the log");
  require(log_only.writes.empty(), "empty status file must skip snapshot I/O");
}

void testMissingCloudPoseSerializesUnavailableGap() {
  Fixture fixture;
  fixture.cloud_sync.last_pose_gap_s = std::numeric_limits<double>::infinity();
  NavStatusPublisher publisher(fixture.config(), 10.0, fixture.actions(), fixture.sink());
  TimingDiagnostics timing;
  publisher.requestImmediate();
  publisher.publishIfDue(fixture.state, fixture.commands, timing, timing);
  publisher.flush();
  require(contains(fixture.writes.back(), "\"last_pose_gap_s\": -1.000000"),
          "an empty pose buffer must produce valid JSON with an unavailable gap");
  fixture.cloud_sync.last_pose_gap_s = 0.125;
  publisher.requestImmediate();
  publisher.publishIfDue(fixture.state, fixture.commands, timing, timing);
  publisher.flush();
  require(contains(fixture.writes.back(), "\"last_pose_gap_s\": 0.125000"),
          "a finite pose gap must retain its measured value");
}

void testStatusSnapshotPreservesPrecedenceCountersFreshnessAndTiming() {
  Fixture fixture;
  auto config = fixture.config();
  config.publish_cmd_vel = true;
  NavStatusPublisher publisher(config, 10.0, fixture.actions(), fixture.sink());
  TimingDiagnostics previous;
  previous.loop_ms = 12.5;
  previous.obstacle_merge_ms = 4.5;
  TimingDiagnostics current;
  current.loop_ms = 99.0;
  current.obstacle_merge_ms = 1.0;
  current.status_log_ms = -1.0;
  current.status_snapshot_ms = -1.0;

  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "immediate autonomy snapshot must publish");
  publisher.flush();
  const std::string autonomy = fixture.writes.back();
  require(contains(autonomy, "\"product_session_id\": \"product-session-test\""),
          "status must expose the Product session identity");
  require(contains(autonomy, "\"owner\": \"active_path_blockage_overlay\"") &&
              !contains(autonomy, "local_planner_footprint"),
          "live obstacle overlay diagnostics must not claim robot footprint ownership");
  require(!contains(autonomy, "config_fingerprint"),
          "status must not expose a redundant native config fingerprint");
  require(contains(autonomy, "\"active_cmd_source\": \"autonomy\""),
          "active path must select autonomy source");
  require(contains(autonomy, "\"stop_confirmation_timeout_s\": 4.000000"),
          "status must expose the active stop confirmation product contract");
  require(contains(autonomy,
                   "\"stop_confirmation_evidence\": \"driver_ack_and_odometry\""),
          "status must expose the active stop evidence policy");
  require(contains(autonomy,
                   "\"motion_stop_evidence\": {\"state\": \"NOT_REQUESTED\", "
                   "\"reason\": \"not_requested\", \"output_sequence\": 0, "
                   "\"updated_at_s\": 0.000000, "
                   "\"confirmation_state\": \"not_requested\", "
                   "\"driver_ack_observed\": false, \"driver_accepted\": false, "
                   "\"quiet_odometry_samples\": 0, "
                   "\"required_quiet_odometry_samples\": 0, "
                   "\"last_linear_speed_mps\": null, "
                   "\"last_angular_speed_radps\": null, "
                   "\"linear_speed_threshold_mps\": 0.000000, "
                   "\"angular_speed_threshold_radps\": 0.000000}"),
          "status must expose a complete not-requested motion-stop evidence snapshot");
  require(contains(autonomy, "\"operator_motion\": {\"schema_version\": 1") &&
              contains(autonomy, "\"interface_enabled\": false") &&
              contains(autonomy, "\"authority_owner\": \"native_endpoint\"") &&
              contains(autonomy, "\"control_ack_scope\": \"claim_hold_release\"") &&
              contains(autonomy, "\"sample_evidence\": \"status_sequences\"") &&
              contains(autonomy, "\"ack_sent\": 4") &&
              contains(autonomy, "\"ack_publish_failed\": 1") &&
              contains(autonomy, "\"status_sent\": 8") &&
              contains(autonomy, "\"status_publish_failed\": 2"),
          "autonomy without takeover must expose a disabled truthful operator-motion contract");
  require(contains(autonomy, "\"last_ack\": {\"observed\": true") &&
              contains(autonomy, "\"published\": true, \"source_id\": \"gateway\"") &&
              contains(autonomy, "\"source_sequence\": 12") &&
              contains(autonomy, "\"accepted_sequence\": 12"),
          "operator-motion status must mirror the last control ACK and publication result");
  require(contains(autonomy, "\"status\": {\"observed\": true") &&
              contains(autonomy, "\"active_source_id\": \"gateway\"") &&
              contains(autonomy, "\"has_active_authority\": true") &&
              contains(autonomy, "\"last_sample_sequence\": 13") &&
              contains(autonomy, "\"admitted_sequence\": 13") &&
              contains(autonomy, "\"final_output_sequence\": 17") &&
              contains(autonomy, "\"input_gate_reason\": \"ready\"") &&
              contains(autonomy, "\"teleop_output\": {\"vx\": 0.250000"),
          "operator-motion status must correlate sample admission with the final DDS output");
  require(contains(autonomy, "\"final_cmd_vel\": {\"vx\": 0.310000"),
          "autonomy final velocity must use local output");
  require(contains(autonomy, "\"manual_mode\": true"),
          "status must expose whether the active teleop sample requested manual escape");
  require(contains(autonomy, "\"recovery_state\": 2") &&
              contains(autonomy, "\"recovery_action\": 1") &&
              contains(autonomy, "\"recovery_attempt\": 2") &&
              contains(autonomy, "\"recovery_candidate_count\": 7") &&
              contains(autonomy, "\"recovery_rotation_target_rad\": -0.600000") &&
              contains(autonomy, "\"recovery_verified\": true") &&
              contains(autonomy, "\"recovery_progress\": 0.625000") &&
              contains(autonomy, "\"recovery_reason\": \"recovery_translation_active\"") &&
              contains(autonomy, "\"recovery_exhausted\": true"),
          "last_local must expose the complete recovery decision and progress evidence");
  require(contains(autonomy, "\"final_output\": {\"published\": true") &&
              contains(autonomy, "\"producer_boot_id\": \"endpoint-a:boot\"") &&
              contains(autonomy, "\"output_sequence\": 17") &&
              contains(autonomy, "\"driver_delivery_accepted\": true"),
          "final output must expose its exact DDS identity and correlated driver ACK");
  require(contains(autonomy, "\"driver_control\": {\"received\": true") &&
              contains(autonomy, "\"ready\": true") &&
              contains(autonomy, "\"last_command_accepted\": true") &&
              contains(autonomy, "\"fresh\": true") && contains(autonomy, "\"age_s\": 0.100000") &&
              contains(autonomy, "\"accepted_output_sequence\": 17"),
          "driver control must expose current acceptance evidence");
  require(contains(autonomy, "\"stamp_s\": 1234.500000"),
          "JSON snapshot must use wall-clock stamp");
  require(contains(autonomy, "\"has_traversability\": true"),
          "fresh traversability must be reported");
  require(contains(autonomy, "\"has_terrain_map\": true") &&
              contains(autonomy, "\"has_terrain_map_ext\": false"),
          "planner freshness flags must map into the snapshot");
  require(contains(autonomy, "\"collision\": {\"enabled\": true") &&
              contains(autonomy, "\"occupied_points_total\": 1") &&
              contains(autonomy, "\"occupied_points\": [[0.550000, 0.050000, 0.450000]]"),
          "local-map diagnostics must expose the exact collision layer consumed by SCAN");
  require(contains(autonomy, "\"source_obstacle_points_total\": 2") &&
              contains(autonomy, "\"obstacle_filter\": \"cmu_height_envelope\"") &&
              contains(autonomy, "\"obstacle_points_total\": 1") &&
              contains(autonomy, "\"obstacle_points\": [[1.000000, 2.000000, 3.000000, 0.500000]]"),
          "CMU local-map diagnostics must omit ground points before bounded sampling");
  require(contains(autonomy, "\"predicted_points\": 40") &&
              contains(autonomy, "\"prediction_clusters\": 2") &&
              contains(autonomy, "\"prediction_horizon_s\": 1.000000"),
          "motion-layer status must expose applied dynamic prediction evidence");
  require(contains(autonomy, "\"odom\": 42") && contains(autonomy, "\"command_requests\": 3"),
          "runtime and command counters must be preserved");
  require(contains(autonomy, "\"command_ack_publish_failed\": 1"),
          "failed command ACK publications must be observable");
  require(contains(autonomy, "\"loop\": 12.500000") &&
              contains(autonomy, "\"obstacle_merge\": 4.500000"),
          "JSON timing must use the previous completed tick");
  require(contains(autonomy, "\"control_loop_health\": {") &&
              contains(autonomy, "\"ready\": true") && contains(autonomy, "\"healthy\": false") &&
              contains(autonomy, "\"reason\": \"deadline_miss_ratio_high\"") &&
              contains(autonomy, "\"window_samples\": 120") &&
              contains(autonomy, "\"deadline_miss_ratio\": 0.066667") &&
              contains(autonomy, "\"p95_utilization\": 0.840000"),
          "JSON snapshot must expose top-level control-loop health state");
  require(contains(autonomy, "\"first_sequence\": 139, \"overruns_ms\": [5.500000,0.000000]") &&
              contains(autonomy, "\"sequence\":139,\"work_ms\":55.500000") &&
              contains(autonomy, "\"runtime_ms\":50.000000"),
          "publication must preserve intermediate overruns and their own stage context");
  require(contains(autonomy,
                   "\"far_input\": {\"required\": false, \"ready\": true, \"reason\": \"not_required\"") &&
              contains(autonomy, "\"navigation_ready\": true"),
          "rolling performance warnings must preserve native motion permission");
  require(contains(autonomy, "\"loop_ms\": {\"mean\": 49.750000") &&
              contains(autonomy, "\"p50\": 50.000000") &&
              contains(autonomy, "\"p95\": 52.500000") &&
              contains(autonomy, "\"p99\": 55.000000") && contains(autonomy, "\"max\": 60.000000"),
          "JSON snapshot must expose the completed loop-time distribution");
  require(contains(autonomy, "\"work_ms\": {\"mean\": 10.250000") &&
              contains(autonomy, "\"p50\": 9.000000") && contains(autonomy, "\"p95\": 18.500000") &&
              contains(autonomy, "\"p99\": 22.000000") && contains(autonomy, "\"max\": 24.500000"),
          "JSON snapshot must expose the control-loop work distribution");
  require(std::abs(current.obstacle_merge_ms - 4.0) < 1e-9,
          "planner merge cost must accumulate into current timing");
  require(current.status_log_ms >= 0.0 && current.status_snapshot_ms >= 0.0,
          "current status timing fields must be updated");

  fixture.state.control_loop_hold = true;
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "native loop hold snapshot must publish");
  publisher.flush();
  require(contains(fixture.writes.back(), "\"control_loop_hold\": true") &&
              contains(fixture.writes.back(), "\"navigation_ready\": false"),
          "an actual native motion hold must block admission");
  fixture.state.control_loop_hold = false;
  fixture.state.driver_control.accepted_output_sequence = 16U;
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "mismatched driver ACK snapshot must publish");
  publisher.flush();
  require(contains(fixture.writes.back(), "\"driver_delivery_accepted\": false"),
          "a different driver output sequence must not acknowledge the final output");

  fixture.state.path_active = false;
  fixture.state.driver_control.accepted_output_sequence = 17U;
  fixture.input_gate.driver_control_age_s = 0.36;
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "stale driver ACK snapshot must publish");
  publisher.flush();
  require(contains(fixture.writes.back(), "\"fresh\": false") &&
              contains(fixture.writes.back(), "\"driver_delivery_accepted\": false"),
          "an over-age driver sample must not acknowledge the final output");

  fixture.input_gate.driver_control_age_s = 0.1;
  fixture.input_gate.reason = "driver_control_stale";
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "stale input-gate reason snapshot must publish");
  publisher.flush();
  require(contains(fixture.writes.back(), "\"fresh\": false") &&
              contains(fixture.writes.back(), "\"driver_delivery_accepted\": false"),
          "a stale input-gate reason must not acknowledge the final output");

  fixture.input_gate.reason = "ready";

  fixture.state.operator_takeover_latched = true;
  fixture.state.teleop_request_active = true;
  fixture.teleop.fresh = false;
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "manual-hold snapshot must publish");
  publisher.flush();
  const std::string manual_hold = fixture.writes.back();
  require(contains(manual_hold, "\"active_cmd_source\": \"manual_hold\""),
          "stale takeover must select manual_hold source");
  require(contains(manual_hold, "\"final_cmd_vel\": {\"vx\": 0.770000"),
          "teleop request presence must select teleop final velocity");

  fixture.teleop.fresh = true;
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "fresh teleop snapshot must publish");
  publisher.flush();
  require(contains(fixture.writes.back(), "\"active_cmd_source\": \"teleop\""),
          "fresh takeover request must select teleop source");

  fixture.state.estop_latched = true;
  fixture.state.estop_reason = "test_estop";
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "estop snapshot must publish");
  publisher.flush();
  const std::string estop = fixture.writes.back();
  require(contains(estop, "\"active_cmd_source\": \"estop\""),
          "estop must have highest source precedence");
  require(contains(estop, "\"final_cmd_vel\": {\"vx\": 0.000000"),
          "estop must force a zero final velocity");
}

void testTrackingSnapshotPreservesProgressAndErrors() {
  Fixture fixture;
  fixture.local.dynamic_avoidance = "waiting";
  fixture.local.prediction_count = 2;
  fixture.local.dynamic_blocked_s = .7;
  fixture.local.tracking.active = true;
  fixture.local.tracking.trajectoryId = 47;
  fixture.local.tracking.executionTimeS = 1.25;
  fixture.local.tracking.durationS = 4.5;
  fixture.local.tracking.positionErrorM = 0.12;
  fixture.local.tracking.headingErrorRad = -0.2;
  fixture.local.tracking.endDistanceM = 2.3;
  fixture.local.tracking.executionFrozen = true;
  fixture.local.tracking.finished = false;
  fixture.local.tracking.speedLimitMps = 0.35;
  NavStatusPublisher publisher(fixture.config(), 10.0, fixture.actions(), fixture.sink());
  TimingDiagnostics previous;
  TimingDiagnostics current;
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "tracking status snapshot must publish");
  publisher.flush();
  require(contains(fixture.writes.back(), "\"dynamic_avoidance\": \"waiting\"") &&
              contains(fixture.writes.back(), "\"prediction_count\": 2") &&
              contains(fixture.writes.back(), "\"dynamic_blocked_s\": 0.700000"),
          "prediction decisions must survive asynchronous status publication");
  require(contains(fixture.writes.back(),
                   "\"tracking\": {\"active\": true, \"trajectory_id\": 47, "
                   "\"execution_time_s\": 1.250000, \"duration_s\": 4.500000, "
                   "\"position_error_m\": 0.120000, \"heading_error_rad\": -0.200000, "
                   "\"end_distance_m\": 2.300000, \"execution_frozen\": true, "
                   "\"finished\": false, \"speed_limit_mps\": 0.350000}"),
          "last_local must preserve reference-point error separately from endpoint distance");

  fixture.local.tracking = {};
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "inactive tracking status snapshot must publish");
  publisher.flush();
  require(contains(fixture.writes.back(),
                   "\"tracking\": {\"active\": false, \"trajectory_id\": 0, "
                   "\"execution_time_s\": 0.000000, \"duration_s\": 0.000000"),
          "inactive tracking must not retain the previous trajectory's progress");
}

void testOperatorMotionReadinessDeclarationFollowsProductMode() {
  Fixture fixture;
  StatusWriterConfig config = fixture.config();
  config.control_mode = "autonomy";
  config.allow_teleop_takeover = true;
  NavStatusPublisher publisher(std::move(config), 10.0, fixture.actions(), fixture.sink());
  TimingDiagnostics previous;
  TimingDiagnostics current;

  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "takeover-enabled status snapshot must publish");
  publisher.flush();
  const std::string status = fixture.writes.back();
  require(contains(status, "\"operator_motion\": {\"schema_version\": 1") &&
              contains(status, "\"interface_enabled\": true") &&
              contains(status, "\"control_mode\": \"autonomy\"") &&
              contains(status, "\"allow_teleop_takeover\": true"),
          "autonomy takeover product must advertise the compiled operator-motion interface");
}

void testConfirmedMotionStopEvidenceIsPublished() {
  Fixture fixture;
  StopConfirmationConfig config;
  config.linear_speed_threshold_mps = 0.03;
  config.angular_speed_threshold_radps = 0.08;
  config.quiet_odometry_samples = 3U;
  MotionStopEvidenceTracker evidence(config);
  evidence.begin(23U, 1234.0);
  StopConfirmationDiagnostics diagnostics;
  diagnostics.driver_ack_output_sequence = 25U;
  diagnostics.driver_ack_observed = true;
  diagnostics.driver_accepted = true;
  diagnostics.quiet_odometry_samples = 3U;
  diagnostics.required_quiet_odometry_samples = 3U;
  diagnostics.last_linear_speed_mps = 0.012;
  diagnostics.last_angular_speed_radps = 0.034;
  evidence.update(StopConfirmationState::Confirmed, diagnostics, 1235.0);
  fixture.state.motion_stop_evidence = evidence.snapshot();

  NavStatusPublisher publisher(fixture.config(), 10.0, fixture.actions(), fixture.sink());
  TimingDiagnostics previous;
  TimingDiagnostics current;
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "confirmed stop-evidence snapshot must publish");
  publisher.flush();

  const std::string &status = fixture.writes.back();
  require(contains(status,
                   "\"motion_stop_evidence\": {\"state\": \"CONFIRMED\", "
                   "\"reason\": \"stop_confirmed\", \"output_sequence\": 25, "
                   "\"updated_at_s\": 1235.000000, "
                   "\"confirmation_state\": \"confirmed\", "
                   "\"driver_ack_observed\": true, \"driver_accepted\": true, "
                   "\"quiet_odometry_samples\": 3, "
                   "\"required_quiet_odometry_samples\": 3, "
                   "\"last_linear_speed_mps\": 0.012000, "
                   "\"last_angular_speed_radps\": 0.034000, "
                   "\"linear_speed_threshold_mps\": 0.030000, "
                   "\"angular_speed_threshold_radps\": 0.080000}"),
          "confirmed stop evidence must identify the refreshed zero ACK and quiet-odometry proof");
}

void testPendingMotionStopEvidenceIsPublished() {
  Fixture fixture;
  StopConfirmationConfig config;
  config.linear_speed_threshold_mps = 0.04;
  config.angular_speed_threshold_radps = 0.09;
  config.quiet_odometry_samples = 4U;
  MotionStopEvidenceTracker evidence(config);
  evidence.begin(22U, 1234.25);
  fixture.state.motion_stop_evidence = evidence.snapshot();

  NavStatusPublisher publisher(fixture.config(), 10.0, fixture.actions(), fixture.sink());
  TimingDiagnostics previous;
  TimingDiagnostics current;
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "pending stop-evidence snapshot must publish");
  publisher.flush();
  require(contains(fixture.writes.back(),
                   "\"motion_stop_evidence\": {\"state\": \"PENDING\", "
                   "\"reason\": \"awaiting_stop_confirmation\", "
                   "\"output_sequence\": 22, \"updated_at_s\": 1234.250000, "
                   "\"confirmation_state\": \"pending\", "
                   "\"driver_ack_observed\": false, \"driver_accepted\": false, "
                   "\"quiet_odometry_samples\": 0, "
                   "\"required_quiet_odometry_samples\": 4, "
                   "\"last_linear_speed_mps\": null, "
                   "\"last_angular_speed_radps\": null, "
                   "\"linear_speed_threshold_mps\": 0.040000, "
                   "\"angular_speed_threshold_radps\": 0.090000}"),
          "pending stop evidence must identify the sequenced zero and active thresholds");
}

void testPendingMotionStopEvidenceRefreshesAtStatusCadence() {
  Fixture fixture;
  StopConfirmationConfig config;
  config.quiet_odometry_samples = 3U;
  MotionStopEvidenceTracker evidence(config);
  evidence.begin(24U, 1234.0);
  fixture.state.motion_stop_evidence = evidence.snapshot();

  NavStatusPublisher publisher(fixture.config(), 0.2, fixture.actions(), fixture.sink());
  TimingDiagnostics previous;
  TimingDiagnostics current;
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "pending stop-evidence entry must publish immediately");
  publisher.flush();

  StopConfirmationDiagnostics diagnostics;
  diagnostics.driver_ack_observed = true;
  diagnostics.driver_accepted = true;
  diagnostics.quiet_odometry_samples = 2U;
  diagnostics.required_quiet_odometry_samples = 3U;
  diagnostics.last_linear_speed_mps = 0.01;
  diagnostics.last_angular_speed_radps = 0.02;
  evidence.update(StopConfirmationState::Pending, diagnostics, 1234.15);
  fixture.state.motion_stop_evidence = evidence.snapshot();

  fixture.steady_s = 10.19;
  require(!publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "pending progress must preserve the normal status cadence");
  fixture.steady_s = 10.2;
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "pending progress must refresh at the field status cadence");
  publisher.flush();

  require(fixture.writes.size() == 2U,
          "pending refresh must add exactly one cadence-limited snapshot");
  require(contains(fixture.writes.back(),
                   "\"motion_stop_evidence\": {\"state\": \"PENDING\", ") &&
              contains(fixture.writes.back(), "\"driver_ack_observed\": true") &&
              contains(fixture.writes.back(), "\"quiet_odometry_samples\": 2"),
          "pending refresh must expose current ACK and quiet-odometry progress");
}

void testFailedMotionStopEvidenceKeepsExactFailure() {
  Fixture fixture;
  StopConfirmationConfig config;
  config.quiet_odometry_samples = 2U;
  MotionStopEvidenceTracker evidence(config);
  StopConfirmationDiagnostics diagnostics;
  diagnostics.driver_ack_observed = true;
  diagnostics.driver_accepted = false;
  diagnostics.required_quiet_odometry_samples = 2U;

  evidence.begin(31U, 2000.0);
  evidence.update(StopConfirmationState::DriverRejected, diagnostics, 2000.1);
  fixture.state.motion_stop_evidence = evidence.snapshot();
  NavStatusPublisher publisher(fixture.config(), 10.0, fixture.actions(), fixture.sink());
  TimingDiagnostics previous;
  TimingDiagnostics current;
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "driver-rejected stop-evidence snapshot must publish");
  publisher.flush();
  require(contains(fixture.writes.back(),
                   "\"motion_stop_evidence\": {\"state\": \"FAILED\", "
                   "\"reason\": \"driver_rejected\", \"output_sequence\": 31") &&
              contains(fixture.writes.back(),
                       "\"confirmation_state\": \"driver_rejected\", "
                       "\"driver_ack_observed\": true, \"driver_accepted\": false"),
          "driver rejection must remain distinct and retain the rejected ACK");

  diagnostics.driver_ack_observed = false;
  evidence.begin(32U, 2001.0);
  evidence.update(StopConfirmationState::TimedOut, diagnostics, 2005.0);
  fixture.state.motion_stop_evidence = evidence.snapshot();
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "timed-out stop-evidence snapshot must publish");
  publisher.flush();
  require(contains(fixture.writes.back(),
                   "\"motion_stop_evidence\": {\"state\": \"FAILED\", "
                   "\"reason\": \"timed_out\", \"output_sequence\": 32") &&
              contains(fixture.writes.back(), "\"confirmation_state\": \"timed_out\""),
          "stop timeout must remain distinct from driver rejection");
}

void testZeroKeepalivePreservesConfirmedMotionStopEvidence() {
  Fixture fixture;
  MotionStopEvidenceTracker evidence;
  StopConfirmationDiagnostics diagnostics;
  diagnostics.driver_ack_observed = true;
  diagnostics.driver_accepted = true;
  diagnostics.quiet_odometry_samples = 8U;
  diagnostics.required_quiet_odometry_samples = 8U;
  diagnostics.last_linear_speed_mps = 0.0;
  diagnostics.last_angular_speed_radps = 0.0;
  evidence.begin(40U, 3000.0);
  evidence.update(StopConfirmationState::Confirmed, diagnostics, 3000.5);
  evidence.observePublishedFinalOutput(41U, nav_kernel::Twist{}, 3001.0);
  fixture.state.motion_stop_evidence = evidence.snapshot();

  NavStatusPublisher publisher(fixture.config(), 10.0, fixture.actions(), fixture.sink());
  TimingDiagnostics previous;
  TimingDiagnostics current;
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "zero-keepalive stop-evidence snapshot must publish");
  publisher.flush();
  require(contains(fixture.writes.back(),
                   "\"motion_stop_evidence\": {\"state\": \"CONFIRMED\", "
                   "\"reason\": \"stop_confirmed\", \"output_sequence\": 40, "
                   "\"updated_at_s\": 3000.500000"),
          "a later zero keepalive must preserve the completed confirmation identity and time");
}

void testNonzeroOutputInvalidatesConfirmedMotionStopEvidence() {
  Fixture fixture;
  MotionStopEvidenceTracker evidence;
  StopConfirmationDiagnostics diagnostics;
  diagnostics.driver_ack_observed = true;
  diagnostics.driver_accepted = true;
  diagnostics.quiet_odometry_samples = 8U;
  diagnostics.required_quiet_odometry_samples = 8U;
  diagnostics.last_linear_speed_mps = 0.01;
  diagnostics.last_angular_speed_radps = 0.02;
  evidence.begin(50U, 4000.0);
  evidence.update(StopConfirmationState::Confirmed, diagnostics, 4000.5);
  nav_kernel::Twist moving;
  moving.vx = 0.2;
  evidence.observePublishedFinalOutput(51U, moving, 4001.0);
  fixture.state.motion_stop_evidence = evidence.snapshot();

  NavStatusPublisher publisher(fixture.config(), 10.0, fixture.actions(), fixture.sink());
  TimingDiagnostics previous;
  TimingDiagnostics current;
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "nonzero-output stop-evidence snapshot must publish");
  publisher.flush();
  require(contains(fixture.writes.back(),
                   "\"motion_stop_evidence\": {\"state\": \"NOT_REQUESTED\", "
                   "\"reason\": \"nonzero_output_published\", \"output_sequence\": 0, "
                   "\"updated_at_s\": 4001.000000, "
                   "\"confirmation_state\": \"not_requested\", "
                   "\"driver_ack_observed\": false, \"driver_accepted\": false, "
                   "\"quiet_odometry_samples\": 0"),
          "a later nonzero final output must invalidate all evidence from the prior stop");
}

void testFarInputReadinessControlsNavigationReadiness() {
  Fixture fixture;
  fixture.loop_health.healthy = true;
  fixture.loop_health.reason = "healthy";
  auto actions = fixture.actions();
  actions.sample_far_input = []() {
    return StatusFarInputSample{true, false, "required occupancy artifact missing", "yard", 4};
  };
  auto config = fixture.config();
  config.global_planner = "far";
  config.publish_cmd_vel = true;
  NavStatusPublisher publisher(std::move(config), 10.0, std::move(actions), fixture.sink());
  TimingDiagnostics previous;
  TimingDiagnostics current;
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "FAR status snapshot must publish when occupancy is unavailable");
  publisher.flush();
  const auto blocked = fixture.writes.back();
  require(contains(blocked,
                   "\"far_input\": {\"required\": true, \"ready\": false, \"reason\": \"required occupancy artifact missing\", \"map_id\": \"yard\", \"content_epoch\": 4}") &&
              contains(blocked, "\"navigation_ready\": false"),
          "missing FAR occupancy must be explicit and must block navigation readiness");

  Fixture ready_fixture;
  ready_fixture.loop_health.healthy = true;
  ready_fixture.loop_health.reason = "healthy";
  auto ready_actions = ready_fixture.actions();
  ready_actions.sample_far_input = []() {
    return StatusFarInputSample{true, true, "ready", "yard", 5};
  };
  auto ready_config = ready_fixture.config();
  ready_config.global_planner = "far";
  ready_config.publish_cmd_vel = true;
  NavStatusPublisher ready_publisher(std::move(ready_config), 10.0,
                                     std::move(ready_actions), ready_fixture.sink());
  ready_publisher.requestImmediate();
  require(ready_publisher.publishIfDue(ready_fixture.state, ready_fixture.commands, previous, current),
          "valid FAR occupancy status snapshot must publish");
  ready_publisher.flush();
  require(contains(ready_fixture.writes.back(),
                   "\"far_input\": {\"required\": true, \"ready\": true, \"reason\": \"ready\", \"map_id\": \"yard\", \"content_epoch\": 5}") &&
              contains(ready_fixture.writes.back(), "\"navigation_ready\": true"),
          "validated FAR occupancy must make the FAR input ready");
}

void testScanFailureCaptureAndSafetyReasonSurviveStatusReset() {
  Fixture fixture;
  fixture.local_debug.backend = nav_kernel::LocalPlannerBackend::Scan;
  auto failure = std::make_shared<nav_kernel::ScanFailureSnapshot>();
  failure->sequence = 7U;
  failure->clock.timestampS = 12.0;
  failure->attempt.attempted = true;
  failure->attempt.attemptId = 3U;
  failure->attempt.stage = "dynamic_feasibility";
  failure->attempt.reason = "velocity_limit_exceeded";
  failure->attempt.dynamicViolationValid = true;
  failure->attempt.dynamicQuantity = "speed";
  failure->attempt.dynamicValue = 1.1;
  failure->attempt.dynamicLimit = 0.8;
  failure->attempt.dynamicTimeS = 0.0;
  failure->collision = fixture.state.local_collision_map;
  failure->collision.inflatedStorage = std::make_shared<const std::vector<std::uint8_t>>(
      failure->collision.inflatedBits,
      failure->collision.inflatedBits + failure->collision.inflatedBytes);
  failure->collision.inflatedBits = failure->collision.inflatedStorage->data();
  failure->collision.gridFromPlanningTranslation = {3.0, 4.0, 1.0};
  failure->collision.gridFromPlanningYaw = 0.5;
  // Failure evidence must not inherit the 512-point periodic path preview cap.
  failure->reference.assign(514U, {1.0, 2.0, 3.0});
  failure->reference.back() = {123.0, 456.0, 789.0};
  failure->candidateControlPoints = {{0.0, 0.0, 0.4}, {1.0, 0.0, 0.4}};
    failure->candidateIntervalS = 0.2;
    failure->predictions = {{{1,2,.5},{2,2,.5},.25,.1,1.2}};
    failure->predictionsObservedAtS = 11.9;
    failure->predictionsHorizonS = 1.0;
  fixture.local_debug.scanAttempt = failure->attempt;
  fixture.local_debug.lastScanFailure = failure;
  fixture.teleop.last_safety_replan.count = 2U;
  fixture.teleop.last_safety_replan.stamp_steady_s = 11.5;
  fixture.teleop.last_safety_replan.reason = "scan_measured_braking_collision";
  fixture.teleop.last_safety_replan.candidate_cmd_vel.vx = 0.3;

  std::mutex writes_mutex;
  std::vector<std::string> failure_writes;
  auto sink = [&](const std::filesystem::path &path, const std::string &snapshot) {
    std::lock_guard<std::mutex> lock(writes_mutex);
    if (path.extension() == ".json" && contains(path.string(), ".scan-failure.json"))
      failure_writes.push_back(snapshot);
    else fixture.writes.push_back(snapshot);
    return true;
  };
  auto config = fixture.config();
  config.local_planner = "scan";
  NavStatusPublisher publisher(config, 1.0, fixture.actions(), sink);
  TimingDiagnostics previous, current;
  publisher.requestImmediate();
  require(publisher.publishIfDue(fixture.state, fixture.commands, previous, current),
          "failed SCAN tick must publish status and its immutable input");
  fixture.collision_bitmap.clear();
  publisher.flush();
  require(failure_writes.size() == 1U, "first failure must write a separate snapshot");
  const auto &saved = failure_writes.front();
  require(contains(saved, "\"product_session_id\": \"product-session-test\"") &&
              contains(saved, "\"coordinate_frame\": \"request_planning_frame\"") &&
              contains(saved, "\"grid_from_planning_translation\": [3, 4, 1]") &&
              contains(saved, "\"grid_from_planning_yaw\": 0.5"),
          "failure snapshot must preserve session and request-to-grid transform");
  require(contains(saved, "[123, 456, 789]") &&
              contains(saved, "\"candidate_control_points\": [[0, 0,") &&
              contains(saved, "\"quantity\": \"speed\", \"value\": 1.1"),
          "failure input must retain full reference, candidate and measured violation");
    const auto bits_start = saved.find("\"bits\": \"") + std::string("\"bits\": \"").size();
    require(contains(saved, "\"predictions\": {\"observed_at_s\": 11.9") &&
                contains(saved, "\"horizon_s\": 1, \"volumes\": [{\"start\": [1, 2, 0.5]") &&
                contains(saved, "\"radius\": 0.25") && contains(saved, "\"cylinderRadius\":"),
            "failure evidence must retain timed prediction geometry and body radius");
  const auto bits_end = saved.find('"', bits_start);
  const auto bits = saved.substr(bits_start, bits_end - bits_start);
  require(bits.size() == failure->collision.inflatedBytes * 2U,
          "captured bitmap must serialize every byte");
  constexpr char hex[] = "0123456789abcdef";
  for (std::size_t i = 0; i < failure->collision.inflatedBytes; ++i) {
    const auto byte = failure->collision.inflatedBits[i];
    require(bits[2U * i] == hex[byte >> 4U] && bits[2U * i + 1U] == hex[byte & 15U],
            "bitmap encoding must preserve exact immutable bytes after live map mutation");
  }
  require(!contains(fixture.writes.back(), "\"bits\":") &&
              contains(fixture.writes.back(), "\"search_ms\": null") &&
              contains(fixture.writes.back(), "\"optimizer_evaluations\": null") &&
              contains(fixture.writes.back(), "\"last_safety_replan\": {\"count\": 2") &&
              contains(fixture.writes.back(), "scan_measured_braking_collision"),
          "periodic status must keep the stop cause and not fabricate SCAN measurements or repeat the bitmap");

  fixture.local_debug = {};
  publisher.requestImmediate();
  publisher.publishIfDue(fixture.state, fixture.commands, previous, current);
  publisher.flush();
  require(failure_writes.size() == 1U &&
              contains(fixture.writes.back(), "\"last_scan_failure\": {\"sequence\": 7"),
          "asynchronous Task reset must retain the failure summary without rewriting its map");

  auto next = std::make_shared<nav_kernel::ScanFailureSnapshot>(*failure);
  next->sequence = 8U;
  next->attempt.stage = "rebound_optimization";
  next->attempt.reason = "start_segment_collision";
  next->attempt.dynamicViolationValid = false;
  next->attempt.collisionValid = true;
  next->attempt.collisionPosition = {2.0, 3.0, 0.4};
  next->attempt.collisionState = 1;
  next->attempt.collisionTimeS = -1.0;
  fixture.local_debug.lastScanFailure = next;
  publisher.requestImmediate();
  publisher.publishIfDue(fixture.state, fixture.commands, previous, current);
  publisher.flush();
  require(failure_writes.size() == 2U && contains(failure_writes.back(), "\"sequence\": 8") &&
              contains(failure_writes.back(), "\"trajectory_time_s\": null, \"state\": 1") &&
              contains(failure_writes.back(), "\"dynamic_violation\": null"),
          "a new episode must replace saved evidence and distinguish collision from dynamic limits");
}

void testScanFailureNonfiniteValueIsJsonNull() {
  nav_kernel::ScanFailureSnapshot failure;
  failure.attempt.attempted = true;
  failure.attempt.dynamicViolationValid = true;
  failure.attempt.dynamicValue = std::numeric_limits<double>::quiet_NaN();
  failure.attempt.collisionValid = true;
  failure.attempt.collisionPosition.x = std::numeric_limits<double>::quiet_NaN();
  const auto json = lingtu::nav::endpoint::scanFailureSnapshotJson(failure, "test");
  require(contains(json, "\"value\": null") && contains(json, "\"position\": [null, 0, 0]") &&
              !contains(json, ": nan"),
          "nonfinite optimizer evidence must remain valid JSON without becoming a measured zero");
}

}  // namespace

int main() {
  testScanFailureCaptureAndSafetyReasonSurviveStatusReset();
  testScanFailureNonfiniteValueIsJsonNull();
  testQueuedSnapshotOwnsCollisionBytesAfterInputChanges();
  testCadenceImmediateDisabledAndEmptyFileLog();
  testMissingCloudPoseSerializesUnavailableGap();
  testStatusSnapshotPreservesPrecedenceCountersFreshnessAndTiming();
  testTrackingSnapshotPreservesProgressAndErrors();
  testOperatorMotionReadinessDeclarationFollowsProductMode();
  testPendingMotionStopEvidenceIsPublished();
  testPendingMotionStopEvidenceRefreshesAtStatusCadence();
  testConfirmedMotionStopEvidenceIsPublished();
  testFailedMotionStopEvidenceKeepsExactFailure();
  testZeroKeepalivePreservesConfirmedMotionStopEvidence();
  testNonzeroOutputInvalidatesConfirmedMotionStopEvidence();
  testFarInputReadinessControlsNavigationReadiness();
  return 0;
}
