#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "input/projector.hpp"
#include "input/planner.hpp"
#include "dds/codec.hpp"
#include "input/obstacle.hpp"
namespace {

using lingtu::nav::endpoint::EndpointState;
using lingtu::nav::endpoint::InputGate;
using lingtu::nav::endpoint::InputGateConfig;
using lingtu::nav::endpoint::kMaxDynamicPredictionPoints;
using lingtu::nav::endpoint::MotionLayer;
using lingtu::nav::endpoint::MotionLayerConfig;
using CoreInputProjector = lingtu::nav::endpoint::InputProjector;
using lingtu::nav::endpoint::InputActions;
using lingtu::nav::endpoint::InputConfig;
using lingtu::nav::endpoint::ClearSource;
using lingtu::nav::endpoint::SteadyClock;
using lingtu::nav::endpoint::TimingDiagnostics;
using lingtu::nav::endpoint::TransformBuffer;

class InputProjector : public CoreInputProjector {
 public:
  using CoreInputProjector::CoreInputProjector;

  void projectTf(const lingtu_dds_TFMessage &message, double receive_steady_s) {
    CoreInputProjector::projectTf(lingtu::nav::endpoint::copyTransformSample(message),
                                  receive_steady_s);
  }

  void projectOdometry(const lingtu_dds_Odometry &message, double receive_steady_s) {
    CoreInputProjector::projectOdometry(lingtu::nav::endpoint::copyOdometrySample(message),
                                        receive_steady_s);
  }

  void projectCloud(const lingtu_dds_PointCloud2 &message, double receive_steady_s,
                    double receive_wall_s, TimingDiagnostics &timing) {
    CoreInputProjector::projectCloud(
        lingtu::nav::endpoint::copyPointCloudSample(message, false), receive_steady_s,
        receive_wall_s, timing);
  }

  void projectTerrainMap(const lingtu_dds_PointCloud2 &message, double receive_steady_s,
                         TimingDiagnostics &timing) {
    CoreInputProjector::projectTerrainMap(
        lingtu::nav::endpoint::copyPointCloudSample(message, true), receive_steady_s, timing);
  }

  void projectTerrainMapExt(const lingtu_dds_PointCloud2 &message, double receive_steady_s,
                            TimingDiagnostics &timing) {
    CoreInputProjector::projectTerrainMapExt(
        lingtu::nav::endpoint::copyPointCloudSample(message, true), receive_steady_s, timing);
  }

  void projectTraversability(const lingtu_dds_OccupancyGrid &message, double receive_steady_s) {
    CoreInputProjector::projectTraversability(
        lingtu::nav::endpoint::copyGridSample(message, "map"), receive_steady_s);
  }

  void projectLocalTraversability(const lingtu_dds_OccupancyGrid &message,
                                  double receive_steady_s) {
    CoreInputProjector::projectLocalTraversability(
        lingtu::nav::endpoint::copyGridSample(message, "odom"), receive_steady_s);
  }

  void projectLocalCollision(const lingtu_dds_MapCollisionLayer &message,
                             double receive_steady_s) {
    CoreInputProjector::projectLocalCollision(
        lingtu::nav::endpoint::copyLocalCollisionSample(message), receive_steady_s);
  }

  void projectDriverControl(const lingtu_dds_DriverControlState &message,
                            SteadyClock::time_point receive_time) {
    const double receive_steady_s =
        std::chrono::duration<double>(receive_time.time_since_epoch()).count();
    CoreInputProjector::projectDriverControl(
        lingtu::nav::endpoint::copyDriverControlSample(message), receive_steady_s);
  }

  void projectLocalizationHealth(const lingtu_dds_Text &message, double receive_steady_s) {
    CoreInputProjector::projectLocalizationHealth(
        lingtu::nav::endpoint::textData(message), receive_steady_s);
  }

  void clearPlannerInputs(const lingtu_dds_Bool &message, ClearSource source) {
    CoreInputProjector::clearPlannerInputs({source, message.data});
  }
};

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

lingtu_dds_TFMessage tfMessage(lingtu_dds_TransformStamped &transform, double stamp_s,
                               double x = 0.0, const char *parent_frame_id = "map",
                               const char *child_frame_id = "odom") {
  transform = {};
  transform.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  transform.header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_s - static_cast<double>(transform.header.stamp.sec)) * 1e9);
  transform.header.frame_id = const_cast<char *>(parent_frame_id);
  transform.child_frame_id = const_cast<char *>(child_frame_id);
  transform.transform.translation.x = x;
  transform.transform.rotation.w = 1.0;

  lingtu_dds_TFMessage message{};
  message.transforms._length = 1;
  message.transforms._maximum = 1;
  message.transforms._buffer = &transform;
  return message;
}
lingtu_dds_Odometry odometryMessage(double stamp_s, const char *frame_id, double x = 0.0,
                                    const char *child_frame_id = "body") {
  lingtu_dds_Odometry message{};
  message.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  message.header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_s - static_cast<double>(message.header.stamp.sec)) * 1e9);
  message.header.frame_id = const_cast<char *>(frame_id);
  message.child_frame_id = const_cast<char *>(child_frame_id);
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
  message.owner = const_cast<char *>("driver");
  message.owner_id = const_cast<char *>("lingtu-driver@robot");
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

struct CollisionFixture {
  PointCloudFixture cloud;
  lingtu_dds_MapCollisionLayer message{};

  CollisionFixture(double stamp_s,
                   std::uint64_t observation_sequence,
                   std::uint64_t generation,
                   bool complete)
      : cloud(stamp_s, "map", 1.0F, 2.0F, 0.5F) {
    message.header = cloud.message.header;
    message.reset_epoch = 3;
    message.observation_sequence = observation_sequence;
    message.generation = generation;
    message.live = true;
    message.resolution = 0.10F;
    message.aabb_min.x = -5.0;
    message.aabb_min.y = -5.0;
    message.aabb_min.z = -2.0;
    message.aabb_max.x = 5.0;
    message.aabb_max.y = 5.0;
    message.aabb_max.z = 2.0;
    message.complete = complete;
    message.occupied = cloud.message;
  }
};

std::vector<float> movingClusterWithNearHazard(float x) {
  std::vector<float> out;
  for (int i = 0; i < 12; ++i) {
    out.insert(out.end(), {x, 0.05f + 0.10f * static_cast<float>(i), 0.05f, 0.4f});
  }
  out.insert(out.end(), {0.25f, -0.55f, 0.05f, 0.4f});
  return out;
}

MotionLayerConfig layerConfig(double voxel_size_m, double decay_s,
                              double inflation_radius_m, double ray_clear_max_range_m,
                              double ray_clearing_interval_s, std::size_t max_clearing_rays,
                              int min_hits, bool ray_clearing_enabled) {
  MotionLayerConfig config;
  config.voxel_size_m = voxel_size_m;
  config.decay_s = decay_s;
  config.inflation_radius_m = inflation_radius_m;
  config.ray_clear_max_range_m = ray_clear_max_range_m;
  config.ray_clearing_interval_s = ray_clearing_interval_s;
  config.max_clearing_rays = max_clearing_rays;
  config.min_hits = min_hits;
  config.ray_clearing_enabled = ray_clearing_enabled;
  return config;
}

std::vector<float> dynamicFreeRays() {
  std::vector<float> out;
  for (const float target_x : {1.05f, 1.15f, 1.25f, 1.35f}) {
    for (int i = 0; i < 12; ++i) {
      const float target_y = 0.05f + 0.10f * static_cast<float>(i);
      out.insert(out.end(), {3.05f, target_y * 3.05f / target_x, 0.05f, 0.4f});
    }
  }
  return out;
}

bool containsNearXy(const std::vector<float> &xyzh, float x, float y, float eps) {
  for (std::size_t offset = 0; offset + 3 < xyzh.size(); offset += 4) {
    if (std::hypot(xyzh[offset] - x, xyzh[offset + 1] - y) <= eps) {
      return true;
    }
  }
  return false;
}

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
  require(state.frame_epoch == 1, "the initial live map frame must have a non-zero epoch identity");
  InputGate input_gate;
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  MotionLayer live_obstacles;
  InputProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   InputConfig{}, InputActions{});

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
  require(state.frame_epoch == 2, "TF clock rebase must start a new input epoch");
  require(state.frames.clock_rebases == 1, "TF clock rebase must increment diagnostics");
  require(state.map_odom_epoch_start_s == 9.0,
          "TF clock rebase must seed the new epoch with the triggering source clock");
}

void testEpochResetClearsInputsBeforeSynchronousEffectsAndAcceptsTriggeringTf() {
  EndpointState state;
  InputGate input_gate;
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  MotionLayer live_obstacles;
  bool callback_called = false;
  bool callback_clear_motion = false;
  InputActions actions;
  actions.on_epoch_reset = [&](double epoch_start_s, const std::string &reason, bool clear_motion) {
    callback_called = true;
    callback_clear_motion = clear_motion;
    require(std::abs(epoch_start_s - 11.0) < 1e-9, "epoch callback must carry source stamp");
    require(reason == "map_frame_jump", "epoch callback must preserve reset reason");
    require(state.frame_epoch == 2, "input epoch must increment before external effects");
    require(state.tf_generation == 1, "triggering TF generation must advance after effects");
    require(!state.map_odom_tf, "old map transform must clear before external effects");
    require(!state.odom_body, "old odometry must clear before external effects");
    require(state.terrain_xyzh.empty(), "terrain must clear before external effects");
    require(state.last_cloud_s == 0.0, "cloud freshness must clear before external effects");
    require(!map_odom_buffer.sample(10.0, 1.0),
            "transform history must clear before external effects");
  };
  InputProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   InputConfig{}, std::move(actions));

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
  MotionLayer map_obstacles;
  InputProjector map_projector(map_state, map_gate, map_pose_buffer, map_tf_buffer,
                                       map_obstacles, InputConfig{},
                                       InputActions{});

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
  MotionLayer odom_obstacles;
  InputProjector odom_projector(odom_state, odom_gate, odom_pose_buffer, odom_tf_buffer,
                                        odom_obstacles, InputConfig{},
                                        InputActions{});

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

void testOdometryRejectsDuplicateSourceStampWithoutPoisoningFiniteState() {
  using lingtu::nav::endpoint::SourceStampDecision;
  using lingtu::nav::endpoint::classifySourceOrder;

  require(classifySourceOrder(10.0, 10.0, 0.35) == SourceStampDecision::kAccept,
          "the shared source-order contract must remain unchanged for other input streams");

  EndpointState state;
  InputGateConfig gate_config;
  gate_config.require_cloud = false;
  gate_config.recovery_frames = 1;
  InputGate input_gate(gate_config);
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  MotionLayer live_obstacles;
  InputProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   InputConfig{}, InputActions{});

  auto initial = odometryMessage(10.0, "map", 0.0);
  projector.projectOdometry(initial, 100.0);
  require(state.odom_generation == 1 && std::isfinite(state.last_odom_linear_speed_mps),
          "the initial odometry sample must establish finite accepted state");

  auto duplicate = odometryMessage(10.0, "map", 1.0);
  projector.projectOdometry(duplicate, 100.1);

  require(state.frames.odom_rejected == 1 &&
              state.frames.last_error == "odom_stamp_not_advanced",
          "duplicate odometry source stamps must be rejected at the projector seam");
  require(state.odom_count == 1 && state.odom_generation == 1,
          "duplicate odometry must not advance accepted state");
  require(std::abs(state.last_odom_s - 10.0) < 1e-9 &&
              std::abs(state.last_odom_receive_s - 100.0) < 1e-9,
          "duplicate odometry must preserve the prior source and receipt clocks");
  require(state.map_body && std::abs(state.map_body->position.x) < 1e-9 &&
              std::isfinite(state.last_odom_linear_speed_mps),
          "duplicate odometry must preserve the prior pose and finite speed evidence");

  const auto gate_state = projector.evaluateGate(100.1, steadyTime(100.1));
  require(gate_state.ready && gate_state.reason == "ready",
          "a rejected duplicate must not poison the input gate with non-finite velocity");
}

void testOdometryRejectsNonCanonicalChildFrames() {
  EndpointState state;
  InputGate input_gate;
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  MotionLayer live_obstacles;
  InputProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   InputConfig{}, InputActions{});

  auto empty_child = odometryMessage(10.0, "map", 0.0, "");
  projector.projectOdometry(empty_child, 100.0);
  require(state.frames.odom_rejected == 1 &&
              state.frames.last_error == "odom_child_frame_empty",
          "empty odometry child frame must fail closed");

  auto model_alias = odometryMessage(10.0, "map", 0.0, "base_link");
  projector.projectOdometry(model_alias, 100.0);
  require(state.frames.odom_rejected == 2 &&
              state.frames.last_error == "odom_child_frame_unsupported",
          "base_link must be normalized by an adapter before native ingress");

  auto sensor_pose = odometryMessage(10.0, "map", 0.0, "camera");
  projector.projectOdometry(sensor_pose, 100.0);
  require(state.frames.odom_rejected == 3 &&
              state.frames.last_error == "odom_child_frame_unsupported",
          "sensor poses must not be accepted as robot odometry");
  require(state.odom_count == 0 && state.odom_generation == 0,
          "rejected child frames must not mutate accepted odometry state");
}

void testTfRejectsReverseMapOdomPayload() {
  EndpointState state;
  InputGate input_gate;
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  MotionLayer live_obstacles;
  InputProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   InputConfig{}, InputActions{});

  lingtu_dds_TransformStamped transform{};
  auto reverse_tf = tfMessage(transform, 10.0, 2.0, "odom", "map");
  projector.projectTf(reverse_tf, 100.0);

  require(state.tf_count == 1 && state.tf_generation == 0,
          "reverse odom-from-map wire payload must be counted but not accepted");
  require(state.frames.last_error == "map_odom_tf_invalid",
          "reverse TF rejection must remain observable");
  require(!state.map_odom_tf.has_value(),
          "reverse TF must not be silently inverted into canonical state");
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
  MotionLayer live_obstacles;
  InputConfig projector_config;
  projector_config.driver_control_max_age_s = 0.35;
  InputProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   projector_config, InputActions{});

  require(projector.driverBlocker(steadyTime(1000.0)) == "driver_control_missing",
          "missing driver control must have highest blocker precedence");

  auto disconnected = driverControlMessage(10.0);
  disconnected.ready = true;
  disconnected.motors_enabled = true;
  disconnected.control_assured = true;
  disconnected.lease_valid = true;
  projector.projectDriverControl(disconnected, steadyTime(1000.0));
  require(state.driver_control_generation == 1, "accepted driver state must advance generation");
  require(state.driver_control_reason == "disconnected",
          "fallback reason precedence must start disconnected");
  require(projector.driverBlocker(steadyTime(1000.1)) == "driver_control_disconnected",
          "fresh not-ready driver state must expose its detailed blocker");

  char producer_reason[] = "producer_fault";
  char producer_boot[] = "boot-a";
  auto rejected = driverControlMessage(10.05);
  rejected.connected = true;
  rejected.ready = false;
  rejected.motors_enabled = true;
  rejected.control_assured = true;
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
  ready.control_assured = true;
  ready.lease_valid = false;
  projector.projectDriverControl(ready, steadyTime(1000.2));
  require(state.driver_control_ready, "fully ready driver state must admit motion");
  require(projector.driverBlocker(steadyTime(1000.3)).empty(),
          "fresh ready driver state must not block motion");
  require(std::abs(projector.driverAge(steadyTime(1000.3)) - 0.1) < 1e-9,
          "driver freshness must use the injected steady clock");

  const auto gate_state = projector.evaluateGate(1000.3, steadyTime(1000.3));
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
  require(projector.driverBlocker(steadyTime(1001.0)) == "driver_control_stale",
          "driver staleness must precede readiness details");
}

void testCloudTerrainAndSnapshotProjectionOwnDdsDataAndExactClocks() {
  EndpointState state;
  InputGate input_gate;
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  MotionLayer live_obstacles(
      layerConfig(0.10, 1.0, 0.0, 4.0, 0.0, 100, 1, false));
  InputConfig config;
  config.sensor_offset = {0.2, 0.0, 0.0};
  config.max_obstacle_points = 100;
  InputProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   config, InputActions{});

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
  require(projector.materializeObstacles(timing),
          "dirty live obstacle layer must materialize once");
  require(!state.obstacle_xyzh.empty(), "materialized live obstacle snapshot must be available");
  require(std::abs(state.obstacle_xyzh.front() - 1.0f) < 0.11f,
          "materialized obstacles must not retain the DDS loan buffer");
  require(!state.obstacle_snapshot_dirty, "materialization must clear the dirty flag");
  require(!projector.materializeObstacles(timing),
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
  MotionLayer live_obstacles;
  std::vector<std::string> invalidation_reasons;
  InputActions actions;
  actions.on_rolling_snapshot_invalidated = [&](const std::string &reason) {
    require(state.terrain_xyzh.empty(), "planner inputs must clear before rolling invalidation");
    require(state.obstacle_xyzh.empty(), "obstacles must clear before rolling invalidation");
    invalidation_reasons.push_back(reason);
    state.frames.last_error = reason;
  };
  InputProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   InputConfig{}, std::move(actions));

  state.terrain_xyzh = {1.0f, 0.0f, 0.0f, 0.0f};
  state.terrain_ext_xyzh = state.terrain_xyzh;
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
  projector.clearPlannerInputs(no_clear, ClearSource::Map);
  require(state.map_clearing_count == 0, "false map clear must be a no-op");
  require(!state.terrain_xyzh.empty(), "false map clear must preserve planner inputs");

  lingtu_dds_Bool clear{};
  clear.data = true;
  projector.clearPlannerInputs(clear, ClearSource::Map);
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
  projector.clearPlannerInputs(clear, ClearSource::Cloud);
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
  MotionLayer live_obstacles;
  InputProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   InputConfig{}, InputActions{});

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

void testCloudClockRebaseClearsOldMotionLayerAndDerivedSnapshots() {
  EndpointState state;
  InputGate input_gate;
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  MotionLayer live_obstacles(
      layerConfig(0.10, 2000.0, 0.0, 4.0, 0.0, 100, 1, true));
  InputProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                           InputConfig{}, InputActions{});

  auto old_odom = odometryMessage(1000.0, "map", 0.0);
  projector.projectOdometry(old_odom, 100.0);
  PointCloudFixture old_cloud(1000.0, "map", 1.0f, 0.0f, 0.2f);
  TimingDiagnostics timing;
  projector.projectCloud(old_cloud.message, 100.0, 1000.0, timing);
  require(projector.materializeObstacles(timing), "old cloud must materialize an obstacle");
  require(containsNearXy(state.obstacle_xyzh, 1.0f, 0.0f, 0.11f),
          "old obstacle fixture must be present before clock rebase");
  state.predicted_obstacle_xyzh = {1.0f, 0.0f, 0.2f, 0.4f};

  pose_buffer.clear();
  auto rebased_odom = odometryMessage(10.0, "map", 0.0);
  const auto pose = lingtu::nav::endpoint::rigidTransformFromOdometry(rebased_odom);
  require(pose.valid, "rebased pose fixture must decode");
  pose_buffer.push(10.0, pose);
  PointCloudFixture rebased_cloud(10.0, "map", 2.0f, 0.0f, 0.2f);
  projector.projectCloud(rebased_cloud.message, 101.0, 1001.0, timing);

  require(state.frames.clock_rebases == 1, "accepted cloud rollback must count as a clock rebase");
  require(state.obstacle_xyzh.empty() && state.predicted_obstacle_xyzh.empty(),
          "cloud clock rebase must clear all derived obstacle snapshots before rematerialization");
  require(projector.materializeObstacles(timing), "rebased cloud must materialize a new snapshot");
  require(!containsNearXy(state.obstacle_xyzh, 1.0f, 0.0f, 0.11f),
          "pre-rebase obstacle must not survive the new source clock");
  require(containsNearXy(state.obstacle_xyzh, 2.0f, 0.0f, 0.11f),
          "triggering rebased scan must seed the cleared MotionLayer");
}

void testLocalCollisionProjectionOwnsPayloadAndRejectsPreClearReplay() {
  EndpointState state;
  InputGate input_gate;
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  MotionLayer live_obstacles;
  InputProjector projector(
      state,
      input_gate,
      pose_buffer,
      map_odom_buffer,
      live_obstacles,
      InputConfig{},
      InputActions{});

  CollisionFixture layer(22.0, 7, 9, false);
  projector.projectLocalCollision(layer.message, 202.0);
  require(state.local_collision_count == 1,
          "accepted collision layer must increment its counter");
  require(state.local_collision_map.occupied_xyz.size() == 3,
          "collision projection must own every occupied center");
  require(!state.local_collision_map.complete,
          "an incomplete wire layer must remain explicitly incomplete");
  require(state.local_collision_map.observation_sequence == 7 &&
              state.local_collision_map.generation == 9,
          "collision identity must survive projection");
  require(std::abs(state.last_local_collision_receive_s - 202.0) < 1e-9,
          "collision freshness must use explicit steady receipt time");
  require(std::abs(state.local_collision_map.view().receiveStampS - 202.0) < 1e-9,
          "planner collision view must carry receiver-clock freshness");
  layer.cloud.setPoint(9.0F, 9.0F, 9.0F);
  require(std::abs(state.local_collision_map.occupied_xyz.front() - 1.0F) < 1e-6F,
          "collision projection must not retain the DDS payload loan");

  layer.message.live = false;
  projector.projectLocalCollision(layer.message, 202.05);
  require(!state.local_collision_map.live &&
              state.local_collision_count == 1 &&
              std::abs(state.local_collision_map.receive_stamp_s - 202.0) < 1e-9,
          "duplicate identity must revoke liveness without refreshing geometry age");

  CollisionFixture stale_identity(22.1, 6, 8, true);
  projector.projectLocalCollision(stale_identity.message, 202.1);
  require(state.local_collision_count == 1 &&
              state.local_collision_rejected == 1,
          "older collision identity must not replace the active snapshot");
  require(state.frames.last_error == "local_collision_identity_stale",
          "collision identity rejection must remain explicit");

  lingtu_dds_Bool clear{};
  clear.data = true;
  projector.clearPlannerInputs(clear, ClearSource::Map);
  require(state.local_collision_map.occupied_xyz.empty() &&
              state.last_local_collision_receive_s == 0.0,
          "planner clear must discard collision payload and receiver freshness");

  projector.projectLocalCollision(layer.message, 202.2);
  require(state.local_collision_count == 1 &&
              state.local_collision_rejected == 2,
          "a queued pre-clear collision layer must not revive");
  require(state.frames.last_error == "local_collision_before_clear",
          "pre-clear collision rejection must remain explicit");

  CollisionFixture fresh(22.2, 8, 10, true);
  projector.projectLocalCollision(fresh.message, 202.3);
  require(state.local_collision_count == 2 &&
              state.local_collision_map.complete,
          "a newer complete collision layer must re-establish planner input");
}

void testObstacleMaterializationKeepsMeasuredBudgetSeparateFromPrediction() {
  EndpointState state;
  InputGate input_gate;
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  MotionLayer live_obstacles(
      layerConfig(0.10, 10.0, 0.0, 4.0, 0.0, 512, 1, true));
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  const auto free_rays = dynamicFreeRays();
  live_obstacles.updateFromScan(origin, free_rays, 0.1);
  live_obstacles.updateFromScan(origin, free_rays, 0.2);
  live_obstacles.updateFromScan(origin, free_rays, 0.3);
  for (int index = 0; index < 4; ++index) {
    const double stamp_s = 1.0 + 0.1 * static_cast<double>(index);
    live_obstacles.updateFromScan(origin, movingClusterWithNearHazard(1.05f + 0.1f * index),
                                  stamp_s);
    live_obstacles.dynamicClusters(32, stamp_s);
  }
  require(!live_obstacles.dynamicClusters(32, 1.3).empty(),
          "test fixture must establish a confirmed moving cluster");
  require(live_obstacles.snapshot(0, 1.3).size() / 4 > 20,
          "test fixture must exceed the measured-obstacle budget");

  InputConfig config;
  config.max_obstacle_points = 20;
  InputProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   config, InputActions{});
  state.last_cloud_s = 1.3;
  state.obstacle_snapshot_dirty = true;
  TimingDiagnostics timing;
  require(projector.materializeObstacles(timing),
          "dirty obstacle state must materialize a fused planner snapshot");
  require(state.obstacle_xyzh.size() / 4 == config.max_obstacle_points,
          "prediction must not consume any measured-obstacle budget");
  require(!state.predicted_obstacle_xyzh.empty() &&
              state.predicted_obstacle_xyzh.size() / 4 <= kMaxDynamicPredictionPoints,
          "future occupancy must remain a separate bounded input");
  require(containsNearXy(state.obstacle_xyzh, 0.25f, -0.55f, 0.12f),
          "bounded measured snapshot must retain the nearest current hazard");
  require(containsNearXy(state.predicted_obstacle_xyzh, 1.60f, 0.60f, 0.20f),
          "future dynamic occupancy must be present without replacing a measurement");
}

void testObstacleMaterializationUsesFullCurrentBudgetWhenNothingIsPredicted() {
  EndpointState state;
  InputGate input_gate;
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  MotionLayer live_obstacles(
      layerConfig(0.10, 10.0, 0.0, 4.0, 0.0, 100, 1, false));
  std::vector<float> current;
  for (int index = 0; index < 30; ++index) {
    current.insert(current.end(), {0.25f + 0.10f * static_cast<float>(index), 0.0f, 0.05f, 0.4f});
  }
  live_obstacles.update(current, 1.0);

  InputConfig config;
  config.max_obstacle_points = 20;
  InputProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   config, InputActions{});
  state.last_cloud_s = 1.0;
  state.obstacle_snapshot_dirty = true;
  TimingDiagnostics timing;
  require(projector.materializeObstacles(timing),
          "static current obstacle state must materialize");
  require(state.obstacle_xyzh.size() / 4 == config.max_obstacle_points,
          "unused prediction reserve must return to the current obstacle budget");
  require(state.predicted_obstacle_xyzh.empty(),
          "static current obstacles must not create a future-occupancy input");
  require(containsNearXy(state.obstacle_xyzh, 0.25f, 0.0f, 0.06f),
          "full current budget must retain the nearest measured hazard");
}

void testPlanViewUsesTickTime() {
  const std::vector<float> measured{
      0.0f, 0.0f, 0.0f, 0.4f, 1.0f, 0.0f, 0.0f, 0.4f, 2.0f, 0.0f, 0.0f, 0.4f,
  };
  const std::vector<float> predicted{
      10.0f, 0.0f, 0.0f, 0.4f, 11.0f, 0.0f, 0.0f, 0.4f,
  };
  const std::vector<float> empty;
  std::vector<float> merged;
  constexpr std::size_t point_budget = 4;

  lingtu::nav::endpoint::TraversabilityGrid traversability;
  traversability.values = {0.0f};
  traversability.rows = 1;
  traversability.cols = 1;
  traversability.resolution = 0.2;
  traversability.generation = 7;
  double traversability_received_s = 9.8;
  double terrain_received_s = 0.0;
  double terrain_ext_received_s = 0.0;
  lingtu::nav::endpoint::PlanConfig config{
      true, true, 0.5, 0.5, point_budget,
  };
  lingtu::nav::endpoint::PlanData data{
      traversability, traversability_received_s, measured, predicted, empty,
      terrain_received_s, empty, terrain_ext_received_s, merged,
  };
  TimingDiagnostics timing;
  const auto view = lingtu::nav::endpoint::makePlanView(config, data, 10.0, timing);

  require(view.obstacles == &merged && merged.size() / 4 == point_budget,
          "registered scan and prediction must share one bounded planner budget");
  require(std::equal(measured.begin(), measured.end(), merged.begin()),
          "appending prediction must not replace or reorder any measured planner point");
  require(containsNearXy(merged, 10.0f, 0.0f, 0.01f),
          "remaining planner budget must admit the first predicted obstacle");
  require(!containsNearXy(merged, 11.0f, 0.0f, 0.01f),
          "planner input must not exceed its total point budget");
  require(view.traversability_fresh && view.traversability.generation == 7,
          "planner freshness must use the caller's tick timestamp");

  traversability_received_s = 9.0;
  const auto stale = lingtu::nav::endpoint::makePlanView(config, data, 10.0, timing);
  require(!stale.traversability_fresh && stale.traversability.values == nullptr,
          "stale traversability must not enter the planner view");

  const auto collision_view =
      lingtu::nav::endpoint::makePlanView(config, data, 10.0, timing, true);
  require(collision_view.obstacles == &merged && merged.empty(),
          "authoritative collision input must bypass unused legacy obstacle fusion");
}

void testLocalTraversabilityProjectionKeepsOdomSeparateFromMap() {
  EndpointState state;
  InputGate input_gate;
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  MotionLayer live_obstacles;
  InputProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   InputConfig{}, InputActions{});

  TraversabilityFixture local_grid(21.0, "odom");
  projector.projectLocalTraversability(local_grid.message, 201.0);
  require(state.local_traversability_count == 1,
          "accepted odom grid must increment the local-grid counter");
  require(state.local_traversability_generation == 1,
          "accepted odom grid must advance its own generation");
  require(state.local_traversability_grid.generation == 1,
          "local grid must retain its own projector generation");
  require(std::abs(state.last_local_traversability_receive_s - 201.0) < 1e-9,
          "local-grid freshness must use explicit steady receipt time");
  require(state.traversability_count == 0 && state.traversability_grid.values.empty(),
          "local odom grid must never populate the legacy map grid");
  local_grid.data[0] = 99;
  require(state.local_traversability_grid.values.front() == 1.0f,
          "local-grid projection must not retain DDS payload");

  TraversabilityFixture wrong_local_frame(21.1, "map");
  projector.projectLocalTraversability(wrong_local_frame.message, 201.1);
  require(state.local_traversability_count == 1,
          "map-frame grid must not be accepted on the local odom channel");
  require(state.local_traversability_rejected == 1,
          "wrong local-grid frame must increment only the local rejection counter");
  require(state.frames.last_error == "local_grid_frame_unsupported",
          "local-grid frame rejection must remain explicit");

  lingtu_dds_Bool clear{};
  clear.data = true;
  projector.clearPlannerInputs(clear, ClearSource::Map);
  require(state.local_traversability_grid.values.empty(),
          "planner-input clear must discard stale local grid payload");
  require(std::abs(state.last_local_traversability_s - 21.0) < 1e-9 &&
              state.last_local_traversability_receive_s == 0.0,
          "planner-input clear must retain a source watermark but discard local-grid freshness");

  projector.projectLocalTraversability(local_grid.message, 201.2);
  require(state.local_traversability_count == 1,
          "a pre-clear local grid must not revive after planner-input clear");
  require(state.local_traversability_rejected == 2,
          "pre-clear local grid must increment the local rejection counter");
  require(state.frames.last_error == "local_traversability_before_clear",
          "pre-clear local grid rejection must remain explicit");

  TraversabilityFixture fresh_local_grid(21.2, "odom");
  projector.projectLocalTraversability(fresh_local_grid.message, 201.3);
  require(state.local_traversability_count == 2 &&
              state.local_traversability_generation == 2,
          "a newer odom grid must re-establish the local risk view after clear");
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
  MotionLayer live_obstacles;
  InputProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                                   InputConfig{}, InputActions{});

  lingtu_dds_TransformStamped transform{};
  auto initial_tf = tfMessage(transform, 10.0, 0.0);
  auto initial_odom = odometryMessage(10.0, "odom", 0.0);
  projector.projectTf(initial_tf, 100.0);
  projector.projectOdometry(initial_odom, 100.0);
  require(projector.evaluateGate(100.05, steadyTime(100.05)).ready,
          "initial complete generation set must open a one-frame gate");

  auto jumped_tf = tfMessage(transform, 11.0, 0.6);
  projector.projectTf(jumped_tf, 101.0);
  require(!state.odom_body && state.last_odom_s == 0.0,
          "epoch reset must immediately remove old odometry");

  auto recovered_odom = odometryMessage(11.0, "odom", 0.0);
  projector.projectOdometry(recovered_odom, 101.05);
  auto gate_state = projector.evaluateGate(101.1, steadyTime(101.1));
  require(gate_state.ready,
          "triggering TF generation must count toward recovery because baseline precedes it");
}

void testSensorBatchOwnsDdsSamplesAndAppliesInFrameOrder() {
  EndpointState state;
  InputGate input_gate;
  TransformBuffer pose_buffer;
  TransformBuffer map_odom_buffer;
  MotionLayer live_obstacles(
      layerConfig(0.10, 1.0, 0.0, 4.0, 0.0, 100, 1, false));
  CoreInputProjector projector(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                               InputConfig{}, InputActions{});

  auto odometry = odometryMessage(42.0, "map", 1.0);
  PointCloudFixture cloud(42.0, "map", 2.0f, 0.0f, 0.2f);
  lingtu::nav::endpoint::SensorBatch batch;
  batch.receive_steady_s = 100.0;
  batch.receive_wall_s = 1000.0;
  batch.odometry.push_back(lingtu::nav::endpoint::copyOdometrySample(odometry));
  batch.obstacles = lingtu::nav::endpoint::copyPointCloudSample(cloud.message, false);

  odometry.pose.pose.position.x = 99.0;
  cloud.setPoint(99.0f, 0.0f, 0.2f);
  TimingDiagnostics timing;
  projector.apply(std::move(batch), timing);
  require(state.map_body && std::abs(state.map_body->position.x - 1.0) < 1e-9,
          "sensor batch must own odometry before the DDS loan is returned");
  require(projector.materializeObstacles(timing),
          "sensor batch must apply an accepted cloud after its pose");
  require(!state.obstacle_xyzh.empty() &&
              std::abs(state.obstacle_xyzh.front() - 2.0f) < 0.11f,
          "sensor batch must own point data before the DDS loan is returned");
}

}  // namespace

int main() {
  try {
  testTfProjectionPreservesCountersGenerationsAndReceiveClock();
  testEpochResetClearsInputsBeforeSynchronousEffectsAndAcceptsTriggeringTf();
  testOdometryAcceptsMapFrameAndRequiresTimeAlignedTfForOdomFrame();
  testOdometryRejectsDuplicateSourceStampWithoutPoisoningFiniteState();
  testOdometryRejectsNonCanonicalChildFrames();
  testTfRejectsReverseMapOdomPayload();
  testDriverProjectionCopiesLoansUsesSteadyFreshnessAndPreservesBlockerPrecedence();
  testCloudTerrainAndSnapshotProjectionOwnDdsDataAndExactClocks();
  testCloudClockRebaseClearsOldMotionLayerAndDerivedSnapshots();
  testObstacleMaterializationKeepsMeasuredBudgetSeparateFromPrediction();
  testObstacleMaterializationUsesFullCurrentBudgetWhenNothingIsPredicted();
  testPlanViewUsesTickTime();
  testTraversabilityAndLocalizationProjectionOwnPayloadsAndAdvanceExactGenerations();
  testLocalCollisionProjectionOwnsPayloadAndRejectsPreClearReplay();
  testLocalTraversabilityProjectionKeepsOdomSeparateFromMap();
  testPlannerClearingUsesDistinctSynchronousReasonsWithoutResettingCloudEpoch();
  testEpochRecoveryBaselinesBeforeTriggeringTfGeneration();
  testSensorBatchOwnsDdsSamplesAndAppliesInFrameOrder();
  std::cout << "test_input_projector passed\n";
  return 0;
  } catch (const std::exception &error) {
    std::cerr << "test_input_projector failed: " << error.what() << '\n';
    return 1;
  }
}
