#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#include <process.h>
#else
#include <unistd.h>
#endif

#include "dds.hpp"
#include "dds/dds.h"
#include "messages.h"
#include "message/cpp/qos.hpp"
#include "message/cpp/topics.hpp"

namespace {

using lingtu::maps::CloudLayout;
using lingtu::maps::OwnedPointCloud;
using lingtu::maps::mapd::ActivationOperation;
using lingtu::maps::mapd::ActivationResult;
using lingtu::maps::mapd::Dds;
using lingtu::maps::mapd::DdsLimits;
using lingtu::maps::mapd::DdsOutputState;
using lingtu::maps::mapd::EvaluateReadiness;
using lingtu::maps::mapd::PublicationCursor;
using lingtu::maps::mapd::PublicationProgress;
using lingtu::maps::mapd::Snapshot;
using lingtu::maps::mapd::State;

int TestDomain(int offset) {
#if defined(_WIN32)
  const auto process_id = static_cast<unsigned int>(_getpid());
#else
  const auto process_id = static_cast<unsigned int>(getpid());
#endif
  constexpr unsigned int kFirstDomain = 20U;
  constexpr unsigned int kDomainSpan = 180U;
  return static_cast<int>(kFirstDomain +
                          ((process_id * 37U + static_cast<unsigned int>(offset)) % kDomainSpan));
}

dds_entity_t Checked(dds_return_t value) {
  assert(value >= 0);
  return static_cast<dds_entity_t>(value);
}

void FillHeader(lingtu_dds_Header &header, const char *frame) {
  header.stamp.sec = 100;
  header.stamp.nanosec = 25U;
  header.frame_id = const_cast<char *>(frame);
}

dds_entity_t CreateWriter(dds_entity_t participant, dds_entity_t publisher, const char *topic_name,
                          const dds_topic_descriptor_t *descriptor) {
  const auto topic =
      Checked(dds_create_topic(participant, descriptor, topic_name, nullptr, nullptr));
  auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic_name));
  return Checked(dds_create_writer(publisher, topic, qos.get(), nullptr));
}

dds_entity_t CreateReader(dds_entity_t participant, dds_entity_t subscriber, const char *topic_name,
                          const dds_topic_descriptor_t *descriptor) {
  const auto topic =
      Checked(dds_create_topic(participant, descriptor, topic_name, nullptr, nullptr));
  auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic_name));
  return Checked(dds_create_reader(subscriber, topic, qos.get(), nullptr));
}

struct ObservationMessage {
  lingtu_dds_MapObservation message{};
  lingtu_dds_PointField fields[4]{};
  std::vector<std::uint8_t> data;

  ObservationMessage() {
    const float points[6] = {1.0F, 2.0F, 0.5F, 3.0F, 4.0F, 0.75F};
    data.resize(sizeof(points));
    std::memcpy(data.data(), points, sizeof(points));
    FillHeader(message.header, "map");
    message.observation_sequence = 9U;
    message.reset_epoch = 4U;
    message.sensor_frame = const_cast<char *>("lidar");
    message.map_sensor.translation.x = 10.0;
    message.map_sensor.translation.y = 20.0;
    message.map_sensor.rotation.w = 1.0;
    message.sensor_origin.x = 10.0;
    message.sensor_origin.y = 20.0;
    message.pose_confidence = 0.9F;
    message.localization_quality = 0.8F;
    message.pose_state = const_cast<char *>("tracking");
    message.pose_reason = const_cast<char *>("");
    FillHeader(message.scan.header, "lidar");
    message.scan.height = 1U;
    message.scan.width = 2U;
    message.scan.point_step = 12U;
    message.scan.row_step = 24U;
    message.scan.is_bigendian = false;
    message.scan.is_dense = true;
    const char *names[3] = {"x", "y", "z"};
    for (std::uint32_t index = 0U; index < 3U; ++index) {
      fields[index].name = const_cast<char *>(names[index]);
      fields[index].offset = index * 4U;
      fields[index].datatype = 7U;
      fields[index].count = 1U;
    }
    message.scan.fields._maximum = 3U;
    message.scan.fields._length = 3U;
    message.scan.fields._buffer = fields;
    message.scan.fields._release = false;
    message.scan.data._maximum = static_cast<std::uint32_t>(data.size());
    message.scan.data._length = static_cast<std::uint32_t>(data.size());
    message.scan.data._buffer = data.data();
    message.scan.data._release = false;
  }
};

template <typename Mutator>
void AssertObservationRejected(int domain_id, DdsLimits limits, Mutator mutate,
                               const char *expected_error) {
  Dds mapd(domain_id, limits);
  const auto participant = Checked(dds_create_participant(domain_id, nullptr, nullptr));
  const auto publisher = Checked(dds_create_publisher(participant, nullptr, nullptr));
  const auto writer =
      CreateWriter(participant, publisher, lingtu::message::kSlamMapObservation.dds_topic.data(),
                   &lingtu_dds_MapObservation_desc);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  ObservationMessage observation;
  mutate(observation);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (mapd.GetInputState().rejected_samples == 0U &&
         std::chrono::steady_clock::now() < deadline) {
    assert(dds_write(writer, &observation.message) >= 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    assert(!mapd.TakeLatestObservation().has_value());
  }
  const auto state = mapd.GetInputState();
  assert(state.rejected_samples > 0U);
  assert(state.decoded_samples == 0U);
  assert(state.last_error.find(expected_error) != std::string::npos);
  dds_delete(participant);
}

OwnedPointCloud OnePointCloud() {
  OwnedPointCloud cloud;
  cloud.frame_id = "map";
  cloud.stamp_ns = 100000000025LL;
  cloud.layout = CloudLayout::kXyzF32Interleaved;
  cloud.point_count = 1U;
  cloud.interleaved = {1.0F, 2.0F, 0.5F};
  return cloud;
}

template <typename Message>
bool TakeOne(dds_entity_t reader, const dds_topic_descriptor_t *descriptor, Message *output) {
  void *sample = dds_alloc(sizeof(Message));
  std::memset(sample, 0, sizeof(Message));
  dds_sample_info_t info{};
  void *samples[1] = {sample};
  const dds_return_t count = dds_take(reader, samples, &info, 1U, 1U);
  bool received = false;
  if (count == 1 && info.valid_data) {
    *output = *static_cast<Message *>(sample);
    dds_free(sample);
    received = true;
  }
  if (!received) {
    dds_sample_free(sample, descriptor, DDS_FREE_ALL);
  }
  return received;
}

void FreeTaken(void *sample, const dds_topic_descriptor_t *descriptor) {
  dds_sample_free(sample, descriptor, DDS_FREE_CONTENTS);
}

void TestMapdDdsRoundTrip() {
  const int kDomainId = TestDomain(0);
  Dds mapd(kDomainId, 100U);
  const auto participant = Checked(dds_create_participant(kDomainId, nullptr, nullptr));
  const auto publisher = Checked(dds_create_publisher(participant, nullptr, nullptr));
  const auto subscriber = Checked(dds_create_subscriber(participant, nullptr, nullptr));
  const auto observation_writer =
      CreateWriter(participant, publisher, lingtu::message::kSlamMapObservation.dds_topic.data(),
                   &lingtu_dds_MapObservation_desc);
  const auto state_reader =
      CreateReader(participant, subscriber, lingtu::message::kMapsState.dds_topic.data(),
                   &lingtu_dds_MapRuntimeState_desc);
  const auto scene_reader =
      CreateReader(participant, subscriber, lingtu::message::kMapsScene.dds_topic.data(),
                   &lingtu_dds_MapScene_desc);
  const auto collision_reader =
      CreateReader(participant, subscriber, lingtu::message::kMapsLocalCollision.dds_topic.data(),
                   &lingtu_dds_MapCollisionLayer_desc);
  const auto activation_writer =
      CreateWriter(participant, publisher, lingtu::message::kMapsActivationRequest.dds_topic.data(),
                   &lingtu_dds_MapActivationRequest_desc);
  const auto activation_ack_reader =
      CreateReader(participant, subscriber, lingtu::message::kMapsActivationAck.dds_topic.data(),
                   &lingtu_dds_MapActivationAck_desc);
  const auto snapshot_request_reader = CreateReader(
      participant, subscriber, lingtu::message::kSlamMapSnapshotRequest.dds_topic.data(),
      &lingtu_dds_SlamMapSnapshotRequest_desc);
  const auto snapshot_ack_writer =
      CreateWriter(participant, publisher, lingtu::message::kSlamMapSnapshotAck.dds_topic.data(),
                   &lingtu_dds_SlamMapSnapshotAck_desc);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  lingtu_dds_MapArtifactIdentity target_artifact{};
  target_artifact.type = const_cast<char *>("pointcloud");
  target_artifact.uri = const_cast<char *>("/maps/office/map.pcd");
  lingtu_dds_MapActivationRequest activation_message{};
  activation_message.request_id = const_cast<char *>("activation-roundtrip");
  activation_message.operation = lingtu_dds_MAP_ACTIVATION_STAGE;
  activation_message.target.present = true;
  activation_message.target.map_id = const_cast<char *>("office");
  activation_message.target.content_epoch = 7;
  activation_message.target.frame_id = const_cast<char *>("map");
  activation_message.target.artifacts._maximum = 1U;
  activation_message.target.artifacts._length = 1U;
  activation_message.target.artifacts._buffer = &target_artifact;
  activation_message.target.artifacts._release = false;
  activation_message.previous.present = false;
  activation_message.previous.map_id = const_cast<char *>("");
  activation_message.previous.content_epoch = 0;
  activation_message.previous.frame_id = const_cast<char *>("");
  activation_message.caller = const_cast<char *>("mapd-dds-test");
  activation_message.reason = const_cast<char *>("roundtrip");
  std::vector<lingtu::maps::mapd::ActivationRequest> activation_requests;
  const auto activation_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (activation_requests.empty() && std::chrono::steady_clock::now() < activation_deadline) {
    assert(dds_write(activation_writer, &activation_message) >= 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    activation_requests = mapd.TakeActivationRequests();
  }
  assert(activation_requests.size() == 1U);
  assert(activation_requests.front().operation == ActivationOperation::kStage);
  assert(activation_requests.front().target.map_id == "office");
  assert(activation_requests.front().target.content_epoch == 7);
  assert(activation_requests.front().target.artifacts.size() == 1U);
  assert(!activation_requests.front().previous.present);

  ActivationResult activation_result;
  activation_result.request_id = "activation-roundtrip";
  activation_result.operation = ActivationOperation::kStage;
  activation_result.accepted = true;
  activation_result.message = "staged";
  activation_result.changed = true;
  activation_result.target = activation_requests.front().target;
  activation_result.previous = activation_requests.front().previous;
  activation_result.active = activation_requests.front().target;
  activation_result.producer_boot_id = mapd.ProducerBootId();
  lingtu_dds_MapActivationAck activation_ack{};
  bool received_activation_ack = false;
  const auto ack_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!received_activation_ack && std::chrono::steady_clock::now() < ack_deadline) {
    assert(mapd.PublishActivationAck(activation_result));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    received_activation_ack =
        TakeOne(activation_ack_reader, &lingtu_dds_MapActivationAck_desc, &activation_ack);
  }
  assert(received_activation_ack);
  assert(activation_ack.operation == lingtu_dds_MAP_ACTIVATION_STAGE);
  assert(activation_ack.accepted);
  assert(activation_ack.changed);
  assert(std::string(activation_ack.message) == "staged");
  assert(std::string(activation_ack.active.map_id) == "office");
  assert(activation_ack.active.artifacts._length == 1U);
  FreeTaken(&activation_ack, &lingtu_dds_MapActivationAck_desc);

  lingtu::maps::mapd::SlamSnapshotRequest snapshot_request;
  snapshot_request.request_id = "snapshot-roundtrip";
  snapshot_request.map_id = "office";
  snapshot_request.product_session_id = "product-session-1";
  snapshot_request.output_path = "/tmp/snapshot-roundtrip/map.pcd";
  snapshot_request.save_patches = true;
  lingtu_dds_SlamMapSnapshotRequest snapshot_request_message{};
  bool received_snapshot_request = false;
  const auto snapshot_request_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!received_snapshot_request &&
         std::chrono::steady_clock::now() < snapshot_request_deadline) {
    assert(mapd.Publish(snapshot_request));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    received_snapshot_request =
        TakeOne(snapshot_request_reader, &lingtu_dds_SlamMapSnapshotRequest_desc,
                &snapshot_request_message);
  }
  assert(received_snapshot_request);
  assert(std::string(snapshot_request_message.request_id) == "snapshot-roundtrip");
  assert(std::string(snapshot_request_message.map_id) == "office");
  assert(std::string(snapshot_request_message.product_session_id) == "product-session-1");
  assert(std::string(snapshot_request_message.output_path) == "/tmp/snapshot-roundtrip/map.pcd");
  assert(snapshot_request_message.save_patches);
  FreeTaken(&snapshot_request_message, &lingtu_dds_SlamMapSnapshotRequest_desc);

  lingtu_dds_SlamMapSnapshotAck snapshot_ack_message{};
  snapshot_ack_message.request_id = const_cast<char *>("snapshot-roundtrip");
  snapshot_ack_message.map_id = const_cast<char *>("office");
  snapshot_ack_message.success = true;
  snapshot_ack_message.message = const_cast<char *>("captured");
  snapshot_ack_message.output_path = const_cast<char *>("/tmp/snapshot-roundtrip/map.pcd");
  snapshot_ack_message.runtime_instance_id = const_cast<char *>("fastlio2:boot:1");
  snapshot_ack_message.product_session_id = const_cast<char *>("product-session-1");
  snapshot_ack_message.reset_epoch = 4U;
  snapshot_ack_message.observation_sequence = 9U;
  snapshot_ack_message.captured_at_ns = 100000000025ULL;
  snapshot_ack_message.frame_id = const_cast<char *>("map");
  snapshot_ack_message.point_count = 42U;
  snapshot_ack_message.state = const_cast<char *>("tracking");
  snapshot_ack_message.healthy = true;
  snapshot_ack_message.health_message = const_cast<char *>("");
  std::vector<lingtu::maps::mapd::SlamSnapshotAck> snapshot_acks;
  const auto snapshot_ack_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (snapshot_acks.empty() && std::chrono::steady_clock::now() < snapshot_ack_deadline) {
    assert(dds_write(snapshot_ack_writer, &snapshot_ack_message) >= 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    snapshot_acks = mapd.TakeAcks();
  }
  assert(snapshot_acks.size() == 1U);
  assert(snapshot_acks.front().request_id == "snapshot-roundtrip");
  assert(snapshot_acks.front().map_id == "office");
  assert(snapshot_acks.front().runtime_instance_id == "fastlio2:boot:1");
  assert(snapshot_acks.front().point_count == 42U);

  ObservationMessage observation;
  std::optional<lingtu::maps::mapd::Observation> decoded;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!decoded.has_value() && std::chrono::steady_clock::now() < deadline) {
    assert(dds_write(observation_writer, &observation.message) >= 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    decoded = mapd.TakeLatestObservation();
  }
  assert(decoded.has_value());
  assert(decoded->reset_epoch == 4U);
  assert(decoded->sequence == 9U);
  assert(decoded->scan.point_count == 2U);
  assert(decoded->scan.x[1] == 3.0F);
  assert(decoded->map_sensor.x == 10.0);
  assert(decoded->pose_state == "tracking");

  State state;
  state.running = true;
  state.live = true;
  state.reset_epoch = 4U;
  state.sequence = 9U;
  state.generation = 12U;
  state.processed_observations = 1U;
  state.live_points = 1U;
  state.voxel_points = 1U;
  state.accumulated_cells = 3U;
  state.accumulated_snapshot_cells = 2U;
  state.pose_quality = 0.8F;
  state.pose_state = "TRACKING";

  Snapshot snapshot;
  snapshot.frame_id = "map";
  snapshot.stamp_ns = 100000000025LL;
  snapshot.reset_epoch = 4U;
  snapshot.sequence = 9U;
  snapshot.generation = 12U;
  snapshot.map_sensor.x = 10.0;
  snapshot.live_cloud = OnePointCloud();
  snapshot.voxel_cloud = OnePointCloud();
  snapshot.collision.generation = 7U;
  snapshot.collision.resolution_m = 0.2F;
  snapshot.collision.min_x_m = -1.0F;
  snapshot.collision.min_y_m = -2.0F;
  snapshot.collision.min_z_m = -0.5F;
  snapshot.collision.max_x_m = 3.0F;
  snapshot.collision.max_y_m = 2.0F;
  snapshot.collision.max_z_m = 1.5F;
  snapshot.collision.size_x = 20;
  snapshot.collision.size_y = 20;
  snapshot.collision.size_z = 10;
  snapshot.collision.occupied_cells = 1U;
  snapshot.collision.complete = true;
  snapshot.collision.occupied_bits.assign(500U, 0U);
  snapshot.collision.occupied_bits.front() = 1U;
  snapshot.occupancy = lingtu::maps::layers::makeGrid2D(2, 2, 0.2, 0.0, 0.0, 0.0F);
  snapshot.elevation.minZ = lingtu::maps::layers::makeGrid2D(2, 2, 0.2, 0.0, 0.0, 0.1F);
  snapshot.esdf.distance = lingtu::maps::layers::makeGrid2D(2, 2, 0.2, 0.0, 0.0, 1.0F);

  lingtu_dds_MapRuntimeState state_message{};
  lingtu_dds_MapScene scene_message{};
  lingtu_dds_MapCollisionLayer collision_message{};
  bool received_state = false;
  bool received_scene = false;
  bool received_collision = false;
  const auto state_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while ((!received_state || !received_scene || !received_collision) &&
         std::chrono::steady_clock::now() < state_deadline) {
    PublicationProgress publications;
    publications.realtime_clouds.Complete(true, state.generation, state.live);
    publications.map_layers.Complete(true, state.generation, state.live);
    publications.scene.Complete(true, state.generation, state.live);
    assert(mapd.PublishState(state, publications, activation_requests.front().target));
    assert(mapd.PublishRealtimeClouds(state, snapshot));
    assert(mapd.PublishMapLayers(state, snapshot));
    assert(mapd.PublishScene(state, snapshot));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (!received_state) {
      received_state = TakeOne(state_reader, &lingtu_dds_MapRuntimeState_desc, &state_message);
    }
    if (!received_scene) {
      received_scene = TakeOne(scene_reader, &lingtu_dds_MapScene_desc, &scene_message);
    }
    if (!received_collision) {
      received_collision =
          TakeOne(collision_reader, &lingtu_dds_MapCollisionLayer_desc, &collision_message);
    }
  }
  assert(received_state);
  assert(state_message.reset_epoch == 4U);
  assert(state_message.observation_sequence == 9U);
  assert(state_message.accumulated_snapshot_cells == 2U);
  assert(state_message.pose_state != nullptr);
  assert(std::string(state_message.pose_state) == "TRACKING");
  assert(state_message.dds_received_samples >= 1U);
  assert(state_message.dds_decoded_samples >= 1U);
  assert(state_message.dds_rejected_samples == 0U);
  assert(state_message.dds_write_failures == 0U);
  assert(state_message.dds_unhealthy_writers == 0U);
  assert(state_message.voxel_capacity_rejections == 0U);
  assert(state_message.accumulated_capacity_rejections == 0U);
  assert(!state_message.capacity_limited);
  assert(state_message.required_publications_ready);
  assert(state_message.current_generation_published);
  assert(state_message.state_published_generation == 12U);
  assert(state_message.realtime_clouds_published_generation == 12U);
  assert(state_message.map_layers_published_generation == 12U);
  assert(state_message.scene_published_generation == 12U);
  assert(state_message.active_map.present);
  assert(state_message.active_map.map_id != nullptr);
  assert(std::string(state_message.active_map.map_id) == "office");
  assert(state_message.active_map.content_epoch == 7);
  assert(received_scene);
  assert(scene_message.generation == 12U);
  assert(scene_message.live_cloud.cloud.width == 1U);
  assert(scene_message.occupancy.data._length == 4U);
  assert(received_collision);
  assert(collision_message.reset_epoch == 4U);
  assert(collision_message.observation_sequence == 9U);
  assert(collision_message.generation == 7U);
  assert(collision_message.live);
  assert(collision_message.complete);
  assert(collision_message.resolution == 0.2F);
  assert(collision_message.aabb_min.x == -1.0);
  assert(collision_message.aabb_max.z == 1.5);
  assert(collision_message.size_x == 20U);
  assert(collision_message.size_y == 20U);
  assert(collision_message.size_z == 10U);
  assert(collision_message.inflated_occupied_bits._length == 500U);
  assert(collision_message.inflated_occupied_bits._buffer[0] == 1U);
  FreeTaken(&state_message, &lingtu_dds_MapRuntimeState_desc);
  FreeTaken(&scene_message, &lingtu_dds_MapScene_desc);
  FreeTaken(&collision_message, &lingtu_dds_MapCollisionLayer_desc);
  dds_delete(participant);
}

void TestPublicationCursorRetriesFailedGeneration() {
  State state;
  state.running = true;
  state.generation = 7U;
  state.live = true;
  PublicationCursor cursor;

  assert(cursor.Pending(state));
  assert(!cursor.published_once);
  cursor.Complete(false, state.generation, state.live);
  assert(cursor.generation == 0U);
  assert(!cursor.published_once);
  assert(cursor.Pending(state));

  cursor.Complete(true, state.generation, state.live);
  assert(cursor.generation == 7U);
  assert(cursor.live);
  assert(cursor.published_once);
  assert(cursor.PublishedFor(state));
  assert(!cursor.Pending(state));

  state.live = false;
  assert(cursor.Pending(state));
  assert(!cursor.PublishedFor(state));

  PublicationProgress progress;
  assert(!progress.BootComplete());
  progress.state.Complete(true, 7U, true);
  progress.realtime_clouds.Complete(true, 7U, true);
  progress.map_layers.Complete(true, 7U, true);
  assert(!progress.BootComplete());
  progress.scene.Complete(true, 7U, true);
  state.live = true;
  assert(progress.BootComplete());
  assert(progress.CurrentGenerationPublished(state));

  DdsOutputState output;
  const auto ready = EvaluateReadiness(state, output, progress);
  assert(ready.ready);
  assert(std::string(ready.status) == "ready");
  progress.scene = {};
  const auto incomplete = EvaluateReadiness(state, output, progress);
  assert(!incomplete.ready);
  assert(std::string(incomplete.status) == "publishing_required_channels");
  progress.scene.Complete(true, 7U, true);
  state.generation = 8U;
  const auto lagging = EvaluateReadiness(state, output, progress);
  assert(!lagging.ready);
  assert(std::string(lagging.status) == "publishing_current_generation");
  progress.state.Complete(true, 8U, true);
  progress.realtime_clouds.Complete(true, 8U, true);
  progress.map_layers.Complete(true, 8U, true);
  progress.scene.Complete(true, 8U, true);
  assert(EvaluateReadiness(state, output, progress).ready);
  output.unhealthy_writers = 1U;
  assert(std::string(EvaluateReadiness(state, output, progress).status) == "dds_output_degraded");
  output.unhealthy_writers = 0U;
  state.capacity_limited = true;
  assert(std::string(EvaluateReadiness(state, output, progress).status) == "map_capacity_limited");
}

void TestObservationResourceLimitsFailClosed() {
  DdsLimits bytes_limit;
  bytes_limit.max_cloud_bytes = 16U;
  AssertObservationRejected(
      TestDomain(1), bytes_limit, [](ObservationMessage &) {}, "byte sequence exceeds");

  DdsLimits point_step_limit;
  point_step_limit.max_point_step = 12U;
  AssertObservationRejected(
      TestDomain(2), point_step_limit,
      [](ObservationMessage &observation) {
        observation.data.resize(32U, 0U);
        observation.message.scan.point_step = 16U;
        observation.message.scan.row_step = 32U;
        observation.message.scan.data._maximum =
            static_cast<std::uint32_t>(observation.data.size());
        observation.message.scan.data._length = static_cast<std::uint32_t>(observation.data.size());
        observation.message.scan.data._buffer = observation.data.data();
      },
      "point_step exceeds");

  DdsLimits field_limit;
  field_limit.max_point_fields = 3U;
  AssertObservationRejected(
      TestDomain(3), field_limit,
      [](ObservationMessage &observation) {
        observation.fields[3].name = const_cast<char *>("intensity");
        observation.fields[3].offset = 0U;
        observation.fields[3].datatype = 7U;
        observation.fields[3].count = 1U;
        observation.message.scan.fields._maximum = 4U;
        observation.message.scan.fields._length = 4U;
      },
      "field count exceeds");

  DdsLimits string_limit;
  string_limit.max_string_bytes = 8U;
  AssertObservationRejected(
      TestDomain(4), string_limit,
      [](ObservationMessage &observation) {
        observation.message.sensor_frame = const_cast<char *>("lidar_frame_name_is_too_long");
      },
      "sensor_frame exceeds");
}

void TestScenePayloadLimitFailsClosedAndRecovers() {
  DdsLimits limits;
  limits.max_scene_bytes = 1U;
  Dds mapd(TestDomain(5), limits);
  State state;
  state.running = true;
  state.live = true;
  state.generation = 1U;
  Snapshot snapshot;
  snapshot.generation = 1U;
  snapshot.live_cloud = OnePointCloud();
  assert(!mapd.PublishScene(state, snapshot));
  auto output = mapd.GetOutputState();
  assert(output.serialization_rejections == 1U);
  assert(output.scene_oversize_rejections == 1U);
  assert(output.unhealthy_writers == 1U);

  snapshot.live_cloud = {};
  assert(mapd.PublishScene(state, snapshot));
  output = mapd.GetOutputState();
  assert(output.unhealthy_writers == 0U);
  assert(output.scene_oversize_rejections == 1U);
}

}  // namespace

int main() {
  TestPublicationCursorRetriesFailedGeneration();
  TestObservationResourceLimitsFailClosed();
  TestScenePayloadLimitFailsClosedAndRecovers();
  TestMapdDdsRoundTrip();
  return 0;
}
