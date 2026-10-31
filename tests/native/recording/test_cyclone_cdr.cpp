#include <chrono>
#include <filesystem>
#include "mcap/reader.hpp"
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "dds/dds.h"
#include "lingtu/recording/cyclone_cdr.hpp"
#include "lingtu/recording/recording_core.hpp"
#include "lingtu/recording/topic_catalog.hpp"
#include "messages.h"

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void test_joint_telemetry_cdr_round_trip(dds_entity_t participant) {
  const auto *binding = lingtu::recording::find_sensor_topic("/robot/joint_states");
  require(binding != nullptr, "motor joint telemetry cannot be recorded/replayed");
  const auto topic = dds_create_topic(participant, binding->descriptor,
                                     "rt/lingtu/test/native_recording/joints", nullptr, nullptr);
  const auto writer = dds_create_writer(participant, topic, nullptr, nullptr);
  const auto reader = dds_create_reader(participant, topic, nullptr, nullptr);
  require(writer > 0 && reader > 0, "could not create joint telemetry endpoints");
  lingtu_dds_JointState input{};
  input.header.frame_id = const_cast<char *>("body");
  input.header.stamp.sec = 15;
  input.robot_model = const_cast<char *>("test-platform");
  char *names[]{const_cast<char *>("hip"), const_cast<char *>("knee")};
  double positions[]{0.25, -1.2};
  double velocities[]{1.5, -0.75};
  double torques[]{3.5, -2.25};
  input.names = {2, 2, names, false};
  input.position = {2, 2, positions, false};
  input.velocity = {2, 2, velocities, false};
  input.effort = {2, 2, torques, false};
  require(dds_write_ts(writer, &input, 15000000000LL) == DDS_RETCODE_OK,
          "could not publish joint telemetry");
  std::uint32_t sequence = 0;
  std::vector<lingtu::recording::RecordedMessage> captured;
  for (int attempt = 0; attempt < 100 && captured.empty(); ++attempt) {
    captured = lingtu::recording::take_cdr_messages(reader, binding->contract->dds_topic, sequence);
    if (captured.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  require(captured.size() == 1, "joint telemetry did not reach the CDR recorder");
  require(lingtu::recording::validate_cdr_payload(*binding, captured[0].payload).empty(),
          "recorded joint telemetry is not valid typed CDR");
  const auto file = std::filesystem::temp_directory_path() /
      ("lingtu-joints-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".mcap");
  {
    lingtu::recording::McapSessionWriter storage(
        file, "module fixture {};", {lingtu::recording::channel_definition(*binding)}, 1024);
    storage.write(captured[0]);
    storage.commit();
  }
  mcap::McapReader stored;
  require(stored.open(file.string()).ok(), "could not open recorded motor MCAP");
  std::vector<std::byte> recorded_payload;
  for (const auto &message : stored.readMessages()) {
    require(message.channel->metadata.at("lingtu.idl_type") == "lingtu.dds.JointState",
            "joint MCAP lost typed schema identity");
    recorded_payload.assign(message.message.data, message.message.data + message.message.dataSize);
  }
  stored.close();
  require(recorded_payload == captured[0].payload, "joint bytes changed in MCAP storage");
  std::filesystem::remove(file);
  const auto replay_topic = dds_create_topic(participant, binding->descriptor,
                                            "rt/lingtu/test/native_recording/joints/replay", nullptr, nullptr);
  const auto replay_writer = dds_create_writer(participant, replay_topic, nullptr, nullptr);
  const auto replay_reader = dds_create_reader(participant, replay_topic, nullptr, nullptr);
  require(replay_writer > 0 && replay_reader > 0, "could not create joint replay endpoints");
  require(lingtu::recording::forward_cdr_message(replay_writer, *binding,
                                               recorded_payload, 15000000000LL).empty(),
          "could not replay recorded joint telemetry");
  auto *output = lingtu_dds_JointState__alloc();
  void *samples[]{output};
  dds_sample_info_t info{};
  dds_return_t count = 0;
  for (int attempt = 0; attempt < 100 && count == 0; ++attempt) {
    count = dds_take(replay_reader, samples, &info, 1, 1);
    if (count == 0) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  require(count == 1 && info.valid_data, "replayed joint telemetry did not arrive");
  require(std::strcmp(output->robot_model, "test-platform") == 0 && output->names._length == 2 &&
          std::strcmp(output->names._buffer[1], "knee") == 0,
          "joint robot/name metadata was lost");
  require(output->position._length == 2 && output->position._buffer[1] == -1.2 &&
          output->velocity._length == 2 && output->velocity._buffer[0] == 1.5 &&
          output->effort._length == 2 && output->effort._buffer[1] == -2.25,
          "joint position, velocity or torque changed during replay");
  require(info.source_timestamp == 15000000000LL, "joint acquisition timestamp was changed");
  lingtu_dds_JointState_free(output, static_cast<dds_free_op_t>(DDS_FREE_ALL_BIT));
}

}  // namespace

int main() {
  std::unordered_set<std::string> default_topics;
  for (const auto *default_binding : lingtu::recording::default_recording_topics()) {
    require(default_binding != nullptr, "default recording binding is null");
    require(default_binding->contract != nullptr, "default recording contract is null");
    require(default_binding->descriptor != nullptr, "default DDS descriptor is null");
    require(default_binding->allocate_sample != nullptr, "default allocator is null");
    require(default_binding->free_sample != nullptr, "default deallocator is null");
    const std::string topic(default_binding->contract->topic);
    require(lingtu::recording::is_sensor_replay_topic(topic),
            "default recording topic is outside the sensor allowlist");
    require(default_topics.insert(topic).second, "default recording topic is duplicated");
  }
  require(default_topics.count("/lidar/raw_frame") == 1,
          "default recording topics omitted raw lidar");
  require(default_topics.count("/slam/map_observation") == 0,
          "default recording topics included both raw lidar and MapObservation");

  const auto generic_plan = lingtu::recording::dds_recording_plan("generic-sensors-v1");
  std::unordered_set<std::string> generic_topics;
  for (const auto &topic_name : generic_plan.selected_topics) {
    generic_topics.insert(topic_name);
  }
  require(default_topics == generic_topics,
          "direct recorder defaults disagree with the generic-sensors-v1 preset");

  const auto inspection_plan = lingtu::recording::dds_recording_plan("inspection-evidence-v1");
  for (const auto &topic_name : inspection_plan.selected_topics) {
    const auto *recording_binding = lingtu::recording::find_recording_topic(topic_name);
    require(recording_binding != nullptr,
            "inspection Product selected a topic the DDS recorder cannot capture");
    require(recording_binding->allocate_sample != nullptr,
            "inspection recording topic cannot be decoded for validation");
    require(recording_binding->free_sample != nullptr,
            "inspection recording topic has no validation deallocator");
  }
  require(lingtu::recording::find_sensor_topic("/nav/inspection/task/event") == nullptr,
          "inspection task events became replayable sensor input");
  require(lingtu::recording::find_sensor_topic("/nav/cmd_vel") == nullptr,
          "logical motion evidence became replayable sensor input");

  const auto require_policy = [](const std::string &topic,
                                 lingtu::recording::ReplayPolicy expected) {
    const auto *recording = lingtu::recording::find_recording_topic(topic);
    require(recording != nullptr, "required recording topic binding is missing");
    require(recording->replay_policy == expected, "recording topic has the wrong replay policy");
  };
  std::unordered_set<std::string> catalog_topics;
  for (const auto &catalog_binding : lingtu::recording::recording_topic_catalog()) {
    const std::string topic(catalog_binding.contract->topic);
    require(catalog_topics.insert(topic).second, "recording catalog contains a duplicate topic");
    const bool replayable =
        catalog_binding.replay_policy == lingtu::recording::ReplayPolicy::Replayable;
    require((lingtu::recording::find_sensor_topic(topic) != nullptr) == replayable,
            "sensor catalog disagrees with the binding replay policy");
    require(lingtu::recording::is_sensor_replay_topic(topic) == replayable,
            "explicit replay validation disagrees with the binding replay policy");
  }

  for (const auto *topic_name : {"/tf", "/tf_static", "/slam/map_observation", "/gnss/fix",
                                 "/gnss/odom", "/robot/joint_states", "/camera/depth/image_raw",
                                 "/camera/color/camera_info"}) {
    require_policy(topic_name, lingtu::recording::ReplayPolicy::Replayable);
  }
  for (const auto *topic_name : {
           "/driver/profile/request",
           "/driver/profile/state",
           "/slam/relocalization/request",
           "/slam/relocalization/response",
           "/gnss/status",
           "/slam/localization_health",
           "/slam/localization_quality",
           "/nav/command/request",
           "/nav/command/ack",
           "/nav/operator_motion/control",
           "/nav/operator_motion/sample",
           "/nav/operator_motion/ack",
           "/nav/inspection/task/request",
           "/nav/inspection/task/ack",
           "/nav/inspection/status",
           "/nav/inspection/evidence/request",
       }) {
    require_policy(topic_name, lingtu::recording::ReplayPolicy::RecordOnly);
  }

  const auto *binding = lingtu::recording::find_sensor_topic("/imu/raw");
  require(binding != nullptr, "IMU topic binding is missing");

  const dds_entity_t participant = dds_create_participant(197, nullptr, nullptr);
  require(participant > 0, "failed to create DDS participant");
  const dds_entity_t topic = dds_create_topic(
      participant, binding->descriptor, "rt/lingtu/test/native_recording/imu", nullptr, nullptr);
  require(topic > 0, "failed to create DDS topic");
  const dds_entity_t writer = dds_create_writer(participant, topic, nullptr, nullptr);
  const dds_entity_t reader = dds_create_reader(participant, topic, nullptr, nullptr);
  require(writer > 0, "failed to create DDS writer");
  require(reader > 0, "failed to create DDS reader");

  auto *input = lingtu_dds_Imu__alloc();
  input->header.frame_id = dds_string_dup("imu_link");
  input->header.stamp.sec = 12;
  input->header.stamp.nanosec = 34;
  input->angular_velocity.x = 1.25;
  input->linear_acceleration.z = 9.81;
  constexpr dds_time_t kSourceTimestamp = 123456789;
  require(dds_write_ts(writer, input, kSourceTimestamp) == DDS_RETCODE_OK,
          "failed to publish DDS IMU sample");
  lingtu_dds_Imu_free(input, static_cast<dds_free_op_t>(DDS_FREE_ALL_BIT));

  std::uint32_t sequence = 0;
  std::vector<lingtu::recording::RecordedMessage> captured;
  for (int attempt = 0; attempt < 100 && captured.empty(); ++attempt) {
    captured =
        lingtu::recording::take_cdr_messages(reader, binding->contract->dds_topic, sequence, 8);
    if (captured.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  require(captured.size() == 1, "did not capture exactly one DDS sample");
  require(captured.front().publish_time_ns == kSourceTimestamp,
          "DDS source timestamp was not preserved");
  require(captured.front().payload.size() > 4, "captured CDR payload is empty");
  require(captured.front().payload[0] == std::byte{0x00},
          "captured CDR representation byte is incorrect");
  require(captured.front().payload[1] == std::byte{0x01},
          "captured CDR endianness byte is incorrect");

  const dds_entity_t replay_topic =
      dds_create_topic(participant, binding->descriptor,
                       "rt/lingtu/test/native_recording/imu/replay", nullptr, nullptr);
  require(replay_topic > 0, "failed to create replay DDS topic");
  const dds_entity_t replay_writer = dds_create_writer(participant, replay_topic, nullptr, nullptr);
  const dds_entity_t replay_reader = dds_create_reader(participant, replay_topic, nullptr, nullptr);
  require(replay_writer > 0, "failed to create replay DDS writer");
  require(replay_reader > 0, "failed to create replay DDS reader");

  const std::string error = lingtu::recording::forward_cdr_message(
      replay_writer, *binding, captured.front().payload, kSourceTimestamp);
  require(error.empty(), "failed to forward captured CDR sample");

  auto *output = lingtu_dds_Imu__alloc();
  void *samples[] = {output};
  dds_sample_info_t replay_info{};
  dds_return_t replay_count = 0;
  for (int attempt = 0; attempt < 100 && replay_count == 0; ++attempt) {
    replay_count = dds_take(replay_reader, samples, &replay_info, 1, 1);
    if (replay_count == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  require(replay_count == 1 && replay_info.valid_data, "did not receive the forwarded DDS sample");
  require(replay_info.source_timestamp == kSourceTimestamp,
          "forwarded DDS source timestamp was not preserved");
  require(std::strcmp(output->header.frame_id, "imu_link") == 0,
          "forwarded CDR frame id is incorrect");
  require(output->header.stamp.sec == 12, "forwarded CDR seconds are incorrect");
  require(output->header.stamp.nanosec == 34, "forwarded CDR nanoseconds are incorrect");
  require(output->angular_velocity.x == 1.25, "forwarded angular velocity is incorrect");
  require(output->linear_acceleration.z == 9.81, "forwarded acceleration is incorrect");
  lingtu_dds_Imu_free(output, static_cast<dds_free_op_t>(DDS_FREE_ALL_BIT));

  auto malformed = captured.front().payload;
  malformed[1] = std::byte{0x00};
  require(
      !lingtu::recording::forward_cdr_message(replay_writer, *binding, malformed, kSourceTimestamp)
           .empty(),
      "malformed CDR representation header was accepted");

  test_joint_telemetry_cdr_round_trip(participant);
  dds_delete(participant);
  return 0;
}
