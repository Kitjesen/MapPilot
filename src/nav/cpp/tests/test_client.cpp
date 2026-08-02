#include "client.hpp"
#include "client_c.h"
#include "clock_sync.hpp"

#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/dds_topics.hpp"
#include "message/cpp/exploration_command.hpp"
#include "message/cpp/inspection_command.hpp"
#include "message/cpp/navigation_command.hpp"
#include "message/cpp/operator_motion.hpp"

#include "dds/dds.h"
#include "lingtu_slam.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <exception>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

namespace {

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

double stampSeconds(const lingtu_dds_Time& stamp) {
  return static_cast<double>(stamp.sec) +
      static_cast<double>(stamp.nanosec) * 1e-9;
}

dds_entity_t checked(dds_return_t value, const char* operation) {
  if (value < 0) {
    throw std::runtime_error(
        std::string(operation) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

void check(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

dds_entity_t createReader(
    dds_entity_t participant,
    dds_entity_t subscriber,
    const lingtu::message::TopicContract& contract,
    const dds_topic_descriptor_t* descriptor) {
  const dds_entity_t topic = checked(
      dds_create_topic(
          participant, descriptor, contract.dds_topic.data(), nullptr, nullptr),
      "dds_create_topic(test_nav_client_reader)");
  auto qos = lingtu::dds::make_qos(
      lingtu::dds::qos_for_topic(contract.dds_topic));
  return checked(
      dds_create_reader(subscriber, topic, qos.get(), nullptr),
      "dds_create_reader(test_nav_client)");
}

dds_entity_t createWriter(
    dds_entity_t participant,
    dds_entity_t publisher,
    const lingtu::message::TopicContract& contract,
    const dds_topic_descriptor_t* descriptor) {
  const dds_entity_t topic = checked(
      dds_create_topic(
          participant, descriptor, contract.dds_topic.data(), nullptr, nullptr),
      "dds_create_topic(test_nav_client_writer)");
  auto qos = lingtu::dds::make_qos(
      lingtu::dds::qos_for_topic(contract.dds_topic));
  return checked(
      dds_create_writer(publisher, topic, qos.get(), nullptr),
      "dds_create_writer(test_nav_client)");
}

template <typename Message>
Message* takeOne(dds_entity_t reader, int timeout_ms = 1000) {
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    void* raw_sample = nullptr;
    dds_sample_info_t info{};
    const dds_return_t count = dds_take(reader, &raw_sample, &info, 1, 1);
    if (count < 0) {
      throw std::runtime_error(
          std::string("dds_take(test_nav_client): ") + dds_strretcode(-count));
    }
    if (count == 1 && info.valid_data) {
      return static_cast<Message*>(raw_sample);
    }
    if (count == 1) {
      checked(
          dds_return_loan(reader, &raw_sample, 1),
          "dds_return_loan(test_nav_client_invalid)");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  throw std::runtime_error("timed out waiting for navigation command request");
}

template <typename Message>
void returnLoan(dds_entity_t reader, Message* sample) {
  void* raw_sample = sample;
  checked(
      dds_return_loan(reader, &raw_sample, 1),
      "dds_return_loan(test_nav_client)");
}

void writeAck(
    dds_entity_t writer,
    const std::string& request_id,
    lingtu::message::NavigationCommandKind kind,
    bool accepted,
    const std::string& reason,
    double endpoint_stamp_s = nowSeconds(),
    const std::string& task_id = {}) {
  lingtu_dds_NavigationCommandAck ack{};
  ack.header.stamp.sec = static_cast<std::int32_t>(endpoint_stamp_s);
  ack.header.stamp.nanosec = static_cast<std::uint32_t>(
      (endpoint_stamp_s - static_cast<double>(ack.header.stamp.sec)) * 1e9);
  ack.header.frame_id = const_cast<char*>("map");
  ack.task_id = const_cast<char*>(task_id.c_str());
  ack.request_id = const_cast<char*>(request_id.c_str());
  ack.kind = static_cast<std::int32_t>(kind);
  ack.accepted = accepted;
  ack.reason = const_cast<char*>(reason.c_str());
  checked(dds_write(writer, &ack), "dds_write(test_nav_client_ack)");
}

void writeExplorationAck(
    dds_entity_t writer,
    const std::string& request_id,
    const std::string& exploration_run_id,
    lingtu::message::ExplorationCommandKind kind,
    bool accepted,
    const std::string& reason,
    const std::string& session_id,
    bool duplicate = false) {
  lingtu_dds_ExplorationCommandAck ack{};
  const double stamp_s = nowSeconds();
  ack.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  ack.header.stamp.nanosec = static_cast<std::uint32_t>(
      (stamp_s - static_cast<double>(ack.header.stamp.sec)) * 1e9);
  ack.header.frame_id = const_cast<char*>("map");
  ack.request_id = const_cast<char*>(request_id.c_str());
  ack.exploration_run_id = const_cast<char*>(exploration_run_id.c_str());
  ack.kind = static_cast<std::int32_t>(kind);
  ack.accepted = accepted;
  ack.duplicate = duplicate;
  ack.reason = const_cast<char*>(reason.c_str());
  ack.session_id = const_cast<char*>(session_id.c_str());
  ack.intent_revision = 17U;
  checked(dds_write(writer, &ack), "dds_write(test_exploration_client_ack)");
}

void writeInspectionTaskAck(
    dds_entity_t writer,
    const std::string& task_id,
    const std::string& request_id,
    lingtu::message::InspectionCommandKind kind,
    bool accepted,
    const std::string& reason) {
  lingtu_dds_InspectionTaskAck ack{};
  const double stamp_s = nowSeconds();
  ack.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  ack.header.stamp.nanosec = static_cast<std::uint32_t>(
      (stamp_s - static_cast<double>(ack.header.stamp.sec)) * 1e9);
  ack.header.frame_id = const_cast<char*>("map");
  ack.task_id = const_cast<char*>(task_id.c_str());
  ack.request_id = const_cast<char*>(request_id.c_str());
  ack.kind = static_cast<std::int32_t>(kind);
  ack.accepted = accepted;
  ack.reason = const_cast<char*>(reason.c_str());
  ack.run_id = const_cast<char*>("inspection-test-run");
  checked(dds_write(writer, &ack), "dds_write(test_inspection_task_client_ack)");
}

void writeOperatorMotionAck(
    dds_entity_t writer,
    const std::string& request_id,
    const std::string& source_id,
    std::uint64_t source_epoch,
    std::uint64_t sequence,
    std::int32_t action,
    bool accepted = true,
    const std::string& reason = "accepted",
    std::uint64_t final_output_sequence = 0U) {
  lingtu_dds_OperatorMotionAck ack{};
  const double stamp_s = nowSeconds();
  ack.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  ack.header.stamp.nanosec = static_cast<std::uint32_t>(
      (stamp_s - static_cast<double>(ack.header.stamp.sec)) * 1e9);
  ack.header.frame_id = const_cast<char*>("");
  ack.source_id = const_cast<char*>(source_id.c_str());
  ack.source_epoch = source_epoch;
  ack.source_sequence = sequence;
  ack.request_id = const_cast<char*>(request_id.c_str());
  ack.action = action;
  ack.accepted = accepted;
  ack.reason = const_cast<char*>(reason.c_str());
  ack.accepted_sequence = accepted ? sequence : 0U;
  ack.final_output_sequence = final_output_sequence;
  checked(dds_write(writer, &ack), "dds_write(test_operator_motion_ack)");
}

void writeGoalStatus(
    dds_entity_t writer,
    const std::string& boot_id,
    std::uint64_t sequence,
    const std::string& task_id,
    const std::string& request_id,
    lingtu::message::NavigationGoalState state,
    std::uint64_t goal_epoch,
    const std::string& reason) {
  lingtu_dds_NavigationGoalStatus status{};
  const double stamp_s = nowSeconds();
  status.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  status.header.stamp.nanosec = static_cast<std::uint32_t>(
      (stamp_s - static_cast<double>(status.header.stamp.sec)) * 1e9);
  status.header.frame_id = const_cast<char*>("map");
  status.boot_id = const_cast<char*>(boot_id.c_str());
  status.event_sequence = sequence;
  status.task_id = const_cast<char*>(task_id.c_str());
  status.request_id = const_cast<char*>(request_id.c_str());
  status.state = static_cast<std::int32_t>(state);
  status.goal_epoch = goal_epoch;
  status.reason = const_cast<char*>(reason.c_str());
  checked(dds_write(writer, &status), "dds_write(test_navigation_goal_status)");
}

void writePath(
    dds_entity_t writer,
    const std::vector<lingtu::nav::commands::PathPoint>& points) {
  lingtu_dds_Path path{};
  const double stamp_s = nowSeconds();
  path.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  path.header.stamp.nanosec = static_cast<std::uint32_t>(
      (stamp_s - static_cast<double>(path.header.stamp.sec)) * 1e9);
  path.header.frame_id = const_cast<char*>("map");
  std::vector<lingtu_dds_PoseStamped> poses(points.size());
  for (std::size_t i = 0U; i < points.size(); ++i) {
    poses[i].header = path.header;
    poses[i].pose.position.x = points[i].x;
    poses[i].pose.position.y = points[i].y;
    poses[i].pose.position.z = points[i].z;
    poses[i].pose.orientation.w = 1.0;
  }
  path.poses._maximum = static_cast<std::uint32_t>(poses.size());
  path.poses._length = static_cast<std::uint32_t>(poses.size());
  path.poses._buffer = poses.data();
  path.poses._release = false;
  checked(dds_write(writer, &path), "dds_write(test_navigation_path)");
}

struct MapSceneCloudFixture {
  lingtu_dds_MapCloudLayer layer{};
  std::array<lingtu_dds_PointField, 4U> fields{};
  std::vector<std::uint8_t> bytes;

  MapSceneCloudFixture(
      const char* name,
      const std::vector<std::array<float, 4U>>& points,
      std::uint64_t reset_epoch,
      std::uint64_t observation_sequence,
      std::uint64_t generation,
      double stamp_s) {
    layer.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
    layer.header.stamp.nanosec = static_cast<std::uint32_t>(
        (stamp_s - static_cast<double>(layer.header.stamp.sec)) * 1e9);
    layer.header.frame_id = const_cast<char*>("map");
    layer.layer = const_cast<char*>(name);
    layer.reset_epoch = reset_epoch;
    layer.observation_sequence = observation_sequence;
    layer.generation = generation;
    layer.live = true;
    layer.cloud.header = layer.header;
    layer.cloud.height = 1U;
    layer.cloud.width = static_cast<std::uint32_t>(points.size());
    layer.cloud.is_bigendian = false;
    layer.cloud.point_step = 4U * sizeof(float);
    layer.cloud.row_step = layer.cloud.width * layer.cloud.point_step;
    layer.cloud.is_dense = true;
    const char* names[4] = {"x", "y", "z", "intensity"};
    for (std::size_t index = 0U; index < fields.size(); ++index) {
      fields[index].name = const_cast<char*>(names[index]);
      fields[index].offset =
          static_cast<std::uint32_t>(index * sizeof(float));
      fields[index].datatype = 7U;
      fields[index].count = 1U;
    }
    layer.cloud.fields._maximum = fields.size();
    layer.cloud.fields._length = fields.size();
    layer.cloud.fields._buffer = fields.data();
    layer.cloud.fields._release = false;
    bytes.resize(points.size() * 4U * sizeof(float));
    for (std::size_t index = 0U; index < points.size(); ++index) {
      std::memcpy(
          bytes.data() + index * 4U * sizeof(float),
          points[index].data(),
          4U * sizeof(float));
    }
    layer.cloud.data._maximum = bytes.size();
    layer.cloud.data._length = bytes.size();
    layer.cloud.data._buffer = bytes.data();
    layer.cloud.data._release = false;
  }
};

struct MapSceneGridFixture {
  lingtu_dds_MapGrid grid{};
  std::vector<float> cells;

  MapSceneGridFixture(
      const char* name,
      std::vector<float> values,
      std::uint64_t reset_epoch,
      std::uint64_t observation_sequence,
      std::uint64_t generation,
      double stamp_s)
      : cells(std::move(values)) {
    grid.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
    grid.header.stamp.nanosec = static_cast<std::uint32_t>(
        (stamp_s - static_cast<double>(grid.header.stamp.sec)) * 1e9);
    grid.header.frame_id = const_cast<char*>("map");
    grid.layer = const_cast<char*>(name);
    grid.info.map_load_time = grid.header.stamp;
    grid.info.resolution = 0.25F;
    grid.info.width = static_cast<std::uint32_t>(cells.size());
    grid.info.height = cells.empty() ? 0U : 1U;
    grid.info.origin.orientation.w = 1.0;
    grid.data._maximum = cells.size();
    grid.data._length = cells.size();
    grid.data._buffer = cells.data();
    grid.data._release = false;
    grid.reset_epoch = reset_epoch;
    grid.observation_sequence = observation_sequence;
    grid.generation = generation;
    grid.live = true;
  }
};

void writeMapScene(
    dds_entity_t writer,
    std::uint64_t generation,
    bool oversized = false) {
  const double stamp_s = nowSeconds();
  constexpr std::uint64_t reset_epoch = 9U;
  constexpr std::uint64_t observation_sequence = 17U;
  MapSceneCloudFixture live(
      "live", {{{1.0F, 2.0F, 0.1F, 4.0F}}},
      reset_epoch, observation_sequence, generation, stamp_s);
  MapSceneCloudFixture voxel(
      "voxel", {{{1.1F, 2.1F, 0.2F, 5.0F}}},
      reset_epoch, observation_sequence, generation, stamp_s);
  MapSceneCloudFixture accumulated(
      "accumulated",
      {{{1.2F, 2.2F, 0.3F, 0.8F}, {3.0F, 4.0F, 0.4F, 0.9F}}},
      reset_epoch, observation_sequence, generation, stamp_s);
  if (oversized) {
    live.layer.cloud.width =
        LINGTU_NAV_MAP_SCENE_MAX_POINTS_PER_LAYER + 1U;
    live.layer.cloud.row_step =
        live.layer.cloud.width * live.layer.cloud.point_step;
  }
  MapSceneGridFixture occupancy(
      "occupancy", {0.0F, 1.0F},
      reset_epoch, observation_sequence, generation, stamp_s);
  MapSceneGridFixture elevation(
      "elevation", {0.1F, 0.2F},
      reset_epoch, observation_sequence, generation, stamp_s);
  MapSceneGridFixture esdf(
      "esdf", {1.0F, 2.0F},
      reset_epoch, observation_sequence, generation, stamp_s);

  lingtu_dds_MapScene scene{};
  scene.header = live.layer.header;
  scene.producer_boot_id = const_cast<char*>("mapd-test-boot");
  scene.reset_epoch = reset_epoch;
  scene.observation_sequence = observation_sequence;
  scene.generation = generation;
  scene.live = true;
  scene.map_sensor.position.x = 1.0;
  scene.map_sensor.orientation.w = 1.0;
  scene.live_cloud = live.layer;
  scene.voxel_cloud = voxel.layer;
  scene.accumulated_cloud = accumulated.layer;
  scene.occupancy = occupancy.grid;
  scene.elevation = elevation.grid;
  scene.esdf = esdf.grid;
  checked(dds_write(writer, &scene), "dds_write(test_map_scene)");
}

void writeMapRuntimeState(
    dds_entity_t writer,
    std::uint64_t generation) {
  lingtu_dds_MapRuntimeState state{};
  const double stamp_s = nowSeconds();
  state.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  state.header.stamp.nanosec = static_cast<std::uint32_t>(
      (stamp_s - static_cast<double>(state.header.stamp.sec)) * 1e9);
  state.header.frame_id = const_cast<char*>("map");
  state.producer_boot_id = const_cast<char*>("mapd-test-boot");
  state.running = true;
  state.live = true;
  state.reset_epoch = 9U;
  state.observation_sequence = 17U;
  state.generation = generation;
  state.pose_quality = 1.0F;
  state.pose_state = const_cast<char*>("TRACKING");
  state.pose_reason = const_cast<char*>("");
  state.required_publications_ready = true;
  state.current_generation_published = true;
  state.state_published_generation = generation;
  state.realtime_clouds_published_generation = generation;
  state.map_layers_published_generation = generation;
  state.scene_published_generation = generation;
  state.engine_error = const_cast<char*>("");
  state.input_error = const_cast<char*>("");
  state.output_error = const_cast<char*>("");
  checked(dds_write(writer, &state), "dds_write(test_map_runtime_state)");
}

template <typename Send, typename Verify>
void sendAndReply(
    dds_entity_t request_reader,
    dds_entity_t ack_writer,
    Send&& send,
    Verify&& verify,
    bool accepted = true,
    const std::string& reason = "accepted") {
  std::exception_ptr sender_error;
  std::thread sender([&]() {
    try {
      send();
    } catch (...) {
      sender_error = std::current_exception();
    }
  });
  auto* request = takeOne<lingtu_dds_NavigationCommandRequest>(request_reader);
  check(
      request->client_id != nullptr && std::string(request->client_id).find("nav-client-") == 0,
      "navigation command client id must identify the native client");
  verify(*request);
  const std::string request_id = request->request_id;
  const std::string task_id = request->task_id == nullptr ? "" : request->task_id;
  const auto kind = static_cast<lingtu::message::NavigationCommandKind>(request->kind);
  writeAck(ack_writer, request_id, kind, accepted, reason, nowSeconds(), task_id);
  returnLoan(request_reader, request);
  sender.join();
  if (sender_error) {
    std::rethrow_exception(sender_error);
  }
}

template <typename Send, typename Verify>
void sendExplorationAndReply(
    dds_entity_t request_reader,
    dds_entity_t ack_writer,
    Send&& send,
    Verify&& verify,
    bool accepted = true,
    const std::string& reason = "accepted",
    bool duplicate = false,
    const std::string& ack_exploration_run_id = {},
    const std::string& ack_session_id = {}) {
  std::exception_ptr sender_error;
  std::thread sender([&]() {
    try {
      send();
    } catch (...) {
      sender_error = std::current_exception();
    }
  });
  auto* request = takeOne<lingtu_dds_ExplorationCommandRequest>(request_reader);
  verify(*request);
  const std::string request_id = request->request_id;
  const std::string exploration_run_id = request->exploration_run_id;
  const std::string session_id = request->session_id;
  const auto kind =
      static_cast<lingtu::message::ExplorationCommandKind>(request->kind);
  writeExplorationAck(
      ack_writer,
      request_id,
      ack_exploration_run_id.empty()
          ? exploration_run_id
          : ack_exploration_run_id,
      kind,
      accepted,
      reason,
      ack_session_id.empty() ? session_id : ack_session_id,
      duplicate);
  returnLoan(request_reader, request);
  sender.join();
  if (sender_error) {
    std::rethrow_exception(sender_error);
  }
}

template <typename Send, typename Verify>
void sendInspectionTaskAndReply(
    dds_entity_t request_reader,
    dds_entity_t ack_writer,
    Send&& send,
    Verify&& verify,
    bool accepted = true,
    const std::string& reason = "accepted") {
  std::exception_ptr sender_error;
  std::thread sender([&]() {
    try {
      send();
    } catch (...) {
      sender_error = std::current_exception();
    }
  });
  auto* request = takeOne<lingtu_dds_InspectionTaskRequest>(request_reader);
  verify(*request);
  const std::string task_id = request->task_id;
  const std::string request_id = request->request_id;
  const auto kind =
      static_cast<lingtu::message::InspectionCommandKind>(request->kind);
  writeInspectionTaskAck(
      ack_writer, task_id, request_id, kind, accepted, reason);
  returnLoan(request_reader, request);
  sender.join();
  if (sender_error) {
    std::rethrow_exception(sender_error);
  }
}

void testExplorationCommands() {
  using ExplorationKind = lingtu::message::ExplorationCommandKind;
  const std::string exploration_run_id = "01ARZ3NDEKTSV4RRFFQ69G5FAV";
  const int domain_id = 205 + static_cast<int>(getpid() % 10);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_exploration_client)");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber(test_exploration_client)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_exploration_client)");
  const dds_entity_t request_reader = createReader(
      participant,
      subscriber,
      lingtu::message::kNavExplorationCommand,
      &lingtu_dds_ExplorationCommandRequest_desc);
  const dds_entity_t ack_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavExplorationAck,
      &lingtu_dds_ExplorationCommandAck_desc);

  lingtu::nav::commands::Client session(domain_id);
  auto& exploration = session.exploration();
  sendExplorationAndReply(
      request_reader,
      ack_writer,
      [&]() {
        exploration.start(
            exploration_run_id,
            "session-a",
            "operator_start",
            1000,
            "explore-start");
      },
      [&](const lingtu_dds_ExplorationCommandRequest& request) {
        check(
            request.kind == static_cast<std::int32_t>(ExplorationKind::kStart),
            "exploration start kind mismatch");
        check(std::string(request.header.frame_id) == "map",
              "exploration command frame mismatch");
        check(std::string(request.session_id) == "session-a",
              "exploration session id mismatch");
        check(std::string(request.exploration_run_id) == exploration_run_id,
              "exploration run id mismatch");
        check(std::string(request.reason) == "operator_start",
              "exploration start reason mismatch");
      });
  sendExplorationAndReply(
      request_reader,
      ack_writer,
      [&]() {
        (void)exploration.pause(
            exploration_run_id, "session-a", "operator_pause", 1000, "explore-pause");
      },
      [&](const lingtu_dds_ExplorationCommandRequest& request) {
        check(
            request.kind == static_cast<std::int32_t>(ExplorationKind::kPause),
            "exploration pause kind mismatch");
        check(std::string(request.session_id) == "session-a",
              "exploration pause must retain the active session binding");
      });
  sendExplorationAndReply(
      request_reader,
      ack_writer,
      [&]() {
        (void)exploration.resume(
            exploration_run_id, "session-a", "operator_resume", 1000, "explore-resume");
      },
      [&](const lingtu_dds_ExplorationCommandRequest& request) {
        check(
            request.kind == static_cast<std::int32_t>(ExplorationKind::kResume),
            "exploration resume kind mismatch");
        check(std::string(request.session_id) == "session-a",
              "exploration resume must retain the active session binding");
      });
  sendExplorationAndReply(
      request_reader,
      ack_writer,
      [&]() {
        (void)exploration.stop(
            exploration_run_id, "session-a", "operator_stop", 1000, "explore-stop");
      },
      [&](const lingtu_dds_ExplorationCommandRequest& request) {
        check(
            request.kind == static_cast<std::int32_t>(ExplorationKind::kStop),
            "exploration stop kind mismatch");
        check(std::string(request.session_id) == "session-a",
              "exploration stop must retain the active session binding");
        check(std::string(request.reason) == "operator_stop",
              "exploration stop reason mismatch");
      });
  sendExplorationAndReply(
      request_reader,
      ack_writer,
      [&]() {
        exploration.setDirectedTarget(
            12.5,
            -8.25,
            45.0,
            exploration_run_id,
            "session-a",
            "operator_directed_explore",
            1000,
            "explore-directed-set");
      },
      [&](const lingtu_dds_ExplorationCommandRequest& request) {
        check(
            request.kind == static_cast<std::int32_t>(
                ExplorationKind::kSetDirectedTarget),
            "directed target set kind mismatch");
        check(
            std::string(request.request_id) == "explore-directed-set",
            "directed target set request id mismatch");
        check(request.has_directed_target, "directed target set flag mismatch");
        check(std::abs(request.directed_target_x - 12.5) < 1e-9,
              "directed target x mismatch");
        check(std::abs(request.directed_target_y + 8.25) < 1e-9,
              "directed target y mismatch");
        check(std::abs(request.directed_target_ttl_s - 45.0) < 1e-9,
              "directed target ttl mismatch");
        check(std::string(request.session_id) == "session-a",
              "directed target session mismatch");
        check(std::string(request.reason) == "operator_directed_explore",
              "directed target reason mismatch");
      });
  sendExplorationAndReply(
      request_reader,
      ack_writer,
      [&]() {
        exploration.clearDirectedTarget(
            exploration_run_id,
            "session-a",
            "operator_clear_directed_explore",
            1000,
            "explore-directed-clear");
      },
      [&](const lingtu_dds_ExplorationCommandRequest& request) {
        check(
            request.kind == static_cast<std::int32_t>(
                ExplorationKind::kClearDirectedTarget),
            "directed target clear kind mismatch");
        check(
            std::string(request.request_id) == "explore-directed-clear",
            "directed target clear request id mismatch");
        check(
            std::string(request.session_id) == "session-a",
            "directed target clear session mismatch");
        check(!request.has_directed_target, "directed target clear flag mismatch");
        check(std::abs(request.directed_target_x) < 1e-12,
              "directed target clear x must be empty");
        check(std::abs(request.directed_target_y) < 1e-12,
              "directed target clear y must be empty");
        check(std::abs(request.directed_target_ttl_s) < 1e-12,
              "directed target clear ttl must be empty");
        check(std::string(request.reason) == "operator_clear_directed_explore",
              "directed target clear reason mismatch");
      });

  lingtu::nav::commands::ExplorationCommandReceipt rejected_receipt;
  sendExplorationAndReply(
      request_reader,
      ack_writer,
      [&]() {
        rejected_receipt = exploration.start(
            exploration_run_id,
            "session-b",
            "operator_start",
            1000,
            "explore-reject");
      },
      [&](const lingtu_dds_ExplorationCommandRequest&) {},
      false,
      "exploration_inputs_not_ready");
  check(
      !rejected_receipt.accepted &&
          rejected_receipt.request_id == "explore-reject" &&
          rejected_receipt.exploration_run_id == exploration_run_id &&
          rejected_receipt.reason == "exploration_inputs_not_ready" &&
          !rejected_receipt.duplicate,
      "exploration command rejection was not returned as a typed receipt");

  lingtu::nav::commands::ExplorationCommandReceipt duplicate_receipt;
  sendExplorationAndReply(
      request_reader,
      ack_writer,
      [&]() {
        duplicate_receipt = exploration.pause(
            exploration_run_id,
            "session-a",
            "operator_pause",
            1000,
            "explore-pause-duplicate");
      },
      [&](const lingtu_dds_ExplorationCommandRequest&) {},
      true,
      "already_applied",
      true);
  check(
      duplicate_receipt.accepted && duplicate_receipt.duplicate &&
          duplicate_receipt.reason == "already_applied",
      "exploration duplicate ACK was not preserved in the typed receipt");

  bool corrupt_ack_rejected = false;
  try {
    sendExplorationAndReply(
        request_reader,
        ack_writer,
        [&]() {
          (void)exploration.resume(
              exploration_run_id,
              "session-a",
              "operator_resume",
              1000,
              "explore-corrupt-ack");
        },
        [&](const lingtu_dds_ExplorationCommandRequest&) {},
        true,
        "accepted",
        false,
        "01ARZ3NDEKTSV4RRFFQ69G5FAW");
  } catch (const std::runtime_error& exc) {
    corrupt_ack_rejected =
        std::string(exc.what()).find("exploration_ack_run_id_mismatch") !=
        std::string::npos;
  }
  check(corrupt_ack_rejected, "exploration ACK run identity mismatch was accepted");

  bool missing_identity_rejected = false;
  try {
    (void)exploration.stop(
        "", "session-a", "operator_stop", 1, "missing-run-id");
  } catch (const std::invalid_argument&) {
    missing_identity_rejected = true;
  }
  check(missing_identity_rejected, "empty exploration run id crossed DDS");

  lingtu_nav_client_handle c_client = lingtu_nav_client_create(domain_id);
  check(c_client != nullptr, "C exploration receipt client creation failed");
  lingtu_nav_exploration_command_receipt_v1 c_receipt{};
  c_receipt.abi_version = LINGTU_NAV_EXPLORATION_COMMAND_RECEIPT_ABI_VERSION;
  c_receipt.struct_size = sizeof(c_receipt);
  sendExplorationAndReply(
      request_reader,
      ack_writer,
      [&]() {
        check(
            lingtu_nav_client_stop_exploration_with_receipt_v1(
                c_client,
                "c-explore-stop",
                exploration_run_id.c_str(),
                "session-a",
                "operator_stop",
                1000,
                &c_receipt) == 0,
            "C exploration command receipt call failed");
      },
      [&](const lingtu_dds_ExplorationCommandRequest&) {},
      false,
      "stop_not_allowed",
      true);
  check(
      c_receipt.abi_version ==
              LINGTU_NAV_EXPLORATION_COMMAND_RECEIPT_ABI_VERSION &&
          c_receipt.struct_size == sizeof(c_receipt) &&
          c_receipt.accepted == 0 && c_receipt.duplicate != 0 &&
          std::string(c_receipt.request_id) == "c-explore-stop" &&
          std::string(c_receipt.exploration_run_id) == exploration_run_id &&
          std::string(c_receipt.reason) == "stop_not_allowed",
      "C exploration receipt lost business rejection fields");
  lingtu_nav_client_destroy(c_client);
  dds_delete(participant);
}

void testExplorationRunEventReader() {
  using EventKind = lingtu::message::ExplorationRunEventKind;
  using RunState = lingtu::message::ExplorationRunState;
  const int domain_id = 195 + static_cast<int>(getpid() % 10);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_exploration_run_event_client)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_exploration_run_event_client)");
  const dds_entity_t event_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavExplorationRunEvent,
      &lingtu_dds_ExplorationRunEvent_desc);
  lingtu::nav::commands::Client client(domain_id);
  lingtu_nav_client_handle c_client = lingtu_nav_client_create(domain_id);
  check(c_client != nullptr, "C exploration run event client creation failed");

  const std::string run_id = "01ARZ3NDEKTSV4RRFFQ69G5FAV";
  auto publish = [&](std::uint64_t sequence,
                     EventKind kind,
                     RunState state,
                     bool motion_stop_confirmed,
                     const std::string& motion_stop_reason) {
    lingtu_dds_ExplorationRunEvent event{};
    event.timestamp_s = nowSeconds();
    event.frame_id = const_cast<char*>("map");
    event.boot_id = const_cast<char*>("explore-boot-a");
    event.event_sequence = sequence;
    event.kind = static_cast<std::int32_t>(kind);
    event.exploration_run_id = const_cast<char*>(run_id.c_str());
    event.start_request_id = const_cast<char*>("explore-start-1");
    event.command_request_id = const_cast<char*>("explore-command-1");
    event.product_session_id = const_cast<char*>("product-session-a");
    event.state = static_cast<std::int32_t>(state);
    event.route = const_cast<char*>("live");
    event.map_id = const_cast<char*>("");
    event.map_version = 0;
    event.artifact_hash = const_cast<char*>("");
    event.reason = const_cast<char*>("state_changed");
    event.motion_stop_confirmed = motion_stop_confirmed;
    event.motion_stop_reason = const_cast<char*>(motion_stop_reason.c_str());
    checked(dds_write(event_writer, &event), "dds_write(test_exploration_run_event)");
  };
  auto take = [&](int timeout_ms) {
    lingtu::nav::commands::ExplorationRunEventSnapshot event;
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
      if (client.takeExplorationRunEvent(&event)) {
        return std::optional<
            lingtu::nav::commands::ExplorationRunEventSnapshot>(std::move(event));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return std::optional<lingtu::nav::commands::ExplorationRunEventSnapshot>{};
  };

  bool first_received = false;
  for (int attempt = 0; attempt < 100; ++attempt) {
    publish(1U, EventKind::kStateChanged, RunState::kRunning, false, "");
    const auto first = take(10);
    if (!first.has_value()) {
      continue;
    }
    check(
        first->timestamp_s > 0.0 && first->frame_id == "map" &&
            first->boot_id == "explore-boot-a" && first->event_sequence == 1U &&
            first->kind == static_cast<std::int32_t>(EventKind::kStateChanged) &&
            first->exploration_run_id == run_id &&
            first->start_request_id == "explore-start-1" &&
            first->command_request_id == "explore-command-1" &&
            first->product_session_id == "product-session-a" &&
            first->state == static_cast<std::int32_t>(RunState::kRunning) &&
            first->route == "live" && first->map_id.empty() &&
            first->map_version == 0 && first->artifact_hash.empty() &&
            first->reason == "state_changed" && !first->motion_stop_confirmed &&
            first->motion_stop_reason.empty(),
        "exploration run event snapshot lost lifecycle fields");
    first_received = true;
    break;
  }
  check(first_received, "exploration run event was not received");

  publish(1U, EventKind::kStateChanged, RunState::kFailed, true, "parked");
  publish(2U, EventKind::kStateChanged, RunState::kPaused, true, "parked");
  const auto paused = take(1000);
  check(
      paused.has_value() && paused->event_sequence == 2U &&
          paused->motion_stop_confirmed && paused->motion_stop_reason == "parked",
      "duplicate exploration run sequence leaked or parking evidence was lost");

  publish(3U, EventKind::kStateChanged, RunState::kCompleted, false, "");
  publish(4U, EventKind::kStateChanged, RunState::kCancelling, false, "");
  const auto cancelling = take(1000);
  check(
      cancelling.has_value() && cancelling->event_sequence == 4U,
      "unparked terminal exploration event was not rejected");

  publish(5U, EventKind::kStateChanged, RunState::kCancelled, true, "parked");
  lingtu_nav_exploration_run_event_v1 c_event{};
  c_event.abi_version = LINGTU_NAV_EXPLORATION_RUN_EVENT_ABI_VERSION;
  c_event.struct_size = sizeof(c_event);
  int c_result = 0;
  for (int attempt = 0; attempt < 200 && c_result == 0; ++attempt) {
    c_result = lingtu_nav_client_take_exploration_run_event_v1(c_client, &c_event);
    if (c_result == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  check(
      c_result == 1 && c_event.abi_version ==
              LINGTU_NAV_EXPLORATION_RUN_EVENT_ABI_VERSION &&
          c_event.struct_size == sizeof(c_event) && c_event.event_sequence > 0U &&
          std::string(c_event.exploration_run_id) == run_id &&
          std::string(c_event.product_session_id) == "product-session-a",
      "C exploration run event ABI did not return the native fact");

  lingtu_nav_client_destroy(c_client);
  dds_delete(participant);
}

void testInspectionTaskCommands() {
  using InspectionKind = lingtu::message::InspectionCommandKind;
  // CycloneDDS' default UDP port mapping overflows above domain 232.
  const int domain_id = 220 + static_cast<int>(getpid() % 10);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_inspection_client)");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber(test_inspection_client)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_inspection_client)");
  const dds_entity_t request_reader = createReader(
      participant,
      subscriber,
      lingtu::message::kNavInspectionTaskRequest,
      &lingtu_dds_InspectionTaskRequest_desc);
  const dds_entity_t ack_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavInspectionTaskAck,
      &lingtu_dds_InspectionTaskAck_desc);

  lingtu::nav::commands::Client session(domain_id);
  auto& inspection = session.inspection();
  lingtu::nav::commands::InspectionTaskCommandReceipt start_receipt;
  sendInspectionTaskAndReply(
      request_reader,
      ack_writer,
      [&]() {
        start_receipt = inspection.startTask(
            "inspection-task-1", "route-a", 7, 1000, "inspection-start");
      },
      [&](const lingtu_dds_InspectionTaskRequest& request) {
        check(request.kind == static_cast<std::int32_t>(InspectionKind::kStart),
              "inspection start kind mismatch");
        check(std::string(request.header.frame_id) == "map",
              "inspection frame mismatch");
        check(std::string(request.task_id) == "inspection-task-1",
              "inspection task id mismatch");
        check(std::string(request.request_id) == "inspection-start",
              "inspection request id mismatch");
        check(std::string(request.route_id) == "route-a",
              "inspection route id mismatch");
        check(request.route_revision == 7U,
              "inspection route revision mismatch");
      });
  check(
      start_receipt.accepted && start_receipt.task_id == "inspection-task-1" &&
          start_receipt.request_id == "inspection-start" &&
          start_receipt.run_id == "inspection-test-run",
      "inspection task start receipt lost correlation identity");

  sendInspectionTaskAndReply(
      request_reader,
      ack_writer,
      [&]() {
        (void)inspection.pauseTask(
            "inspection-task-1", "operator_hold", 1000, "inspection-pause");
      },
      [&](const lingtu_dds_InspectionTaskRequest& request) {
        check(request.kind == static_cast<std::int32_t>(InspectionKind::kPause),
              "inspection pause kind mismatch");
        check(std::string(request.reason) == "operator_hold",
              "inspection pause reason mismatch");
        check(std::string(request.task_id) == "inspection-task-1",
              "inspection pause task id mismatch");
      });
  sendInspectionTaskAndReply(
      request_reader,
      ack_writer,
      [&]() {
        (void)inspection.resumeTask(
            "inspection-task-1", "operator_resume", 1000, "inspection-resume");
      },
      [&](const lingtu_dds_InspectionTaskRequest& request) {
        check(request.kind == static_cast<std::int32_t>(InspectionKind::kResume),
              "inspection resume kind mismatch");
      });
  sendInspectionTaskAndReply(
      request_reader,
      ack_writer,
      [&]() {
        (void)inspection.cancelTask(
            "inspection-task-1", "operator_cancel", 1000, "inspection-cancel");
      },
      [&](const lingtu_dds_InspectionTaskRequest& request) {
        check(request.kind == static_cast<std::int32_t>(InspectionKind::kCancel),
              "inspection cancel kind mismatch");
      });

  bool rejected = false;
  try {
    sendInspectionTaskAndReply(
        request_reader,
        ack_writer,
        [&]() {
          (void)inspection.startTask(
              "inspection-task-2", "route-b", 1, 1000, "inspection-reject");
        },
        [&](const lingtu_dds_InspectionTaskRequest&) {},
        false,
        "inspection_route_not_found");
  } catch (const std::runtime_error& exc) {
    rejected =
        std::string(exc.what()).find("inspection_route_not_found") !=
        std::string::npos;
  }
  check(rejected, "inspection rejection ACK was not surfaced to caller");

  bool rejected_missing_task = false;
  try {
    (void)inspection.pauseTask("", "operator_hold", 10, "missing-task");
  } catch (const std::invalid_argument&) {
    rejected_missing_task = true;
  }
  check(rejected_missing_task,
        "inspection task client must reject an empty task id before DDS delivery");

  lingtu_nav_client_handle c_client = lingtu_nav_client_create(domain_id);
  check(c_client != nullptr, "C inspection task client creation failed");
  sendInspectionTaskAndReply(
      request_reader,
      ack_writer,
      [&]() {
        check(
            lingtu_nav_client_start_inspection_task(
                c_client,
                "inspection-task-c",
                "inspection-start-c",
                "route-c",
                11U,
                1000) == 0,
            "C inspection task start failed");
      },
      [&](const lingtu_dds_InspectionTaskRequest& request) {
        check(std::string(request.task_id) == "inspection-task-c",
              "C inspection task id mismatch");
        check(std::string(request.request_id) == "inspection-start-c",
              "C inspection request id mismatch");
      });
  lingtu_nav_client_destroy(c_client);
  dds_delete(participant);
}

void testOperatorMotionAckCorrelationUsesFullSourceIdentity() {
  using OperatorAction = lingtu::message::OperatorMotionAction;
  const int domain_id = 205 + static_cast<int>(getpid() % 5);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_operator_motion_client)");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber(test_operator_motion_client)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_operator_motion_client)");
  const dds_entity_t control_reader = createReader(
      participant,
      subscriber,
      lingtu::message::kOperatorMotionControl,
      &lingtu_dds_OperatorMotionControl_desc);
  const dds_entity_t ack_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kOperatorMotionAck,
      &lingtu_dds_OperatorMotionAck_desc);

  lingtu::nav::commands::Client client(domain_id);
  bool rejected_zero_sequence = false;
  try {
    client.operatorMotion().claim(
        "ws:operator-a", 42U, 0U, 1000U, 2000, "zero-sequence");
  } catch (const std::invalid_argument&) {
    rejected_zero_sequence = true;
  }
  check(
      rejected_zero_sequence,
      "operator motion control must reject sequence zero before DDS delivery");
  std::exception_ptr sender_error;
  std::atomic<bool> sender_done{false};
  std::thread sender([&]() {
    try {
      client.operatorMotion().claim(
          "ws:operator-a", 42U, 1U, 1000U, 2000, "shared-request-id");
    } catch (...) {
      sender_error = std::current_exception();
    }
    sender_done.store(true, std::memory_order_release);
  });

  auto* request = takeOne<lingtu_dds_OperatorMotionControl>(control_reader, 2000);
  check(
      std::string(request->source_id) == "ws:operator-a" &&
          request->source_epoch == 42U && request->source_sequence == 1U,
      "operator motion claim identity mismatch");
  const std::string request_id = request->request_id;
  const std::int32_t action = request->action;
  returnLoan(control_reader, request);

  writeOperatorMotionAck(
      ack_writer, request_id, "ws:operator-b", 42U, 1U, action);
  writeOperatorMotionAck(
      ack_writer, request_id, "ws:operator-a", 41U, 1U, action);
  writeOperatorMotionAck(
      ack_writer, request_id, "ws:operator-a", 42U, 7U, action);
  writeOperatorMotionAck(
      ack_writer,
      request_id,
      "ws:operator-a",
      42U,
      1U,
      static_cast<std::int32_t>(OperatorAction::Hold));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const bool completed_before_matching_ack =
      sender_done.load(std::memory_order_acquire);

  writeOperatorMotionAck(
      ack_writer, request_id, "ws:operator-a", 42U, 1U, action);
  sender.join();
  if (sender_error) {
    std::rethrow_exception(sender_error);
  }
  check(
      sender_done.load(std::memory_order_acquire),
      "matching operator ACK did not complete the pending claim");
  check(
      !completed_before_matching_ack,
      "foreign or stale operator ACK must not complete the pending claim");
  dds_delete(participant);
}
void testOperatorMotionReceiptApis() {
  using OperatorAction = lingtu::message::OperatorMotionAction;
  using Receipt = lingtu::nav::commands::OperatorMotionCommandReceipt;
  const int domain_id = 95 + static_cast<int>(getpid() % 5);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_operator_motion_receipts)");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber(test_operator_motion_receipts)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_operator_motion_receipts)");
  const dds_entity_t control_reader = createReader(
      participant,
      subscriber,
      lingtu::message::kOperatorMotionControl,
      &lingtu_dds_OperatorMotionControl_desc);
  const dds_entity_t ack_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kOperatorMotionAck,
      &lingtu_dds_OperatorMotionAck_desc);

  lingtu::nav::commands::Client client(domain_id);

  std::optional<Receipt> accepted_receipt;
  std::exception_ptr accepted_error;
  std::thread accepted_sender([&]() {
    try {
      accepted_receipt = client.operatorMotion().holdWithReceipt(
          "ws:receipt-a", 51U, 2U, "disconnect_hold", 2000);
    } catch (...) {
      accepted_error = std::current_exception();
    }
  });
  auto* accepted_request =
      takeOne<lingtu_dds_OperatorMotionControl>(control_reader, 2000);
  const std::string generated_request_id = accepted_request->request_id;
  const auto accepted_action = accepted_request->action;
  check(
      !generated_request_id.empty(),
      "empty operator request id was not materialized before DDS publication");
  returnLoan(control_reader, accepted_request);
  writeOperatorMotionAck(
      ack_writer,
      generated_request_id,
      "ws:receipt-a",
      51U,
      2U,
      accepted_action,
      true,
      "hold_zero_published",
      71U);
  accepted_sender.join();
  if (accepted_error) {
    std::rethrow_exception(accepted_error);
  }
  check(accepted_receipt.has_value(), "accepted operator receipt was not returned");
  check(
      accepted_receipt->accepted &&
          accepted_receipt->action ==
              static_cast<std::int32_t>(OperatorAction::Hold) &&
          accepted_receipt->request_id == generated_request_id &&
          accepted_receipt->source_id == "ws:receipt-a" &&
          accepted_receipt->source_epoch == 51U &&
          accepted_receipt->source_sequence == 2U &&
          accepted_receipt->accepted_sequence == 2U &&
          accepted_receipt->final_output_sequence == 71U &&
          accepted_receipt->endpoint_timestamp_s > 0.0 &&
          accepted_receipt->reason == "hold_zero_published",
      "accepted operator receipt did not preserve the complete ACK");

  std::optional<Receipt> rejected_receipt;
  std::exception_ptr rejected_error;
  std::thread rejected_sender([&]() {
    try {
      rejected_receipt = client.operatorMotion().releaseWithReceipt(
          "ws:receipt-a",
          51U,
          3U,
          "operator_release",
          2000,
          "release-rejected");
    } catch (...) {
      rejected_error = std::current_exception();
    }
  });
  auto* rejected_request =
      takeOne<lingtu_dds_OperatorMotionControl>(control_reader, 2000);
  returnLoan(control_reader, rejected_request);
  writeOperatorMotionAck(
      ack_writer,
      "release-rejected",
      "ws:receipt-a",
      51U,
      3U,
      static_cast<std::int32_t>(OperatorAction::Release),
      false,
      "operator_source_not_owner");
  rejected_sender.join();
  if (rejected_error) {
    std::rethrow_exception(rejected_error);
  }
  check(
      rejected_receipt.has_value() && !rejected_receipt->accepted &&
          rejected_receipt->request_id == "release-rejected" &&
          rejected_receipt->accepted_sequence == 0U &&
          rejected_receipt->final_output_sequence == 0U &&
          rejected_receipt->reason == "operator_source_not_owner",
      "rejected operator ACK was not returned as a structured receipt");

  std::string legacy_rejection;
  std::exception_ptr legacy_error;
  std::thread legacy_sender([&]() {
    try {
      client.operatorMotion().hold(
          "ws:receipt-a",
          51U,
          4U,
          "legacy_hold",
          2000,
          "legacy-rejected");
    } catch (const std::runtime_error& exc) {
      legacy_rejection = exc.what();
    } catch (...) {
      legacy_error = std::current_exception();
    }
  });
  auto* legacy_request =
      takeOne<lingtu_dds_OperatorMotionControl>(control_reader, 2000);
  returnLoan(control_reader, legacy_request);
  writeOperatorMotionAck(
      ack_writer,
      "legacy-rejected",
      "ws:receipt-a",
      51U,
      4U,
      static_cast<std::int32_t>(OperatorAction::Hold),
      false,
      "legacy_denied");
  legacy_sender.join();
  if (legacy_error) {
    std::rethrow_exception(legacy_error);
  }
  check(
      legacy_rejection.find("legacy_denied") != std::string::npos,
      "legacy operator API no longer throws on a rejected ACK");

  lingtu_nav_client_handle c_client = lingtu_nav_client_create(domain_id);
  check(c_client != nullptr, "C operator receipt client creation failed");
  check(
      lingtu_nav_client_operator_motion_sample(
          c_client,
          "invalid-deadman",
          "c:receipt",
          77U,
          1U,
          2,
          0.2,
          0.0,
          0.0,
          350U,
          1) == -1,
      "C operator sample accepted a non-boolean deadman");
  check(
      std::string(lingtu_nav_client_last_error(c_client)).find(
          "deadman must be 0 or 1") != std::string::npos,
      "C operator sample did not explain invalid deadman");
  lingtu_nav_operator_motion_receipt_v1 invalid_version{};
  invalid_version.abi_version =
      LINGTU_NAV_OPERATOR_MOTION_RECEIPT_ABI_VERSION + 1U;
  invalid_version.struct_size = sizeof(invalid_version);
  check(
      lingtu_nav_client_operator_motion_claim_with_receipt_v1(
          c_client,
          "invalid-version",
          "c:receipt",
          77U,
          1U,
          1000U,
          1,
          &invalid_version) == -1,
      "C receipt ABI accepted an unsupported caller version");
  check(
      std::string(lingtu_nav_client_last_error(c_client)).find(
          "ABI version mismatch") != std::string::npos,
      "C receipt ABI did not explain a version mismatch");

  lingtu_nav_operator_motion_receipt_v1 invalid_size{};
  invalid_size.abi_version =
      LINGTU_NAV_OPERATOR_MOTION_RECEIPT_ABI_VERSION;
  invalid_size.struct_size = sizeof(invalid_size) - 1U;
  check(
      lingtu_nav_client_operator_motion_claim_with_receipt_v1(
          c_client,
          "invalid-size",
          "c:receipt",
          77U,
          1U,
          1000U,
          1,
          &invalid_size) == -1,
      "C receipt ABI accepted an undersized caller buffer");
  check(
      std::string(lingtu_nav_client_last_error(c_client)).find(
          "struct is too small") != std::string::npos,
      "C receipt ABI did not explain an undersized buffer");
  lingtu_nav_operator_motion_receipt_v1 c_receipt{};
  c_receipt.abi_version = LINGTU_NAV_OPERATOR_MOTION_RECEIPT_ABI_VERSION;
  c_receipt.struct_size = sizeof(c_receipt) + 64U;
  int c_result = -2;
  std::thread c_sender([&]() {
    c_result = lingtu_nav_client_operator_motion_claim_with_receipt_v1(
        c_client,
        nullptr,
        "c:receipt",
        77U,
        1U,
        1000U,
        2000,
        &c_receipt);
  });
  auto* c_request =
      takeOne<lingtu_dds_OperatorMotionControl>(control_reader, 2000);
  const std::string c_generated_request_id = c_request->request_id;
  returnLoan(control_reader, c_request);
  writeOperatorMotionAck(
      ack_writer,
      c_generated_request_id,
      "c:receipt",
      77U,
      1U,
      static_cast<std::int32_t>(OperatorAction::Claim),
      true,
      "authority_claimed");
  c_sender.join();
  check(c_result == 0, "C receipt call failed for an accepted ACK");
  check(
      c_receipt.abi_version ==
              LINGTU_NAV_OPERATOR_MOTION_RECEIPT_ABI_VERSION &&
          c_receipt.struct_size == sizeof(c_receipt) &&
          c_receipt.accepted != 0 &&
          c_receipt.action ==
              static_cast<std::int32_t>(OperatorAction::Claim) &&
          std::string(c_receipt.request_id) == c_generated_request_id &&
          std::string(c_receipt.source_id) == "c:receipt" &&
          c_receipt.source_epoch == 77U &&
          c_receipt.source_sequence == 1U &&
          c_receipt.accepted_sequence == 1U &&
          c_receipt.final_output_sequence == 0U &&
          c_receipt.endpoint_timestamp_s > 0.0 &&
          std::string(c_receipt.reason) == "authority_claimed",
      "C receipt out-parameter did not preserve the complete ACK");

  lingtu_nav_operator_motion_receipt_v1 c_rejected{};
  c_rejected.abi_version = LINGTU_NAV_OPERATOR_MOTION_RECEIPT_ABI_VERSION;
  c_rejected.struct_size = sizeof(c_rejected);
  int c_rejected_result = -2;
  std::thread c_rejected_sender([&]() {
    c_rejected_result =
        lingtu_nav_client_operator_motion_release_with_receipt_v1(
            c_client,
            "c-release-rejected",
            "c:receipt",
            77U,
            2U,
            "operator_release",
            2000,
            &c_rejected);
  });
  auto* c_rejected_request =
      takeOne<lingtu_dds_OperatorMotionControl>(control_reader, 2000);
  returnLoan(control_reader, c_rejected_request);
  writeOperatorMotionAck(
      ack_writer,
      "c-release-rejected",
      "c:receipt",
      77U,
      2U,
      static_cast<std::int32_t>(OperatorAction::Release),
      false,
      "release_denied");
  c_rejected_sender.join();
  check(
      c_rejected_result == 0 && c_rejected.accepted == 0 &&
          c_rejected.accepted_sequence == 0U &&
          c_rejected.final_output_sequence == 0U &&
          std::string(c_rejected.reason) == "release_denied",
      "C receipt ABI treated a reliable rejected ACK as a transport error");

  lingtu_nav_client_destroy(c_client);
  dds_delete(participant);
}

void testEstopBypassesGoalAckWait() {
  using NavigationKind = lingtu::message::NavigationCommandKind;
  const int domain_id = 215 + static_cast<int>(getpid() % 5);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_estop_priority)");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber(test_estop_priority)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_estop_priority)");
  const dds_entity_t request_reader = createReader(
      participant,
      subscriber,
      lingtu::message::kNavCommandRequest,
      &lingtu_dds_NavigationCommandRequest_desc);
  const dds_entity_t ack_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavCommandAck,
      &lingtu_dds_NavigationCommandAck_desc);

  lingtu_nav_client_handle client = lingtu_nav_client_create(domain_id);
  check(client != nullptr, "C ABI client creation failed in priority test");
  int goal_result = 0;
  int estop_result = 0;
  std::string goal_error;
  std::string estop_error;
  std::exception_ptr observation_error;
  lingtu_nav_navigation_command_receipt_v1 goal_receipt{};
  goal_receipt.abi_version =
      LINGTU_NAV_NAVIGATION_COMMAND_RECEIPT_ABI_VERSION;
  goal_receipt.struct_size = sizeof(goal_receipt);
  std::thread goal_sender([&]() {
    goal_result = lingtu_nav_client_start_task_with_receipt_v1(
        client,
        "priority-task",
        "goal-without-ack",
        1.0,
        2.0,
        0.0,
        0.0,
        500,
        &goal_receipt);
    if (goal_result != 0) {
      goal_error = lingtu_nav_client_last_error(client);
    }
  });

  auto* goal_request =
      takeOne<lingtu_dds_NavigationCommandRequest>(request_reader);
  check(
      std::string(goal_request->request_id) == "goal-without-ack",
      "priority test did not observe the blocked goal");
  returnLoan(request_reader, goal_request);

  const auto estop_started = std::chrono::steady_clock::now();
  std::chrono::steady_clock::duration estop_wire_delay{};
  std::thread estop_sender([&]() {
    estop_result = lingtu_nav_client_estop_with_id(
        client, "priority-estop", "operator_estop", 500);
    if (estop_result != 0) {
      estop_error = lingtu_nav_client_last_error(client);
    }
  });

  try {
    auto* estop_request =
        takeOne<lingtu_dds_NavigationCommandRequest>(request_reader, 250);
    estop_wire_delay = std::chrono::steady_clock::now() - estop_started;
    check(
        std::string(estop_request->request_id) == "priority-estop",
        "priority test observed the wrong request id");
    check(
        estop_request->kind == static_cast<std::int32_t>(NavigationKind::Estop),
        "priority request must be estop");
    writeAck(
        ack_writer,
        "priority-estop",
        NavigationKind::Estop,
        true,
        "estop_latched");
    returnLoan(request_reader, estop_request);
  } catch (...) {
    observation_error = std::current_exception();
  }

  estop_sender.join();
  goal_sender.join();
  lingtu_nav_client_destroy(client);
  dds_delete(participant);
  if (observation_error) {
    std::rethrow_exception(observation_error);
  }
  check(
      estop_result == 0,
      estop_error.empty() ? "estop failed in priority test" : estop_error.c_str());
  check(goal_result != 0, "goal without ACK must time out");
  check(
      goal_error.find("timed out waiting for navigation command ACK") !=
          std::string::npos,
      "goal timeout did not preserve the calling thread's C ABI error");
  check(
      estop_wire_delay < std::chrono::milliseconds(250),
      "estop was blocked behind the goal ACK wait");
}

void runTest() {
  using CommandKind = lingtu::message::NavigationCommandKind;
  const int domain_id = 170 + static_cast<int>(getpid() % 20);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_nav_client)");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber(test_nav_client)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_nav_client)");
  const dds_entity_t request_reader = createReader(
      participant,
      subscriber,
      lingtu::message::kNavCommandRequest,
      &lingtu_dds_NavigationCommandRequest_desc);
  const dds_entity_t ack_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavCommandAck,
      &lingtu_dds_NavigationCommandAck_desc);

  lingtu::nav::commands::Client client(domain_id);
  lingtu::nav::commands::NavigationCommandReceipt accepted_goal_receipt;
  sendAndReply(
      request_reader,
      ack_writer,
      [&]() {
        accepted_goal_receipt = client.navigation().startTask(
            1.25, -2.5, 0.4, 0.6, 1000, "", "goal-001");
      },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(std::string(request.request_id) == "goal-001", "goal request id mismatch");
        check(request.task_id != nullptr && std::string(request.task_id) != "goal-001", "goal task id must differ from request id");
        check(request.kind == static_cast<std::int32_t>(CommandKind::Goal), "goal kind mismatch");
        check(std::abs(request.goal.position.x - 1.25) < 1e-9, "goal x mismatch");
        check(std::abs(request.goal.position.y + 2.5) < 1e-9, "goal y mismatch");
        check(std::string(request.header.frame_id) == "map", "goal frame mismatch");
        check(std::abs(request.goal.orientation.z - std::sin(0.3)) < 1e-9, "goal yaw mismatch");
      });
  check(
      accepted_goal_receipt.accepted &&
          accepted_goal_receipt.request_id == "goal-001" &&
          !accepted_goal_receipt.task_id.empty() &&
          accepted_goal_receipt.task_id != accepted_goal_receipt.request_id,
      "startTask must return distinct task/request ids for lifecycle correlation");
  lingtu::nav::commands::NavigationCommandReceipt pause_receipt;
  sendAndReply(
      request_reader,
      ack_writer,
      [&]() {
        pause_receipt = client.navigation().pauseTask(
            accepted_goal_receipt.task_id, "operator_pause", 1000, "pause-001");
      },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(request.kind == static_cast<std::int32_t>(CommandKind::TaskPause),
              "task pause kind mismatch");
        check(std::string(request.task_id) == accepted_goal_receipt.task_id,
              "task pause task id mismatch");
        check(std::string(request.reason) == "operator_pause", "task pause reason mismatch");
      },
      true,
      "pause_requested");
  check(pause_receipt.accepted && pause_receipt.task_id == accepted_goal_receipt.task_id &&
            pause_receipt.request_id == "pause-001",
        "pauseTask must retain task identity and return its causal request id");

  lingtu::nav::commands::NavigationCommandReceipt resume_receipt;
  sendAndReply(
      request_reader,
      ack_writer,
      [&]() {
        resume_receipt = client.navigation().resumeTask(
            accepted_goal_receipt.task_id, "operator_resume", 1000, "resume-001");
      },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(request.kind == static_cast<std::int32_t>(CommandKind::TaskResume),
              "task resume kind mismatch");
        check(std::string(request.task_id) == accepted_goal_receipt.task_id,
              "task resume task id mismatch");
        check(std::string(request.reason) == "operator_resume", "task resume reason mismatch");
      },
      true,
      "resume_requested");
  check(resume_receipt.accepted && resume_receipt.task_id == accepted_goal_receipt.task_id &&
            resume_receipt.request_id == "resume-001",
        "resumeTask must retain task identity and use a new request id");
  lingtu::nav::commands::NavigationCommandReceipt cancel_receipt;
  sendAndReply(
      request_reader,
      ack_writer,
      [&]() {
        cancel_receipt = client.navigation().cancelTask(
            accepted_goal_receipt.task_id,
            "operator_cancel",
            1000,
            "cancel-001");
      },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(request.kind == static_cast<std::int32_t>(CommandKind::TaskCancel), "task cancel kind mismatch");
        check(std::string(request.task_id) == accepted_goal_receipt.task_id, "cancel task id mismatch");
        check(std::string(request.reason) == "operator_cancel", "cancel reason mismatch");
      });
  check(
      cancel_receipt.accepted &&
          cancel_receipt.task_id == accepted_goal_receipt.task_id &&
          cancel_receipt.request_id == "cancel-001",
      "cancelTask must target the accepted goal task");

  lingtu_nav_client_handle c_client = lingtu_nav_client_create(domain_id);
  check(c_client != nullptr, "C navigation client creation failed");
  lingtu_nav_navigation_command_receipt_v1 c_goal_receipt{};
  c_goal_receipt.abi_version =
      LINGTU_NAV_NAVIGATION_COMMAND_RECEIPT_ABI_VERSION;
  c_goal_receipt.struct_size = sizeof(c_goal_receipt);
  sendAndReply(
      request_reader,
      ack_writer,
      [&]() {
        check(
            lingtu_nav_client_start_task_with_receipt_v1(
                c_client,
                "",
                "c-goal-001",
                2.0,
                3.0,
                0.0,
                0.0,
                1000,
                &c_goal_receipt) == 0,
            "C start task receipt call failed");
      },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(std::string(request.request_id) == "c-goal-001", "C goal request id mismatch");
        check(request.task_id != nullptr && std::string(request.task_id) != "c-goal-001", "C goal task id must differ from request id");
        check(request.kind == static_cast<std::int32_t>(CommandKind::Goal), "C goal kind mismatch");
      });
  check(
      c_goal_receipt.accepted != 0 &&
          std::string(c_goal_receipt.request_id) == "c-goal-001" &&
          std::string(c_goal_receipt.task_id) != "c-goal-001",
      "C start task receipt did not preserve task/request identity");

  lingtu_nav_navigation_command_receipt_v1 c_pause_receipt{};
  c_pause_receipt.abi_version =
      LINGTU_NAV_NAVIGATION_COMMAND_RECEIPT_ABI_VERSION;
  c_pause_receipt.struct_size = sizeof(c_pause_receipt);
  sendAndReply(
      request_reader,
      ack_writer,
      [&]() {
        check(
            lingtu_nav_client_pause_task_with_receipt_v1(
                c_client,
                c_goal_receipt.task_id,
                "c-pause-001",
                "operator_pause",
                1000,
                &c_pause_receipt) == 0,
            "C pause task receipt call failed");
      },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(
            std::string(request.task_id) == c_goal_receipt.task_id,
            "C pause task id mismatch");
        check(
            std::string(request.request_id) == "c-pause-001",
            "C pause request id mismatch");
        check(
            request.kind == static_cast<std::int32_t>(CommandKind::TaskPause),
            "C pause kind mismatch");
      },
      true,
      "pause_requested");
  check(
      c_pause_receipt.accepted != 0 &&
          std::string(c_pause_receipt.task_id) == c_goal_receipt.task_id &&
          std::string(c_pause_receipt.request_id) == "c-pause-001",
      "C pause task receipt did not preserve task/request identity");

  lingtu_nav_navigation_command_receipt_v1 c_resume_receipt{};
  c_resume_receipt.abi_version =
      LINGTU_NAV_NAVIGATION_COMMAND_RECEIPT_ABI_VERSION;
  c_resume_receipt.struct_size = sizeof(c_resume_receipt);
  sendAndReply(
      request_reader,
      ack_writer,
      [&]() {
        check(
            lingtu_nav_client_resume_task_with_receipt_v1(
                c_client,
                c_goal_receipt.task_id,
                "c-resume-001",
                "operator_resume",
                1000,
                &c_resume_receipt) == 0,
            "C resume task receipt call failed");
      },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(
            std::string(request.task_id) == c_goal_receipt.task_id,
            "C resume task id mismatch");
        check(
            std::string(request.request_id) == "c-resume-001",
            "C resume request id mismatch");
        check(
            request.kind == static_cast<std::int32_t>(CommandKind::TaskResume),
            "C resume kind mismatch");
      },
      true,
      "resume_requested");
  check(
      c_resume_receipt.accepted != 0 &&
          std::string(c_resume_receipt.task_id) == c_goal_receipt.task_id &&
          std::string(c_resume_receipt.request_id) == "c-resume-001",
      "C resume task receipt did not preserve task/request identity");

  lingtu_nav_navigation_command_receipt_v1 c_cancel_receipt{};
  c_cancel_receipt.abi_version =
      LINGTU_NAV_NAVIGATION_COMMAND_RECEIPT_ABI_VERSION;
  c_cancel_receipt.struct_size = sizeof(c_cancel_receipt);
  sendAndReply(
      request_reader,
      ack_writer,
      [&]() {
        check(
            lingtu_nav_client_cancel_task_with_receipt_v1(
                c_client,
                c_goal_receipt.task_id,
                "c-cancel-001",
                "operator_cancel",
                1000,
                &c_cancel_receipt) == 0,
            "C cancel task receipt call failed");
      },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(std::string(request.task_id) == c_goal_receipt.task_id, "C cancel task id mismatch");
        check(std::string(request.request_id) == "c-cancel-001", "C cancel request id mismatch");
        check(request.kind == static_cast<std::int32_t>(CommandKind::TaskCancel), "C task cancel kind mismatch");
      });
  check(
      c_cancel_receipt.accepted != 0 &&
          std::string(c_cancel_receipt.task_id) == c_goal_receipt.task_id,
      "C cancel task receipt did not target the original task");
  lingtu_nav_client_destroy(c_client);

  sendAndReply(
      request_reader,
      ack_writer,
      [&]() { client.navigation().stop("operator_stop", 1000, "stop-001"); },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(request.kind == static_cast<std::int32_t>(CommandKind::Stop), "stop kind mismatch");
        check(std::string(request.reason) == "operator_stop", "stop reason mismatch");
      });

  sendAndReply(
      request_reader,
      ack_writer,
      [&]() { client.navigation().estop("operator_estop", 1000, "estop-001"); },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(request.kind == static_cast<std::int32_t>(CommandKind::Estop), "estop kind mismatch");
        check(std::string(request.reason) == "operator_estop", "estop reason mismatch");
      });

  sendAndReply(
      request_reader,
      ack_writer,
      [&]() { client.navigation().clearEstop("operator_reset", 1000, "clear-estop-001"); },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(
            request.kind == static_cast<std::int32_t>(CommandKind::ClearEstop),
            "clear estop kind mismatch");
        check(std::string(request.reason) == "operator_reset", "clear estop reason mismatch");
      });

  sendAndReply(
      request_reader,
      ack_writer,
      [&]() { client.navigation().resumeAutonomy("operator_resume", 1000, "resume-001"); },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(
            request.kind == static_cast<std::int32_t>(CommandKind::ResumeAutonomy),
            "resume autonomy kind mismatch");
        check(
            std::string(request.reason) == "operator_resume",
            "resume autonomy reason mismatch");
      });

  lingtu::nav::commands::NavigationCommandReceipt rejected_goal;
  sendAndReply(
      request_reader,
      ack_writer,
      [&]() {
        rejected_goal = client.navigation().startTask(
            9.0, 9.0, 0.0, 0.0, 1000, "goal-reject-task", "goal-reject");
      },
      [&](const lingtu_dds_NavigationCommandRequest&) {},
      false,
      "active_octomap_not_configured");
  check(
      !rejected_goal.accepted &&
          rejected_goal.reason == "active_octomap_not_configured",
      "rejected command ACK was not surfaced to caller");

  std::string stale_rejection;
  std::exception_ptr stale_sender_error;
  std::thread stale_sender([&]() {
    try {
      client.navigation().clearEstop(
          "operator_reset", 1000, "clear-estop-reject-with-clock-diagnostics");
    } catch (...) {
      stale_sender_error = std::current_exception();
    }
  });
  auto* first_stale =
      takeOne<lingtu_dds_NavigationCommandRequest>(request_reader);
  check(
      std::string(first_stale->request_id) ==
          "clear-estop-reject-with-clock-diagnostics",
      "first stale clear-estop request id mismatch");
  writeAck(
      ack_writer,
      first_stale->request_id,
      CommandKind::ClearEstop,
      false,
      "clear_estop_source_stamp_stale");
  returnLoan(request_reader, first_stale);

  auto* recovery_sync =
      takeOne<lingtu_dds_NavigationCommandRequest>(request_reader);
  check(
      recovery_sync->kind == static_cast<std::int32_t>(CommandKind::Stop) &&
          std::string(recovery_sync->reason) == "client_clock_sync",
      "clock rejection recovery must establish time through a safe stop");
  writeAck(
      ack_writer,
      recovery_sync->request_id,
      CommandKind::Stop,
      true,
      "stopped");
  returnLoan(request_reader, recovery_sync);

  auto* retry_stale =
      takeOne<lingtu_dds_NavigationCommandRequest>(request_reader);
  check(
      std::string(retry_stale->request_id) ==
          "clear-estop-reject-with-clock-diagnostics-clock-retry-1",
      "clock recovery retry must use a distinct traceable request id");
  writeAck(
      ack_writer,
      retry_stale->request_id,
      CommandKind::ClearEstop,
      false,
      "clear_estop_source_stamp_stale");
  returnLoan(request_reader, retry_stale);
  stale_sender.join();
  if (stale_sender_error) {
    try {
      std::rethrow_exception(stale_sender_error);
    } catch (const std::runtime_error& exc) {
      stale_rejection = exc.what();
    }
  }
  check(
      stale_rejection.find("clear_estop_source_stamp_stale") != std::string::npos,
      "stale clear-estop rejection was not surfaced to caller");
  for (const char* field : {
           "sync_rtt_ms=",
           "endpoint_stamp_s=",
           "local_wall_receive_s=",
           "clock_offset_s=",
           "send_source_stamp_s=",
       }) {
    check(
        stale_rejection.find(field) != std::string::npos,
        "clock rejection exception is missing audit diagnostics");
  }

  dds_delete(participant);
}

void testNavigationGoalStatusReaderAndRetention() {
  const int domain_id = 110 + static_cast<int>(getpid() % 10);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_goal_status_client)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_goal_status_client)");
  const dds_entity_t status_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavGoalStatus,
      &lingtu_dds_NavigationGoalStatus_desc);
  lingtu::nav::commands::Client client(domain_id);
  lingtu_nav_client_handle c_client = lingtu_nav_client_create(domain_id);
  check(c_client != nullptr, "C goal status client creation failed");

  auto take = [&](int timeout_ms) {
    lingtu::nav::commands::NavigationGoalStatusSnapshot status;
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
      if (client.takeNavigationGoalStatus(&status)) {
        return std::optional<
            lingtu::nav::commands::NavigationGoalStatusSnapshot>(
            std::move(status));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return std::optional<
        lingtu::nav::commands::NavigationGoalStatusSnapshot>{};
  };

  const std::string task_id = "task-lifecycle-1";
  const std::string request_id = "goal-lifecycle-1";
  for (int attempt = 0; attempt < 100; ++attempt) {
    writeGoalStatus(
        status_writer,
        "navd-boot-a",
        1U,
        task_id,
        request_id,
        lingtu::message::NavigationGoalState::Planning,
        7U,
        "planning");
    auto first = take(10);
    if (first.has_value()) {
      check(first->task_id == task_id, "goal status task id mismatch");
      check(first->request_id == request_id, "goal status request id mismatch");
      check(first->sequence == 1U, "goal status sequence mismatch");
      break;
    }
  }
  const auto retained_planning = client.navigationGoalStatus(request_id);
  check(retained_planning.has_value(), "goal status must be retained by request id");
  const auto retained_task_planning = client.navigationTaskStatus(task_id);
  check(retained_task_planning.has_value(), "goal status must be retained by task id");

  writeGoalStatus(
      status_writer,
      "navd-boot-a",
      1U,
      task_id,
      request_id,
      lingtu::message::NavigationGoalState::Failed,
      7U,
      "duplicate_must_be_ignored");
  writeGoalStatus(
      status_writer,
      "navd-boot-a",
      2U,
      task_id,
      request_id,
      lingtu::message::NavigationGoalState::Reached,
      7U,
      "goal_reached");
  const auto terminal = take(1000);
  check(terminal.has_value(), "terminal goal status was not received");
  check(
      terminal->state == static_cast<std::int32_t>(
          lingtu::message::NavigationGoalState::Reached),
      "duplicate sequence replaced the terminal lifecycle event");
  check(terminal->sequence == 2U, "terminal goal status sequence mismatch");

  const auto retained_terminal = client.navigationGoalStatus(request_id);
  check(retained_terminal.has_value(), "terminal goal status was not retained");
  check(
      retained_terminal->state == terminal->state &&
          retained_terminal->sequence == terminal->sequence,
      "retained goal status does not match the latest event");
  const auto retained_task_terminal = client.navigationTaskStatus(task_id);
  check(
      retained_task_terminal.has_value() &&
          retained_task_terminal->state == terminal->state &&
          retained_task_terminal->request_id == request_id,
      "retained task status does not match the latest event");
  lingtu_nav_navigation_goal_status_v1 c_task_status{};
  c_task_status.abi_version = LINGTU_NAV_NAVIGATION_GOAL_STATUS_ABI_VERSION;
  c_task_status.struct_size = sizeof(c_task_status);
  bool c_task_status_found = false;
  for (int attempt = 0; attempt < 100 && !c_task_status_found; ++attempt) {
    const int result = lingtu_nav_client_get_navigation_task_status_v1(
        c_client, task_id.c_str(), &c_task_status);
    check(result >= 0, "C task status query failed");
    c_task_status_found = result == 1;
    if (!c_task_status_found) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  check(
      c_task_status_found &&
          std::string(c_task_status.task_id) == task_id &&
          std::string(c_task_status.request_id) == request_id &&
          c_task_status.state == terminal->state,
      "C task status query did not return the retained terminal state");
  check(
      !take(50).has_value(),
      "duplicate goal status sequence leaked into the event queue");

  lingtu_nav_client_destroy(c_client);
  dds_delete(participant);
}

void testNavigationPathTelemetry() {
  const int domain_id = 125 + static_cast<int>(getpid() % 10);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_path_client)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_path_client)");
  const dds_entity_t global_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavGlobalPath,
      &lingtu_dds_Path_desc);
  const dds_entity_t local_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavLocalPath,
      &lingtu_dds_Path_desc);
  lingtu::nav::commands::Client client(domain_id);
  const std::vector<lingtu::nav::commands::PathPoint> global_points{
      {1.0, 2.0, 0.1},
      {3.0, 4.0, 0.2},
  };
  const std::vector<lingtu::nav::commands::PathPoint> local_points{
      {0.1, 0.2, 0.0},
      {0.3, 0.4, 0.0},
      {0.5, 0.6, 0.0},
  };

  lingtu::nav::commands::PathSnapshot global;
  lingtu::nav::commands::PathSnapshot local;
  bool received = false;
  for (int attempt = 0; attempt < 100 && !received; ++attempt) {
    writePath(global_writer, global_points);
    writePath(local_writer, local_points);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    received =
        client.takeGlobalPath(&global) && client.takeLocalPath(&local);
  }
  check(received, "native client did not receive global/local path telemetry");
  check(global.frame_id == "map", "global path frame mismatch");
  check(global.points.size() == global_points.size(), "global path point count mismatch");
  check(local.points.size() == local_points.size(), "local path point count mismatch");
  check(
      std::abs(global.points[1].z - 0.2) < 1e-12,
      "global path point payload mismatch");
  check(
      global.receive_sequence > 0U && local.receive_sequence > 0U,
      "path receive sequence was not assigned");

  lingtu_nav_client_handle c_client = lingtu_nav_client_create(domain_id);
  check(c_client != nullptr, "C path telemetry client creation failed");
  lingtu_nav_path_header header{};
  int result = 0;
  for (int attempt = 0; attempt < 100 && result == 0; ++attempt) {
    writePath(global_writer, global_points);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    result = lingtu_nav_client_take_global_path(c_client, &header, nullptr, 0U);
  }
  check(result == 2, "C path telemetry probe did not request a point buffer");
  check(
      header.point_count == global_points.size(),
      "C path telemetry probe returned the wrong point count");
  std::vector<lingtu_nav_path_point> copied(
      static_cast<std::size_t>(header.point_count));
  result = lingtu_nav_client_take_global_path(
      c_client,
      &header,
      copied.data(),
      static_cast<unsigned long long>(copied.size()));
  check(result == 1, "C path telemetry buffered copy failed");
  check(
      std::abs(copied[1].x - 3.0) < 1e-12,
      "C path telemetry copied the wrong point payload");
  lingtu_nav_client_destroy(c_client);
  dds_delete(participant);
}

void testMapSceneTelemetryAndCapacityGate() {
  const int domain_id = 135 + static_cast<int>(getpid() % 10);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_map_scene_client)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_map_scene_client)");
  const dds_entity_t scene_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kMapsScene,
      &lingtu_dds_MapScene_desc);
  const dds_entity_t state_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kMapsState,
      &lingtu_dds_MapRuntimeState_desc);
  lingtu_nav_client_handle client = lingtu_nav_client_create(domain_id);
  check(client != nullptr, "C MapScene client creation failed");

  lingtu_nav_map_scene_header_v1 header{};
  int result = 0;
  lingtu_nav_map_scene_health_v1 health{};
  for (int attempt = 0; attempt < 100; ++attempt) {
    writeMapRuntimeState(state_writer, 3U);
    writeMapScene(scene_writer, 3U);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    result = lingtu_nav_client_take_map_scene_v1(
        client, &header, nullptr);
    check(
        lingtu_nav_client_read_map_scene_health_v1(client, &health) == 1,
        "C MapScene health read failed");
    if (result == 2 && health.state_received != 0) {
      break;
    }
  }
  check(result == 2, "C MapScene probe did not retain a buffered sample");
  check(
      header.abi_version == LINGTU_NAV_MAP_SCENE_ABI_VERSION &&
          header.generation == 3U &&
          header.live_point_count == 1U &&
          header.voxel_point_count == 1U &&
          header.accumulated_point_count == 2U,
      "C MapScene probe returned the wrong identity or point counts");
  check(
      health.state_running != 0 && health.state_live != 0 &&
          health.state_current_generation_published != 0 &&
          health.state_scene_published_generation == 3U,
      "C MapScene health did not expose current mapd state evidence");

  std::vector<lingtu_nav_map_scene_point_v1> live(
      static_cast<std::size_t>(header.live_point_count));
  std::vector<lingtu_nav_map_scene_point_v1> voxel(
      static_cast<std::size_t>(header.voxel_point_count));
  std::vector<lingtu_nav_map_scene_point_v1> accumulated(
      static_cast<std::size_t>(header.accumulated_point_count));
  std::vector<float> occupancy(header.occupancy.cell_count);
  std::vector<float> elevation(header.elevation.cell_count);
  std::vector<float> esdf(header.esdf.cell_count);
  lingtu_nav_map_scene_buffers_v1 buffers{};
  buffers.abi_version = LINGTU_NAV_MAP_SCENE_ABI_VERSION;
  buffers.struct_size = sizeof(buffers);
  buffers.live_points = live.data();
  buffers.live_point_capacity = live.size();
  buffers.voxel_points = voxel.data();
  buffers.voxel_point_capacity = voxel.size();
  buffers.accumulated_points = accumulated.data();
  buffers.accumulated_point_capacity = accumulated.size();
  buffers.occupancy_cells = occupancy.data();
  buffers.occupancy_cell_capacity = occupancy.size();
  buffers.elevation_cells = elevation.data();
  buffers.elevation_cell_capacity = elevation.size();
  buffers.esdf_cells = esdf.data();
  buffers.esdf_cell_capacity = esdf.size();
  result = lingtu_nav_client_take_map_scene_v1(
      client, &header, &buffers);
  check(result == 1, "C MapScene buffered copy failed");
  check(
      std::abs(accumulated[1].x - 3.0F) < 1e-6F &&
          std::abs(esdf[1] - 2.0F) < 1e-6F,
      "C MapScene copied the wrong point or grid payload");

  bool rejected = false;
  for (int attempt = 0; attempt < 100 && !rejected; ++attempt) {
    writeMapScene(scene_writer, 4U, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    check(
        lingtu_nav_client_read_map_scene_health_v1(client, &health) == 1,
        "C MapScene capacity health read failed");
    rejected = health.capacity_rejections > 0U;
  }
  check(rejected, "oversized MapScene did not trip the product capacity gate");
  check(
      health.last_generation == 3U,
      "oversized MapScene advanced the accepted generation");
  result = lingtu_nav_client_take_map_scene_v1(
      client, &header, nullptr);
  check(result == 0, "oversized MapScene leaked into the consumer queue");

  lingtu_nav_client_destroy(client);
  dds_delete(participant);
}

void testSourceStampIsRefreshedAfterDiscovery() {
  using CommandKind = lingtu::message::NavigationCommandKind;
  const int domain_id = 195 + static_cast<int>(getpid() % 20);
  lingtu::nav::commands::Client client(domain_id);
  std::exception_ptr sender_error;
  std::thread sender([&]() {
    try {
      client.navigation().clearEstop(
          "operator_reset", 2000, "clear-estop-delayed-discovery");
    } catch (...) {
      sender_error = std::current_exception();
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_nav_client_delayed)");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber(test_nav_client_delayed)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_nav_client_delayed)");
  const dds_entity_t request_reader = createReader(
      participant,
      subscriber,
      lingtu::message::kNavCommandRequest,
      &lingtu_dds_NavigationCommandRequest_desc);
  const dds_entity_t ack_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavCommandAck,
      &lingtu_dds_NavigationCommandAck_desc);

  auto* sync = takeOne<lingtu_dds_NavigationCommandRequest>(request_reader, 2000);
  check(
      sync->kind == static_cast<std::int32_t>(CommandKind::Stop),
      "delayed first clear-estop must establish the endpoint clock");
  writeAck(
      ack_writer,
      sync->request_id,
      CommandKind::Stop,
      true,
      "stopped");
  returnLoan(request_reader, sync);
  auto* request = takeOne<lingtu_dds_NavigationCommandRequest>(request_reader, 2000);
  check(
      request->kind == static_cast<std::int32_t>(CommandKind::ClearEstop),
      "clock synchronization must be followed by clear-estop");
  const double publication_age_s = nowSeconds() - stampSeconds(request->header.stamp);
  check(
      publication_age_s >= 0.0 && publication_age_s < 0.25,
      "source stamp must be refreshed after delayed DDS discovery");
  writeAck(
      ack_writer,
      request->request_id,
      CommandKind::ClearEstop,
      true,
      "accepted");
  returnLoan(request_reader, request);
  sender.join();
  if (sender_error) {
    std::rethrow_exception(sender_error);
  }
  dds_delete(participant);
}

void testFirstClockSensitiveCommandUsesEndpointClockAnchor() {
  using CommandKind = lingtu::message::NavigationCommandKind;
  constexpr double kEndpointClockOffsetS = -1.0;
  const int domain_id = 150 + static_cast<int>(getpid() % 10);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_nav_client_clock_anchor)");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber(test_nav_client_clock_anchor)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_nav_client_clock_anchor)");
  const dds_entity_t request_reader = createReader(
      participant,
      subscriber,
      lingtu::message::kNavCommandRequest,
      &lingtu_dds_NavigationCommandRequest_desc);
  const dds_entity_t ack_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavCommandAck,
      &lingtu_dds_NavigationCommandAck_desc);

  lingtu::nav::commands::Client client(domain_id);
  std::exception_ptr sender_error;
  std::thread sender([&]() {
    try {
      client.navigation().clearEstop(
          "operator_reset", 2000, "clear-estop-clock-anchor");
    } catch (...) {
      sender_error = std::current_exception();
    }
  });

  auto* first = takeOne<lingtu_dds_NavigationCommandRequest>(request_reader, 2000);
  const bool first_was_clock_sync =
      first->kind == static_cast<std::int32_t>(CommandKind::Stop) &&
      std::string(first->reason) == "client_clock_sync";
  lingtu_dds_NavigationCommandRequest* command = first;
  if (first_was_clock_sync) {
    const double sync_endpoint_s = nowSeconds() + kEndpointClockOffsetS;
    writeAck(
        ack_writer,
        first->request_id,
        CommandKind::Stop,
        true,
        "stopped",
        sync_endpoint_s);
    returnLoan(request_reader, first);
    command = takeOne<lingtu_dds_NavigationCommandRequest>(request_reader, 2000);
  }
  const bool second_was_clear_estop =
      command->kind == static_cast<std::int32_t>(CommandKind::ClearEstop);
  const double endpoint_receive_s = nowSeconds() + kEndpointClockOffsetS;
  const double source_age_s =
      endpoint_receive_s - stampSeconds(command->header.stamp);
  const bool source_age_valid = source_age_s >= -0.05 && source_age_s < 0.25;
  writeAck(
      ack_writer,
      command->request_id,
      CommandKind::ClearEstop,
      true,
      "accepted",
      endpoint_receive_s);
  returnLoan(request_reader, command);

  sender.join();
  if (sender_error) {
    std::rethrow_exception(sender_error);
  }
  check(
      first_was_clock_sync,
      "a fresh motion client must establish endpoint time with a safe stop");
  check(
      second_was_clear_estop,
      "clock synchronization must be followed by the requested clear-estop command");
  check(
      source_age_valid,
      "first clear-estop source stamp must use the endpoint clock domain");
  dds_delete(participant);
}

void testDelayedClockAckIsResampledBeforeClockSensitiveCommand() {
  using CommandKind = lingtu::message::NavigationCommandKind;
  constexpr double kEndpointClockOffsetS = -1.0;
  const int domain_id = 140 + static_cast<int>(getpid() % 10);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_nav_client_delayed_clock_ack)");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber(test_nav_client_delayed_clock_ack)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_nav_client_delayed_clock_ack)");
  const dds_entity_t request_reader = createReader(
      participant,
      subscriber,
      lingtu::message::kNavCommandRequest,
      &lingtu_dds_NavigationCommandRequest_desc);
  const dds_entity_t ack_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavCommandAck,
      &lingtu_dds_NavigationCommandAck_desc);

  lingtu::nav::commands::Client client(domain_id);
  std::exception_ptr sender_error;
  std::thread sender([&]() {
    try {
      client.navigation().clearEstop(
          "operator_reset", 2500, "clear-estop-delayed-clock-ack");
    } catch (...) {
      sender_error = std::current_exception();
    }
  });

  auto* first_sync =
      takeOne<lingtu_dds_NavigationCommandRequest>(request_reader, 2000);
  check(
      first_sync->kind == static_cast<std::int32_t>(CommandKind::Stop) &&
          std::string(first_sync->reason) == "client_clock_sync",
      "first delayed-clock request must be a safe clock synchronization stop");
  const double delayed_ack_stamp_s = nowSeconds() + kEndpointClockOffsetS;
  const std::string first_sync_id = first_sync->request_id;
  returnLoan(request_reader, first_sync);
  std::this_thread::sleep_for(std::chrono::milliseconds(350));
  writeAck(
      ack_writer,
      first_sync_id,
      CommandKind::Stop,
      true,
      "stopped",
      delayed_ack_stamp_s);

  auto* second =
      takeOne<lingtu_dds_NavigationCommandRequest>(request_reader, 2000);
  const bool delayed_sample_was_resampled =
      second->kind == static_cast<std::int32_t>(CommandKind::Stop) &&
      std::string(second->reason) == "client_clock_sync";
  lingtu_dds_NavigationCommandRequest* command = second;
  if (delayed_sample_was_resampled) {
    writeAck(
        ack_writer,
        second->request_id,
        CommandKind::Stop,
        true,
        "stopped",
        nowSeconds() + kEndpointClockOffsetS);
    returnLoan(request_reader, second);
    command =
        takeOne<lingtu_dds_NavigationCommandRequest>(request_reader, 2000);
  }
  const bool clear_estop_followed_sync =
      command->kind == static_cast<std::int32_t>(CommandKind::ClearEstop);
  const double endpoint_receive_s = nowSeconds() + kEndpointClockOffsetS;
  const double source_age_s =
      endpoint_receive_s - stampSeconds(command->header.stamp);
  const bool source_age_valid = source_age_s >= -0.05 && source_age_s < 0.25;
  writeAck(
      ack_writer,
      command->request_id,
      CommandKind::ClearEstop,
      true,
      "accepted",
      endpoint_receive_s);
  returnLoan(request_reader, command);

  sender.join();
  if (sender_error) {
    std::rethrow_exception(sender_error);
  }
  check(
      delayed_sample_was_resampled,
      "a delayed clock ACK must be resampled instead of aging the command");
  check(
      clear_estop_followed_sync,
      "clock resampling must preserve the clear-estop request");
  check(
      source_age_valid,
      "clear-estop after clock resampling must satisfy the endpoint freshness window");
  dds_delete(participant);
}

void testConcurrentClientsKeepClockAcksIsolated() {
  using CommandKind = lingtu::message::NavigationCommandKind;
  constexpr double kEndpointClockOffsetS = -0.75;
  const int domain_id = 120 + static_cast<int>(getpid() % 20);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_nav_client_concurrent)");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber(test_nav_client_concurrent)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_nav_client_concurrent)");
  const dds_entity_t request_reader = createReader(
      participant,
      subscriber,
      lingtu::message::kNavCommandRequest,
      &lingtu_dds_NavigationCommandRequest_desc);
  const dds_entity_t ack_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavCommandAck,
      &lingtu_dds_NavigationCommandAck_desc);

  lingtu::nav::commands::Client first_client(domain_id);
  lingtu::nav::commands::Client second_client(domain_id);
  std::exception_ptr first_error;
  std::exception_ptr second_error;
  std::thread first_sender([&]() {
    try {
      first_client.navigation().clearEstop(
          "operator_reset_a", 2000, "clear-estop-client-a");
    } catch (...) {
      first_error = std::current_exception();
    }
  });
  std::thread second_sender([&]() {
    try {
      second_client.navigation().clearEstop(
          "operator_reset_b", 2000, "clear-estop-client-b");
    } catch (...) {
      second_error = std::current_exception();
    }
  });

  std::set<std::string> sync_ids;
  bool first_command_seen = false;
  bool second_command_seen = false;
  bool source_ages_valid = true;
  for (int i = 0; i < 4; ++i) {
    auto* request =
        takeOne<lingtu_dds_NavigationCommandRequest>(request_reader, 2000);
    const auto kind = static_cast<CommandKind>(request->kind);
    const std::string request_id = request->request_id;
    if (kind == CommandKind::Stop &&
        std::string(request->reason) == "client_clock_sync") {
      sync_ids.insert(request_id);
      writeAck(
          ack_writer,
          request_id,
          CommandKind::Stop,
          true,
          "stopped",
          nowSeconds() + kEndpointClockOffsetS);
    } else if (kind == CommandKind::ClearEstop) {
      const double endpoint_receive_s = nowSeconds() + kEndpointClockOffsetS;
      const double source_age_s =
          endpoint_receive_s - stampSeconds(request->header.stamp);
      source_ages_valid = source_ages_valid &&
          source_age_s >= -0.05 && source_age_s < 0.25;
      if (request_id == "clear-estop-client-a") {
        first_command_seen = std::string(request->reason) == "operator_reset_a";
      } else if (request_id == "clear-estop-client-b") {
        second_command_seen = std::string(request->reason) == "operator_reset_b";
      }
      writeAck(
          ack_writer,
          request_id,
          CommandKind::ClearEstop,
          true,
          "accepted",
          endpoint_receive_s);
    }
    returnLoan(request_reader, request);
  }

  first_sender.join();
  second_sender.join();
  if (first_error) {
    std::rethrow_exception(first_error);
  }
  if (second_error) {
    std::rethrow_exception(second_error);
  }
  check(sync_ids.size() == 2, "each concurrent client must use its own clock ACK");
  check(first_command_seen, "first concurrent clear-estop was not delivered");
  check(second_command_seen, "second concurrent clear-estop was not delivered");
  check(
      source_ages_valid,
      "concurrent clock-sensitive command timestamps must remain in endpoint time");
  dds_delete(participant);
}

void testClockOffsetTracksRealtimeRollback() {
  constexpr double kEndpointAckStampS = 1000.0;
  constexpr double kLocalReceiveWallS = 1001.0;
  constexpr double kSteadyElapsedS = 0.20;
  constexpr double kLocalWallAfterRollbackS = 1000.60;
  constexpr double kFutureToleranceS = 0.05;

  const double offset = lingtu::nav::commands::endpointClockOffset(
      kEndpointAckStampS,
      kLocalReceiveWallS);
  const double endpoint_now_s = kLocalWallAfterRollbackS + offset;
  const double old_steady_projection_s = kEndpointAckStampS + kSteadyElapsedS;
  const double offset_projection_s = lingtu::nav::commands::endpointSourceTime(
      kLocalWallAfterRollbackS,
      offset);

  check(
      old_steady_projection_s - endpoint_now_s > kFutureToleranceS,
      "regression setup must make the steady anchor future-dated after rollback");
  check(
      std::abs(offset_projection_s - endpoint_now_s) < 1e-12,
      "clock offset projection must follow endpoint CLOCK_REALTIME rollback");
}

}  // namespace

int main() {
  try {
    check(
        lingtu_nav_client_abi_version() == LINGTU_NAV_CLIENT_ABI_VERSION,
        "navigation client ABI version mismatch");
    check(
        (lingtu_nav_client_capabilities() &
         LINGTU_NAV_CLIENT_CAP_NAVIGATION) != 0U,
        "navigation client capability missing");
    check(
        (lingtu_nav_client_capabilities() &
         LINGTU_NAV_CLIENT_CAP_INSPECTION) != 0U,
        "inspection client capability missing");
    check(
        (lingtu_nav_client_capabilities() &
         LINGTU_NAV_CLIENT_CAP_EXPLORATION) != 0U,
        "exploration client capability missing");
    check(
        (lingtu_nav_client_capabilities() &
         LINGTU_NAV_CLIENT_CAP_DIRECTED_EXPLORATION) != 0U,
        "directed exploration client capability missing");
    check(
        (lingtu_nav_client_capabilities() &
         LINGTU_NAV_CLIENT_CAP_GOAL_STATUS) != 0U,
        "goal status client capability missing");
    check(
        (lingtu_nav_client_capabilities() &
         LINGTU_NAV_CLIENT_CAP_PATH_TELEMETRY) != 0U,
        "path telemetry client capability missing");
    check(
        (lingtu_nav_client_capabilities() &
         LINGTU_NAV_CLIENT_CAP_MAP_SCENE) != 0U,
        "map scene client capability missing");
    check(
        (lingtu_nav_client_capabilities() &
         LINGTU_NAV_CLIENT_CAP_OPERATOR_MOTION_RECEIPT) != 0U,
        "operator motion receipt client capability missing");
    check(
        (lingtu_nav_client_capabilities() &
         LINGTU_NAV_CLIENT_CAP_NAVIGATION_COMMAND_RECEIPT) != 0U,
        "navigation command receipt client capability missing");
    check(
        (lingtu_nav_client_capabilities() &
         LINGTU_NAV_CLIENT_CAP_NAVIGATION_TASK_STATUS) != 0U,
        "navigation task status client capability missing");
    check(
        (lingtu_nav_client_capabilities() &
         LINGTU_NAV_CLIENT_CAP_EXPLORATION_RUN_EVENTS) != 0U,
        "exploration run event client capability missing");
    runTest();
    testNavigationGoalStatusReaderAndRetention();
    testNavigationPathTelemetry();
    testMapSceneTelemetryAndCapacityGate();
    testExplorationCommands();
    testExplorationRunEventReader();
    testInspectionTaskCommands();
    testOperatorMotionAckCorrelationUsesFullSourceIdentity();
    testOperatorMotionReceiptApis();
    testEstopBypassesGoalAckWait();
    testSourceStampIsRefreshedAfterDiscovery();
    testFirstClockSensitiveCommandUsesEndpointClockAnchor();
    testDelayedClockAckIsResampledBeforeClockSensitiveCommand();
    testConcurrentClientsKeepClockAcksIsolated();
    testClockOffsetTracksRealtimeRollback();
    std::puts("test_nav_client: PASS");
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "test_nav_client: FAIL: %s\n", exc.what());
    return 1;
  }
}
