#include "runtime/semantic/view_query.hpp"
#include "dds/runtime.hpp"
#include "dds/dds.h"
#include "messages.h"
#include "message/generated/topics.hpp"
#include "transport/dds/qos.hpp"

#include <octomap/OcTree.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
using lingtu::nav::endpoint::SemanticViewContext;
using lingtu::nav::endpoint::SemanticViewQuery;

void expect(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

lingtu::nav::semantic::ViewResult finish(SemanticViewQuery& worker) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    if (auto result = worker.poll()) return *result;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  throw std::runtime_error("semantic query did not finish");
}

struct Fixture {
  std::filesystem::path root = std::filesystem::temp_directory_path() /
      ("lingtu-semantic-query-" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::path map_path = root / "semantic-map" / "octomap.ot";
  Fixture() {
    std::filesystem::create_directories(map_path.parent_path());
    octomap::OcTree tree(0.2);
    for (int x = -15; x <= 15; ++x)
      for (int y = -15; y <= 15; ++y) {
        const float px = static_cast<float>((x + 0.5) * 0.2);
        const float py = static_cast<float>((y + 0.5) * 0.2);
        // Missing support separates two rooms even though their body space is free.
        if (x != 3 && x != 4) tree.updateNode(octomap::point3d(px, py, -0.1F), true);
        for (const float z : {0.1F, 0.3F, 0.5F})
          tree.updateNode(octomap::point3d(px, py, z), false);
      }
    expect(tree.write(map_path.string()), "could not write OctoMap fixture");
  }
  ~Fixture() { std::error_code error; std::filesystem::remove_all(root, error); }
};

void ddsRoundTrip(SemanticViewQuery& worker, SemanticViewContext context) {
  constexpr int domain = 148;
  lingtu::nav::endpoint::Dds runtime(domain);
  const auto participant = dds_create_participant(domain, nullptr, nullptr);
  expect(participant > 0, "DDS peer creation failed");
  const auto create_endpoint = [&](const auto& contract, const auto* descriptor, bool writer) {
    const auto topic = dds_create_topic(participant, descriptor, contract.dds_topic.data(), nullptr, nullptr);
    expect(topic > 0, "DDS topic creation failed");
    auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(contract.dds_topic));
    const auto endpoint = writer ? dds_create_writer(participant, topic, qos.get(), nullptr)
                                : dds_create_reader(participant, topic, qos.get(), nullptr);
    expect(endpoint > 0, "DDS endpoint creation failed");
    return endpoint;
  };
  const auto writer = create_endpoint(lingtu::message::kNavSemanticViewRequest,
                                      &lingtu_dds_SemanticViewRequest_desc, true);
  const auto reader = create_endpoint(lingtu::message::kNavSemanticViewResult,
                                      &lingtu_dds_SemanticViewResult_desc, false);
  lingtu_dds_SemanticViewRequest message{};
  message.header.frame_id = const_cast<char*>("map");
  message.request_id = const_cast<char*>("dds-query");
  message.boot_id = const_cast<char*>(context.boot_id.c_str());
  message.map_id = const_cast<char*>(context.map.map_id.c_str());
  message.map_content_epoch = context.map.content_epoch;
  message.frame_epoch = context.frame_epoch;
  message.reference_z = 0.1;
  message.camera_range_m = 1.5;
  message.camera_horizontal_fov_rad = 0.8;
  lingtu_dds_SemanticCameraView view{};
  view.x = context.robot.x;
  view.y = context.robot.y;
  view.yaw = 0.5;
  view.range_m = 1.0;
  view.horizontal_fov_rad = 0.7;
  message.views._buffer = &view;
  message.views._length = message.views._maximum = 1;
  bool received = false;
  for (int attempt = 0; attempt < 200 && !received; ++attempt) {
    expect(dds_write(writer, &message) >= 0, "DDS query write failed");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    for (const auto& command : runtime.takeCommands(0.0).ordered) {
      const auto* query = std::get_if<lingtu::nav::semantic::ViewQuery>(&command);
      if (!query) continue;
      context.query = *query;
      received = true;
      break;
    }
  }
  expect(received && context.query.camera_range_m == 1.5 &&
      context.query.camera_horizontal_fov_rad == 0.8 && context.query.views.size() == 1 &&
      context.query.views.front().yaw == 0.5, "DDS query lost camera calibration or history");
  expect(worker.start(context), "DDS query worker start failed");
  const auto result = finish(worker);
  expect(result.available && !result.candidates.empty(), "DDS query failed: " + result.reason);
  bool matched = false;
  for (int attempt = 0; attempt < 200 && !matched; ++attempt) {
    expect(runtime.publish(lingtu::nav::endpoint::OutputEvent{result}).published, "DDS result write failed");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    void* samples[1]{};
    dds_sample_info_t infos[1]{};
    const auto count = dds_take(reader, samples, infos, 1, 1);
    expect(count >= 0, "DDS result read failed");
    if (count == 1 && infos[0].valid_data) {
      const auto& reply = *static_cast<lingtu_dds_SemanticViewResult*>(samples[0]);
      matched = std::string(reply.request_id) == "dds-query" && reply.available &&
          std::string(reply.boot_id) == context.boot_id && reply.frame_epoch == context.frame_epoch &&
          reply.candidates._length == result.candidates.size() &&
          reply.candidates._buffer[0].visible_cells == result.candidates.front().visible_cells &&
          reply.candidates._buffer[0].position.z == result.reference_z;
    }
    if (count > 0) dds_return_loan(reader, samples, count);
  }
  dds_delete(participant);
  expect(matched, "DDS result lost identity, height or visibility evidence");
}
}  // namespace

int main() {
  try {
    Fixture fixture;
    const lingtu::nav::plan::MapIdentity identity{"semantic-map", 1, "map"};
    auto gate = std::make_shared<lingtu::nav::endpoint::ActiveOctomapGate>(identity);
    lingtu::nav::plan::GlobalPlannerOptions options;
    options.robot_radius = 0.1;
    options.ground_support_depth_cells = 2;
    SemanticViewQuery worker(gate, fixture.map_path.string(), options);
    SemanticViewContext context;
    context.map = identity;
    context.boot_id = "native-test";
    context.frame_epoch = 7;
    context.timestamp_s = 123.0;
    context.robot = {0.1, 0.1, 0.11};
    context.query.request_id = "query-test";
    context.query.boot_id = context.boot_id;
    context.query.map = identity;
    context.query.camera_range_m = 2.0;
    context.query.camera_horizontal_fov_rad = 1.0;
    auto run = [&](const SemanticViewContext& request) {
      expect(worker.start(request), "worker rejected idle query");
      expect(!worker.start(request), "worker allowed overlapping queries");
      return finish(worker);
    };
    const auto ready = run(context);
    expect(ready.available && !ready.candidates.empty(), "query failed: " + ready.reason);
    expect(ready.request_id == "query-test" && ready.boot_id == context.boot_id &&
        ready.frame_epoch == 7 && lingtu::nav::plan::sameMapIdentity(ready.map, identity),
        "query lost its native/map identity");
    expect(std::abs(ready.reference_z - 0.1) < 1e-6, "candidate height was not snapped to tested layer");
    for (const auto& candidate : ready.candidates) {
      expect(candidate.position.x < 0.6, "candidate crossed a missing ground-support stripe");
      expect(candidate.position.z == ready.reference_z, "candidate used an untested height");
      expect(candidate.visible_cells > 0, "candidate reported zero visible geometry");
    }
    auto narrow_camera = context;
    narrow_camera.query.camera_range_m = 0.21;
    narrow_camera.query.camera_horizontal_fov_rad = 0.1;
    const auto narrow = run(narrow_camera);
    expect(narrow.available && !narrow.candidates.empty(), "narrow camera query failed");
    for (const auto& candidate : narrow.candidates)
      expect(candidate.visible_cells <= 2, "candidate gain ignored the camera range/FOV");
    auto stale = context;
    stale.query.frame_epoch = 6;
    expect(run(stale).reason == "semantic_search_context_mismatch", "stale localization was accepted");
    stale = context;
    stale.query.boot_id = "previous-process";
    expect(!run(stale).available, "previous process history was accepted");
    stale = context;
    ++stale.query.map.content_epoch;
    expect(!run(stale).available, "previous map history was accepted");
    auto observed = context;
    observed.query.views.push_back({0.1, 0.1, 0.0, 2.0, 1.0});
    expect(!run(observed).available, "history without a localization epoch was accepted");
    observed.query.frame_epoch = context.frame_epoch;
    observed.query.reference_z = ready.reference_z + 0.2;
    expect(run(observed).reason == "semantic_search_height_layer_changed", "history from another floor was reused");
    observed.query.reference_z = ready.reference_z;
    expect(run(observed).available, "matching history was rejected");
    auto invalid = context;
    invalid.query.camera_horizontal_fov_rad = 0.0;
    expect(!run(invalid).available, "missing camera model silently used a 360-degree LiDAR");
    expect(worker.start(context), "cancel query did not start");
    worker.cancel();
    const auto cancelled = finish(worker);
    expect(!cancelled.available && !cancelled.geometry_exhausted && cancelled.candidates.empty(),
        "cancelled query returned candidates");
    expect(run(context).available, "cancellation poisoned the next query");
    ddsRoundTrip(worker, context);
    std::cout << "semantic OctoMap query, support, height, camera, identity and cancellation passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
