#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "lingtu/maps/json.hpp"
#include "lingtu/maps/mapd/save_coordinator.hpp"
#include "lingtu/maps/mapd/service_dispatch.hpp"
#include "lingtu/maps/service.hpp"

namespace {

using lingtu::maps::JsonObjectBoolAtPath;
using lingtu::maps::JsonObjectStringAtPath;
using lingtu::maps::MapsServiceConfig;
using lingtu::maps::MapsServiceCore;
using lingtu::maps::MapStoreConfig;
using lingtu::maps::mapd::SaveCoordinator;
using lingtu::maps::mapd::SaveCoordinatorConfig;
using lingtu::maps::mapd::SlamSnapshotAck;
using lingtu::maps::mapd::SlamSnapshotExchange;
using lingtu::maps::mapd::SlamSnapshotRequest;

std::filesystem::path TempRoot() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root = std::filesystem::temp_directory_path() /
                    ("lingtu_mapd_save_coordinator_test_" + std::to_string(stamp));
  std::filesystem::create_directories(root);
  return root;
}

class FakeExchange final : public SlamSnapshotExchange {
 public:
  bool Publish(const SlamSnapshotRequest &request) override {
    published.push_back(request);
    return publish_result;
  }

  std::vector<SlamSnapshotAck> TakeAcks() override { return std::exchange(acks, {}); }

  bool publish_result{true};
  std::vector<SlamSnapshotRequest> published;
  std::vector<SlamSnapshotAck> acks;
};

void WriteSnapshotPcd(const std::filesystem::path &path) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  output << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
            "COUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n1 2 3\n";
}

void TestSaveMapRequestsSlamAndConsumesReceipt() {
  const auto root = TempRoot();
  MapsServiceCore service(MapsServiceConfig{MapStoreConfig{root}});
  FakeExchange exchange;
  SaveCoordinatorConfig config;
  config.product = "map";
  config.product_session_id = "product-session-7";
  config.save_patches = true;
  config.request_defaults.require.occupancy = false;
  config.request_defaults.require.octomap = false;
  config.request_defaults.require.esdf = false;
  config.request_defaults.require.traversability = false;
  SaveCoordinator coordinator(service, exchange, config);

  const std::string submitted = coordinator.SaveMapJson("save-7", "office");
  assert(JsonObjectBoolAtPath(submitted, {"accepted"}) == true);
  assert(JsonObjectStringAtPath(submitted, {"action"}) == "save_map");
  assert(JsonObjectStringAtPath(submitted, {"status", "job_id"}) == "save-7");
  assert(exchange.published.size() == 1U);
  const SlamSnapshotRequest &request = exchange.published.front();
  assert(request.request_id.find("save-7-capture-") == 0U);
  assert(request.map_id == "office");
  assert(request.product_session_id == "product-session-7");
  assert(request.save_patches);
  assert(request.output_path.filename() == "map.pcd");

  WriteSnapshotPcd(request.output_path);
  SlamSnapshotAck ack;
  ack.request_id = request.request_id;
  ack.map_id = request.map_id;
  ack.success = true;
  ack.output_path = request.output_path;
  ack.runtime_instance_id = "fastlio2:boot:1";
  ack.product_session_id = request.product_session_id;
  ack.reset_epoch = 3U;
  ack.observation_sequence = 11U;
  ack.captured_at_ns = 123456789U;
  ack.frame_id = "map";
  ack.point_count = 1U;
  ack.state = "tracking";
  ack.healthy = false;
  ack.health_message = "tracking quality below save threshold";
  exchange.acks.push_back(std::move(ack));

  coordinator.Poll();
  const std::string status = service.GetSaveMapStatusJson("save-7");
  assert(JsonObjectBoolAtPath(status, {"success"}) == true);
  const auto state = JsonObjectStringAtPath(status, {"status", "state"});
  assert(state.has_value());
  assert(*state == "FAILED");
  assert(JsonObjectStringAtPath(status, {"status", "reason_code"}) == "slam_unhealthy");

  const std::string mismatch_submitted = coordinator.SaveMapJson("save-8", "warehouse");
  assert(JsonObjectBoolAtPath(mismatch_submitted, {"accepted"}) == true);
  assert(exchange.published.size() == 2U);
  SlamSnapshotAck mismatch;
  mismatch.request_id = exchange.published.back().request_id;
  mismatch.map_id = "another-map";
  mismatch.success = true;
  mismatch.output_path = exchange.published.back().output_path;
  mismatch.product_session_id = config.product_session_id;
  exchange.acks.push_back(std::move(mismatch));
  coordinator.Poll();
  const std::string mismatch_status = service.GetSaveMapStatusJson("save-8");
  assert(JsonObjectStringAtPath(mismatch_status, {"status", "state"}) == "FAILED");
  assert(JsonObjectStringAtPath(mismatch_status, {"status", "reason_code"}) ==
         "snapshot_ack_identity_mismatch");

  const std::string rejected_submitted = coordinator.SaveMapJson("save-9", "yard");
  assert(JsonObjectBoolAtPath(rejected_submitted, {"accepted"}) == true);
  SlamSnapshotAck rejected;
  const std::string rejected_capture_id = exchange.published.back().request_id;
  rejected.request_id = rejected_capture_id;
  rejected.map_id = "yard";
  rejected.success = false;
  rejected.message = "SLAM is not tracking";
  exchange.acks.push_back(std::move(rejected));
  coordinator.Poll();
  const std::string rejected_status = service.GetSaveMapStatusJson("save-9");
  assert(JsonObjectStringAtPath(rejected_status, {"status", "state"}) == "FAILED");
  assert(JsonObjectStringAtPath(rejected_status, {"status", "reason_code"}) ==
         "slam_snapshot_failed");
  assert(JsonObjectStringAtPath(rejected_status, {"status", "message"}) == "SLAM is not tracking");

  const std::string retried = coordinator.RetrySaveMapJson("save-9");
  assert(JsonObjectBoolAtPath(retried, {"accepted"}) == true);
  assert(JsonObjectStringAtPath(retried, {"status", "state"}) == "WAITING_SNAPSHOT");
  assert(exchange.published.size() == 4U);
  const std::string retry_capture_id = exchange.published.back().request_id;
  assert(retry_capture_id != rejected_capture_id);

  SlamSnapshotAck stale;
  stale.request_id = rejected_capture_id;
  stale.map_id = "yard";
  stale.success = true;
  stale.output_path = exchange.published.back().output_path;
  stale.product_session_id = config.product_session_id;
  exchange.acks.push_back(std::move(stale));
  coordinator.Poll();
  const std::string waiting = service.GetSaveMapStatusJson("save-9");
  assert(JsonObjectStringAtPath(waiting, {"status", "state"}) == "WAITING_SNAPSHOT");

  SlamSnapshotAck retry_rejected;
  retry_rejected.request_id = retry_capture_id;
  retry_rejected.map_id = "yard";
  retry_rejected.success = false;
  retry_rejected.message = "SLAM still not tracking";
  exchange.acks.push_back(std::move(retry_rejected));
  coordinator.Poll();
  const std::string retry_status = service.GetSaveMapStatusJson("save-9");
  assert(JsonObjectStringAtPath(retry_status, {"status", "state"}) == "FAILED");
  assert(JsonObjectStringAtPath(retry_status, {"status", "message"}) == "SLAM still not tracking");

  assert(JsonObjectBoolAtPath(coordinator.SaveMapJson("save-10", "garage"), {"accepted"}) == true);
  const std::string cancelled_capture_id = exchange.published.back().request_id;
  assert(JsonObjectBoolAtPath(coordinator.CancelSaveMapJson("save-10"), {"accepted"}) == true);
  assert(JsonObjectStringAtPath(service.GetSaveMapStatusJson("save-10"), {"status", "state"}) ==
         "CANCELLED");
  assert(JsonObjectBoolAtPath(coordinator.RetrySaveMapJson("save-10"), {"accepted"}) == true);
  const std::string cancel_retry_capture_id = exchange.published.back().request_id;
  assert(cancel_retry_capture_id != cancelled_capture_id);

  SlamSnapshotAck cancelled_stale;
  cancelled_stale.request_id = cancelled_capture_id;
  cancelled_stale.map_id = "garage";
  cancelled_stale.success = true;
  cancelled_stale.output_path = exchange.published.back().output_path;
  cancelled_stale.product_session_id = config.product_session_id;
  exchange.acks.push_back(std::move(cancelled_stale));
  coordinator.Poll();
  assert(JsonObjectStringAtPath(service.GetSaveMapStatusJson("save-10"), {"status", "state"}) ==
         "WAITING_SNAPSHOT");

  SlamSnapshotAck cancel_retry_rejected;
  cancel_retry_rejected.request_id = cancel_retry_capture_id;
  cancel_retry_rejected.map_id = "garage";
  cancel_retry_rejected.success = false;
  cancel_retry_rejected.message = "capture rejected after retry";
  exchange.acks.push_back(std::move(cancel_retry_rejected));
  coordinator.Poll();
  assert(JsonObjectStringAtPath(service.GetSaveMapStatusJson("save-10"), {"status", "state"}) ==
         "FAILED");

  std::filesystem::remove_all(root);
}

void TestSaveMapRequiresMapProductSession() {
  const auto root = TempRoot();
  MapsServiceCore service(MapsServiceConfig{MapStoreConfig{root}});
  FakeExchange exchange;
  SaveCoordinatorConfig config;
  config.product = "nav";
  config.product_session_id = "nav-session";
  SaveCoordinator nav(service, exchange, config);
  const std::string wrong_product = nav.SaveMapJson("save-nav", "office");
  assert(JsonObjectBoolAtPath(wrong_product, {"accepted"}) == false);
  assert(JsonObjectStringAtPath(wrong_product, {"reason_code"}) == "map_product_required");
  assert(JsonObjectStringAtPath(nav.RetrySaveMapJson("job"), {"reason_code"}) ==
         "map_product_required");
  assert(JsonObjectStringAtPath(nav.CancelSaveMapJson("job"), {"reason_code"}) ==
         "map_product_required");

  config.product = "map";
  config.product_session_id.clear();
  SaveCoordinator no_session(service, exchange, config);
  const std::string missing_session = no_session.SaveMapJson("save-no-session", "office");
  assert(JsonObjectBoolAtPath(missing_session, {"accepted"}) == false);
  assert(JsonObjectStringAtPath(missing_session, {"reason_code"}) == "map_product_required");
  assert(exchange.published.empty());
  std::filesystem::remove_all(root);
}

void TestLegacyTwoStepActionsAreNotPublic() {
  const auto root = TempRoot();
  MapsServiceCore service(MapsServiceConfig{MapStoreConfig{root}});
  FakeExchange exchange;
  SaveCoordinatorConfig config;
  config.product = "map";
  config.product_session_id = "product-session-legacy";
  SaveCoordinator coordinator(service, exchange, config);

  const auto begin = lingtu::maps::mapd::query::DispatchServiceJson(
      service, &coordinator,
      R"({"action":"begin_save_map","request_id":"legacy","map_id":"legacy"})");
  assert(!begin.ok);
  assert(JsonObjectStringAtPath(begin.json, {"reason_code"}) == "unknown_action");
  const auto provide = lingtu::maps::mapd::query::DispatchServiceJson(
      service, &coordinator,
      R"({"action":"provide_save_map_snapshot","job_id":"legacy","source_dir":"."})");
  assert(!provide.ok);
  assert(JsonObjectStringAtPath(provide.json, {"reason_code"}) == "unknown_action");

  std::filesystem::remove_all(root);
}

}  // namespace

int main() {
  TestSaveMapRequestsSlamAndConsumesReceipt();
  TestSaveMapRequiresMapProductSession();
  TestLegacyTwoStepActionsAreNotPublic();
  return 0;
}
