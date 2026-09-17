#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "lingtu/maps/json.hpp"
#include "lingtu/maps/mapd/service_dispatch.hpp"
#include "lingtu/maps/mapd/query_protocol.hpp"
#include "lingtu/maps/service.hpp"

namespace {

std::filesystem::path TempRoot() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root = std::filesystem::temp_directory_path() /
                    ("lingtu_mapd_dispatch_test_" + std::to_string(stamp));
  std::filesystem::create_directories(root);
  return root;
}

}  // namespace

int main() {
  const auto root = TempRoot();
  lingtu::maps::MapsServiceCore service(
      lingtu::maps::MapsServiceConfig{lingtu::maps::MapStoreConfig{root}});
  using lingtu::maps::mapd::query::DispatchServiceJson;

  auto listed = DispatchServiceJson(service, R"({"action":"list_maps"})");
  assert(listed.ok);
  assert(lingtu::maps::JsonObjectBoolAtPath(listed.json, {"success"}) == true);

  auto created = DispatchServiceJson(service, R"({"action":"create_map","map_id":"dispatch_map"})");
  assert(created.ok);
  assert(created.json.find("\"map_dir\"") == std::string::npos);
  auto relisted = DispatchServiceJson(service, R"({"action":"list_maps"})");
  assert(relisted.ok);
  assert(relisted.json.find("\"map_dir\"") == std::string::npos);
  auto record = DispatchServiceJson(service, R"({"action":"get_record","map_id":"dispatch_map"})");
  assert(record.ok);
  assert(record.json.find("dispatch_map") != std::string::npos);
  assert(record.json.find("\"map_dir\"") == std::string::npos);
  auto deleted = DispatchServiceJson(service, R"({"action":"delete_map","map_id":"dispatch_map"})");
  assert(deleted.ok);

  auto save_without_coordinator = DispatchServiceJson(
      service, R"({"action":"save_map","request_id":"dispatch-save","map_id":"saved_map"})");
  assert(!save_without_coordinator.ok);
  assert(lingtu::maps::JsonObjectStringAtPath(save_without_coordinator.json, {"reason_code"}) ==
         "internal_error");
  auto legacy_begin = DispatchServiceJson(
      service, R"({"action":"begin_save_map","request_id":"legacy","map_id":"saved_map"})");
  assert(!legacy_begin.ok);
  assert(lingtu::maps::JsonObjectStringAtPath(legacy_begin.json, {"reason_code"}) ==
         "unknown_action");

  auto natural_u64 = DispatchServiceJson(service, R"({"action":"get_map_points","max_points":1})");
  assert(lingtu::maps::JsonObjectStringAtPath(natural_u64.json, {"reason_code"}) !=
         "invalid_request");
  auto compatible_string_u64 =
      DispatchServiceJson(service, R"({"action":"get_map_points","max_points":"1"})");
  assert(lingtu::maps::JsonObjectStringAtPath(compatible_string_u64.json, {"reason_code"}) !=
         "invalid_request");
  for (const std::string invalid : {
           R"({"action":"get_map_points","max_points":-1})",
           R"({"action":"get_map_points","max_points":1.5})",
           R"({"action":"get_map_points","max_points":18446744073709551616})",
       }) {
    const auto result = DispatchServiceJson(service, invalid);
    assert(!result.ok);
    assert(lingtu::maps::JsonObjectStringAtPath(result.json, {"reason_code"}) == "invalid_request");
  }
  for (const std::string action : {"set_active_map", "clear_active_map"}) {
    const auto result =
        DispatchServiceJson(service, "{\"action\":\"" + action + "\",\"map_id\":\"dispatch_map\"}");
    assert(!result.ok);
    assert(lingtu::maps::JsonObjectStringAtPath(result.json, {"reason_code"}) == "unknown_action");
  }
  auto missing = DispatchServiceJson(service, R"({"action":"create_map"})");
  assert(!missing.ok);
  assert(lingtu::maps::JsonObjectStringAtPath(missing.json, {"reason_code"}) == "invalid_request");
  auto unknown = DispatchServiceJson(service, R"({"action":"not_an_action"})");
  assert(!unknown.ok);
  assert(lingtu::maps::JsonObjectStringAtPath(unknown.json, {"reason_code"}) == "unknown_action");

  // Match the saved-map preview size requested by the Web client.
  const auto source = root / "preview.pcd";
  {
    std::ofstream points(source);
    points << "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
              "WIDTH 80000\nHEIGHT 1\nPOINTS 80000\nDATA ascii\n";
    for (int i = 0; i < 80000; ++i) {
      points << "12.34567 -9.876543 0.456789\n";
    }
  }
  const auto imported = service.ImportPcdJson("preview", source, 0.0, {});
  assert(lingtu::maps::JsonObjectBoolAtPath(imported, {"success"}) == true);
  const auto preview = DispatchServiceJson(
      service, R"({"action":"get_map_points","map_id":"preview","max_points":80000})");
  assert(preview.ok);
  assert(preview.json.find("\"returned\":80000") != std::string::npos);
  std::cout << "80k preview bytes: " << preview.json.size() << std::endl;
  const bool preview_fits = preview.json.size() <= lingtu::maps::mapd::query::kDefaultMaxJsonBytes;
  std::filesystem::remove_all(root);
  return preview_fits ? 0 : 1;
}
