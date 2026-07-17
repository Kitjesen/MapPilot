#include "lingtu/maps/c_api/store.h"
#include "lingtu/maps/c_api/service.h"

#include <cassert>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

std::filesystem::path TempRoot() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  auto root = std::filesystem::temp_directory_path() /
      ("lingtu_maps_store_c_api_test_" + std::to_string(stamp));
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  return root;
}

void Touch(const std::filesystem::path& path) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary);
  file << "x";
}

void WriteValidOccupancyMetadata(const std::filesystem::path& path) {
  constexpr const char* kXHash =
      "2d711642b726b04401627ca9fbac32f5c8530fb1903cc4db02258717921a4881";
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << "{\"frame_id\":\"map\",\"artifacts\":{"
       << "\"map_pcd\":{\"sha256\":\"" << kXHash << "\"},"
       << "\"occupancy_grid\":{\"sha256\":\"" << kXHash
       << "\",\"source_map_sha256\":\"" << kXHash << "\"}}}";
}

std::string ReadString(int32_t rc, const std::vector<char>& buf) {
  assert(rc == 0);
  return std::string(buf.data());
}

std::string ReadStoreJson(
    LingtuMapsStoreHandle* store,
    int32_t (*fn)(LingtuMapsStoreHandle*, char*, uint64_t, uint64_t*)) {
  uint64_t needed = 0;
  fn(store, nullptr, 0, &needed);
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  return ReadString(fn(store, buf.data(), buf.size(), &needed), buf);
}

std::string ReadStoreRecordJson(LingtuMapsStoreHandle* store, const char* map_id) {
  uint64_t needed = 0;
  lingtu_maps_store_record_json(store, map_id, nullptr, 0, &needed);
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  return ReadString(
      lingtu_maps_store_record_json(store, map_id, buf.data(), buf.size(), &needed),
      buf);
}

std::string ReadStoreBundleJson(
    LingtuMapsStoreHandle* store,
    const char* map_id,
    const char* capability) {
  uint64_t needed = 0;
  assert(lingtu_maps_store_bundle_json(
             store, map_id, capability, nullptr, 0, &needed) == 1);
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  return ReadString(
      lingtu_maps_store_bundle_json(
          store, map_id, capability, buf.data(), buf.size(), &needed),
      buf);
}

std::string ReadStoreValidationJson(
    LingtuMapsStoreHandle* store,
    const char* map_id,
    bool require_octomap,
    bool require_occupancy,
    const char* expected_frame_id) {
  uint64_t needed = 0;
  assert(lingtu_maps_store_validate_artifacts_json(
             store,
             map_id,
             require_octomap ? 1U : 0U,
             require_occupancy ? 1U : 0U,
             expected_frame_id,
             nullptr,
             0,
             &needed) == 1);
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  return ReadString(
      lingtu_maps_store_validate_artifacts_json(
          store,
          map_id,
          require_octomap ? 1U : 0U,
          require_occupancy ? 1U : 0U,
          expected_frame_id,
          buf.data(),
          buf.size(),
          &needed),
      buf);
}

}  // namespace

int main() {
  const auto root = TempRoot();
  LingtuMapsServiceHandle* service =
      lingtu_maps_service_create(root.string().c_str(), nullptr);
  assert(service != nullptr);
  LingtuMapsStoreHandle* store = lingtu_maps_store_create(root.string().c_str(), nullptr);
  assert(store != nullptr);
  assert(lingtu_maps_store_validate_map_id("map_1") == 1);
  assert(lingtu_maps_store_validate_map_id("../bad") == 0);

  assert(lingtu_maps_store_create_map(store, "map_1") == 0);
  assert(lingtu_maps_store_set_active_map(store, "map_1", 1) == 2);
  Touch(root / "map_1" / "map.pcd");
  Touch(root / "map_1" / "occupancy.npz");
  WriteValidOccupancyMetadata(root / "map_1" / "metadata.json");
  const auto validation =
      ReadStoreValidationJson(store, "map_1", false, true, "map");
  assert(validation.find("\"success\":true") != std::string::npos);
  assert(validation.find("\"map_id\":\"map_1\"") != std::string::npos);
  assert(validation.find("\"ok\":true") != std::string::npos);
  assert(validation.find("\"checked_frame_id\":\"map\"") != std::string::npos);
  assert(lingtu_maps_store_set_active_map(store, "map_1", 1) == 0);

  std::vector<char> buf(128, '\0');
  uint64_t needed = 0;
  assert(ReadString(
      lingtu_maps_store_active_map_id(store, buf.data(), buf.size(), &needed),
      buf) == "map_1");

  std::fill(buf.begin(), buf.end(), '\0');
  assert(ReadString(
      lingtu_maps_store_list_map_ids(store, buf.data(), buf.size(), &needed),
      buf) == "map_1");

  uint64_t artifact_count = 0;
  assert(lingtu_maps_store_artifact_count(store, "map_1", &artifact_count) == 0);
  assert(artifact_count == 2);

  const std::string record_json = ReadStoreRecordJson(store, "map_1");
  assert(record_json.find("\"map_id\":\"map_1\"") != std::string::npos);
  assert(record_json.find("\"hash\":\"2d711642b726b04401627ca9fbac32f5c8530fb1903cc4db02258717921a4881\"") != std::string::npos);
  assert(record_json.find("\"active_allowed\":true") != std::string::npos);
  assert(record_json.find("\"localization_stability\":null") != std::string::npos);
  assert(record_json.find("\"planning_success_rate\":null") != std::string::npos);
  assert(record_json.find("\"collision_rate\":null") != std::string::npos);
  assert(record_json.find("\"overall_score\":null") != std::string::npos);
  assert(record_json.find("\"status\":\"unknown\"") != std::string::npos);
  assert(record_json.find("\"reason_code\":\"not_enough_data\"") != std::string::npos);
  const std::string bundle_json =
      ReadStoreBundleJson(store, "map_1", "path_planning_2d");
  assert(bundle_json.find("\"schema_version\":\"map.bundle\"") != std::string::npos);
  assert(bundle_json.find("\"success\":true") != std::string::npos);
  assert(bundle_json.find("\"type\":\"OCCUPANCY_2D\"") != std::string::npos);
  assert(bundle_json.find("\"map_dir\":") != std::string::npos);
  const std::string missing_bundle =
      ReadStoreBundleJson(store, "map_1", "trajectory_optimization");
  assert(missing_bundle.find("\"reason_code\":\"missing_capability\"") != std::string::npos);

  assert(ReadStoreJson(store, lingtu_maps_store_list_records_json)
             .find("\"schema_version\":\"map.record.native\"") != std::string::npos);

  assert(lingtu_maps_store_rename_map(store, "map_1", "map_2") == 0);
  std::fill(buf.begin(), buf.end(), '\0');
  assert(ReadString(
      lingtu_maps_store_active_map_id(store, buf.data(), buf.size(), &needed),
      buf) == "map_2");
  needed = 0;
  assert(lingtu_maps_store_active_record_json(store, nullptr, 0, &needed) == 1);
  std::vector<char> active_json_buf(static_cast<size_t>(needed), '\0');
  assert(ReadString(
      lingtu_maps_store_active_record_json(
          store, active_json_buf.data(), active_json_buf.size(), &needed),
      active_json_buf).find("\"map_id\":\"map_2\"") != std::string::npos);
  assert(!std::filesystem::exists(root / "active"));

  assert(lingtu_maps_store_clear_active_map(store) == 0);
  std::fill(buf.begin(), buf.end(), '\0');
  assert(ReadString(
      lingtu_maps_store_active_map_id(store, buf.data(), buf.size(), &needed),
      buf).empty());
  assert(lingtu_maps_store_set_active_map(store, "map_2", 0) == 0);

  assert(lingtu_maps_store_delete_map(store, "map_2") == 0);
  std::fill(buf.begin(), buf.end(), '\0');
  assert(ReadString(
      lingtu_maps_store_active_map_id(store, buf.data(), buf.size(), &needed),
      buf).empty());

  lingtu_maps_store_destroy(store);
  lingtu_maps_service_destroy(service);
  std::filesystem::remove_all(root);
  return 0;
}
