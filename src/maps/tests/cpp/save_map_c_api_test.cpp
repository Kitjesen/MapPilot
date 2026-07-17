#include "lingtu/maps/c_api/service.h"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

[[noreturn]] void Fail(const std::string& message) {
  std::cerr << message << "\n";
  std::exit(1);
}

void Require(bool condition, const std::string& message) {
  if (!condition) Fail(message);
}

std::string ReadCommand(
    const std::function<int32_t(char*, uint64_t, uint64_t*)>& call) {
  uint64_t needed = 0;
  Require(call(nullptr, 0, &needed) == 1, "command probe failed");
  std::vector<char> buffer(static_cast<std::size_t>(needed), '\0');
  Require(call(buffer.data(), buffer.size(), &needed) == 0, "command read failed");
  return buffer.data();
}

std::string ReadQuery(
    const std::function<int32_t(char*, uint64_t, uint64_t*)>& call) {
  uint64_t needed = 0;
  Require(call(nullptr, 0, &needed) == 1, "query probe failed");
  for (int attempt = 0; attempt < 4; ++attempt) {
    std::vector<char> buffer(static_cast<std::size_t>(needed), '\0');
    const int32_t result = call(buffer.data(), buffer.size(), &needed);
    if (result == 0) {
      return buffer.data();
    }
    Require(result == 1, "query read failed");
  }
  Fail("query output kept changing during read");
}

std::filesystem::path TempRoot() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  auto root = std::filesystem::temp_directory_path() /
      ("lingtu_maps_save_map_c_api_test_" + std::to_string(stamp));
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  return root;
}

void WritePcd(const std::filesystem::path& path) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary);
  file << "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
       << "WIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA ascii\n0 0 0\n1 0 0\n0 1 0\n";
}

std::string Converter(const std::filesystem::path& root) {
#if defined(_WIN32)
  const auto script = root / "converter.cmd";
  std::ofstream file(script, std::ios::binary);
  file << "@echo off\r\necho OCTOMAP > \"%~2\"\r\n";
  return "\"" + script.string() + "\" {input} {output}";
#else
  const auto script = root / "converter.sh";
  std::ofstream file(script, std::ios::binary);
  file << "#!/bin/sh\nprintf 'OCTOMAP\\n' > \"$2\"\n";
  file.close();
  std::filesystem::permissions(
      script,
      std::filesystem::perms::owner_exec |
          std::filesystem::perms::owner_read |
          std::filesystem::perms::owner_write,
      std::filesystem::perm_options::add);
  return "'" + script.string() + "' {input} {output}";
#endif
}

template <typename T>
void Init(T* value) {
  value->struct_size = sizeof(T);
  value->abi_version = LINGTU_MAPS_SAVE_MAP_ABI_VERSION;
}

}  // namespace

int main() {
  const auto root = TempRoot();
  const auto maps = root / "maps";
  const auto source = root / "source";
  WritePcd(source / "map.pcd");
  const auto converter = Converter(root);

  auto* service = lingtu_maps_service_create(maps.string().c_str(), "active_map.txt");
  Require(service != nullptr, "failed to create maps service");

  LingtuMapsSaveMapRequest request{};
  Init(&request);
  Init(&request.require);
  Init(&request.source);
  Init(&request.octomap);
  request.request_id = "c_api_save";
  request.map_id = "warehouse";
  request.require.occupancy = 1U;
  request.require.octomap = 1U;
  request.require.esdf = 1U;
  request.require.traversability = 1U;
  request.source.dynamic_filter_enabled = 0U;
  request.source.optimizer_strategy = "none";
  request.octomap.converter_command = converter.c_str();
  request.octomap.build_mode = "external_pcl_converter";
  request.octomap.resolution = 0.2;
  request.octomap.support_dilation_cells = 1;
  request.octomap.free_layers_above = 3;
  request.octomap.free_dilation_cells = 1;
  request.octomap.frame_id = "map";
  request.octomap.source_profile = "test";
  request.octomap.data_source = "test";
  request.octomap.slam_source = "test";
  request.octomap.localization_source = "test";
  request.octomap.mapping_source = "test";
  request.octomap.timeout_sec = 10.0;
  request.require_slam_healthy = 1U;
  request.minimum_point_count = 1U;

  const auto begin = ReadCommand([&](char* out, uint64_t capacity, uint64_t* needed) {
    return lingtu_maps_service_save_map_begin_json(
        service, &request, out, capacity, needed);
  });
  Require(begin.find("\"accepted\":true") != std::string::npos, "SaveMap begin rejected");
  const auto listed = ReadQuery([&](char* out, uint64_t capacity, uint64_t* needed) {
    return lingtu_maps_service_list_save_map_jobs_json(
        service, 100U, out, capacity, needed);
  });
  Require(listed.find("c_api_save") != std::string::npos,
          "SaveMap job list does not contain submitted job");

  LingtuMapsMapSnapshot snapshot{};
  Init(&snapshot);
  snapshot.snapshot_id = "c_api_snapshot";
  const std::string source_string = source.string();
  snapshot.source_dir = source_string.c_str();
  snapshot.frame_id = "map";
  snapshot.slam_healthy = 1U;
  const auto provided = ReadCommand([&](char* out, uint64_t capacity, uint64_t* needed) {
    return lingtu_maps_service_save_map_provide_snapshot_json(
        service, "c_api_save", &snapshot, out, capacity, needed);
  });
  Require(provided.find("\"accepted\":true") != std::string::npos, "snapshot rejected");

  std::string status;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  do {
    status = ReadQuery([&](char* out, uint64_t capacity, uint64_t* needed) {
      return lingtu_maps_service_save_map_status_json(
          service, "c_api_save", out, capacity, needed);
    });
    if (status.find("\"state\":\"SUCCEEDED\"") != std::string::npos) break;
    if (status.find("\"state\":\"FAILED\"") != std::string::npos) Fail(status);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  } while (std::chrono::steady_clock::now() < deadline);
  Require(status.find("\"state\":\"SUCCEEDED\"") != std::string::npos, "C ABI SaveMap timed out");
  Require(std::filesystem::is_regular_file(maps / "warehouse" / "current_version.txt"),
          "C ABI SaveMap did not publish version pointer");

  const auto versions = ReadQuery([&](char* out, uint64_t capacity, uint64_t* needed) {
    return lingtu_maps_service_list_map_versions_json(
        service, "warehouse", out, capacity, needed);
  });
  Require(versions.find("\"version\":1,\"current\":true") != std::string::npos,
          "C ABI version query did not expose the committed version");
  const auto rollback = ReadCommand([&](char* out, uint64_t capacity, uint64_t* needed) {
    return lingtu_maps_service_rollback_map_version_json(
        service, "warehouse", 1, out, capacity, needed);
  });
  Require(rollback.find("\"success\":true") != std::string::npos,
          "C ABI rollback round trip failed");

  lingtu_maps_service_destroy(service);
  std::filesystem::remove_all(root);
  return 0;
}
