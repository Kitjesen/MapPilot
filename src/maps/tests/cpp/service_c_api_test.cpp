#include "lingtu/maps/c_api/service.h"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#if defined(_WIN32)
#  define NOMINMAX
#  include <windows.h>
#else
#  include <unistd.h>
#endif

namespace {

[[noreturn]] void Fail(const std::string& message) {
  std::cerr << message << "\n";
  std::exit(1);
}

void Require(bool condition, const std::string& message) {
  if (!condition) {
    Fail(message);
  }
}

void RequireContains(const std::string& value, const std::string& needle) {
  Require(value.find(needle) != std::string::npos, "missing substring: " + needle + "\n" + value);
}

std::string JsonStringValue(const std::string& value, const std::string& key) {
  const std::string marker = "\"" + key + "\":\"";
  const auto start = value.find(marker);
  Require(start != std::string::npos, "missing json string key: " + key + "\n" + value);
  const auto offset = start + marker.size();
  const auto end = value.find('"', offset);
  Require(end != std::string::npos, "unterminated json string key: " + key + "\n" + value);
  return value.substr(offset, end - offset);
}

std::filesystem::path TempRoot() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  auto root = std::filesystem::temp_directory_path() /
      ("lingtu_maps_service_c_api_test_" + std::to_string(stamp));
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  return root;
}

bool HasTransactionDirectory(const std::filesystem::path& map_dir) {
  const auto builds = map_dir / ".builds";
  if (!std::filesystem::is_directory(builds)) {
    return false;
  }
  for (const auto& entry : std::filesystem::directory_iterator(builds)) {
    if (!entry.is_directory()) {
      continue;
    }
    const auto name = entry.path().filename().string();
    if (name.find("_transaction") != std::string::npos) {
      return true;
    }
  }
  return false;
}

void Touch(const std::filesystem::path& path) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary);
  file << "x";
}

void SetEnvVar(const char* name, const char* value) {
#if defined(_WIN32)
  _putenv_s(name, value);
#else
  setenv(name, value, 1);
#endif
}

void UnsetEnvVar(const char* name) {
#if defined(_WIN32)
  _putenv_s(name, "");
#else
  unsetenv(name);
#endif
}

std::uint64_t CurrentProcessIdValue() {
#if defined(_WIN32)
  return static_cast<std::uint64_t>(GetCurrentProcessId());
#else
  return static_cast<std::uint64_t>(getpid());
#endif
}

std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  std::ostringstream text;
  text << file.rdbuf();
  return text.str();
}

void WriteTextFile(const std::filesystem::path& path, const std::string& value) {
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << value;
}

void WriteAsciiPcd(const std::filesystem::path& path) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary);
  file
      << "VERSION 0.7\n"
      << "FIELDS x y z\n"
      << "SIZE 4 4 4\n"
      << "TYPE F F F\n"
      << "COUNT 1 1 1\n"
      << "WIDTH 3\n"
      << "HEIGHT 1\n"
      << "POINTS 3\n"
      << "DATA ascii\n"
      << "0.0 0.0 0.0\n"
      << "0.1 0.0 0.0\n"
      << "2.0 0.0 0.0\n";
}

std::string QuoteCommandPath(const std::filesystem::path& path) {
#if defined(_WIN32)
  return "\"" + path.string() + "\"";
#else
  std::string out = "'";
  for (const char ch : path.string()) {
    if (ch == '\'') {
      out += "'\\''";
    } else {
      out.push_back(ch);
    }
  }
  out += "'";
  return out;
#endif
}

std::string WriteFakeOctomapConverter(const std::filesystem::path& root) {
#if defined(_WIN32)
  const auto script = root / "fake_octomap_converter.cmd";
  std::ofstream file(script, std::ios::binary);
  file
      << "@echo off\r\n"
      << "echo FAKE_BT > \"%~2\"\r\n"
      << "type \"%~1\" >> \"%~2\"\r\n";
#else
  const auto script = root / "fake_octomap_converter.sh";
  std::ofstream file(script, std::ios::binary);
  file
      << "#!/bin/sh\n"
      << "printf 'FAKE_BT\\n' > \"$2\"\n"
      << "cat \"$1\" >> \"$2\"\n";
  file.close();
  std::filesystem::permissions(
      script,
      std::filesystem::perms::owner_exec |
          std::filesystem::perms::owner_read |
          std::filesystem::perms::owner_write,
      std::filesystem::perm_options::add);
#endif
  return QuoteCommandPath(script) + " {input} {output}";
}

std::string WriteSlowOctomapConverter(const std::filesystem::path& root) {
#if defined(_WIN32)
  const auto script = root / "slow_octomap_converter.cmd";
  std::ofstream file(script, std::ios::binary);
  file
      << "@echo off\r\n"
      << "echo starting slow converter\r\n"
      << "ping -n 4 127.0.0.1 > nul\r\n"
      << "echo should_not_finish > \"%~2\"\r\n";
#else
  const auto script = root / "slow_octomap_converter.sh";
  std::ofstream file(script, std::ios::binary);
  file
      << "#!/bin/sh\n"
      << "echo starting slow converter\n"
      << "sleep 5\n"
      << "printf 'should_not_finish\\n' > \"$2\"\n";
  file.close();
  std::filesystem::permissions(
      script,
      std::filesystem::perms::owner_exec |
          std::filesystem::perms::owner_read |
          std::filesystem::perms::owner_write,
      std::filesystem::perm_options::add);
#endif
  return QuoteCommandPath(script) + " {input} {output}";
}

std::string WriteFakeOctomapEditor(const std::filesystem::path& root) {
#if defined(_WIN32)
  const auto script = root / "fake_octomap_editor.cmd";
  std::ofstream file(script, std::ios::binary);
  file
      << "@echo off\r\n"
      << "type \"%~1\" > \"%~2\"\r\n"
      << "echo edited>> \"%~2\"\r\n"
      << "echo {\"ok\":true,\"edited_voxels\":7,\"effective_state\":\"occupied\"}\r\n";
#else
  const auto script = root / "fake_octomap_editor.sh";
  std::ofstream file(script, std::ios::binary);
  file
      << "#!/bin/sh\n"
      << "cat \"$1\" > \"$2\"\n"
      << "printf 'edited\\n' >> \"$2\"\n"
      << "printf '{\"ok\":true,\"edited_voxels\":7,\"effective_state\":\"occupied\"}\\n'\n";
  file.close();
  std::filesystem::permissions(
      script,
      std::filesystem::perms::owner_exec |
          std::filesystem::perms::owner_read |
          std::filesystem::perms::owner_write,
      std::filesystem::perm_options::add);
#endif
  return QuoteCommandPath(script) + " {map} {output}";
}

std::string WriteFakeSourceOptimizer(const std::filesystem::path& root) {
#if defined(_WIN32)
  const auto script = root / "fake_source_optimizer.cmd";
  std::ofstream file(script, std::ios::binary);
  file
      << "@echo off\r\n"
      << "echo VERSION 0.7 > \"%~1\\map.pcd\"\r\n"
      << "echo FIELDS x y z >> \"%~1\\map.pcd\"\r\n"
      << "echo SIZE 4 4 4 >> \"%~1\\map.pcd\"\r\n"
      << "echo TYPE F F F >> \"%~1\\map.pcd\"\r\n"
      << "echo COUNT 1 1 1 >> \"%~1\\map.pcd\"\r\n"
      << "echo WIDTH 2 >> \"%~1\\map.pcd\"\r\n"
      << "echo HEIGHT 1 >> \"%~1\\map.pcd\"\r\n"
      << "echo POINTS 2 >> \"%~1\\map.pcd\"\r\n"
      << "echo DATA ascii >> \"%~1\\map.pcd\"\r\n"
      << "echo 0 0 0 >> \"%~1\\map.pcd\"\r\n"
      << "echo 1 0 0 >> \"%~1\\map.pcd\"\r\n"
      << "echo {\"ok\":true,\"patch_count\":1}\r\n";
#else
  const auto script = root / "fake_source_optimizer.sh";
  std::ofstream file(script, std::ios::binary);
  file
      << "#!/bin/sh\n"
      << "cat > \"$1/map.pcd\" <<'PCD'\n"
      << "VERSION 0.7\n"
      << "FIELDS x y z\n"
      << "SIZE 4 4 4\n"
      << "TYPE F F F\n"
      << "COUNT 1 1 1\n"
      << "WIDTH 2\n"
      << "HEIGHT 1\n"
      << "POINTS 2\n"
      << "DATA ascii\n"
      << "0 0 0\n"
      << "1 0 0\n"
      << "PCD\n"
      << "printf '{\"ok\":true,\"patch_count\":1}\\n'\n";
  file.close();
  std::filesystem::permissions(
      script,
      std::filesystem::perms::owner_exec |
          std::filesystem::perms::owner_read |
          std::filesystem::perms::owner_write,
      std::filesystem::perm_options::add);
#endif
#if defined(_WIN32)
  return "call " + QuoteCommandPath(script) + " {map}";
#else
  return QuoteCommandPath(script) + " {map}";
#endif
}

std::string ReadQueryJson(
    LingtuMapsServiceHandle* service,
    int32_t (*fn)(LingtuMapsServiceHandle*, char*, uint64_t, uint64_t*)) {
  uint64_t needed = 0;
  Require(fn(service, nullptr, 0, &needed) == 1, "query probe failed");
  Require(needed > 1U, "query probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed + 4096U), '\0');
  Require(fn(service, buf.data(), buf.size(), &needed) == 0, "query read failed");
  return std::string(buf.data());
}

std::string ReadQueryJsonWithId(
    LingtuMapsServiceHandle* service,
    const char* map_id,
    int32_t (*fn)(LingtuMapsServiceHandle*, const char*, char*, uint64_t, uint64_t*)) {
  uint64_t needed = 0;
  Require(fn(service, map_id, nullptr, 0, &needed) == 1, "id query probe failed");
  Require(needed > 1U, "id query probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed + 4096U), '\0');
  Require(fn(service, map_id, buf.data(), buf.size(), &needed) == 0, "id query read failed");
  return std::string(buf.data());
}

std::string ReadCommandJsonWithId(
    LingtuMapsServiceHandle* service,
    const char* map_id,
    int32_t (*fn)(LingtuMapsServiceHandle*, const char*, char*, uint64_t, uint64_t*)) {
  uint64_t needed = 0;
  Require(fn(service, map_id, nullptr, 0, &needed) == 1, "id command probe failed");
  Require(needed > 1U, "id command probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed + 4096U), '\0');
  Require(fn(service, map_id, buf.data(), buf.size(), &needed) == 0, "id command read failed");
  return std::string(buf.data());
}

std::string ReadExportVersionJson(
    LingtuMapsServiceHandle* service,
    const char* map_id,
    std::int64_t version,
    const std::filesystem::path& package_dir) {
  uint64_t needed = 0U;
  auto call = [&](char* out, uint64_t capacity) {
    return lingtu_maps_service_export_version_json(
        service, map_id, version, package_dir.string().c_str(), 1U, out, capacity, &needed);
  };
  Require(call(nullptr, 0U) == 1, "export_version probe failed");
  std::vector<char> buf(static_cast<std::size_t>(needed + 4096U), '\0');
  int32_t rc = call(buf.data(), buf.size());
  if (rc == 1) {
    buf.assign(static_cast<std::size_t>(needed), '\0');
    rc = call(buf.data(), buf.size());
  }
  Require(rc == 0, "export_version read failed");
  return std::string(buf.data());
}

std::string ReadImportPackageJson(
    LingtuMapsServiceHandle* service,
    const std::filesystem::path& package_dir) {
  uint64_t needed = 0U;
  auto call = [&](char* out, uint64_t capacity) {
    return lingtu_maps_service_import_package_json(
        service, package_dir.string().c_str(), "imported", 1U, out, capacity, &needed);
  };
  Require(call(nullptr, 0U) == 1, "import_package probe failed");
  std::vector<char> buf(static_cast<std::size_t>(needed + 4096U), '\0');
  int32_t rc = call(buf.data(), buf.size());
  if (rc == 1) {
    buf.assign(static_cast<std::size_t>(needed), '\0');
    rc = call(buf.data(), buf.size());
  }
  Require(rc == 0, "import_package read failed");
  return std::string(buf.data());
}

std::string ReadSetActiveJson(LingtuMapsServiceHandle* service, const char* map_id, bool strict) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_set_active_map_json(
          service, map_id, strict ? 1U : 0U, nullptr, 0, &needed) == 1,
      "set_active command probe failed");
  Require(needed > 1U, "set_active command probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_set_active_map_json(
          service, map_id, strict ? 1U : 0U, buf.data(), buf.size(), &needed) == 0,
      "set_active command read failed");
  return std::string(buf.data());
}

std::string ReadRollbackActiveJson(LingtuMapsServiceHandle* service) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_rollback_active_map_json(service, nullptr, 0, &needed) == 1,
      "rollback_active command probe failed");
  std::vector<char> buf(static_cast<size_t>(needed + 4096U), '\0');
  int32_t rc = lingtu_maps_service_rollback_active_map_json(
      service, buf.data(), buf.size(), &needed);
  if (rc == 1) {
    buf.assign(static_cast<size_t>(needed), '\0');
    rc = lingtu_maps_service_rollback_active_map_json(
        service, buf.data(), buf.size(), &needed);
  }
  Require(rc == 0, "rollback_active command read failed");
  return std::string(buf.data());
}

std::string ReadListPoiJson(LingtuMapsServiceHandle* service, const char* map_id) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_list_poi_json(service, map_id, nullptr, 0, &needed) == 1,
      "list_poi query probe failed");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_list_poi_json(service, map_id, buf.data(), buf.size(), &needed) == 0,
      "list_poi query read failed");
  return std::string(buf.data());
}

std::string ReadSetPoiJson(
    LingtuMapsServiceHandle* service,
    const char* map_id,
    const char* name,
    double x,
    double y) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_set_poi_json(
          service, map_id, name, x, y, 0.0, 0.0, 0U, "map", "{\"kind\":\"test\"}", nullptr, 0, &needed) == 1,
      "set_poi command probe failed");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_set_poi_json(
          service, map_id, name, x, y, 0.0, 0.0, 0U, "map", "{\"kind\":\"test\"}", buf.data(), buf.size(), &needed) == 0,
      "set_poi command read failed");
  return std::string(buf.data());
}

std::string ReadDeletePoiJson(
    LingtuMapsServiceHandle* service,
    const char* map_id,
    const char* name) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_delete_poi_json(service, map_id, name, nullptr, 0, &needed) == 1,
      "delete_poi command probe failed");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_delete_poi_json(service, map_id, name, buf.data(), buf.size(), &needed) == 0,
      "delete_poi command read failed");
  return std::string(buf.data());
}

std::string ReadListGraphJson(LingtuMapsServiceHandle* service) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_list_map_graph_json(service, nullptr, 0, &needed) == 1,
      "list_map_graph query probe failed");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_list_map_graph_json(service, buf.data(), buf.size(), &needed) == 0,
      "list_map_graph query read failed");
  return std::string(buf.data());
}

std::string ReadSetMapEdgeJson(
    LingtuMapsServiceHandle* service,
    const char* from,
    const char* to) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_set_map_edge_json(
          service, from, to, "door", 1U, nullptr, 0, &needed) == 1,
      "set_map_edge command probe failed");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_set_map_edge_json(
          service, from, to, "door", 1U, buf.data(), buf.size(), &needed) == 0,
      "set_map_edge command read failed");
  return std::string(buf.data());
}

std::string ReadShortestRouteJson(
    LingtuMapsServiceHandle* service,
    const char* from,
    const char* to) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_shortest_route_json(service, from, to, nullptr, 0, &needed) == 1,
      "shortest_route query probe failed");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_shortest_route_json(service, from, to, buf.data(), buf.size(), &needed) == 0,
      "shortest_route query read failed");
  return std::string(buf.data());
}

std::string ReadSetActiveSlotJson(
    LingtuMapsServiceHandle* service,
    const char* slot,
    const char* map_id,
    bool strict) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_set_active_slot_json(
          service, slot, map_id, strict ? 1U : 0U, nullptr, 0, &needed) == 1,
      "set_active_slot command probe failed");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_set_active_slot_json(
          service, slot, map_id, strict ? 1U : 0U, buf.data(), buf.size(), &needed) == 0,
      "set_active_slot command read failed");
  return std::string(buf.data());
}

std::string ReadClearActiveSlotJson(LingtuMapsServiceHandle* service, const char* slot) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_clear_active_slot_json(service, slot, nullptr, 0, &needed) == 1,
      "clear_active_slot command probe failed");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_clear_active_slot_json(service, slot, buf.data(), buf.size(), &needed) == 0,
      "clear_active_slot command read failed");
  return std::string(buf.data());
}

std::string ReadEnqueueBuildJson(
    LingtuMapsServiceHandle* service,
    const char* map_id,
    const char* artifact_type) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_enqueue_build_json(
          service, map_id, artifact_type, nullptr, 0, &needed) == 1,
      "enqueue_build command probe failed");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_enqueue_build_json(
          service, map_id, artifact_type, buf.data(), buf.size(), &needed) == 0,
      "enqueue_build command read failed");
  return std::string(buf.data());
}

std::string ReadBeginBuildJson(
    LingtuMapsServiceHandle* service,
    const char* map_id,
    const char* artifact_type) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_begin_build_json(
          service, map_id, artifact_type, nullptr, 0, &needed) == 1,
      "begin_build command probe failed");
  Require(needed > 1U, "begin_build command probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_begin_build_json(
          service, map_id, artifact_type, buf.data(), buf.size(), &needed) == 0,
      "begin_build command read failed");
  return std::string(buf.data());
}

std::string ReadFinishBuildJson(
    LingtuMapsServiceHandle* service,
    const char* map_id,
    const char* build_id,
    bool success,
    const char* message) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_finish_build_json(
          service,
          map_id,
          build_id,
          success ? 1U : 0U,
          message,
          nullptr,
          0,
          &needed) == 1,
      "finish_build command probe failed");
  Require(needed > 1U, "finish_build command probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_finish_build_json(
          service,
          map_id,
          build_id,
          success ? 1U : 0U,
          message,
          buf.data(),
          buf.size(),
          &needed) == 0,
      "finish_build command read failed");
  return std::string(buf.data());
}

std::string ReadBuildStatusJson(LingtuMapsServiceHandle* service, const char* map_id) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_get_build_status_json(service, map_id, nullptr, 0, &needed) == 1,
      "build_status query probe failed");
  Require(needed > 1U, "build_status query probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_get_build_status_json(
          service, map_id, buf.data(), buf.size(), &needed) == 0,
      "build_status query read failed");
  return std::string(buf.data());
}

std::string ReadBuildOccupancyJson(LingtuMapsServiceHandle* service, const char* map_id) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_build_occupancy_snapshot_json(service, map_id, nullptr, 0, &needed) == 1,
      "build_occupancy command probe failed");
  Require(needed > 1U, "build_occupancy command probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_build_occupancy_snapshot_json(
          service, map_id, buf.data(), buf.size(), &needed) == 0,
      "build_occupancy command read failed");
  return std::string(buf.data());
}

std::string ReadBuildOctomapJson(
    LingtuMapsServiceHandle* service,
    const char* map_id,
    const std::string& converter_command,
    double timeout_sec = 60.0) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_build_octomap_artifact_json(
          service,
          map_id,
          converter_command.c_str(),
          "external_pcl_converter",
          0.2,
          1,
          3,
          1,
          "map",
          "test_profile",
          "test_data",
          "test_slam",
          "test_slam",
          "test_native_build_octomap",
          timeout_sec,
          nullptr,
          0,
          &needed) == 1,
      "build_octomap command probe failed");
  Require(needed > 1U, "build_octomap command probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_build_octomap_artifact_json(
          service,
          map_id,
          converter_command.c_str(),
          "external_pcl_converter",
          0.2,
          1,
          3,
          1,
          "map",
          "test_profile",
          "test_data",
          "test_slam",
          "test_slam",
          "test_native_build_octomap",
          timeout_sec,
          buf.data(),
          buf.size(),
          &needed) == 0,
      "build_octomap command read failed");
  return std::string(buf.data());
}

std::string ReadVoxelEditsJson(LingtuMapsServiceHandle* service, const char* map_id) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_get_voxel_edits_json(service, map_id, nullptr, 0, &needed) == 1,
      "get_voxel_edits query probe failed");
  Require(needed > 1U, "get_voxel_edits query probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_get_voxel_edits_json(
          service, map_id, buf.data(), buf.size(), &needed) == 0,
      "get_voxel_edits query read failed");
  return std::string(buf.data());
}

std::string ReadEditOctomapJson(
    LingtuMapsServiceHandle* service,
    const char* map_id,
    const std::string& editor_command,
    double x_m = 1.0) {
  uint64_t needed = 0;
  auto call = [&](char* out, uint64_t capacity) {
    return lingtu_maps_service_edit_octomap_voxels_json(
        service,
        map_id,
        editor_command.c_str(),
        "preblocked",
        "sphere",
        x_m,
        2.0,
        0.5,
        0.3,
        15.0,
        out,
        capacity,
        &needed);
  };
  Require(call(nullptr, 0) == 1, "edit_octomap command probe failed");
  Require(needed > 1U, "edit_octomap command probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(call(buf.data(), buf.size()) == 0, "edit_octomap command read failed");
  return std::string(buf.data());
}

std::string ReadBuildNavigationPackageJson(
    LingtuMapsServiceHandle* service,
    const char* map_id,
    const std::string& converter_command,
    double timeout_sec = 60.0) {
  uint64_t needed = 0;
  auto call = [&](char* out, uint64_t capacity) {
    return lingtu_maps_service_build_navigation_package_json(
          service,
          map_id,
          converter_command.c_str(),
          "external_pcl_converter",
          0.2,
          1,
          3,
          1,
          "map",
          "test_profile",
          "test_data",
          "test_slam",
          "test_slam",
          "test_native_build_navigation_package",
          timeout_sec,
          1U,
          1U,
          out,
          capacity,
          &needed);
  };
  Require(
      call(nullptr, 0) == 1,
      "build_navigation_package command probe failed");
  Require(needed > 1U, "build_navigation_package command probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  int32_t rc = call(buf.data(), buf.size());
  if (rc == 1) {
    Require(needed > buf.size(), "build_navigation_package retry did not grow capacity");
    buf.assign(static_cast<size_t>(needed), '\0');
    rc = call(buf.data(), buf.size());
  }
  Require(rc == 0, "build_navigation_package command read failed");
  return std::string(buf.data());
}

std::string ReadBuildEsdfJson(LingtuMapsServiceHandle* service, const char* map_id) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_build_esdf_artifact_json(service, map_id, nullptr, 0, &needed) == 1,
      "build_esdf command probe failed");
  Require(needed > 1U, "build_esdf command probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_build_esdf_artifact_json(
          service, map_id, buf.data(), buf.size(), &needed) == 0,
      "build_esdf command read failed");
  return std::string(buf.data());
}

std::string ReadBuildTraversabilityJson(LingtuMapsServiceHandle* service, const char* map_id) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_build_traversability_artifact_json(
          service, map_id, nullptr, 0, &needed) == 1,
      "build_traversability command probe failed");
  Require(needed > 1U, "build_traversability command probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_build_traversability_artifact_json(
          service, map_id, buf.data(), buf.size(), &needed) == 0,
      "build_traversability command read failed");
  return std::string(buf.data());
}

std::string ReadImportPcdJson(
    LingtuMapsServiceHandle* service,
    const char* map_id,
    const char* source_path,
    double voxel_size) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_import_pcd_json(
          service,
          map_id,
          source_path,
          voxel_size,
          0U,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          nullptr,
          0,
          &needed) == 1,
      "import_pcd command probe failed");
  Require(needed > 1U, "import_pcd command probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_import_pcd_json(
          service,
          map_id,
          source_path,
          voxel_size,
          0U,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          buf.data(),
          buf.size(),
          &needed) == 0,
      "import_pcd command read failed");
  return std::string(buf.data());
}

std::string ReadCommitSavedSourceJson(
    LingtuMapsServiceHandle* service,
    const char* map_id,
    const char* source_dir,
    const std::string& optimizer_command) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_commit_saved_source_json(
          service,
          map_id,
          source_dir,
          0.0,
          0U,
          0U,
          "",
          300.0,
          "pgo",
          0U,
          optimizer_command.c_str(),
          120.0,
          nullptr,
          0,
          &needed) == 1,
      "commit_saved_source command probe failed");
  Require(needed > 1U, "commit_saved_source command probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_commit_saved_source_json(
          service,
          map_id,
          source_dir,
          0.0,
          0U,
          0U,
          "",
          300.0,
          "pgo",
          0U,
          optimizer_command.c_str(),
          120.0,
          buf.data(),
          buf.size(),
          &needed) == 0,
      "commit_saved_source command read failed");
  return std::string(buf.data());
}

std::string ReadCropPcdJson(
    LingtuMapsServiceHandle* service,
    const char* map_id,
    double min_x,
    double max_x) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_crop_pcd_json(
          service,
          map_id,
          0U,
          0.0,
          1U,
          min_x,
          -0.5,
          -0.5,
          max_x,
          0.5,
          0.5,
          nullptr,
          0,
          &needed) == 1,
      "crop_pcd command probe failed");
  Require(needed > 1U, "crop_pcd command probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_crop_pcd_json(
          service,
          map_id,
          0U,
          0.0,
          1U,
          min_x,
          -0.5,
          -0.5,
          max_x,
          0.5,
          0.5,
          buf.data(),
          buf.size(),
          &needed) == 0,
      "crop_pcd command read failed");
  return std::string(buf.data());
}

std::string ReadRenameJson(
    LingtuMapsServiceHandle* service,
    const char* map_id,
    const char* new_map_id) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_rename_map_json(service, map_id, new_map_id, nullptr, 0, &needed) == 1,
      "rename command probe failed");
  Require(needed > 1U, "rename command probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_rename_map_json(
          service, map_id, new_map_id, buf.data(), buf.size(), &needed) == 0,
      "rename command read failed");
  return std::string(buf.data());
}

std::string ReadBundleJson(
    LingtuMapsServiceHandle* service,
    const char* map_id,
    const char* capability) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_get_bundle_json(service, map_id, capability, nullptr, 0, &needed) == 1,
      "bundle query probe failed");
  Require(needed > 1U, "bundle query probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_get_bundle_json(
          service, map_id, capability, buf.data(), buf.size(), &needed) == 0,
      "bundle query read failed");
  return std::string(buf.data());
}

std::string ReadValidateArtifactsJson(
    LingtuMapsServiceHandle* service,
    const char* map_id,
    bool require_octomap,
    bool require_occupancy,
    const char* expected_frame_id = "map") {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_validate_artifacts_json(
          service,
          map_id,
          require_octomap ? 1U : 0U,
          require_occupancy ? 1U : 0U,
          expected_frame_id,
          nullptr,
          0,
          &needed) == 1,
      "validate_artifacts query probe failed");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_validate_artifacts_json(
          service,
          map_id,
          require_octomap ? 1U : 0U,
          require_occupancy ? 1U : 0U,
          expected_frame_id,
          buf.data(),
          buf.size(),
          &needed) == 0,
      "validate_artifacts query read failed");
  return std::string(buf.data());
}

std::string ReadMapPointsJson(
    LingtuMapsServiceHandle* service,
    const char* map_id,
    uint64_t max_points) {
  uint64_t needed = 0;
  Require(
      lingtu_maps_service_get_map_points_json(
          service, map_id, max_points, nullptr, 0, &needed) == 1,
      "map_points query probe failed");
  Require(needed > 1U, "map_points query probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(
      lingtu_maps_service_get_map_points_json(
          service, map_id, max_points, buf.data(), buf.size(), &needed) == 0,
      "map_points query read failed");
  return std::string(buf.data());
}

std::string ReadClearActiveJson(LingtuMapsServiceHandle* service) {
  uint64_t needed = 0;
  Require(lingtu_maps_service_clear_active_map_json(service, nullptr, 0, &needed) == 1,
          "clear_active command probe failed");
  Require(needed > 1U, "clear_active command probe did not return a usable size");
  std::vector<char> buf(static_cast<size_t>(needed), '\0');
  Require(lingtu_maps_service_clear_active_map_json(service, buf.data(), buf.size(), &needed) == 0,
          "clear_active command read failed");
  return std::string(buf.data());
}

}  // namespace

int main() {
  const auto root = TempRoot();
  auto* service = lingtu_maps_service_create(root.string().c_str(), nullptr);
  Require(service != nullptr, "service create failed");

  const auto created =
      ReadCommandJsonWithId(service, "map_1", lingtu_maps_service_create_map_json);
  RequireContains(created, "\"action\":\"create\"");
  RequireContains(created, "\"success\":true");

  const auto outside_exchange = root.parent_path() / "outside_maps_exchange";
  std::filesystem::create_directories(outside_exchange);
  const auto rejected_export = ReadExportVersionJson(service, "map_1", 1, outside_exchange);
  RequireContains(rejected_export, "\"reason_code\":\"unsafe_exchange_path\"");
  const auto rejected_package = ReadImportPackageJson(service, outside_exchange);
  RequireContains(rejected_package, "\"reason_code\":\"unsafe_exchange_path\"");

  const auto missing_active = ReadQueryJson(service, lingtu_maps_service_get_active_json);
  RequireContains(missing_active, "\"success\":false");

  const auto map_types = ReadQueryJson(service, lingtu_maps_service_get_map_types_json);
  RequireContains(map_types, "\"schema_version\":\"map.types\"");
  RequireContains(map_types, "\"record_schema_version\":\"map.record\"");
  RequireContains(map_types, "\"path_planning_2d\":\"OCCUPANCY_2D\"");
  RequireContains(map_types, "\"builders\":{");
  RequireContains(map_types, "\"algorithm_embedded\":false");
  RequireContains(map_types, "\"supported_build_modes\":[\"external_pcl_converter\"]");

  const auto failed_active = ReadSetActiveJson(service, "map_1", true);
  RequireContains(failed_active, "\"success\":false");

  const auto begin_build = ReadBeginBuildJson(service, "map_1", "OCCUPANCY_2D");
  RequireContains(begin_build, "\"action\":\"begin_build\"");
  RequireContains(begin_build, "\"success\":true");
  RequireContains(begin_build, "\"status\":\"RUNNING\"");
  const std::string build_id = JsonStringValue(begin_build, "build_id");

  const auto blocked_build = ReadBeginBuildJson(service, "map_1", "OCTOMAP_3D");
  RequireContains(blocked_build, "\"success\":false");
  RequireContains(blocked_build, "\"reason_code\":\"build_in_progress\"");

  const auto running_status = ReadBuildStatusJson(service, "map_1");
  RequireContains(running_status, "\"schema_version\":\"map.build.status\"");
  RequireContains(running_status, "\"running\":true");

  const auto finished_build =
      ReadFinishBuildJson(service, "map_1", build_id.c_str(), true, "ok");
  RequireContains(finished_build, "\"success\":true");
  RequireContains(finished_build, "\"status\":\"SUCCEEDED\"");

  const auto large_begin = ReadBeginBuildJson(service, "map_1", "LARGE_RESPONSE");
  const std::string large_build_id = JsonStringValue(large_begin, "build_id");
  const std::string large_message(12'000U, 'x');
  uint64_t large_needed = 0U;
  Require(
      lingtu_maps_service_finish_build_json(
          service, "map_1", large_build_id.c_str(), 1U, large_message.c_str(), nullptr, 0,
          &large_needed) == 1,
      "large finish_build probe failed");
  std::vector<char> small_command_buffer(4096U, '\0');
  Require(
      lingtu_maps_service_finish_build_json(
          service, "map_1", large_build_id.c_str(), 1U, large_message.c_str(),
          small_command_buffer.data(), small_command_buffer.size(), &large_needed) == 1,
      "large finish_build did not report truncation");
  Require(large_needed > small_command_buffer.size(),
          "large finish_build did not return exact required size");
  std::vector<char> large_command_buffer(static_cast<std::size_t>(large_needed), '\0');
  Require(
      lingtu_maps_service_finish_build_json(
          service, "map_1", large_build_id.c_str(), 1U, large_message.c_str(),
          large_command_buffer.data(), large_command_buffer.size(), &large_needed) == 0,
      "large finish_build cached response read failed");
  const std::string large_finish(large_command_buffer.data());
  RequireContains(large_finish, "\"status\":\"SUCCEEDED\"");
  RequireContains(large_finish, large_message.substr(0, 128U));

  const auto finished_status = ReadBuildStatusJson(service, "map_1");
  RequireContains(finished_status, "\"running\":false");
  RequireContains(finished_status, "\"status\":\"SUCCEEDED\"");

  const auto source_pcd = root / "source.pcd";
  WriteAsciiPcd(source_pcd);
  const auto imported = ReadImportPcdJson(service, "map_1", source_pcd.string().c_str(), 0.5);
  RequireContains(imported, "\"action\":\"import_pcd\"");
  RequireContains(imported, "\"success\":true");
  RequireContains(imported, "\"transactional_visibility\":\"staged_until_commit\"");
  RequireContains(imported, "\"point_count\":2");
  Require(std::filesystem::is_regular_file(root / "map_1" / "map.pcd"), "import did not write map.pcd");
  Require(
      std::filesystem::is_regular_file(root / "map_1" / "metadata.json"),
      "import did not publish invalidated metadata.json");
  RequireContains(ReadTextFile(root / "map_1" / "metadata.json"), "\"metadata_state\":\"invalidated\"");
  Require(
      !HasTransactionDirectory(root / "map_1"),
      "successful import left source transaction staging directory");

  const auto saved_source_created =
      ReadCommandJsonWithId(service, "saved_source_map", lingtu_maps_service_create_map_json);
  RequireContains(saved_source_created, "\"success\":true");
  const auto source_stage = root / "source_stage";
  WriteAsciiPcd(source_stage / "map.pcd");
  {
    std::ofstream poses(source_stage / "poses.txt", std::ios::binary);
    poses << "000001.pcd 0 0 0 1 0 0 0\n";
  }
  std::filesystem::create_directories(source_stage / "patches");
  WriteAsciiPcd(source_stage / "patches" / "000001.pcd");
  const auto committed_source = ReadCommitSavedSourceJson(
      service,
      "saved_source_map",
      source_stage.string().c_str(),
      WriteFakeSourceOptimizer(root));
  RequireContains(committed_source, "\"action\":\"commit_saved_source\"");
  RequireContains(committed_source, "\"success\":true");
  RequireContains(committed_source, "\"mode\":\"native_saved_source_transaction\"");
  RequireContains(committed_source, "\"map_optimization_ok\":true");
  RequireContains(committed_source, "\"map_optimization_performed\":true");
  RequireContains(committed_source, "\"performed\":true");
  RequireContains(committed_source, "\"patch_count\":1");
  Require(
      std::filesystem::is_regular_file(root / "saved_source_map" / "map.pcd"),
      "commit_saved_source did not publish map.pcd");
  Require(
      std::filesystem::is_regular_file(root / "saved_source_map" / "poses.txt"),
      "commit_saved_source did not publish poses.txt");
  Require(
      std::filesystem::is_directory(root / "saved_source_map" / "patches"),
      "commit_saved_source did not publish patches directory");
  Require(
      !HasTransactionDirectory(root / "saved_source_map"),
      "successful commit_saved_source left transaction staging directory");

  const auto map_points = ReadMapPointsJson(service, "map_1", 1U);
  RequireContains(map_points, "\"action\":\"get_map_points\"");
  RequireContains(map_points, "\"success\":true");
  RequireContains(map_points, "\"returned\":1");

  const auto cropped = ReadCropPcdJson(service, "map_1", -0.5, 0.5);
  RequireContains(cropped, "\"action\":\"crop\"");
  RequireContains(cropped, "\"success\":true");
  RequireContains(cropped, "\"transactional_visibility\":\"staged_until_commit\"");
  RequireContains(cropped, "\"point_count\":1");
  RequireContains(cropped, "\"removed_points\":1");
  RequireContains(ReadTextFile(root / "map_1" / "metadata.json"), "\"metadata_state\":\"invalidated\"");
  Require(
      !HasTransactionDirectory(root / "map_1"),
      "successful crop left source transaction staging directory");

  const auto built_occupancy = ReadBuildOccupancyJson(service, "map_1");
  RequireContains(built_occupancy, "\"action\":\"build_occupancy_snapshot\"");
  RequireContains(built_occupancy, "\"success\":true");
  RequireContains(built_occupancy, "\"mode\":\"projection_native\"");
  Require(std::filesystem::is_regular_file(root / "map_1" / "occupancy.npz"), "occupancy build did not write npz");
  Require(std::filesystem::is_regular_file(root / "map_1" / "map.pgm"), "occupancy build did not write pgm");
  Require(std::filesystem::is_regular_file(root / "map_1" / "map.yaml"), "occupancy build did not write yaml");

  const auto built_esdf = ReadBuildEsdfJson(service, "map_1");
  RequireContains(built_esdf, "\"action\":\"build_esdf_artifact\"");
  RequireContains(built_esdf, "\"success\":true");
  RequireContains(built_esdf, "\"mode\":\"native_grid_artifact\"");
  Require(std::filesystem::is_regular_file(root / "map_1" / "esdf.npz"), "esdf build did not write npz");

  const auto built_traversability = ReadBuildTraversabilityJson(service, "map_1");
  RequireContains(built_traversability, "\"action\":\"build_traversability_artifact\"");
  RequireContains(built_traversability, "\"success\":true");
  RequireContains(built_traversability, "\"mode\":\"native_grid_artifact\"");
  Require(
      std::filesystem::is_regular_file(root / "map_1" / "traversability.npz"),
      "traversability build did not write npz");

  const auto converter_command = WriteFakeOctomapConverter(root);
  const auto built_octomap = ReadBuildOctomapJson(service, "map_1", converter_command);
  RequireContains(built_octomap, "\"action\":\"build_octomap\"");
  RequireContains(built_octomap, "\"success\":true");
  RequireContains(built_octomap, "\"status\":\"built\"");
  RequireContains(built_octomap, "\"mode\":\"native_transaction\"");
  RequireContains(built_octomap, "\"transactional_visibility\":\"staged_until_commit\"");
  Require(
      std::filesystem::is_regular_file(root / "map_1" / "octomap.ot"),
      "octomap build did not write octomap.ot");
  Require(
      std::filesystem::is_regular_file(root / "map_1" / "metadata.json"),
      "octomap build did not write metadata.json");
  {
    std::ifstream metadata(root / "map_1" / "metadata.json", std::ios::binary);
    std::ostringstream metadata_text;
    metadata_text << metadata.rdbuf();
    RequireContains(metadata_text.str(), "\"schema_version\":\"lingtu.saved_map_artifacts.v1\"");
    RequireContains(metadata_text.str(), "\"source_map_sha256\"");
    RequireContains(metadata_text.str(), "\"octomap\"");
  }

  const auto editor_command = WriteFakeOctomapEditor(root);
  const auto edited_octomap = ReadEditOctomapJson(service, "map_1", editor_command);
  RequireContains(edited_octomap, "\"action\":\"edit_voxels\"");
  RequireContains(edited_octomap, "\"success\":true");
  RequireContains(edited_octomap, "\"transactional_visibility\":\"staged_until_commit\"");
  RequireContains(edited_octomap, "\"edited_voxels\":7");
  RequireContains(ReadTextFile(root / "map_1" / "metadata.json"), "\"manual_voxel_edit\":true");
  RequireContains(ReadTextFile(root / "map_1" / "metadata.json"), "\"voxel_edits.jsonl\"");
  RequireContains(ReadTextFile(root / "map_1" / "voxel_edits.jsonl"), "\"state\":\"preblocked\"");
  const auto queried_edits = ReadVoxelEditsJson(service, "map_1");
  RequireContains(queried_edits, "\"count\":1");
  RequireContains(queried_edits, "\"state\":\"preblocked\"");
  Require(!HasTransactionDirectory(root / "map_1"), "successful edit left transaction staging");

  const auto octomap_before_failed_edit = ReadTextFile(root / "map_1" / "octomap.ot");
  const auto metadata_before_failed_edit = ReadTextFile(root / "map_1" / "metadata.json");
  const auto journal_before_failed_edit = ReadTextFile(root / "map_1" / "voxel_edits.jsonl");
  SetEnvVar("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER", "1");
  const auto failed_edit = ReadEditOctomapJson(service, "map_1", editor_command);
  UnsetEnvVar("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER");
  RequireContains(failed_edit, "\"success\":false");
  RequireContains(failed_edit, "\"reason_code\":\"transaction_commit_failed\"");
  RequireContains(failed_edit, "\"rolled_back\":true");
  Require(
      ReadTextFile(root / "map_1" / "octomap.ot") == octomap_before_failed_edit,
      "failed edit did not restore octomap.ot");
  Require(
      ReadTextFile(root / "map_1" / "metadata.json") == metadata_before_failed_edit,
      "failed edit did not restore metadata.json");
  Require(
      ReadTextFile(root / "map_1" / "voxel_edits.jsonl") == journal_before_failed_edit,
      "failed edit did not restore voxel edit journal");
  Require(!HasTransactionDirectory(root / "map_1"), "failed edit left transaction staging");

  const auto invalid_edit = ReadEditOctomapJson(service, "map_1", editor_command, 501.0);
  RequireContains(invalid_edit, "\"success\":false");
  RequireContains(invalid_edit, "\"reason_code\":\"invalid_edit_center\"");

  const auto timeout_created =
      ReadCommandJsonWithId(service, "timeout_map", lingtu_maps_service_create_map_json);
  RequireContains(timeout_created, "\"success\":true");
  const auto timeout_import =
      ReadImportPcdJson(service, "timeout_map", source_pcd.string().c_str(), 0.0);
  RequireContains(timeout_import, "\"success\":true");
  const auto timeout_octomap =
      ReadBuildOctomapJson(service, "timeout_map", WriteSlowOctomapConverter(root), 0.1);
  RequireContains(timeout_octomap, "\"success\":false");
  RequireContains(timeout_octomap, "\"reason_code\":\"octomap_build_failed\"");
  RequireContains(timeout_octomap, "\"reason_code\":\"converter_timeout\"");
  RequireContains(timeout_octomap, "\"timeout_supported\":true");
  Require(
      !std::filesystem::is_regular_file(root / "timeout_map" / "octomap.ot"),
      "timeout converter unexpectedly wrote octomap.ot");

  const auto failed_import_created =
      ReadCommandJsonWithId(service, "failed_import_map", lingtu_maps_service_create_map_json);
  RequireContains(failed_import_created, "\"success\":true");
  SetEnvVar("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER", "1");
  const auto failed_import =
      ReadImportPcdJson(service, "failed_import_map", source_pcd.string().c_str(), 0.0);
  UnsetEnvVar("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER");
  RequireContains(failed_import, "\"success\":false");
  RequireContains(failed_import, "\"reason_code\":\"source_map_commit_failed\"");
  RequireContains(failed_import, "\"rolled_back\":true");
  Require(
      !std::filesystem::is_regular_file(root / "failed_import_map" / "map.pcd"),
      "failed import left a half-published map.pcd");
  Require(
      !std::filesystem::is_regular_file(root / "failed_import_map" / "metadata.json"),
      "failed import left a half-published metadata.json");
  Require(
      !HasTransactionDirectory(root / "failed_import_map"),
      "failed import left transaction staging directory");

  const auto failed_crop_created =
      ReadCommandJsonWithId(service, "failed_crop_map", lingtu_maps_service_create_map_json);
  RequireContains(failed_crop_created, "\"success\":true");
  const auto failed_crop_import =
      ReadImportPcdJson(service, "failed_crop_map", source_pcd.string().c_str(), 0.0);
  RequireContains(failed_crop_import, "\"success\":true");
  const auto failed_crop_size_before =
      std::filesystem::file_size(root / "failed_crop_map" / "map.pcd");
  SetEnvVar("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER", "1");
  const auto failed_crop = ReadCropPcdJson(service, "failed_crop_map", -0.5, 0.5);
  UnsetEnvVar("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER");
  RequireContains(failed_crop, "\"success\":false");
  RequireContains(failed_crop, "\"reason_code\":\"source_map_commit_failed\"");
  RequireContains(failed_crop, "\"rolled_back\":true");
  Require(
      std::filesystem::file_size(root / "failed_crop_map" / "map.pcd") == failed_crop_size_before,
      "failed crop did not restore original map.pcd");
  RequireContains(
      ReadTextFile(root / "failed_crop_map" / "metadata.json"),
      "\"metadata_state\":\"invalidated\"");
  Require(
      !HasTransactionDirectory(root / "failed_crop_map"),
      "failed crop left transaction staging directory");

  const auto restore_created =
      ReadCommandJsonWithId(service, "restore_map", lingtu_maps_service_create_map_json);
  RequireContains(restore_created, "\"success\":true");
  const auto restore_import =
      ReadImportPcdJson(service, "restore_map", source_pcd.string().c_str(), 0.0);
  RequireContains(restore_import, "\"success\":true");
  std::filesystem::copy_file(
      root / "restore_map" / "map.pcd",
      root / "restore_map" / "map.pcd.preclean",
      std::filesystem::copy_options::overwrite_existing);
  RequireContains(
      ReadBuildNavigationPackageJson(service, "restore_map", converter_command),
      "\"success\":true");
  RequireContains(ReadSetActiveJson(service, "restore_map", true), "\"success\":true");
  const auto restored = ReadCommandJsonWithId(
      service, "restore_map", lingtu_maps_service_restore_source_backup_json);
  RequireContains(restored, "\"success\":true");
  RequireContains(restored, "\"state\":\"STALE\"");
  RequireContains(restored, "\"deactivated\":true");
  Require(
      !std::filesystem::is_regular_file(root / "restore_map" / "occupancy.npz"),
      "restore left stale occupancy.npz");
  Require(
      !std::filesystem::is_regular_file(root / "restore_map" / "octomap.ot"),
      "restore left stale octomap.ot");
  RequireContains(
      ReadTextFile(root / "restore_map" / "metadata.json"),
      "\"metadata_state\":\"invalidated\"");
  Require(!HasTransactionDirectory(root / "restore_map"), "restore left transaction staging");

  const auto restore_fail_created =
      ReadCommandJsonWithId(service, "restore_fail", lingtu_maps_service_create_map_json);
  RequireContains(restore_fail_created, "\"success\":true");
  RequireContains(
      ReadImportPcdJson(service, "restore_fail", source_pcd.string().c_str(), 0.0),
      "\"success\":true");
  std::filesystem::copy_file(
      root / "restore_fail" / "map.pcd",
      root / "restore_fail" / "map.pcd.preclean",
      std::filesystem::copy_options::overwrite_existing);
  RequireContains(
      ReadBuildNavigationPackageJson(service, "restore_fail", converter_command),
      "\"success\":true");
  RequireContains(ReadSetActiveJson(service, "restore_fail", true), "\"success\":true");
  const auto restore_fail_pcd = ReadTextFile(root / "restore_fail" / "map.pcd");
  const auto restore_fail_occupancy = ReadTextFile(root / "restore_fail" / "occupancy.npz");
  const auto restore_fail_metadata = ReadTextFile(root / "restore_fail" / "metadata.json");
  SetEnvVar("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER", "1");
  const auto failed_restore = ReadCommandJsonWithId(
      service, "restore_fail", lingtu_maps_service_restore_source_backup_json);
  UnsetEnvVar("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER");
  RequireContains(failed_restore, "\"success\":false");
  RequireContains(failed_restore, "\"reason_code\":\"source_map_commit_failed\"");
  RequireContains(failed_restore, "\"rolled_back\":true");
  Require(ReadTextFile(root / "restore_fail" / "map.pcd") == restore_fail_pcd,
          "failed restore did not roll back map.pcd");
  Require(ReadTextFile(root / "restore_fail" / "occupancy.npz") == restore_fail_occupancy,
          "failed restore did not roll back occupancy.npz");
  Require(ReadTextFile(root / "restore_fail" / "metadata.json") == restore_fail_metadata,
          "failed restore did not roll back metadata.json");
  RequireContains(ReadQueryJson(service, lingtu_maps_service_get_active_json),
                  "\"map_id\":\"restore_fail\"");
  Require(!HasTransactionDirectory(root / "restore_fail"),
          "failed restore left transaction staging");

  const auto failed_occupancy_created =
      ReadCommandJsonWithId(service, "failed_occupancy_map", lingtu_maps_service_create_map_json);
  RequireContains(failed_occupancy_created, "\"success\":true");
  const auto failed_occupancy_import =
      ReadImportPcdJson(service, "failed_occupancy_map", source_pcd.string().c_str(), 0.0);
  RequireContains(failed_occupancy_import, "\"success\":true");
  SetEnvVar("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER", "1");
  const auto failed_occupancy = ReadBuildOccupancyJson(service, "failed_occupancy_map");
  UnsetEnvVar("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER");
  RequireContains(failed_occupancy, "\"success\":false");
  RequireContains(failed_occupancy, "\"reason_code\":\"transaction_commit_failed\"");
  RequireContains(failed_occupancy, "\"rolled_back\":true");
  Require(
      !std::filesystem::is_regular_file(root / "failed_occupancy_map" / "occupancy.npz"),
      "failed occupancy publish left occupancy.npz");
  Require(
      !std::filesystem::is_regular_file(root / "failed_occupancy_map" / "map.pgm"),
      "failed occupancy publish left map.pgm");
  Require(
      !std::filesystem::is_regular_file(root / "failed_occupancy_map" / "map.yaml"),
      "failed occupancy publish left map.yaml");
  Require(
      std::filesystem::is_regular_file(root / "failed_occupancy_map" / "map.pcd"),
      "failed occupancy publish mutated source map.pcd");
  Require(
      !HasTransactionDirectory(root / "failed_occupancy_map"),
      "failed occupancy publish left transaction staging directory");

  const auto failed_esdf_created =
      ReadCommandJsonWithId(service, "failed_esdf_map", lingtu_maps_service_create_map_json);
  RequireContains(failed_esdf_created, "\"success\":true");
  const auto failed_esdf_import =
      ReadImportPcdJson(service, "failed_esdf_map", source_pcd.string().c_str(), 0.0);
  RequireContains(failed_esdf_import, "\"success\":true");
  const auto failed_esdf_occupancy = ReadBuildOccupancyJson(service, "failed_esdf_map");
  RequireContains(failed_esdf_occupancy, "\"success\":true");
  SetEnvVar("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER", "0");
  const auto failed_esdf = ReadBuildEsdfJson(service, "failed_esdf_map");
  UnsetEnvVar("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER");
  RequireContains(failed_esdf, "\"success\":false");
  RequireContains(failed_esdf, "\"reason_code\":\"transaction_commit_failed\"");
  RequireContains(failed_esdf, "\"rolled_back\":true");
  Require(
      !std::filesystem::is_regular_file(root / "failed_esdf_map" / "esdf.npz"),
      "failed esdf publish left esdf.npz");
  Require(
      std::filesystem::is_regular_file(root / "failed_esdf_map" / "occupancy.npz"),
      "failed esdf publish removed prerequisite occupancy.npz");
  Require(
      !HasTransactionDirectory(root / "failed_esdf_map"),
      "failed esdf publish left transaction staging directory");

  const auto failed_trav_created =
      ReadCommandJsonWithId(service, "failed_trav_map", lingtu_maps_service_create_map_json);
  RequireContains(failed_trav_created, "\"success\":true");
  const auto failed_trav_import =
      ReadImportPcdJson(service, "failed_trav_map", source_pcd.string().c_str(), 0.0);
  RequireContains(failed_trav_import, "\"success\":true");
  const auto failed_trav_occupancy = ReadBuildOccupancyJson(service, "failed_trav_map");
  RequireContains(failed_trav_occupancy, "\"success\":true");
  SetEnvVar("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER", "1");
  const auto failed_trav = ReadBuildTraversabilityJson(service, "failed_trav_map");
  UnsetEnvVar("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER");
  RequireContains(failed_trav, "\"success\":false");
  RequireContains(failed_trav, "\"reason_code\":\"transaction_commit_failed\"");
  RequireContains(failed_trav, "\"rolled_back\":true");
  Require(
      !std::filesystem::is_regular_file(root / "failed_trav_map" / "esdf.npz"),
      "failed traversability publish left esdf.npz");
  Require(
      !std::filesystem::is_regular_file(root / "failed_trav_map" / "traversability.npz"),
      "failed traversability publish left traversability.npz");
  Require(
      std::filesystem::is_regular_file(root / "failed_trav_map" / "occupancy.npz"),
      "failed traversability publish removed prerequisite occupancy.npz");
  Require(
      !HasTransactionDirectory(root / "failed_trav_map"),
      "failed traversability publish left transaction staging directory");

  const auto failed_octomap_created =
      ReadCommandJsonWithId(service, "failed_octomap_map", lingtu_maps_service_create_map_json);
  RequireContains(failed_octomap_created, "\"success\":true");
  const auto failed_octomap_import =
      ReadImportPcdJson(service, "failed_octomap_map", source_pcd.string().c_str(), 0.0);
  RequireContains(failed_octomap_import, "\"success\":true");
  SetEnvVar("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER", "1");
  const auto failed_octomap =
      ReadBuildOctomapJson(service, "failed_octomap_map", converter_command);
  UnsetEnvVar("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER");
  RequireContains(failed_octomap, "\"success\":false");
  RequireContains(failed_octomap, "\"reason_code\":\"transaction_commit_failed\"");
  RequireContains(failed_octomap, "\"rolled_back\":true");
  Require(
      !std::filesystem::is_regular_file(root / "failed_octomap_map" / "octomap.ot"),
      "failed octomap publish left octomap.ot");
  RequireContains(
      ReadTextFile(root / "failed_octomap_map" / "metadata.json"),
      "\"metadata_state\":\"invalidated\"");
  Require(
      !HasTransactionDirectory(root / "failed_octomap_map"),
      "failed octomap publish left transaction staging directory");

  const auto package_created =
      ReadCommandJsonWithId(service, "package_map", lingtu_maps_service_create_map_json);
  RequireContains(package_created, "\"success\":true");
  const auto package_import =
      ReadImportPcdJson(service, "package_map", source_pcd.string().c_str(), 0.0);
  RequireContains(package_import, "\"success\":true");
  const auto built_package =
      ReadBuildNavigationPackageJson(service, "package_map", converter_command);
  RequireContains(built_package, "\"action\":\"build_navigation_package\"");
  RequireContains(built_package, "\"success\":true");
  RequireContains(built_package, "\"mode\":\"native_transaction\"");
  RequireContains(built_package, "\"transactional_visibility\":\"staged_until_commit\"");
  RequireContains(built_package, "\"rolled_back\":false");
  Require(
      std::filesystem::is_regular_file(root / "package_map" / "occupancy.npz"),
      "navigation package did not write occupancy.npz");
  Require(
      std::filesystem::is_regular_file(root / "package_map" / "esdf.npz"),
      "navigation package did not write esdf.npz");
  Require(
      std::filesystem::is_regular_file(root / "package_map" / "traversability.npz"),
      "navigation package did not write traversability.npz");
  Require(
      std::filesystem::is_regular_file(root / "package_map" / "octomap.ot"),
      "navigation package did not write octomap.ot");
  Require(
      std::filesystem::is_regular_file(root / "package_map" / "metadata.json"),
      "navigation package did not write metadata.json");
  Require(
      !HasTransactionDirectory(root / "package_map"),
      "successful navigation package left transaction staging directory");

  const auto rollback_created =
      ReadCommandJsonWithId(service, "rollback_map", lingtu_maps_service_create_map_json);
  RequireContains(rollback_created, "\"success\":true");
  const auto rollback_import =
      ReadImportPcdJson(service, "rollback_map", source_pcd.string().c_str(), 0.0);
  RequireContains(rollback_import, "\"success\":true");
  const auto rollback_package =
      ReadBuildNavigationPackageJson(service, "rollback_map", WriteSlowOctomapConverter(root), 0.1);
  RequireContains(rollback_package, "\"action\":\"build_navigation_package\"");
  RequireContains(rollback_package, "\"success\":false");
  RequireContains(rollback_package, "\"transactional_visibility\":\"staged_until_commit\"");
  RequireContains(rollback_package, "\"rolled_back\":true");
  Require(
      !std::filesystem::is_regular_file(root / "rollback_map" / "occupancy.npz"),
      "rollback left occupancy.npz after failed navigation package build");
  Require(
      !std::filesystem::is_regular_file(root / "rollback_map" / "esdf.npz"),
      "rollback left esdf.npz after failed navigation package build");
  Require(
      !std::filesystem::is_regular_file(root / "rollback_map" / "traversability.npz"),
      "rollback left traversability.npz after failed navigation package build");
  Require(
      !std::filesystem::is_regular_file(root / "rollback_map" / "octomap.ot"),
      "rollback left octomap.ot after failed navigation package build");
  Require(
      std::filesystem::is_regular_file(root / "rollback_map" / "metadata.json"),
      "rollback did not restore pre-existing invalidated metadata.json");
  RequireContains(
      ReadTextFile(root / "rollback_map" / "metadata.json"),
      "\"metadata_state\":\"invalidated\"");
  Require(
      !HasTransactionDirectory(root / "rollback_map"),
      "failed navigation package left transaction staging directory");

  const auto active = ReadSetActiveJson(service, "map_1", true);
  RequireContains(active, "\"success\":true");
  RequireContains(active, "\"active\":\"map_1\"");
  const auto valid_artifacts =
      ReadValidateArtifactsJson(service, "map_1", true, true);
  RequireContains(valid_artifacts, "\"action\":\"validate_artifacts\"");
  RequireContains(valid_artifacts, "\"success\":true");
  RequireContains(valid_artifacts, "\"ok\":true");
  RequireContains(valid_artifacts, "\"sha256_ok\":true");
  RequireContains(valid_artifacts, "\"source_map_sha256_matches_map\":true");

  const auto tampered_source_created =
      ReadCommandJsonWithId(service, "tampered_source", lingtu_maps_service_create_map_json);
  RequireContains(tampered_source_created, "\"success\":true");
  RequireContains(
      ReadImportPcdJson(service, "tampered_source", source_pcd.string().c_str(), 0.0),
      "\"success\":true");
  RequireContains(
      ReadBuildOctomapJson(service, "tampered_source", converter_command),
      "\"success\":true");
  {
    std::ofstream tampered_map(
        root / "tampered_source" / "map.pcd", std::ios::binary | std::ios::app);
    tampered_map << "# tampered after artifact build\n";
  }
  const auto tampered_source_validation =
      ReadValidateArtifactsJson(service, "tampered_source", true, false);
  RequireContains(tampered_source_validation, "\"ok\":false");
  RequireContains(tampered_source_validation, "map.pcd sha256 does not match metadata");
  RequireContains(
      tampered_source_validation,
      "octomap source_map_sha256 does not match current map_pcd sha256");
  const auto rejected_tampered_source = ReadSetActiveJson(service, "tampered_source", true);
  RequireContains(rejected_tampered_source, "\"success\":false");
  RequireContains(rejected_tampered_source, "map.pcd sha256 does not match metadata");
  RequireContains(
      ReadQueryJson(service, lingtu_maps_service_get_active_json),
      "\"active\":\"map_1\"");

  const auto tampered_octomap_created =
      ReadCommandJsonWithId(service, "tampered_octomap", lingtu_maps_service_create_map_json);
  RequireContains(tampered_octomap_created, "\"success\":true");
  RequireContains(
      ReadImportPcdJson(service, "tampered_octomap", source_pcd.string().c_str(), 0.0),
      "\"success\":true");
  RequireContains(
      ReadBuildOctomapJson(service, "tampered_octomap", converter_command),
      "\"success\":true");
  {
    std::ofstream tampered_octomap(
        root / "tampered_octomap" / "octomap.ot", std::ios::binary | std::ios::app);
    tampered_octomap << "tampered after artifact build\n";
  }
  const auto tampered_octomap_validation =
      ReadValidateArtifactsJson(service, "tampered_octomap", true, false);
  RequireContains(tampered_octomap_validation, "\"ok\":false");
  RequireContains(tampered_octomap_validation, "octomap sha256 does not match metadata");
  const auto rejected_tampered_octomap =
      ReadSetActiveJson(service, "tampered_octomap", true);
  RequireContains(rejected_tampered_octomap, "\"success\":false");
  RequireContains(rejected_tampered_octomap, "octomap sha256 does not match metadata");
  RequireContains(
      ReadQueryJson(service, lingtu_maps_service_get_active_json),
      "\"active\":\"map_1\"");

  const auto wrong_frame_created =
      ReadCommandJsonWithId(service, "wrong_frame_map", lingtu_maps_service_create_map_json);
  RequireContains(wrong_frame_created, "\"success\":true");
  RequireContains(
      ReadImportPcdJson(service, "wrong_frame_map", source_pcd.string().c_str(), 0.0),
      "\"success\":true");
  RequireContains(
      ReadBuildOctomapJson(service, "wrong_frame_map", converter_command),
      "\"success\":true");
  const auto wrong_frame_metadata_path = root / "wrong_frame_map" / "metadata.json";
  std::string wrong_frame_metadata = ReadTextFile(wrong_frame_metadata_path);
  const std::string top_level_frame = "\"frame_id\":\"map\",";
  const auto top_level_frame_offset = wrong_frame_metadata.find(top_level_frame);
  Require(
      top_level_frame_offset != std::string::npos,
      "generated metadata is missing its top-level frame_id");
  wrong_frame_metadata.erase(top_level_frame_offset, top_level_frame.size());
  const auto closing_brace = wrong_frame_metadata.rfind('}');
  Require(closing_brace != std::string::npos, "generated metadata is not a JSON object");
  wrong_frame_metadata.insert(closing_brace, ",\"frame_id\":\"odom\"");
  WriteTextFile(wrong_frame_metadata_path, wrong_frame_metadata);
  const auto wrong_frame_validation =
      ReadValidateArtifactsJson(service, "wrong_frame_map", true, false, "map");
  RequireContains(wrong_frame_validation, "\"checked_frame_id\":\"odom\"");
  RequireContains(wrong_frame_validation, "\"ok\":false");
  RequireContains(wrong_frame_validation, "frame mismatch: expected map, got odom");
  const auto wrong_frame_record = ReadQueryJsonWithId(
      service, "wrong_frame_map", lingtu_maps_service_get_record_json);
  RequireContains(wrong_frame_record, "\"frame_id\":\"odom\"");
  {
    const auto held_map_lock = root / ".map_locks" / "wrong_frame_map";
    std::filesystem::create_directories(held_map_lock);
    WriteTextFile(
        held_map_lock / "owner.state",
        "owner=service-c-api-test\npid=" + std::to_string(CurrentProcessIdValue()) + "\n");
    const auto busy_points = ReadMapPointsJson(service, "wrong_frame_map", 1U);
    RequireContains(busy_points, "\"success\":false");
    RequireContains(busy_points, "\"reason_code\":\"map_write_in_progress\"");
    std::filesystem::remove_all(held_map_lock);
  }
  const auto wrong_frame_points = ReadMapPointsJson(service, "wrong_frame_map", 1U);
  RequireContains(wrong_frame_points, "\"frame_id\":\"odom\"");
  RequireContains(wrong_frame_points, "\"version_id\":\"wrong_frame_map:v1\"");
  const std::string wrong_frame_map_sha = JsonStringValue(wrong_frame_validation, "sha256");
  Require(wrong_frame_map_sha.size() == 64U, "artifact gate returned an invalid map.pcd sha256");
  RequireContains(
      wrong_frame_points,
      "\"map_pcd_sha256\":\"" + wrong_frame_map_sha + "\"");
  const auto rejected_wrong_frame = ReadSetActiveJson(service, "wrong_frame_map", true);
  RequireContains(rejected_wrong_frame, "\"success\":false");
  RequireContains(rejected_wrong_frame, "frame mismatch: expected map, got odom");
  RequireContains(
      ReadQueryJson(service, lingtu_maps_service_get_active_json),
      "\"active\":\"map_1\"");

  const auto wrong_frame =
      ReadValidateArtifactsJson(service, "map_1", true, true, "world");
  RequireContains(wrong_frame, "\"ok\":false");
  RequireContains(wrong_frame, "frame mismatch");

  const auto poi_set = ReadSetPoiJson(service, "map_1", "home", 1.0, 2.0);
  RequireContains(poi_set, "\"action\":\"poi_set\"");
  RequireContains(poi_set, "\"success\":true");
  RequireContains(poi_set, "\"map_id\":\"map_1\"");
  const auto poi_list = ReadListPoiJson(service, "map_1");
  RequireContains(poi_list, "\"schema_version\":\"map.poi.v1\"");
  RequireContains(poi_list, "\"home\"");
  RequireContains(poi_list, "\"kind\":\"test\"");
  const auto poi_delete = ReadDeletePoiJson(service, "map_1", "home");
  RequireContains(poi_delete, "\"success\":true");

  const auto graph_edge = ReadSetMapEdgeJson(service, "map_1", "package_map");
  RequireContains(graph_edge, "\"action\":\"map_edge_set\"");
  RequireContains(graph_edge, "\"success\":true");
  const auto graph = ReadListGraphJson(service);
  RequireContains(graph, "\"schema_version\":\"map.graph.v1\"");
  RequireContains(graph, "\"storage\":\"map_graph.ltg\"");
  RequireContains(graph, "\"from\":\"map_1\"");
  RequireContains(graph, "\"to\":\"package_map\"");
  Require(!std::filesystem::is_regular_file(root / "map_graph.tsv"), "legacy TSV map graph was written");
  Require(std::filesystem::is_regular_file(root / "map_graph.ltg"), "native map graph was not written");
  const auto route = ReadShortestRouteJson(service, "map_1", "package_map");
  RequireContains(route, "\"action\":\"shortest_route\"");
  RequireContains(route, "\"success\":true");
  RequireContains(route, "\"nodes\":[\"map_1\",\"package_map\"]");

  const auto slots = ReadQueryJson(service, lingtu_maps_service_list_active_slots_json);
  RequireContains(slots, "\"schema_version\":\"map.active_slots.v1\"");
  RequireContains(slots, "\"primary\":\"navigation\"");
  RequireContains(slots, "\"slot\":\"navigation\"");
  RequireContains(slots, "\"map_id\":\"map_1\"");
  const auto reference_slot = ReadSetActiveSlotJson(service, "reference", "package_map", false);
  RequireContains(reference_slot, "\"action\":\"set_active_slot\"");
  RequireContains(reference_slot, "\"slot\":\"reference\"");
  RequireContains(reference_slot, "\"map_id\":\"package_map\"");
  const auto cleared_reference = ReadClearActiveSlotJson(service, "reference");
  RequireContains(cleared_reference, "\"action\":\"clear_active_slot\"");
  RequireContains(cleared_reference, "\"slot\":\"reference\"");

  const auto queued = ReadEnqueueBuildJson(service, "map_1", "OCTOMAP_3D");
  RequireContains(queued, "\"state\":\"QUEUED\"");
  RequireContains(queued, "\"request_id\":");
  const auto queue = ReadQueryJson(service, lingtu_maps_service_get_build_queue_json);
  RequireContains(queue, "\"schema_version\":\"map.artifact_jobs.v2\"");
  RequireContains(queue, "\"worker_owned\":true");
  RequireContains(queue, "\"artifact_type\":\"OCTOMAP_3D\"");

  Touch(root / "package_map" / ".build_lock");
  const auto locked_active = ReadSetActiveJson(service, "package_map", true);
  RequireContains(locked_active, "\"success\":false");
  RequireContains(locked_active, "\"reason_code\":\"map_build_in_progress\"");
  std::filesystem::remove(root / "package_map" / ".build_lock");
  const auto package_active = ReadSetActiveJson(service, "package_map", true);
  RequireContains(package_active, "\"success\":true");
  RequireContains(package_active, "\"previous_active\":\"map_1\"");
  const auto rolled_back_active = ReadRollbackActiveJson(service);
  RequireContains(rolled_back_active, "\"success\":true");
  RequireContains(rolled_back_active, "\"active\":\"map_1\"");

  const auto listed = ReadQueryJson(service, lingtu_maps_service_list_maps_json);
  RequireContains(listed, "\"maps\":[");
  RequireContains(listed, "\"has_occupancy\":true");
  RequireContains(listed, "\"has_esdf\":true");
  RequireContains(listed, "\"has_traversability\":true");

  const auto record =
      ReadQueryJsonWithId(service, "map_1", lingtu_maps_service_get_record_json);
  RequireContains(record, "\"record\":{");
  RequireContains(record, "\"path_planning_2d\"");

  const auto health =
      ReadQueryJsonWithId(service, "map_1", lingtu_maps_service_get_health_json);
  RequireContains(health, "\"active_allowed\":true");

  const auto bundle = ReadBundleJson(service, "map_1", "path_planning_2d");
  RequireContains(bundle, "\"schema_version\":\"map.bundle\"");
  RequireContains(bundle, "\"success\":true");
  RequireContains(bundle, "\"artifact\":{");
  RequireContains(bundle, "\"type\":\"OCCUPANCY_2D\"");

  const auto missing_bundle = ReadBundleJson(service, "map_1", "trajectory_optimization");
  RequireContains(missing_bundle, "\"success\":true");
  RequireContains(missing_bundle, "\"type\":\"ESDF\"");

  const auto traversability_bundle = ReadBundleJson(service, "map_1", "traversability");
  RequireContains(traversability_bundle, "\"success\":true");
  RequireContains(traversability_bundle, "\"type\":\"TRAVERSABILITY\"");

  const auto renamed = ReadRenameJson(service, "map_1", "map_2");
  RequireContains(renamed, "\"map_id\":\"map_2\"");
  const auto renamed_slots = ReadQueryJson(service, lingtu_maps_service_list_active_slots_json);
  RequireContains(renamed_slots, "\"slot\":\"navigation\",\"map_id\":\"map_2\"");

  const auto cleared = ReadClearActiveJson(service);
  RequireContains(cleared, "\"active\":\"\"");

  const auto deleted =
      ReadCommandJsonWithId(service, "map_2", lingtu_maps_service_delete_map_json);
  RequireContains(deleted, "\"success\":true");

  const auto retire_created =
      ReadCommandJsonWithId(service, "retire_map", lingtu_maps_service_create_map_json);
  RequireContains(retire_created, "\"success\":true");
  RequireContains(ReadSetActiveJson(service, "retire_map", false), "\"success\":true");
  RequireContains(
      ReadSetActiveSlotJson(service, "reference", "retire_map", false),
      "\"success\":true");
  const auto retired =
      ReadCommandJsonWithId(service, "retire_map", lingtu_maps_service_retire_map_json);
  RequireContains(retired, "\"action\":\"retire\"");
  RequireContains(retired, "\"success\":true");
  RequireContains(retired, "\"state\":\"RETIRED\"");
  RequireContains(retired, "\"slot_cleanup_ok\":true");
  const auto retired_record =
      ReadQueryJsonWithId(service, "retire_map", lingtu_maps_service_get_record_json);
  RequireContains(retired_record, "\"state\":\"RETIRED\"");
  const auto retired_active = ReadSetActiveJson(service, "retire_map", false);
  RequireContains(retired_active, "\"success\":false");
  RequireContains(retired_active, "\"reason_code\":\"map_retired\"");
  const auto retired_slot = ReadSetActiveSlotJson(service, "reference", "retire_map", false);
  RequireContains(retired_slot, "\"success\":false");
  RequireContains(retired_slot, "\"reason_code\":\"map_retired\"");
  const auto slots_after_retire =
      ReadQueryJson(service, lingtu_maps_service_list_active_slots_json);
  Require(slots_after_retire.find("\"map_id\":\"retire_map\"") == std::string::npos,
          "retired map remained in an active slot");
  const auto retired_renamed = ReadRenameJson(service, "retire_map", "retired_map");
  RequireContains(retired_renamed, "\"success\":true");
  RequireContains(retired_renamed, "\"state\":\"RETIRED\"");
  RequireContains(
      ReadCommandJsonWithId(service, "retired_map", lingtu_maps_service_delete_map_json),
      "\"success\":true");

  lingtu_maps_service_destroy(service);
  std::filesystem::remove_all(root);
  return 0;
}
