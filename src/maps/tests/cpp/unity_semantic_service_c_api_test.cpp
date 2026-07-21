#include "lingtu/maps/c_api/service.h"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace {

[[noreturn]] void Fail(const std::string& message) {
  std::cerr << message << '\n';
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

std::filesystem::path TempRoot() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  auto root = std::filesystem::temp_directory_path() /
      ("lingtu_unity_semantic_service_" + std::to_string(stamp));
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  return root;
}

void Write(const std::filesystem::path& path, const std::string& text) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << text;
  Require(file.good(), "failed to write " + path.string());
}

std::string Read(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  Require(file.good(), "failed to read " + path.string());
  return std::string((std::istreambuf_iterator<char>(file)), {});
}

void WriteInputs(const std::filesystem::path& root) {
  Write(
      root / "taxonomy.json",
      R"JSON({
        "name": "lingtu.semantic",
        "version": 3,
        "classes": [
          {"id": 0, "name": "unknown", "color": "#000000", "aliases": ["background"]},
          {"id": 1, "name": "ground", "color": "#111111", "aliases": ["floor"]},
          {"id": 6, "name": "chair", "color": "#666666", "aliases": []},
          {"id": 9, "name": "person", "color": "#999999", "aliases": ["pedestrian"]}
        ]
      })JSON");
  Write(
      root / "scene" / "environment" / "Categories.csv",
      "name,cleaned,nyuId,nyu40id,nyuClass,nyu40class\n"
      "floor,floor,11,2,floor,floor\n"
      "human model,person,31,31,person,person\n");
  Write(
      root / "scene" / "object_list.txt",
      "0 0 0 -0.05 3 3 0.10 0 \"floor\"\n"
      "1 0.5 0.2 0.5 0.8 0.8 1.0 0.3 \"chair\"\n"
      "2 2.0 2.0 0.8 0.5 0.5 1.6 0 \"person\"\n");
}

std::string ReadCreateMap(LingtuMapsServiceHandle* service, const char* map_id) {
  std::uint64_t needed = 0U;
  Require(
      lingtu_maps_service_create_map_json(service, map_id, nullptr, 0U, &needed) == 1,
      "create map probe failed");
  std::vector<char> output(static_cast<std::size_t>(needed), '\0');
  Require(
      lingtu_maps_service_create_map_json(
          service, map_id, output.data(), output.size(), &needed) == 0,
      "create map read failed");
  return output.data();
}

std::string ReadImport(
    LingtuMapsServiceHandle* service,
    const std::filesystem::path& scene,
    const LingtuMapsUnitySemanticImportOptions& options) {
  std::uint64_t needed = 0U;
  auto call = [&](char* output, std::uint64_t capacity) {
    return lingtu_maps_service_import_unity_semantic_artifact_json(
        service,
        "unity_map",
        scene.string().c_str(),
        &options,
        output,
        capacity,
        &needed);
  };
  Require(call(nullptr, 0U) == 1, "Unity semantic import probe failed");
  std::vector<char> output(static_cast<std::size_t>(needed), '\0');
  Require(call(output.data(), output.size()) == 0, "Unity semantic import read failed");
  return output.data();
}

bool HasTransactionDirectory(const std::filesystem::path& map_dir) {
  const auto builds = map_dir / ".builds";
  if (!std::filesystem::is_directory(builds)) {
    return false;
  }
  for (const auto& entry : std::filesystem::directory_iterator(builds)) {
    if (entry.is_directory() &&
        entry.path().filename().string().find("_transaction") != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

int main() {
  const auto root = TempRoot();
  WriteInputs(root);
  LingtuMapsServiceHandle* service =
      lingtu_maps_service_create(root.string().c_str(), "active_map.state");
  Require(service != nullptr, "failed to create maps service");
  RequireContains(ReadCreateMap(service, "unity_map"), "\"success\":true");

  LingtuMapsUnitySemanticImportOptions options{};
  options.struct_size = sizeof(options);
  options.abi_version = LINGTU_MAPS_UNITY_SEMANTIC_IMPORT_ABI_VERSION;
  const std::string taxonomy_path = (root / "taxonomy.json").string();
  options.taxonomy_path = taxonomy_path.c_str();
  options.frame_id = "map";
  options.voxel_size_m = 0.25;
  options.occupied_probability = 0.95;
  options.shell_thickness_voxels = 0.75;
  options.generation = 17U;
  options.max_objects = 100U;
  options.max_voxels = 100'000U;
  options.max_voxel_checks = 1'000'000U;
  options.exclude_dynamic_classes = 1U;

  const auto imported = ReadImport(service, root / "scene", options);
  RequireContains(imported, "\"action\":\"import_unity_semantic_artifact\"");
  RequireContains(imported, "\"success\":true");
  RequireContains(imported, "\"mode\":\"native_transaction\"");
  RequireContains(imported, "\"generation\":17");
  RequireContains(imported, "\"skipped_dynamic_objects\":1");
  RequireContains(imported, "\"transactional_visibility\":\"staged_until_commit\"");
  RequireContains(imported, "\"navigation_ready\":false");

  const auto artifact = root / "unity_map" / "semantic_map.bin";
  Require(std::filesystem::is_regular_file(artifact), "semantic_map.bin was not committed");
  const std::string committed = Read(artifact);
  Require(!HasTransactionDirectory(root / "unity_map"), "successful import left transaction data");

  options.max_voxels = 1U;
  const auto rejected = ReadImport(service, root / "scene", options);
  RequireContains(rejected, "\"success\":false");
  RequireContains(rejected, "\"reason_code\":\"invalid_unity_semantic_source\"");
  Require(Read(artifact) == committed, "failed import replaced the committed semantic artifact");
  Require(!HasTransactionDirectory(root / "unity_map"), "failed import left transaction data");

  options.max_voxels = 100'000U;
  options.abi_version = LINGTU_MAPS_UNITY_SEMANTIC_IMPORT_ABI_VERSION + 1U;
  std::uint64_t needed = 0U;
  Require(
      lingtu_maps_service_import_unity_semantic_artifact_json(
          service,
          "unity_map",
          (root / "scene").string().c_str(),
          &options,
          nullptr,
          0U,
          &needed) == -1,
      "unsupported Unity semantic ABI was accepted");
  Require(Read(artifact) == committed, "ABI rejection modified the committed artifact");

  lingtu_maps_service_destroy(service);
  std::filesystem::remove_all(root);
  return 0;
}
