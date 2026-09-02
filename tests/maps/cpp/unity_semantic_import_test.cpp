#include "lingtu/maps/json.hpp"
#include "lingtu/maps/semantic_map_persistence.hpp"
#include "lingtu/maps/semantic_taxonomy.hpp"
#include "lingtu/maps/sources/unity_scene.hpp"

#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <stdexcept>
#include <string>

namespace {

std::filesystem::path TempRoot() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  auto root = std::filesystem::temp_directory_path() /
      ("lingtu_unity_semantic_import_" + std::to_string(stamp));
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root / "scene" / "environment");
  return root;
}

void Write(const std::filesystem::path& path, const std::string& text) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << text;
  assert(file.good());
}

std::string Read(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(file)), {});
}

std::string TaxonomyJson() {
  return R"JSON({
    "name": "lingtu.semantic",
    "version": 7,
    "classes": [
      {"id": 0, "name": "unknown", "color": "#000000", "aliases": ["background"]},
      {"id": 1, "name": "ground", "color": "#111111", "aliases": ["floor"]},
      {"id": 2, "name": "wall", "color": "#222222", "aliases": []},
      {"id": 3, "name": "vegetation", "color": "#333333", "aliases": ["plant", "tree"]},
      {"id": 6, "name": "chair", "color": "#666666", "aliases": []},
      {"id": 8, "name": "vehicle", "color": "#888888", "aliases": ["car"]},
      {"id": 9, "name": "person", "color": "#999999", "aliases": ["pedestrian"]},
      {"id": 10, "name": "animal", "color": "#aaaaaa", "aliases": ["dog"]}
    ]
  })JSON";
}

void WriteScene(const std::filesystem::path& root) {
  Write(root / "taxonomy.json", TaxonomyJson());
  Write(
      root / "scene" / "environment" / "Categories.csv",
      "name,cleaned,nyuId,nyu40id,nyuClass,nyu40class\n"
      "floor,floor,11,2,floor,floor\n"
      "\"sapling, indoor\",indoor sapling,3,40,tree,otherprop\n"
      "human model,person,31,31,person,person\n"
      "bottle,bottle,22,40,bottle,otherprop\n");
  Write(
      root / "scene" / "object_list.txt",
      "0 0 0 -0.05 4 4 0.10 0 \"floor\"\n"
      "1 0.6 0.2 0.5 0.8 0.8 1.0 0.785398 \"chair\"\n"
      "2 -1.0 0.0 0.6 0.6 0.6 1.2 0 \"indoor sapling\"\n"
      "3 2.0 2.0 0.8 0.5 0.5 1.6 0 \"person\"\n"
      "4 1.5 -1.5 0.2 0.1 0.1 0.4 0 \"bottle\"\n");
}

lingtu::maps::sources::UnitySemanticImportConfig Config(
    const std::filesystem::path& root) {
  lingtu::maps::sources::UnitySemanticImportConfig config;
  config.taxonomy_path = root / "taxonomy.json";
  config.voxel_size_m = 0.25F;
  config.generation = 9U;
  config.max_voxels = 100'000U;
  config.max_voxel_checks = 1'000'000U;
  return config;
}

void TestTaxonomy(const std::filesystem::path& root) {
  const auto taxonomy = lingtu::maps::SemanticTaxonomy::LoadJson(root / "taxonomy.json");
  assert(taxonomy.name() == "lingtu.semantic");
  assert(taxonomy.version() == 7U);
  assert(taxonomy.Resolve(" FLOOR ") == 1U);
  assert(taxonomy.Resolve("indoor-tree") == std::nullopt);
  assert(taxonomy.Resolve("pedestrian") == 9U);
  assert(taxonomy.Find(6U) != nullptr);
  assert(taxonomy.Find(6U)->name == "chair");
}

void TestStrictJsonNumberGrammar() {
  assert(lingtu::maps::IsValidJsonObject(
      R"JSON({"fraction":0.125,"exponent":-1.5e+2})JSON"));
  assert(!lingtu::maps::IsValidJsonObject(R"JSON({"overflow":1e9999})JSON"));
  assert(!lingtu::maps::IsValidJsonObject(R"JSON({"leading_zero":01})JSON"));

  const std::string object =
      R"JSON({"schema_version":"example.v1","flags":{"enabled":false}})JSON";
  assert(lingtu::maps::JsonObjectStringAtPath(object, {"schema_version"}) == "example.v1");
  const auto enabled = lingtu::maps::JsonObjectBoolAtPath(object, {"flags", "enabled"});
  assert(enabled.has_value() && !*enabled);
  const auto number = lingtu::maps::JsonObjectNumberAtPath(
      R"JSON({"metrics":{"resolution":0.125}})JSON",
      {"metrics", "resolution"});
  assert(number.has_value() && *number == 0.125);
  assert(!lingtu::maps::JsonObjectNumberAtPath(object, {"schema_version"}).has_value());
  assert(!lingtu::maps::JsonObjectBoolAtPath(object, {"enabled"}).has_value());
  assert(!lingtu::maps::JsonObjectStringAtPath(
      R"JSON({"schema_version":"a","schema_version":"b"})JSON",
      {"schema_version"}).has_value());
  const std::string nested_success =
      R"JSON({"success":false,"detail":{"success":true}})JSON";
  assert(lingtu::maps::JsonObjectBoolAtPath(nested_success, {"success"}) == false);
}

void TestImportAndDeterminism(const std::filesystem::path& root) {
  const auto first = root / "first.bin";
  const auto second = root / "second.bin";
  const auto config = Config(root);
  const auto stats = lingtu::maps::sources::ImportUnitySemanticMap(
      root / "scene", first, config);
  assert(stats.object_rows == 5U);
  assert(stats.accepted_objects == 3U);
  assert(stats.skipped_dynamic_objects == 1U);
  assert(stats.skipped_unmapped_objects == 1U);
  assert(stats.output_voxels > 0U);
  assert(stats.candidate_voxel_checks > stats.output_voxels);
  assert(stats.unmapped_labels == std::vector<std::string>{"bottle"});
  assert(lingtu::maps::ValidateSemanticMapBinary(first));

  const auto loaded = lingtu::maps::ReadSemanticMapBinary(first);
  assert(loaded.generation == 9U);
  assert(loaded.frame_id == "map");
  assert(loaded.taxonomy == "lingtu.semantic");
  assert(loaded.taxonomy_version == 7U);
  std::set<std::uint16_t> labels(
      loaded.data->dominant_label.begin(), loaded.data->dominant_label.end());
  assert(labels.count(1U) == 1U);
  assert(labels.count(3U) == 1U);
  assert(labels.count(6U) == 1U);
  assert(labels.count(9U) == 0U);

  lingtu::maps::sources::ImportUnitySemanticMap(root / "scene", second, config);
  assert(Read(first) == Read(second));
}

void TestFailureDoesNotExposePartialArtifact(const std::filesystem::path& root) {
  const auto output = root / "guarded.bin";
  auto config = Config(root);
  lingtu::maps::sources::ImportUnitySemanticMap(root / "scene", output, config);
  const std::string before = Read(output);
  config.max_voxels = 1U;
  bool failed = false;
  try {
    lingtu::maps::sources::ImportUnitySemanticMap(root / "scene", output, config);
  } catch (const std::length_error&) {
    failed = true;
  }
  assert(failed);
  assert(Read(output) == before);
}

void TestMalformedObjectFailsClosed(const std::filesystem::path& root) {
  const auto scene = root / "malformed";
  std::filesystem::create_directories(scene / "environment");
  std::filesystem::copy_file(
      root / "scene" / "environment" / "Categories.csv",
      scene / "environment" / "Categories.csv");
  Write(scene / "object_list.txt", "0 1 2 broken\n");
  bool failed = false;
  try {
    static_cast<void>(lingtu::maps::sources::BuildUnitySemanticMap(scene, Config(root)));
  } catch (const std::invalid_argument&) {
    failed = true;
  }
  assert(failed);
}

}  // namespace

int main() {
  const auto root = TempRoot();
  WriteScene(root);
  TestTaxonomy(root);
  TestStrictJsonNumberGrammar();
  TestImportAndDeterminism(root);
  TestFailureDoesNotExposePartialArtifact(root);
  TestMalformedObjectFailsClosed(root);
  std::filesystem::remove_all(root);
  return 0;
}
