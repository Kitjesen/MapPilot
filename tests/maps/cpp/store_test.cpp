#include "lingtu/maps/storage/store.hpp"
#include "lingtu/maps/service.hpp"
#include "lingtu/maps/build/occupancy_snapshot.hpp"
#include "lingtu/maps/io/pcd.hpp"
#include "lingtu/maps/io/patch_bundle.hpp"
#include "lingtu/maps/build/pipeline.hpp"
#include "lingtu/maps/io/json.hpp"
#include "lingtu/maps/no_go.hpp"

#include <cassert>
#include <chrono>
#include <filesystem>
#include <cmath>
#include <fstream>
#include <map>
#include <string>
#include <tuple>

#if defined(LINGTU_MAPS_HAS_OCTOMAP)
#include <octomap/OcTree.h>

#include "lingtu/maps/octomap/io.hpp"
#endif

using lingtu::maps::ArtifactType;
using lingtu::maps::MapState;
using lingtu::maps::MapStore;
using lingtu::maps::MapStoreConfig;

namespace {

std::filesystem::path TempRoot() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  auto root = std::filesystem::temp_directory_path() /
      ("lingtu_maps_store_test_" + std::to_string(stamp));
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  return root;
}

void Touch(const std::filesystem::path& path) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary);
  file << "x";
}

void WriteText(const std::filesystem::path& path, const std::string& value) {
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << value;
}

void WriteValidOccupancyMetadata(const std::filesystem::path& path) {
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << "{\"frame_id\":\"map\",\"data_source\":\"field\","
       << "\"source_profile\":\"ltlio\",\"artifacts\":{"
       << "\"map_pcd\":{\"path\":\"map.pcd\",\"frame_id\":\"map\","
          "\"data_source\":\"field\",\"source_profile\":\"ltlio\"},"
       << "\"occupancy_grid\":{\"path\":\"occupancy.npz\",\"frame_id\":\"map\","
          "\"data_source\":\"field\",\"source_profile\":\"ltlio\"},"
       << "\"octomap\":{\"path\":\"octomap.ot\",\"frame_id\":\"map\","
          "\"data_source\":\"field\",\"source_profile\":\"ltlio\","
          "\"evidence_source\":\"saved_rays\",\"navigation_ready\":true,"
          "\"encoding\":\"full_log_odds\","
          "\"build_mode\":\"native_octomap\"}}}";
}

void WriteValidPlanningArtifacts(const std::filesystem::path& map_dir) {
  const std::vector<lingtu::maps::PointXyz> points = {
      {0.0F, 0.0F, 0.5F},
      {1.0F, 1.0F, 0.5F},
  };
  std::string error;
  assert(lingtu::maps::WriteBinaryXyzPcd(map_dir / "map.pcd", points, &error));
  std::filesystem::create_directories(map_dir / "patches");
  assert(lingtu::maps::WriteBinaryXyzPcd(map_dir / "patches/0.pcd", points, &error));
  WriteText(map_dir / "poses.txt", "0.pcd 0 0 0 1 0 0 0\n");
  WriteText(map_dir / "scan_origin.txt", "lidar_origin_in_patch 0 0 0\n");
  WriteText(map_dir / "patch_bundle.manifest",
            "LINGTU_PATCH_BUNDLE_V1\ncomplete 1\ndropped_count 0\n"
            "first_sequence 0\nlast_sequence 0\npatch_count 1\n");
  const auto occupancy = lingtu::maps::BuildOccupancyProjectionSnapshot(map_dir, true);
  assert(occupancy.ok);
#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  octomap::OcTree tree(0.1);
  tree.updateNode(octomap::point3d(0.0F, 0.0F, 0.5F), true);
  assert(tree.write((map_dir / "octomap.ot").string()));
#else
  std::ofstream octomap(map_dir / "octomap.ot", std::ios::binary | std::ios::trunc);
  octomap << "# Octomap OcTree file\nid OcTree\nsize 1\nres 0.1\ndata\n";
  octomap.put('\0');
  assert(octomap.good());
#endif
}

void TestQueryActivationSummary(const std::filesystem::path& root) {
  using namespace lingtu::maps;
  const auto directory = root / ".query_maps";
  MapStore store(MapStoreConfig{directory});
  assert(store.CreateMap("room").ok);
  WriteValidPlanningArtifacts(directory / "room");
  WriteValidOccupancyMetadata(directory / "room/metadata.json");
  assert(store.CheckMapActivation("room").ok);
  MapsServiceCore service(MapsServiceConfig{MapStoreConfig{directory}});
  const auto listing = ParseJson(service.ListMapsJson());
  const auto& item = listing.AsObject("list").at("maps").AsArray("maps").at(0).AsObject("map");
  assert(std::get<bool>(item.at("can_activate").value));
  const auto& health = item.at("record").AsObject("record").at("health").AsObject("health");
  assert(std::get<bool>(health.at("activation_ready").value));
  assert(store.SetActiveMap("room", true).ok);
  assert(JsonObjectBoolAtPath(service.GetActiveMapJson(),
                              {"record", "health", "activation_ready"}) == true);
  assert(store.ClearActiveMap("room").ok);
  // Query summaries do not load large payloads. Activation still validates them.
  WriteText(directory / "room/map.pcd", "invalid payload\n");
  assert(store.CheckMapActivation("room", false).ok);
  assert(!store.CheckMapActivation("room").ok);
  assert(!store.SetActiveMap("room", true).ok);
}

void WriteDuplicateFrameMetadata(const std::filesystem::path& path) {
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << "{\"frame_id\":\"map\",\"frame_id\":\"odom\",\"artifacts\":{"
       << "\"map_pcd\":{\"path\":\"map.pcd\"},"
       << "\"occupancy_grid\":{\"path\":\"occupancy.npz\"}}}";
}

#if defined(LINGTU_MAPS_HAS_OCTOMAP)
// A saved .ot must return every voxel with the log-odds it was saved with,
// keep unknown space unknown, and be read back under the recorded sensor model.
void TestOctomapRoundTripKeepsEvidence(const std::filesystem::path& root) {
  octomap::OcTree tree(0.1);
  for (int hits = 1; hits <= 6; ++hits) {
    for (int i = 0; i < hits; ++i) {
      tree.updateNode(octomap::point3d(0.1F * static_cast<float>(hits), 0.05F, 0.05F), true);
      tree.updateNode(octomap::point3d(0.1F * static_cast<float>(hits), 1.05F, 0.05F), false);
    }
  }
  tree.updateInnerOccupancy();
  const auto leaves = [](const octomap::OcTree& source) {
    std::map<std::tuple<unsigned, unsigned, unsigned, unsigned>, float> out;
    for (auto it = source.begin_leafs(), end = source.end_leafs(); it != end; ++it) {
      const auto key = it.getKey();
      out[{key[0], key[1], key[2], it.getDepth()}] = it->getLogOdds();
    }
    return out;
  };
  const auto before = leaves(tree);

  const auto ot_path = root / "round_trip.ot";
  assert(lingtu::maps::SaveOctomapTree(tree, ot_path));
  const auto loaded = lingtu::maps::LoadOctomapTree(ot_path);
  assert(loaded != nullptr);
  assert(leaves(*loaded) == before);
  assert(loaded->search(5.05, 5.05, 5.05) == nullptr);
  const auto& model = lingtu::maps::kSavedMapSensorModel;
  const auto near = [](double a, double b) { return std::abs(a - b) < 1e-4; };
  assert(near(loaded->getProbHit(), model.prob_hit));
  assert(near(loaded->getProbMiss(), model.prob_miss));
  assert(near(loaded->getOccupancyThres(), model.occupancy_threshold));
  assert(near(loaded->getClampingThresMin(), model.clamping_min));
  assert(near(loaded->getClampingThresMax(), model.clamping_max));

  // Writing a .bt must not collapse the tree that is still in memory.
  assert(lingtu::maps::SaveOctomapTree(tree, root / "round_trip.bt", true));
  assert(leaves(tree) == before);
}

// Saved-ray replay keeps the evidence each voxel accumulated, writes no
// occupancy that no ray measured, keeps a ray's grazing end from erasing a
// retained surface while its far part still clears one, and leaves no trace of
// filtered returns.
void TestSavedRayEvidence(MapStore& store, const std::filesystem::path& root) {
  const std::string map_id = "ray_evidence";
  assert(store.CreateMap(map_id).ok);
  const auto dir = root / map_id;
  std::filesystem::create_directories(dir / "patches");
  const lingtu::maps::PointXyz floor{1.65F, 0.05F, 0.05F};   // crossed 0.4 m before the wall
  const lingtu::maps::PointXyz ghost{0.35F, 0.05F, 0.05F};   // crossed 1.7 m before the wall
  const lingtu::maps::PointXyz wall{2.05F, 0.05F, 0.05F};    // hit by every scan
  const lingtu::maps::PointXyz post{2.05F, 1.05F, 0.05F};    // hit once
  const lingtu::maps::PointXyz person{1.05F, -1.05F, 0.05F}; // filtered at save time
  const lingtu::maps::PointXyz unhit{3.05F, 3.05F, 0.05F};    // retained, never measured
  std::string error;
  assert(lingtu::maps::WriteBinaryXyzPcd(dir / "map.pcd", {floor, ghost, wall, post, unhit}, &error));
  assert(lingtu::maps::WriteBinaryXyzPcd(dir / "patches/0.pcd", {floor, ghost, wall, post}, &error));
  assert(lingtu::maps::WriteBinaryXyzPcd(dir / "patches/1.pcd", {wall, person}, &error));
  assert(lingtu::maps::WriteBinaryXyzPcd(dir / "patches/2.pcd", {wall}, &error));
  assert(lingtu::maps::WriteBinaryXyzPcd(dir / "patches/3.pcd", {wall}, &error));
  WriteText(dir / "poses.txt",
            "0.pcd 0 0 0 1 0 0 0\n1.pcd 0 0 0 1 0 0 0\n"
            "2.pcd 0 0 0 1 0 0 0\n3.pcd 0 0 0 1 0 0 0\n");
  WriteText(dir / "scan_origin.txt", "lidar_origin_in_patch 0 0 0\n");
  WriteText(dir / "patch_bundle.manifest",
            "LINGTU_PATCH_BUNDLE_V1\ncomplete 1\ndropped_count 0\n"
            "first_sequence 0\nlast_sequence 3\npatch_count 4\n");

  lingtu::maps::MapPipelineCore pipeline(store);
  lingtu::maps::OctomapBuildOptions options;
  options.resolution = 0.1;
  const auto result = pipeline.BuildOctomapArtifactJson(map_id, options);
  assert(lingtu::maps::JsonObjectBoolAtPath(result, {"success"}) == true);
  const auto stat = [&](const char* name) {
    return lingtu::maps::JsonObjectNumberAtPath(result, {"octomap_result", "report", "saved_rays", name});
  };
  assert(stat("inserted_points") == 7.0);
  assert(stat("valid_endpoints") == 8.0);
  assert(stat("retained_endpoints") == 7.0);
  assert(stat("dropped_endpoints") == 1.0);
  assert(stat("hit_updates") == 7.0);
  assert(stat("free_updates").value_or(0.0) > 0.0);
  assert(stat("guarded_miss_suppressions").value_or(0.0) > 0.0);
  const auto read_metadata = [&] {
    std::ifstream file(dir / "metadata.json");
    return std::string((std::istreambuf_iterator<char>(file)), {});
  };
  const auto assert_stats = [&] {
    const auto metadata = read_metadata();
    for (const char* name : {"valid_endpoints", "retained_endpoints", "dropped_endpoints",
                             "free_updates", "hit_updates", "guarded_miss_suppressions"}) {
      assert(lingtu::maps::JsonObjectNumberAtPath(
          metadata, {"artifacts", "octomap", "stats", name}) == stat(name));
    }
  };
  assert_stats();
  assert(!lingtu::maps::JsonObjectNumberAtPath(
      result, {"octomap_result", "report", "saved_rays", "raised_voxels"}).has_value());

  const auto tree = lingtu::maps::LoadOctomapTree(dir / "octomap.ot");
  assert(tree != nullptr);
  const auto log_odds = [&](const lingtu::maps::PointXyz& at) {
    const auto* node = tree->search(at.x, at.y, at.z);
    assert(node != nullptr && tree->isNodeOccupied(node));
    return node->getLogOdds();
  };
  const float one_hit = tree->getProbHitLog();
  // Evidence strength survives: four hits outweigh one, neither is clamped.
  assert(std::abs(log_odds(post) - one_hit) < 1e-5F);
  assert(log_odds(wall) > log_odds(post));
  assert(log_odds(wall) < tree->getClampingThresMaxLog());
  // Three later rays cross the floor voxel within a metre of their wall
  // endpoint: grazing misses do not erase its one measured hit.
  assert(std::abs(log_odds(floor) - one_hit) < 1e-5F);
  // The cell beside the floor voxel, a hole in the sampled floor, is crossed
  // by the same grazing ray ends and stays unknown rather than free.
  assert(tree->search(floor.x - 0.1, floor.y, floor.z) == nullptr);
  // The same rays cross the ghost far from their endpoint and outvote its hit.
  const auto* ghost_node = tree->search(ghost.x, ghost.y, ghost.z);
  assert(ghost_node != nullptr && !tree->isNodeOccupied(ghost_node));
  // A retained point no scan endpoint reached is not invented as occupied.
  assert(tree->search(unhit.x, unhit.y, unhit.z) == nullptr);
  // Space the rays crossed is free; space they never reached stays unknown.
  const auto* crossed = tree->search(0.55, 0.05, 0.05);
  assert(crossed != nullptr && !tree->isNodeOccupied(crossed));
  assert(tree->search(2.05, 0.05, 1.05) == nullptr);
  // A discarded return contributes its measured prefix, but no endpoint hit.
  assert(tree->search(person.x, person.y, person.z) == nullptr);
  const auto* dropped_prefix = tree->search(0.55, -0.55, 0.05);
  assert(dropped_prefix != nullptr && !tree->isNodeOccupied(dropped_prefix));
  // Editing a saved map produced before the surface guard counter existed
  // must preserve its six historical replay counters.
  auto legacy_metadata=read_metadata();
  const std::string optional_counter=",\"surface_guarded_miss_suppressions\":0";
  const auto optional_at=legacy_metadata.find(optional_counter);
  assert(optional_at!=std::string::npos);
  legacy_metadata.erase(optional_at,optional_counter.size());
  WriteText(dir / "metadata.json",legacy_metadata);
  lingtu::maps::OctomapEditOptions edit;
  edit.state = "occupied";
  edit.x_m = 4.0;
  edit.radius_m = 0.1;
  assert(lingtu::maps::JsonObjectBoolAtPath(
      pipeline.EditOctomapVoxelsJson(map_id, edit), {"success"}) == true);
  assert_stats();
}

// An imported point cloud has no saved rays: its OctoMap remains marked as a
// preview, but a valid full OcTree can still be activated. Provenance is not
// a runtime motion gate.
void TestPointCloudPreview(MapStore& store, const std::filesystem::path& root) {
  const std::string map_id = "imported_room";
  assert(store.CreateMap(map_id).ok);
  const auto dir = root / map_id;
  std::vector<lingtu::maps::PointXyz> room;
  for (int x = 0; x < 120; ++x)
    for (int y = 0; y < 80; ++y) room.push_back({x * 0.05F, y * 0.05F, 0.0F});
  std::string error;
  assert(lingtu::maps::WriteBinaryXyzPcd(dir / "map.pcd", room, &error));

  lingtu::maps::MapPipelineCore pipeline(store);
  lingtu::maps::OctomapBuildOptions options;
  options.resolution = 0.1;
  assert(lingtu::maps::JsonObjectBoolAtPath(
             pipeline.BuildOctomapArtifactJson(map_id, options), {"success"}) == true);
  std::ifstream file(dir / "metadata.json");
  const std::string metadata((std::istreambuf_iterator<char>(file)), {});
  assert(lingtu::maps::JsonObjectStringAtPath(
             metadata, {"artifacts", "octomap", "evidence_source"}) == "sampled_points");
  assert(lingtu::maps::JsonObjectBoolAtPath(
             metadata, {"artifacts", "octomap", "navigation_ready"}) == false);
  const auto activation = store.CheckMapActivation(map_id);
  assert(activation.ok);
}

// One operator edit must decide a voxel regardless of how much ray evidence
// it holds, and must not disturb the evidence of voxels outside the edit.
void TestVoxelEditsSetState(MapStore& store, const std::filesystem::path& root) {
  const std::string map_id = "edit_semantics";
  assert(store.CreateMap(map_id).ok);
  const auto map_dir = root / map_id;
  WriteValidPlanningArtifacts(map_dir);
  WriteValidOccupancyMetadata(map_dir / "metadata.json");

  const octomap::point3d wall(0.05F, 0.05F, 0.55F);
  const octomap::point3d corridor(1.05F, 0.05F, 0.55F);
  const octomap::point3d weak(2.05F, 0.05F, 0.55F);
  const octomap::point3d unseen(3.05F, 0.05F, 0.55F);
  octomap::OcTree tree(0.1);
  for (int i = 0; i < 20; ++i) {
    tree.updateNode(wall, true);
    tree.updateNode(corridor, false);
  }
  tree.updateNode(weak, true);
  tree.updateInnerOccupancy();
  const float weak_log_odds = tree.search(weak)->getLogOdds();
  assert(lingtu::maps::SaveOctomapTree(tree, map_dir / "octomap.ot"));

  lingtu::maps::MapPipelineCore pipeline(store);
  const auto edit = [&](const octomap::point3d& at, const char* state) {
    lingtu::maps::OctomapEditOptions options;
    options.state = state;
    options.x_m = at.x();
    options.y_m = at.y();
    options.z_m = at.z();
    options.radius_m = 0.04;
    const auto result = pipeline.EditOctomapVoxelsJson(map_id, options);
    assert(lingtu::maps::JsonObjectBoolAtPath(result, {"success"}) == true);
    assert(lingtu::maps::JsonObjectNumberAtPath(result, {"edit", "edited_voxels"}) == 1.0);
    return *lingtu::maps::JsonObjectNumberAtPath(result, {"edit", "changed_voxels"});
  };
  const auto occupied = [&](const octomap::point3d& at) {
    const auto saved = lingtu::maps::LoadOctomapTree(map_dir / "octomap.ot");
    assert(saved != nullptr);
    const auto* node = saved->search(at);
    assert(node != nullptr);
    return saved->isNodeOccupied(node);
  };

  assert(edit(wall, "free") == 1.0);
  assert(!occupied(wall));
  assert(edit(corridor, "preblocked") == 1.0);
  assert(occupied(corridor));
  assert(edit(corridor, "occupied") == 0.0);
  assert(edit(unseen, "traversable") == 1.0);
  assert(!occupied(unseen));
  assert(edit(wall, "clear") == 0.0);
  assert(!occupied(wall));

  const auto saved = lingtu::maps::LoadOctomapTree(map_dir / "octomap.ot");
  assert(saved->search(weak)->getLogOdds() == weak_log_odds);
}

void TestVoxelEditRebuild(MapStore& store, const std::filesystem::path& root) {
  const std::string map_id = "edit_rebuild";
  assert(store.CreateMap(map_id).ok);
  const auto dir = root / map_id;
  WriteValidPlanningArtifacts(dir);
  lingtu::maps::MapPipelineCore pipeline(store);
  lingtu::maps::OctomapBuildOptions build;
  build.resolution = .1;
  assert(lingtu::maps::JsonObjectBoolAtPath(
      pipeline.BuildOctomapArtifactJson(map_id, build), {"success"}) == true);
  const auto edit = [&](double x, double y, const char* state) {
    lingtu::maps::OctomapEditOptions options;
    options.state = state;
    options.x_m = x;
    options.y_m = y;
    options.z_m = .5;
    options.radius_m = .15;
    assert(lingtu::maps::JsonObjectBoolAtPath(
        pipeline.EditOctomapVoxelsJson(map_id, options), {"success"}) == true);
  };
  edit(0, 0, "free");
  edit(2, 2, "preblocked");
  edit(3, 3, "occupied");
  edit(3, 3, "clear");
  const auto read = [](const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(file), {});
  };
  const auto journal = read(dir / "voxel_edits.jsonl");
  const auto check = [&] {
    const auto tree = lingtu::maps::LoadOctomapTree(dir / "octomap.ot");
    assert(tree && tree->getResolution() == build.resolution);
    const auto occupied = [&](double x, double y) {
      const auto* node = tree->search(x, y, .5);
      assert(node);
      return tree->isNodeOccupied(node);
    };
    assert(!occupied(0, 0));
    assert(occupied(2, 2));
    assert(!occupied(3, 3));
    assert(read(dir / "voxel_edits.jsonl") == journal);
    const auto metadata = read(dir / "metadata.json");
    assert(lingtu::maps::JsonObjectNumberAtPath(metadata, {"manual_voxel_edits", "count"}) == 4);
    assert(lingtu::maps::JsonObjectBoolAtPath(
        metadata, {"artifacts", "octomap", "manual_voxel_edit"}) == true);
    assert(lingtu::maps::JsonObjectStringAtPath(metadata, {"builder_version"}) == "0.3.2");
    assert(lingtu::maps::JsonObjectNumberAtPath(
        metadata, {"artifacts", "octomap", "stats", "valid_endpoints"}).has_value());
  };
  // Navigation-package builds force the same-resolution replay; artifact builds
  // exercise the same world-space edits at a different voxel resolution.
  assert(lingtu::maps::JsonObjectBoolAtPath(
      pipeline.BuildNavigationPackageJson(map_id, build, false, false), {"success"}) == true);
  check();
  build.resolution = .2;
  assert(lingtu::maps::JsonObjectBoolAtPath(
      pipeline.BuildOctomapArtifactJson(map_id, build), {"success"}) == true);
  check();
  const auto octomap = read(dir / "octomap.ot");
  const auto metadata = read(dir / "metadata.json");
  const auto epoch = store.ContentEpoch(map_id);
  WriteText(dir / "voxel_edits.jsonl", journal + "{}\n");
  build.resolution = .05;
  const auto failed = pipeline.BuildOctomapArtifactJson(map_id, build);
  assert(lingtu::maps::JsonObjectBoolAtPath(failed, {"success"}) == false);
  assert(read(dir / "octomap.ot") == octomap);
  assert(read(dir / "metadata.json") == metadata);
  assert(read(dir / "voxel_edits.jsonl") == journal + "{}\n");
  assert(store.ContentEpoch(map_id) == epoch);
}
#endif

void TestNoGoRegions(MapStore& store, const std::filesystem::path& root) {
  using namespace lingtu::maps;
  const std::string id = "no_go_map";
  assert(store.CreateMap(id).ok);
  WriteValidPlanningArtifacts(root / id);
  MapPipelineCore pipeline(store);
  const auto read = [](const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    return std::string{std::istreambuf_iterator<char>(file), {}};
  };
  const auto original_pcd = read(root / id / "map.pcd");
  const auto original_tree = read(root / id / "octomap.ot");
  const NoGoRegion downstairs{"stairs", {.2, .3, .1}, {.8, .9, 1.0}, "east glass", "glass"};
  auto epoch = store.ContentEpoch(id);
  assert(JsonObjectBoolAtPath(pipeline.SetNoGoRegionJson(id, downstairs, epoch), {"success"}) == true);
  assert(store.ContentEpoch(id) > epoch);
  assert(read(root / id / "map.pcd") == original_pcd);
  assert(read(root / id / "octomap.ot") == original_tree);
  auto regions = ReadNoGoRegions(root / id);
  assert(regions.size() == 1 && regions.front().id == "stairs");
  assert(regions.front().label == "east glass" && regions.front().kind == "glass");
  assert(IntersectsNoGo(regions[0], {.4, .5, .5}, {.5, .6, .7}));
  assert(!IntersectsNoGo(regions[0], {.4, .5, 2.0}, {.5, .6, 2.2}));
  assert(SegmentIntersectsNoGo(regions[0], {0, .5, .5}, {1, .5, .5}));
  assert(!SegmentIntersectsNoGo(regions[0], {0, .5, 2}, {1, .5, 2}));
  assert(SegmentIntersectsNoGo(regions[0], {0, 1.0, .5}, {1, 1.0, .5}, {.1, .1, .1}));
  assert(JsonObjectBoolAtPath(pipeline.DeleteNoGoRegionJson(id, "stairs", epoch), {"success"}) == false);
  epoch = store.ContentEpoch(id);
  assert(JsonObjectBoolAtPath(pipeline.SetNoGoRegionJson(id,
      {"invalid", {0, 0, 0}, {1, 1, 0}}, epoch), {"success"}) == false);
  assert(store.ContentEpoch(id) == epoch);
  assert(JsonObjectBoolAtPath(pipeline.SetNoGoRegionJson(id,
      {"upstairs", {.2, .3, 3}, {.8, .9, 4}}, epoch), {"success"}) == true);
  regions = ReadNoGoRegions(root / id);
  assert(regions.size() == 2);
  const auto policy = read(root / id / kNoGoFilename);
  const auto copied = pipeline.SaveEditedSourceJson(id, "no_go_copy", store.ContentEpoch(id), {});
  assert(JsonObjectBoolAtPath(copied, {"success"}) == true);
  assert(read(root / "no_go_copy" / kNoGoFilename) == policy);
#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  OctomapBuildOptions options;
  options.resolution = .2;
  assert(JsonObjectBoolAtPath(pipeline.BuildOctomapArtifactJson("no_go_copy", options), {"success"}) == true);
  assert(read(root / "no_go_copy" / kNoGoFilename) == policy);
#endif
  epoch = store.ContentEpoch(id);
#if defined(_WIN32)
  _putenv_s("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER", "1");
#else
  setenv("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER", "1", 1);
#endif
  const auto failed = pipeline.DeleteNoGoRegionJson(id, "stairs", epoch);
#if defined(_WIN32)
  _putenv_s("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER", "");
#else
  unsetenv("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER");
#endif
  assert(JsonObjectBoolAtPath(failed, {"success"}) == false);
  assert(store.ContentEpoch(id) == epoch);
  assert(read(root / id / kNoGoFilename) == policy);
  assert(JsonObjectBoolAtPath(pipeline.DeleteNoGoRegionJson(id, "stairs", epoch), {"success"}) == true);
  regions = ReadNoGoRegions(root / id);
  assert(regions.size() == 1 && regions.front().id == "upstairs");
  assert(read(root / id / "map.pcd") == original_pcd);
  assert(read(root / id / "octomap.ot") == original_tree);
  assert(JsonObjectBoolAtPath(pipeline.GetNoGoRegionsJson(id), {"success"}) == true);
  SourceCommitOptions source_options;
  source_options.dynamic_filter_enabled = false;
  assert(JsonObjectBoolAtPath(pipeline.CommitSavedSourceJson(id, root / "no_go_copy", source_options),
                              {"success"}) == true);
  // A same-map save preserves user restrictions when the capture has no policy file.
  std::filesystem::remove(root / "no_go_copy" / kNoGoFilename);
  assert(JsonObjectBoolAtPath(pipeline.CommitSavedSourceJson(id, root / "no_go_copy", source_options),
                              {"success"}) == true);
  assert(ReadNoGoRegions(root / id).size() == 2);
  const auto preserved_policy = read(root / id / kNoGoFilename);
  PcdBounds bounds{true, -1, -1, -1, 2, 2, 2};
  assert(JsonObjectBoolAtPath(pipeline.CropPcdJson(id, bounds, false, 0), {"success"}) == true);
  assert(read(root / id / kNoGoFilename) == preserved_policy);
  WriteText(root / "no_go_copy" / kNoGoFilename, "{}");
  bool malformed = false;
  try { static_cast<void>(ReadNoGoRegions(root / "no_go_copy")); }
  catch (const std::exception&) { malformed = true; }
  assert(malformed);
}

}  // namespace

int main() {
  const auto root = TempRoot();
  TestQueryActivationSummary(root);
  MapStore store(MapStoreConfig{root});

  assert(MapStore::IsValidMapId("building_1f"));
  assert(!MapStore::IsValidMapId("../bad"));
  assert(!MapStore::IsValidMapId("map..bak"));
  assert(!MapStore::IsValidMapId("-bad"));
  assert(!MapStore::IsValidMapId("map:v123"));
  assert(!MapStore::IsValidMapId("map:e123"));

  auto created = store.CreateMap("building_1f");
  assert(created.ok);
  assert(created.record.has_value());
  assert(created.record->state == MapState::kDraft);
  assert(std::filesystem::is_directory(root / "building_1f"));

  auto direct = store.CreateMap("direct");
  assert(direct.ok);
  Touch(root / "direct" / "map.pcd");
  Touch(root / "direct" / "occupancy.npz");
  WriteValidOccupancyMetadata(root / "direct" / "metadata.json");
  WriteText(root / "direct" / "current_version.txt", "obsolete\n");
  assert(store.ContentPath("direct") == root / "direct");
  assert(store.ContentEpoch("direct") > 0);
  const auto direct_record = store.GetMapRecord("direct");
  assert(direct_record.has_value());
  assert(!direct_record->artifacts.empty());
  assert(store.DeleteMap("direct").ok);

  auto corrupt_lifecycle = store.CreateMap("corrupt_lifecycle");
  assert(corrupt_lifecycle.ok);
  Touch(root / "corrupt_lifecycle" / "map.pcd");
  Touch(root / "corrupt_lifecycle" / "occupancy.npz");
  WriteValidOccupancyMetadata(root / "corrupt_lifecycle" / "metadata.json");
  WriteText(root / "corrupt_lifecycle" / "lifecycle_state.txt", "\n");
  const auto corrupt_lifecycle_record = store.GetMapRecord("corrupt_lifecycle");
  assert(corrupt_lifecycle_record.has_value());
  assert(corrupt_lifecycle_record->state == MapState::kFailed);
  assert(!store.SetActiveMap("corrupt_lifecycle", false).ok);
  assert(store.ActiveMapId().empty());
  assert(store.DeleteMap("corrupt_lifecycle").ok);

  auto strict_active = store.SetActiveMap("building_1f", true);
  assert(!strict_active.ok);

  Touch(root / "building_1f" / "map.pcd");
  auto stale = store.GetMapRecord("building_1f");
  assert(stale.has_value());
  assert(stale->state == MapState::kStale);
  Touch(root / "building_1f" / "occupancy.npz");
  WriteDuplicateFrameMetadata(root / "building_1f" / "metadata.json");
  assert(!store.SetActiveMap("building_1f", true).ok);
  lingtu::maps::ArtifactValidationOptions validation_options;
  validation_options.require_occupancy = true;
  validation_options.validate_metadata_identity = true;
  validation_options.expected_frame_id = "map";
  validation_options.expected_data_source = "field";
  validation_options.expected_source_profile = "ltlio";
  const auto malformed = store.ValidateArtifacts("building_1f", validation_options);
  assert(!malformed.map_pcd.format_ok);
  assert(!malformed.occupancy_grid.format_ok);
  WriteValidPlanningArtifacts(root / "building_1f");
  WriteValidOccupancyMetadata(root / "building_1f" / "metadata.json");
  const auto validated = store.ValidateArtifacts("building_1f", validation_options);
  assert(validated.map_pcd.format_ok);
  assert(validated.occupancy_grid.format_ok);
  assert(validated.metadata_identity_ok);
  assert(validated.metadata_blockers.empty());
  auto wrong_source = validation_options;
  wrong_source.expected_data_source = "sim";
  const auto source_mismatch = store.ValidateArtifacts("building_1f", wrong_source);
  assert(!source_mismatch.ok);
  assert(!source_mismatch.metadata_identity_ok);
  const auto activation_check = store.CheckMapActivation("building_1f");
  assert(activation_check.ok);
  assert(activation_check.content_epoch == store.ContentEpoch("building_1f"));

  // A valid OctoMap may be activated regardless of provenance. Saved-ray
  // markers remain diagnostic metadata rather than a motion gate.
  const auto metadata_file = root / "building_1f" / "metadata.json";
  const auto valid_metadata = [&] {
    std::ifstream file(metadata_file);
    return std::string((std::istreambuf_iterator<char>(file)), {});
  }();
  const auto blocked_with = [&](const std::string& metadata, const std::string& reason) {
    WriteText(metadata_file, metadata);
    const auto check = store.CheckMapActivation("building_1f");
    assert(!check.ok);
    bool found = false;
    for (const auto& blocker : check.blockers) found |= blocker.find(reason) != std::string::npos;
    assert(found);
  };
  std::string preview = valid_metadata;
  preview.replace(preview.find("\"navigation_ready\":true"), 23, "\"navigation_ready\":false");
  WriteText(metadata_file, preview);
  assert(store.CheckMapActivation("building_1f").ok);
  std::string legacy = valid_metadata;
  legacy.replace(legacy.find(",\"navigation_ready\":true"), 24, "");
  WriteText(metadata_file, legacy);
  assert(store.CheckMapActivation("building_1f").ok);
  assert(store.ValidateArtifacts("building_1f", validation_options).ok);
  WriteText(metadata_file, valid_metadata);
  assert(store.CheckMapActivation("building_1f").ok);
  WriteText(root / "building_1f" / "patch_bundle.manifest", "patches=1\n");
  const auto incomplete_bundle = store.CheckMapActivation("building_1f");
  assert(incomplete_bundle.ok);
  WriteText(root / "building_1f" / "patch_bundle.manifest",
            "LINGTU_PATCH_BUNDLE_V1\ncomplete 1\ndropped_count 0\n"
            "first_sequence 0\nlast_sequence 0\npatch_count 1\n");
  std::filesystem::remove(root / "building_1f" / "patches" / "0.pcd");
  const auto missing_patch = store.CheckMapActivation("building_1f");
  assert(missing_patch.ok);
  std::string error;
  assert(lingtu::maps::WriteBinaryXyzPcd(
      root / "building_1f" / "patches" / "0.pcd",
      {{0.0F, 0.0F, 0.5F}, {1.0F, 1.0F, 0.5F}}, &error));
  assert(store.CheckMapActivation("building_1f").ok);

#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  lingtu::maps::MapPipelineCore pipeline(store);
  lingtu::maps::OctomapEditOptions edit;
  edit.state = "occupied";
  edit.z_m = 0.5;
  edit.radius_m = 0.1;
  const auto edited = pipeline.EditOctomapVoxelsJson("building_1f", edit);
  assert(lingtu::maps::JsonObjectBoolAtPath(edited, {"success"}) == true);
  assert(store.CheckMapActivation("building_1f").ok);
  {
    std::ifstream file(metadata_file);
    const std::string edited_metadata((std::istreambuf_iterator<char>(file)), {});
    assert(!lingtu::maps::JsonObjectHasPath(edited_metadata, {"artifacts", "octomap", "stats"}));
    assert(lingtu::maps::JsonObjectStringAtPath(edited_metadata, {"builder_version"}) != "0.3.1");
  }
  WriteValidOccupancyMetadata(root / "building_1f" / "metadata.json");
  octomap::OcTree binary_tree(0.1);
  binary_tree.updateNode(octomap::point3d(0.0F, 0.0F, 0.5F), true);
  assert(binary_tree.writeBinary((root / "building_1f" / "octomap.ot").string()));
  assert(!store.CheckMapActivation("building_1f").ok);
  // Binary trees remain valid for offline readers, including a .bt-only map.
  assert(store.ValidateArtifacts("building_1f", validation_options).ok);
  std::filesystem::rename(root / "building_1f" / "octomap.ot",
                          root / "building_1f" / "octomap.bt");
  assert(!store.CheckMapActivation("building_1f").ok);
  std::filesystem::remove(root / "building_1f" / "octomap.bt");
  octomap::OcTree empty_tree(0.1);
  assert(empty_tree.write((root / "building_1f" / "octomap.ot").string()));
  assert(!store.CheckMapActivation("building_1f").ok);
  WriteValidPlanningArtifacts(root / "building_1f");
#endif

  std::string wrong_encoding = valid_metadata;
  wrong_encoding.replace(wrong_encoding.find("full_log_odds"), 13, "binary_max_likelihood");
  blocked_with(wrong_encoding, "full_log_odds");
  std::string wrong_path = valid_metadata;
  wrong_path.replace(wrong_path.find("octomap.ot"), 10, "octomap.bt");
  blocked_with(wrong_path, "reference octomap.ot");
  WriteText(metadata_file, valid_metadata);

  WriteText(root / "building_1f" / "map.pcd", "not a pcd\n");
  const auto bad_pcd_activation = store.CheckMapActivation("building_1f");
  assert(!bad_pcd_activation.ok);
  assert(!bad_pcd_activation.map_pcd.format_ok);
  assert(!store.SetActiveMap("building_1f", true).ok);
  WriteValidPlanningArtifacts(root / "building_1f");
  WriteValidOccupancyMetadata(root / "building_1f" / "metadata.json");

  WriteText(root / "building_1f" / "octomap.ot", "not an octomap\n");
  const auto bad_octomap_activation = store.CheckMapActivation("building_1f");
  assert(!bad_octomap_activation.ok);
  assert(!bad_octomap_activation.octomap.format_ok);
  assert(!store.SetActiveMap("building_1f", true).ok);
  WriteValidPlanningArtifacts(root / "building_1f");
  WriteValidOccupancyMetadata(root / "building_1f" / "metadata.json");
  auto ready = store.GetMapRecord("building_1f");
  assert(ready.has_value());
  assert(ready->state == MapState::kValidated);
  auto active = store.SetActiveMap("building_1f", true);
  assert(active.ok);
  assert(active.previous_active_map_id.empty());
  assert(store.ActiveMapId() == "building_1f");
  assert(active.record.has_value());
  assert(active.record->state == MapState::kActive);
  assert(active.record->artifacts.size() == 3);
  assert(active.record->health.localization_stability == 0.0);
  assert(active.record->health.planning_success_rate == 0.0);
  assert(active.record->health.collision_rate == 0.0);
  assert(active.record->health.freshness == 0.0);
  assert(active.record->health.overall_score == 0.0);

  lingtu::maps::MapPipelineCore active_pipeline(store);
  const auto active_epoch = store.ContentEpoch("building_1f");
  const auto active_import = active_pipeline.ImportPcdJson(
      "building_1f", root / "building_1f" / "map.pcd", 0.0, {});
  assert(lingtu::maps::JsonObjectBoolAtPath(active_import, {"success"}) == false);
  assert(lingtu::maps::JsonObjectStringAtPath(active_import, {"reason_code"}) ==
         "active_map_conflict");
#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  const auto active_rebuild =
      active_pipeline.BuildOctomapArtifactJson("building_1f", {});
  assert(lingtu::maps::JsonObjectBoolAtPath(active_rebuild, {"success"}) == false);
  assert(lingtu::maps::JsonObjectStringAtPath(active_rebuild, {"reason_code"}) ==
         "active_map_conflict");
#endif
  assert(store.ContentEpoch("building_1f") == active_epoch);
  assert(!std::filesystem::exists(root / "building_1f" / ".build_lock"));

  auto semantic_only = store.CreateMap("semantic_only");
  assert(semantic_only.ok);
  Touch(root / "semantic_only" / "semantic_map.bin");
  auto semantic_record = store.GetMapRecord("semantic_only");
  assert(semantic_record.has_value());
  assert(semantic_record->state == MapState::kDraft);
  assert(semantic_record->artifacts.size() == 1U);
  assert(semantic_record->artifacts[0].type == ArtifactType::kSemantic);
  assert(semantic_record->health.planning_success_rate == 0.0);
  auto semantic_active = store.SetActiveMap("semantic_only", true);
  assert(!semantic_active.ok);
  auto stale_activate = store.SetActiveMap("semantic_only", false, "unexpected_active");
  assert(!stale_activate.ok);
  assert(store.ActiveMapId() == "building_1f");
  auto stale_clear = store.ClearActiveMap("unexpected_active");
  assert(!stale_clear.ok);
  assert(store.ActiveMapId() == "building_1f");

  const auto active_rename = store.RenameMap("building_1f", "building_2f");
  assert(!active_rename.ok);
  assert(active_rename.message == "active map conflict: building_1f");
  assert(store.ActiveMapId() == "building_1f");
  assert(std::filesystem::is_directory(root / "building_1f"));
  assert(!std::filesystem::exists(root / "building_2f"));

  const auto active_retire = store.RetireMap("building_1f");
  assert(!active_retire.ok);
  assert(active_retire.message == "active map conflict: building_1f");
  assert(store.ActiveMapId() == "building_1f");
  auto record = store.GetActiveMap();
  assert(record.has_value());
  assert(record->map_id == "building_1f");
  assert(record->state == MapState::kActive);

  const auto active_delete = store.DeleteMap("building_1f");
  assert(!active_delete.ok);
  assert(active_delete.message == "active map conflict: building_1f");
  assert(store.ActiveMapId() == "building_1f");
  assert(std::filesystem::is_directory(root / "building_1f"));

  const auto inactive_rename = store.RenameMap("semantic_only", "semantic_renamed");
  assert(inactive_rename.ok);
  assert(store.ActiveMapId() == "building_1f");
  const auto inactive_retire = store.RetireMap("semantic_renamed");
  assert(inactive_retire.ok);
  assert(store.ActiveMapId() == "building_1f");
  const auto inactive_delete = store.DeleteMap("semantic_renamed");
  assert(inactive_delete.ok);
  assert(store.ActiveMapId() == "building_1f");

  assert(store.ClearActiveMap("building_1f").ok);
  assert(store.DeleteMap("building_1f").ok);
  assert(store.ListMapIds().empty());

  auto guarded = store.CreateMap("guarded");
  assert(guarded.ok);
  WriteText(root / "active_map.txt", "../corrupt\n");
  assert(store.ActiveMapId().empty());
  std::string active_state_error;
  assert(!store.ValidateActiveState(&active_state_error));
  assert(active_state_error == "active map state is corrupt");
  assert(!store.SetActiveMap("guarded", false).ok);
  assert(!store.ClearActiveMap().ok);
  assert(!store.RenameMap("guarded", "guarded_renamed").ok);
  assert(!store.RetireMap("guarded").ok);
  assert(!store.DeleteMap("guarded").ok);
  assert(std::filesystem::is_directory(root / "guarded"));

  WriteText(root / "active_map.txt", "\n");
  assert(store.ValidateActiveState());
  assert(store.DeleteMap("guarded").ok);

#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  assert(store.CreateMap("sampled_support").ok);
  std::vector<lingtu::maps::PointXyz> points;
  for (int x = 0; x < 9; ++x)
    for (int y = 0; y < 5; ++y)
      points.push_back({x * 0.2F + 0.1F, y * 0.2F + 0.1F, -0.3F});
  for (int x = 0; x < 5; ++x)
    for (int y = 0; y < 5; ++y)
      points.push_back({x * 0.2F + 0.1F, y * 0.2F + 0.1F, -0.1F});
  points.push_back({4.1F, 4.1F, 0.1F});
  std::string pcd_error;
  assert(lingtu::maps::WriteBinaryXyzPcd(
      root / "sampled_support/map.pcd", points, &pcd_error));
  lingtu::maps::OctomapBuildOptions sampled_options;
  sampled_options.resolution = 0.2;
  const auto sampled_result = pipeline.BuildOctomapArtifactJson("sampled_support", sampled_options);
  assert(lingtu::maps::JsonObjectBoolAtPath(sampled_result, {"success"}) == true);
  {
    std::ifstream file(root / "sampled_support/metadata.json");
    const std::string sampled_metadata((std::istreambuf_iterator<char>(file)), {});
    assert(lingtu::maps::JsonObjectBoolAtPath(
               sampled_metadata, {"artifacts", "octomap", "navigation_ready"}) == false);
  }
  const auto sampled_loaded = lingtu::maps::LoadOctomapTree(root / "sampled_support/octomap.ot");
  assert(sampled_loaded != nullptr);
  const auto& sampled_tree = *sampled_loaded;
  for (const auto& point : points) {
    const auto* node = sampled_tree.search(point.x, point.y, point.z);
    assert(node && sampled_tree.isNodeOccupied(node));
  }
  const auto* raised_floor = sampled_tree.search(1.1, 0.5, -0.1);
  assert(!raised_floor || !sampled_tree.isNodeOccupied(raised_floor));
  assert(sampled_tree.search(4.3, 4.1, 0.1) == nullptr);
  assert(sampled_tree.search(4.1, 4.1, 0.3) == nullptr);
  assert(store.CreateMap("ray_support").ok);
  const auto ray_dir = root / "ray_support";
  std::filesystem::create_directories(ray_dir / "patches");
  assert(lingtu::maps::WriteBinaryXyzPcd(ray_dir / "map.pcd", {{1.25F,.25F,-.75F}}, &pcd_error));
  assert(lingtu::maps::WriteBinaryXyzPcd(ray_dir / "patches/0.pcd",
      {{1.25F,.25F,-.75F}, {1.25F,-.75F,.25F}}, &pcd_error));
  WriteText(ray_dir / "poses.txt", "0.pcd 0 0 0 1 0 0 0\n");
  WriteText(ray_dir / "scan_origin.txt", "lidar_origin_in_patch 0 0 0\n");
  WriteText(ray_dir / "patch_bundle.manifest",
            "LINGTU_PATCH_BUNDLE_V1\ncomplete 1\ndropped_count 0\n"
            "first_sequence 0\nlast_sequence 0\npatch_count 1\n");
  const auto ray_result = pipeline.BuildOctomapArtifactJson("ray_support",sampled_options);
  assert(lingtu::maps::JsonObjectBoolAtPath(ray_result,{"success"}) == true);
  const auto ray_loaded = lingtu::maps::LoadOctomapTree(ray_dir / "octomap.ot");
  assert(ray_loaded != nullptr);
  const auto& ray_tree = *ray_loaded;
  const auto* ray_hit=ray_tree.search(1.25,.25,-.75);
  assert(ray_hit && ray_tree.isNodeOccupied(ray_hit));
  const auto* ray_free=ray_tree.search(.5,.1,-.3);
  assert(ray_free && !ray_tree.isNodeOccupied(ray_free));
  assert(ray_tree.search(1.25,.25,.5)==nullptr);
  assert(ray_tree.search(1.25,-.75,.25)==nullptr);
  {
    std::ifstream octomap_file(ray_dir / "octomap.ot");
    std::string header;
    assert(std::getline(octomap_file, header) && header == "# Octomap OcTree file");
  }
  // An artifact written by the retired external converter, or one built at
  // another resolution, is rebuilt by the embedded builder.
  const auto read_metadata = [&]() {
    std::ifstream file(ray_dir / "metadata.json");
    return std::string((std::istreambuf_iterator<char>(file)), {});
  };
  std::string metadata = read_metadata();
  const std::string native_mode = "native_octomap";
  assert(metadata.find(native_mode) != std::string::npos);
  for (auto pos = metadata.find(native_mode); pos != std::string::npos;
       pos = metadata.find(native_mode)) {
    metadata.replace(pos, native_mode.size(), "external_pcl_converter");
  }
  WriteText(ray_dir / "metadata.json", metadata);
  assert(lingtu::maps::JsonObjectBoolAtPath(
      pipeline.BuildOctomapArtifactJson("ray_support", sampled_options), {"success"}) == true);
  assert(lingtu::maps::JsonObjectStringAtPath(read_metadata(), {"build_mode"}) == native_mode);
  auto finer_options = sampled_options;
  finer_options.resolution = .1;
  assert(lingtu::maps::JsonObjectBoolAtPath(
      pipeline.BuildOctomapArtifactJson("ray_support", finer_options), {"success"}) == true);
  assert(lingtu::maps::JsonObjectNumberAtPath(read_metadata(), {"resolution"}) == .1);

  // The artifact records its encoding and sensor model; a lossy binary
  // artifact is never reused as a saved-map OctoMap.
  metadata = read_metadata();
  assert(lingtu::maps::JsonObjectStringAtPath(metadata, {"artifacts", "octomap", "encoding"}) ==
         "full_log_odds");
  assert(lingtu::maps::JsonObjectBoolAtPath(
             metadata, {"artifacts", "octomap", "navigation_ready"}) == true);
  assert(lingtu::maps::JsonObjectNumberAtPath(
             metadata, {"artifacts", "octomap", "sensor_model", "prob_hit"}) ==
         lingtu::maps::kSavedMapSensorModel.prob_hit);
  assert(lingtu::maps::JsonObjectNumberAtPath(
             metadata, {"artifacts", "octomap", "sensor_model", "clamping_max"}) ==
         lingtu::maps::kSavedMapSensorModel.clamping_max);
  const std::string lossless = "full_log_odds";
  metadata.replace(metadata.find(lossless), lossless.size(), "binary_max_likelihood");
  WriteText(ray_dir / "metadata.json", metadata);
  const auto rebuilt = pipeline.BuildOctomapArtifactJson("ray_support", finer_options);
  assert(lingtu::maps::JsonObjectStringAtPath(rebuilt, {"status"}) != "reused");
  assert(lingtu::maps::JsonObjectStringAtPath(
             read_metadata(), {"artifacts", "octomap", "encoding"}) == lossless);

  const auto build_status = [&] {
    return lingtu::maps::JsonObjectStringAtPath(
        pipeline.BuildOctomapArtifactJson("ray_support", finer_options),
        {"octomap_result", "status"});
  };
  assert(build_status() == "reused");
  metadata = read_metadata();
  assert(lingtu::maps::JsonObjectStringAtPath(metadata, {"builder_version"}) == "0.3.2");
  const std::string version_field = "\"builder_version\":\"0.3.2\"";
  metadata.replace(metadata.find(version_field), version_field.size(),
                   "\"builder_version\":\"0.3.1\"");
  WriteText(ray_dir / "metadata.json", metadata);
  assert(build_status() == "built");
  // Missing any replay counter makes a saved-ray artifact ineligible for reuse.
  for (const char* name : {"valid_endpoints", "retained_endpoints", "dropped_endpoints",
                           "free_updates", "hit_updates", "guarded_miss_suppressions"}) {
    metadata = read_metadata();
    const std::string key = std::string("\"") + name + "\"";
    metadata.replace(metadata.find(key), key.size(), "\"old_counter\"");
    WriteText(ray_dir / "metadata.json", metadata);
    assert(build_status() == "built");
  }
  assert(build_status() == "reused");

  const auto source_epoch = store.ContentEpoch("ray_support");
  for (const auto &[name, resolution] : std::vector<std::pair<std::string, double>>{
           {"ray_5cm", .05}, {"ray_10cm", .1}, {"ray_20cm", .2}}) {
    const auto copied = pipeline.SaveEditedSourceJson("ray_support", name, source_epoch, {});
    assert(lingtu::maps::JsonObjectBoolAtPath(copied, {"success"}) == true);
    assert(lingtu::maps::HasSavedRays(root / name));
    auto resolution_options = finer_options;
    resolution_options.resolution = resolution;
    const auto built = pipeline.BuildOctomapArtifactJson(name, resolution_options);
    assert(lingtu::maps::JsonObjectBoolAtPath(built, {"success"}) == true);
    const auto changed_tree = lingtu::maps::LoadOctomapTree(root / name / "octomap.ot");
    assert(changed_tree && changed_tree->getResolution() == resolution);
    const auto* hit = changed_tree->search(1.25, .25, -.75);
    assert(hit && changed_tree->isNodeOccupied(hit));
    const auto* free = changed_tree->search(.625, .125, -.375);
    assert(free && !changed_tree->isNodeOccupied(free));
    assert(changed_tree->search(1.25, .25, .5) == nullptr);
    std::ifstream metadata_file(root / name / "metadata.json");
    const std::string derivative_metadata{std::istreambuf_iterator<char>(metadata_file), {}};
    assert(lingtu::maps::JsonObjectStringAtPath(derivative_metadata, {"source_map_id"}) == "ray_support");
    assert(lingtu::maps::JsonObjectNumberAtPath(derivative_metadata, {"resolution"}) == resolution);
  }
  assert(store.ContentEpoch("ray_support") == source_epoch);
  assert(lingtu::maps::LoadOctomapTree(ray_dir / "octomap.ot")->getResolution() == .1);

  lingtu::maps::SourceEditOperation manual;
  manual.operation = "add_plane";
  manual.bounds = {true, 3, 3, -.5, 4, 4, .5};
  manual.axis = "z";
  manual.offset = 0;
  manual.spacing = .1;
  const auto patch_preview = pipeline.PreviewSourceEditsJson("ray_support", source_epoch, {manual});
  assert(lingtu::maps::JsonObjectBoolAtPath(patch_preview, {"success"}) == true);
  assert(lingtu::maps::JsonObjectNumberAtPath(patch_preview, {"added_points"}) == 121);
  assert(lingtu::maps::JsonObjectNumberAtPath(patch_preview, {"removed_points"}) == 0);
  assert(lingtu::maps::JsonObjectBoolAtPath(pipeline.SaveEditedSourceJson(
      "ray_support", "manual_patch", source_epoch, {manual}), {"success"}) == true);
  const auto patch_epoch = store.ContentEpoch("manual_patch");
  assert(std::filesystem::is_regular_file(root / "manual_patch" / "point_edits.json"));
  for (double resolution : {.1, .2}) {
    auto patch_options = finer_options;
    patch_options.resolution = resolution;
    assert(lingtu::maps::JsonObjectBoolAtPath(pipeline.BuildOctomapArtifactJson(
        "manual_patch", patch_options), {"success"}) == true);
    const auto patch_tree = lingtu::maps::LoadOctomapTree(root / "manual_patch" / "octomap.ot");
    const auto* patch_hit = patch_tree->search(3.5, 3.5, 0);
    assert(patch_hit && patch_tree->isNodeOccupied(patch_hit));
    assert(patch_tree->search(3.5, 3.5, .5) == nullptr);
  }
  assert(store.ContentEpoch("manual_patch") >= patch_epoch);
  assert(lingtu::maps::JsonObjectBoolAtPath(pipeline.SaveEditedSourceJson(
      "manual_patch", "manual_copy", store.ContentEpoch("manual_patch"), {}), {"success"}) == true);
  assert(lingtu::maps::JsonObjectBoolAtPath(pipeline.BuildOctomapArtifactJson(
      "manual_copy", finer_options), {"success"}) == true);
  const auto copied_patch = lingtu::maps::LoadOctomapTree(root / "manual_copy" / "octomap.ot");
  assert(copied_patch->isNodeOccupied(copied_patch->search(3.5, 3.5, 0)));
  assert(copied_patch->search(3.5, 3.5, .5) == nullptr);
  assert(lingtu::maps::JsonObjectBoolAtPath(pipeline.CropPcdJson("manual_copy",
      {true, -10, -10, -10, 3.55, 4, 4}, false, 0), {"success"}) == true);
  assert(lingtu::maps::JsonObjectBoolAtPath(pipeline.BuildOctomapArtifactJson(
      "manual_copy", finer_options), {"success"}) == true);
  const auto cropped_patch = lingtu::maps::LoadOctomapTree(root / "manual_copy" / "octomap.ot");
  assert(cropped_patch->isNodeOccupied(cropped_patch->search(3.5, 3.5, 0)));
  assert(cropped_patch->search(3.9, 3.5, 0) == nullptr);

  TestOctomapRoundTripKeepsEvidence(root);
  TestSavedRayEvidence(store, root);
  TestPointCloudPreview(store, root);
  TestVoxelEditsSetState(store, root);
  TestVoxelEditRebuild(store, root);
#endif

  TestNoGoRegions(store, root);
  std::filesystem::remove_all(root);
  return 0;
}
