#include "lingtu/maps/store.hpp"
#include "lingtu/maps/build/occupancy_snapshot.hpp"
#include "lingtu/maps/build/pcd.hpp"
#include "lingtu/maps/build/pipeline.hpp"
#include "lingtu/maps/json.hpp"

#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#if defined(LINGTU_MAPS_HAS_OCTOMAP)
#include <octomap/OcTree.h>
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
       << "\"source_profile\":\"fastlio2\",\"artifacts\":{"
       << "\"map_pcd\":{\"path\":\"map.pcd\",\"frame_id\":\"map\","
          "\"data_source\":\"field\",\"source_profile\":\"fastlio2\"},"
       << "\"occupancy_grid\":{\"path\":\"occupancy.npz\",\"frame_id\":\"map\","
          "\"data_source\":\"field\",\"source_profile\":\"fastlio2\"},"
       << "\"octomap\":{\"path\":\"octomap.ot\",\"frame_id\":\"map\","
          "\"data_source\":\"field\",\"source_profile\":\"fastlio2\"}}}";
}

void WriteValidPlanningArtifacts(const std::filesystem::path& map_dir) {
  const std::vector<lingtu::maps::PointXyz> points = {
      {0.0F, 0.0F, 0.5F},
      {1.0F, 1.0F, 0.5F},
  };
  std::string error;
  assert(lingtu::maps::WriteBinaryXyzPcd(map_dir / "map.pcd", points, &error));
  const auto occupancy = lingtu::maps::BuildOccupancyProjectionSnapshot(map_dir, true);
  assert(occupancy.ok);
#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  octomap::OcTree tree(0.1);
  tree.updateNode(octomap::point3d(0.0F, 0.0F, 0.5F), true);
  assert(tree.write((map_dir / "octomap.ot").string()));
#else
  std::ofstream octomap(map_dir / "octomap.ot", std::ios::binary | std::ios::trunc);
  octomap << "# Octomap OcTree binary file\nid OcTree\nsize 1\nres 0.1\ndata\n";
  octomap.put('\0');
  assert(octomap.good());
#endif
}

void WriteDuplicateFrameMetadata(const std::filesystem::path& path) {
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << "{\"frame_id\":\"map\",\"frame_id\":\"odom\",\"artifacts\":{"
       << "\"map_pcd\":{\"path\":\"map.pcd\"},"
       << "\"occupancy_grid\":{\"path\":\"occupancy.npz\"}}}";
}

}  // namespace

int main() {
  const auto root = TempRoot();
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
  validation_options.expected_source_profile = "fastlio2";
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

#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  lingtu::maps::MapPipelineCore pipeline(store);
  lingtu::maps::OctomapEditOptions edit;
  edit.state = "occupied";
  edit.z_m = 0.5;
  edit.radius_m = 0.1;
  const auto edited = pipeline.EditOctomapVoxelsJson("building_1f", edit);
  assert(lingtu::maps::JsonObjectBoolAtPath(edited, {"success"}) == true);
  assert(store.CheckMapActivation("building_1f").ok);
  WriteValidOccupancyMetadata(root / "building_1f" / "metadata.json");
  octomap::OcTree binary_tree(0.1);
  binary_tree.updateNode(octomap::point3d(0.0F, 0.0F, 0.5F), true);
  assert(binary_tree.writeBinary((root / "building_1f" / "octomap.ot").string()));
  assert(store.CheckMapActivation("building_1f").ok);
  octomap::OcTree empty_tree(0.1);
  assert(empty_tree.write((root / "building_1f" / "octomap.ot").string()));
  assert(!store.CheckMapActivation("building_1f").ok);
  WriteValidPlanningArtifacts(root / "building_1f");
#endif

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

  auto renamed = store.RenameMap("building_1f", "building_2f");
  assert(renamed.ok);
  assert(store.ActiveMapId() == "building_2f");
  assert(std::filesystem::is_directory(root / "building_2f"));
  assert(!std::filesystem::exists(root / "active"));

  auto record = store.GetActiveMap();
  assert(record.has_value());
  assert(record->map_id == "building_2f");
  assert(record->artifacts[0].source_map_id == "building_2f");

  auto deleted = store.DeleteMap("building_2f");
  assert(deleted.ok);
  assert(store.ActiveMapId().empty());
  auto semantic_deleted = store.DeleteMap("semantic_only");
  assert(semantic_deleted.ok);
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
  sampled_options.build_mode = "native_octomap";
  sampled_options.resolution = 0.2;
  const auto sampled_result = pipeline.BuildOctomapArtifactJson("sampled_support", sampled_options);
  assert(lingtu::maps::JsonObjectBoolAtPath(sampled_result, {"success"}) == true);
  octomap::OcTree sampled_tree(0.2);
  assert(sampled_tree.readBinary((root / "sampled_support/octomap.ot").string()));
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
  const auto ray_result = pipeline.BuildOctomapArtifactJson("ray_support",sampled_options);
  assert(lingtu::maps::JsonObjectBoolAtPath(ray_result,{"success"}) == true);
  octomap::OcTree ray_tree(.2);
  assert(ray_tree.readBinary((ray_dir / "octomap.ot").string()));
  const auto* ray_hit=ray_tree.search(1.25,.25,-.75);
  assert(ray_hit && ray_tree.isNodeOccupied(ray_hit));
  const auto* ray_free=ray_tree.search(.5,.1,-.3);
  assert(ray_free && !ray_tree.isNodeOccupied(ray_free));
  assert(ray_tree.search(1.25,.25,.5)==nullptr);
  assert(ray_tree.search(1.25,-.75,.25)==nullptr);
  // A matching external artifact can be reused; changed resolution must
  // execute the converter rather than silently returning the old map.
  std::ifstream metadata_file(ray_dir / "metadata.json");
  std::string metadata((std::istreambuf_iterator<char>(metadata_file)), {});
  const std::string native_mode = "native_octomap";
  auto mode_pos = metadata.find(native_mode);
  assert(mode_pos != std::string::npos);
  while (mode_pos != std::string::npos) {
    metadata.replace(mode_pos, native_mode.size(), "external_pcl_converter");
    mode_pos = metadata.find(native_mode);
  }
  WriteText(ray_dir / "metadata.json", metadata);
  auto reuse_options = sampled_options;
  reuse_options.build_mode = "external_pcl_converter";
  reuse_options.converter_command = "false";
  assert(lingtu::maps::JsonObjectBoolAtPath(
      pipeline.BuildOctomapArtifactJson("ray_support", reuse_options), {"success"}) == true);
  reuse_options.resolution = .1;
  assert(lingtu::maps::JsonObjectBoolAtPath(
      pipeline.BuildOctomapArtifactJson("ray_support", reuse_options), {"success"}) == false);
#endif

  std::filesystem::remove_all(root);
  return 0;
}
