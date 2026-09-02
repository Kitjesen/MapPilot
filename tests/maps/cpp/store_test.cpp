#include "lingtu/maps/store.hpp"
#include "lingtu/maps/build/occupancy_snapshot.hpp"
#include "lingtu/maps/build/pcd.hpp"

#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

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
  std::ofstream octomap(map_dir / "octomap.ot", std::ios::binary | std::ios::trunc);
  octomap << "# Octomap OcTree binary file\nid OcTree\nsize 1\nres 0.1\ndata\n";
  octomap.put('\0');
  assert(octomap.good());
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

  std::filesystem::remove_all(root);
  return 0;
}
