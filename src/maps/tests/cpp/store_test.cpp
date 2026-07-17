#include "lingtu/maps/store.hpp"

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

void WriteValidOccupancyMetadata(const std::filesystem::path& path) {
  constexpr const char* kXHash =
      "2d711642b726b04401627ca9fbac32f5c8530fb1903cc4db02258717921a4881";
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << "{\"frame_id\":\"map\",\"artifacts\":{"
       << "\"map_pcd\":{\"sha256\":\"" << kXHash << "\"},"
       << "\"occupancy_grid\":{\"sha256\":\"" << kXHash
       << "\",\"source_map_sha256\":\"" << kXHash << "\"}}}";
}

}  // namespace

int main() {
  const auto root = TempRoot();
  MapStore store(MapStoreConfig{root});

  assert(MapStore::IsValidMapId("building_1f"));
  assert(!MapStore::IsValidMapId("../bad"));
  assert(!MapStore::IsValidMapId("map..bak"));
  assert(!MapStore::IsValidMapId("-bad"));

  auto created = store.CreateMap("building_1f");
  assert(created.ok);
  assert(created.record.has_value());
  assert(created.record->state == MapState::kDraft);
  assert(std::filesystem::is_directory(root / "building_1f"));

  auto strict_active = store.SetActiveMap("building_1f", true);
  assert(!strict_active.ok);

  Touch(root / "building_1f" / "map.pcd");
  auto stale = store.GetMapRecord("building_1f");
  assert(stale.has_value());
  assert(stale->state == MapState::kStale);
  Touch(root / "building_1f" / "occupancy.npz");
  WriteValidOccupancyMetadata(root / "building_1f" / "metadata.json");
  auto ready = store.GetMapRecord("building_1f");
  assert(ready.has_value());
  assert(ready->state == MapState::kValidated);
  auto active = store.SetActiveMap("building_1f", true);
  assert(active.ok);
  assert(store.ActiveMapId() == "building_1f");
  assert(active.record.has_value());
  assert(active.record->state == MapState::kActive);
  assert(active.record->artifacts.size() == 2);
  assert(active.record->artifacts[0].sha256.size() == 64);
  assert(active.record->artifacts[1].sha256.size() == 64);
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

  std::filesystem::remove_all(root);
  return 0;
}
