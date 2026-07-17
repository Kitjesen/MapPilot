#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "lingtu/maps/model.hpp"

namespace lingtu::maps {

class MapLock;

struct MapStoreConfig {
  std::filesystem::path root_dir;
  std::string active_state_filename{"active_map.txt"};
};

struct MapStoreResult {
  bool ok{false};
  std::string message;
  std::optional<MapRecord> record;
};

struct ArtifactValidationOptions {
  bool require_octomap{false};
  bool require_occupancy{false};
  bool require_runtime_planning_artifact{false};
  std::string expected_frame_id;
};

struct ArtifactCheck {
  bool exists{false};
  bool sha256_ok{false};
  bool source_map_sha256_matches_map{false};
  std::filesystem::path path;
  std::string sha256;
};

struct ArtifactValidationResult {
  bool ok{false};
  bool map_found{false};
  bool metadata_ok{false};
  bool version_integrity_ok{true};
  std::string map_id;
  std::filesystem::path map_dir;
  std::string checked_frame_id;
  std::string expected_frame_id;
  std::string version_integrity_message;
  ArtifactCheck map_pcd;
  ArtifactCheck octomap;
  ArtifactCheck occupancy_grid;
  std::vector<std::string> blockers;
};

class MapStore {
 public:
  explicit MapStore(MapStoreConfig config);

  static bool IsValidMapId(const std::string& map_id);
  static std::string NormalizeMapId(const std::string& map_id);

  const std::filesystem::path& RootDir() const { return root_dir_; }
  std::filesystem::path MapPath(const std::string& map_id) const;
  std::filesystem::path ContentPath(const std::string& map_id) const;
  std::int64_t CurrentVersion(const std::string& map_id) const;

  std::vector<std::string> ListMapIds() const;
  std::optional<MapRecord> GetMapRecord(const std::string& map_id) const;
  std::optional<MapRecord> GetActiveMap() const;
  std::string ActiveMapId() const;

  MapStoreResult CreateMap(const std::string& map_id);
  MapStoreResult DeleteMap(const std::string& map_id);
  MapStoreResult RenameMap(const std::string& map_id, const std::string& new_map_id);
  MapStoreResult RetireMap(const std::string& map_id);
  ArtifactValidationResult ValidateArtifacts(
      const std::string& map_id,
      const ArtifactValidationOptions& options) const;
  MapStoreResult SetActiveMap(const std::string& map_id, bool strict);
  MapStoreResult SetActiveMapWhileLocked(
      const std::string& map_id,
      bool strict,
      const MapLock& map_lock);
  void ClearActiveMap();

 private:
  std::filesystem::path ActiveStatePath() const;
  MapRecord ScanRecord(const std::string& map_id, MapState state) const;
  std::vector<MapArtifact> ScanArtifacts(
      const std::filesystem::path& map_dir,
      const std::string& map_id) const;
  bool HasRuntimePlanningArtifact(const std::filesystem::path& map_dir) const;
  ArtifactValidationResult ValidateArtifactsUnlocked(
      const std::string& map_id,
      const ArtifactValidationOptions& options) const;
  MapStoreResult SetActiveMapUnlocked(const std::string& map_id, bool strict);
  bool LockProtectsMap(const MapLock& map_lock, const std::string& map_id) const;
  void WriteActiveMapId(const std::string& map_id) const;

  MapStoreConfig config_;
  std::filesystem::path root_dir_;
};

}  // namespace lingtu::maps
