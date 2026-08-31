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
  std::string previous_active_map_id;
};

struct ArtifactValidationOptions {
  bool require_octomap{false};
  bool require_occupancy{false};
  bool validate_metadata_identity{false};
  std::string expected_frame_id;
  std::string expected_data_source;
  std::string expected_source_profile;
};

struct ArtifactCheck {
  bool exists{false};
  bool format_ok{false};
  std::filesystem::path path;
};

struct DeclaredArtifactIdentity {
  std::string map_id;
  std::int64_t content_epoch{0};
  ArtifactType type{ArtifactType::kPointCloud};
  std::filesystem::path map_dir;
  std::filesystem::path artifact_path;
  std::string frame_id;

  bool valid() const {
    return !map_id.empty() && content_epoch > 0 && !frame_id.empty() && !artifact_path.empty();
  }
};

struct DeclaredArtifactIdentityResult {
  std::optional<DeclaredArtifactIdentity> identity;
  std::string reason;

  bool ok() const { return identity.has_value(); }
};

struct ArtifactValidationResult {
  bool ok{false};
  bool map_found{false};
  bool metadata_ok{false};
  bool metadata_identity_ok{false};
  std::string map_id;
  std::int64_t content_epoch{0};
  std::filesystem::path map_dir;
  std::string checked_frame_id;
  std::string expected_frame_id;
  std::string checked_data_source;
  std::string checked_source_profile;
  ArtifactCheck map_pcd;
  ArtifactCheck octomap;
  ArtifactCheck occupancy_grid;
  std::vector<std::string> metadata_blockers;
  std::vector<std::string> blockers;
};

class MapStore {
 public:
  explicit MapStore(MapStoreConfig config);

  static bool IsValidMapId(const std::string& map_id);
  static std::string NormalizeMapId(const std::string& map_id);
  static constexpr const char* ContentEpochFilename() { return ".content_epoch"; }

  const std::filesystem::path& RootDir() const { return root_dir_; }
  std::filesystem::path MapPath(const std::string& map_id) const;
  std::filesystem::path ContentPath(const std::string& map_id) const;
  std::int64_t ContentEpoch(const std::string& map_id) const;
  std::int64_t AllocateContentEpoch() const;

  std::vector<std::string> ListMapIds() const;
  std::optional<MapRecord> GetMapRecord(const std::string& map_id) const;
  std::optional<MapRecord> GetActiveMap() const;
  std::string ActiveMapId() const;
  bool ValidateActiveState(std::string* error = nullptr) const;

  MapStoreResult CreateMap(const std::string& map_id);
  MapStoreResult DeleteMap(const std::string& map_id);
  MapStoreResult RenameMap(const std::string& map_id, const std::string& new_map_id);
  MapStoreResult RetireMap(const std::string& map_id);
  DeclaredArtifactIdentityResult ReadDeclaredArtifactIdentity(
      const std::string& map_id,
      ArtifactType type,
      const std::string& expected_frame_id) const;
  ArtifactValidationResult ValidateArtifacts(
      const std::string& map_id,
      const ArtifactValidationOptions& options) const;
  ArtifactValidationResult CheckMapActivation(const std::string& map_id) const;
  ArtifactValidationResult CheckMapActivationWhileLocked(
      const std::string& map_id,
      const MapLock& map_lock) const;
  MapStoreResult SetActiveMap(
      const std::string& map_id,
      bool strict,
      const std::optional<std::string>& expected_active_map_id = std::nullopt);
  MapStoreResult SetActiveMapWhileLocked(
      const std::string& map_id,
      bool strict,
      const MapLock& map_lock,
      const std::optional<std::string>& expected_active_map_id = std::nullopt);
  MapStoreResult ClearActiveMap(
      const std::optional<std::string>& expected_active_map_id = std::nullopt);

 private:
  std::filesystem::path ActiveStatePath() const;
  MapRecord ScanRecord(const std::string& map_id, MapState state) const;
  std::vector<MapArtifact> ScanArtifacts(
      const std::filesystem::path& map_dir,
      const std::string& map_id) const;
  bool HasNavigationArtifacts(const std::filesystem::path& map_dir) const;
  ArtifactValidationResult ValidateArtifactsUnlocked(
      const std::string& map_id,
      const ArtifactValidationOptions& options) const;
  ArtifactValidationResult CheckMapActivationUnlocked(const std::string& map_id) const;
  MapStoreResult SetActiveMapUnlocked(
      const std::string& map_id,
      bool strict,
      const std::optional<std::string>& expected_active_map_id);
  bool LockProtectsMap(const MapLock& map_lock, const std::string& map_id) const;
  std::optional<std::string> ReadActiveMapIdStrict(std::string* error) const;
  void WriteActiveMapId(const std::string& map_id) const;

  MapStoreConfig config_;
  std::filesystem::path root_dir_;
};

}  // namespace lingtu::maps
