#pragma once

#include <filesystem>
#include <functional>
#include <string>

#include "lingtu/maps/build/pcd.hpp"
#include "lingtu/maps/store.hpp"

namespace lingtu::maps {

struct OctomapBuildOptions {
  std::string converter_command;
  std::string build_mode{"external_pcl_converter"};
  double resolution{0.20};
  int support_dilation_cells{1};
  int free_layers_above{3};
  int free_dilation_cells{1};
  std::string frame_id{"map"};
  std::string source_profile{"map_pipeline"};
  std::string data_source{"map_pipeline"};
  std::string slam_source{"unknown"};
  std::string localization_source{"unknown"};
  std::string mapping_source{"lingtu_maps_pipeline"};
  double timeout_sec{60.0};
  std::function<bool()> cancel_requested;
};

struct SourceCommitOptions {
  double voxel_size{0.0};
  bool dynamic_filter_enabled{true};
  bool dynamic_filter_required{false};
  std::string dynamic_filter_command;
  double dynamic_filter_timeout_sec{300.0};
  std::string optimizer_strategy{"off"};
  bool optimizer_required{false};
  std::string optimizer_command;
  double optimizer_timeout_sec{120.0};
  std::function<bool()> cancel_requested;
};

struct OctomapEditOptions {
  std::string editor_command;
  std::string state;
  std::string shape{"sphere"};
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  double radius_m{0.20};
  double timeout_sec{15.0};
  std::function<bool()> cancel_requested;
};

class MapPipelineCore {
 public:
  explicit MapPipelineCore(MapStore& store);

  std::string BeginBuildJson(const std::string& map_id, const std::string& artifact_type);
  std::string FinishBuildJson(
      const std::string& map_id,
      const std::string& build_id,
      bool success,
      const std::string& message);
  std::string GetBuildStatusJson(const std::string& map_id) const;
  std::string ImportPcdJson(
      const std::string& map_id,
      const std::filesystem::path& source_path,
      double voxel_size,
      const PcdBounds& bounds);
  std::string CommitSavedSourceJson(
      const std::string& map_id,
      const std::filesystem::path& source_dir,
      const SourceCommitOptions& options);
  std::string CropPcdJson(
      const std::string& map_id,
      const PcdBounds& bounds,
      bool invert,
      double voxel_size);
  std::string RestoreSourceBackupJson(const std::string& map_id);
  std::string BuildOccupancySnapshotJson(const std::string& map_id);
  std::string BuildOctomapArtifactJson(
      const std::string& map_id,
      const OctomapBuildOptions& options);
  std::string GetVoxelEditsJson(const std::string& map_id) const;
  std::string EditOctomapVoxelsJson(
      const std::string& map_id,
      const OctomapEditOptions& options);
  std::string BuildNavigationPackageJson(
      const std::string& map_id,
      const OctomapBuildOptions& options,
      bool include_esdf,
      bool include_traversability);
  std::string BuildEsdfArtifactJson(const std::string& map_id);
  std::string BuildTraversabilityArtifactJson(const std::string& map_id);
  std::string BuildSemanticArtifactJson(const std::string& map_id);

 private:
  std::filesystem::path BuildDir(const std::string& map_id) const;
  std::filesystem::path LockPath(const std::string& map_id) const;
  std::filesystem::path LockInfoPath(const std::string& map_id) const;
  std::filesystem::path LatestPath(const std::string& map_id) const;
  std::filesystem::path StatusPath(const std::string& map_id, const std::string& build_id) const;

  std::filesystem::path BackupExisting(const std::filesystem::path& path, const std::string& label) const;
  void ClearDerivedArtifacts(const std::filesystem::path& map_dir) const;
  std::string MakeBuildId(const std::string& artifact_type) const;
  std::string ReadLockText(const std::string& map_id) const;
  std::string ReadText(const std::filesystem::path& path) const;
  void WriteText(const std::filesystem::path& path, const std::string& text) const;
  void WriteStatus(
      const std::string& map_id,
      const std::string& build_id,
      const std::string& artifact_type,
      const std::string& status,
      double progress,
      const std::string& message) const;
  std::string FailureJson(
      const std::string& action,
      const std::string& message,
      const std::string& reason_code) const;

  MapStore& store_;
};

}  // namespace lingtu::maps
