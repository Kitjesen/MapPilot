#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "lingtu/maps/build/pipeline.hpp"
#include "lingtu/maps/health.hpp"
#include "lingtu/maps/maintenance.hpp"
#include "lingtu/maps/save.hpp"
#include "lingtu/maps/store.hpp"

namespace lingtu::maps {

struct MapsServiceConfig {
  MapStoreConfig store;
};

class MapsServiceCore {
 public:
  explicit MapsServiceCore(MapsServiceConfig config);

  std::string ListMapsJson() const;
  std::string GetMapTypesJson() const;
  std::string GetRecordJson(const std::string &map_id) const;
  std::string GetActiveMapJson() const;
  std::string GetHealthJson(const std::string &map_id) const;
  std::string ValidateArtifactsJson(const std::string &map_id, bool require_octomap,
                                    bool require_occupancy, const std::string &expected_frame_id,
                                    const std::string &expected_data_source,
                                    const std::string &expected_source_profile) const;
  std::string IngestLocalizationHealthJson(const std::string &map_id, double timestamp_s,
                                           bool localized, double position_error_m,
                                           double covariance_trace, double quality,
                                           const std::string &source);
  std::string IngestPlanningOutcomeJson(const std::string &map_id, double timestamp_s, bool success,
                                        const std::string &planner, const std::string &reason);
  std::string IngestCollisionEventJson(const std::string &map_id, double timestamp_s,
                                       double severity, const std::string &source,
                                       const std::string &reason);
  std::string GetBundleJson(const std::string &map_id, const std::string &capability) const;
  std::string GetMapPointsJson(const std::string &map_id, std::uint64_t max_points) const;
  std::string ListPoiJson(const std::string &map_id) const;
  std::string SetPoiJson(const std::string &map_id, const std::string &name, double x_m, double y_m,
                         double z_m, double yaw_rad, bool has_yaw, const std::string &frame_id,
                         const std::string &tags_json);
  std::string DeletePoiJson(const std::string &map_id, const std::string &name);
  std::string ListMapGraphJson() const;
  std::string SetMapEdgeJson(const std::string &from_map_id, const std::string &to_map_id,
                             const std::string &edge_type, bool bidirectional);
  std::string DeleteMapEdgeJson(const std::string &from_map_id, const std::string &to_map_id);
  std::string ShortestRouteJson(const std::string &start_map_id,
                                const std::string &goal_map_id) const;
  std::string CreateMapJson(const std::string &map_id);
  std::string DeleteMapJson(const std::string &map_id);
  std::string RenameMapJson(const std::string &map_id, const std::string &new_map_id);
  std::string RetireMapJson(const std::string &map_id);
  std::string ImportPcdJson(const std::string &map_id, const std::filesystem::path &source_path,
                            double voxel_size, const PcdBounds &bounds);
  std::string CommitSavedSourceJson(const std::string &map_id,
                                    const std::filesystem::path &source_dir,
                                    const SourceCommitOptions &options);
  std::string CropPcdJson(const std::string &map_id, const PcdBounds &bounds, bool invert,
                          double voxel_size);
  std::string BuildOccupancySnapshotJson(const std::string &map_id);
  std::string BuildOctomapArtifactJson(const std::string &map_id,
                                       const OctomapBuildOptions &options);
  std::string GetVoxelEditsJson(const std::string &map_id) const;
  std::string EditOctomapVoxelsJson(const std::string &map_id, const OctomapEditOptions &options);
  std::string BuildNavigationPackageJson(const std::string &map_id,
                                         const OctomapBuildOptions &options, bool include_esdf,
                                         bool include_traversability);
  std::string BuildEsdfArtifactJson(const std::string &map_id);
  std::string BuildTraversabilityArtifactJson(const std::string &map_id);
  std::string BuildSemanticArtifactJson(const std::string &map_id);
  std::string BeginSaveMapJson(const SaveMapRequest &request);
  std::string ProvideSaveMapSnapshotJson(const std::string &job_id, const MapSnapshot &snapshot);
  std::string RejectSaveMapSnapshotJson(const std::string &job_id, const std::string &reason_code,
                                        const std::string &message);
  std::string GetSaveMapStatusJson(const std::string &job_id) const;
  std::string ListSaveMapJobsJson(std::size_t limit = 100U) const;
  std::string CancelSaveMapJson(const std::string &job_id);
  std::string RetrySaveMapJson(const std::string &job_id);
  std::string AuditMapsJson(bool dry_run) const;
  std::string QuarantineCorruptMapsJson(bool dry_run);
  std::string ExportMapPackageJson(const std::string &map_id,
                                   const std::filesystem::path &package_dir, bool dry_run) const;
  std::string ImportMapPackageJson(const std::filesystem::path &package_dir,
                                   const std::string &requested_map_id, bool dry_run);

  MapStore &Store() { return store_; }
  const MapStore &Store() const { return store_; }

 private:
  std::string RecordJson(const MapRecord &record) const;
  std::string ArtifactJson(const MapArtifact &artifact) const;
  std::string ArtifactsJson(const MapRecord &record) const;
  std::string CapabilitiesJson(const MapRecord &record) const;
  std::string HealthJson(const MapRecord &record) const;
  health::MapHealthModel *HealthModelFor(const MapRecord &record) const;
  std::string ActiveArtifactsJson(const std::string &map_id) const;
  std::filesystem::path PoiPath(const std::string &map_id) const;
  std::filesystem::path GraphPath() const;
  std::string MaintenanceReportJson(const std::string &action,
                                    const MaintenanceReport &report) const;
  const MapArtifact *FindArtifactForCapability(const MapRecord &record,
                                               const std::string &capability) const;
  std::string FailureJson(const std::string &action, const std::string &message,
                          const std::string &reason_code = "operation_failed") const;
  SaveMapEngine &SaveEngine();
  const SaveMapEngine &SaveEngine() const;

  MapStore store_;
  MapPipelineCore pipeline_;
  mutable std::once_flag save_engine_once_;
  mutable std::unique_ptr<SaveMapEngine> save_engine_;
  MapPackageMaintenance maintenance_;
  struct HealthEntry {
    std::int64_t content_epoch{-1};
    std::string artifact_signature;
    std::unique_ptr<health::MapHealthModel> model;
  };
  mutable std::mutex health_mutex_;
  mutable std::unordered_map<std::string, HealthEntry> health_models_;
};

}  // namespace lingtu::maps
