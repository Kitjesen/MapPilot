#pragma once

#include <cstdint>
#include <filesystem>
#include <set>
#include <string>
#include <vector>

namespace lingtu::maps {

enum class MaintenanceAction {
  kAudit,
  kQuarantine,
  kExport,
  kImport,
};

struct MaintenanceIssue {
  std::string code;
  std::string map_id;
  std::filesystem::path path;
  std::string message;
};

struct MaintenanceChange {
  MaintenanceAction action{MaintenanceAction::kAudit};
  std::string map_id;
  std::filesystem::path path;
  std::filesystem::path target_path;
  std::string message;
};

struct MaintenanceReport {
  bool ok{true};
  bool dry_run{false};
  std::vector<MaintenanceIssue> issues;
  std::vector<MaintenanceChange> changes;

  void AddIssue(MaintenanceIssue issue);
  void AddChange(MaintenanceChange change);
};

struct MapMaintenanceConfig {
  std::filesystem::path root_dir;
  std::string active_state_filename{"active_map.txt"};
  std::string quarantine_dir_name{".quarantine"};
  std::int64_t min_supported_schema{1};
  std::int64_t max_supported_schema{1};
  std::set<std::string> protected_map_ids;
};

struct PackageImportResult {
  MaintenanceReport report;
  std::string map_id;
};

class MapPackageMaintenance {
 public:
  explicit MapPackageMaintenance(MapMaintenanceConfig config);

  const std::filesystem::path& RootDir() const { return root_dir_; }

  MaintenanceReport AuditMaps(bool dry_run = true) const;
  MaintenanceReport QuarantineCorruptMaps(bool dry_run);
  MaintenanceReport ExportMapPackage(
      const std::string& map_id,
      const std::filesystem::path& package_dir,
      bool dry_run) const;
  PackageImportResult ImportMapPackage(
      const std::filesystem::path& package_dir,
      const std::string& requested_map_id,
      bool dry_run);

 private:
  struct MapRef {
    std::string map_id;
    std::filesystem::path dir;
    bool active{false};
    bool protected_map{false};
  };

  std::vector<MapRef> ListMaps() const;
  std::string ActiveMapId() const;
  bool IsProtected(const MapRef& ref) const;
  std::filesystem::path MapPath(const std::string& map_id) const;
  std::filesystem::path QuarantinePath(const MapRef& ref) const;

  MapMaintenanceConfig config_;
  std::filesystem::path root_dir_;
};

bool IsSafePackageRelativePath(const std::filesystem::path& path);

}  // namespace lingtu::maps
