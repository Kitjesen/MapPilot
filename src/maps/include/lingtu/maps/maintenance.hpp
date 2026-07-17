#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace lingtu::maps {

enum class MaintenanceAction {
  kAudit,
  kQuarantine,
  kGarbageCollect,
  kExport,
  kImport,
  kMigrate,
};

struct MaintenanceIssue {
  std::string code;
  std::string map_id;
  std::int64_t version{0};
  std::filesystem::path path;
  std::string message;
};

struct MaintenanceChange {
  MaintenanceAction action{MaintenanceAction::kAudit};
  std::string map_id;
  std::int64_t version{0};
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
  std::size_t retain_latest_versions{2};
  std::set<std::string> protected_map_ids;
  std::map<std::string, std::set<std::int64_t>> protected_versions;
};

struct PackageImportResult {
  MaintenanceReport report;
  std::string map_id;
  std::int64_t version{0};
};

class MapPackageMaintenance {
 public:
  using MigrationHook = std::function<bool(
      const std::filesystem::path& version_dir,
      std::int64_t from_schema,
      std::int64_t to_schema,
      std::string* error)>;

  explicit MapPackageMaintenance(MapMaintenanceConfig config);

  const std::filesystem::path& RootDir() const { return root_dir_; }

  void RegisterMigration(std::int64_t from_schema, std::int64_t to_schema, MigrationHook hook);

  MaintenanceReport AuditImmutableVersions(bool dry_run = true) const;
  MaintenanceReport QuarantineCorruptVersions(bool dry_run);
  MaintenanceReport GarbageCollectVersions(bool dry_run);
  MaintenanceReport ExportMapVersion(
      const std::string& map_id,
      std::int64_t version,
      const std::filesystem::path& package_dir,
      bool dry_run) const;
  PackageImportResult ImportMapPackage(
      const std::filesystem::path& package_dir,
      const std::string& requested_map_id,
      bool dry_run);
  MaintenanceReport MigrateSchemas(bool dry_run);

 private:
  struct VersionRef {
    std::string map_id;
    std::int64_t version{0};
    std::filesystem::path dir;
    bool current{false};
    bool protected_version{false};
  };

  std::vector<VersionRef> ListVersions() const;
  std::string ActiveMapId() const;
  std::int64_t CurrentVersion(const std::filesystem::path& map_dir) const;
  bool IsProtected(const VersionRef& ref) const;
  std::filesystem::path MapPath(const std::string& map_id) const;
  std::filesystem::path QuarantinePath(const VersionRef& ref) const;

  MapMaintenanceConfig config_;
  std::filesystem::path root_dir_;
  std::map<std::pair<std::int64_t, std::int64_t>, MigrationHook> migrations_;
};

bool IsSafePackageRelativePath(const std::filesystem::path& path);

}  // namespace lingtu::maps
