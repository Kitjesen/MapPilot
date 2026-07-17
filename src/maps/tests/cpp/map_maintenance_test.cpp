#include "lingtu/maps/maintenance.hpp"

#include "lingtu/maps/hash.hpp"
#include "lingtu/maps/version.hpp"

#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using lingtu::maps::MaintenanceAction;
using lingtu::maps::MapMaintenanceConfig;
using lingtu::maps::MapPackageMaintenance;

namespace {

std::filesystem::path TempRoot() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  auto root = std::filesystem::temp_directory_path() /
      ("lingtu_maps_maintenance_test_" + std::to_string(stamp));
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  return root;
}

std::string VersionName(std::int64_t version) {
  std::string out = std::to_string(version);
  while (out.size() < 20U) {
    out.insert(out.begin(), '0');
  }
  return out;
}

void WriteText(const std::filesystem::path& path, const std::string& text) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << text;
  file.flush();
  assert(file);
}

void WriteValidVersion(
    const std::filesystem::path& root,
    const std::string& map_id,
    std::int64_t version,
    std::int64_t schema = 1) {
  const auto dir = root / map_id / ".versions" / VersionName(version);
  WriteText(dir / "map.pcd", "pcd-" + map_id + "-" + std::to_string(version));
  WriteText(dir / "occupancy.npz", "occupancy");
  WriteText(dir / "schema_version.txt", std::to_string(schema) + "\n");
  const std::vector<std::pair<std::string, std::string>> hashes = {
      {"map.pcd", lingtu::maps::Sha256File(dir / "map.pcd")},
      {"occupancy.npz", lingtu::maps::Sha256File(dir / "occupancy.npz")},
      {"schema_version.txt", lingtu::maps::Sha256File(dir / "schema_version.txt")},
  };
  WriteText(dir / lingtu::maps::kArtifactChecksumsFilename,
            lingtu::maps::BuildArtifactChecksums(hashes));
  WriteText(dir / "save_manifest.json", "{\"map_id\":\"" + map_id + "\"}\n");
  WriteText(dir / "save_manifest.sha256",
            lingtu::maps::Sha256File(dir / "save_manifest.json") + "\n" +
                lingtu::maps::Sha256File(dir / lingtu::maps::kArtifactChecksumsFilename) +
                "\n");
}

bool HasAction(const lingtu::maps::MaintenanceReport& report, MaintenanceAction action) {
  for (const auto& change : report.changes) {
    if (change.action == action) {
      return true;
    }
  }
  return false;
}

bool HasIssue(const lingtu::maps::MaintenanceReport& report, const std::string& code) {
  for (const auto& issue : report.issues) {
    if (issue.code == code) {
      return true;
    }
  }
  return false;
}

}  // namespace

int main() {
  const auto root = TempRoot();
  WriteValidVersion(root, "field", 1);
  WriteValidVersion(root, "field", 2);
  WriteValidVersion(root, "field", 3);
  WriteValidVersion(root, "field", 4);
  WriteText(root / "field" / "current_version.txt", VersionName(2) + "\n");
  WriteText(root / "active_map.txt", "field\n");

  MapMaintenanceConfig config;
  config.root_dir = root;
  config.retain_latest_versions = 1;
  config.protected_versions["field"].insert(1);
  MapPackageMaintenance maintenance(config);

  assert(!lingtu::maps::IsSafePackageRelativePath("../escape"));
  assert(!lingtu::maps::IsSafePackageRelativePath("/absolute"));
  assert(lingtu::maps::IsSafePackageRelativePath("version/map.pcd"));

  auto audit = maintenance.AuditImmutableVersions();
  assert(audit.ok);
  assert(audit.issues.empty());

  auto gc_dry = maintenance.GarbageCollectVersions(true);
  assert(gc_dry.ok);
  assert(gc_dry.dry_run);
  assert(HasAction(gc_dry, MaintenanceAction::kGarbageCollect));
  assert(std::filesystem::is_directory(root / "field" / ".versions" / VersionName(3)));
  assert(std::filesystem::is_directory(root / "field" / ".versions" / VersionName(4)));

  auto gc = maintenance.GarbageCollectVersions(false);
  assert(gc.ok);
  assert(std::filesystem::is_directory(root / "field" / ".versions" / VersionName(1)));
  assert(std::filesystem::is_directory(root / "field" / ".versions" / VersionName(2)));
  assert(!std::filesystem::exists(root / "field" / ".versions" / VersionName(3)));
  assert(std::filesystem::is_directory(root / "field" / ".versions" / VersionName(4)));

  WriteValidVersion(root, "broken", 1);
  WriteText(root / "broken" / ".versions" / VersionName(1) / "map.pcd", "tampered");
  auto corrupt_audit = maintenance.AuditImmutableVersions();
  assert(!corrupt_audit.ok);
  assert(HasIssue(corrupt_audit, "version_integrity_failed"));
  auto quarantine_dry = maintenance.QuarantineCorruptVersions(true);
  assert(HasAction(quarantine_dry, MaintenanceAction::kQuarantine));
  assert(std::filesystem::is_directory(root / "broken" / ".versions" / VersionName(1)));
  auto quarantine = maintenance.QuarantineCorruptVersions(false);
  assert(HasAction(quarantine, MaintenanceAction::kQuarantine));
  assert(!std::filesystem::exists(root / "broken" / ".versions" / VersionName(1)));
  assert(std::filesystem::is_directory(root / ".quarantine" / "broken" / VersionName(1)));

  const auto package = root / "field_v2.ltmap";
  auto export_dry = maintenance.ExportMapVersion("field", 2, package, true);
  assert(export_dry.ok);
  assert(HasAction(export_dry, MaintenanceAction::kExport));
  assert(!std::filesystem::exists(package));
  auto exported = maintenance.ExportMapVersion("field", 2, package, false);
  assert(exported.ok);
  assert(std::filesystem::is_regular_file(package / "package_manifest.txt"));
  assert(std::filesystem::is_regular_file(package / "file_index.tsv"));

  WriteText(package / "file_index.tsv", "abc\t../escape\n");
  auto rejected_import = maintenance.ImportMapPackage(package, "imported", true);
  assert(!rejected_import.report.ok);
  assert(HasIssue(rejected_import.report, "package_integrity_failed"));
  exported = maintenance.ExportMapVersion("field", 2, package, false);
  assert(exported.ok);
  auto imported = maintenance.ImportMapPackage(package, "imported", false);
  assert(imported.report.ok);
  assert(imported.map_id == "imported");
  assert(imported.version == 2);
  assert(std::filesystem::is_directory(root / "imported" / ".versions" / VersionName(2)));
  assert(std::filesystem::is_regular_file(root / "imported" / "current_version.txt"));

  const auto migrate_root = TempRoot();
  WriteValidVersion(migrate_root, "old_schema", 1, 0);
  MapMaintenanceConfig migrate_config;
  migrate_config.root_dir = migrate_root;
  migrate_config.min_supported_schema = 2;
  migrate_config.max_supported_schema = 2;
  MapPackageMaintenance migrator(migrate_config);
  auto missing = migrator.MigrateSchemas(true);
  assert(!missing.ok);
  assert(HasIssue(missing, "missing_migration"));
  bool hook_called = false;
  migrator.RegisterMigration(
      1, 2,
      [&hook_called](const std::filesystem::path& version_dir,
                     std::int64_t from,
                     std::int64_t to,
                     std::string* error) {
        hook_called = true;
        assert(from == 1);
        assert(to == 2);
        WriteText(version_dir / "migration_marker.txt", "done\n");
        if (error != nullptr) {
          error->clear();
        }
        return true;
      });
  auto migrated = migrator.MigrateSchemas(false);
  assert(migrated.ok);
  assert(hook_called);
  assert(std::filesystem::is_regular_file(migrate_root / "old_schema" / ".versions" / VersionName(1) /
                                          "migration_marker.txt"));

  std::filesystem::remove_all(migrate_root);
  std::filesystem::remove_all(root);
  return 0;
}
