#include "lingtu/maps/maintenance.hpp"

#include "lingtu/maps/lock.hpp"

#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

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

void WriteText(const std::filesystem::path& path, const std::string& text) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << text;
  file.flush();
  assert(file);
}

std::string ReadText(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

void WriteMap(
    const std::filesystem::path& root,
    const std::string& map_id,
    const std::string& content,
    std::int64_t schema = 1) {
  const auto dir = root / map_id;
  WriteText(dir / "map.pcd", content);
  WriteText(dir / "occupancy.npz", "occupancy");
  WriteText(dir / "schema_version.txt", std::to_string(schema) + "\n");
}

bool HasAction(const lingtu::maps::MaintenanceReport& report, MaintenanceAction action) {
  for (const auto& change : report.changes) {
    if (change.action == action) return true;
  }
  return false;
}

bool HasIssue(const lingtu::maps::MaintenanceReport& report, const std::string& code) {
  for (const auto& issue : report.issues) {
    if (issue.code == code) return true;
  }
  return false;
}

}  // namespace

int main() {
  const auto root = TempRoot();
  WriteMap(root, "field", "field-map");
  WriteText(root / "active_map.txt", "field\n");

  MapMaintenanceConfig config;
  config.root_dir = root;
  MapPackageMaintenance maintenance(config);

  assert(!lingtu::maps::IsSafePackageRelativePath("../escape"));
  assert(lingtu::maps::IsSafePackageRelativePath("map/map.pcd"));

  const auto audit = maintenance.AuditMaps();
  assert(audit.ok);

  const auto package = root / "field.ltmap";
  const auto exported = maintenance.ExportMapPackage("field", package, false);
  assert(exported.ok);
  assert(HasAction(exported, MaintenanceAction::kExport));
  assert(std::filesystem::is_regular_file(package / "map" / "map.pcd"));
  assert(ReadText(package / "package_manifest.txt").find("version=") == std::string::npos);

  auto package_lock = lingtu::maps::MapLock::TryAcquire(
      root, "__map_packages__", "test-package-contention");
  assert(package_lock.has_value());
  const auto blocked = maintenance.ImportMapPackage(package, "blocked", false);
  assert(!blocked.report.ok);
  assert(HasIssue(blocked.report, "package_exchange_busy"));
  package_lock.reset();

  WriteMap(root, "imported", "old-map");
  WriteText(root / "imported" / "stale.bin", "stale");
  const auto imported = maintenance.ImportMapPackage(package, "imported", false);
  assert(imported.report.ok);
  assert(imported.map_id == "imported");
  assert(ReadText(root / "imported" / "map.pcd") == "field-map");
  assert(!std::filesystem::exists(root / "imported" / "stale.bin"));
  std::filesystem::create_directories(root / "broken");
  const auto corrupt = maintenance.AuditMaps();
  assert(!corrupt.ok);
  assert(HasIssue(corrupt, "map_artifact_missing"));
  const auto quarantined = maintenance.QuarantineCorruptMaps(false);
  assert(HasAction(quarantined, MaintenanceAction::kQuarantine));
  assert(!std::filesystem::exists(root / "broken"));
  assert(std::filesystem::is_directory(root / ".quarantine" / "broken"));

  std::filesystem::remove_all(root);
  return 0;
}
