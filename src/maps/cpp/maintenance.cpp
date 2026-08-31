#include "lingtu/maps/maintenance.hpp"

#include "lingtu/maps/lock.hpp"
#include "lingtu/maps/store.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <system_error>

#if defined(_WIN32)
#  define NOMINMAX
#  include <windows.h>
#else
#  include <fcntl.h>
#  include <unistd.h>
#endif

namespace lingtu::maps {
namespace {

constexpr const char* kPackageManifest = "package_manifest.txt";
constexpr const char* kSchemaFile = "schema_version.txt";
constexpr const char* kPackageExchangeLockId = "__map_packages__";

std::string Trim(std::string value) {
  while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())) != 0) {
    value.pop_back();
  }
  const auto begin = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  });
  return std::string(begin, value.end());
}

std::int64_t ParsePositiveInt(const std::string& value) {
  if (value.empty() || !std::all_of(value.begin(), value.end(), [](unsigned char ch) {
        return std::isdigit(ch) != 0;
      })) {
    return 0;
  }
  try {
    return std::stoll(value);
  } catch (...) {
    return 0;
  }
}

void SyncPath(const std::filesystem::path& path, bool directory) {
#if defined(_WIN32)
  const DWORD flags = directory ? FILE_FLAG_BACKUP_SEMANTICS : FILE_ATTRIBUTE_NORMAL;
  HANDLE handle = CreateFileW(
      path.wstring().c_str(),
      GENERIC_READ,
      FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
      nullptr,
      OPEN_EXISTING,
      flags,
      nullptr);
  if (handle != INVALID_HANDLE_VALUE) {
    FlushFileBuffers(handle);
    CloseHandle(handle);
  }
#else
  const int flags = directory ? (O_RDONLY | O_DIRECTORY) : O_RDONLY;
  const int fd = open(path.c_str(), flags);
  if (fd >= 0) {
    fsync(fd);
    close(fd);
  }
#endif
}

void AtomicReplace(const std::filesystem::path& source, const std::filesystem::path& target) {
#if defined(_WIN32)
  if (MoveFileExW(
          source.wstring().c_str(),
          target.wstring().c_str(),
          MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) == 0) {
    throw std::runtime_error("failed to atomically replace " + target.string());
  }
#else
  if (::rename(source.c_str(), target.c_str()) != 0) {
    throw std::runtime_error("failed to atomically replace " + target.string());
  }
#endif
  SyncPath(target.parent_path(), true);
}

void WriteTextAtomic(const std::filesystem::path& path, const std::string& value) {
  std::filesystem::create_directories(path.parent_path());
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto tmp = path.parent_path() / (path.filename().string() + ".tmp-" + std::to_string(stamp));
  {
    std::ofstream file(tmp, std::ios::binary | std::ios::trunc);
    if (!file) {
      throw std::runtime_error("failed to write temp file: " + tmp.string());
    }
    file.write(value.data(), static_cast<std::streamsize>(value.size()));
    file.flush();
    if (!file) {
      throw std::runtime_error("failed to flush temp file: " + tmp.string());
    }
  }
  try {
    SyncPath(tmp, false);
    AtomicReplace(tmp, path);
  } catch (...) {
    std::error_code ignored;
    std::filesystem::remove(tmp, ignored);
    throw;
  }
}

bool IsQuarantineOrLockDir(const std::filesystem::path& path, const MapMaintenanceConfig& config) {
  const auto name = path.filename().string();
  return name == config.quarantine_dir_name || name == ".map_locks" || name == ".save_jobs";
}

bool IsSymlink(const std::filesystem::directory_entry& entry) {
  std::error_code error;
  return std::filesystem::is_symlink(entry.symlink_status(error));
}

void CopyTreeChecked(
    const std::filesystem::path& source,
    const std::filesystem::path& target,
    const std::filesystem::path& relative_root) {
  if (!std::filesystem::is_directory(source)) {
    throw std::runtime_error("source directory does not exist: " + source.string());
  }
  std::filesystem::create_directories(target);
  for (const auto& entry : std::filesystem::recursive_directory_iterator(source)) {
    if (IsSymlink(entry)) {
      throw std::runtime_error("symlink rejected in package tree: " + entry.path().string());
    }
    const auto rel = std::filesystem::relative(entry.path(), relative_root);
    if (!IsSafePackageRelativePath(rel)) {
      throw std::runtime_error("unsafe package path: " + rel.generic_string());
    }
    const auto destination = target / rel;
    if (entry.is_directory()) {
      std::filesystem::create_directories(destination);
    } else if (entry.is_regular_file()) {
      std::filesystem::create_directories(destination.parent_path());
      std::filesystem::copy_file(
          entry.path(), destination, std::filesystem::copy_options::overwrite_existing);
    }
  }
}

void ReplaceDirectoryRecoverably(
    const std::filesystem::path& staged,
    const std::filesystem::path& target) {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto backup = target.parent_path() /
      (target.filename().string() + ".backup-" + std::to_string(stamp));
  const bool had_target = std::filesystem::exists(target);
  if (had_target) {
    std::filesystem::rename(target, backup);
  }
  try {
    std::filesystem::rename(staged, target);
  } catch (...) {
    if (had_target) {
      std::error_code rollback_error;
      std::filesystem::rename(backup, target, rollback_error);
      if (rollback_error) {
        throw std::runtime_error(
            "package publish failed and previous package restore failed: " +
            rollback_error.message());
      }
    }
    throw;
  }
  if (had_target) {
    std::error_code ignored;
    std::filesystem::remove_all(backup, ignored);
  }
}

std::map<std::string, std::string> ReadKeyValueFile(const std::filesystem::path& path) {
  std::map<std::string, std::string> out;
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("package manifest is unreadable");
  }
  std::string line;
  while (std::getline(file, line)) {
    const auto pos = line.find('=');
    if (pos == std::string::npos || pos == 0U ||
        line.find('=', pos + 1U) != std::string::npos) {
      throw std::runtime_error("package manifest contains a malformed entry");
    }
    const auto key = Trim(line.substr(0, pos));
    const auto value = Trim(line.substr(pos + 1));
    if (key.empty() || value.empty() || key != line.substr(0, pos) ||
        value != line.substr(pos + 1) || !out.emplace(key, value).second) {
      throw std::runtime_error("package manifest contains an invalid or duplicate entry");
    }
  }
  if (file.bad()) {
    throw std::runtime_error("package manifest could not be read completely");
  }
  return out;
}

std::int64_t ReadSchema(const std::filesystem::path& map_dir) {
  const auto path = map_dir / kSchemaFile;
  std::error_code status_error;
  if (!std::filesystem::exists(path, status_error)) {
    return status_error ? -1 : 1;
  }
  if (!std::filesystem::is_regular_file(path, status_error) || status_error) {
    return -1;
  }
  std::ifstream file(path, std::ios::binary);
  std::string line;
  if (!file || !std::getline(file, line) || line != Trim(line)) {
    return -1;
  }
  std::string trailing;
  if (std::getline(file, trailing) || file.bad()) {
    return -1;
  }
  if (line == "0") {
    return 1;
  }
  const auto schema = ParsePositiveInt(line);
  return schema > 0 ? schema : -1;
}

}  // namespace

void MaintenanceReport::AddIssue(MaintenanceIssue issue) {
  ok = false;
  issues.push_back(std::move(issue));
}

void MaintenanceReport::AddChange(MaintenanceChange change) {
  changes.push_back(std::move(change));
}

bool IsSafePackageRelativePath(const std::filesystem::path& path) {
  if (path.empty() || path.is_absolute() || path.has_root_path() || !path.root_name().empty()) {
    return false;
  }
  for (const auto& part : path) {
    if (part.empty() || part == "." || part == "..") {
      return false;
    }
  }
  return true;
}

MapPackageMaintenance::MapPackageMaintenance(MapMaintenanceConfig config)
    : config_(std::move(config)), root_dir_(config_.root_dir) {
  if (root_dir_.empty()) {
    throw std::invalid_argument("MapPackageMaintenance root_dir is required");
  }
  std::filesystem::create_directories(root_dir_);
}

MaintenanceReport MapPackageMaintenance::AuditMaps(bool dry_run) const {
  MaintenanceReport report;
  report.dry_run = dry_run;
  MapStore store(MapStoreConfig{root_dir_, config_.active_state_filename});
  std::string active_state_error;
  if (!store.ValidateActiveState(&active_state_error)) {
    report.AddIssue({
        "active_state_corrupt", {},
        root_dir_ / config_.active_state_filename,
        active_state_error});
  }
  for (const auto& ref : ListMaps()) {
    if (!std::filesystem::is_regular_file(ref.dir / "map.pcd")) {
      report.AddIssue({"map_artifact_missing", ref.map_id, ref.dir, "map.pcd is missing"});
    }
    const auto schema = ReadSchema(ref.dir);
    if (schema <= 0) {
      report.AddIssue({
          "schema_invalid", ref.map_id, ref.dir / kSchemaFile,
          "schema version is malformed"});
      continue;
    }
    if (schema < config_.min_supported_schema) {
      report.AddIssue({"schema_too_old", ref.map_id, ref.dir, "schema is older than supported minimum"});
    }
    if (schema > config_.max_supported_schema) {
      report.AddIssue({"schema_too_new", ref.map_id, ref.dir, "schema is newer than supported maximum"});
    }
  }
  for (const auto& map_id : std::vector<std::string>{ActiveMapId()}) {
    if (map_id.empty()) {
      continue;
    }
    const auto map_dir = MapPath(map_id);
    if (!std::filesystem::is_directory(map_dir)) {
      report.AddIssue({"active_map_missing", map_id, map_dir, "active map directory is missing"});
    }
  }
  return report;
}

MaintenanceReport MapPackageMaintenance::QuarantineCorruptMaps(bool dry_run) {
  MaintenanceReport report;
  report.dry_run = dry_run;
  for (const auto& ref : ListMaps()) {
    if (IsProtected(ref)) {
      continue;
    }
    if (std::filesystem::is_regular_file(ref.dir / "map.pcd")) {
      continue;
    }
    const auto lock = MapLock::TryAcquire(root_dir_, ref.map_id, "quarantine-corrupt");
    if (!lock.has_value()) {
      report.AddIssue({"lock_busy", ref.map_id, ref.dir, "map write lock is busy"});
      continue;
    }
    const auto target = QuarantinePath(ref);
    report.AddChange({MaintenanceAction::kQuarantine, ref.map_id, ref.dir, target, "map.pcd is missing"});
    if (!dry_run) {
      std::filesystem::create_directories(target.parent_path());
      std::filesystem::rename(ref.dir, target);
    }
  }
  return report;
}

MaintenanceReport MapPackageMaintenance::ExportMapPackage(
    const std::string& map_id,
    const std::filesystem::path& package_dir,
    bool dry_run) const {
  MaintenanceReport report;
  report.dry_run = dry_run;
  std::string id;
  try {
    id = MapStore::NormalizeMapId(map_id);
  } catch (const std::exception& exc) {
    report.AddIssue({"invalid_map_id", map_id, {}, exc.what()});
    return report;
  }
  if (package_dir.empty()) {
    report.AddIssue({"invalid_package_path", id, package_dir, "package path is required"});
    return report;
  }
  std::optional<MapLock> package_lock;
  std::optional<MapLock> map_lock;
  if (!dry_run) {
    package_lock = MapLock::TryAcquire(root_dir_, kPackageExchangeLockId, "export-map-package");
    if (!package_lock.has_value()) {
      report.AddIssue({
          "package_exchange_busy", id, package_dir,
          "another map package operation is in progress"});
      return report;
    }
    map_lock = MapLock::TryAcquire(root_dir_, id, "export-map-package");
    if (!map_lock.has_value()) {
      report.AddIssue({"lock_busy", id, package_dir, "map write lock is busy"});
      return report;
    }
  }
  const auto source = MapPath(id);
  if (!std::filesystem::is_directory(source) ||
      !std::filesystem::is_regular_file(source / "map.pcd")) {
    report.AddIssue({"map_artifact_missing", id, source, "map.pcd is missing"});
    return report;
  }
  const auto source_schema = ReadSchema(source);
  if (source_schema <= 0) {
    report.AddIssue({
        "schema_invalid", id, source / kSchemaFile,
        "schema version is malformed"});
    return report;
  }
  report.AddChange({MaintenanceAction::kExport, id, source, package_dir, "export verified map package"});
  if (dry_run) {
    return report;
  }
  const auto stage = package_dir.parent_path() /
      (package_dir.filename().string() + ".staging-" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::remove_all(stage);
  std::filesystem::create_directories(stage / "map");
  CopyTreeChecked(source, stage / "map", source);
  WriteTextAtomic(
      stage / kPackageManifest,
      "format=lingtu-map-package-v1\nmap_id=" + id +
          "\nschema=" + std::to_string(source_schema) + "\n");
  ReplaceDirectoryRecoverably(stage, package_dir);
  return report;
}

PackageImportResult MapPackageMaintenance::ImportMapPackage(
    const std::filesystem::path& package_dir,
    const std::string& requested_map_id,
    bool dry_run) {
  PackageImportResult result;
  result.report.dry_run = dry_run;
  if (!std::filesystem::is_directory(package_dir)) {
    result.report.AddIssue({
        "package_integrity_failed", {}, package_dir, "package directory is missing"});
    return result;
  }
  std::optional<MapLock> package_lock;
  if (!dry_run) {
    package_lock = MapLock::TryAcquire(root_dir_, kPackageExchangeLockId, "import-map-package");
    if (!package_lock.has_value()) {
      result.report.AddIssue({
          "package_exchange_busy", {}, package_dir,
          "another map package operation is in progress"});
      return result;
    }
  }
  std::map<std::string, std::string> manifest;
  try {
    manifest = ReadKeyValueFile(package_dir / kPackageManifest);
  } catch (const std::exception& exc) {
    result.report.AddIssue({"package_manifest_invalid", {}, package_dir, exc.what()});
    return result;
  }
  const std::vector<std::string> required_manifest_keys = {
      "format", "map_id", "schema"};
  if (manifest.size() != required_manifest_keys.size() ||
      !std::all_of(required_manifest_keys.begin(), required_manifest_keys.end(),
                   [&](const std::string& key) { return manifest.count(key) == 1U; })) {
    result.report.AddIssue({
        "package_manifest_invalid", {}, package_dir,
        "package manifest fields do not match the required schema"});
    return result;
  }
  if (manifest.at("format") != "lingtu-map-package-v1") {
    result.report.AddIssue({"unsupported_package_format", {}, package_dir, "unsupported package format"});
    return result;
  }
  std::string source_id;
  try {
    source_id = MapStore::NormalizeMapId(manifest.at("map_id"));
  } catch (const std::exception& exc) {
    result.report.AddIssue({"package_manifest_invalid", {}, package_dir, exc.what()});
    return result;
  }
  std::string id;
  try {
    id = requested_map_id.empty()
        ? MapStore::NormalizeMapId(source_id)
        : MapStore::NormalizeMapId(requested_map_id);
  } catch (const std::exception& exc) {
    result.report.AddIssue({"invalid_map_id", requested_map_id.empty() ? source_id : requested_map_id, package_dir, exc.what()});
    return result;
  }
  const auto map_source = package_dir / "map";
  const auto manifest_schema = ParsePositiveInt(manifest.at("schema"));
  if (manifest_schema <= 0 ||
      manifest_schema != ReadSchema(map_source)) {
    result.report.AddIssue({
        "package_manifest_invalid", id, package_dir,
        "package schema is invalid or inconsistent"});
    return result;
  }
  if (!std::filesystem::is_directory(map_source) ||
      !std::filesystem::is_regular_file(map_source / "map.pcd")) {
    result.report.AddIssue({"map_artifact_missing", id, map_source, "map.pcd is missing"});
    return result;
  }
  const auto map_dir = MapPath(id);
  result.map_id = id;
  result.report.AddChange({
      MaintenanceAction::kImport,
      id,
      map_source,
      map_dir,
      "import verified map package"});
  if (dry_run) {
    return result;
  }
  const auto lock = MapLock::TryAcquire(root_dir_, id, "import-map-package");
  if (!lock.has_value()) {
    result.report.AddIssue({"lock_busy", id, package_dir, "map write lock is busy"});
    return result;
  }
  const auto stage = root_dir_ /
      (".import-staging-" + id + "-" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::remove_all(stage);
  CopyTreeChecked(map_source, stage, map_source);
  MapStore epoch_store(MapStoreConfig{root_dir_});
  WriteTextAtomic(
      stage / MapStore::ContentEpochFilename(),
      std::to_string(epoch_store.AllocateContentEpoch()) + "\n");
  ReplaceDirectoryRecoverably(stage, map_dir);
  return result;
}

std::vector<MapPackageMaintenance::MapRef> MapPackageMaintenance::ListMaps() const {
  std::vector<MapRef> out;
  if (!std::filesystem::is_directory(root_dir_)) {
    return out;
  }
  const auto active = ActiveMapId();
  for (const auto& map_entry : std::filesystem::directory_iterator(root_dir_)) {
    if (!map_entry.is_directory() || IsQuarantineOrLockDir(map_entry.path(), config_)) {
      continue;
    }
    const auto map_id = map_entry.path().filename().string();
    if (!MapStore::IsValidMapId(map_id)) {
      continue;
    }
    MapRef ref;
    ref.map_id = map_id;
    ref.dir = map_entry.path();
    ref.active = map_id == active;
    ref.protected_map = IsProtected(ref);
    out.push_back(ref);
  }
  return out;
}

std::string MapPackageMaintenance::ActiveMapId() const {
  MapStore store(MapStoreConfig{root_dir_, config_.active_state_filename});
  return store.ActiveMapId();
}

bool MapPackageMaintenance::IsProtected(const MapRef& ref) const {
  return ref.active || config_.protected_map_ids.count(ref.map_id) != 0U;
}

std::filesystem::path MapPackageMaintenance::MapPath(const std::string& map_id) const {
  return root_dir_ / MapStore::NormalizeMapId(map_id);
}

std::filesystem::path MapPackageMaintenance::QuarantinePath(const MapRef& ref) const {
  return root_dir_ / config_.quarantine_dir_name / ref.map_id;
}

}  // namespace lingtu::maps
