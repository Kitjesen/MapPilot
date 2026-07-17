#include "lingtu/maps/maintenance.hpp"

#include "lingtu/maps/hash.hpp"
#include "lingtu/maps/lock.hpp"
#include "lingtu/maps/store.hpp"
#include "lingtu/maps/version.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace lingtu::maps {
namespace {

constexpr const char* kVersionsDir = ".versions";
constexpr const char* kCurrentVersionFile = "current_version.txt";
constexpr const char* kPackageManifest = "package_manifest.txt";
constexpr const char* kPackageIndex = "file_index.tsv";
constexpr const char* kSchemaFile = "schema_version.txt";

std::string Trim(std::string value) {
  while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())) != 0) {
    value.pop_back();
  }
  const auto begin = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  });
  return std::string(begin, value.end());
}

std::string VersionName(std::int64_t version) {
  std::ostringstream out;
  out.width(20);
  out.fill('0');
  out << version;
  return out.str();
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

std::string ReadFirstLine(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  std::string line;
  std::getline(file, line);
  return Trim(line);
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
  std::filesystem::rename(tmp, path);
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

std::vector<std::pair<std::string, std::string>> HashTree(const std::filesystem::path& root) {
  std::vector<std::pair<std::string, std::string>> hashes;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
    if (IsSymlink(entry)) {
      throw std::runtime_error("symlink rejected in package tree: " + entry.path().string());
    }
    if (!entry.is_regular_file()) {
      continue;
    }
    const auto rel = std::filesystem::relative(entry.path(), root);
    if (!IsSafePackageRelativePath(rel)) {
      throw std::runtime_error("unsafe package path: " + rel.generic_string());
    }
    const auto rel_text = rel.generic_string();
    if (rel_text == kPackageIndex) {
      continue;
    }
    hashes.push_back({rel_text, Sha256File(entry.path())});
  }
  std::sort(hashes.begin(), hashes.end());
  return hashes;
}

std::map<std::string, std::string> ReadKeyValueFile(const std::filesystem::path& path) {
  std::map<std::string, std::string> out;
  std::ifstream file(path, std::ios::binary);
  std::string line;
  while (std::getline(file, line)) {
    const auto pos = line.find('=');
    if (pos == std::string::npos) {
      continue;
    }
    out[Trim(line.substr(0, pos))] = Trim(line.substr(pos + 1));
  }
  return out;
}

std::int64_t ReadSchema(const std::filesystem::path& version_dir) {
  const auto schema = ParsePositiveInt(ReadFirstLine(version_dir / kSchemaFile));
  return schema > 0 ? schema : 1;
}

void WritePackageIndex(const std::filesystem::path& package_dir) {
  const auto hashes = HashTree(package_dir);
  std::ostringstream out;
  for (const auto& item : hashes) {
    out << item.second << '\t' << item.first << '\n';
  }
  WriteTextAtomic(package_dir / kPackageIndex, out.str());
}

bool VerifyPackageIndex(const std::filesystem::path& package_dir, std::string* error) {
  std::ifstream file(package_dir / kPackageIndex, std::ios::binary);
  if (!file) {
    if (error != nullptr) *error = "package file index is missing";
    return false;
  }
  std::vector<std::pair<std::string, std::string>> indexed;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    const auto sep = line.find('\t');
    if (sep == std::string::npos) {
      if (error != nullptr) *error = "invalid package index entry";
      return false;
    }
    const auto hash = line.substr(0, sep);
    const auto rel_text = line.substr(sep + 1);
    const auto rel = std::filesystem::path(rel_text);
    if (rel_text.find('\t') != std::string::npos || rel_text.find('\n') != std::string::npos ||
        !IsSafePackageRelativePath(rel)) {
      if (error != nullptr) *error = "unsafe package index path: " + rel_text;
      return false;
    }
    const auto path = package_dir / rel;
    if (!std::filesystem::is_regular_file(path)) {
      if (error != nullptr) *error = "indexed package file is missing: " + rel_text;
      return false;
    }
    if (hash != Sha256File(path)) {
      if (error != nullptr) *error = "package hash mismatch: " + rel_text;
      return false;
    }
    indexed.push_back({rel.generic_string(), hash});
  }
  std::sort(indexed.begin(), indexed.end());
  auto actual = HashTree(package_dir);
  std::vector<std::pair<std::string, std::string>> actual_by_path;
  actual_by_path.reserve(actual.size());
  for (const auto& item : actual) {
    actual_by_path.push_back({item.first, item.second});
  }
  std::sort(actual_by_path.begin(), actual_by_path.end());
  if (indexed != actual_by_path) {
    if (error != nullptr) *error = "package file index does not match package contents";
    return false;
  }
  if (error != nullptr) error->clear();
  return true;
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

void MapPackageMaintenance::RegisterMigration(
    std::int64_t from_schema,
    std::int64_t to_schema,
    MigrationHook hook) {
  if (from_schema <= 0 || to_schema <= from_schema || !hook) {
    throw std::invalid_argument("invalid map schema migration hook");
  }
  migrations_[{from_schema, to_schema}] = std::move(hook);
}

MaintenanceReport MapPackageMaintenance::AuditImmutableVersions(bool dry_run) const {
  MaintenanceReport report;
  report.dry_run = dry_run;
  for (const auto& ref : ListVersions()) {
    std::string error;
    if (!VerifyMapVersion(ref.dir, &error)) {
      report.AddIssue({"version_integrity_failed", ref.map_id, ref.version, ref.dir, error});
    }
    const auto schema = ReadSchema(ref.dir);
    if (schema < config_.min_supported_schema) {
      report.AddIssue({"schema_too_old", ref.map_id, ref.version, ref.dir, "schema is older than supported minimum"});
    }
    if (schema > config_.max_supported_schema) {
      report.AddIssue({"schema_too_new", ref.map_id, ref.version, ref.dir, "schema is newer than supported maximum"});
    }
  }
  for (const auto& map_id : std::vector<std::string>{ActiveMapId()}) {
    if (map_id.empty()) {
      continue;
    }
    const auto map_dir = MapPath(map_id);
    const auto current = CurrentVersion(map_dir);
    if (current <= 0 || !std::filesystem::is_directory(map_dir / kVersionsDir / VersionName(current))) {
      report.AddIssue({"active_current_missing", map_id, current, map_dir, "active map has no verified current version"});
    }
  }
  return report;
}

MaintenanceReport MapPackageMaintenance::QuarantineCorruptVersions(bool dry_run) {
  MaintenanceReport report;
  report.dry_run = dry_run;
  for (const auto& ref : ListVersions()) {
    if (IsProtected(ref)) {
      continue;
    }
    std::string error;
    if (VerifyMapVersion(ref.dir, &error)) {
      continue;
    }
    const auto lock = MapLock::TryAcquire(root_dir_, ref.map_id, "quarantine-corrupt");
    if (!lock.has_value()) {
      report.AddIssue({"lock_busy", ref.map_id, ref.version, ref.dir, "map write lock is busy"});
      continue;
    }
    const auto target = QuarantinePath(ref);
    report.AddChange({MaintenanceAction::kQuarantine, ref.map_id, ref.version, ref.dir, target, error});
    if (!dry_run) {
      std::filesystem::create_directories(target.parent_path());
      std::filesystem::rename(ref.dir, target);
    }
  }
  return report;
}

MaintenanceReport MapPackageMaintenance::GarbageCollectVersions(bool dry_run) {
  MaintenanceReport report;
  report.dry_run = dry_run;
  std::map<std::string, std::vector<VersionRef>> by_map;
  for (const auto& ref : ListVersions()) {
    by_map[ref.map_id].push_back(ref);
  }
  for (auto& item : by_map) {
    auto& versions = item.second;
    std::sort(versions.begin(), versions.end(), [](const auto& lhs, const auto& rhs) {
      return lhs.version > rhs.version;
    });
    std::size_t retained = 0;
    for (const auto& ref : versions) {
      const bool keep_by_count = retained < config_.retain_latest_versions;
      if (keep_by_count) {
        ++retained;
      }
      if (keep_by_count || IsProtected(ref)) {
        continue;
      }
      const auto lock = MapLock::TryAcquire(root_dir_, ref.map_id, "gc-version");
      if (!lock.has_value()) {
        report.AddIssue({"lock_busy", ref.map_id, ref.version, ref.dir, "map write lock is busy"});
        continue;
      }
      report.AddChange({MaintenanceAction::kGarbageCollect, ref.map_id, ref.version, ref.dir, {}, "retention delete"});
      if (!dry_run) {
        std::filesystem::remove_all(ref.dir);
      }
    }
  }
  return report;
}

MaintenanceReport MapPackageMaintenance::ExportMapVersion(
    const std::string& map_id,
    std::int64_t version,
    const std::filesystem::path& package_dir,
    bool dry_run) const {
  MaintenanceReport report;
  report.dry_run = dry_run;
  std::string id;
  try {
    id = MapStore::NormalizeMapId(map_id);
  } catch (const std::exception& exc) {
    report.AddIssue({"invalid_map_id", map_id, version, {}, exc.what()});
    return report;
  }
  if (version <= 0) {
    report.AddIssue({"invalid_version", id, version, {}, "version must be positive"});
    return report;
  }
  if (package_dir.empty()) {
    report.AddIssue({"invalid_package_path", id, version, package_dir, "package path is required"});
    return report;
  }
  const auto source = MapPath(id) / kVersionsDir / VersionName(version);
  std::string integrity_error;
  if (!VerifyMapVersion(source, &integrity_error)) {
    report.AddIssue({"version_integrity_failed", id, version, source, integrity_error});
    return report;
  }
  report.AddChange({MaintenanceAction::kExport, id, version, source, package_dir, "export verified map package"});
  if (dry_run) {
    return report;
  }
  const auto stage = package_dir.parent_path() /
      (package_dir.filename().string() + ".staging-" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::remove_all(stage);
  std::filesystem::create_directories(stage / "version");
  CopyTreeChecked(source, stage / "version", source);
  WriteTextAtomic(
      stage / kPackageManifest,
      "format=lingtu-map-package-v1\nmap_id=" + id + "\nversion=" +
          std::to_string(version) + "\nschema=" + std::to_string(ReadSchema(source)) + "\n");
  WritePackageIndex(stage);
  std::filesystem::remove_all(package_dir);
  std::filesystem::rename(stage, package_dir);
  return report;
}

PackageImportResult MapPackageMaintenance::ImportMapPackage(
    const std::filesystem::path& package_dir,
    const std::string& requested_map_id,
    bool dry_run) {
  PackageImportResult result;
  result.report.dry_run = dry_run;
  std::string package_error;
  if (!VerifyPackageIndex(package_dir, &package_error)) {
    result.report.AddIssue({"package_integrity_failed", {}, 0, package_dir, package_error});
    return result;
  }
  const auto manifest = ReadKeyValueFile(package_dir / kPackageManifest);
  if (manifest.find("format") == manifest.end() ||
      manifest.at("format") != "lingtu-map-package-v1") {
    result.report.AddIssue({"unsupported_package_format", {}, 0, package_dir, "unsupported package format"});
    return result;
  }
  const std::string source_id = manifest.count("map_id") ? manifest.at("map_id") : "";
  std::string id;
  try {
    id = requested_map_id.empty()
        ? MapStore::NormalizeMapId(source_id)
        : MapStore::NormalizeMapId(requested_map_id);
  } catch (const std::exception& exc) {
    result.report.AddIssue({"invalid_map_id", requested_map_id.empty() ? source_id : requested_map_id, 0, package_dir, exc.what()});
    return result;
  }
  const auto source_version = ParsePositiveInt(manifest.count("version") ? manifest.at("version") : "");
  const auto version_source = package_dir / "version";
  std::string integrity_error;
  if (!VerifyMapVersion(version_source, &integrity_error)) {
    result.report.AddIssue({"version_integrity_failed", id, source_version, version_source, integrity_error});
    return result;
  }
  const auto lock = MapLock::TryAcquire(root_dir_, id, "import-map-package");
  if (!lock.has_value()) {
    result.report.AddIssue({"lock_busy", id, source_version, package_dir, "map write lock is busy"});
    return result;
  }
  const auto map_dir = MapPath(id);
  const auto versions_dir = map_dir / kVersionsDir;
  std::filesystem::create_directories(versions_dir);
  std::int64_t version = source_version;
  while (version <= 0 || std::filesystem::exists(versions_dir / VersionName(version))) {
    ++version;
  }
  const auto target = versions_dir / VersionName(version);
  result.map_id = id;
  result.version = version;
  result.report.AddChange({MaintenanceAction::kImport, id, version, version_source, target, "import verified map package"});
  if (dry_run) {
    return result;
  }
  const auto stage = versions_dir / (".import-staging-" + std::to_string(version));
  std::filesystem::remove_all(stage);
  CopyTreeChecked(version_source, stage, version_source);
  if (!VerifyMapVersion(stage, &integrity_error)) {
    std::filesystem::remove_all(stage);
    result.report.AddIssue({"version_integrity_failed", id, version, stage, integrity_error});
    return result;
  }
  std::filesystem::rename(stage, target);
  if (CurrentVersion(map_dir) <= 0) {
    WriteTextAtomic(map_dir / kCurrentVersionFile, VersionName(version) + "\n");
  }
  return result;
}

MaintenanceReport MapPackageMaintenance::MigrateSchemas(bool dry_run) {
  MaintenanceReport report;
  report.dry_run = dry_run;
  for (const auto& ref : ListVersions()) {
    const auto schema = ReadSchema(ref.dir);
    if (schema >= config_.min_supported_schema) {
      continue;
    }
    const auto migration = migrations_.find({schema, config_.min_supported_schema});
    if (migration == migrations_.end()) {
      report.AddIssue({"missing_migration", ref.map_id, ref.version, ref.dir, "no schema migration hook registered"});
      continue;
    }
    if (IsProtected(ref)) {
      report.AddIssue({"protected_version_migration_blocked", ref.map_id, ref.version, ref.dir, "protected/current versions are not modified in place"});
      continue;
    }
    const auto lock = MapLock::TryAcquire(root_dir_, ref.map_id, "migrate-schema");
    if (!lock.has_value()) {
      report.AddIssue({"lock_busy", ref.map_id, ref.version, ref.dir, "map write lock is busy"});
      continue;
    }
    report.AddChange({MaintenanceAction::kMigrate, ref.map_id, ref.version, ref.dir, ref.dir, "schema migration"});
    if (!dry_run) {
      std::string error;
      if (!migration->second(ref.dir, schema, config_.min_supported_schema, &error)) {
        report.AddIssue({"migration_failed", ref.map_id, ref.version, ref.dir, error});
      }
    }
  }
  return report;
}

std::vector<MapPackageMaintenance::VersionRef> MapPackageMaintenance::ListVersions() const {
  std::vector<VersionRef> out;
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
    const auto map_dir = map_entry.path();
    const auto current = CurrentVersion(map_dir);
    const auto versions_dir = map_dir / kVersionsDir;
    if (!std::filesystem::is_directory(versions_dir)) {
      continue;
    }
    for (const auto& version_entry : std::filesystem::directory_iterator(versions_dir)) {
      if (!version_entry.is_directory()) {
        continue;
      }
      const auto version = ParsePositiveInt(version_entry.path().filename().string());
      if (version <= 0) {
        continue;
      }
      VersionRef ref;
      ref.map_id = map_id;
      ref.version = version;
      ref.dir = version_entry.path();
      ref.current = (map_id == active && version == current) || version == current;
      ref.protected_version = IsProtected(ref);
      out.push_back(ref);
    }
  }
  return out;
}

std::string MapPackageMaintenance::ActiveMapId() const {
  const auto active = ReadFirstLine(root_dir_ / config_.active_state_filename);
  return MapStore::IsValidMapId(active) ? active : std::string{};
}

std::int64_t MapPackageMaintenance::CurrentVersion(const std::filesystem::path& map_dir) const {
  return ParsePositiveInt(ReadFirstLine(map_dir / kCurrentVersionFile));
}

bool MapPackageMaintenance::IsProtected(const VersionRef& ref) const {
  if (ref.current || config_.protected_map_ids.count(ref.map_id) != 0U) {
    return true;
  }
  const auto found = config_.protected_versions.find(ref.map_id);
  return found != config_.protected_versions.end() && found->second.count(ref.version) != 0U;
}

std::filesystem::path MapPackageMaintenance::MapPath(const std::string& map_id) const {
  return root_dir_ / MapStore::NormalizeMapId(map_id);
}

std::filesystem::path MapPackageMaintenance::QuarantinePath(const VersionRef& ref) const {
  return root_dir_ / config_.quarantine_dir_name / ref.map_id / VersionName(ref.version);
}

}  // namespace lingtu::maps
