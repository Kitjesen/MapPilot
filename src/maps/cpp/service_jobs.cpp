#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <sstream>
#include <string>

#include "lingtu/maps/service.hpp"

namespace lingtu::maps {
namespace {

std::string JsonEscape(const std::string &value) {
  std::ostringstream stream;
  for (const unsigned char ch : value) {
    switch (ch) {
      case '"':
        stream << "\\\"";
        break;
      case '\\':
        stream << "\\\\";
        break;
      case '\b':
        stream << "\\b";
        break;
      case '\f':
        stream << "\\f";
        break;
      case '\n':
        stream << "\\n";
        break;
      case '\r':
        stream << "\\r";
        break;
      case '\t':
        stream << "\\t";
        break;
      default:
        if (ch < 0x20U) {
          constexpr char kHex[] = "0123456789abcdef";
          stream << "\\u00" << kHex[(ch >> 4U) & 0x0FU] << kHex[ch & 0x0FU];
        } else {
          stream << static_cast<char>(ch);
        }
    }
  }
  return stream.str();
}

std::string JsonString(const std::string &value) {
  return "\"" + JsonEscape(value) + "\"";
}

std::string BoolJson(bool value) {
  return value ? "true" : "false";
}

std::filesystem::path ExchangeRoot(const std::filesystem::path &map_root, const char *env_name,
                                   const char *leaf) {
  const char *configured = std::getenv(env_name);
  if (configured != nullptr && *configured != '\0') {
    return std::filesystem::absolute(configured).lexically_normal();
  }
  return std::filesystem::absolute(map_root / ".exchange" / leaf).lexically_normal();
}

std::filesystem::path BoundaryPath(const std::filesystem::path &path) {
  std::error_code ec;
  auto resolved = std::filesystem::weakly_canonical(path, ec);
  if (!ec)
    return resolved.lexically_normal();
  return std::filesystem::absolute(path).lexically_normal();
}

bool PathWithin(const std::filesystem::path &candidate, const std::filesystem::path &root) {
  const auto relative = BoundaryPath(candidate).lexically_relative(BoundaryPath(root));
  if (relative.empty() || relative.is_absolute() || relative == ".")
    return false;
  return std::none_of(relative.begin(), relative.end(),
                      [](const auto &part) { return part == ".."; });
}

}  // namespace

std::string MapsServiceCore::ImportPcdJson(const std::string &map_id,
                                           const std::filesystem::path &source_path,
                                           double voxel_size, const PcdBounds &bounds) {
  return pipeline_.ImportPcdJson(map_id, source_path, voxel_size, bounds);
}

std::string MapsServiceCore::CommitSavedSourceJson(const std::string &map_id,
                                                   const std::filesystem::path &source_dir,
                                                   const SourceCommitOptions &options) {
  return pipeline_.CommitSavedSourceJson(map_id, source_dir, options);
}

std::string MapsServiceCore::CropPcdJson(const std::string &map_id, const PcdBounds &bounds,
                                         bool invert, double voxel_size) {
  return pipeline_.CropPcdJson(map_id, bounds, invert, voxel_size);
}

std::string MapsServiceCore::BuildOccupancySnapshotJson(const std::string &map_id) {
  return pipeline_.BuildOccupancySnapshotJson(map_id);
}

std::string MapsServiceCore::BuildOctomapArtifactJson(const std::string &map_id,
                                                      const OctomapBuildOptions &options) {
  return pipeline_.BuildOctomapArtifactJson(map_id, options);
}

std::string MapsServiceCore::GetVoxelEditsJson(const std::string &map_id) const {
  return pipeline_.GetVoxelEditsJson(map_id);
}

std::string MapsServiceCore::EditOctomapVoxelsJson(const std::string &map_id,
                                                   const OctomapEditOptions &options) {
  return pipeline_.EditOctomapVoxelsJson(map_id, options);
}

std::string MapsServiceCore::BuildNavigationPackageJson(const std::string &map_id,
                                                        const OctomapBuildOptions &options,
                                                        bool include_esdf,
                                                        bool include_traversability) {
  return pipeline_.BuildNavigationPackageJson(map_id, options, include_esdf,
                                              include_traversability);
}

std::string MapsServiceCore::BuildEsdfArtifactJson(const std::string &map_id) {
  return pipeline_.BuildEsdfArtifactJson(map_id);
}

std::string MapsServiceCore::BuildTraversabilityArtifactJson(const std::string &map_id) {
  return pipeline_.BuildTraversabilityArtifactJson(map_id);
}

std::string MapsServiceCore::BuildSemanticArtifactJson(const std::string &map_id) {
  return pipeline_.BuildSemanticArtifactJson(map_id);
}

std::string MapsServiceCore::BeginSaveMapJson(const SaveMapRequest &request) {
  return SaveEngine().BeginJson(request);
}

std::string MapsServiceCore::ProvideSaveMapSnapshotJson(const std::string &job_id,
                                                        const MapSnapshot &snapshot) {
  return SaveEngine().ProvideSnapshotJson(job_id, snapshot);
}

std::string MapsServiceCore::RejectSaveMapSnapshotJson(const std::string &job_id,
                                                       const std::string &reason_code,
                                                       const std::string &message) {
  return SaveEngine().RejectSnapshotJson(job_id, reason_code, message);
}

std::string MapsServiceCore::GetSaveMapStatusJson(const std::string &job_id) const {
  return SaveEngine().GetStatusJson(job_id);
}

std::string MapsServiceCore::ListSaveMapJobsJson(std::size_t limit) const {
  return SaveEngine().ListStatusesJson(limit);
}

std::string MapsServiceCore::CancelSaveMapJson(const std::string &job_id) {
  return SaveEngine().CancelJson(job_id);
}

std::string MapsServiceCore::RetrySaveMapJson(const std::string &job_id) {
  return SaveEngine().RetryJson(job_id);
}

std::string MapsServiceCore::AuditMapsJson(bool dry_run) const {
  return MaintenanceReportJson("audit_maps", maintenance_.AuditMaps(dry_run));
}

std::string MapsServiceCore::QuarantineCorruptMapsJson(bool dry_run) {
  return MaintenanceReportJson("quarantine_corrupt_maps",
                               maintenance_.QuarantineCorruptMaps(dry_run));
}

std::string MapsServiceCore::ExportMapPackageJson(const std::string &map_id,
                                                  const std::filesystem::path &package_dir,
                                                  bool dry_run) const {
  const auto export_root = ExchangeRoot(store_.RootDir(), "LINGTU_MAP_EXPORT_DIR", "export");
  if (!PathWithin(package_dir, export_root)) {
    return FailureJson("export_map_package",
                       "package path escapes configured map export root: " + export_root.string(),
                       "unsafe_exchange_path");
  }
  return MaintenanceReportJson("export_map_package",
                               maintenance_.ExportMapPackage(map_id, package_dir, dry_run));
}

std::string MapsServiceCore::ImportMapPackageJson(const std::filesystem::path &package_dir,
                                                  const std::string &requested_map_id,
                                                  bool dry_run) {
  const auto import_root = ExchangeRoot(store_.RootDir(), "LINGTU_MAP_IMPORT_DIR", "import");
  if (!PathWithin(package_dir, import_root) || !std::filesystem::is_directory(package_dir)) {
    return FailureJson("import_map_package",
                       "package path must be an existing directory under: " + import_root.string(),
                       "unsafe_exchange_path");
  }
  const auto imported = maintenance_.ImportMapPackage(package_dir, requested_map_id, dry_run);
  std::string payload = MaintenanceReportJson("import_map_package", imported.report);
  if (payload.size() >= 1U && payload.back() == '}') {
    payload.pop_back();
    payload += ",\"map_id\":" + JsonString(imported.map_id) + "}";
  }
  return payload;
}

std::string MapsServiceCore::MaintenanceReportJson(const std::string &action,
                                                   const MaintenanceReport &report) const {
  std::ostringstream issues;
  issues << "[";
  for (std::size_t i = 0; i < report.issues.size(); ++i) {
    if (i != 0U)
      issues << ",";
    const auto &issue = report.issues[i];
    issues << "{"
           << "\"code\":" << JsonString(issue.code) << ","
           << "\"map_id\":" << JsonString(issue.map_id) << ","
           << "\"path\":" << JsonString(issue.path.string()) << ","
           << "\"message\":" << JsonString(issue.message) << "}";
  }
  issues << "]";
  std::ostringstream changes;
  changes << "[";
  for (std::size_t i = 0; i < report.changes.size(); ++i) {
    if (i != 0U)
      changes << ",";
    const auto &change = report.changes[i];
    changes << "{"
            << "\"map_id\":" << JsonString(change.map_id) << ","
            << "\"path\":" << JsonString(change.path.string()) << ","
            << "\"target_path\":" << JsonString(change.target_path.string()) << ","
            << "\"message\":" << JsonString(change.message) << "}";
  }
  changes << "]";
  return "{"
         "\"action\":" +
         JsonString(action) + "," + "\"success\":" + BoolJson(report.ok) + "," +
         "\"schema_version\":\"map.maintenance.v1\"," + "\"dry_run\":" + BoolJson(report.dry_run) +
         "," + "\"issue_count\":" + std::to_string(report.issues.size()) + "," +
         "\"change_count\":" + std::to_string(report.changes.size()) + "," +
         "\"issues\":" + issues.str() + "," + "\"changes\":" + changes.str() + "}";
}

}  // namespace lingtu::maps
