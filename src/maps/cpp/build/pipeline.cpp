#include "lingtu/maps/build/pipeline.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "lingtu/maps/build/grid_artifacts.hpp"
#include "lingtu/maps/build/occupancy_snapshot.hpp"
#include "lingtu/maps/build/process.hpp"
#include "lingtu/maps/json.hpp"
#include "lingtu/maps/lock.hpp"
#include "lingtu/maps/semantic_map_persistence.hpp"

#if defined(_WIN32)
#define NOMINMAX
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

#if defined(LINGTU_MAPS_HAS_OCTOMAP)
#include <octomap/OcTree.h>
#endif

namespace lingtu::maps {
namespace {

std::uint64_t CurrentProcessIdValue() {
#if defined(_WIN32)
  return static_cast<std::uint64_t>(GetCurrentProcessId());
#else
  return static_cast<std::uint64_t>(getpid());
#endif
}

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

std::string NowStamp() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

std::string SanitizedArtifactType(const std::string &value) {
  std::string out;
  for (const unsigned char ch : value) {
    if (std::isalnum(ch) != 0 || ch == '_' || ch == '-') {
      out.push_back(static_cast<char>(ch));
    } else if (ch == '.' || ch == ':' || ch == '/') {
      out.push_back('_');
    }
  }
  return out.empty() ? "artifact" : out;
}

std::string FirstLine(const std::string &text) {
  const auto pos = text.find('\n');
  return pos == std::string::npos ? text : text.substr(0, pos);
}

std::string Lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

int PcdPointCount(const std::filesystem::path &path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return 0;
  }
  std::string line;
  bool data_section = false;
  int counted_rows = 0;
  while (std::getline(file, line)) {
    const std::string trimmed = line;
    const std::string lower = Lower(trimmed);
    if (lower.rfind("points", 0) == 0) {
      std::istringstream stream(trimmed.substr(6));
      int value = 0;
      if (stream >> value) {
        return std::max(0, value);
      }
    }
    if (data_section && !trimmed.empty()) {
      ++counted_rows;
    }
    if (lower.rfind("data", 0) == 0) {
      data_section = true;
    }
  }
  return counted_rows;
}

std::string EnvValue(const char *name) {
  const char *value = std::getenv(name);
  return value == nullptr ? std::string{} : std::string(value);
}

std::string FirstNonEmpty(const std::string &first, const std::string &second,
                          const std::string &third, const std::string &fallback) {
  if (!first.empty()) {
    return first;
  }
  if (!second.empty()) {
    return second;
  }
  if (!third.empty()) {
    return third;
  }
  return fallback;
}

std::string NativeOctomapEmbeddedJson() {
#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  return "true";
#else
  return "false";
#endif
}

std::string SupportedOctomapBuildModesJson() {
#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  return "[\"native_octomap\",\"external_pcl_converter\"]";
#else
  return "[\"external_pcl_converter\"]";
#endif
}

std::string ResolveConverterCommand(const OctomapBuildOptions &options) {
  if (!options.converter_command.empty()) {
    return options.converter_command;
  }
  for (const char *env_name : {
           "LINGTU_MAP_ARTIFACT_CONVERTER",
           "LINGTU_OCTOPLANNER3D_PCD_CONVERTER",
           "LINGTU_OCTOMAP_CONVERTER",
       }) {
    const std::string value = EnvValue(env_name);
    if (!value.empty()) {
      return value;
    }
  }
  return {};
}

std::string ShellQuote(const std::string &value) {
#if defined(_WIN32)
  std::string out = "\"";
  for (const char ch : value) {
    if (ch == '"') {
      out += "\\\"";
    } else {
      out.push_back(ch);
    }
  }
  out += "\"";
  return out;
#else
  std::string out = "'";
  for (const char ch : value) {
    if (ch == '\'') {
      out += "'\\''";
    } else {
      out.push_back(ch);
    }
  }
  out += "'";
  return out;
#endif
}

std::string PosixShellQuote(const std::string &value) {
  std::string out = "'";
  for (const char ch : value) {
    if (ch == '\'') {
      out += "'\\''";
    } else {
      out.push_back(ch);
    }
  }
  out += "'";
  return out;
}

std::string WslPath(const std::filesystem::path &path) {
  std::string value = std::filesystem::absolute(path).string();
  std::replace(value.begin(), value.end(), '\\', '/');
#if defined(_WIN32)
  if (value.size() >= 3U && value[1] == ':' && value[2] == '/') {
    const char drive = static_cast<char>(
        std::tolower(static_cast<unsigned char>(value.front())));
    value = "/mnt/" + std::string(1U, drive) + "/" + value.substr(3U);
  }
#endif
  return value;
}

void ReplaceAll(std::string &value, const std::string &needle, const std::string &replacement) {
  if (needle.empty()) {
    return;
  }
  std::size_t pos = 0;
  while ((pos = value.find(needle, pos)) != std::string::npos) {
    value.replace(pos, needle.size(), replacement);
    pos += replacement.size();
  }
}

std::string NormalizeShellCommandForSystem(std::string command) {
#if defined(_WIN32)
  if (!command.empty() && command.front() == '"') {
    return "\"" + command + "\"";
  }
#endif
  return command;
}

std::string BuildConverterShellCommand(std::string command, const std::filesystem::path &pcd_path,
                                       const std::filesystem::path &octomap_path,
                                       const std::filesystem::path &map_dir,
                                       const OctomapBuildOptions &options) {
  const bool has_placeholder =
      command.find('{') != std::string::npos && command.find('}') != std::string::npos;
  if (has_placeholder) {
    ReplaceAll(command, "{input_wsl}", PosixShellQuote(WslPath(pcd_path)));
    ReplaceAll(command, "{output_wsl}", PosixShellQuote(WslPath(octomap_path)));
    ReplaceAll(command, "{map_dir_wsl}", PosixShellQuote(WslPath(map_dir)));
    ReplaceAll(command, "{frame_wsl}", PosixShellQuote(options.frame_id));
    ReplaceAll(command, "{input}", ShellQuote(std::filesystem::absolute(pcd_path).string()));
    ReplaceAll(command, "{output}", ShellQuote(std::filesystem::absolute(octomap_path).string()));
    ReplaceAll(command, "{map_dir}", ShellQuote(std::filesystem::absolute(map_dir).string()));
    ReplaceAll(command, "{resolution}", std::to_string(options.resolution));
    ReplaceAll(command, "{support_dilation_cells}",
               std::to_string(std::max(0, options.support_dilation_cells)));
    ReplaceAll(command, "{free_layers_above}",
               std::to_string(std::max(0, options.free_layers_above)));
    ReplaceAll(command, "{free_dilation_cells}",
               std::to_string(std::max(0, options.free_dilation_cells)));
    ReplaceAll(command, "{frame}", ShellQuote(options.frame_id));
    return NormalizeShellCommandForSystem(command);
  }

  std::ostringstream stream;
  stream << command << " --input " << ShellQuote(std::filesystem::absolute(pcd_path).string())
         << " --output " << ShellQuote(std::filesystem::absolute(octomap_path).string())
         << " --resolution " << options.resolution << " --support-dilation-cells "
         << std::max(0, options.support_dilation_cells) << " --free-layers-above "
         << std::max(0, options.free_layers_above) << " --free-dilation-cells "
         << std::max(0, options.free_dilation_cells) << " --frame " << ShellQuote(options.frame_id);
  return NormalizeShellCommandForSystem(stream.str());
}

std::string MetadataJson(const std::string &map_id, const std::filesystem::path &map_dir,
                         const std::filesystem::path &pcd_path,
                         const std::filesystem::path &octomap_path,
                         const OctomapBuildOptions &options,
                         bool manual_voxel_edit = false, std::size_t manual_edit_count = 0U,
                         const std::string &last_edit_json = "null") {
  const std::filesystem::path occupancy_path = map_dir / "occupancy.npz";
  const bool has_occupancy = std::filesystem::is_regular_file(occupancy_path);
  const std::string source_profile =
      FirstNonEmpty(options.source_profile, EnvValue("LINGTU_PROFILE"), "", "maps_pipeline");
  const std::string data_source = FirstNonEmpty(
      options.data_source, EnvValue("LINGTU_RUNTIME_DATA_SOURCE"), source_profile, source_profile);
  const std::string slam_source = options.slam_source.empty() ? "unknown" : options.slam_source;
  const std::string localization_source =
      options.localization_source.empty() ? slam_source : options.localization_source;
  const std::string mapping_source =
      options.mapping_source.empty() ? "lingtu_maps_pipeline" : options.mapping_source;
  const int point_count = PcdPointCount(pcd_path);

  std::ostringstream artifacts;
  artifacts << "\"map_pcd\":{"
            << "\"path\":\"map.pcd\","
            << "\"source_profile\":" << JsonString(source_profile) << ","
            << "\"data_source\":" << JsonString(data_source) << ","
            << "\"slam_source\":" << JsonString(slam_source) << ","
            << "\"frame_id\":" << JsonString(options.frame_id) << ","
            << "\"point_count\":" << point_count << "},";
  if (has_occupancy) {
    artifacts << "\"occupancy_grid\":{"
              << "\"path\":\"occupancy.npz\","
              << "\"source_profile\":" << JsonString(source_profile) << ","
              << "\"data_source\":" << JsonString(data_source) << ","
              << "\"frame_id\":" << JsonString(options.frame_id) << "},";
  }
  artifacts << "\"octomap\":{"
            << "\"path\":" << JsonString(octomap_path.filename().string()) << ","
            << "\"source_profile\":" << JsonString(source_profile) << ","
            << "\"data_source\":" << JsonString(data_source) << ","
            << "\"frame_id\":" << JsonString(options.frame_id) << ","
            << "\"resolution\":" << options.resolution << ","
            << "\"support_dilation_cells\":" << std::max(0, options.support_dilation_cells) << ","
            << "\"free_layers_above\":" << std::max(0, options.free_layers_above) << ","
            << "\"free_dilation_cells\":" << std::max(0, options.free_dilation_cells) << ","
            << "\"build_mode\":" << JsonString(options.build_mode) << ","
            << (manual_voxel_edit ? "\"manual_voxel_edit\":true," : "")
            << "\"builder\":{\"name\":\"LingTu MapsPipelineCore\",\"version\":\"0.2.0\"}"
            << "}";

  const std::string manual_edit_summary = manual_voxel_edit
      ? "\"manual_voxel_edits\":{\"count\":" + std::to_string(manual_edit_count) +
            ",\"journal\":\"voxel_edits.jsonl\",\"last_edit\":" + last_edit_json + "},"
      : std::string{};

  return "{"
         "\"schema_version\":\"lingtu.saved_map_artifacts.v1\","
         "\"source_profile\":" +
         JsonString(source_profile) +
         ","
         "\"data_source\":" +
         JsonString(data_source) +
         ","
         "\"slam_source\":" +
         JsonString(slam_source) +
         ","
         "\"localization_source\":" +
         JsonString(localization_source) +
         ","
         "\"mapping_source\":" +
         JsonString(mapping_source) +
         ","
         "\"frame_id\":" +
         JsonString(options.frame_id) +
         ","
         "\"created_at\":" +
         JsonString(NowStamp()) +
         ","
         "\"artifacts\":{" +
         artifacts.str() +
         "},"
         + manual_edit_summary +
         "\"map_name\":" +
         JsonString(map_id) +
         ","
         "\"source\":\"lingtu_maps_pipeline\","
         "\"build_mode\":" +
         JsonString(options.build_mode) +
         ","
         "\"supported_build_modes\":" +
         SupportedOctomapBuildModesJson() +
         ","
         "\"resolution\":" +
         std::to_string(options.resolution) +
         ","
         "\"support_dilation_cells\":" +
         std::to_string(std::max(0, options.support_dilation_cells)) +
         ","
         "\"free_layers_above\":" +
         std::to_string(std::max(0, options.free_layers_above)) +
         ","
         "\"free_dilation_cells\":" +
         std::to_string(std::max(0, options.free_dilation_cells)) +
         ","
         "\"frame\":" +
         JsonString(options.frame_id) +
         ","
         "\"builder\":{\"name\":\"LingTu MapsPipelineCore\",\"version\":\"0.2.0\"},"
         "\"builder_version\":\"0.2.0\","
         "\"octomap\":{\"path\":" + JsonString(octomap_path.filename().string()) + "}"
         "}\n";
}

bool ExistingMetadataAllowsReuse(const std::filesystem::path &metadata_path) {
  if (!std::filesystem::is_regular_file(metadata_path)) {
    return false;
  }
  std::error_code size_error;
  const auto metadata_size = std::filesystem::file_size(metadata_path, size_error);
  if (size_error || metadata_size > 4U * 1024U * 1024U) {
    return false;
  }
  std::ifstream file(metadata_path, std::ios::binary);
  if (!file) {
    return false;
  }
  std::ostringstream stream;
  stream << file.rdbuf();
  if (file.bad()) {
    return false;
  }
  const std::string text = stream.str();
  return JsonObjectStringAtPath(text, {"schema_version"}) ==
             "lingtu.saved_map_artifacts.v1" &&
         JsonObjectBoolAtPath(text, {"invalidated"}) != true &&
         JsonObjectStringAtPath(text, {"octomap", "path"}) == "octomap.ot";
}

bool JsonSucceeded(const std::string &json) {
  return JsonObjectBoolAtPath(json, {"success"}) == true;
}

struct TransactionArtifactBackup {
  std::string filename;
  std::filesystem::path final_path;
  std::filesystem::path backup_path;
  bool existed{false};
};

enum class TransactionPhase {
  kPrepared,
  kPublishing,
  kCommitted,
};

struct PersistentTransaction {
  std::string map_id;
  TransactionPhase phase{TransactionPhase::kPrepared};
  std::int64_t base_epoch{0};
  std::int64_t next_epoch{0};
  std::vector<TransactionArtifactBackup> artifacts;
};

bool WriteTransactionManifest(
    const std::filesystem::path &transaction_dir,
    const PersistentTransaction &transaction,
    std::string *error);

bool ReadTransactionManifest(
    const std::filesystem::path &transaction_dir,
    const std::filesystem::path &map_dir,
    PersistentTransaction *transaction,
    std::string *error);

bool CopyPathRecursive(const std::filesystem::path &from, const std::filesystem::path &to,
                       std::string *error) {
  std::error_code ec;
  std::filesystem::remove_all(to, ec);
  ec.clear();
  std::filesystem::create_directories(to.parent_path(), ec);
  if (ec) {
    if (error != nullptr) {
      *error = "failed to create destination directory: " + ec.message();
    }
    return false;
  }
  ec.clear();
  if (std::filesystem::is_directory(from)) {
    std::filesystem::copy(from, to,
                          std::filesystem::copy_options::recursive |
                              std::filesystem::copy_options::overwrite_existing,
                          ec);
  } else {
    std::filesystem::copy_file(from, to, std::filesystem::copy_options::overwrite_existing, ec);
  }
  if (ec) {
    if (error != nullptr) {
      *error = "failed to copy " + from.string() + ": " + ec.message();
    }
    return false;
  }
  return true;
}

std::vector<std::string> NavigationPackageArtifactNames(bool include_esdf,
                                                        bool include_traversability) {
  std::vector<std::string> names{
      "occupancy.npz", "map.pgm", "map.yaml", "octomap.ot", "octomap.bt", "metadata.json",
  };
  if (include_esdf || include_traversability) {
    names.push_back("esdf.npz");
  }
  if (include_traversability) {
    names.push_back("traversability.npz");
  }
  return names;
}

std::vector<std::string> OccupancySnapshotArtifactNames() {
  return {"occupancy.npz", "map.pgm", "map.yaml"};
}

std::vector<std::string> OctomapArtifactNames() {
  return {"octomap.ot", "octomap.bt", "metadata.json"};
}

std::vector<std::string> OctomapEditArtifactNames() {
  return {"octomap.ot", "octomap.bt", "metadata.json", "voxel_edits.jsonl"};
}

std::vector<std::string> EsdfArtifactNames() {
  return {"esdf.npz"};
}

std::vector<std::string> TraversabilityArtifactNames() {
  return {"esdf.npz", "traversability.npz"};
}

std::vector<std::string> SemanticArtifactNames() {
  return {kSemanticMapArtifactFilename};
}

std::vector<std::string> SourceMapMutationArtifactNames() {
  return {
      "map.pcd",       "occupancy.npz",      "map.pgm",
      "map.yaml",      "octomap.ot",         "octomap.bt",
      "metadata.json", "voxel_edits.json",
      "voxel_edits.jsonl",
      "esdf.npz",      "traversability.npz",
  };
}

std::vector<std::string> SavedSourceArtifactNames() {
  auto names = SourceMapMutationArtifactNames();
  names.push_back("poses.txt");
  names.push_back("trajectory.txt");
  names.push_back("patches");
  names.push_back("patch_bundle.manifest");
  names.push_back("map.clean.pcd");
  names.push_back("map.removed.pcd");
  names.push_back("map_optimization.json");
  return names;
}

std::vector<TransactionArtifactBackup>
BackupNamedArtifacts(MapStore &store, const std::string &map_id,
                     const std::filesystem::path &map_dir,
                     const std::filesystem::path &transaction_dir,
                     const std::vector<std::string> &names) {
  std::vector<TransactionArtifactBackup> backups;
  const auto backup_dir = transaction_dir / "backup";
  std::filesystem::create_directories(backup_dir);
  std::vector<std::string> transaction_names = names;
  if (std::find(transaction_names.begin(), transaction_names.end(),
                MapStore::ContentEpochFilename()) == transaction_names.end()) {
    transaction_names.push_back(MapStore::ContentEpochFilename());
  }
  for (const auto &filename : transaction_names) {
    TransactionArtifactBackup backup;
    backup.filename = filename;
    backup.final_path = map_dir / filename;
    backup.backup_path = backup_dir / filename;
    backup.existed = std::filesystem::exists(backup.final_path);
    if (backup.existed) {
      std::string error;
      if (!CopyPathRecursive(backup.final_path, backup.backup_path, &error)) {
        throw std::runtime_error(error);
      }
    }
    backups.push_back(std::move(backup));
  }
  PersistentTransaction transaction;
  transaction.map_id = map_id;
  transaction.phase = TransactionPhase::kPrepared;
  transaction.base_epoch = store.ContentEpoch(map_id);
  if (transaction.base_epoch <= 0) {
    throw std::runtime_error("map content epoch is invalid: " + map_id);
  }
  transaction.artifacts = backups;
  std::string manifest_error;
  if (!WriteTransactionManifest(transaction_dir, transaction, &manifest_error)) {
    throw std::runtime_error(manifest_error);
  }
  return backups;
}

std::vector<TransactionArtifactBackup>
BackupTransactionArtifacts(MapStore &store, const std::string &map_id,
                           const std::filesystem::path &map_dir,
                           const std::filesystem::path &transaction_dir, bool include_esdf,
                           bool include_traversability) {
  return BackupNamedArtifacts(store, map_id, map_dir, transaction_dir,
                              NavigationPackageArtifactNames(include_esdf, include_traversability));
}

bool RollbackTransactionArtifacts(const std::vector<TransactionArtifactBackup> &backups) {
  std::error_code ec;
  bool ok = true;
  for (const auto &backup : backups) {
    std::filesystem::remove_all(backup.final_path, ec);
    if (ec) {
      ok = false;
    }
    ec.clear();
    if (backup.existed && !std::filesystem::exists(backup.backup_path)) {
      ok = false;
      continue;
    }
    if (backup.existed) {
      std::string error;
      if (!CopyPathRecursive(backup.backup_path, backup.final_path, &error)) {
        ok = false;
      }
    }
  }
  return ok;
}

bool RollbackTransactionArtifactsVerified(
    MapStore &store, const std::string &map_id,
    const std::vector<TransactionArtifactBackup> &backups,
    std::int64_t base_epoch, std::string *error) {
  const bool inject_failure =
      std::getenv("LINGTU_MAPS_INJECT_ROLLBACK_FAILURE") != nullptr;
  const bool complete = !inject_failure &&
      RollbackTransactionArtifacts(backups) &&
      store.ContentEpoch(map_id) == base_epoch;
  if (!complete && error != nullptr) {
    *error = "rollback incomplete: " + *error;
  }
  return complete;
}

bool TransactionNeedsRecovery(const std::string &error) {
  return error.rfind("rollback incomplete:", 0U) == 0U;
}

std::string TransactionRolledBackJson(const std::string &error) {
  return TransactionNeedsRecovery(error) ? "false" : "true";
}

std::string ArtifactPathJson(const std::filesystem::path &map_dir, const std::string &filename) {
  const auto path = map_dir / filename;
  return std::filesystem::is_regular_file(path) ? JsonString(path.string()) : "null";
}

bool WriteTextFile(const std::filesystem::path &path, const std::string &text) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  if (!file) {
    return false;
  }
  file.write(text.data(), static_cast<std::streamsize>(text.size()));
  file.flush();
  return file.good();
}

const char *TransactionPhaseName(TransactionPhase phase) {
  switch (phase) {
    case TransactionPhase::kPrepared:
      return "PREPARED";
    case TransactionPhase::kPublishing:
      return "PUBLISHING";
    case TransactionPhase::kCommitted:
      return "COMMITTED";
  }
  return "PREPARED";
}

bool IsSafeTransactionArtifactName(const std::string &filename) {
  return !filename.empty() && filename != "." && filename != ".." &&
      filename.find('/') == std::string::npos &&
      filename.find('\\') == std::string::npos &&
      filename.find('\t') == std::string::npos &&
      filename.find('\n') == std::string::npos &&
      (filename.front() != '.' || filename == MapStore::ContentEpochFilename());
}

bool WriteTransactionManifest(
    const std::filesystem::path &transaction_dir,
    const PersistentTransaction &transaction,
    std::string *error) {
  std::ostringstream out;
  out << "LINGTU_MAP_TRANSACTION\t1\n"
      << "map_id\t" << transaction.map_id << "\n"
      << "phase\t" << TransactionPhaseName(transaction.phase) << "\n"
      << "base_epoch\t" << transaction.base_epoch << "\n"
      << "next_epoch\t" << transaction.next_epoch << "\n";
  for (const auto &artifact : transaction.artifacts) {
    if (!IsSafeTransactionArtifactName(artifact.filename)) {
      if (error != nullptr) {
        *error = "transaction contains an unsafe artifact name";
      }
      return false;
    }
    out << "artifact\t" << artifact.filename << '\t'
        << (artifact.existed ? '1' : '0') << "\n";
  }

  const auto path = transaction_dir / "transaction.state";
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto temp = transaction_dir /
      ("transaction.state.tmp-" + std::to_string(CurrentProcessIdValue()) + "-" +
       std::to_string(stamp));
  if (!WriteTextFile(temp, out.str())) {
    if (error != nullptr) {
      *error = "failed to write map transaction manifest";
    }
    return false;
  }
  bool replaced = false;
#if defined(_WIN32)
  replaced = MoveFileExW(
      temp.wstring().c_str(), path.wstring().c_str(),
      MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0;
#else
  replaced = ::rename(temp.c_str(), path.c_str()) == 0;
#endif
  if (!replaced) {
    std::error_code ignored;
    std::filesystem::remove(temp, ignored);
    if (error != nullptr) {
      *error = "failed to publish map transaction manifest";
    }
    return false;
  }
  return true;
}

bool ParsePositiveEpoch(const std::string &text, std::int64_t *value) {
  if (value == nullptr || text.empty()) {
    return false;
  }
  std::int64_t parsed_value = 0;
  const auto parsed = std::from_chars(
      text.data(), text.data() + text.size(), parsed_value);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
      parsed_value <= 0 || parsed_value > 9'007'199'254'740'991LL) {
    return false;
  }
  *value = parsed_value;
  return true;
}

std::vector<std::string> SplitTabs(const std::string &line) {
  std::vector<std::string> fields;
  std::size_t start = 0U;
  for (;;) {
    const auto delimiter = line.find('\t', start);
    fields.push_back(line.substr(start, delimiter - start));
    if (delimiter == std::string::npos) {
      return fields;
    }
    start = delimiter + 1U;
  }
}

bool ReadTransactionManifest(
    const std::filesystem::path &transaction_dir,
    const std::filesystem::path &map_dir,
    PersistentTransaction *transaction,
    std::string *error) {
  if (transaction == nullptr) {
    return false;
  }
  std::ifstream file(transaction_dir / "transaction.state", std::ios::binary);
  if (!file) {
    if (error != nullptr) {
      *error = "map transaction manifest is missing";
    }
    return false;
  }
  std::string line;
  if (!std::getline(file, line) || line != "LINGTU_MAP_TRANSACTION\t1") {
    if (error != nullptr) {
      *error = "map transaction manifest header is invalid";
    }
    return false;
  }

  PersistentTransaction parsed;
  bool have_map_id = false;
  bool have_phase = false;
  bool have_base_epoch = false;
  bool have_next_epoch = false;
  std::unordered_set<std::string> artifact_names;
  while (std::getline(file, line)) {
    const auto fields = SplitTabs(line);
    if (fields.size() == 2U && fields[0] == "map_id" && !have_map_id) {
      parsed.map_id = fields[1];
      have_map_id = true;
    } else if (fields.size() == 2U && fields[0] == "phase" && !have_phase) {
      if (fields[1] == "PREPARED") {
        parsed.phase = TransactionPhase::kPrepared;
      } else if (fields[1] == "PUBLISHING") {
        parsed.phase = TransactionPhase::kPublishing;
      } else if (fields[1] == "COMMITTED") {
        parsed.phase = TransactionPhase::kCommitted;
      } else {
        if (error != nullptr) {
          *error = "map transaction phase is invalid";
        }
        return false;
      }
      have_phase = true;
    } else if (fields.size() == 2U && fields[0] == "base_epoch" &&
               !have_base_epoch) {
      have_base_epoch = ParsePositiveEpoch(fields[1], &parsed.base_epoch);
      if (!have_base_epoch) {
        if (error != nullptr) {
          *error = "map transaction base epoch is invalid";
        }
        return false;
      }
    } else if (fields.size() == 2U && fields[0] == "next_epoch" &&
               !have_next_epoch) {
      if (fields[1] == "0") {
        parsed.next_epoch = 0;
        have_next_epoch = true;
      } else {
        have_next_epoch = ParsePositiveEpoch(fields[1], &parsed.next_epoch);
      }
      if (!have_next_epoch) {
        if (error != nullptr) {
          *error = "map transaction next epoch is invalid";
        }
        return false;
      }
    } else if (fields.size() == 3U && fields[0] == "artifact" &&
               IsSafeTransactionArtifactName(fields[1]) &&
               (fields[2] == "0" || fields[2] == "1") &&
               artifact_names.insert(fields[1]).second) {
      TransactionArtifactBackup artifact;
      artifact.filename = fields[1];
      artifact.final_path = map_dir / fields[1];
      artifact.backup_path = transaction_dir / "backup" / fields[1];
      artifact.existed = fields[2] == "1";
      parsed.artifacts.push_back(std::move(artifact));
    } else {
      if (error != nullptr) {
        *error = "map transaction manifest contains an invalid field";
      }
      return false;
    }
  }
  if (file.bad() || !have_map_id || !have_phase || !have_base_epoch ||
      !have_next_epoch || parsed.map_id != map_dir.filename().string() ||
      parsed.artifacts.empty() ||
      parsed.artifacts.back().filename != MapStore::ContentEpochFilename() ||
      (parsed.phase == TransactionPhase::kPrepared && parsed.next_epoch != 0) ||
      (parsed.phase != TransactionPhase::kPrepared &&
       parsed.next_epoch <= parsed.base_epoch)) {
    if (error != nullptr) {
      *error = "map transaction manifest is incomplete";
    }
    return false;
  }
  *transaction = std::move(parsed);
  return true;
}

bool ReadTextFileLimited(const std::filesystem::path &path, std::uint64_t max_bytes,
                         std::string *text, std::string *error) {
  if (text == nullptr) {
    return false;
  }
  if (!std::filesystem::is_regular_file(path)) {
    text->clear();
    return true;
  }
  std::error_code ec;
  const auto size = std::filesystem::file_size(path, ec);
  if (ec) {
    if (error != nullptr) {
      *error = "failed to stat " + path.string() + ": " + ec.message();
    }
    return false;
  }
  if (size > max_bytes) {
    if (error != nullptr) {
      *error = path.filename().string() + " exceeds the native size limit";
    }
    return false;
  }
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    if (error != nullptr) {
      *error = "failed to open " + path.string();
    }
    return false;
  }
  std::ostringstream stream;
  stream << file.rdbuf();
  if (file.bad()) {
    if (error != nullptr) {
      *error = "failed to read " + path.string();
    }
    return false;
  }
  *text = stream.str();
  return true;
}

std::string JsonStringField(const std::string &json, const std::string &key,
                            const std::string &fallback) {
  const auto value = JsonObjectStringAtPath(json, {key});
  return value.has_value() ? *value : fallback;
}

double JsonNumberField(const std::string &json, const std::string &key, double fallback) {
  const auto value = JsonObjectNumberAtPath(json, {key});
  return value.has_value() ? *value : fallback;
}

std::int64_t JsonIntegerField(const std::string &json, const std::string &key,
                              std::int64_t fallback) {
  const auto value = JsonNumberField(json, key, static_cast<double>(fallback));
  if (value < 0.0 || std::floor(value) != value ||
      value > static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
    return fallback;
  }
  return static_cast<std::int64_t>(value);
}

double UnixSecondsNow() {
  return std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string ResolveOctomapEditorCommand(const OctomapEditOptions &options) {
  if (!options.editor_command.empty()) {
    return options.editor_command;
  }
  return EnvValue("LINGTU_OCTOMAP_EDITOR");
}

std::string BuildEditorShellCommand(std::string command,
                                    const std::filesystem::path &input,
                                    const std::filesystem::path &output,
                                    const OctomapEditOptions &options) {
  const std::string x = std::to_string(options.x_m);
  const std::string y = std::to_string(options.y_m);
  const std::string z = std::to_string(options.z_m);
  const std::string radius = std::to_string(options.radius_m);
  const bool has_placeholder =
      command.find('{') != std::string::npos && command.find('}') != std::string::npos;
  if (has_placeholder) {
    ReplaceAll(command, "{map}", ShellQuote(std::filesystem::absolute(input).string()));
    ReplaceAll(command, "{input}", ShellQuote(std::filesystem::absolute(input).string()));
    ReplaceAll(command, "{output}", ShellQuote(std::filesystem::absolute(output).string()));
    ReplaceAll(command, "{state}", ShellQuote(options.state));
    ReplaceAll(command, "{shape}", ShellQuote(options.shape));
    ReplaceAll(command, "{x}", x);
    ReplaceAll(command, "{y}", y);
    ReplaceAll(command, "{z}", z);
    ReplaceAll(command, "{radius}", radius);
    return NormalizeShellCommandForSystem(command);
  }
  std::ostringstream stream;
  stream << command
         << " --map " << ShellQuote(std::filesystem::absolute(input).string())
         << " --output " << ShellQuote(std::filesystem::absolute(output).string())
         << " --state " << ShellQuote(options.state)
         << " --x " << x << " --y " << y << " --z " << z
         << " --radius " << radius << " --shape " << ShellQuote(options.shape);
  return NormalizeShellCommandForSystem(stream.str());
}

struct OctomapEditRun {
  bool ok{false};
  std::string reason_code;
  std::string message;
  std::string mode;
  std::string effective_state;
  std::uint64_t edited_voxels{0U};
  std::string command;
  ProcessRunResult process;
};

OctomapEditRun RunNativeOctomapEdit(const std::filesystem::path &input,
                                    const std::filesystem::path &output,
                                    const OctomapEditOptions &options) {
#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  OctomapEditRun result;
  result.mode = "native_octomap";
  result.effective_state =
      options.state == "occupied" || options.state == "preblocked" ? "occupied" : "free";
  octomap::OcTree tree(0.10);
  if (!tree.readBinary(input.string())) {
    result.reason_code = "octomap_read_failed";
    result.message = "OctoMap OcTree::readBinary failed";
    return result;
  }
  const double resolution = tree.getResolution();
  if (!(resolution > 0.0) || !std::isfinite(resolution)) {
    result.reason_code = "invalid_octomap_resolution";
    result.message = "OctoMap reported an invalid resolution";
    return result;
  }
  const double radius_cells_value = std::ceil(options.radius_m / resolution);
  if (!std::isfinite(radius_cells_value) || radius_cells_value < 0.0 ||
      radius_cells_value > static_cast<double>(std::numeric_limits<std::int64_t>::max() / 2 - 1)) {
    result.reason_code = "edit_too_large";
    result.message = "voxel edit radius cannot be represented at the map resolution";
    return result;
  }
  const auto radius_cells = static_cast<std::int64_t>(radius_cells_value);
  const std::uint64_t width = static_cast<std::uint64_t>(radius_cells * 2 + 1);
  constexpr std::uint64_t kMaxEditCells = 4'000'000U;
  const bool too_large = width == 0U || width > kMaxEditCells ||
      width > kMaxEditCells / width ||
      width * width > kMaxEditCells / width;
  if (too_large) {
    result.reason_code = "edit_too_large";
    result.message = "voxel edit exceeds the 4,000,000-cell safety limit";
    return result;
  }
  const bool occupied = result.effective_state == "occupied";
  const double radius_squared = options.radius_m * options.radius_m;
  for (std::int64_t ix = -radius_cells; ix <= radius_cells; ++ix) {
    for (std::int64_t iy = -radius_cells; iy <= radius_cells; ++iy) {
      for (std::int64_t iz = -radius_cells; iz <= radius_cells; ++iz) {
        const double dx = static_cast<double>(ix) * resolution;
        const double dy = static_cast<double>(iy) * resolution;
        const double dz = static_cast<double>(iz) * resolution;
        if (options.shape == "sphere" && dx * dx + dy * dy + dz * dz > radius_squared) {
          continue;
        }
        const octomap::point3d point(
            static_cast<float>(options.x_m + dx),
            static_cast<float>(options.y_m + dy),
            static_cast<float>(options.z_m + dz));
        if (tree.updateNode(point, occupied) != nullptr) {
          ++result.edited_voxels;
        }
      }
    }
  }
  tree.updateInnerOccupancy();
  tree.prune();
  std::filesystem::create_directories(output.parent_path());
  if (!tree.writeBinary(output.string())) {
    result.reason_code = "octomap_write_failed";
    result.message = "OctoMap OcTree::writeBinary failed";
    return result;
  }
  result.ok = true;
  result.message = "OctoMap edited by embedded C++ implementation";
  return result;
#else
  (void)input;
  (void)output;
  (void)options;
  return {false, "native_octomap_unavailable",
          "embedded OctoMap editing requires OctoMap development libraries",
          "native_octomap", "", 0U, "", {}};
#endif
}

OctomapEditRun RunExternalOctomapEdit(const std::filesystem::path &input,
                                      const std::filesystem::path &output,
                                      const OctomapEditOptions &options) {
  OctomapEditRun result;
  result.mode = "external_editor";
  result.effective_state =
      options.state == "occupied" || options.state == "preblocked" ? "occupied" : "free";
  const auto editor = ResolveOctomapEditorCommand(options);
  if (editor.empty()) {
    result.reason_code = "octomap_editor_unavailable";
    result.message = "no native OctoMap support and LINGTU_OCTOMAP_EDITOR is not configured";
    return result;
  }
  result.command = BuildEditorShellCommand(editor, input, output, options);
  ProcessRunOptions run_options;
  run_options.cwd = input.parent_path();
  run_options.timeout_sec = options.timeout_sec > 0.0 ? options.timeout_sec : 15.0;
  run_options.cancel_requested = options.cancel_requested;
  result.process = RunShellCommand(result.command, run_options);
  if (result.process.cancelled) {
    result.reason_code = "editor_cancelled";
    result.message = "OctoMap editor was cancelled";
    return result;
  }
  if (result.process.timed_out) {
    result.reason_code = "editor_timeout";
    result.message = "OctoMap editor timed out";
    return result;
  }
  if (result.process.launch_failed) {
    result.reason_code = "editor_launch_failed";
    result.message = result.process.error.empty() ? "OctoMap editor launch failed" : result.process.error;
    return result;
  }
  if (result.process.exit_code != 0) {
    result.reason_code = "editor_failed";
    result.message = !result.process.stderr_text.empty()
        ? result.process.stderr_text
        : (!result.process.stdout_text.empty() ? result.process.stdout_text
                                               : "OctoMap editor failed");
    return result;
  }
  if (!std::filesystem::is_regular_file(output) || std::filesystem::file_size(output) == 0U) {
    result.reason_code = "editor_output_missing";
    result.message = "OctoMap editor did not produce a non-empty output artifact";
    return result;
  }
  result.edited_voxels = static_cast<std::uint64_t>(
      JsonIntegerField(result.process.stdout_text, "edited_voxels", 0));
  result.effective_state = JsonStringField(
      result.process.stdout_text, "effective_state", result.effective_state);
  result.ok = true;
  result.message = "OctoMap edited by native process boundary";
  return result;
}

OctomapBuildOptions MetadataOptions(const std::filesystem::path &metadata_path) {
  OctomapBuildOptions options;
  std::string metadata;
  std::string error;
  if (!ReadTextFileLimited(metadata_path, 4U * 1024U * 1024U, &metadata, &error)) {
    return options;
  }
  options.build_mode = JsonStringField(metadata, "build_mode", options.build_mode);
  options.frame_id = JsonStringField(metadata, "frame_id", options.frame_id);
  options.source_profile = JsonStringField(metadata, "source_profile", options.source_profile);
  options.data_source = JsonStringField(metadata, "data_source", options.data_source);
  options.slam_source = JsonStringField(metadata, "slam_source", options.slam_source);
  options.localization_source =
      JsonStringField(metadata, "localization_source", options.localization_source);
  options.mapping_source = JsonStringField(metadata, "mapping_source", options.mapping_source);
  options.resolution = JsonNumberField(metadata, "resolution", options.resolution);
  options.support_dilation_cells = static_cast<int>(
      JsonIntegerField(metadata, "support_dilation_cells", options.support_dilation_cells));
  options.free_layers_above = static_cast<int>(
      JsonIntegerField(metadata, "free_layers_above", options.free_layers_above));
  options.free_dilation_cells = static_cast<int>(
      JsonIntegerField(metadata, "free_dilation_cells", options.free_dilation_cells));
  return options;
}

bool ReadVoxelEditJournal(const std::filesystem::path &path, std::vector<std::string> *edits,
                          std::string *error) {
  if (edits == nullptr) {
    return false;
  }
  edits->clear();
  std::string text;
  if (!ReadTextFileLimited(path, 16U * 1024U * 1024U, &text, error)) {
    return false;
  }
  std::istringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }
    if (line.size() > 64U * 1024U || line.front() != '{' || line.back() != '}') {
      if (error != nullptr) {
        *error = "voxel_edits.jsonl contains an invalid native record";
      }
      return false;
    }
    edits->push_back(line);
  }
  return true;
}

std::string VoxelEditsArrayJson(const std::vector<std::string> &edits) {
  std::ostringstream out;
  out << '[';
  for (std::size_t i = 0; i < edits.size(); ++i) {
    if (i != 0U) {
      out << ',';
    }
    out << edits[i];
  }
  out << ']';
  return out.str();
}

struct OctomapVoxelKey {
  unsigned int x{0U};
  unsigned int y{0U};
  unsigned int z{0U};

  bool operator==(const OctomapVoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct OctomapVoxelKeyHash {
  std::size_t operator()(const OctomapVoxelKey &key) const {
    std::size_t seed = std::hash<unsigned int>{}(key.x);
    seed ^= std::hash<unsigned int>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<unsigned int>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

std::string BuildNativeOctomapInDirectory(const std::string &map_id,
                                          const std::filesystem::path &map_dir,
                                          const OctomapBuildOptions &options) {
#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  const auto pcd_path = map_dir / "map.pcd";
  const auto octomap_path = map_dir / "octomap.ot";
  const auto metadata_path = map_dir / "metadata.json";
  if (!std::filesystem::is_regular_file(pcd_path)) {
    return "{"
           "\"action\":\"build_octomap\","
           "\"success\":false,"
           "\"reason_code\":\"missing_map_pcd\","
           "\"message\":" +
           JsonString("missing required source map.pcd at " + pcd_path.string()) +
           ","
           "\"map_id\":" +
           JsonString(map_id) + "}";
  }
  auto loaded = LoadPcdXyz(pcd_path);
  if (!loaded.ok || loaded.points.empty()) {
    return "{"
           "\"action\":\"build_octomap\","
           "\"success\":false,"
           "\"reason_code\":\"source_pcd_unreadable\","
           "\"message\":" +
           JsonString(loaded.message.empty() ? "map.pcd has no readable XYZ points"
                                             : loaded.message) +
           ","
           "\"map_id\":" +
           JsonString(map_id) + "}";
  }

  octomap::OcTree tree(options.resolution > 0.0 ? options.resolution : 0.20);
  std::unordered_map<OctomapVoxelKey, int, OctomapVoxelKeyHash> counts;
  for (const auto &point : loaded.points) {
    octomap::OcTreeKey key;
    if (!tree.coordToKeyChecked(point.x, point.y, point.z, key)) {
      continue;
    }
    ++counts[OctomapVoxelKey{key.k[0], key.k[1], key.k[2]}];
  }
  if (counts.empty()) {
    return "{"
           "\"action\":\"build_octomap\","
           "\"success\":false,"
           "\"reason_code\":\"no_valid_octomap_keys\","
           "\"message\":\"map.pcd points cannot be represented by OctoMap keys\","
           "\"map_id\":" +
           JsonString(map_id) + "}";
  }

  std::unordered_set<OctomapVoxelKey, OctomapVoxelKeyHash> occupied;
  const int dilation = std::max(0, options.support_dilation_cells);
  for (const auto &item : counts) {
    if (item.second <= 0) {
      continue;
    }
    for (int dx = -dilation; dx <= dilation; ++dx) {
      for (int dy = -dilation; dy <= dilation; ++dy) {
        const long long x = static_cast<long long>(item.first.x) + dx;
        const long long y = static_cast<long long>(item.first.y) + dy;
        if (x < 0 || y < 0) {
          continue;
        }
        occupied.insert(OctomapVoxelKey{static_cast<unsigned int>(x), static_cast<unsigned int>(y),
                                        item.first.z});
      }
    }
  }

  for (const auto &key : occupied) {
    octomap::OcTreeKey octo_key;
    octo_key.k[0] = key.x;
    octo_key.k[1] = key.y;
    octo_key.k[2] = key.z;
    tree.updateNode(tree.keyToCoord(octo_key), true);
  }

  const int free_layers = std::max(0, options.free_layers_above);
  const int free_dilation = std::max(0, options.free_dilation_cells);
  for (const auto &key : occupied) {
    for (int dx = -free_dilation; dx <= free_dilation; ++dx) {
      for (int dy = -free_dilation; dy <= free_dilation; ++dy) {
        for (int dz = 1; dz <= free_layers; ++dz) {
          const long long x = static_cast<long long>(key.x) + dx;
          const long long y = static_cast<long long>(key.y) + dy;
          const long long z = static_cast<long long>(key.z) + dz;
          if (x < 0 || y < 0 || z < 0) {
            continue;
          }
          OctomapVoxelKey free_key{static_cast<unsigned int>(x), static_cast<unsigned int>(y),
                                   static_cast<unsigned int>(z)};
          if (occupied.count(free_key) > 0U) {
            continue;
          }
          octomap::OcTreeKey octo_key;
          octo_key.k[0] = free_key.x;
          octo_key.k[1] = free_key.y;
          octo_key.k[2] = free_key.z;
          tree.updateNode(tree.keyToCoord(octo_key), false);
        }
      }
    }
  }
  tree.updateInnerOccupancy();
  std::filesystem::create_directories(octomap_path.parent_path());
  if (!tree.writeBinary(octomap_path.string())) {
    return "{"
           "\"action\":\"build_octomap\","
           "\"success\":false,"
           "\"reason_code\":\"octomap_write_failed\","
           "\"message\":\"OctoMap OcTree::writeBinary failed\","
           "\"map_id\":" +
           JsonString(map_id) + "}";
  }
  if (!WriteTextFile(
          metadata_path,
          MetadataJson(map_id, map_dir, pcd_path, octomap_path, options))) {
    return "{"
           "\"action\":\"build_octomap\","
           "\"success\":false,"
           "\"reason_code\":\"metadata_write_failed\","
           "\"message\":\"failed to write metadata.json\","
           "\"map_id\":" +
           JsonString(map_id) + "}";
  }
  return "{"
         "\"action\":\"build_octomap\","
         "\"success\":true,"
         "\"status\":\"built\","
         "\"mode\":\"native_octomap\","
         "\"algorithm_embedded\":true,"
         "\"map_id\":" +
         JsonString(map_id) +
         ","
         "\"octomap\":" +
         JsonString(octomap_path.string()) +
         ","
         "\"metadata\":{\"ok\":true,\"path\":" +
         JsonString(metadata_path.string()) +
         "},"
         "\"report\":{"
         "\"ok\":true,"
         "\"success\":true,"
         "\"status\":\"built\","
         "\"map_dir\":" +
         JsonString(map_dir.string()) +
         ","
         "\"pcd_path\":" +
         JsonString(pcd_path.string()) +
         ","
         "\"octomap_path\":" +
         JsonString(octomap_path.string()) +
         ","
         "\"metadata_path\":" +
         JsonString(metadata_path.string()) +
         ","
         "\"build_mode\":\"native_octomap\","
         "\"resolution\":" +
         std::to_string(options.resolution) +
         ","
         "\"frame_id\":" +
         JsonString(options.frame_id) +
         ","
         "\"input_points\":" +
         std::to_string(loaded.points.size()) +
         ","
         "\"occupied_voxels\":" +
         std::to_string(occupied.size()) +
         "}"
         "}";
#else
  (void)map_dir;
  (void)options;
  return "{"
         "\"action\":\"build_octomap\","
         "\"success\":false,"
         "\"reason_code\":\"native_octomap_unavailable\","
         "\"message\":\"native_octomap build mode requires lingtu_maps built with OctoMap "
         "development libraries\","
         "\"map_id\":" +
         JsonString(map_id) +
         ","
         "\"algorithm_embedded\":false,"
         "\"supported_build_modes\":" +
         SupportedOctomapBuildModesJson() + "}";
#endif
}

std::string BuildOctomapArtifactInDirectory(const std::string &map_id,
                                            const std::filesystem::path &map_dir,
                                            const OctomapBuildOptions &options, bool allow_reuse) {
  if (options.build_mode == "native_octomap") {
    return BuildNativeOctomapInDirectory(map_id, map_dir, options);
  }
  if (options.build_mode != "external_pcl_converter") {
    return "{"
           "\"action\":\"build_octomap\","
           "\"success\":false,"
           "\"reason_code\":\"unsupported_build_mode\","
           "\"message\":" +
           JsonString("unsupported octomap build mode: " + options.build_mode) + "}";
  }

  const auto pcd_path = map_dir / "map.pcd";
  const auto octomap_path = map_dir / "octomap.ot";
  const auto metadata_path = map_dir / "metadata.json";
  if (!std::filesystem::is_regular_file(pcd_path)) {
    return "{"
           "\"action\":\"build_octomap\","
           "\"success\":false,"
           "\"reason_code\":\"missing_map_pcd\","
           "\"message\":" +
           JsonString("missing required source map.pcd at " + pcd_path.string()) +
           ","
           "\"map_id\":" +
           JsonString(map_id) + "}";
  }

  if (allow_reuse && std::filesystem::is_regular_file(octomap_path) &&
      std::filesystem::file_size(octomap_path) > 0U &&
      ExistingMetadataAllowsReuse(metadata_path)) {
      return "{"
             "\"action\":\"build_octomap\","
             "\"success\":true,"
             "\"status\":\"reused\","
             "\"mode\":\"native_external_pcl_converter\","
             "\"map_id\":" +
             JsonString(map_id) +
             ","
             "\"octomap\":" +
             JsonString(octomap_path.string()) +
             ","
             "\"metadata\":{\"ok\":true,\"path\":" +
             JsonString(metadata_path.string()) +
             "},"
             "\"report\":{"
             "\"ok\":true,"
             "\"success\":true,"
             "\"status\":\"reused\","
             "\"reused\":true,"
             "\"map_dir\":" +
             JsonString(map_dir.string()) +
             ","
             "\"pcd_path\":" +
             JsonString(pcd_path.string()) +
             ","
             "\"octomap_path\":" +
             JsonString(octomap_path.string()) +
             ","
             "\"metadata_path\":" +
             JsonString(metadata_path.string()) +
             ","
             "\"build_mode\":" +
             JsonString(options.build_mode) +
             ","
             "\"resolution\":" +
             std::to_string(options.resolution) +
             ","
             "\"frame_id\":" +
             JsonString(options.frame_id) +
             "}"
             "}";
  }

  const std::string converter = ResolveConverterCommand(options);
  if (converter.empty()) {
    return "{"
           "\"action\":\"build_octomap\","
           "\"success\":false,"
           "\"reason_code\":\"missing_converter\","
           "\"message\":\"no external PCL/OctoMap converter configured; pass converter_command or "
           "set LINGTU_MAP_ARTIFACT_CONVERTER\","
           "\"map_id\":" +
           JsonString(map_id) + "}";
  }

  const std::string command =
      BuildConverterShellCommand(converter, pcd_path, octomap_path, map_dir, options);
  ProcessRunOptions process_options;
  process_options.cwd = map_dir;
  process_options.timeout_sec = options.timeout_sec;
  process_options.cancel_requested = options.cancel_requested;
  const ProcessRunResult process = RunShellCommand(command, process_options);
  if (process.timed_out) {
    return "{"
           "\"action\":\"build_octomap\","
           "\"success\":false,"
           "\"reason_code\":\"converter_timeout\","
           "\"status\":\"converter_timeout\","
           "\"message\":" +
           JsonString("converter timed out after " + std::to_string(options.timeout_sec) + "s") +
           ","
           "\"map_id\":" +
           JsonString(map_id) +
           ","
           "\"converter\":{\"command\":" +
           JsonString(command) + ",\"returncode\":" + std::to_string(process.exit_code) +
           ",\"timeout_sec\":" + std::to_string(options.timeout_sec) +
           ",\"timeout_supported\":true,"
           "\"stdout\":" +
           JsonString(process.stdout_text) +
           ","
           "\"stderr\":" +
           JsonString(process.stderr_text) +
           ","
           "\"stdout_truncated\":" +
           std::string(process.stdout_truncated ? "true" : "false") +
           ","
           "\"stderr_truncated\":" +
           std::string(process.stderr_truncated ? "true" : "false") +
           "}"
           "}";
  }
  if (process.launch_failed) {
    return "{"
           "\"action\":\"build_octomap\","
           "\"success\":false,"
           "\"reason_code\":\"converter_launch_failed\","
           "\"status\":\"converter_launch_failed\","
           "\"message\":" +
           JsonString(process.error.empty() ? "failed to launch converter" : process.error) +
           ","
           "\"map_id\":" +
           JsonString(map_id) +
           ","
           "\"converter\":{\"command\":" +
           JsonString(command) + ",\"returncode\":" + std::to_string(process.exit_code) +
           ",\"timeout_supported\":true}"
           "}";
  }
  if (process.exit_code != 0) {
    return "{"
           "\"action\":\"build_octomap\","
           "\"success\":false,"
           "\"reason_code\":\"converter_failed\","
           "\"status\":\"converter_failed\","
           "\"message\":" +
           JsonString("converter exited with code " + std::to_string(process.exit_code)) +
           ","
           "\"map_id\":" +
           JsonString(map_id) +
           ","
           "\"converter\":{\"command\":" +
           JsonString(command) + ",\"returncode\":" + std::to_string(process.exit_code) +
           ",\"timeout_sec\":" + std::to_string(options.timeout_sec) +
           ",\"timeout_supported\":true,"
           "\"stdout\":" +
           JsonString(process.stdout_text) +
           ","
           "\"stderr\":" +
           JsonString(process.stderr_text) +
           ","
           "\"stdout_truncated\":" +
           std::string(process.stdout_truncated ? "true" : "false") +
           ","
           "\"stderr_truncated\":" +
           std::string(process.stderr_truncated ? "true" : "false") +
           "}"
           "}";
  }
  if (!std::filesystem::is_regular_file(octomap_path) ||
      std::filesystem::file_size(octomap_path) == 0U) {
    return "{"
           "\"action\":\"build_octomap\","
           "\"success\":false,"
           "\"reason_code\":\"converter_missing_output\","
           "\"message\":\"converter succeeded but did not write non-empty octomap.ot\","
           "\"map_id\":" +
           JsonString(map_id) + "}";
  }

  if (!WriteTextFile(
          metadata_path,
          MetadataJson(map_id, map_dir, pcd_path, octomap_path, options))) {
    return "{"
           "\"action\":\"build_octomap\","
           "\"success\":false,"
           "\"reason_code\":\"metadata_write_failed\","
           "\"message\":\"failed to write metadata.json\","
           "\"map_id\":" +
           JsonString(map_id) + "}";
  }

  return "{"
         "\"action\":\"build_octomap\","
         "\"success\":true,"
         "\"status\":\"built\","
         "\"mode\":\"native_external_pcl_converter\","
         "\"map_id\":" +
         JsonString(map_id) +
         ","
         "\"octomap\":" +
         JsonString(octomap_path.string()) +
         ","
         "\"metadata\":{\"ok\":true,\"path\":" +
         JsonString(metadata_path.string()) +
         "},"
         "\"report\":{"
         "\"ok\":true,"
         "\"success\":true,"
         "\"status\":\"built\","
         "\"reused\":false,"
         "\"map_dir\":" +
         JsonString(map_dir.string()) +
         ","
         "\"pcd_path\":" +
         JsonString(pcd_path.string()) +
         ","
         "\"octomap_path\":" +
         JsonString(octomap_path.string()) +
         ","
         "\"metadata_path\":" +
         JsonString(metadata_path.string()) +
         ","
         "\"build_mode\":" +
         JsonString(options.build_mode) +
         ","
         "\"resolution\":" +
         std::to_string(options.resolution) +
         ","
         "\"frame_id\":" +
         JsonString(options.frame_id) +
         ","
         "\"artifacts\":{"
         "\"map_pcd\":{\"path\":\"map.pcd\"},"
         "\"octomap\":{\"path\":\"octomap.ot\"}"
         "},"
         "\"converter\":{\"command\":" +
         JsonString(command) +
         ",\"returncode\":0,"
         "\"timeout_sec\":" +
         std::to_string(options.timeout_sec) +
         ","
         "\"timeout_supported\":true,"
         "\"stdout\":" +
         JsonString(process.stdout_text) +
         ","
         "\"stderr\":" +
         JsonString(process.stderr_text) +
         ","
         "\"stdout_truncated\":" +
         std::string(process.stdout_truncated ? "true" : "false") +
         ","
         "\"stderr_truncated\":" +
         std::string(process.stderr_truncated ? "true" : "false") +
         "}"
         "}"
         "}";
}

bool PublishTransactionArtifacts(MapStore &store, const std::string &map_id,
                                 const std::filesystem::path &staging_map_dir,
                                 const std::vector<TransactionArtifactBackup> &backups,
                                 std::string *error) {
  if (backups.empty()) {
    if (error != nullptr) {
      *error = "map transaction has no artifacts";
    }
    return false;
  }
  PersistentTransaction transaction;
  transaction.map_id = map_id;
  transaction.phase = TransactionPhase::kPublishing;
  transaction.base_epoch = store.ContentEpoch(map_id);
  transaction.next_epoch = store.AllocateContentEpoch();
  transaction.artifacts = backups;
  const auto transaction_dir = backups.front().backup_path.parent_path().parent_path();
  if (!WriteTransactionManifest(transaction_dir, transaction, error)) {
    return false;
  }
  const auto fail_before_commit = [&]() {
    static_cast<void>(RollbackTransactionArtifactsVerified(
        store, map_id, backups, transaction.base_epoch, error));
    return false;
  };
  if (!WriteTextFile(
          staging_map_dir / MapStore::ContentEpochFilename(),
          std::to_string(transaction.next_epoch) + "\n")) {
    if (error != nullptr) {
      *error = "failed to stage map content epoch";
    }
    return fail_before_commit();
  }
  std::error_code ec;
  int published = 0;
  for (const auto &backup : backups) {
    if (const char *inject = std::getenv("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER")) {
      char *end = nullptr;
      const long fail_after = std::strtol(inject, &end, 10);
      if (end != inject && fail_after >= 0 && published >= fail_after) {
        if (error != nullptr) {
          *error = "injected publish failure after " + std::to_string(published) + " artifacts";
        }
        return fail_before_commit();
      }
    }
    const auto staged_path = staging_map_dir / backup.filename;
    const bool staged_exists = std::filesystem::exists(staged_path);
    std::filesystem::remove_all(backup.final_path, ec);
    if (ec) {
      if (error != nullptr) {
        *error = "failed to replace " + backup.filename + ": " + ec.message();
      }
      return fail_before_commit();
    }
    ec.clear();
    if (!staged_exists) {
      continue;
    }
    std::filesystem::create_directories(backup.final_path.parent_path(), ec);
    if (ec) {
      if (error != nullptr) {
        *error = "failed to create artifact directory: " + ec.message();
      }
      return fail_before_commit();
    }
    ec.clear();
    std::filesystem::rename(staged_path, backup.final_path, ec);
    if (ec) {
      if (error != nullptr) {
        *error = "failed to publish " + backup.filename + ": " + ec.message();
      }
      return fail_before_commit();
    }
    ++published;
  }
  // Publishing .content_epoch is the irreversible commit point. If the final
  // manifest update fails, recovery observes PUBLISHING + next_epoch and keeps
  // the fully published generation.
  transaction.phase = TransactionPhase::kCommitted;
  const bool inject_committed_manifest_failure =
      std::getenv("LINGTU_MAPS_INJECT_COMMITTED_MANIFEST_FAILURE") != nullptr;
  if (inject_committed_manifest_failure ||
      !WriteTransactionManifest(transaction_dir, transaction, error)) {
    if (inject_committed_manifest_failure && error != nullptr) {
      *error = "injected committed manifest failure";
    }
    return true;
  }
  return true;
}

std::string InvalidatedSourceMetadataJson(const std::string &map_id,
                                          const std::filesystem::path &staging_map_dir,
                                          const std::string &reason) {
  return "{"
         "\"schema_version\":\"lingtu.saved_map_artifacts.v1\","
         "\"metadata_state\":\"invalidated\","
         "\"invalidated\":true,"
         "\"invalidation_reason\":" +
         JsonString(reason) +
         ","
         "\"created_at\":" +
         JsonString(NowStamp()) +
         ","
         "\"map_name\":" +
         JsonString(map_id) +
         ","
         "\"map_dir\":" +
         JsonString(staging_map_dir.string()) +
         ","
         "\"source\":\"lingtu_maps_pipeline\","
         "\"artifacts\":{\"map_pcd\":{\"path\":\"map.pcd\"}},"
         "\"activation_ready\":false,"
         "\"activation_ready_reason\":\"octomap.ot and metadata.json must be rebuilt after source "
         "mutation\""
         "}\n";
}

bool PublishSourceMapTransaction(MapStore &store, const std::string &map_id,
                                 const std::filesystem::path &staging_map_dir,
                                 const std::filesystem::path &map_dir,
                                 const std::filesystem::path &transaction_dir, std::string *error) {
  auto backups =
      BackupNamedArtifacts(store, map_id, map_dir, transaction_dir, SourceMapMutationArtifactNames());
  const std::int64_t base_epoch = store.ContentEpoch(map_id);
  const auto staged_pcd_path = staging_map_dir / "map.pcd";
  if (!std::filesystem::is_regular_file(staged_pcd_path)) {
    if (error != nullptr) {
      *error = "missing staged map.pcd";
    }
    static_cast<void>(RollbackTransactionArtifactsVerified(
        store, map_id, backups, base_epoch, error));
    return false;
  }
  if (!WriteTextFile(staging_map_dir / "metadata.json",
                     InvalidatedSourceMetadataJson(
                         map_id, staging_map_dir, "source_map_mutated"))) {
    if (error != nullptr) {
      *error = "failed to write invalidated metadata.json";
    }
    static_cast<void>(RollbackTransactionArtifactsVerified(
        store, map_id, backups, base_epoch, error));
    return false;
  }
  if (!PublishTransactionArtifacts(store, map_id, staging_map_dir, backups, error)) {
    return false;
  }
  return true;
}

bool StageExistingArtifact(const std::filesystem::path &map_dir,
                           const std::filesystem::path &staging_map_dir,
                           const std::string &filename, std::string *error) {
  const auto source = map_dir / filename;
  if (!std::filesystem::exists(source)) {
    return true;
  }
  return CopyPathRecursive(source, staging_map_dir / filename, error);
}

bool StageExistingArtifacts(const std::filesystem::path &map_dir,
                            const std::filesystem::path &staging_map_dir,
                            const std::vector<std::string> &filenames, std::string *error) {
  for (const auto &filename : filenames) {
    if (!StageExistingArtifact(map_dir, staging_map_dir, filename, error)) {
      return false;
    }
  }
  return true;
}

void MarkSourceAuxiliaryArtifactsStale(const std::filesystem::path &map_dir) {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  for (const auto *filename : {"poses.txt", "patches"}) {
    const auto path = map_dir / filename;
    if (!std::filesystem::exists(path)) {
      continue;
    }
    std::error_code ec;
    std::filesystem::rename(
        path, map_dir / (std::string(filename) + ".stale-" + std::to_string(stamp)), ec);
  }
}

bool HasSavedSourceTrajectory(const std::filesystem::path &source_dir) {
  const auto patches = source_dir / "patches";
  if (!std::filesystem::is_regular_file(source_dir / "poses.txt") ||
      !std::filesystem::is_directory(patches)) {
    return false;
  }
  std::error_code ec;
  for (const auto &entry : std::filesystem::directory_iterator(patches, ec)) {
    if (ec) {
      return false;
    }
    if (entry.is_regular_file() && Lower(entry.path().extension().string()) == ".pcd") {
      return true;
    }
  }
  return false;
}

std::uint64_t CountPatchPcds(const std::filesystem::path &source_dir) {
  const auto patches = source_dir / "patches";
  if (!std::filesystem::is_directory(patches)) {
    return 0U;
  }
  std::uint64_t count = 0U;
  std::error_code ec;
  for (const auto &entry : std::filesystem::directory_iterator(patches, ec)) {
    if (ec) {
      return count;
    }
    if (entry.is_regular_file() && Lower(entry.path().extension().string()) == ".pcd") {
      ++count;
    }
  }
  return count;
}

std::string BuildMapDirShellCommand(std::string command, const std::filesystem::path &source_dir,
                                    const std::string &strategy, const std::string &default_args) {
  const bool has_placeholder =
      command.find('{') != std::string::npos && command.find('}') != std::string::npos;
  if (has_placeholder) {
    const auto quoted_source = ShellQuote(std::filesystem::absolute(source_dir).string());
    ReplaceAll(command, "{map}", quoted_source);
    ReplaceAll(command, "{map_dir}", quoted_source);
    ReplaceAll(command, "{out}", quoted_source);
    ReplaceAll(command, "{out_dir}", quoted_source);
    ReplaceAll(command, "{strategy}", ShellQuote(strategy));
    return NormalizeShellCommandForSystem(command);
  }
  const auto quoted_source = ShellQuote(std::filesystem::absolute(source_dir).string());
  if (default_args.find('{') != std::string::npos && default_args.find('}') != std::string::npos) {
    std::string templated_args = default_args;
    ReplaceAll(templated_args, "{map}", quoted_source);
    ReplaceAll(templated_args, "{map_dir}", quoted_source);
    ReplaceAll(templated_args, "{out}", quoted_source);
    ReplaceAll(templated_args, "{out_dir}", quoted_source);
    ReplaceAll(templated_args, "{strategy}", ShellQuote(strategy));
    return NormalizeShellCommandForSystem(command + templated_args);
  }
  return NormalizeShellCommandForSystem(command + default_args + " " + quoted_source);
}

std::string ResolvePruneCommand(const SourceCommitOptions &options) {
  if (!options.dynamic_filter_command.empty()) {
    return options.dynamic_filter_command;
  }
  const std::string env_bin = EnvValue("LINGTU_PRUNE_BIN");
  if (!env_bin.empty()) {
    return ShellQuote(env_bin);
  }
  for (const auto &candidate : {
           std::filesystem::current_path() / "build" / "prune" / "Release" / "prune.exe",
           std::filesystem::current_path() / "build" / "prune" / "prune",
           std::filesystem::current_path() / "build" / "prune_wsl" / "prune",
           std::filesystem::path("/opt/lingtu/current/build/prune/prune"),
       }) {
    if (std::filesystem::is_regular_file(candidate)) {
      return ShellQuote(candidate.string());
    }
  }
  return "prune";
}

std::string RunSavedSourceCleanerJson(const std::filesystem::path &source_dir,
                                      const SourceCommitOptions &options, bool *required_failure) {
  if (required_failure != nullptr) {
    *required_failure = false;
  }
  if (!options.dynamic_filter_enabled) {
    return "{"
           "\"success\":false,"
           "\"status\":\"disabled\","
           "\"performed\":false,"
           "\"backend\":\"prune\","
           "\"reason_code\":\"dynamic_filter_disabled\","
           "\"message\":\"dynamic filtering disabled\""
           "}";
  }
  if (!HasSavedSourceTrajectory(source_dir)) {
    if (required_failure != nullptr && options.dynamic_filter_required) {
      *required_failure = true;
    }
    return "{"
           "\"success\":false,"
           "\"status\":\"skipped\","
           "\"performed\":false,"
           "\"backend\":\"prune\","
           "\"reason_code\":\"missing_trajectory\","
           "\"message\":\"dynamic filtering requires map.pcd, poses.txt, and patches/*.pcd\","
           "\"patch_count\":" +
           std::to_string(CountPatchPcds(source_dir)) + "}";
  }

  const std::string command =
      BuildMapDirShellCommand(ResolvePruneCommand(options), source_dir, "prune", " --map-dir") +
      " --overwrite --apply";
  ProcessRunOptions run_options;
  run_options.cwd = source_dir;
  run_options.timeout_sec =
      options.dynamic_filter_timeout_sec > 0.0 ? options.dynamic_filter_timeout_sec : 300.0;
  run_options.cancel_requested = options.cancel_requested;
  const auto started = std::chrono::steady_clock::now();
  const auto process = RunShellCommand(command, run_options);
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  const bool ok = !process.timed_out && !process.launch_failed && process.exit_code == 0 &&
      std::filesystem::is_regular_file(source_dir / "map.pcd");
  if (!ok && required_failure != nullptr && options.dynamic_filter_required) {
    *required_failure = true;
  }
  const std::string reason =
      process.timed_out
          ? "dynamic_filter_timeout"
          : (process.launch_failed ? "dynamic_filter_launch_failed"
                                   : (process.exit_code == 0 ? "dynamic_filter_missing_output"
                                                             : "dynamic_filter_failed"));
  return "{"
         "\"success\":" +
         std::string(ok ? "true" : "false") +
         ","
         "\"status\":" +
         JsonString(ok ? "ok" : "failed") +
         ","
         "\"performed\":" +
         std::string(ok ? "true" : "false") +
         ","
         "\"backend\":\"prune\","
         "\"reason_code\":" +
         JsonString(ok ? "filtered" : reason) +
         ","
         "\"message\":" +
         JsonString(ok ? "saved-map dynamic filter finished" : "saved-map dynamic filter failed") +
         ","
         "\"command\":" +
         JsonString(command) +
         ","
         "\"returncode\":" +
         std::to_string(process.exit_code) +
         ","
         "\"timeout_sec\":" +
         std::to_string(run_options.timeout_sec) +
         ","
         "\"elapsed_sec\":" +
         std::to_string(elapsed) +
         ","
         "\"stdout\":" +
         JsonString(process.stdout_text) +
         ","
         "\"stderr\":" +
         JsonString(process.stderr_text) +
         ","
         "\"stdout_truncated\":" +
         std::string(process.stdout_truncated ? "true" : "false") +
         ","
         "\"stderr_truncated\":" +
         std::string(process.stderr_truncated ? "true" : "false") + "}";
}

void CopySavedSourceAuxiliaryArtifacts(const std::filesystem::path &source_dir,
                                       const std::filesystem::path &staging_map_dir) {
  for (const auto *filename : {
           "poses.txt",
           "trajectory.txt",
           "patches",
           "patch_bundle.manifest",
           "map.clean.pcd",
           "map.removed.pcd",
           "map_optimization.json",
       }) {
    const auto source = source_dir / filename;
    if (!std::filesystem::exists(source)) {
      continue;
    }
    std::string error;
    if (!CopyPathRecursive(source, staging_map_dir / filename, &error)) {
      throw std::runtime_error(error);
    }
  }
}

}  // namespace

MapPipelineCore::MapPipelineCore(MapStore &store) : store_(store) {
  RecoverInterruptedBuilds();
}

void MapPipelineCore::RecoverInterruptedBuilds() const {
  for (const auto &map_id : store_.ListMapIds()) {
    static_cast<void>(RecoverInterruptedBuild(map_id));
  }
}

bool MapPipelineCore::RecoverInterruptedBuild(const std::string &map_id) const {
  const std::string id = MapStore::NormalizeMapId(map_id);
  const auto lock_path = LockPath(id);
  const auto owner_state = MapLock::InspectPersistentOwner(lock_path);
  if (owner_state == PersistentOwnerState::kMissing) {
    return true;
  }
  if (owner_state != PersistentOwnerState::kStale) {
    return false;
  }
  auto recovery_lock = MapLock::TryAcquireForBuildRecovery(
      store_.RootDir(), id, "map-pipeline-recovery");
  if (!recovery_lock.has_value()) {
    return false;
  }

  try {
    std::istringstream lock_info(ReadLockText(id));
    std::string build_id;
    std::getline(lock_info, build_id);
    if (build_id.empty() ||
        !std::all_of(build_id.begin(), build_id.end(), [](unsigned char character) {
          return std::isalnum(character) != 0 || character == '_';
        })) {
      return false;
    }

    const auto build_root = BuildDir(id);
    std::vector<std::filesystem::path> candidates;
    for (const auto &suffix : {"_transaction", "_source_transaction"}) {
      const auto candidate = build_root / (build_id + suffix);
      if (std::filesystem::is_directory(candidate)) {
        candidates.push_back(candidate);
      }
    }
    if (candidates.size() > 1U) {
      return false;
    }
    if (candidates.empty()) {
      std::filesystem::remove_all(lock_path);
      return true;
    }

    const auto transaction_dir = candidates.front();
    const auto manifest_path = transaction_dir / "transaction.state";
    if (!std::filesystem::is_regular_file(manifest_path)) {
      const auto backup_dir = transaction_dir / "backup";
      if (std::filesystem::is_directory(backup_dir) &&
          std::filesystem::directory_iterator(backup_dir) !=
              std::filesystem::directory_iterator()) {
        return false;
      }
      std::filesystem::remove_all(transaction_dir);
      std::filesystem::remove_all(lock_path);
      return true;
    }

    PersistentTransaction transaction;
    std::string manifest_error;
    if (!ReadTransactionManifest(
            transaction_dir, store_.MapPath(id), &transaction, &manifest_error)) {
      return false;
    }
    if (transaction.phase == TransactionPhase::kPrepared) {
      std::filesystem::remove_all(transaction_dir);
      std::filesystem::remove_all(lock_path);
      return true;
    }

    const std::int64_t current_epoch = store_.ContentEpoch(id);
    if (transaction.phase == TransactionPhase::kCommitted &&
        current_epoch != transaction.next_epoch) {
      return false;
    }
    if (transaction.phase == TransactionPhase::kPublishing &&
        current_epoch != transaction.next_epoch) {
      if (!RollbackTransactionArtifacts(transaction.artifacts) ||
          store_.ContentEpoch(id) != transaction.base_epoch) {
        return false;
      }
    }
    std::filesystem::remove_all(transaction_dir);
    std::filesystem::remove_all(lock_path);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

std::string MapPipelineCore::ImportPcdJson(const std::string &map_id,
                                           const std::filesystem::path &source_path,
                                           double voxel_size, const PcdBounds &bounds) {
  std::string id;
  std::string build_id;
  std::filesystem::path transaction_dir;
  try {
    id = MapStore::NormalizeMapId(map_id);
    if (source_path.empty()) {
      return FailureJson("import_pcd", "missing source_path", "missing_source_path");
    }
    if (!std::filesystem::is_regular_file(source_path) ||
        Lower(source_path.extension().string()) != ".pcd") {
      return FailureJson("import_pcd", "source PCD not found: " + source_path.string(),
                         "source_pcd_not_found");
    }

    auto loaded = LoadPcdXyz(source_path);
    if (!loaded.ok || loaded.points.empty()) {
      return FailureJson("import_pcd",
                         loaded.message.empty()
                             ? "source PCD has no readable XYZ points: " + source_path.string()
                             : loaded.message,
                         "source_pcd_unreadable");
    }

    PcdFilterOptions options;
    options.voxel_size = voxel_size;
    options.bounds = bounds;
    auto filtered = FilterPcdPoints(loaded.points, options);
    if (filtered.empty()) {
      return FailureJson("import_pcd", "all points were removed by import filters",
                         "empty_after_filter");
    }

    const auto map_dir = store_.MapPath(id);
    std::filesystem::create_directories(map_dir);
    const auto pcd_path = map_dir / "map.pcd";
    static_cast<void>(RecoverInterruptedBuild(id));
    if (std::filesystem::exists(LockPath(id))) {
      const std::string running = FirstLine(ReadLockText(id));
      return "{"
             "\"action\":\"import_pcd\","
             "\"success\":false,"
             "\"reason_code\":\"build_in_progress\","
             "\"message\":" +
             JsonString("map build already running: " + running) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(running) + "}";
    }

    build_id = MakeBuildId("SOURCE_MAP_IMPORT");
    std::filesystem::create_directories(BuildDir(id));
    if (!TryCreateBuildLock(id, build_id + "\nSOURCE_MAP_IMPORT\n")) {
      return FailureJson("import_pcd", "map build already running", "build_in_progress");
    }
    WriteStatus(id, build_id, "SOURCE_MAP_IMPORT", "RUNNING", 0.0, "source map import started");

    transaction_dir = BuildDir(id) / (build_id + "_source_transaction");
    const auto staging_map_dir = transaction_dir / "staging_map";
    std::filesystem::create_directories(staging_map_dir);
    const auto staged_pcd_path = staging_map_dir / "map.pcd";
    std::string error;
    if (!WriteBinaryXyzPcd(staged_pcd_path, filtered, &error)) {
      WriteStatus(id, build_id, "SOURCE_MAP_IMPORT", "FAILED", 0.0, error);
      std::filesystem::remove_all(LockPath(id));
      std::filesystem::remove_all(transaction_dir);
      return FailureJson("import_pcd", error, "pcd_write_failed");
    }
    const std::filesystem::path backup;
    std::string publish_error;
    if (!PublishSourceMapTransaction(store_, id, staging_map_dir, map_dir, transaction_dir,
                                     &publish_error)) {
      WriteStatus(id, build_id, "SOURCE_MAP_IMPORT", "FAILED", 0.0, publish_error);
      if (!TransactionNeedsRecovery(publish_error)) {
        std::filesystem::remove_all(LockPath(id));
        std::filesystem::remove_all(transaction_dir);
      }
      return "{"
             "\"action\":\"import_pcd\","
             "\"success\":false,"
             "\"reason_code\":\"source_map_commit_failed\","
             "\"message\":" +
             JsonString(publish_error) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":" + TransactionRolledBackJson(publish_error) +
             "}";
    }
    MarkSourceAuxiliaryArtifactsStale(map_dir);
    WriteStatus(id, build_id, "SOURCE_MAP_IMPORT", "SUCCEEDED", 1.0, "source map imported");
    std::filesystem::remove_all(LockPath(id));
    std::filesystem::remove_all(transaction_dir);

    return "{"
           "\"action\":\"import_pcd\","
           "\"success\":true,"
           "\"status\":\"partial\","
           "\"transactional_visibility\":\"staged_until_commit\","
           "\"map_id\":" +
           JsonString(id) +
           ","
           "\"build_id\":" +
           JsonString(build_id) +
           ","
           "\"map_dir\":" +
           JsonString(map_dir.string()) +
           ","
           "\"pcd\":" +
           JsonString(pcd_path.string()) +
           ","
           "\"source_path\":" +
           JsonString(source_path.string()) +
           ","
           "\"point_count\":" +
           std::to_string(filtered.size()) +
           ","
           "\"backup\":" +
           (backup.empty() ? "null" : JsonString(backup.string())) +
           ","
           "\"activation_ready\":false,"
           "\"activation_ready_reason\":\"octomap.ot and metadata.json are required for "
           "OctoPlanner3D\""
           "}";
  } catch (const std::exception &exc) {
    if (!id.empty() && !build_id.empty()) {
      WriteStatus(id, build_id, "SOURCE_MAP_IMPORT", "FAILED", 0.0, exc.what());
      std::filesystem::remove_all(LockPath(id));
    }
    if (!transaction_dir.empty()) {
      std::filesystem::remove_all(transaction_dir);
    }
    return FailureJson("import_pcd", exc.what(), "invalid_map_name");
  }
}

std::string MapPipelineCore::CommitSavedSourceJson(const std::string &map_id,
                                                   const std::filesystem::path &source_dir,
                                                   const SourceCommitOptions &options) {
  std::string id;
  std::string build_id;
  std::filesystem::path transaction_dir;
  std::vector<TransactionArtifactBackup> backups;
  try {
    id = MapStore::NormalizeMapId(map_id);
    if (!std::filesystem::is_directory(source_dir)) {
      return FailureJson("commit_saved_source",
                         "source directory not found: " + source_dir.string(),
                         "source_dir_not_found");
    }
    const auto source_pcd_path = source_dir / "map.pcd";
    if (!std::filesystem::is_regular_file(source_pcd_path)) {
      return FailureJson("commit_saved_source",
                         "source map.pcd not found: " + source_pcd_path.string(),
                         "source_pcd_not_found");
    }
    const auto map_dir = store_.MapPath(id);
    std::filesystem::create_directories(map_dir);
    const auto pcd_path = map_dir / "map.pcd";
    static_cast<void>(RecoverInterruptedBuild(id));
    if (std::filesystem::exists(LockPath(id))) {
      const std::string running = FirstLine(ReadLockText(id));
      return "{"
             "\"action\":\"commit_saved_source\","
             "\"success\":false,"
             "\"reason_code\":\"build_in_progress\","
             "\"message\":" +
             JsonString("map build already running: " + running) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(running) + "}";
    }

    build_id = MakeBuildId("SOURCE_MAP_SAVE");
    std::filesystem::create_directories(BuildDir(id));
    if (!TryCreateBuildLock(id, build_id + "\nSOURCE_MAP_SAVE\n")) {
      return FailureJson("commit_saved_source", "map build already running", "build_in_progress");
    }
    WriteStatus(id, build_id, "SOURCE_MAP_SAVE", "RUNNING", 0.0, "saved source commit started");

    bool dynamic_filter_required_failure = false;
    const auto dynamic_filter =
        RunSavedSourceCleanerJson(source_dir, options, &dynamic_filter_required_failure);
    if (dynamic_filter_required_failure) {
      WriteStatus(id, build_id, "SOURCE_MAP_SAVE", "FAILED", 0.0, "dynamic filter failed");
      std::filesystem::remove_all(LockPath(id));
      return "{"
             "\"action\":\"commit_saved_source\","
             "\"success\":false,"
             "\"reason_code\":\"dynamic_filter_failed\","
             "\"message\":\"required dynamic filtering failed\","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"dynamic_filter\":" +
             dynamic_filter + "}";
    }
    auto loaded = LoadPcdXyz(source_pcd_path);
    if (!loaded.ok) {
      const auto message = loaded.message.empty()
          ? "source map.pcd has no readable XYZ points: " + source_pcd_path.string()
          : loaded.message;
      WriteStatus(id, build_id, "SOURCE_MAP_SAVE", "FAILED", 0.0, message);
      std::filesystem::remove_all(LockPath(id));
      return FailureJson("commit_saved_source", message, "source_pcd_unreadable");
    }
    if (loaded.points.empty()) {
      WriteStatus(id, build_id, "SOURCE_MAP_SAVE", "FAILED", 0.0, "dynamic filter removed map.pcd");
      std::filesystem::remove_all(LockPath(id));
      return "{"
             "\"action\":\"commit_saved_source\","
             "\"success\":false,"
             "\"reason_code\":\"map_pcd_missing_after_filter\","
             "\"message\":\"dynamic filtering removed or emptied map.pcd\","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"dynamic_filter\":" +
             dynamic_filter + "}";
    }

    PcdFilterOptions filter;
    filter.voxel_size = options.voxel_size;
    const auto filtered = FilterPcdPoints(loaded.points, filter);
    if (filtered.empty()) {
      WriteStatus(id, build_id, "SOURCE_MAP_SAVE", "FAILED", 0.0, "source map empty after filters");
      std::filesystem::remove_all(LockPath(id));
      return "{"
             "\"action\":\"commit_saved_source\","
             "\"success\":false,"
             "\"reason_code\":\"empty_after_filter\","
             "\"message\":\"all points were removed by source commit filters\","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"dynamic_filter\":" +
             dynamic_filter + "}";
    }

    transaction_dir = BuildDir(id) / (build_id + "_source_transaction");
    const auto staging_map_dir = transaction_dir / "staging_map";
    std::filesystem::create_directories(staging_map_dir);
    const auto staged_pcd_path = staging_map_dir / "map.pcd";
    std::string error;
    if (!WriteBinaryXyzPcd(staged_pcd_path, filtered, &error)) {
      WriteStatus(id, build_id, "SOURCE_MAP_SAVE", "FAILED", 0.0, error);
      std::filesystem::remove_all(LockPath(id));
      std::filesystem::remove_all(transaction_dir);
      return FailureJson("commit_saved_source", error, "pcd_write_failed");
    }
    CopySavedSourceAuxiliaryArtifacts(source_dir, staging_map_dir);
    backups =
        BackupNamedArtifacts(store_, id, map_dir, transaction_dir, SavedSourceArtifactNames());
    std::string publish_error;
    if (!PublishTransactionArtifacts(store_, id, staging_map_dir, backups, &publish_error)) {
      WriteStatus(id, build_id, "SOURCE_MAP_SAVE", "FAILED", 0.0, publish_error);
      if (!TransactionNeedsRecovery(publish_error)) {
        std::filesystem::remove_all(LockPath(id));
        std::filesystem::remove_all(transaction_dir);
      }
      return "{"
             "\"action\":\"commit_saved_source\","
             "\"success\":false,"
             "\"reason_code\":\"source_map_commit_failed\","
             "\"message\":" +
             JsonString(publish_error) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":" + TransactionRolledBackJson(publish_error) + ","
             "\"dynamic_filter\":" +
             dynamic_filter + "}";
    }

    backups.clear();
    WriteStatus(id, build_id, "SOURCE_MAP_SAVE", "SUCCEEDED", 1.0, "saved source committed");
    std::filesystem::remove_all(LockPath(id));
    std::filesystem::remove_all(transaction_dir);
    return "{"
           "\"action\":\"commit_saved_source\","
           "\"success\":true,"
           "\"status\":\"partial\","
           "\"mode\":\"native_saved_source_transaction\","
           "\"transactional_visibility\":\"staged_until_commit\","
           "\"rolled_back\":false,"
           "\"map_id\":" +
           JsonString(id) +
           ","
           "\"build_id\":" +
           JsonString(build_id) +
           ","
           "\"map_dir\":" +
           JsonString(map_dir.string()) +
           ","
           "\"pcd\":" +
           JsonString(pcd_path.string()) +
           ","
           "\"source_dir\":" +
           JsonString(source_dir.string()) +
           ","
           "\"point_count\":" +
           std::to_string(filtered.size()) +
           ","
           "\"source_point_count\":" +
           std::to_string(loaded.points.size()) +
           ","
           "\"dynamic_filter\":" +
           dynamic_filter +
           ","
           "\"published_auxiliary\":{"
           "\"poses\":" +
           std::string(std::filesystem::is_regular_file(map_dir / "poses.txt") ? "true" : "false") +
           ","
           "\"patches\":" +
           std::string(std::filesystem::is_directory(map_dir / "patches") ? "true" : "false") +
           "},"
           "\"activation_ready\":false,"
           "\"activation_ready_reason\":\"octomap.ot and metadata.json are required for "
           "OctoPlanner3D\""
           "}";
  } catch (const std::exception &exc) {
    if (!backups.empty()) {
      RollbackTransactionArtifacts(backups);
    }
    if (!id.empty() && !build_id.empty()) {
      WriteStatus(id, build_id, "SOURCE_MAP_SAVE", "FAILED", 0.0, exc.what());
      std::filesystem::remove_all(LockPath(id));
    }
    if (!transaction_dir.empty()) {
      std::filesystem::remove_all(transaction_dir);
    }
    return FailureJson("commit_saved_source", exc.what(), "invalid_map_name");
  }
}

std::string MapPipelineCore::CropPcdJson(const std::string &map_id, const PcdBounds &bounds,
                                         bool invert, double voxel_size) {
  std::string id;
  std::string build_id;
  std::filesystem::path transaction_dir;
  try {
    id = MapStore::NormalizeMapId(map_id);
    if (!bounds.enabled) {
      return FailureJson("crop", "bounds must be provided", "missing_bounds");
    }
    const auto map_dir = store_.MapPath(id);
    const auto pcd_path = map_dir / "map.pcd";
    if (!std::filesystem::is_regular_file(pcd_path)) {
      return FailureJson("crop", "no PCD file at " + pcd_path.string(), "map_pcd_not_found");
    }

    auto loaded = LoadPcdXyz(pcd_path);
    if (!loaded.ok || loaded.points.empty()) {
      return FailureJson("crop",
                         loaded.message.empty()
                             ? "map.pcd has no readable XYZ points: " + pcd_path.string()
                             : loaded.message,
                         "map_pcd_unreadable");
    }

    PcdFilterOptions options;
    options.voxel_size = voxel_size;
    options.bounds = bounds;
    options.invert_bounds = invert;
    auto kept = FilterPcdPoints(loaded.points, options);
    if (kept.empty()) {
      return FailureJson("crop", "crop would remove all map points", "empty_after_crop");
    }

    static_cast<void>(RecoverInterruptedBuild(id));
    if (std::filesystem::exists(LockPath(id))) {
      const std::string running = FirstLine(ReadLockText(id));
      return "{"
             "\"action\":\"crop\","
             "\"success\":false,"
             "\"reason_code\":\"build_in_progress\","
             "\"message\":" +
             JsonString("map build already running: " + running) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(running) + "}";
    }

    build_id = MakeBuildId("SOURCE_MAP_CROP");
    std::filesystem::create_directories(BuildDir(id));
    if (!TryCreateBuildLock(id, build_id + "\nSOURCE_MAP_CROP\n")) {
      return FailureJson("crop", "map build already running", "build_in_progress");
    }
    WriteStatus(id, build_id, "SOURCE_MAP_CROP", "RUNNING", 0.0, "source map crop started");

    transaction_dir = BuildDir(id) / (build_id + "_source_transaction");
    const auto staging_map_dir = transaction_dir / "staging_map";
    std::filesystem::create_directories(staging_map_dir);
    const auto staged_pcd_path = staging_map_dir / "map.pcd";
    std::string error;
    if (!WriteBinaryXyzPcd(staged_pcd_path, kept, &error)) {
      WriteStatus(id, build_id, "SOURCE_MAP_CROP", "FAILED", 0.0, error);
      std::filesystem::remove_all(LockPath(id));
      std::filesystem::remove_all(transaction_dir);
      return FailureJson("crop", error, "pcd_write_failed");
    }
    const std::filesystem::path backup;
    std::string publish_error;
    if (!PublishSourceMapTransaction(store_, id, staging_map_dir, map_dir, transaction_dir,
                                     &publish_error)) {
      WriteStatus(id, build_id, "SOURCE_MAP_CROP", "FAILED", 0.0, publish_error);
      if (!TransactionNeedsRecovery(publish_error)) {
        std::filesystem::remove_all(LockPath(id));
        std::filesystem::remove_all(transaction_dir);
      }
      return "{"
             "\"action\":\"crop\","
             "\"success\":false,"
             "\"reason_code\":\"source_map_commit_failed\","
             "\"message\":" +
             JsonString(publish_error) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":" + TransactionRolledBackJson(publish_error) +
             "}";
    }
    MarkSourceAuxiliaryArtifactsStale(map_dir);
    WriteStatus(id, build_id, "SOURCE_MAP_CROP", "SUCCEEDED", 1.0, "source map cropped");
    std::filesystem::remove_all(LockPath(id));
    std::filesystem::remove_all(transaction_dir);

    const auto removed =
        loaded.points.size() > kept.size() ? loaded.points.size() - kept.size() : 0U;
    return "{"
           "\"action\":\"crop\","
           "\"success\":true,"
           "\"status\":\"partial\","
           "\"transactional_visibility\":\"staged_until_commit\","
           "\"map_id\":" +
           JsonString(id) +
           ","
           "\"build_id\":" +
           JsonString(build_id) +
           ","
           "\"map_dir\":" +
           JsonString(map_dir.string()) +
           ","
           "\"pcd\":" +
           JsonString(pcd_path.string()) +
           ","
           "\"point_count\":" +
           std::to_string(kept.size()) +
           ","
           "\"removed_points\":" +
           std::to_string(removed) +
           ","
           "\"backup\":" +
           (backup.empty() ? "null" : JsonString(backup.string())) +
           ","
           "\"activation_ready\":false,"
           "\"activation_ready_reason\":\"octomap.ot and metadata.json must be rebuilt after crop\""
           "}";
  } catch (const std::exception &exc) {
    if (!id.empty() && !build_id.empty()) {
      WriteStatus(id, build_id, "SOURCE_MAP_CROP", "FAILED", 0.0, exc.what());
      std::filesystem::remove_all(LockPath(id));
    }
    if (!transaction_dir.empty()) {
      std::filesystem::remove_all(transaction_dir);
    }
    return FailureJson("crop", exc.what(), "invalid_map_name");
  }
}

std::string MapPipelineCore::BuildOccupancySnapshotJson(const std::string &map_id) {
  std::filesystem::path transaction_dir;
  std::vector<TransactionArtifactBackup> backups;
  std::string id;
  std::string build_id;
  try {
    id = MapStore::NormalizeMapId(map_id);
    const auto map_dir = store_.MapPath(id);
    if (!std::filesystem::is_directory(map_dir)) {
      return FailureJson("build_occupancy_snapshot", "map not found: " + id, "map_not_found");
    }
    const auto pcd_path = map_dir / "map.pcd";
    if (!std::filesystem::is_regular_file(pcd_path)) {
      return FailureJson("build_occupancy_snapshot",
                         "missing required source map.pcd at " + pcd_path.string(),
                         "missing_map_pcd");
    }
    static_cast<void>(RecoverInterruptedBuild(id));
    if (std::filesystem::exists(LockPath(id))) {
      const std::string running = FirstLine(ReadLockText(id));
      return "{"
             "\"action\":\"build_occupancy_snapshot\","
             "\"success\":false,"
             "\"reason_code\":\"build_in_progress\","
             "\"message\":" +
             JsonString("map build already running: " + running) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(running) + "}";
    }

    build_id = MakeBuildId("OCCUPANCY_SNAPSHOT");
    std::filesystem::create_directories(BuildDir(id));
    if (!TryCreateBuildLock(id, build_id + "\nOCCUPANCY_SNAPSHOT\n")) {
      return FailureJson("build_occupancy_snapshot", "map build already running",
                         "build_in_progress");
    }
    WriteStatus(id, build_id, "OCCUPANCY_SNAPSHOT", "RUNNING", 0.0,
                "occupancy snapshot build started");

    transaction_dir = BuildDir(id) / (build_id + "_transaction");
    std::filesystem::create_directories(transaction_dir);
    const auto staging_map_dir = transaction_dir / "staging_map";
    std::filesystem::create_directories(staging_map_dir);
    std::string stage_error;
    if (!CopyPathRecursive(pcd_path, staging_map_dir / "map.pcd", &stage_error)) {
      WriteStatus(id, build_id, "OCCUPANCY_SNAPSHOT", "FAILED", 0.0, stage_error);
      std::filesystem::remove_all(LockPath(id));
      std::filesystem::remove_all(transaction_dir);
      return "{"
             "\"action\":\"build_occupancy_snapshot\","
             "\"success\":false,"
             "\"reason_code\":\"staging_failed\","
             "\"message\":" +
             JsonString(stage_error) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":true"
             "}";
    }
    backups = BackupNamedArtifacts(
        store_, id, map_dir, transaction_dir, OccupancySnapshotArtifactNames());

    const auto result = BuildOccupancyProjectionSnapshot(staging_map_dir, true);
    if (!result.ok) {
      RollbackTransactionArtifacts(backups);
      WriteStatus(id, build_id, "OCCUPANCY_SNAPSHOT", "FAILED", 0.0, result.message);
      std::filesystem::remove_all(LockPath(id));
      std::filesystem::remove_all(transaction_dir);
      return "{"
             "\"action\":\"build_occupancy_snapshot\","
             "\"success\":false,"
             "\"reason_code\":\"occupancy_build_failed\","
             "\"message\":" +
             JsonString(result.message) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":true"
             "}";
    }
    std::string publish_error;
    if (!PublishTransactionArtifacts(store_, id, staging_map_dir, backups, &publish_error)) {
      WriteStatus(id, build_id, "OCCUPANCY_SNAPSHOT", "FAILED", 0.0, publish_error);
      if (!TransactionNeedsRecovery(publish_error)) {
        std::filesystem::remove_all(LockPath(id));
        std::filesystem::remove_all(transaction_dir);
      }
      return "{"
             "\"action\":\"build_occupancy_snapshot\","
             "\"success\":false,"
             "\"reason_code\":\"transaction_commit_failed\","
             "\"message\":" +
             JsonString(publish_error) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":" + TransactionRolledBackJson(publish_error) +
             "}";
    }
    backups.clear();
    WriteStatus(id, build_id, "OCCUPANCY_SNAPSHOT", "SUCCEEDED", 1.0,
                "occupancy snapshot built");
    std::filesystem::remove_all(LockPath(id));
    std::filesystem::remove_all(transaction_dir);
    return "{"
           "\"action\":\"build_occupancy_snapshot\","
           "\"success\":true,"
           "\"mode\":\"projection_native\","
           "\"transactional_visibility\":\"staged_until_commit\","
           "\"rolled_back\":false,"
           "\"map_id\":" +
           JsonString(id) +
           ","
           "\"build_id\":" +
           JsonString(build_id) +
           ","
           "\"occupancy\":" +
           ArtifactPathJson(map_dir, "occupancy.npz") +
           ","
           "\"pgm\":" +
           ArtifactPathJson(map_dir, "map.pgm") +
           ","
           "\"yaml\":" +
           ArtifactPathJson(map_dir, "map.yaml") +
           ","
           "\"grid_shape\":[" +
           std::to_string(result.rows) + "," + std::to_string(result.cols) +
           "],"
           "\"resolution\":" +
           std::to_string(result.resolution) +
           ","
           "\"origin\":[" +
           std::to_string(result.origin_x) + "," + std::to_string(result.origin_y) +
           "],"
           "\"counts\":{"
           "\"unknown\":" +
           std::to_string(result.unknown_count) +
           ","
           "\"free\":" +
           std::to_string(result.free_count) +
           ","
           "\"occupied\":" +
           std::to_string(result.occupied_count) +
           "}"
           "}";
  } catch (const std::exception &exc) {
    if (!backups.empty()) {
      RollbackTransactionArtifacts(backups);
    }
    if (!id.empty() && !build_id.empty()) {
      WriteStatus(id, build_id, "OCCUPANCY_SNAPSHOT", "FAILED", 0.0, exc.what());
      std::filesystem::remove_all(LockPath(id));
    }
    if (!transaction_dir.empty()) {
      std::filesystem::remove_all(transaction_dir);
    }
    return FailureJson("build_occupancy_snapshot", exc.what(), "invalid_map_name");
  }
}

std::string MapPipelineCore::BuildOctomapArtifactJson(const std::string &map_id,
                                                       const OctomapBuildOptions &options) {
  std::filesystem::path transaction_dir;
  std::vector<TransactionArtifactBackup> backups;
  std::string id;
  std::string build_id;
  try {
    id = MapStore::NormalizeMapId(map_id);
    const auto map_dir = store_.MapPath(id);
    if (!std::filesystem::is_directory(map_dir)) {
      return FailureJson("build_octomap", "map not found: " + id, "map_not_found");
    }
    const auto pcd_path = map_dir / "map.pcd";
    if (!std::filesystem::is_regular_file(pcd_path)) {
      return FailureJson("build_octomap",
                         "missing required source map.pcd at " + pcd_path.string(),
                         "missing_map_pcd");
    }
    static_cast<void>(RecoverInterruptedBuild(id));
    if (std::filesystem::exists(LockPath(id))) {
      const std::string running = FirstLine(ReadLockText(id));
      return "{"
             "\"action\":\"build_octomap\","
             "\"success\":false,"
             "\"reason_code\":\"build_in_progress\","
             "\"message\":" +
             JsonString("map build already running: " + running) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(running) + "}";
    }

    build_id = MakeBuildId("OCTOMAP_ARTIFACT");
    std::filesystem::create_directories(BuildDir(id));
    if (!TryCreateBuildLock(id, build_id + "\nOCTOMAP_ARTIFACT\n")) {
      return FailureJson("build_octomap", "map build already running", "build_in_progress");
    }
    WriteStatus(id, build_id, "OCTOMAP_ARTIFACT", "RUNNING", 0.0,
                "octomap artifact build started");

    transaction_dir = BuildDir(id) / (build_id + "_transaction");
    std::filesystem::create_directories(transaction_dir);
    const auto staging_map_dir = transaction_dir / "staging_map";
    std::filesystem::create_directories(staging_map_dir);
    std::string stage_error;
    if (!CopyPathRecursive(pcd_path, staging_map_dir / "map.pcd", &stage_error) ||
        !StageExistingArtifacts(map_dir, staging_map_dir, OctomapArtifactNames(), &stage_error)) {
      WriteStatus(id, build_id, "OCTOMAP_ARTIFACT", "FAILED", 0.0, stage_error);
      std::filesystem::remove_all(LockPath(id));
      std::filesystem::remove_all(transaction_dir);
      return "{"
             "\"action\":\"build_octomap\","
             "\"success\":false,"
             "\"reason_code\":\"staging_failed\","
             "\"message\":" +
             JsonString(stage_error) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":true"
             "}";
    }
    backups =
        BackupNamedArtifacts(store_, id, map_dir, transaction_dir, OctomapArtifactNames());

    const auto octomap = BuildOctomapArtifactInDirectory(id, staging_map_dir, options, true);
    if (!JsonSucceeded(octomap)) {
      RollbackTransactionArtifacts(backups);
      WriteStatus(id, build_id, "OCTOMAP_ARTIFACT", "FAILED", 0.0, "octomap build failed");
      std::filesystem::remove_all(LockPath(id));
      std::filesystem::remove_all(transaction_dir);
      return "{"
             "\"action\":\"build_octomap\","
             "\"success\":false,"
             "\"reason_code\":\"octomap_build_failed\","
             "\"message\":\"octomap build failed\","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":true,"
             "\"octomap_result\":" +
             octomap + "}";
    }
    std::string publish_error;
    if (!PublishTransactionArtifacts(store_, id, staging_map_dir, backups, &publish_error)) {
      WriteStatus(id, build_id, "OCTOMAP_ARTIFACT", "FAILED", 0.0, publish_error);
      if (!TransactionNeedsRecovery(publish_error)) {
        std::filesystem::remove_all(LockPath(id));
        std::filesystem::remove_all(transaction_dir);
      }
      return "{"
             "\"action\":\"build_octomap\","
             "\"success\":false,"
             "\"reason_code\":\"transaction_commit_failed\","
             "\"message\":" +
             JsonString(publish_error) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":" + TransactionRolledBackJson(publish_error) +
             "}";
    }
    backups.clear();
    WriteStatus(id, build_id, "OCTOMAP_ARTIFACT", "SUCCEEDED", 1.0, "octomap artifact built");
    std::filesystem::remove_all(LockPath(id));
    std::filesystem::remove_all(transaction_dir);
    return "{"
           "\"action\":\"build_octomap\","
           "\"success\":true,"
           "\"status\":\"built\","
           "\"mode\":\"native_transaction\","
           "\"map_id\":" +
           JsonString(id) +
           ","
           "\"build_id\":" +
           JsonString(build_id) +
           ","
           "\"transactional_visibility\":\"staged_until_commit\","
           "\"rolled_back\":false,"
           "\"octomap\":" +
           ArtifactPathJson(map_dir, "octomap.ot") +
           ","
           "\"metadata\":" +
           ArtifactPathJson(map_dir, "metadata.json") +
           ","
           "\"octomap_result\":" +
           octomap + "}";
  } catch (const std::exception &exc) {
    if (!backups.empty()) {
      RollbackTransactionArtifacts(backups);
    }
    if (!id.empty() && !build_id.empty()) {
      WriteStatus(id, build_id, "OCTOMAP_ARTIFACT", "FAILED", 0.0, exc.what());
      std::filesystem::remove_all(LockPath(id));
    }
    if (!transaction_dir.empty()) {
      std::filesystem::remove_all(transaction_dir);
    }
    return FailureJson("build_octomap", exc.what(), "invalid_map_name");
  }
}

std::string MapPipelineCore::GetVoxelEditsJson(const std::string &map_id) const {
  try {
    const auto id = MapStore::NormalizeMapId(map_id);
    const auto map_dir = store_.MapPath(id);
    if (!std::filesystem::is_directory(map_dir)) {
      return FailureJson("get_voxel_edits", "map not found: " + id, "map_not_found");
    }
    std::vector<std::string> edits;
    std::string error;
    if (!ReadVoxelEditJournal(map_dir / "voxel_edits.jsonl", &edits, &error)) {
      return FailureJson("get_voxel_edits", error, "invalid_voxel_edit_journal");
    }
    return "{"
           "\"action\":\"get_voxel_edits\","
           "\"success\":true,"
           "\"map_id\":" +
        JsonString(id) +
        ",\"schema_version\":\"lingtu.voxel_edits.v1\","
        "\"count\":" +
        std::to_string(edits.size()) +
        ",\"edits\":" +
        VoxelEditsArrayJson(edits) + "}";
  } catch (const std::exception &exc) {
    return FailureJson("get_voxel_edits", exc.what(), "invalid_map_name");
  }
}

std::string MapPipelineCore::EditOctomapVoxelsJson(
    const std::string &map_id, const OctomapEditOptions &options) {
  std::filesystem::path transaction_dir;
  std::vector<TransactionArtifactBackup> backups;
  std::string id;
  std::string build_id;
  try {
    static const std::unordered_set<std::string> kStates{
        "occupied", "free", "preblocked", "traversable", "clear"};
    if (kStates.count(options.state) == 0U) {
      return FailureJson("edit_voxels", "unsupported voxel edit state", "invalid_edit_state");
    }
    if (options.shape != "sphere" && options.shape != "box") {
      return FailureJson("edit_voxels", "shape must be sphere or box", "invalid_edit_shape");
    }
    if (!std::isfinite(options.x_m) || !std::isfinite(options.y_m) ||
        !std::isfinite(options.z_m) || std::abs(options.x_m) > 500.0 ||
        std::abs(options.y_m) > 500.0 || std::abs(options.z_m) > 500.0) {
      return FailureJson("edit_voxels", "voxel edit center is outside the 500m safety bound",
                         "invalid_edit_center");
    }
    if (!std::isfinite(options.radius_m) || options.radius_m <= 0.0 ||
        options.radius_m > 10.0) {
      return FailureJson("edit_voxels", "radius must be in (0, 10]",
                         "invalid_edit_radius");
    }

    id = MapStore::NormalizeMapId(map_id);
    const auto map_dir = store_.MapPath(id);
    if (!std::filesystem::is_directory(map_dir)) {
      return FailureJson("edit_voxels", "map not found: " + id, "map_not_found");
    }
    const auto pcd_path = map_dir / "map.pcd";
    const auto metadata_path = map_dir / "metadata.json";
    if (!std::filesystem::is_regular_file(pcd_path) ||
        !std::filesystem::is_regular_file(metadata_path)) {
      return FailureJson(
          "edit_voxels",
          "manual OctoMap edits require map.pcd and validated metadata.json",
          "map_not_activation_ready");
    }
    std::filesystem::path octomap_path;
    for (const auto &filename : {"octomap.ot", "octomap.bt"}) {
      const auto candidate = map_dir / filename;
      if (std::filesystem::is_regular_file(candidate)) {
        octomap_path = candidate;
        break;
      }
    }
    if (octomap_path.empty()) {
      return FailureJson("edit_voxels", "map has no octomap.ot or octomap.bt artifact",
                         "octomap_missing");
    }
    static_cast<void>(RecoverInterruptedBuild(id));
    if (std::filesystem::exists(LockPath(id))) {
      return FailureJson("edit_voxels", "map build already running: " + FirstLine(ReadLockText(id)),
                         "build_in_progress");
    }

    build_id = MakeBuildId("OCTOMAP_EDIT");
    std::filesystem::create_directories(BuildDir(id));
    if (!TryCreateBuildLock(id, build_id + "\nOCTOMAP_EDIT\n")) {
      return FailureJson("edit_voxels", "map build already running", "build_in_progress");
    }
    WriteStatus(id, build_id, "OCTOMAP_EDIT", "RUNNING", 0.0,
                "transactional OctoMap edit started");

    transaction_dir = BuildDir(id) / (build_id + "_transaction");
    const auto staging_map_dir = transaction_dir / "staging_map";
    std::filesystem::create_directories(staging_map_dir);
    std::string stage_error;
    if (!CopyPathRecursive(pcd_path, staging_map_dir / "map.pcd", &stage_error) ||
        !StageExistingArtifact(map_dir, staging_map_dir, "occupancy.npz", &stage_error) ||
        !StageExistingArtifacts(map_dir, staging_map_dir, OctomapEditArtifactNames(),
                                &stage_error)) {
      WriteStatus(id, build_id, "OCTOMAP_EDIT", "FAILED", 0.0, stage_error);
      std::filesystem::remove_all(LockPath(id));
      std::filesystem::remove_all(transaction_dir);
      return "{\"action\":\"edit_voxels\",\"success\":false,"
             "\"reason_code\":\"staging_failed\",\"message\":" +
          JsonString(stage_error) +
          ",\"map_id\":" + JsonString(id) +
          ",\"transactional_visibility\":\"staged_until_commit\","
          "\"rolled_back\":true}";
    }

    const auto staged_octomap = staging_map_dir / octomap_path.filename();
    const auto edited_octomap = staged_octomap.string() + ".edited";
    std::filesystem::remove(edited_octomap);
    OctomapEditRun edit_run;
    if (!ResolveOctomapEditorCommand(options).empty()) {
      edit_run = RunExternalOctomapEdit(staged_octomap, edited_octomap, options);
    } else {
      edit_run = RunNativeOctomapEdit(staged_octomap, edited_octomap, options);
    }
    if (!edit_run.ok) {
      WriteStatus(id, build_id, "OCTOMAP_EDIT", "FAILED", 0.0, edit_run.message);
      std::filesystem::remove_all(LockPath(id));
      std::filesystem::remove_all(transaction_dir);
      return "{\"action\":\"edit_voxels\",\"success\":false,"
             "\"reason_code\":" +
          JsonString(edit_run.reason_code.empty() ? "octomap_edit_failed" : edit_run.reason_code) +
          ",\"message\":" + JsonString(edit_run.message) +
          ",\"map_id\":" + JsonString(id) +
          ",\"mode\":" + JsonString(edit_run.mode) +
          ",\"transactional_visibility\":\"staged_until_commit\","
          "\"rolled_back\":true,\"stdout\":" +
          JsonString(edit_run.process.stdout_text) +
          ",\"stderr\":" + JsonString(edit_run.process.stderr_text) + "}";
    }

    std::error_code ec;
    std::filesystem::remove(staged_octomap, ec);
    if (ec) {
      throw std::runtime_error("failed to replace staged OctoMap: " + ec.message());
    }
    ec.clear();
    std::filesystem::rename(edited_octomap, staged_octomap, ec);
    if (ec) {
      throw std::runtime_error("failed to publish edited staged OctoMap: " + ec.message());
    }
    std::ostringstream edit_json;
    edit_json << "{"
              << "\"ts\":" << std::setprecision(17) << UnixSecondsNow() << ","
              << "\"state\":" << JsonString(options.state) << ","
              << "\"effective_state\":" << JsonString(edit_run.effective_state) << ","
              << "\"shape\":" << JsonString(options.shape) << ","
              << "\"center\":{\"x\":" << options.x_m << ",\"y\":" << options.y_m
              << ",\"z\":" << options.z_m << "},"
              << "\"radius\":" << options.radius_m << ","
              << "\"edited_voxels\":" << edit_run.edited_voxels << ","
              << "\"mode\":" << JsonString(edit_run.mode)
              << "}";

    std::vector<std::string> edits;
    std::string journal_error;
    const auto journal_path = staging_map_dir / "voxel_edits.jsonl";
    if (!ReadVoxelEditJournal(journal_path, &edits, &journal_error)) {
      throw std::runtime_error(journal_error);
    }
    edits.push_back(edit_json.str());
    std::ostringstream journal;
    for (const auto &entry : edits) {
      journal << entry << '\n';
    }
    if (!WriteTextFile(journal_path, journal.str())) {
      throw std::runtime_error("failed to stage voxel_edits.jsonl");
    }

    const auto metadata_options = MetadataOptions(staging_map_dir / "metadata.json");
    if (!WriteTextFile(
            staging_map_dir / "metadata.json",
            MetadataJson(id, staging_map_dir, staging_map_dir / "map.pcd", staged_octomap,
                         metadata_options, true, edits.size(), edit_json.str()))) {
      throw std::runtime_error("failed to stage edited metadata.json");
    }

    backups =
        BackupNamedArtifacts(store_, id, map_dir, transaction_dir, OctomapEditArtifactNames());
    std::string publish_error;
    if (!PublishTransactionArtifacts(store_, id, staging_map_dir, backups, &publish_error)) {
      WriteStatus(id, build_id, "OCTOMAP_EDIT", "FAILED", 0.0, publish_error);
      if (!TransactionNeedsRecovery(publish_error)) {
        std::filesystem::remove_all(LockPath(id));
        std::filesystem::remove_all(transaction_dir);
      }
      return "{\"action\":\"edit_voxels\",\"success\":false,"
             "\"reason_code\":\"transaction_commit_failed\",\"message\":" +
          JsonString(publish_error) +
          ",\"map_id\":" + JsonString(id) +
          ",\"transactional_visibility\":\"staged_until_commit\","
          "\"rolled_back\":" + TransactionRolledBackJson(publish_error) + "}";
    }

    backups.clear();
    WriteStatus(id, build_id, "OCTOMAP_EDIT", "SUCCEEDED", 1.0,
                "transactional OctoMap edit committed");
    std::filesystem::remove_all(LockPath(id));
    std::filesystem::remove_all(transaction_dir);
    return "{\"action\":\"edit_voxels\",\"success\":true,"
           "\"map_id\":" +
        JsonString(id) +
        ",\"build_id\":" + JsonString(build_id) +
        ",\"mode\":" + JsonString(edit_run.mode) +
        ",\"transactional_visibility\":\"staged_until_commit\","
        "\"rolled_back\":false,\"octomap\":" +
        JsonString((map_dir / octomap_path.filename()).string()) +
        ",\"journal\":" + JsonString((map_dir / "voxel_edits.jsonl").string()) +
        ",\"edit\":" + edit_json.str() + "}";
  } catch (const std::exception &exc) {
    if (!backups.empty()) {
      RollbackTransactionArtifacts(backups);
    }
    if (!id.empty() && !build_id.empty()) {
      WriteStatus(id, build_id, "OCTOMAP_EDIT", "FAILED", 0.0, exc.what());
      std::filesystem::remove_all(LockPath(id));
    }
    if (!transaction_dir.empty()) {
      std::filesystem::remove_all(transaction_dir);
    }
    return FailureJson("edit_voxels", exc.what(), "octomap_edit_failed");
  }
}

std::string MapPipelineCore::BuildNavigationPackageJson(const std::string &map_id,
                                                        const OctomapBuildOptions &options,
                                                        bool include_esdf,
                                                        bool include_traversability) {
  std::filesystem::path transaction_dir;
  std::vector<TransactionArtifactBackup> backups;
  std::string id;
  std::string build_id;
  try {
    id = MapStore::NormalizeMapId(map_id);
    const auto map_dir = store_.MapPath(id);
    if (!std::filesystem::is_directory(map_dir)) {
      return FailureJson("build_navigation_package", "map not found: " + id, "map_not_found");
    }
    const auto pcd_path = map_dir / "map.pcd";
    if (!std::filesystem::is_regular_file(pcd_path)) {
      return FailureJson("build_navigation_package",
                         "missing required source map.pcd at " + pcd_path.string(),
                         "missing_map_pcd");
    }
    static_cast<void>(RecoverInterruptedBuild(id));
    if (std::filesystem::exists(LockPath(id))) {
      const std::string running = FirstLine(ReadLockText(id));
      return "{"
             "\"action\":\"build_navigation_package\","
             "\"success\":false,"
             "\"reason_code\":\"build_in_progress\","
             "\"message\":" +
             JsonString("map build already running: " + running) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(running) + "}";
    }

    build_id = MakeBuildId("NAVIGATION_PACKAGE");
    std::filesystem::create_directories(BuildDir(id));
    if (!TryCreateBuildLock(id, build_id + "\nNAVIGATION_PACKAGE\n")) {
      return FailureJson("build_navigation_package", "map build already running",
                         "build_in_progress");
    }
    WriteStatus(id, build_id, "NAVIGATION_PACKAGE", "RUNNING", 0.0,
                "navigation package build started");

    transaction_dir = BuildDir(id) / (build_id + "_transaction");
    std::filesystem::create_directories(transaction_dir);
    const auto staging_map_dir = transaction_dir / "staging_map";
    std::filesystem::create_directories(staging_map_dir);
    std::error_code copy_error;
    std::filesystem::copy_file(pcd_path, staging_map_dir / "map.pcd",
                               std::filesystem::copy_options::overwrite_existing, copy_error);
    if (copy_error) {
      WriteStatus(id, build_id, "NAVIGATION_PACKAGE", "FAILED", 0.0, copy_error.message());
      std::filesystem::remove_all(LockPath(id));
      std::filesystem::remove_all(transaction_dir);
      return "{"
             "\"action\":\"build_navigation_package\","
             "\"success\":false,"
             "\"reason_code\":\"staging_failed\","
             "\"message\":" +
             JsonString("failed to stage source map.pcd: " + copy_error.message()) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":true"
             "}";
    }
    backups = BackupTransactionArtifacts(
        store_, id, map_dir, transaction_dir, include_esdf, include_traversability);

    const auto occupancy = BuildOccupancyProjectionSnapshot(staging_map_dir, true);
    if (!occupancy.ok) {
      RollbackTransactionArtifacts(backups);
      WriteStatus(id, build_id, "NAVIGATION_PACKAGE", "FAILED", 0.0, occupancy.message);
      std::filesystem::remove_all(LockPath(id));
      std::filesystem::remove_all(transaction_dir);
      return "{"
             "\"action\":\"build_navigation_package\","
             "\"success\":false,"
             "\"reason_code\":\"occupancy_build_failed\","
             "\"message\":" +
             JsonString(occupancy.message) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":true"
             "}";
    }

    GridArtifactResult esdf;
    if (include_esdf || include_traversability) {
      esdf = BuildEsdfArtifact(staging_map_dir, true);
      if (!esdf.ok) {
        RollbackTransactionArtifacts(backups);
        WriteStatus(id, build_id, "NAVIGATION_PACKAGE", "FAILED", 0.0, esdf.message);
        std::filesystem::remove_all(LockPath(id));
        std::filesystem::remove_all(transaction_dir);
        return "{"
               "\"action\":\"build_navigation_package\","
               "\"success\":false,"
               "\"reason_code\":\"esdf_build_failed\","
               "\"message\":" +
               JsonString(esdf.message) +
               ","
               "\"map_id\":" +
               JsonString(id) +
               ","
               "\"build_id\":" +
               JsonString(build_id) +
               ","
               "\"transactional_visibility\":\"staged_until_commit\","
               "\"rolled_back\":true"
               "}";
      }
    }

    GridArtifactResult traversability;
    if (include_traversability) {
      traversability = BuildTraversabilityArtifact(staging_map_dir, true);
      if (!traversability.ok) {
        RollbackTransactionArtifacts(backups);
        WriteStatus(id, build_id, "NAVIGATION_PACKAGE", "FAILED", 0.0, traversability.message);
        std::filesystem::remove_all(LockPath(id));
        std::filesystem::remove_all(transaction_dir);
        return "{"
               "\"action\":\"build_navigation_package\","
               "\"success\":false,"
               "\"reason_code\":\"traversability_build_failed\","
               "\"message\":" +
               JsonString(traversability.message) +
               ","
               "\"map_id\":" +
               JsonString(id) +
               ","
               "\"build_id\":" +
               JsonString(build_id) +
               ","
               "\"transactional_visibility\":\"staged_until_commit\","
               "\"rolled_back\":true"
               "}";
      }
    }

    const auto octomap = BuildOctomapArtifactInDirectory(id, staging_map_dir, options, false);
    if (!JsonSucceeded(octomap)) {
      RollbackTransactionArtifacts(backups);
      WriteStatus(id, build_id, "NAVIGATION_PACKAGE", "FAILED", 0.0, "octomap build failed");
      std::filesystem::remove_all(LockPath(id));
      std::filesystem::remove_all(transaction_dir);
      return "{"
             "\"action\":\"build_navigation_package\","
             "\"success\":false,"
             "\"reason_code\":\"octomap_build_failed\","
             "\"message\":\"octomap build failed\","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":true,"
             "\"octomap_result\":" +
             octomap + "}";
    }

    std::string publish_error;
    if (!PublishTransactionArtifacts(store_, id, staging_map_dir, backups, &publish_error)) {
      WriteStatus(id, build_id, "NAVIGATION_PACKAGE", "FAILED", 0.0, publish_error);
      if (!TransactionNeedsRecovery(publish_error)) {
        std::filesystem::remove_all(LockPath(id));
        std::filesystem::remove_all(transaction_dir);
      }
      return "{"
             "\"action\":\"build_navigation_package\","
             "\"success\":false,"
             "\"reason_code\":\"transaction_commit_failed\","
             "\"message\":" +
             JsonString(publish_error) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":" + TransactionRolledBackJson(publish_error) +
             "}";
    }

    backups.clear();
    WriteStatus(id, build_id, "NAVIGATION_PACKAGE", "SUCCEEDED", 1.0, "navigation package built");
    std::filesystem::remove_all(LockPath(id));
    std::filesystem::remove_all(transaction_dir);
    return "{"
           "\"action\":\"build_navigation_package\","
           "\"success\":true,"
           "\"status\":\"ready\","
           "\"mode\":\"native_transaction\","
           "\"map_id\":" +
           JsonString(id) +
           ","
           "\"build_id\":" +
           JsonString(build_id) +
           ","
           "\"transactional_visibility\":\"staged_until_commit\","
           "\"rolled_back\":false,"
           "\"artifacts\":{"
           "\"occupancy\":" +
           ArtifactPathJson(map_dir, "occupancy.npz") +
           ","
           "\"map_pgm\":" +
           ArtifactPathJson(map_dir, "map.pgm") +
           ","
           "\"map_yaml\":" +
           ArtifactPathJson(map_dir, "map.yaml") +
           ","
           "\"esdf\":" +
           ArtifactPathJson(map_dir, "esdf.npz") +
           ","
           "\"traversability\":" +
           ArtifactPathJson(map_dir, "traversability.npz") +
           ","
           "\"octomap\":" +
           ArtifactPathJson(map_dir, "octomap.ot") +
           ","
           "\"metadata\":" +
           ArtifactPathJson(map_dir, "metadata.json") +
           "},"
           "\"steps\":{"
           "\"occupancy\":{\"success\":true,\"rows\":" +
           std::to_string(occupancy.rows) + ",\"cols\":" + std::to_string(occupancy.cols) +
           "},"
           "\"esdf\":{\"requested\":" +
           std::string((include_esdf || include_traversability) ? "true" : "false") +
           ",\"success\":" +
           std::string((include_esdf || include_traversability) ? "true" : "false") +
           "},"
           "\"traversability\":{\"requested\":" +
           std::string(include_traversability ? "true" : "false") +
           ",\"success\":" + std::string(include_traversability ? "true" : "false") +
           "},"
           "\"octomap\":" +
           octomap +
           "}"
           "}";
  } catch (const std::exception &exc) {
    if (!backups.empty()) {
      RollbackTransactionArtifacts(backups);
    }
    if (!id.empty() && !build_id.empty()) {
      WriteStatus(id, build_id, "NAVIGATION_PACKAGE", "FAILED", 0.0, exc.what());
      std::filesystem::remove_all(LockPath(id));
    }
    if (!transaction_dir.empty()) {
      std::filesystem::remove_all(transaction_dir);
    }
    return FailureJson("build_navigation_package", exc.what(), "invalid_map_name");
  }
}

std::string MapPipelineCore::BuildEsdfArtifactJson(const std::string &map_id) {
  std::filesystem::path transaction_dir;
  std::vector<TransactionArtifactBackup> backups;
  std::string id;
  std::string build_id;
  try {
    id = MapStore::NormalizeMapId(map_id);
    const auto map_dir = store_.MapPath(id);
    if (!std::filesystem::is_directory(map_dir)) {
      return FailureJson("build_esdf_artifact", "map not found: " + id, "map_not_found");
    }
    const auto occupancy_path = map_dir / "occupancy.npz";
    if (!std::filesystem::is_regular_file(occupancy_path)) {
      return FailureJson("build_esdf_artifact",
                         "occupancy.npz is required before building esdf.npz",
                         "missing_occupancy");
    }
    static_cast<void>(RecoverInterruptedBuild(id));
    if (std::filesystem::exists(LockPath(id))) {
      const std::string running = FirstLine(ReadLockText(id));
      return "{"
             "\"action\":\"build_esdf_artifact\","
             "\"success\":false,"
             "\"reason_code\":\"build_in_progress\","
             "\"message\":" +
             JsonString("map build already running: " + running) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(running) + "}";
    }

    build_id = MakeBuildId("ESDF_ARTIFACT");
    std::filesystem::create_directories(BuildDir(id));
    if (!TryCreateBuildLock(id, build_id + "\nESDF_ARTIFACT\n")) {
      return FailureJson("build_esdf_artifact", "map build already running", "build_in_progress");
    }
    WriteStatus(id, build_id, "ESDF_ARTIFACT", "RUNNING", 0.0, "esdf artifact build started");

    transaction_dir = BuildDir(id) / (build_id + "_transaction");
    std::filesystem::create_directories(transaction_dir);
    const auto staging_map_dir = transaction_dir / "staging_map";
    std::filesystem::create_directories(staging_map_dir);
    std::string stage_error;
    if (!StageExistingArtifact(map_dir, staging_map_dir, "occupancy.npz", &stage_error)) {
      WriteStatus(id, build_id, "ESDF_ARTIFACT", "FAILED", 0.0, stage_error);
      std::filesystem::remove_all(LockPath(id));
      std::filesystem::remove_all(transaction_dir);
      return "{"
             "\"action\":\"build_esdf_artifact\","
             "\"success\":false,"
             "\"reason_code\":\"staging_failed\","
             "\"message\":" +
             JsonString(stage_error) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":true"
             "}";
    }
    backups =
        BackupNamedArtifacts(store_, id, map_dir, transaction_dir, EsdfArtifactNames());

    const auto result = BuildEsdfArtifact(staging_map_dir, true);
    if (!result.ok) {
      RollbackTransactionArtifacts(backups);
      WriteStatus(id, build_id, "ESDF_ARTIFACT", "FAILED", 0.0, result.message);
      std::filesystem::remove_all(LockPath(id));
      std::filesystem::remove_all(transaction_dir);
      return "{"
             "\"action\":\"build_esdf_artifact\","
             "\"success\":false,"
             "\"reason_code\":\"esdf_build_failed\","
             "\"message\":" +
             JsonString(result.message) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":true"
             "}";
    }
    std::string publish_error;
    if (!PublishTransactionArtifacts(store_, id, staging_map_dir, backups, &publish_error)) {
      WriteStatus(id, build_id, "ESDF_ARTIFACT", "FAILED", 0.0, publish_error);
      if (!TransactionNeedsRecovery(publish_error)) {
        std::filesystem::remove_all(LockPath(id));
        std::filesystem::remove_all(transaction_dir);
      }
      return "{"
             "\"action\":\"build_esdf_artifact\","
             "\"success\":false,"
             "\"reason_code\":\"transaction_commit_failed\","
             "\"message\":" +
             JsonString(publish_error) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":" + TransactionRolledBackJson(publish_error) +
             "}";
    }
    backups.clear();
    WriteStatus(id, build_id, "ESDF_ARTIFACT", "SUCCEEDED", 1.0, "esdf artifact built");
    std::filesystem::remove_all(LockPath(id));
    std::filesystem::remove_all(transaction_dir);
    return "{"
           "\"action\":\"build_esdf_artifact\","
           "\"success\":true,"
           "\"mode\":\"native_grid_artifact\","
           "\"transactional_visibility\":\"staged_until_commit\","
           "\"rolled_back\":false,"
           "\"map_id\":" +
           JsonString(id) +
           ","
           "\"build_id\":" +
           JsonString(build_id) +
           ","
           "\"esdf\":" +
           ArtifactPathJson(map_dir, "esdf.npz") +
           ","
           "\"grid_shape\":[" +
           std::to_string(result.rows) + "," + std::to_string(result.cols) +
           "],"
           "\"resolution\":" +
           std::to_string(result.resolution) +
           ","
           "\"origin\":[" +
           std::to_string(result.origin_x) + "," + std::to_string(result.origin_y) +
           "]"
           "}";
  } catch (const std::exception &exc) {
    if (!backups.empty()) {
      RollbackTransactionArtifacts(backups);
    }
    if (!id.empty() && !build_id.empty()) {
      WriteStatus(id, build_id, "ESDF_ARTIFACT", "FAILED", 0.0, exc.what());
      std::filesystem::remove_all(LockPath(id));
    }
    if (!transaction_dir.empty()) {
      std::filesystem::remove_all(transaction_dir);
    }
    return FailureJson("build_esdf_artifact", exc.what(), "invalid_map_name");
  }
}

std::string MapPipelineCore::BuildTraversabilityArtifactJson(const std::string &map_id) {
  std::filesystem::path transaction_dir;
  std::vector<TransactionArtifactBackup> backups;
  std::string id;
  std::string build_id;
  try {
    id = MapStore::NormalizeMapId(map_id);
    const auto map_dir = store_.MapPath(id);
    if (!std::filesystem::is_directory(map_dir)) {
      return FailureJson("build_traversability_artifact", "map not found: " + id, "map_not_found");
    }
    if (!std::filesystem::is_regular_file(map_dir / "occupancy.npz")) {
      return FailureJson("build_traversability_artifact",
                         "occupancy.npz is required before building traversability.npz",
                         "missing_occupancy");
    }
    static_cast<void>(RecoverInterruptedBuild(id));
    if (std::filesystem::exists(LockPath(id))) {
      const std::string running = FirstLine(ReadLockText(id));
      return "{"
             "\"action\":\"build_traversability_artifact\","
             "\"success\":false,"
             "\"reason_code\":\"build_in_progress\","
             "\"message\":" +
             JsonString("map build already running: " + running) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(running) + "}";
    }

    build_id = MakeBuildId("TRAVERSABILITY_ARTIFACT");
    std::filesystem::create_directories(BuildDir(id));
    if (!TryCreateBuildLock(id, build_id + "\nTRAVERSABILITY_ARTIFACT\n")) {
      return FailureJson("build_traversability_artifact", "map build already running",
                         "build_in_progress");
    }
    WriteStatus(id, build_id, "TRAVERSABILITY_ARTIFACT", "RUNNING", 0.0,
                "traversability artifact build started");

    transaction_dir = BuildDir(id) / (build_id + "_transaction");
    std::filesystem::create_directories(transaction_dir);
    const auto staging_map_dir = transaction_dir / "staging_map";
    std::filesystem::create_directories(staging_map_dir);
    std::string stage_error;
    if (!StageExistingArtifact(map_dir, staging_map_dir, "occupancy.npz", &stage_error) ||
        !StageExistingArtifact(map_dir, staging_map_dir, "esdf.npz", &stage_error)) {
      WriteStatus(id, build_id, "TRAVERSABILITY_ARTIFACT", "FAILED", 0.0, stage_error);
      std::filesystem::remove_all(LockPath(id));
      std::filesystem::remove_all(transaction_dir);
      return "{"
             "\"action\":\"build_traversability_artifact\","
             "\"success\":false,"
             "\"reason_code\":\"staging_failed\","
             "\"message\":" +
             JsonString(stage_error) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":true"
             "}";
    }
    backups =
        BackupNamedArtifacts(store_, id, map_dir, transaction_dir, TraversabilityArtifactNames());

    const auto result = BuildTraversabilityArtifact(staging_map_dir, true);
    if (!result.ok) {
      RollbackTransactionArtifacts(backups);
      WriteStatus(id, build_id, "TRAVERSABILITY_ARTIFACT", "FAILED", 0.0, result.message);
      std::filesystem::remove_all(LockPath(id));
      std::filesystem::remove_all(transaction_dir);
      return "{"
             "\"action\":\"build_traversability_artifact\","
             "\"success\":false,"
             "\"reason_code\":\"traversability_build_failed\","
             "\"message\":" +
             JsonString(result.message) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":true"
             "}";
    }
    std::string publish_error;
    if (!PublishTransactionArtifacts(store_, id, staging_map_dir, backups, &publish_error)) {
      WriteStatus(id, build_id, "TRAVERSABILITY_ARTIFACT", "FAILED", 0.0, publish_error);
      if (!TransactionNeedsRecovery(publish_error)) {
        std::filesystem::remove_all(LockPath(id));
        std::filesystem::remove_all(transaction_dir);
      }
      return "{"
             "\"action\":\"build_traversability_artifact\","
             "\"success\":false,"
             "\"reason_code\":\"transaction_commit_failed\","
             "\"message\":" +
             JsonString(publish_error) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":" + TransactionRolledBackJson(publish_error) +
             "}";
    }
    backups.clear();
    WriteStatus(id, build_id, "TRAVERSABILITY_ARTIFACT", "SUCCEEDED", 1.0,
                "traversability artifact built");
    std::filesystem::remove_all(LockPath(id));
    std::filesystem::remove_all(transaction_dir);
    return "{"
           "\"action\":\"build_traversability_artifact\","
           "\"success\":true,"
           "\"mode\":\"native_grid_artifact\","
           "\"transactional_visibility\":\"staged_until_commit\","
           "\"rolled_back\":false,"
           "\"map_id\":" +
           JsonString(id) +
           ","
           "\"build_id\":" +
           JsonString(build_id) +
           ","
           "\"traversability\":" +
           ArtifactPathJson(map_dir, "traversability.npz") +
           ","
           "\"grid_shape\":[" +
           std::to_string(result.rows) + "," + std::to_string(result.cols) +
           "],"
           "\"resolution\":" +
           std::to_string(result.resolution) +
           ","
           "\"origin\":[" +
           std::to_string(result.origin_x) + "," + std::to_string(result.origin_y) +
           "]"
           "}";
  } catch (const std::exception &exc) {
    if (!backups.empty()) {
      RollbackTransactionArtifacts(backups);
    }
    if (!id.empty() && !build_id.empty()) {
      WriteStatus(id, build_id, "TRAVERSABILITY_ARTIFACT", "FAILED", 0.0, exc.what());
      std::filesystem::remove_all(LockPath(id));
    }
    if (!transaction_dir.empty()) {
      std::filesystem::remove_all(transaction_dir);
    }
    return FailureJson("build_traversability_artifact", exc.what(), "invalid_map_name");
  }
}

std::string MapPipelineCore::BuildSemanticArtifactJson(const std::string &map_id) {
  std::filesystem::path transaction_dir;
  std::vector<TransactionArtifactBackup> backups;
  std::string id;
  std::string build_id;
  try {
    id = MapStore::NormalizeMapId(map_id);
    const auto map_dir = store_.MapPath(id);
    if (!std::filesystem::is_directory(map_dir)) {
      return FailureJson("build_semantic_artifact", "map not found: " + id, "map_not_found");
    }
    const auto artifact = map_dir / kSemanticMapArtifactFilename;
    std::string validation_error;
    if (!ValidateSemanticMapBinary(artifact, &validation_error)) {
      return FailureJson("build_semantic_artifact",
                         validation_error.empty() ? "invalid semantic_map.bin" : validation_error,
                         "invalid_semantic_source");
    }
    static_cast<void>(RecoverInterruptedBuild(id));
    if (std::filesystem::exists(LockPath(id))) {
      const std::string running = FirstLine(ReadLockText(id));
      return "{"
             "\"action\":\"build_semantic_artifact\","
             "\"success\":false,"
             "\"reason_code\":\"build_in_progress\","
             "\"message\":" +
             JsonString("map build already running: " + running) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(running) + "}";
    }

    build_id = MakeBuildId("SEMANTIC_ARTIFACT");
    std::filesystem::create_directories(BuildDir(id));
    if (!TryCreateBuildLock(id, build_id + "\nSEMANTIC_ARTIFACT\n")) {
      return FailureJson("build_semantic_artifact", "map build already running",
                         "build_in_progress");
    }
    WriteStatus(id, build_id, "SEMANTIC_ARTIFACT", "RUNNING", 0.0,
                "semantic artifact build started");

    transaction_dir = BuildDir(id) / (build_id + "_transaction");
    std::filesystem::create_directories(transaction_dir);
    const auto staging_map_dir = transaction_dir / "staging_map";
    std::filesystem::create_directories(staging_map_dir);
    std::string stage_error;
    if (!CopyPathRecursive(artifact, staging_map_dir / kSemanticMapArtifactFilename,
                           &stage_error)) {
      WriteStatus(id, build_id, "SEMANTIC_ARTIFACT", "FAILED", 0.0, stage_error);
      std::filesystem::remove_all(LockPath(id));
      std::filesystem::remove_all(transaction_dir);
      return "{"
             "\"action\":\"build_semantic_artifact\","
             "\"success\":false,"
             "\"reason_code\":\"staging_failed\","
             "\"message\":" +
             JsonString(stage_error) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":true"
             "}";
    }
    backups =
        BackupNamedArtifacts(store_, id, map_dir, transaction_dir, SemanticArtifactNames());
    if (!ValidateSemanticMapBinary(staging_map_dir / kSemanticMapArtifactFilename,
                                   &validation_error)) {
      RollbackTransactionArtifacts(backups);
      WriteStatus(id, build_id, "SEMANTIC_ARTIFACT", "FAILED", 0.0, validation_error);
      std::filesystem::remove_all(LockPath(id));
      std::filesystem::remove_all(transaction_dir);
      return "{"
             "\"action\":\"build_semantic_artifact\","
             "\"success\":false,"
             "\"reason_code\":\"invalid_semantic_source\","
             "\"message\":" +
             JsonString(validation_error.empty() ? "invalid semantic_map.bin" : validation_error) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":true"
             "}";
    }
    const auto semantic_map = ReadSemanticMapBinary(staging_map_dir / kSemanticMapArtifactFilename);
    std::string publish_error;
    if (!PublishTransactionArtifacts(store_, id, staging_map_dir, backups, &publish_error)) {
      WriteStatus(id, build_id, "SEMANTIC_ARTIFACT", "FAILED", 0.0, publish_error);
      if (!TransactionNeedsRecovery(publish_error)) {
        std::filesystem::remove_all(LockPath(id));
        std::filesystem::remove_all(transaction_dir);
      }
      return "{"
             "\"action\":\"build_semantic_artifact\","
             "\"success\":false,"
             "\"reason_code\":\"transaction_commit_failed\","
             "\"message\":" +
             JsonString(publish_error) +
             ","
             "\"map_id\":" +
             JsonString(id) +
             ","
             "\"build_id\":" +
             JsonString(build_id) +
             ","
             "\"transactional_visibility\":\"staged_until_commit\","
             "\"rolled_back\":" + TransactionRolledBackJson(publish_error) +
             "}";
    }
    backups.clear();
    WriteStatus(id, build_id, "SEMANTIC_ARTIFACT", "SUCCEEDED", 1.0,
                "semantic artifact validated");
    std::filesystem::remove_all(LockPath(id));
    std::filesystem::remove_all(transaction_dir);
    return "{"
           "\"action\":\"build_semantic_artifact\","
           "\"success\":true,"
           "\"mode\":\"native_transaction\","
           "\"map_id\":" +
           JsonString(id) +
           ","
           "\"build_id\":" +
           JsonString(build_id) +
           ","
           "\"artifact_type\":\"SEMANTIC\","
           "\"semantic\":" +
           ArtifactPathJson(map_dir, kSemanticMapArtifactFilename) +
           ","
           "\"generation\":" +
           std::to_string(semantic_map.generation) +
           ","
           "\"voxel_count\":" +
           std::to_string(semantic_map.Size()) +
           ","
           "\"frame_id\":" +
           JsonString(semantic_map.frame_id) +
           ","
           "\"taxonomy\":" +
           JsonString(semantic_map.taxonomy) +
           ","
           "\"taxonomy_version\":" +
           std::to_string(semantic_map.taxonomy_version) +
           ","
           "\"transactional_visibility\":\"staged_until_commit\","
           "\"rolled_back\":false,"
           "\"activation_ready\":false,"
           "\"activation_ready_reason\":\"semantic_map.bin is query-only and is not a planning "
           "artifact\""
           "}";
  } catch (const std::exception &exc) {
    if (!backups.empty()) {
      RollbackTransactionArtifacts(backups);
    }
    if (!id.empty() && !build_id.empty()) {
      WriteStatus(id, build_id, "SEMANTIC_ARTIFACT", "FAILED", 0.0, exc.what());
      std::filesystem::remove_all(LockPath(id));
    }
    if (!transaction_dir.empty()) {
      std::filesystem::remove_all(transaction_dir);
    }
    return FailureJson("build_semantic_artifact", exc.what(), "invalid_semantic_source");
  }
}

std::filesystem::path MapPipelineCore::BuildDir(const std::string &map_id) const {
  return store_.MapPath(map_id) / ".builds";
}

std::filesystem::path MapPipelineCore::LockPath(const std::string &map_id) const {
  return store_.MapPath(map_id) / ".build_lock";
}

std::filesystem::path MapPipelineCore::LockInfoPath(const std::string &map_id) const {
  return LockPath(map_id) / "metadata.txt";
}

bool MapPipelineCore::TryCreateBuildLock(
    const std::string &map_id,
    const std::string &metadata) const {
  static_cast<void>(RecoverInterruptedBuild(map_id));
  auto coordination_lock =
      MapLock::TryAcquire(store_.RootDir(), map_id, "map-pipeline-build-lock");
  if (!coordination_lock.has_value()) {
    return false;
  }
  const auto lock_path = LockPath(map_id);
  if (!std::filesystem::create_directory(lock_path)) {
    return false;
  }
  try {
    WriteText(
        lock_path / "owner.state",
        "owner=map-pipeline\npid=" + std::to_string(CurrentProcessIdValue()) + "\n");
    WriteText(LockInfoPath(map_id), metadata);
  } catch (...) {
    std::error_code ignored;
    std::filesystem::remove_all(lock_path, ignored);
    throw;
  }
  return true;
}

std::filesystem::path MapPipelineCore::LatestPath(const std::string &map_id) const {
  return BuildDir(map_id) / "latest.json";
}

std::filesystem::path MapPipelineCore::StatusPath(const std::string &map_id,
                                                  const std::string &build_id) const {
  return BuildDir(map_id) / (build_id + ".json");
}

std::filesystem::path MapPipelineCore::BackupExisting(const std::filesystem::path &path,
                                                      const std::string &label) const {
  if (!std::filesystem::exists(path)) {
    return {};
  }
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  const auto backup =
      path.parent_path() / (path.filename().string() + "." + label + "-" + std::to_string(stamp));
  std::filesystem::copy_file(path, backup, std::filesystem::copy_options::overwrite_existing);
  return backup;
}

void MapPipelineCore::ClearDerivedArtifacts(const std::filesystem::path &map_dir) const {
  for (const auto *filename : {
           "occupancy.npz",
           "octomap.ot",
           "octomap.bt",
           "map.pgm",
           "map.yaml",
            "metadata.json",
            "voxel_edits.json",
            "voxel_edits.jsonl",
        }) {
    const auto path = map_dir / filename;
    if (std::filesystem::exists(path)) {
      std::filesystem::remove(path);
    }
  }
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  for (const auto *filename : {"poses.txt", "patches"}) {
    const auto path = map_dir / filename;
    if (std::filesystem::exists(path)) {
      std::filesystem::rename(
          path, map_dir / (std::string(filename) + ".stale-" + std::to_string(stamp)));
    }
  }
}

std::string MapPipelineCore::MakeBuildId(const std::string &artifact_type) const {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return SanitizedArtifactType(artifact_type) + "_" +
         std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

std::string MapPipelineCore::ReadLockText(const std::string &map_id) const {
  const auto lock = LockPath(map_id);
  if (std::filesystem::is_directory(lock)) {
    return ReadText(LockInfoPath(map_id));
  }
  return ReadText(lock);
}

std::string MapPipelineCore::ReadText(const std::filesystem::path &path) const {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("failed to read: " + path.string());
  }
  std::ostringstream stream;
  stream << file.rdbuf();
  return stream.str();
}

void MapPipelineCore::WriteText(const std::filesystem::path &path, const std::string &text) const {
  std::filesystem::create_directories(path.parent_path());
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto temp = path.parent_path() /
      (path.filename().string() + ".tmp-" + std::to_string(CurrentProcessIdValue()) + "-" +
       std::to_string(stamp));
  try {
    {
      std::ofstream file(temp, std::ios::binary | std::ios::trunc);
      if (!file) {
        throw std::runtime_error("failed to write: " + temp.string());
      }
      file.write(text.data(), static_cast<std::streamsize>(text.size()));
      file.flush();
      if (!file) {
        throw std::runtime_error("failed to flush: " + temp.string());
      }
    }
#if defined(_WIN32)
    if (MoveFileExW(
            temp.wstring().c_str(),
            path.wstring().c_str(),
            MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) == 0) {
      throw std::runtime_error("failed to atomically replace: " + path.string());
    }
#else
    const int file_fd = open(temp.c_str(), O_RDONLY);
    if (file_fd >= 0) {
      fsync(file_fd);
      close(file_fd);
    }
    if (::rename(temp.c_str(), path.c_str()) != 0) {
      throw std::runtime_error("failed to atomically replace: " + path.string());
    }
    const int directory_fd = open(path.parent_path().c_str(), O_RDONLY | O_DIRECTORY);
    if (directory_fd >= 0) {
      fsync(directory_fd);
      close(directory_fd);
    }
#endif
  } catch (...) {
    std::error_code ignored;
    std::filesystem::remove(temp, ignored);
    throw;
  }
}

void MapPipelineCore::WriteStatus(const std::string &map_id, const std::string &build_id,
                                  const std::string &artifact_type, const std::string &status,
                                  double progress, const std::string &message) const {
  const std::string json =
      "{"
      "\"schema_version\":\"map.build.status\","
      "\"action\":\"get_build_status\","
      "\"success\":true,"
      "\"has_build\":true,"
      "\"running\":" +
      std::string(status == "RUNNING" ? "true" : "false") +
      ","
      "\"map_id\":" +
      JsonString(map_id) +
      ","
      "\"build_id\":" +
      JsonString(build_id) +
      ","
      "\"artifact_type\":" +
      JsonString(artifact_type) +
      ","
      "\"status\":" +
      JsonString(status) +
      ","
      "\"progress\":" +
      std::to_string(progress) +
      ","
      "\"message\":" +
      JsonString(message) +
      ","
      "\"timestamp_ms\":" +
      JsonString(NowStamp()) + "}\n";
  WriteText(StatusPath(map_id, build_id), json);
  WriteText(LatestPath(map_id), json);
}

std::string MapPipelineCore::FailureJson(const std::string &action, const std::string &message,
                                         const std::string &reason_code) const {
  return "{"
         "\"action\":" +
         JsonString(action) +
         ","
         "\"success\":false,"
         "\"reason_code\":" +
         JsonString(reason_code) +
         ","
         "\"message\":" +
         JsonString(message) + "}";
}

}  // namespace lingtu::maps
