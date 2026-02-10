// ota_service.cpp - OTA 构造函数 + 内部辅助方法
//
// RPC 实现已拆分到:
//   ota_config.cpp    — LoadConfig (YAML 配置加载)
//   ota_file_ops.cpp  — UploadFile / ListRemoteFiles / DeleteRemoteFile / DownloadFromUrl
//   ota_update.cpp    — CheckUpdateReadiness / ApplyUpdate / GetInstalledVersions
//                       Rollback / GetUpgradeHistory / ValidateSystemVersion
//   ota_device.cpp    — GetDeviceInfo / ManageService

#include "ota_service.hpp"
#include "utils.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>

namespace ota {

// ──────────────── 构造 ────────────────

OtaServiceImpl::OtaServiceImpl(const OtaDaemonConfig &config)
    : config_(config), system_version_("1.0.0") {
  // 确保目录存在
  MakeDirs(config_.ota_backup_dir);
  MakeDirs(config_.staging_dir);
  auto parent = [](const std::string &p) {
    auto pos = p.rfind('/');
    return pos != std::string::npos ? p.substr(0, pos) : ".";
  };
  MakeDirs(parent(config_.ota_manifest_path));
  MakeDirs(parent(config_.ota_history_path));
  MakeDirs(parent(config_.system_version_path));

  LoadInstalledManifest();
  LoadSystemVersionJson();

  OtaLogInfo("OtaService initialized: %zu installed, %zu rollback entries",
           installed_artifacts_.size(), rollback_entries_.size());
}

// ──────────────── 路径白名单 ────────────────

bool OtaServiceImpl::IsPathAllowed(const std::string &path) const {
  if (config_.allowed_directories.empty()) return true;
  for (const auto &dir : config_.allowed_directories) {
    if (path.find(dir) == 0) return true;
  }
  return false;
}

// ──────────────── 路径解析 (接收方优先) ────────────────

std::string OtaServiceImpl::ResolveTargetPath(
    const robot::v1::OtaArtifact &artifact) const {

  // 1. 精确匹配: artifact name -> install_path
  auto it = config_.artifact_paths.find(artifact.name());
  if (it != config_.artifact_paths.end() && !it->second.install_path.empty()) {
    OtaLogInfo("ResolveTargetPath: %s -> %s (exact match from artifact_paths.yaml)",
             artifact.name().c_str(), it->second.install_path.c_str());
    return it->second.install_path;
  }

  // 2. 分类默认: category -> default_dir/filename
  std::string cat_key;
  switch (artifact.category()) {
    case robot::v1::OTA_CATEGORY_MODEL:    cat_key = "model"; break;
    case robot::v1::OTA_CATEGORY_CONFIG:   cat_key = "config"; break;
    case robot::v1::OTA_CATEGORY_MAP:      cat_key = "map"; break;
    case robot::v1::OTA_CATEGORY_FIRMWARE: cat_key = "firmware"; break;
    default: break;
  }
  if (!cat_key.empty()) {
    auto cat_it = config_.category_defaults.find(cat_key);
    if (cat_it != config_.category_defaults.end()) {
      std::string filename = artifact.filename();
      if (filename.empty()) filename = artifact.name();
      std::string path = cat_it->second + "/" + filename;
      OtaLogInfo("ResolveTargetPath: %s -> %s (category default: %s)",
               artifact.name().c_str(), path.c_str(), cat_key.c_str());
      return path;
    }
  }

  // 3. Fallback: manifest 中的 target_path (不推荐, 仅兜底)
  if (!artifact.target_path().empty()) {
    OtaLogWarn("ResolveTargetPath: %s -> %s (FALLBACK: manifest target_path, "
             "consider adding to artifact_paths.yaml)",
             artifact.name().c_str(), artifact.target_path().c_str());
    return artifact.target_path();
  }

  // 4. 无法解析
  OtaLogError("ResolveTargetPath: %s -> EMPTY (no mapping, no category default, no manifest path)",
            artifact.name().c_str());
  return "";
}

// ──────────────── Manifest I/O ────────────────

bool OtaServiceImpl::LoadInstalledManifest() {
  std::lock_guard<std::mutex> lock(ota_mutex_);

  if (!FileExists(config_.ota_manifest_path)) {
    OtaLogInfo("No installed manifest at %s, starting fresh", config_.ota_manifest_path.c_str());
    return true;
  }

  std::ifstream file(config_.ota_manifest_path);
  if (!file.is_open()) {
    OtaLogError("Failed to open manifest: %s", config_.ota_manifest_path.c_str());
    return false;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream iss(line);
    std::string type;
    iss >> type;

    if (type == "INSTALLED") {
      std::string name, version, path, sha256, installed_at;
      int category_int = 0;
      iss >> name >> category_int >> version >> path >> sha256 >> installed_at;
      if (!name.empty()) {
        robot::v1::InstalledArtifact a;
        a.set_name(name);
        a.set_category(static_cast<robot::v1::OtaCategory>(category_int));
        a.set_version(version);
        a.set_path(path);
        a.set_sha256(sha256);
        a.set_installed_at(installed_at);
        installed_artifacts_[name] = a;
      }
    } else if (type == "ROLLBACK") {
      std::string name, version, backup_path;
      iss >> name >> version >> backup_path;
      if (!name.empty()) {
        robot::v1::RollbackEntry e;
        e.set_name(name);
        e.set_version(version);
        e.set_backup_path(backup_path);
        rollback_entries_[name] = e;
      }
    }
  }

  OtaLogInfo("Loaded manifest: %zu installed, %zu rollback", installed_artifacts_.size(), rollback_entries_.size());
  return true;
}

bool OtaServiceImpl::SaveInstalledManifest() {
  auto dir = config_.ota_manifest_path.substr(0, config_.ota_manifest_path.rfind('/'));
  std::filesystem::create_directories(dir);

  std::ofstream file(config_.ota_manifest_path, std::ios::trunc);
  if (!file.is_open()) {
    OtaLogError("Failed to save manifest to %s", config_.ota_manifest_path.c_str());
    return false;
  }

  file << "# OTA Installed Manifest - auto-generated, do not edit\n"
       << "# Format: INSTALLED <name> <category> <version> <path> <sha256> <installed_at>\n"
       << "# Format: ROLLBACK <name> <version> <backup_path>\n\n";

  for (const auto &[name, a] : installed_artifacts_) {
    file << "INSTALLED " << a.name() << " " << static_cast<int>(a.category())
         << " " << a.version() << " " << a.path() << " " << a.sha256()
         << " " << a.installed_at() << "\n";
  }
  for (const auto &[name, r] : rollback_entries_) {
    file << "ROLLBACK " << r.name() << " " << r.version() << " " << r.backup_path() << "\n";
  }
  return true;
}

void OtaServiceImpl::LoadSystemVersionJson() {
  std::string content = ReadFileToString(config_.system_version_path);
  if (!content.empty()) {
    // Simple parse for system_version field
    auto pos = content.find("\"system_version\"");
    if (pos != std::string::npos) {
      auto q1 = content.find('"', pos + 16);
      auto q2 = content.find('"', q1 + 1);
      auto q3 = content.find('"', q2 + 1);
      if (q3 != std::string::npos) {
        system_version_ = content.substr(q2 + 1, q3 - q2 - 1);
      }
    }
  }
}

void OtaServiceImpl::SaveSystemVersionJson() {
  // 调用方须持有 ota_mutex_
  std::ostringstream json;
  json << "{\n  \"system_version\": \"" << system_version_ << "\",\n  \"components\": {\n";
  bool first = true;
  for (const auto &[name, a] : installed_artifacts_) {
    if (!first) json << ",\n";
    first = false;
    json << "    \"" << name << "\": {\"version\": \"" << a.version() << "\"}";
  }
  json << "\n  }\n}\n";

  std::string tmp = config_.system_version_path + ".tmp";
  auto dir = config_.system_version_path.substr(0, config_.system_version_path.rfind('/'));
  std::filesystem::create_directories(dir);
  std::ofstream out(tmp);
  if (out.is_open()) {
    out << json.str();
    out.close();
    std::filesystem::rename(tmp, config_.system_version_path);
  }
}

// ──────────────── 备份 ────────────────

bool OtaServiceImpl::BackupArtifact(const std::string &name,
                                     const std::string &current_path,
                                     std::string *backup_path) {
  if (!FileExists(current_path) || !backup_path) return false;
  std::filesystem::create_directories(config_.ota_backup_dir);

  std::string ext;
  auto dot = current_path.rfind('.');
  if (dot != std::string::npos) ext = current_path.substr(dot);

  std::string ts = NowISO8601();
  // Convert ISO to simple timestamp
  std::string simple_ts;
  for (char c : ts) {
    if (std::isdigit(c)) simple_ts += c;
  }
  simple_ts = simple_ts.substr(0, 14); // YYYYMMDDHHMMSS

  *backup_path = config_.ota_backup_dir + "/" + name + "_" + simple_ts + ext;
  try {
    std::filesystem::copy_file(current_path, *backup_path,
                               std::filesystem::copy_options::overwrite_existing);
    OtaLogInfo("Backed up %s -> %s", current_path.c_str(), backup_path->c_str());
    return true;
  } catch (const std::exception &e) {
    OtaLogError("Backup failed: %s", e.what());
    return false;
  }
}

// ──────────────── 健康检查 (systemd-based, 不依赖 ROS2) ────────────────

bool OtaServiceImpl::PostInstallHealthCheck(
    robot::v1::OtaSafetyLevel safety_level, std::string *failure_reason) {

  // HOT: 文件级更新, 不需要检查服务
  if (safety_level == robot::v1::OTA_SAFETY_LEVEL_HOT ||
      safety_level == robot::v1::OTA_SAFETY_LEVEL_UNSPECIFIED) {
    if (failure_reason) *failure_reason = "skipped (HOT: file-level update)";
    return true;
  }

  // COLD: 需要重启, systemd ExecStartPre 负责
  if (safety_level == robot::v1::OTA_SAFETY_LEVEL_COLD) {
    if (failure_reason) *failure_reason = "skipped (COLD: will restart service)";
    return true;
  }

  // WARM: 检查 navigation.service 是否仍正常运行
  OtaLogInfo("Post-install health check (WARM): checking navigation.service...");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  auto status = GetServiceStatus("navigation.service");
  if (status.active_state == "active" && status.sub_state == "running") {
    if (failure_reason) *failure_reason = "passed (navigation.service running)";
    return true;
  }

  // 等待更长时间
  OtaLogWarn("navigation.service not running yet, waiting up to %d seconds...",
           config_.health_check_timeout_sec);
  for (int i = 0; i < config_.health_check_timeout_sec; i += 3) {
    std::this_thread::sleep_for(std::chrono::seconds(3));
    status = GetServiceStatus("navigation.service");
    if (status.active_state == "active" && status.sub_state == "running") {
      if (failure_reason) *failure_reason = "passed (navigation.service recovered)";
      return true;
    }
  }

  if (failure_reason) {
    *failure_reason = "navigation.service state=" + status.active_state + "/" + status.sub_state;
  }
  return false;
}

// ──────────────── 升级历史 ────────────────

void OtaServiceImpl::AppendUpgradeHistory(
    const std::string &action, const std::string &artifact_name,
    const std::string &from_version, const std::string &to_version,
    const std::string &status, robot::v1::OtaFailureCode failure_code,
    const std::string &failure_reason, uint64_t duration_ms,
    const std::string &health_check) {
  auto dir = config_.ota_history_path.substr(0, config_.ota_history_path.rfind('/'));
  std::filesystem::create_directories(dir);

  std::string escaped = failure_reason;
  for (size_t pos = 0; (pos = escaped.find('"', pos)) != std::string::npos; pos += 2)
    escaped.replace(pos, 1, "\\\"");

  std::ofstream file(config_.ota_history_path, std::ios::app);
  if (file.is_open()) {
    file << "{\"ts\":\"" << NowISO8601()
         << "\",\"action\":\"" << action
         << "\",\"artifact\":\"" << artifact_name
         << "\",\"from\":\"" << from_version
         << "\",\"to\":\"" << to_version
         << "\",\"status\":\"" << status
         << "\",\"failure_code\":" << static_cast<int>(failure_code)
         << ",\"failure_reason\":\"" << escaped
         << "\",\"duration_ms\":" << duration_ms
         << ",\"health_check\":\"" << health_check
         << "\"}\n";
  }
}

} // namespace ota
