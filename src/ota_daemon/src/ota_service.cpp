// ota_service.cpp — OTA 核心逻辑 (独立于 ROS2)
// 从 remote_monitoring/data_service.cpp 提取, 去除所有 rclcpp 依赖

#include "ota_service.hpp"
#include "utils.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>

#include <yaml-cpp/yaml.h>

#ifndef OTA_DAEMON_VERSION
#define OTA_DAEMON_VERSION "1.0.0"
#endif

namespace ota {

// ──────────────── 配置加载 ────────────────

OtaDaemonConfig LoadConfig(const std::string &yaml_path) {
  OtaDaemonConfig cfg;
  try {
    YAML::Node root = YAML::LoadFile(yaml_path);

    if (root["grpc_port"]) cfg.grpc_port = root["grpc_port"].as<int>();
    if (root["bind_address"]) cfg.bind_address = root["bind_address"].as<std::string>();
    if (root["robot_id"]) cfg.robot_id = root["robot_id"].as<std::string>();
    if (root["hw_id"]) cfg.hw_id = root["hw_id"].as<std::string>();
    if (root["ota_manifest_path"]) cfg.ota_manifest_path = root["ota_manifest_path"].as<std::string>();
    if (root["ota_backup_dir"]) cfg.ota_backup_dir = root["ota_backup_dir"].as<std::string>();
    if (root["ota_history_path"]) cfg.ota_history_path = root["ota_history_path"].as<std::string>();
    if (root["system_version_path"]) cfg.system_version_path = root["system_version_path"].as<std::string>();
    if (root["ota_public_key_path"]) cfg.ota_public_key_path = root["ota_public_key_path"].as<std::string>();
    if (root["tls_cert_path"]) cfg.tls_cert_path = root["tls_cert_path"].as<std::string>();
    if (root["tls_key_path"]) cfg.tls_key_path = root["tls_key_path"].as<std::string>();
    if (root["health_check_timeout_sec"]) cfg.health_check_timeout_sec = root["health_check_timeout_sec"].as<int>();
    if (root["log_level"]) cfg.log_level = root["log_level"].as<std::string>();

    if (root["staging_dir"]) cfg.staging_dir = root["staging_dir"].as<std::string>();

    if (root["managed_services"]) {
      cfg.managed_services.clear();
      for (const auto &s : root["managed_services"])
        cfg.managed_services.push_back(s.as<std::string>());
    }
    if (root["allowed_directories"]) {
      cfg.allowed_directories.clear();
      for (const auto &d : root["allowed_directories"])
        cfg.allowed_directories.push_back(d.as<std::string>());
    }
  } catch (const std::exception &e) {
    LOG_ERROR("Failed to parse config %s: %s", yaml_path.c_str(), e.what());
  }

  // ── 加载制品路径映射 (artifact_paths.yaml) ──
  // 查找: 同目录下的 artifact_paths.yaml
  std::string paths_yaml = yaml_path;
  auto slash = paths_yaml.rfind('/');
  if (slash != std::string::npos) {
    paths_yaml = paths_yaml.substr(0, slash) + "/artifact_paths.yaml";
  } else {
    paths_yaml = "artifact_paths.yaml";
  }

  try {
    if (FileExists(paths_yaml)) {
      YAML::Node paths_root = YAML::LoadFile(paths_yaml);

      if (paths_root["category_defaults"]) {
        for (const auto &kv : paths_root["category_defaults"]) {
          cfg.category_defaults[kv.first.as<std::string>()] =
              kv.second.as<std::string>();
        }
      }

      if (paths_root["artifacts"]) {
        for (const auto &kv : paths_root["artifacts"]) {
          ArtifactPathEntry entry;
          if (kv.second["install_path"])
            entry.install_path = kv.second["install_path"].as<std::string>();
          if (kv.second["description"])
            entry.description = kv.second["description"].as<std::string>();
          cfg.artifact_paths[kv.first.as<std::string>()] = entry;
        }
      }

      LOG_INFO("Loaded artifact paths: %zu exact, %zu category defaults",
               cfg.artifact_paths.size(), cfg.category_defaults.size());
    } else {
      LOG_WARN("artifact_paths.yaml not found at %s, using manifest target_path as fallback",
               paths_yaml.c_str());
    }
  } catch (const std::exception &e) {
    LOG_ERROR("Failed to parse %s: %s", paths_yaml.c_str(), e.what());
  }

  return cfg;
}

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

  LOG_INFO("OtaService initialized: %zu installed, %zu rollback entries",
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

  // 1. 精确匹配: artifact name → install_path
  auto it = config_.artifact_paths.find(artifact.name());
  if (it != config_.artifact_paths.end() && !it->second.install_path.empty()) {
    LOG_INFO("ResolveTargetPath: %s → %s (exact match from artifact_paths.yaml)",
             artifact.name().c_str(), it->second.install_path.c_str());
    return it->second.install_path;
  }

  // 2. 分类默认: category → default_dir/filename
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
      LOG_INFO("ResolveTargetPath: %s → %s (category default: %s)",
               artifact.name().c_str(), path.c_str(), cat_key.c_str());
      return path;
    }
  }

  // 3. Fallback: manifest 中的 target_path (不推荐, 仅兜底)
  if (!artifact.target_path().empty()) {
    LOG_WARN("ResolveTargetPath: %s → %s (FALLBACK: manifest target_path, "
             "consider adding to artifact_paths.yaml)",
             artifact.name().c_str(), artifact.target_path().c_str());
    return artifact.target_path();
  }

  // 4. 无法解析
  LOG_ERROR("ResolveTargetPath: %s → EMPTY (no mapping, no category default, no manifest path)",
            artifact.name().c_str());
  return "";
}

// ──────────────── Manifest I/O ────────────────

bool OtaServiceImpl::LoadInstalledManifest() {
  std::lock_guard<std::mutex> lock(ota_mutex_);

  if (!FileExists(config_.ota_manifest_path)) {
    LOG_INFO("No installed manifest at %s, starting fresh", config_.ota_manifest_path.c_str());
    return true;
  }

  std::ifstream file(config_.ota_manifest_path);
  if (!file.is_open()) {
    LOG_ERROR("Failed to open manifest: %s", config_.ota_manifest_path.c_str());
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

  LOG_INFO("Loaded manifest: %zu installed, %zu rollback", installed_artifacts_.size(), rollback_entries_.size());
  return true;
}

bool OtaServiceImpl::SaveInstalledManifest() {
  auto dir = config_.ota_manifest_path.substr(0, config_.ota_manifest_path.rfind('/'));
  std::filesystem::create_directories(dir);

  std::ofstream file(config_.ota_manifest_path, std::ios::trunc);
  if (!file.is_open()) {
    LOG_ERROR("Failed to save manifest to %s", config_.ota_manifest_path.c_str());
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
    LOG_INFO("Backed up %s -> %s", current_path.c_str(), backup_path->c_str());
    return true;
  } catch (const std::exception &e) {
    LOG_ERROR("Backup failed: %s", e.what());
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
  LOG_INFO("Post-install health check (WARM): checking navigation.service...");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  auto status = GetServiceStatus("navigation.service");
  if (status.active_state == "active" && status.sub_state == "running") {
    if (failure_reason) *failure_reason = "passed (navigation.service running)";
    return true;
  }

  // 等待更长时间
  LOG_WARN("navigation.service not running yet, waiting up to %d seconds...",
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

// =====================================================================
// RPC: UploadFile
// =====================================================================
grpc::Status OtaServiceImpl::UploadFile(
    grpc::ServerContext *ctx,
    grpc::ServerReader<robot::v1::UploadFileChunk> *reader,
    robot::v1::UploadFileResponse *response) {

  robot::v1::UploadFileChunk chunk;
  std::string remote_path;
  std::string filename;
  uint64_t total_size = 0;
  std::string expected_sha256;
  uint64_t resume_from = 0;
  std::ofstream output;
  uint64_t bytes_written = 0;

  while (reader->Read(&chunk)) {
    // First chunk: extract metadata
    if (chunk.has_metadata() && remote_path.empty()) {
      const auto &meta = chunk.metadata();
      remote_path = meta.remote_path();
      filename = meta.filename();
      total_size = meta.total_size();
      expected_sha256 = meta.sha256();
      resume_from = meta.resume_from_offset();

      if (remote_path.empty()) {
        response->set_success(false);
        response->set_message("remote_path is required");
        return grpc::Status::OK;
      }
      if (!IsPathAllowed(remote_path)) {
        response->set_success(false);
        response->set_message("Path not in allowed directories: " + remote_path);
        return grpc::Status::OK;
      }
      if (remote_path.find("..") != std::string::npos) {
        response->set_success(false);
        response->set_message("Path traversal not allowed");
        return grpc::Status::OK;
      }

      // Ensure parent dir
      auto dir = remote_path.substr(0, remote_path.rfind('/'));
      std::filesystem::create_directories(dir);

      auto mode = (resume_from > 0) ? (std::ios::binary | std::ios::app)
                                     : (std::ios::binary | std::ios::trunc);
      output.open(remote_path, mode);
      if (!output.is_open()) {
        response->set_success(false);
        response->set_message("Cannot open file for writing: " + remote_path);
        return grpc::Status::OK;
      }

      LOG_INFO("UploadFile: %s size=%lu resume=%lu", remote_path.c_str(),
               static_cast<unsigned long>(total_size), static_cast<unsigned long>(resume_from));
    }

    // Write data
    if (!chunk.data().empty() && output.is_open()) {
      output.write(chunk.data().data(), chunk.data().size());
      bytes_written += chunk.data().size();
    }
  }

  if (output.is_open()) output.close();

  // SHA256 check
  std::string actual_sha256;
  if (!remote_path.empty()) {
    actual_sha256 = ComputeSHA256(remote_path);
  }

  if (!expected_sha256.empty() && actual_sha256 != expected_sha256) {
    response->set_success(false);
    response->set_message("SHA256 mismatch: expected=" + expected_sha256 +
                          " actual=" + actual_sha256);
    return grpc::Status::OK;
  }

  response->set_success(true);
  response->set_remote_path(remote_path);
  response->set_bytes_received(bytes_written);
  response->set_sha256(actual_sha256);
  response->set_message("Upload complete");
  response->set_resumed_from(resume_from);
  LOG_INFO("UploadFile complete: %s (%lu bytes)", remote_path.c_str(),
           static_cast<unsigned long>(bytes_written));
  return grpc::Status::OK;
}

// =====================================================================
// RPC: ListRemoteFiles
// =====================================================================
grpc::Status OtaServiceImpl::ListRemoteFiles(
    grpc::ServerContext *, const robot::v1::ListRemoteFilesRequest *request,
    robot::v1::ListRemoteFilesResponse *response) {

  std::string dir = request->directory();
  if (dir.empty() || dir.find("..") != std::string::npos) {
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_INVALID_REQUEST);
    return grpc::Status::OK;
  }
  if (!IsPathAllowed(dir)) {
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_FORBIDDEN);
    return grpc::Status::OK;
  }

  try {
    uint64_t total = 0;
    for (const auto &entry : std::filesystem::directory_iterator(dir)) {
      if (!entry.is_regular_file()) continue;
      auto *info = response->add_files();
      info->set_path(entry.path().string());
      info->set_filename(entry.path().filename().string());
      info->set_size(entry.file_size());
      total += entry.file_size();

      auto ftime = std::filesystem::last_write_time(entry);
      auto sctp = std::chrono::time_point_cast<std::chrono::seconds>(
          std::chrono::file_clock::to_sys(ftime));
      auto t = std::chrono::system_clock::to_time_t(sctp);
      char tbuf[32];
      strftime(tbuf, sizeof(tbuf), "%Y-%m-%dT%H:%M:%SZ", gmtime(&t));
      info->set_modified_time(tbuf);
    }
    response->set_total_size(total);
    response->set_free_space(GetDiskFreeBytes(dir));
  } catch (const std::exception &e) {
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_INTERNAL_ERROR);
    response->mutable_base()->set_error_message(e.what());
  }
  return grpc::Status::OK;
}

// =====================================================================
// RPC: DeleteRemoteFile
// =====================================================================
grpc::Status OtaServiceImpl::DeleteRemoteFile(
    grpc::ServerContext *, const robot::v1::DeleteRemoteFileRequest *request,
    robot::v1::DeleteRemoteFileResponse *response) {

  const std::string &path = request->remote_path();
  if (path.empty() || path.find("..") != std::string::npos || !IsPathAllowed(path)) {
    response->set_success(false);
    response->set_message("Invalid or disallowed path");
    return grpc::Status::OK;
  }

  try {
    if (std::filesystem::remove(path)) {
      response->set_success(true);
      response->set_message("Deleted: " + path);
    } else {
      response->set_success(false);
      response->set_message("File not found: " + path);
    }
  } catch (const std::exception &e) {
    response->set_success(false);
    response->set_message(std::string("Delete failed: ") + e.what());
  }
  return grpc::Status::OK;
}

// =====================================================================
// RPC: DownloadFromUrl
// =====================================================================
grpc::Status OtaServiceImpl::DownloadFromUrl(
    grpc::ServerContext *ctx, const robot::v1::DownloadFromUrlRequest *request,
    grpc::ServerWriter<robot::v1::OtaProgress> *writer) {

  const std::string &url = request->url();
  const std::string &staging_path = request->staging_path();

  if (url.empty() || staging_path.empty()) {
    robot::v1::OtaProgress p;
    p.set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
    p.set_message("url and staging_path are required");
    writer->Write(p);
    return grpc::Status::OK;
  }
  if (staging_path.find("..") != std::string::npos || !IsPathAllowed(staging_path)) {
    robot::v1::OtaProgress p;
    p.set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
    p.set_message("Path not allowed: " + staging_path);
    writer->Write(p);
    return grpc::Status::OK;
  }

  auto dir = staging_path.substr(0, staging_path.rfind('/'));
  std::filesystem::create_directories(dir);

  LOG_INFO("DownloadFromUrl: %s -> %s", url.c_str(), staging_path.c_str());

  // Start progress
  {
    robot::v1::OtaProgress p;
    p.set_status(robot::v1::OTA_UPDATE_STATUS_PENDING);
    p.set_progress_percent(0.0f);
    p.set_bytes_total(request->expected_size());
    p.set_message("Starting download...");
    writer->Write(p);
  }

  // Build curl command
  std::string cmd = "curl -fSL --connect-timeout 15 --max-time 3600";
  for (const auto &[key, val] : request->headers()) {
    cmd += " -H '" + key + ": " + val + "'";
  }
  cmd += " -o '" + staging_path + "'";
  cmd += " -w '\\n__CURL_DONE__ %{http_code} %{size_download}'";
  cmd += " '" + url + "' 2>&1";

  FILE *pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    robot::v1::OtaProgress p;
    p.set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
    p.set_message("Failed to execute curl");
    writer->Write(p);
    return grpc::Status::OK;
  }

  char line[1024];
  auto last_progress = std::chrono::steady_clock::now();
  uint64_t expected = request->expected_size();

  while (fgets(line, sizeof(line), pipe) && !ctx->IsCancelled()) {
    if (std::string(line).find("__CURL_DONE__") != std::string::npos) break;

    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_progress).count();
    if (elapsed_ms >= 500 && expected > 0 && FileExists(staging_path)) {
      std::ifstream probe(staging_path, std::ios::binary | std::ios::ate);
      uint64_t cur = static_cast<uint64_t>(probe.tellg());

      robot::v1::OtaProgress p;
      p.set_status(robot::v1::OTA_UPDATE_STATUS_INSTALLING);
      p.set_progress_percent(std::min(static_cast<float>(cur) / expected * 100.0f, 99.9f));
      p.set_bytes_completed(cur);
      p.set_bytes_total(expected);
      p.set_message("Downloading...");
      writer->Write(p);
      last_progress = now;
    }
  }

  int ret = pclose(pipe);

  if (ctx->IsCancelled()) {
    std::filesystem::remove(staging_path);
    return grpc::Status(grpc::CANCELLED, "Download cancelled");
  }

  if (ret != 0) {
    robot::v1::OtaProgress p;
    p.set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
    p.set_message("curl failed with exit code " + std::to_string(WEXITSTATUS(ret)));
    writer->Write(p);
    return grpc::Status::OK;
  }

  // SHA256 verification
  if (!request->expected_sha256().empty()) {
    robot::v1::OtaProgress v;
    v.set_status(robot::v1::OTA_UPDATE_STATUS_VERIFYING);
    v.set_progress_percent(100.0f);
    v.set_message("Verifying SHA256...");
    writer->Write(v);

    std::string actual = ComputeSHA256(staging_path);
    if (actual != request->expected_sha256()) {
      std::filesystem::remove(staging_path);
      robot::v1::OtaProgress p;
      p.set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
      p.set_message("SHA256 mismatch");
      writer->Write(p);
      return grpc::Status::OK;
    }
  }

  uint64_t final_size = 0;
  if (FileExists(staging_path)) {
    std::ifstream f(staging_path, std::ios::binary | std::ios::ate);
    final_size = static_cast<uint64_t>(f.tellg());
  }

  LOG_INFO("DownloadFromUrl complete: %s (%lu bytes)", staging_path.c_str(),
           static_cast<unsigned long>(final_size));

  robot::v1::OtaProgress done;
  done.set_status(robot::v1::OTA_UPDATE_STATUS_SUCCESS);
  done.set_progress_percent(100.0f);
  done.set_bytes_completed(final_size);
  done.set_bytes_total(final_size);
  done.set_message("Download complete: " + staging_path);
  writer->Write(done);
  return grpc::Status::OK;
}

// =====================================================================
// RPC: CheckUpdateReadiness
// =====================================================================
grpc::Status OtaServiceImpl::CheckUpdateReadiness(
    grpc::ServerContext *, const robot::v1::CheckUpdateReadinessRequest *request,
    robot::v1::CheckUpdateReadinessResponse *response) {

  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  bool all_passed = true;

  // 1. Disk space
  {
    uint64_t need = 0;
    for (const auto &a : request->artifacts()) need += a.size();
    need *= 2;
    uint64_t free_bytes = GetDiskFreeBytes("/opt/robot");
    auto *c = response->add_checks();
    c->set_check_name("disk_space");
    bool ok = free_bytes >= need;
    c->set_passed(ok);
    c->set_message(ok ? "Sufficient disk space" : "Insufficient disk space");
    c->set_detail("free=" + std::to_string(free_bytes) + " required=" + std::to_string(need));
    if (!ok) all_passed = false;
  }

  // 2. Battery
  {
    int battery = GetBatteryPercent();
    int min_bat = 0;
    for (const auto &a : request->artifacts())
      min_bat = std::max(min_bat, static_cast<int>(a.min_battery_percent()));

    auto *c = response->add_checks();
    c->set_check_name("battery");
    bool ok = (battery < 0) || (battery >= min_bat);  // -1 = no battery = always ok
    c->set_passed(ok);
    c->set_message(ok ? "Battery OK" : "Battery too low");
    c->set_detail("battery=" + std::to_string(battery) + " min=" + std::to_string(min_bat));
    if (!ok) all_passed = false;
  }

  // 3. Hardware compatibility
  {
    auto *c = response->add_checks();
    c->set_check_name("hw_compat");
    bool ok = true;
    std::string detail;
    for (const auto &a : request->artifacts()) {
      bool compat = false;
      for (const auto &hw : a.hw_compat()) {
        if (hw == "*" || hw == config_.hw_id) { compat = true; break; }
      }
      if (!compat) {
        ok = false;
        detail += a.name() + " incompatible; ";
      }
    }
    c->set_passed(ok);
    c->set_message(ok ? "Hardware compatible" : "Hardware incompatible");
    c->set_detail(detail);
    if (!ok) all_passed = false;
  }

  // 4. Dependencies
  {
    auto *c = response->add_checks();
    c->set_check_name("dependencies");
    bool ok = true;
    std::string detail;
    std::lock_guard<std::mutex> lock(ota_mutex_);
    for (const auto &a : request->artifacts()) {
      for (const auto &dep : a.dependencies()) {
        auto it = installed_artifacts_.find(dep.artifact_name());
        if (it == installed_artifacts_.end()) {
          ok = false;
          detail += "missing:" + dep.artifact_name() + "; ";
        } else if (!dep.min_version().empty() &&
                   CompareSemver(it->second.version(), dep.min_version()) < 0) {
          ok = false;
          detail += dep.artifact_name() + " too old; ";
        }
      }
    }
    c->set_passed(ok);
    c->set_message(ok ? "Dependencies satisfied" : "Dependencies not met");
    c->set_detail(detail);
    if (!ok) all_passed = false;
  }

  // 5. Ed25519 signature
  {
    auto *c = response->add_checks();
    c->set_check_name("signature");
    if (request->manifest_signature().empty() || config_.ota_public_key_path.empty()) {
      c->set_passed(true);
      c->set_message("Signature check skipped (no signature or key)");
    } else {
      // Reconstruct canonical manifest for verification (simplified)
      bool ok = VerifyEd25519(config_.ota_public_key_path,
                               request->manifest_signature(),  // placeholder
                               request->manifest_signature());
      c->set_passed(ok);
      c->set_message(ok ? "Signature valid" : "Signature invalid");
      if (!ok) all_passed = false;
    }
  }

  response->set_ready(all_passed);
  return grpc::Status::OK;
}

// =====================================================================
// RPC: ApplyUpdate
// =====================================================================
grpc::Status OtaServiceImpl::ApplyUpdate(
    grpc::ServerContext *, const robot::v1::ApplyUpdateRequest *request,
    robot::v1::ApplyUpdateResponse *response) {

  const auto &artifact = request->artifact();
  const std::string &staged_path = request->staged_path();

  // ── 关键: 接收方路径解析 (本地映射 > manifest target_path) ──
  const std::string target_path = ResolveTargetPath(artifact);

  LOG_INFO("ApplyUpdate: name=%s version=%s category=%d resolved_target=%s",
           artifact.name().c_str(), artifact.version().c_str(),
           static_cast<int>(artifact.category()), target_path.c_str());

  // 1. Path validation
  if (staged_path.empty() || target_path.empty()) {
    response->set_success(false);
    response->set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
    response->set_message("staged_path is empty or target_path could not be resolved "
                          "(check artifact_paths.yaml for artifact: " + artifact.name() + ")");
    return grpc::Status::OK;
  }
  if (staged_path.find("..") != std::string::npos ||
      target_path.find("..") != std::string::npos) {
    response->set_success(false);
    response->set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
    response->set_message("Path traversal not allowed");
    return grpc::Status::OK;
  }

  // 2. Check staged file exists
  if (!FileExists(staged_path)) {
    response->set_success(false);
    response->set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
    response->set_message("Staged file not found: " + staged_path);
    return grpc::Status::OK;
  }

  // 3. SHA256 verification
  if (!artifact.sha256().empty()) {
    std::string actual = ComputeSHA256(staged_path);
    if (actual != artifact.sha256()) {
      response->set_success(false);
      response->set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
      response->set_failure_code(robot::v1::OTA_FAILURE_SHA256_MISMATCH);
      response->set_message("SHA256 mismatch");
      return grpc::Status::OK;
    }
  }

  // 4. HW compatibility
  if (!request->force() && !artifact.hw_compat().empty()) {
    bool compat = false;
    for (const auto &hw : artifact.hw_compat())
      if (hw == "*" || hw == config_.hw_id) { compat = true; break; }
    if (!compat) {
      response->set_success(false);
      response->set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
      response->set_failure_code(robot::v1::OTA_FAILURE_HW_INCOMPAT);
      response->set_message("Hardware incompatible");
      return grpc::Status::OK;
    }
  }

  // 5. Dependency check
  if (!request->force()) {
    std::lock_guard<std::mutex> lock(ota_mutex_);
    for (const auto &dep : artifact.dependencies()) {
      auto it = installed_artifacts_.find(dep.artifact_name());
      if (it == installed_artifacts_.end() ||
          (!dep.min_version().empty() && CompareSemver(it->second.version(), dep.min_version()) < 0)) {
        response->set_success(false);
        response->set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
        response->set_failure_code(robot::v1::OTA_FAILURE_DEPENDENCY);
        response->set_message("Missing/outdated dependency: " + dep.artifact_name());
        return grpc::Status::OK;
      }
    }
  }

  // 6. COLD safety: stop navigation.service before installing
  if (artifact.safety_level() == robot::v1::OTA_SAFETY_LEVEL_COLD) {
    LOG_INFO("COLD update: stopping navigation.service...");
    ota::ManageService("navigation.service", "stop");
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  // 7. Transaction log
  const std::string txn_path = config_.ota_backup_dir + "/txn_" + artifact.name() + ".json";
  {
    std::ofstream txn(txn_path);
    if (txn.is_open()) {
      txn << "{\"artifact\":\"" << artifact.name()
          << "\",\"version\":\"" << artifact.version()
          << "\",\"status\":\"installing\",\"started_at\":\"" << NowISO8601() << "\"}";
    }
  }

  // 8. Backup old version
  std::string previous_version;
  {
    std::lock_guard<std::mutex> lock(ota_mutex_);
    auto it = installed_artifacts_.find(artifact.name());
    if (it != installed_artifacts_.end()) previous_version = it->second.version();
  }

  if (artifact.rollback_safe() && FileExists(target_path)) {
    std::string backup_path;
    if (BackupArtifact(artifact.name(), target_path, &backup_path)) {
      std::lock_guard<std::mutex> lock(ota_mutex_);
      robot::v1::RollbackEntry entry;
      entry.set_name(artifact.name());
      entry.set_version(previous_version);
      entry.set_backup_path(backup_path);
      rollback_entries_[artifact.name()] = entry;
    }
  }

  // 9. Install
  auto install_start = std::chrono::steady_clock::now();
  bool install_ok = false;
  std::string install_msg;

  switch (artifact.apply_action()) {
    case robot::v1::OTA_APPLY_ACTION_COPY_ONLY:
    case robot::v1::OTA_APPLY_ACTION_RELOAD_MODEL: {
      auto dir = target_path.substr(0, target_path.rfind('/'));
      std::filesystem::create_directories(dir);
      try {
        std::filesystem::copy_file(staged_path, target_path,
                                   std::filesystem::copy_options::overwrite_existing);
        install_ok = true;
        install_msg = "Copied to " + target_path;
      } catch (const std::exception &e) {
        install_msg = std::string("Copy failed: ") + e.what();
      }
      break;
    }
    case robot::v1::OTA_APPLY_ACTION_INSTALL_DEB: {
      std::string cmd = "dpkg -i '" + staged_path + "' 2>&1";
      int r = std::system(cmd.c_str());
      install_ok = (r == 0);
      install_msg = "dpkg returned " + std::to_string(r);
      if (!install_ok) std::system("apt-get -f install -y 2>&1");
      break;
    }
    case robot::v1::OTA_APPLY_ACTION_FLASH_MCU: {
      std::string script = "/usr/local/bin/apply_firmware.sh";
      if (FileExists(script)) {
        int r = std::system((script + " '" + staged_path + "' 2>&1").c_str());
        install_ok = (r == 0);
        install_msg = "Flash script returned " + std::to_string(r);
      } else {
        install_msg = "Flash script not found: " + script;
      }
      break;
    }
    case robot::v1::OTA_APPLY_ACTION_INSTALL_SCRIPT: {
      auto dir = staged_path.substr(0, staged_path.rfind('/'));
      std::string script = dir + "/install.sh";
      if (FileExists(script)) {
        int r = std::system(("bash '" + script + "' '" + staged_path + "' 2>&1").c_str());
        install_ok = (r == 0);
        install_msg = "Install script returned " + std::to_string(r);
      } else {
        // Fallback: copy
        auto tdir = target_path.substr(0, target_path.rfind('/'));
        std::filesystem::create_directories(tdir);
        try {
          std::filesystem::copy_file(staged_path, target_path,
                                     std::filesystem::copy_options::overwrite_existing);
          install_ok = true;
          install_msg = "Copied (no install.sh)";
        } catch (const std::exception &e) {
          install_msg = std::string("Copy failed: ") + e.what();
        }
      }
      break;
    }
    default: {
      auto tdir = target_path.substr(0, target_path.rfind('/'));
      std::filesystem::create_directories(tdir);
      try {
        std::filesystem::copy_file(staged_path, target_path,
                                   std::filesystem::copy_options::overwrite_existing);
        install_ok = true;
        install_msg = "Copied to " + target_path;
      } catch (const std::exception &e) {
        install_msg = std::string("Copy failed: ") + e.what();
      }
    }
  }

  if (!install_ok) {
    std::filesystem::remove(txn_path);
    AppendUpgradeHistory("install", artifact.name(), previous_version,
                         artifact.version(), "failed", robot::v1::OTA_FAILURE_INSTALL_SCRIPT,
                         install_msg, 0, "skipped");
    response->set_success(false);
    response->set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
    response->set_message(install_msg);
    response->set_failure_code(robot::v1::OTA_FAILURE_INSTALL_SCRIPT);

    // COLD: restart navigation after failure
    if (artifact.safety_level() == robot::v1::OTA_SAFETY_LEVEL_COLD) {
      ota::ManageService("navigation.service", "start");
    }
    return grpc::Status::OK;
  }

  // 10. Health check
  std::string health_reason;
  bool health_ok = PostInstallHealthCheck(artifact.safety_level(), &health_reason);

  if (!health_ok) {
    LOG_WARN("Health check FAILED for %s: %s — auto-rolling back", artifact.name().c_str(), health_reason.c_str());
    bool rb_ok = false;
    {
      std::lock_guard<std::mutex> lock(ota_mutex_);
      auto rb_it = rollback_entries_.find(artifact.name());
      if (rb_it != rollback_entries_.end() && FileExists(rb_it->second.backup_path())) {
        try {
          std::filesystem::copy_file(rb_it->second.backup_path(), target_path,
                                     std::filesystem::copy_options::overwrite_existing);
          rb_ok = true;
        } catch (...) {}
      }
    }
    std::filesystem::remove(txn_path);
    AppendUpgradeHistory("install", artifact.name(), previous_version,
                         artifact.version(), "rolled_back_health_fail",
                         robot::v1::OTA_FAILURE_HEALTH_CHECK, health_reason, 0, "failed");

    response->set_success(false);
    response->set_status(robot::v1::OTA_UPDATE_STATUS_ROLLED_BACK);
    response->set_message("Health check failed: " + health_reason);
    response->set_failure_code(robot::v1::OTA_FAILURE_HEALTH_CHECK);
    return grpc::Status::OK;
  }

  // 11. Update manifest (under lock)
  std::filesystem::remove(txn_path);
  {
    std::lock_guard<std::mutex> lock(ota_mutex_);
    robot::v1::InstalledArtifact inst;
    inst.set_name(artifact.name());
    inst.set_category(artifact.category());
    inst.set_version(artifact.version());
    inst.set_path(target_path);
    inst.set_sha256(artifact.sha256());
    inst.set_installed_at(NowISO8601());
    installed_artifacts_[artifact.name()] = inst;
    SaveInstalledManifest();
    SaveSystemVersionJson();
  }

  auto dur_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - install_start).count();

  AppendUpgradeHistory("install", artifact.name(), previous_version,
                       artifact.version(), "success", robot::v1::OTA_FAILURE_NONE,
                       "", dur_ms, health_reason);

  // COLD: restart navigation.service
  if (artifact.safety_level() == robot::v1::OTA_SAFETY_LEVEL_COLD) {
    LOG_INFO("COLD update complete, restarting navigation.service...");
    ota::ManageService("navigation.service", "start");
  }

  LOG_INFO("ApplyUpdate SUCCESS: %s v%s -> %s", artifact.name().c_str(),
           artifact.version().c_str(), target_path.c_str());

  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  response->set_success(true);
  response->set_status(robot::v1::OTA_UPDATE_STATUS_SUCCESS);
  response->set_message(install_msg);
  response->set_installed_path(target_path);
  response->set_previous_version(previous_version);
  response->set_failure_code(robot::v1::OTA_FAILURE_NONE);
  return grpc::Status::OK;
}

// =====================================================================
// RPC: GetInstalledVersions
// =====================================================================
grpc::Status OtaServiceImpl::GetInstalledVersions(
    grpc::ServerContext *, const robot::v1::GetInstalledVersionsRequest *request,
    robot::v1::GetInstalledVersionsResponse *response) {

  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  response->set_robot_id(config_.robot_id);
  response->set_hw_id(config_.hw_id);
  response->set_system_version(system_version_);
  response->set_system_version_json(ReadFileToString(config_.system_version_path));

  std::lock_guard<std::mutex> lock(ota_mutex_);
  auto cat = request->category_filter();
  for (const auto &[name, a] : installed_artifacts_) {
    if (cat != robot::v1::OTA_CATEGORY_UNSPECIFIED && a.category() != cat) continue;
    response->add_installed()->CopyFrom(a);
  }
  for (const auto &[name, r] : rollback_entries_) {
    response->add_rollback_available()->CopyFrom(r);
  }
  return grpc::Status::OK;
}

// =====================================================================
// RPC: Rollback
// =====================================================================
grpc::Status OtaServiceImpl::Rollback(
    grpc::ServerContext *, const robot::v1::RollbackRequest *request,
    robot::v1::RollbackResponse *response) {

  const std::string &name = request->artifact_name();
  if (name.empty()) {
    response->set_success(false);
    response->set_message("artifact_name is required");
    return grpc::Status::OK;
  }

  std::lock_guard<std::mutex> lock(ota_mutex_);

  auto rb_it = rollback_entries_.find(name);
  if (rb_it == rollback_entries_.end()) {
    response->set_success(false);
    response->set_message("No rollback entry for: " + name);
    return grpc::Status::OK;
  }

  if (!FileExists(rb_it->second.backup_path())) {
    response->set_success(false);
    response->set_message("Backup file missing");
    rollback_entries_.erase(rb_it);
    SaveInstalledManifest();
    return grpc::Status::OK;
  }

  auto inst_it = installed_artifacts_.find(name);
  if (inst_it == installed_artifacts_.end()) {
    response->set_success(false);
    response->set_message("No installed record for: " + name);
    return grpc::Status::OK;
  }

  std::string target = inst_it->second.path();
  try {
    std::filesystem::copy_file(rb_it->second.backup_path(), target,
                               std::filesystem::copy_options::overwrite_existing);
  } catch (const std::exception &e) {
    response->set_success(false);
    response->set_message(std::string("Rollback copy failed: ") + e.what());
    return grpc::Status::OK;
  }

  std::string current_ver = inst_it->second.version();
  std::string restored_ver = rb_it->second.version();
  inst_it->second.set_version(restored_ver);
  inst_it->second.set_installed_at(NowISO8601());
  rollback_entries_.erase(rb_it);

  SaveInstalledManifest();
  SaveSystemVersionJson();

  AppendUpgradeHistory("rollback", name, current_ver, restored_ver,
                       "success", robot::v1::OTA_FAILURE_NONE, "", 0, "skipped");

  LOG_INFO("Rollback SUCCESS: %s -> v%s", name.c_str(), restored_ver.c_str());

  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  response->set_success(true);
  response->set_message("Rolled back to " + restored_ver);
  response->set_restored_version(restored_ver);
  return grpc::Status::OK;
}

// =====================================================================
// RPC: GetUpgradeHistory
// =====================================================================
grpc::Status OtaServiceImpl::GetUpgradeHistory(
    grpc::ServerContext *, const robot::v1::GetUpgradeHistoryRequest *request,
    robot::v1::GetUpgradeHistoryResponse *response) {

  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);

  std::ifstream file(config_.ota_history_path);
  if (!file.is_open()) return grpc::Status::OK;

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty()) lines.push_back(line);
  }

  // Reverse for newest-first
  std::reverse(lines.begin(), lines.end());

  uint32_t limit = request->limit() > 0 ? request->limit() : 100;
  for (uint32_t i = 0; i < limit && i < lines.size(); ++i) {
    auto *entry = response->add_entries();
    // Simple JSON field extraction
    auto extract = [&](const std::string &key) -> std::string {
      std::string needle = "\"" + key + "\":\"";
      auto pos = lines[i].find(needle);
      if (pos == std::string::npos) {
        // Try numeric value
        needle = "\"" + key + "\":";
        pos = lines[i].find(needle);
        if (pos == std::string::npos) return "";
        auto start = pos + needle.size();
        auto end = lines[i].find_first_of(",}", start);
        return lines[i].substr(start, end - start);
      }
      auto start = pos + needle.size();
      auto end = lines[i].find('"', start);
      return lines[i].substr(start, end - start);
    };

    entry->set_timestamp(extract("ts"));
    entry->set_action(extract("action"));
    entry->set_artifact_name(extract("artifact"));
    entry->set_from_version(extract("from"));
    entry->set_to_version(extract("to"));
    entry->set_status(extract("status"));
    entry->set_failure_reason(extract("failure_reason"));
    entry->set_health_check_result(extract("health_check"));

    std::string fc_str = extract("failure_code");
    if (!fc_str.empty()) {
      entry->set_failure_code(static_cast<robot::v1::OtaFailureCode>(std::stoi(fc_str)));
    }
    std::string dur_str = extract("duration_ms");
    if (!dur_str.empty()) entry->set_duration_ms(std::stoull(dur_str));
  }

  return grpc::Status::OK;
}

// =====================================================================
// RPC: ValidateSystemVersion
// =====================================================================
grpc::Status OtaServiceImpl::ValidateSystemVersion(
    grpc::ServerContext *, const robot::v1::ValidateSystemVersionRequest *request,
    robot::v1::ValidateSystemVersionResponse *response) {

  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  response->set_actual_system_version(system_version_);

  bool all_match = true;
  std::lock_guard<std::mutex> lock(ota_mutex_);

  for (const auto &expected : request->expected_components()) {
    auto *m = response->add_mismatches();
    m->set_component_name(expected.name());
    m->set_expected_version(expected.version());

    auto it = installed_artifacts_.find(expected.name());
    if (it == installed_artifacts_.end()) {
      m->set_actual_version("");
      m->set_status("missing");
      all_match = false;
    } else {
      m->set_actual_version(it->second.version());
      if (it->second.version() == expected.version()) {
        m->set_status("match");
      } else {
        m->set_status("mismatch");
        all_match = false;
      }
    }
  }

  response->set_consistent(all_match);
  return grpc::Status::OK;
}

// =====================================================================
// RPC: GetDeviceInfo
// =====================================================================
grpc::Status OtaServiceImpl::GetDeviceInfo(
    grpc::ServerContext *, const google::protobuf::Empty *,
    robot::v1::DeviceInfoResponse *response) {

  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  response->set_hostname(GetHostname());
  response->set_robot_id(config_.robot_id);
  response->set_hw_id(config_.hw_id);

  for (const auto &ip : GetIPAddresses()) {
    response->add_ip_addresses(ip);
  }

  response->set_disk_total_bytes(0);  // TODO: implement
  response->set_disk_free_bytes(GetDiskFreeBytes("/opt/robot"));
  response->set_battery_percent(GetBatteryPercent());
  response->set_uptime_seconds(GetUptimeSeconds());
  response->set_os_version(GetOSVersion());
  response->set_ota_daemon_version(OTA_DAEMON_VERSION);

  // Service statuses
  for (const auto &svc : config_.managed_services) {
    auto status = ota::GetServiceStatus(svc);
    auto *s = response->add_services();
    s->set_name(svc);
    s->set_state(status.active_state);
    s->set_sub_state(status.sub_state);
    s->set_uptime_seconds(status.uptime_seconds);
    s->set_restart_count(status.restart_count);
  }

  return grpc::Status::OK;
}

// =====================================================================
// RPC: ManageService
// =====================================================================
grpc::Status OtaServiceImpl::ManageService(
    grpc::ServerContext *, const robot::v1::ManageServiceRequest *request,
    robot::v1::ManageServiceResponse *response) {

  const std::string &svc = request->service_name();

  // Security: only allow managed services
  bool allowed = false;
  for (const auto &s : config_.managed_services) {
    if (s == svc) { allowed = true; break; }
  }
  if (!allowed) {
    response->set_success(false);
    response->set_message("Service not in managed list: " + svc);
    return grpc::Status::OK;
  }

  std::string action;
  switch (request->action()) {
    case robot::v1::SERVICE_ACTION_START: action = "start"; break;
    case robot::v1::SERVICE_ACTION_STOP: action = "stop"; break;
    case robot::v1::SERVICE_ACTION_RESTART: action = "restart"; break;
    case robot::v1::SERVICE_ACTION_STATUS: {
      auto st = ota::GetServiceStatus(svc);
      response->set_success(true);
      response->set_message("Status queried");
      auto *s = response->mutable_status();
      s->set_name(svc);
      s->set_state(st.active_state);
      s->set_sub_state(st.sub_state);
      s->set_uptime_seconds(st.uptime_seconds);
      s->set_restart_count(st.restart_count);
      return grpc::Status::OK;
    }
    default:
      response->set_success(false);
      response->set_message("Unknown action");
      return grpc::Status::OK;
  }

  LOG_INFO("ManageService: %s %s", action.c_str(), svc.c_str());
  bool ok = ota::ManageService(svc, action);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  auto st = ota::GetServiceStatus(svc);

  response->set_success(ok);
  response->set_message(ok ? action + " succeeded" : action + " failed");
  auto *s = response->mutable_status();
  s->set_name(svc);
  s->set_state(st.active_state);
  s->set_sub_state(st.sub_state);
  s->set_uptime_seconds(st.uptime_seconds);
  s->set_restart_count(st.restart_count);
  return grpc::Status::OK;
}

} // namespace ota
