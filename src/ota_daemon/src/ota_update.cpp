// ota_update.cpp - OTA update management RPC implementations
//   CheckUpdateReadiness / ApplyUpdate / GetInstalledVersions
//   Rollback / GetUpgradeHistory / ValidateSystemVersion
// Split from ota_service.cpp

#include "ota_service.hpp"
#include "utils.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>

namespace ota {

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
    c->set_detail("free=" + std::to_string(free_bytes) +
                  " required=" + std::to_string(need));
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
    bool ok = (battery < 0) || (battery >= min_bat);
    c->set_passed(ok);
    c->set_message(ok ? "Battery OK" : "Battery too low");
    c->set_detail("battery=" + std::to_string(battery) +
                  " min=" + std::to_string(min_bat));
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
                   CompareSemver(it->second.version(),
                                 dep.min_version()) < 0) {
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
    if (request->manifest_signature().empty() ||
        config_.ota_public_key_path.empty()) {
      c->set_passed(true);
      c->set_message("Signature check skipped (no signature or key)");
    } else {
      bool ok = VerifyEd25519(config_.ota_public_key_path,
                               request->manifest_signature(),
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
// RPC: ApplyUpdate  (largest single RPC)
// =====================================================================
grpc::Status OtaServiceImpl::ApplyUpdate(
    grpc::ServerContext *, const robot::v1::ApplyUpdateRequest *request,
    robot::v1::ApplyUpdateResponse *response) {

  const auto &artifact = request->artifact();
  const std::string &staged_path = request->staged_path();
  const std::string target_path = ResolveTargetPath(artifact);

  OtaLogInfo("ApplyUpdate: name=%s version=%s category=%d target=%s",
           artifact.name().c_str(), artifact.version().c_str(),
           static_cast<int>(artifact.category()), target_path.c_str());

  // 1. Path validation
  if (staged_path.empty() || target_path.empty()) {
    response->set_success(false);
    response->set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
    response->set_message(
        "staged_path is empty or target_path could not be resolved "
        "(check artifact_paths.yaml for artifact: " +
        artifact.name() + ")");
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
          (!dep.min_version().empty() &&
           CompareSemver(it->second.version(), dep.min_version()) < 0)) {
        response->set_success(false);
        response->set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
        response->set_failure_code(robot::v1::OTA_FAILURE_DEPENDENCY);
        response->set_message("Missing/outdated dependency: " +
                              dep.artifact_name());
        return grpc::Status::OK;
      }
    }
  }

  // 6. COLD safety: stop navigation.service before installing
  if (artifact.safety_level() == robot::v1::OTA_SAFETY_LEVEL_COLD) {
    OtaLogInfo("COLD update: stopping navigation.service...");
    ota::ManageService("navigation.service", "stop");
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  // 7. Transaction log
  const std::string txn_path =
      config_.ota_backup_dir + "/txn_" + artifact.name() + ".json";
  {
    std::ofstream txn(txn_path);
    if (txn.is_open()) {
      txn << "{\"artifact\":\"" << artifact.name()
          << "\",\"version\":\"" << artifact.version()
          << "\",\"status\":\"installing\",\"started_at\":\""
          << NowISO8601() << "\"}";
    }
  }

  // 8. Backup old version
  std::string previous_version;
  {
    std::lock_guard<std::mutex> lock(ota_mutex_);
    auto it = installed_artifacts_.find(artifact.name());
    if (it != installed_artifacts_.end())
      previous_version = it->second.version();
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
    case robot::v1::OTA_APPLY_ACTION_COPY_ONLY: {
      auto dir = target_path.substr(0, target_path.rfind('/'));
      std::filesystem::create_directories(dir);
      try {
        std::filesystem::copy_file(
            staged_path, target_path,
            std::filesystem::copy_options::overwrite_existing);
        install_ok = true;
        install_msg = "Copied to " + target_path;
      } catch (const std::exception &e) {
        install_msg = std::string("Copy failed: ") + e.what();
      }
      break;
    }
    case robot::v1::OTA_APPLY_ACTION_RELOAD_MODEL: {
      auto dir = target_path.substr(0, target_path.rfind('/'));
      std::filesystem::create_directories(dir);
      try {
        std::filesystem::copy_file(
            staged_path, target_path,
            std::filesystem::copy_options::overwrite_existing);
        install_ok = true;
        install_msg = "Copied to " + target_path;
      } catch (const std::exception &e) {
        install_msg = std::string("Copy failed: ") + e.what();
        break;
      }
      // Notify Dog Board to hot-reload
      if (artifact.target_board() == "dog" &&
          !config_.dog_reload_script.empty()) {
        OtaLogInfo("RELOAD_MODEL: notifying Dog Board (%s:%d) for %s",
                 config_.dog_board_host.c_str(), config_.dog_board_port,
                 artifact.name().c_str());
        std::string cmd = config_.dog_reload_script +
                          " '" + target_path + "'" +
                          " '" + config_.dog_board_host + "'" +
                          " " + std::to_string(config_.dog_board_port) +
                          " 2>&1";
        int r = std::system(cmd.c_str());
        if (r != 0) {
          OtaLogWarn("Dog Board reload script returned %d (non-fatal)", r);
          install_msg += " (reload notification failed, exit=" +
                         std::to_string(r) + ")";
        } else {
          OtaLogInfo("Dog Board reload succeeded for %s",
                   artifact.name().c_str());
          install_msg += " + Dog Board notified to reload";
        }
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
        int r = std::system(
            (script + " '" + staged_path + "' 2>&1").c_str());
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
        int r = std::system(
            ("bash '" + script + "' '" + staged_path + "' 2>&1").c_str());
        install_ok = (r == 0);
        install_msg = "Install script returned " + std::to_string(r);
      } else {
        auto tdir = target_path.substr(0, target_path.rfind('/'));
        std::filesystem::create_directories(tdir);
        try {
          std::filesystem::copy_file(
              staged_path, target_path,
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
        std::filesystem::copy_file(
            staged_path, target_path,
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
                         artifact.version(), "failed",
                         robot::v1::OTA_FAILURE_INSTALL_SCRIPT,
                         install_msg, 0, "skipped");
    response->set_success(false);
    response->set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
    response->set_message(install_msg);
    response->set_failure_code(robot::v1::OTA_FAILURE_INSTALL_SCRIPT);
    if (artifact.safety_level() == robot::v1::OTA_SAFETY_LEVEL_COLD) {
      ota::ManageService("navigation.service", "start");
    }
    return grpc::Status::OK;
  }

  // 10. Health check
  std::string health_reason;
  bool health_ok =
      PostInstallHealthCheck(artifact.safety_level(), &health_reason);

  if (!health_ok) {
    OtaLogWarn("Health check FAILED for %s: %s - auto-rolling back",
             artifact.name().c_str(), health_reason.c_str());
    {
      std::lock_guard<std::mutex> lock(ota_mutex_);
      auto rb_it = rollback_entries_.find(artifact.name());
      if (rb_it != rollback_entries_.end() &&
          FileExists(rb_it->second.backup_path())) {
        try {
          std::filesystem::copy_file(
              rb_it->second.backup_path(), target_path,
              std::filesystem::copy_options::overwrite_existing);
        } catch (...) {
        }
      }
    }
    std::filesystem::remove(txn_path);
    AppendUpgradeHistory("install", artifact.name(), previous_version,
                         artifact.version(), "rolled_back_health_fail",
                         robot::v1::OTA_FAILURE_HEALTH_CHECK,
                         health_reason, 0, "failed");
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
                    std::chrono::steady_clock::now() - install_start)
                    .count();

  AppendUpgradeHistory("install", artifact.name(), previous_version,
                       artifact.version(), "success",
                       robot::v1::OTA_FAILURE_NONE, "", dur_ms,
                       health_reason);

  if (artifact.safety_level() == robot::v1::OTA_SAFETY_LEVEL_COLD) {
    OtaLogInfo("COLD update complete, restarting navigation.service...");
    ota::ManageService("navigation.service", "start");
  }

  OtaLogInfo("ApplyUpdate SUCCESS: %s v%s -> %s",
           artifact.name().c_str(), artifact.version().c_str(),
           target_path.c_str());

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
    grpc::ServerContext *,
    const robot::v1::GetInstalledVersionsRequest *request,
    robot::v1::GetInstalledVersionsResponse *response) {

  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  response->set_robot_id(config_.robot_id);
  response->set_hw_id(config_.hw_id);
  response->set_system_version(system_version_);
  response->set_system_version_json(
      ReadFileToString(config_.system_version_path));

  std::lock_guard<std::mutex> lock(ota_mutex_);
  auto cat = request->category_filter();
  for (const auto &[name, a] : installed_artifacts_) {
    if (cat != robot::v1::OTA_CATEGORY_UNSPECIFIED && a.category() != cat)
      continue;
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
    std::filesystem::copy_file(
        rb_it->second.backup_path(), target,
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
                       "success", robot::v1::OTA_FAILURE_NONE, "", 0,
                       "skipped");

  OtaLogInfo("Rollback SUCCESS: %s -> v%s", name.c_str(),
           restored_ver.c_str());

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
    grpc::ServerContext *,
    const robot::v1::GetUpgradeHistoryRequest *request,
    robot::v1::GetUpgradeHistoryResponse *response) {

  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);

  std::ifstream file(config_.ota_history_path);
  if (!file.is_open()) return grpc::Status::OK;

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty()) lines.push_back(line);
  }

  std::reverse(lines.begin(), lines.end());

  uint32_t limit = request->limit() > 0 ? request->limit() : 100;
  for (uint32_t i = 0; i < limit && i < lines.size(); ++i) {
    auto *entry = response->add_entries();
    auto extract = [&](const std::string &key) -> std::string {
      std::string needle = "\"" + key + "\":\"";
      auto pos = lines[i].find(needle);
      if (pos == std::string::npos) {
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
      entry->set_failure_code(
          static_cast<robot::v1::OtaFailureCode>(std::stoi(fc_str)));
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
    grpc::ServerContext *,
    const robot::v1::ValidateSystemVersionRequest *request,
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

} // namespace ota
