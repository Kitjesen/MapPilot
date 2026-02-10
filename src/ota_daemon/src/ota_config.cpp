// ota_config.cpp — OTA 配置加载 (YAML)
// 从 ota_service.cpp 拆分

#include "ota_service.hpp"
#include "utils.hpp"

#include <yaml-cpp/yaml.h>

namespace ota {

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

    // Dog Board OTA proxy
    if (root["dog_board"]) {
      auto dog = root["dog_board"];
      if (dog["host"]) cfg.dog_board_host = dog["host"].as<std::string>();
      if (dog["port"]) cfg.dog_board_port = dog["port"].as<int>();
      if (dog["reload_script"]) cfg.dog_reload_script = dog["reload_script"].as<std::string>();
      if (dog["timeout_sec"]) cfg.dog_timeout_sec = dog["timeout_sec"].as<int>();
    }
  } catch (const std::exception &e) {
    OtaLogError("Failed to parse config %s: %s", yaml_path.c_str(), e.what());
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

      OtaLogInfo("Loaded artifact paths: %zu exact, %zu category defaults",
               cfg.artifact_paths.size(), cfg.category_defaults.size());
    } else {
      OtaLogWarn("artifact_paths.yaml not found at %s, using manifest target_path as fallback",
               paths_yaml.c_str());
    }
  } catch (const std::exception &e) {
    OtaLogError("Failed to parse %s: %s", paths_yaml.c_str(), e.what());
  }

  return cfg;
}

} // namespace ota
