#pragma once

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "data.grpc.pb.h"
#include <grpcpp/grpcpp.h>

namespace ota {

// ──────────────── 配置 ────────────────

// 单个制品的本地路径映射
struct ArtifactPathEntry {
  std::string install_path;
  std::string description;
};

struct OtaDaemonConfig {
  int grpc_port = 50052;
  std::string bind_address = "0.0.0.0";

  std::string robot_id = "robot_001";
  std::string hw_id = "dog_v2";

  std::string ota_manifest_path = "/opt/robot/ota/installed_manifest.json";
  std::string ota_backup_dir = "/opt/robot/ota/backup";
  std::string ota_history_path = "/opt/robot/ota/upgrade_history.jsonl";
  std::string system_version_path = "/opt/robot/ota/system_version.json";
  std::string ota_public_key_path;

  // 下载暂存目录 (替代 /tmp, 受 systemd 保护)
  std::string staging_dir = "/opt/robot/ota/staging";

  std::string tls_cert_path;
  std::string tls_key_path;

  std::vector<std::string> managed_services = {"navigation.service"};
  int health_check_timeout_sec = 30;
  std::string log_level = "INFO";
  std::vector<std::string> allowed_directories;

  // ── 制品路径映射 (接收方决定安装位置) ──
  // category → 默认目录
  std::unordered_map<std::string, std::string> category_defaults;
  // artifact name → 精确安装路径
  std::unordered_map<std::string, ArtifactPathEntry> artifact_paths;
};

OtaDaemonConfig LoadConfig(const std::string &yaml_path);

// ──────────────── OTA Service 实现 ────────────────

class OtaServiceImpl final : public robot::v1::OtaService::Service {
public:
  explicit OtaServiceImpl(const OtaDaemonConfig &config);

  // ---- 文件传输 ----
  grpc::Status UploadFile(
      grpc::ServerContext *ctx,
      grpc::ServerReader<robot::v1::UploadFileChunk> *reader,
      robot::v1::UploadFileResponse *response) override;

  grpc::Status ListRemoteFiles(
      grpc::ServerContext *ctx,
      const robot::v1::ListRemoteFilesRequest *request,
      robot::v1::ListRemoteFilesResponse *response) override;

  grpc::Status DeleteRemoteFile(
      grpc::ServerContext *ctx,
      const robot::v1::DeleteRemoteFileRequest *request,
      robot::v1::DeleteRemoteFileResponse *response) override;

  grpc::Status DownloadFromUrl(
      grpc::ServerContext *ctx,
      const robot::v1::DownloadFromUrlRequest *request,
      grpc::ServerWriter<robot::v1::OtaProgress> *writer) override;

  // ---- OTA 更新管理 ----
  grpc::Status CheckUpdateReadiness(
      grpc::ServerContext *ctx,
      const robot::v1::CheckUpdateReadinessRequest *request,
      robot::v1::CheckUpdateReadinessResponse *response) override;

  grpc::Status ApplyUpdate(
      grpc::ServerContext *ctx,
      const robot::v1::ApplyUpdateRequest *request,
      robot::v1::ApplyUpdateResponse *response) override;

  grpc::Status GetInstalledVersions(
      grpc::ServerContext *ctx,
      const robot::v1::GetInstalledVersionsRequest *request,
      robot::v1::GetInstalledVersionsResponse *response) override;

  grpc::Status Rollback(
      grpc::ServerContext *ctx,
      const robot::v1::RollbackRequest *request,
      robot::v1::RollbackResponse *response) override;

  grpc::Status GetUpgradeHistory(
      grpc::ServerContext *ctx,
      const robot::v1::GetUpgradeHistoryRequest *request,
      robot::v1::GetUpgradeHistoryResponse *response) override;

  grpc::Status ValidateSystemVersion(
      grpc::ServerContext *ctx,
      const robot::v1::ValidateSystemVersionRequest *request,
      robot::v1::ValidateSystemVersionResponse *response) override;

  // ---- 设备管理 ----
  grpc::Status GetDeviceInfo(
      grpc::ServerContext *ctx,
      const google::protobuf::Empty *request,
      robot::v1::DeviceInfoResponse *response) override;

  grpc::Status ManageService(
      grpc::ServerContext *ctx,
      const robot::v1::ManageServiceRequest *request,
      robot::v1::ManageServiceResponse *response) override;

private:
  // OTA 内部方法
  bool LoadInstalledManifest();
  bool SaveInstalledManifest();
  void LoadSystemVersionJson();
  void SaveSystemVersionJson();  // 调用方须持有 ota_mutex_

  bool BackupArtifact(const std::string &name,
                      const std::string &current_path,
                      std::string *backup_path);

  bool PostInstallHealthCheck(robot::v1::OtaSafetyLevel safety_level,
                              std::string *failure_reason);

  void AppendUpgradeHistory(const std::string &action,
                            const std::string &artifact_name,
                            const std::string &from_version,
                            const std::string &to_version,
                            const std::string &status,
                            robot::v1::OtaFailureCode failure_code,
                            const std::string &failure_reason,
                            uint64_t duration_ms,
                            const std::string &health_check);

  bool IsPathAllowed(const std::string &path) const;

  // 解析制品实际安装路径 (本地映射 > category 默认 > manifest target_path)
  std::string ResolveTargetPath(const robot::v1::OtaArtifact &artifact) const;

  // 配置
  OtaDaemonConfig config_;

  // OTA 状态 (guarded by ota_mutex_)
  mutable std::mutex ota_mutex_;
  std::string system_version_;
  std::unordered_map<std::string, robot::v1::InstalledArtifact>
      installed_artifacts_;
  std::unordered_map<std::string, robot::v1::RollbackEntry>
      rollback_entries_;
};

} // namespace ota
