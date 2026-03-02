#pragma once
// reporter.hpp — OTA 部署结果上报模块
// 在每次 ApplyUpdate / Rollback 完成后，向 infra/ota Server 发送 HTTP 报告。
// 采用 fire-and-forget 语义：上报失败不影响部署结果。

#include <string>

namespace ota {

struct OtaReporterConfig {
  bool        enabled         = false;
  std::string server_url;           // 例: https://ota.inovxio.com/api
  std::string device_id_file;       // 运行时与 Python Agent 共享的 device_id 文件
  std::string api_key_file;         // 可选: API 密钥文件
};

class OtaReporter {
 public:
  OtaReporter() = default;

  void Configure(const OtaReporterConfig& cfg);

  // 上报一次部署/回滚结果
  // status: "success" | "failure" | "rolled_back"
  void Report(const std::string& package_name,
              const std::string& from_version,
              const std::string& to_version,
              const std::string& status,
              const std::string& error_message = "");

  bool IsEnabled() const { return cfg_.enabled && !cfg_.server_url.empty(); }

 private:
  OtaReporterConfig cfg_;
  std::string       device_id_;   // 延迟加载，避免启动时文件尚未创建
  std::string       api_key_;

  void         LoadDeviceId();
  bool         PostJson(const std::string& endpoint, const std::string& body);
  static std::string EscapeJson(const std::string& s);
};

}  // namespace ota
