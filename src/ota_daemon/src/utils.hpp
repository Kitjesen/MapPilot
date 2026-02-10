#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ota {

// ──────────────── 日志 ────────────────
enum class LogLevel { DEBUG, INFO, WARN, ERROR };

void SetLogLevel(LogLevel level);
void Log(LogLevel level, const char *fmt, ...);

#define LOG_DEBUG(...) ::ota::Log(::ota::LogLevel::DEBUG, __VA_ARGS__)
#define LOG_INFO(...)  ::ota::Log(::ota::LogLevel::INFO,  __VA_ARGS__)
#define LOG_WARN(...)  ::ota::Log(::ota::LogLevel::WARN,  __VA_ARGS__)
#define LOG_ERROR(...) ::ota::Log(::ota::LogLevel::ERROR, __VA_ARGS__)

// ──────────────── 文件 / 密码学 ────────────────
std::string ComputeSHA256(const std::string &file_path);
std::string ComputeSHA256FromBytes(const void *data, size_t len);

// Ed25519 验签 (OpenSSL EVP)
// public_key_pem: PEM 格式公钥文件路径
// message: 待验证的原始消息
// signature_hex: hex 编码的签名
bool VerifyEd25519(const std::string &public_key_pem,
                   const std::string &message,
                   const std::string &signature_hex);

// ──────────────── 版本 ────────────────
// 返回 -1 (a<b), 0 (a==b), +1 (a>b)
int CompareSemver(const std::string &a, const std::string &b);

// ──────────────── 系统工具 ────────────────
uint64_t GetDiskFreeBytes(const std::string &path);
uint64_t GetDiskTotalBytes(const std::string &path);
int GetBatteryPercent();  // -1 = 未知
uint64_t GetUptimeSeconds();
std::string GetHostname();
std::vector<std::string> GetIPAddresses();
std::string GetOSVersion();

// ──────────────── systemd 服务管理 ────────────────
struct SystemdStatus {
  std::string active_state;   // "active", "inactive", "failed"
  std::string sub_state;      // "running", "dead", "exited"
  uint64_t uptime_seconds;
  uint32_t restart_count;
};

SystemdStatus GetServiceStatus(const std::string &service_name);
bool ManageService(const std::string &service_name,
                   const std::string &action);  // "start", "stop", "restart"

// ──────────────── 时间 ────────────────
std::string NowISO8601();

// ──────────────── 文件操作 ────────────────
bool FileExists(const std::string &path);
bool MakeDirs(const std::string &path);
std::string ReadFileToString(const std::string &path);
bool WriteStringToFile(const std::string &path, const std::string &content);

} // namespace ota
