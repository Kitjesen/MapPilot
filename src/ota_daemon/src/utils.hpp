#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ota {

// ──────────────── 日志 ────────────────
// Avoid conflict with glog/syslog macros that define INFO, ERROR, DEBUG etc.
enum class LogLevel { kDebug = 0, kInfo, kWarn, kError };

void SetLogLevel(LogLevel level);
void Log(LogLevel level, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

// Use template wrappers instead of macros to avoid macro conflicts
template <typename... Args>
inline void OtaLogDebug(const char *fmt, Args... args) { Log(LogLevel::kDebug, fmt, args...); }
template <typename... Args>
inline void OtaLogInfo(const char *fmt, Args... args) { Log(LogLevel::kInfo, fmt, args...); }
template <typename... Args>
inline void OtaLogWarn(const char *fmt, Args... args) { Log(LogLevel::kWarn, fmt, args...); }
template <typename... Args>
inline void OtaLogError(const char *fmt, Args... args) { Log(LogLevel::kError, fmt, args...); }

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
