#include "utils.hpp"

#include <array>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sstream>
#include <sys/statvfs.h>
#include <sys/utsname.h>
#include <unistd.h>

#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/sha.h>

namespace ota {

// ──────────────── 日志 ────────────────

static LogLevel g_log_level = LogLevel::kInfo;

void SetLogLevel(LogLevel level) { g_log_level = level; }

void Log(LogLevel level, const char *fmt, ...) {
  if (level < g_log_level) return;

  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) % 1000;
  struct tm tm_buf;
  localtime_r(&time_t_now, &tm_buf);

  const char *level_str = "???";
  FILE *out = stdout;
  switch (level) {
    case LogLevel::kDebug: level_str = "DEBUG"; break;
    case LogLevel::kInfo:  level_str = "INFO";  break;
    case LogLevel::kWarn:  level_str = "WARN";  out = stderr; break;
    case LogLevel::kError: level_str = "ERROR"; out = stderr; break;
  }

  fprintf(out, "%04d-%02d-%02d %02d:%02d:%02d.%03d [%s] ",
          tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
          tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec,
          static_cast<int>(ms.count()), level_str);

  va_list args;
  va_start(args, fmt);
  vfprintf(out, fmt, args);
  va_end(args);
  fprintf(out, "\n");
  fflush(out);
}

// ──────────────── SHA256 ────────────────

std::string ComputeSHA256(const std::string &file_path) {
  std::ifstream ifs(file_path, std::ios::binary);
  if (!ifs) return "";

  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256_CTX ctx;
  SHA256_Init(&ctx);

  char buf[8192];
  while (ifs.read(buf, sizeof(buf)) || ifs.gcount() > 0) {
    SHA256_Update(&ctx, buf, static_cast<size_t>(ifs.gcount()));
  }
  SHA256_Final(hash, &ctx);

  std::ostringstream oss;
  for (int i = 0; i < SHA256_DIGEST_LENGTH; ++i)
    oss << std::hex << std::setfill('0') << std::setw(2)
        << static_cast<int>(hash[i]);
  return oss.str();
}

std::string ComputeSHA256FromBytes(const void *data, size_t len) {
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256(static_cast<const unsigned char *>(data), len, hash);

  std::ostringstream oss;
  for (int i = 0; i < SHA256_DIGEST_LENGTH; ++i)
    oss << std::hex << std::setfill('0') << std::setw(2)
        << static_cast<int>(hash[i]);
  return oss.str();
}

// ──────────────── Ed25519 ────────────────

bool VerifyEd25519(const std::string &public_key_pem,
                   const std::string &message,
                   const std::string &signature_hex) {
  if (public_key_pem.empty() || signature_hex.empty()) return false;

  FILE *fp = fopen(public_key_pem.c_str(), "r");
  if (!fp) {
    OtaLogWarn("Ed25519: cannot open public key: %s", public_key_pem.c_str());
    return false;
  }

  EVP_PKEY *pkey = PEM_read_PUBKEY(fp, nullptr, nullptr, nullptr);
  fclose(fp);
  if (!pkey) {
    OtaLogWarn("Ed25519: failed to parse public key");
    return false;
  }

  // Decode hex signature
  std::vector<unsigned char> sig_bytes;
  sig_bytes.reserve(signature_hex.size() / 2);
  for (size_t i = 0; i + 1 < signature_hex.size(); i += 2) {
    unsigned int byte;
    if (sscanf(signature_hex.c_str() + i, "%2x", &byte) == 1)
      sig_bytes.push_back(static_cast<unsigned char>(byte));
  }

  EVP_MD_CTX *md_ctx = EVP_MD_CTX_new();
  bool ok = false;
  if (md_ctx) {
    if (EVP_DigestVerifyInit(md_ctx, nullptr, nullptr, nullptr, pkey) == 1) {
      int rc = EVP_DigestVerify(
          md_ctx,
          sig_bytes.data(), sig_bytes.size(),
          reinterpret_cast<const unsigned char *>(message.data()),
          message.size());
      ok = (rc == 1);
    }
    EVP_MD_CTX_free(md_ctx);
  }
  EVP_PKEY_free(pkey);
  return ok;
}

// ──────────────── Semver ────────────────

int CompareSemver(const std::string &a, const std::string &b) {
  auto parse = [](const std::string &s, int out[3]) {
    out[0] = out[1] = out[2] = 0;
    std::string v = s;
    if (!v.empty() && v[0] == 'v') v = v.substr(1);
    sscanf(v.c_str(), "%d.%d.%d", &out[0], &out[1], &out[2]);
  };
  int va[3], vb[3];
  parse(a, va);
  parse(b, vb);
  for (int i = 0; i < 3; ++i) {
    if (va[i] < vb[i]) return -1;
    if (va[i] > vb[i]) return  1;
  }
  return 0;
}

// ──────────────── 系统工具 ────────────────

uint64_t GetDiskFreeBytes(const std::string &path) {
  struct statvfs stat;
  if (statvfs(path.c_str(), &stat) != 0) return 0;
  return static_cast<uint64_t>(stat.f_bavail) * stat.f_frsize;
}

uint64_t GetDiskTotalBytes(const std::string &path) {
  struct statvfs stat;
  if (statvfs(path.c_str(), &stat) != 0) return 0;
  return static_cast<uint64_t>(stat.f_blocks) * stat.f_frsize;
}

int GetBatteryPercent() {
  std::ifstream ifs("/sys/class/power_supply/battery/capacity");
  if (!ifs) return -1;
  int pct = -1;
  ifs >> pct;
  return pct;
}

uint64_t GetUptimeSeconds() {
  std::ifstream ifs("/proc/uptime");
  if (!ifs) return 0;
  double up = 0;
  ifs >> up;
  return static_cast<uint64_t>(up);
}

std::string GetHostname() {
  char buf[256] = {};
  gethostname(buf, sizeof(buf) - 1);
  return buf;
}

std::vector<std::string> GetIPAddresses() {
  std::vector<std::string> result;
  struct ifaddrs *ifas = nullptr;
  if (getifaddrs(&ifas) != 0) return result;

  for (auto *ifa = ifas; ifa; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr) continue;
    if (ifa->ifa_addr->sa_family != AF_INET) continue;

    std::string name = ifa->ifa_name;
    if (name == "lo") continue;

    char addr_buf[INET_ADDRSTRLEN];
    auto *sa = reinterpret_cast<struct sockaddr_in *>(ifa->ifa_addr);
    inet_ntop(AF_INET, &sa->sin_addr, addr_buf, sizeof(addr_buf));
    result.emplace_back(std::string(name) + ":" + addr_buf);
  }
  freeifaddrs(ifas);
  return result;
}

std::string GetOSVersion() {
  struct utsname u;
  if (uname(&u) != 0) return "unknown";
  return std::string(u.sysname) + " " + u.release;
}

// ──────────────── systemd ────────────────

static std::string ExecCommand(const std::string &cmd) {
  std::array<char, 256> buf;
  std::string result;
  FILE *pipe = popen(cmd.c_str(), "r");
  if (!pipe) return "";
  while (fgets(buf.data(), buf.size(), pipe))
    result += buf.data();
  pclose(pipe);
  // Trim trailing newline
  while (!result.empty() && (result.back() == '\n' || result.back() == '\r'))
    result.pop_back();
  return result;
}

SystemdStatus GetServiceStatus(const std::string &service_name) {
  SystemdStatus s;
  s.active_state = ExecCommand(
      "systemctl show -p ActiveState --value " + service_name + " 2>/dev/null");
  s.sub_state = ExecCommand(
      "systemctl show -p SubState --value " + service_name + " 2>/dev/null");

  // Uptime: ActiveEnterTimestampMonotonic
  std::string ts = ExecCommand(
      "systemctl show -p ActiveEnterTimestampMonotonic --value " +
      service_name + " 2>/dev/null");
  if (!ts.empty()) {
    uint64_t enter_us = std::stoull(ts);
    // Get current monotonic time
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    uint64_t now_us =
        static_cast<uint64_t>(tp.tv_sec) * 1000000 + tp.tv_nsec / 1000;
    if (now_us > enter_us && enter_us > 0)
      s.uptime_seconds = (now_us - enter_us) / 1000000;
  }

  std::string nrestarts = ExecCommand(
      "systemctl show -p NRestarts --value " + service_name + " 2>/dev/null");
  if (!nrestarts.empty()) s.restart_count = std::stoul(nrestarts);

  return s;
}

bool ManageService(const std::string &service_name,
                   const std::string &action) {
  // Validate action
  if (action != "start" && action != "stop" && action != "restart") {
    OtaLogError("Invalid service action: %s", action.c_str());
    return false;
  }
  // Validate service name (security: only allow alphanumeric, dash, dot)
  for (char c : service_name) {
    if (!std::isalnum(c) && c != '-' && c != '_' && c != '.') {
      OtaLogError("Invalid service name: %s", service_name.c_str());
      return false;
    }
  }

  std::string cmd = "systemctl " + action + " " + service_name + " 2>&1";
  OtaLogInfo("Executing: %s", cmd.c_str());
  int ret = system(cmd.c_str());
  return ret == 0;
}

// ──────────────── 时间 ────────────────

std::string NowISO8601() {
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  struct tm tm_buf;
  gmtime_r(&time_t_now, &tm_buf);
  char buf[64];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm_buf);
  return buf;
}

// ──────────────── 文件操作 ────────────────

bool FileExists(const std::string &path) {
  return access(path.c_str(), F_OK) == 0;
}

bool MakeDirs(const std::string &path) {
  std::string cmd = "mkdir -p '" + path + "'";
  return system(cmd.c_str()) == 0;
}

std::string ReadFileToString(const std::string &path) {
  std::ifstream ifs(path);
  if (!ifs) return "";
  return std::string(std::istreambuf_iterator<char>(ifs),
                     std::istreambuf_iterator<char>());
}

bool WriteStringToFile(const std::string &path, const std::string &content) {
  std::ofstream ofs(path);
  if (!ofs) return false;
  ofs << content;
  return ofs.good();
}

} // namespace ota
