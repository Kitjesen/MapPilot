// reporter.cpp — OTA 部署结果上报
// 向 infra/ota Server 的 POST /api/agent/report 接口发送部署结果。
// 使用已有的 libcurl 依赖（DownloadFromUrl 同样依赖）。
// fire-and-forget：上报失败仅打警告，不影响部署流程。

#include "reporter.hpp"
#include "utils.hpp"

#include <curl/curl.h>
#include <fstream>
#include <memory>
#include <sstream>

namespace ota {

// ── 内部辅助：丢弃 curl 响应体 ────────────────────────────────────────
static size_t DevNull(void*, size_t size, size_t nmemb, void*) {
  return size * nmemb;
}

// ─────────────────────────────────────────────────────────────────────

void OtaReporter::Configure(const OtaReporterConfig& cfg) {
  cfg_ = cfg;
}

void OtaReporter::LoadDeviceId() {
  if (!device_id_.empty()) return;
  if (cfg_.device_id_file.empty()) return;

  std::ifstream f(cfg_.device_id_file);
  if (f.is_open()) {
    std::getline(f, device_id_);
    // 去除可能的换行/空格
    while (!device_id_.empty() &&
           (device_id_.back() == '\n' || device_id_.back() == '\r' ||
            device_id_.back() == ' '))
      device_id_.pop_back();
  }

  if (!cfg_.api_key_file.empty() && api_key_.empty()) {
    std::ifstream kf(cfg_.api_key_file);
    if (kf.is_open()) {
      std::getline(kf, api_key_);
      while (!api_key_.empty() &&
             (api_key_.back() == '\n' || api_key_.back() == '\r' ||
              api_key_.back() == ' '))
        api_key_.pop_back();
    }
  }
}

void OtaReporter::Report(const std::string& package_name,
                         const std::string& from_version,
                         const std::string& to_version,
                         const std::string& status,
                         const std::string& error_message) {
  if (!IsEnabled()) return;

  LoadDeviceId();

  // 构造 JSON body
  std::ostringstream body;
  body << "{"
       << "\"device_id\":\""    << EscapeJson(device_id_)    << "\","
       << "\"package_name\":\"" << EscapeJson(package_name)  << "\","
       << "\"from_version\":\"" << EscapeJson(from_version)  << "\","
       << "\"current_version\":\"" << EscapeJson(to_version) << "\","
       << "\"status\":\""       << EscapeJson(status)        << "\","
       << "\"error_message\":\"" << EscapeJson(error_message) << "\","
       << "\"timestamp\":\""    << NowISO8601()               << "\","
       << "\"source\":\"daemon\""   // 区分 Python Agent 上报
       << "}";

  if (!PostJson("/api/agent/report", body.str())) {
    OtaLogWarn("OtaReporter: 上报失败 (artifact=%s status=%s)，继续运行",
               package_name.c_str(), status.c_str());
  } else {
    OtaLogInfo("OtaReporter: 已上报 %s %s→%s [%s]",
               package_name.c_str(), from_version.c_str(),
               to_version.c_str(), status.c_str());
  }
}

bool OtaReporter::PostJson(const std::string& endpoint,
                           const std::string& body) {
  // RAII wrappers for curl resources
  std::unique_ptr<CURL, decltype(&curl_easy_cleanup)> curl(
      curl_easy_init(), &curl_easy_cleanup);
  if (!curl) return false;

  std::string url = cfg_.server_url + endpoint;

  std::unique_ptr<struct curl_slist, decltype(&curl_slist_free_all)> headers(
      nullptr, &curl_slist_free_all);
  headers.reset(curl_slist_append(headers.release(), "Content-Type: application/json"));
  if (!api_key_.empty()) {
    std::string auth = "X-API-Key: " + api_key_;
    headers.reset(curl_slist_append(headers.release(), auth.c_str()));
  }

  curl_easy_setopt(curl.get(), CURLOPT_URL,            url.c_str());
  curl_easy_setopt(curl.get(), CURLOPT_POST,           1L);
  curl_easy_setopt(curl.get(), CURLOPT_POSTFIELDS,     body.c_str());
  curl_easy_setopt(curl.get(), CURLOPT_HTTPHEADER,     headers.get());
  curl_easy_setopt(curl.get(), CURLOPT_TIMEOUT,        10L);
  curl_easy_setopt(curl.get(), CURLOPT_CONNECTTIMEOUT, 5L);
  curl_easy_setopt(curl.get(), CURLOPT_WRITEFUNCTION,  DevNull);
  curl_easy_setopt(curl.get(), CURLOPT_NOSIGNAL,       1L);  // 线程安全

  CURLcode res = curl_easy_perform(curl.get());
  return res == CURLE_OK;
}

std::string OtaReporter::EscapeJson(const std::string& s) {
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    switch (c) {
      case '"':  out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n";  break;
      case '\r': out += "\\r";  break;
      case '\t': out += "\\t";  break;
      default:   out += c;
    }
  }
  return out;
}

}  // namespace ota
