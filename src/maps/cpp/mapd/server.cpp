#include "server.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <exception>
#include <cstdlib>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "lingtu/maps/io/json.hpp"

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <afunix.h>
#elif defined(__linux__)
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#endif

namespace lingtu::maps::mapd::query {
namespace {

struct Request {
  Opcode opcode{Opcode::kPing};
  std::uint16_t flags{0U};
  std::uint32_t deadline_ms{0U};
  std::string request_id;
  std::string json;
};

struct Response {
  Status status{Status::kOk};
  std::uint16_t flags{0U};
  std::string json;
  int fd{-1};
};

std::uint16_t ReadU16(const unsigned char* data) {
  return static_cast<std::uint16_t>((static_cast<std::uint16_t>(data[0]) << 8U) |
                                    static_cast<std::uint16_t>(data[1]));
}

std::uint32_t ReadU32(const unsigned char* data) {
  return (static_cast<std::uint32_t>(data[0]) << 24U) |
         (static_cast<std::uint32_t>(data[1]) << 16U) |
         (static_cast<std::uint32_t>(data[2]) << 8U) |
         static_cast<std::uint32_t>(data[3]);
}

void WriteU16(unsigned char* data, std::uint16_t value) {
  data[0] = static_cast<unsigned char>((value >> 8U) & 0xffU);
  data[1] = static_cast<unsigned char>(value & 0xffU);
}

void WriteU32(unsigned char* data, std::uint32_t value) {
  data[0] = static_cast<unsigned char>((value >> 24U) & 0xffU);
  data[1] = static_cast<unsigned char>((value >> 16U) & 0xffU);
  data[2] = static_cast<unsigned char>((value >> 8U) & 0xffU);
  data[3] = static_cast<unsigned char>(value & 0xffU);
}

std::string ErrorJson(const std::string& reason, const std::string& message) {
  return "{\"success\":false,\"reason_code\":\"" + JsonEscape(reason) +
         "\",\"message\":\"" + JsonEscape(message) + "\"}";
}

bool IsCleanField(const std::string& value) {
  return std::none_of(value.begin(), value.end(), [](unsigned char ch) {
    return ch == 0U || ch < 0x20U || ch == 0x7fU;
  });
}

#if defined(__linux__) || defined(_WIN32)
bool SocketInterrupted() {
#if defined(_WIN32)
  return ::WSAGetLastError() == WSAEINTR;
#else
  return errno == EINTR;
#endif
}

std::string SocketError(const std::string& operation) {
#if defined(_WIN32)
  return operation + " failed: Windows socket error " + std::to_string(::WSAGetLastError());
#else
  return operation + " failed: " + std::strerror(errno);
#endif
}

void CloseSocket(QuerySocket fd) noexcept {
#if defined(_WIN32)
  ::closesocket(fd);
#else
  ::close(fd);
#endif
}

void SetSocketTimeout(QuerySocket fd, std::uint32_t timeout_ms) {
#if defined(_WIN32)
  const DWORD timeout = timeout_ms;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout));
  ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout));
#else
  timeval timeout {};
  timeout.tv_sec = static_cast<time_t>(timeout_ms / 1000U);
  timeout.tv_usec = static_cast<suseconds_t>((timeout_ms % 1000U) * 1000U);
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
#endif
}

bool RecvAll(QuerySocket fd, unsigned char* data, std::size_t size) {
  std::size_t offset = 0U;
  while (offset < size) {
#if defined(_WIN32)
    const int n = ::recv(fd, reinterpret_cast<char*>(data + offset),
                         static_cast<int>(size - offset), 0);
#else
    const ssize_t n = ::recv(fd, data + offset, size - offset, 0);
#endif
    if (n == 0) {
      return false;
    }
    if (n < 0) {
      if (SocketInterrupted()) {
        continue;
      }
      return false;
    }
    offset += static_cast<std::size_t>(n);
  }
  return true;
}

bool SendAll(QuerySocket fd, const char* data, std::size_t size) {
  std::size_t offset = 0U;
  while (offset < size) {
#if defined(_WIN32)
    const int n = ::send(fd, data + offset, static_cast<int>(size - offset), 0);
#else
    const ssize_t n = ::send(fd, data + offset, size - offset, MSG_NOSIGNAL);
#endif
    if (n < 0) {
      if (SocketInterrupted()) {
        continue;
      }
      return false;
    }
    if (n == 0) {
      return false;
    }
    offset += static_cast<std::size_t>(n);
  }
  return true;
}

std::optional<Request> ReadRequest(QuerySocket fd, std::size_t max_json_bytes, std::string* error) {
  unsigned char header[kRequestHeaderSize]{};
  if (!RecvAll(fd, header, sizeof(header))) {
    *error = "short request header";
    return std::nullopt;
  }
  if (!std::equal(kRequestMagic, kRequestMagic + 4, reinterpret_cast<const char*>(header))) {
    *error = "bad request magic";
    return std::nullopt;
  }
  if (header[4] != kVersion) {
    *error = "unsupported request version";
    return std::nullopt;
  }
  Request request;
  request.opcode = static_cast<Opcode>(header[5]);
  request.flags = ReadU16(header + 6);
  request.deadline_ms = ReadU32(header + 8);
  const std::uint16_t request_id_len = ReadU16(header + 12);
  const std::uint32_t json_len = ReadU32(header + 14);
  if (request.flags != 0U) {
    *error = "request flags must be zero";
    return std::nullopt;
  }
  if (request_id_len > kMaxRequestIdBytes || json_len > max_json_bytes) {
    *error = "request fields exceed protocol bounds";
    return std::nullopt;
  }
  std::string fields;
  fields.resize(static_cast<std::size_t>(request_id_len) + json_len);
  if (!fields.empty() &&
      !RecvAll(fd, reinterpret_cast<unsigned char*>(fields.data()), fields.size())) {
    *error = "short request fields";
    return std::nullopt;
  }
  request.request_id = fields.substr(0U, request_id_len);
  request.json = fields.substr(request_id_len, json_len);
  if (!IsCleanField(request.request_id)) {
    *error = "request id contains control bytes";
    return std::nullopt;
  }
  if (!lingtu::maps::IsValidJsonObject(request.json)) {
    *error = "request JSON must be an object";
    return std::nullopt;
  }
  switch (request.opcode) {
    case Opcode::kPing:
    case Opcode::kService:
    case Opcode::kOpenArtifact:
      break;
    default:
      *error = "unknown opcode";
      return std::nullopt;
  }
  return request;
}

bool SendResponse(QuerySocket fd, Response response, std::size_t max_json_bytes) {
  if (response.json.size() > max_json_bytes) {
    if (response.fd >= 0) {
#if defined(__linux__)
      ::close(response.fd);
#endif
      response.fd = -1;
    }
    response.status = Status::kError;
    response.flags = 0U;
    response.json = ErrorJson("response_too_large", "mapd query JSON response exceeded limit");
  }
  if (response.json.size() > max_json_bytes ||
      response.json.size() > static_cast<std::size_t>(UINT32_MAX)) {
    if (response.fd >= 0) {
#if defined(__linux__)
      ::close(response.fd);
#endif
    }
    return false;
  }
  if (response.fd >= 0) {
    response.flags |= kResponseFlagHasFd;
  } else {
    response.flags &= static_cast<std::uint16_t>(~kResponseFlagHasFd);
  }
  unsigned char header[kResponseHeaderSize]{};
  std::copy(kResponseMagic, kResponseMagic + 4, reinterpret_cast<char*>(header));
  header[4] = kVersion;
  header[5] = static_cast<std::uint8_t>(response.status);
  WriteU16(header + 6, response.flags);
  WriteU32(header + 8, static_cast<std::uint32_t>(response.json.size()));
  WriteU32(header + 12, 0U);

#if defined(__linux__)
  struct iovec iov {};
  iov.iov_base = header;
  iov.iov_len = sizeof(header);
  struct msghdr msg {};
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;
  alignas(struct cmsghdr) unsigned char control[CMSG_SPACE(sizeof(int))]{};
  if (response.fd >= 0) {
    msg.msg_control = control;
    msg.msg_controllen = sizeof(control);
    struct cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
    cmsg->cmsg_level = SOL_SOCKET;
    cmsg->cmsg_type = SCM_RIGHTS;
    cmsg->cmsg_len = CMSG_LEN(sizeof(int));
    std::memcpy(CMSG_DATA(cmsg), &response.fd, sizeof(int));
    msg.msg_controllen = cmsg->cmsg_len;
  }
  ssize_t header_sent = -1;
  for (;;) {
    header_sent = ::sendmsg(fd, &msg, MSG_NOSIGNAL);
    if (header_sent >= 0) {
      break;
    }
    if (errno == EINTR) {
      continue;
    }
    break;
  }
  if (response.fd >= 0) {
    ::close(response.fd);
  }
  if (header_sent <= 0 || header_sent > static_cast<ssize_t>(sizeof(header))) {
    return false;
  }
  const auto header_bytes_sent = static_cast<std::size_t>(header_sent);
  if (header_bytes_sent < sizeof(header) &&
      !SendAll(fd, reinterpret_cast<const char*>(header) + header_bytes_sent,
               sizeof(header) - header_bytes_sent)) {
    return false;
  }
#else
  // Windows AF_UNIX supports the JSON service protocol, without SCM_RIGHTS.
  if (response.fd >= 0 || !SendAll(fd, reinterpret_cast<const char*>(header), sizeof(header))) {
    return false;
  }
#endif
  return SendAll(fd, response.json.data(), response.json.size());
}

#endif

}  // namespace

std::string DefaultQuerySocketPath() {
  const char* configured = std::getenv("LINGTU_MAPD_QUERY_SOCKET");
  if (configured != nullptr && configured[0] != '\0') {
    return configured;
  }
  const char* session_root = std::getenv("LINGTU_SESSION_ROOT");
  if (session_root != nullptr && session_root[0] != '\0') {
    return (std::filesystem::path(session_root) / "mapd.sock").string();
  }
  return kDefaultSocketPath;
}

QueryServer::QueryServer(MapQueryCore& query, QueryServerConfig config)
    : query_(query), config_(std::move(config)) {
  if (config_.socket_path.empty() || config_.max_json_bytes == 0U ||
      config_.handshake_timeout_ms == 0U) {
    throw std::invalid_argument("mapd query server configuration is invalid");
  }
}

QueryServer::~QueryServer() {
  Stop();
}

void QueryServer::Start() {
#if defined(__linux__) || defined(_WIN32)
  if (running_.exchange(true)) {
    return;
  }
#if defined(_WIN32)
  WSADATA data {};
  const int error = ::WSAStartup(MAKEWORD(2, 2), &data);
  if (error != 0) {
    running_ = false;
    throw std::runtime_error("mapd Winsock startup failed: " + std::to_string(error));
  }
  winsock_started_ = true;
#endif
  {
    std::lock_guard<std::mutex> lock(startup_mutex_);
    startup_done_ = false;
    startup_error_.clear();
  }
  worker_ = std::thread(&QueryServer::Run, this);
  std::unique_lock<std::mutex> lock(startup_mutex_);
  startup_cv_.wait(lock, [this] { return startup_done_; });
  if (!startup_error_.empty()) {
    const std::string error = startup_error_;
    lock.unlock();
    Stop();
    throw std::runtime_error(error);
  }
#else
  throw std::runtime_error("mapd query server requires Linux or Windows AF_UNIX support");
#endif
}

void QueryServer::Stop() {
  running_.store(false);
  CloseListenSocket();
  if (worker_.joinable()) {
    worker_.join();
  }
#if defined(__linux__) || defined(_WIN32)
  std::error_code ec;
  std::filesystem::remove(config_.socket_path, ec);
#endif
#if defined(_WIN32)
  if (winsock_started_) {
    ::WSACleanup();
    winsock_started_ = false;
  }
#endif
}

void QueryServer::CloseListenSocket() noexcept {
#if defined(__linux__) || defined(_WIN32)
  const QuerySocket fd = listen_fd_.exchange(kInvalidQuerySocket);
  if (fd != kInvalidQuerySocket) {
#if defined(_WIN32)
    ::shutdown(fd, SD_BOTH);
#else
    ::shutdown(fd, SHUT_RDWR);
#endif
    CloseSocket(fd);
  }
#endif
}

void QueryServer::Run() {
#if defined(__linux__) || defined(_WIN32)
  const auto signal_startup = [this](std::string error) {
    {
      std::lock_guard<std::mutex> lock(startup_mutex_);
      startup_error_ = std::move(error);
      startup_done_ = true;
    }
    startup_cv_.notify_all();
  };
  try {
    std::error_code ec;
    std::filesystem::create_directories(config_.socket_path.parent_path(), ec);
    if (ec) {
      throw std::filesystem::filesystem_error(
          "failed to create mapd query socket directory", config_.socket_path.parent_path(), ec);
    }
    const std::string socket_path = config_.socket_path.u8string();
    if (socket_path.size() >= sizeof(sockaddr_un::sun_path)) {
      throw std::runtime_error("mapd query socket path is too long");
    }
    std::filesystem::remove(config_.socket_path, ec);
#if defined(_WIN32)
    const QuerySocket fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
#else
    const QuerySocket fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
#endif
    listen_fd_ = fd;
    if (fd == kInvalidQuerySocket) {
      throw std::runtime_error(SocketError("socket"));
    }
    sockaddr_un address {};
    address.sun_family = AF_UNIX;
    std::strncpy(address.sun_path, socket_path.c_str(), sizeof(address.sun_path) - 1U);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
      throw std::runtime_error(SocketError("bind"));
    }
#if defined(__linux__)
    if (::chmod(config_.socket_path.c_str(), 0660) != 0) {
      throw std::runtime_error(std::string("chmod failed: ") + std::strerror(errno));
    }
#endif
    if (::listen(fd, 16) != 0) {
      throw std::runtime_error(SocketError("listen"));
    }
    signal_startup({});
    while (running_) {
      const QuerySocket active_fd = listen_fd_.load();
      if (active_fd == kInvalidQuerySocket) {
        break;
      }
#if defined(_WIN32)
      const QuerySocket client = ::accept(active_fd, nullptr, nullptr);
#else
      const QuerySocket client = ::accept4(active_fd, nullptr, nullptr, SOCK_CLOEXEC);
#endif
      if (client == kInvalidQuerySocket) {
        if (SocketInterrupted()) {
          continue;
        }
        if (!running_) {
          break;
        }
        continue;
      }
      HandleClient(client);
      CloseSocket(client);
    }
  } catch (...) {
    std::string error = "mapd query server startup failed";
    try {
      throw;
    } catch (const std::exception& exc) {
      error = exc.what();
    } catch (...) {
    }
    running_ = false;
    CloseListenSocket();
    signal_startup(error);
  }
#endif
}

void QueryServer::HandleClient(QuerySocket client_fd) noexcept {
#if defined(__linux__) || defined(_WIN32)
  try {
    const auto initial_timeout_ms = std::min<std::uint32_t>(config_.handshake_timeout_ms, 60000U);
    SetSocketTimeout(client_fd, initial_timeout_ms);
    std::string error;
    const auto request = ReadRequest(client_fd, config_.max_json_bytes, &error);
    if (!request.has_value()) {
      static_cast<void>(
          SendResponse(client_fd, {Status::kError, 0U, ErrorJson("bad_request", error), -1},
                       config_.max_json_bytes));
      return;
    }
    if (request->deadline_ms > 0U) {
      const auto bounded =
          std::min<std::uint32_t>(std::max<std::uint32_t>(request->deadline_ms, 1U), 60000U);
      SetSocketTimeout(client_fd, bounded);
    }
    switch (request->opcode) {
      case Opcode::kPing:
        static_cast<void>(SendResponse(
            client_fd, {Status::kOk, 0U, query_.PingJson(), -1},
            config_.max_json_bytes));
        return;
      case Opcode::kService: {
        const auto result = query_.ServiceJson(request->json);
        static_cast<void>(SendResponse(
            client_fd,
            {result.ok ? Status::kOk : Status::kError, 0U, result.json, -1},
            config_.max_json_bytes));
        return;
      }
      case Opcode::kOpenArtifact: {
        const auto job_id = lingtu::maps::JsonObjectStringAtPath(request->json, {"job_id"});
        const auto map_id = lingtu::maps::JsonObjectStringAtPath(request->json, {"map_id"});
        const auto capability =
            lingtu::maps::JsonObjectStringAtPath(request->json, {"capability"});
        if ((!job_id.has_value() && (!map_id.has_value() || !capability.has_value())) ||
            (job_id.has_value() && (map_id.has_value() || capability.has_value()))) {
          static_cast<void>(SendResponse(
              client_fd,
              {Status::kError, 0U,
               ErrorJson("bad_request", "open_artifact requires job_id or map_id and capability"), -1},
              config_.max_json_bytes));
          return;
        }
        const auto opened = job_id.has_value() ? query_.OpenSaveSource(*job_id)
                                               : query_.OpenArtifact(*map_id, *capability);
        if (!opened.ok) {
          static_cast<void>(SendResponse(
              client_fd,
              {Status::kError, 0U, opened.json, -1},
              config_.max_json_bytes));
          return;
        }
        static_cast<void>(SendResponse(
            client_fd,
            {Status::kOk, kResponseFlagHasFd, opened.json, opened.fd},
            config_.max_json_bytes));
        return;
      }
    }
  } catch (const std::exception& exc) {
    static_cast<void>(SendResponse(
        client_fd, {Status::kError, 0U, ErrorJson("internal_error", exc.what()), -1},
        config_.max_json_bytes));
  }
#else
  (void)client_fd;
#endif
}

}  // namespace lingtu::maps::mapd::query
