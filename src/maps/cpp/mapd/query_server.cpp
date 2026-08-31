#include "lingtu/maps/mapd/query_server.hpp"

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

#include "lingtu/maps/json.hpp"

#if defined(__linux__)
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

std::string JsonEscape(const std::string& value) {
  std::ostringstream out;
  for (const unsigned char ch : value) {
    switch (ch) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (ch < 0x20U) {
          constexpr char kHex[] = "0123456789abcdef";
          out << "\\u00" << kHex[(ch >> 4U) & 0x0fU] << kHex[ch & 0x0fU];
        } else {
          out << static_cast<char>(ch);
        }
        break;
    }
  }
  return out.str();
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

#if defined(__linux__)
bool RecvAll(int fd, unsigned char* data, std::size_t size) {
  std::size_t offset = 0U;
  while (offset < size) {
    const ssize_t n = ::recv(fd, data + offset, size - offset, 0);
    if (n == 0) {
      return false;
    }
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    offset += static_cast<std::size_t>(n);
  }
  return true;
}

bool SendAll(int fd, const char* data, std::size_t size) {
  std::size_t offset = 0U;
  while (offset < size) {
    const ssize_t n = ::send(fd, data + offset, size - offset, MSG_NOSIGNAL);
    if (n < 0) {
      if (errno == EINTR) {
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

std::optional<Request> ReadRequest(int fd, std::size_t max_json_bytes, std::string* error) {
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

bool SendResponse(int fd, Response response, std::size_t max_json_bytes) {
  if (response.json.size() > max_json_bytes) {
    if (response.fd >= 0) {
      ::close(response.fd);
      response.fd = -1;
    }
    response.status = Status::kError;
    response.flags = 0U;
    response.json = ErrorJson("response_too_large", "mapd query JSON response exceeded limit");
  }
  if (response.json.size() > max_json_bytes ||
      response.json.size() > static_cast<std::size_t>(UINT32_MAX)) {
    if (response.fd >= 0) {
      ::close(response.fd);
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
#if defined(__linux__)
  if (running_.exchange(true)) {
    return;
  }
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
    running_ = false;
    CloseListenSocket();
    if (worker_.joinable()) {
      worker_.join();
    }
    throw std::runtime_error(error);
  }
#else
  throw std::runtime_error("mapd query server requires Linux AF_UNIX support");
#endif
}

void QueryServer::Stop() {
  running_.store(false);
  CloseListenSocket();
  if (worker_.joinable()) {
    worker_.join();
  }
#if defined(__linux__)
  std::error_code ec;
  std::filesystem::remove(config_.socket_path, ec);
#endif
}

void QueryServer::CloseListenSocket() noexcept {
#if defined(__linux__)
  const int fd = listen_fd_.exchange(-1);
  if (fd >= 0) {
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
  }
#endif
}

void QueryServer::Run() {
#if defined(__linux__)
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
    if (config_.socket_path.string().size() >= sizeof(sockaddr_un::sun_path)) {
      throw std::runtime_error("mapd query socket path is too long");
    }
    std::filesystem::remove(config_.socket_path, ec);
    const int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    listen_fd_ = fd;
    if (fd < 0) {
      throw std::runtime_error(std::string("socket failed: ") + std::strerror(errno));
    }
    sockaddr_un address {};
    address.sun_family = AF_UNIX;
    std::strncpy(address.sun_path, config_.socket_path.c_str(), sizeof(address.sun_path) - 1U);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
      throw std::runtime_error(std::string("bind failed: ") + std::strerror(errno));
    }
    if (::chmod(config_.socket_path.c_str(), 0660) != 0) {
      throw std::runtime_error(std::string("chmod failed: ") + std::strerror(errno));
    }
    if (::listen(fd, 16) != 0) {
      throw std::runtime_error(std::string("listen failed: ") + std::strerror(errno));
    }
    signal_startup({});
    while (running_) {
      const int active_fd = listen_fd_.load();
      if (active_fd < 0) {
        break;
      }
      const int client = ::accept4(active_fd, nullptr, nullptr, SOCK_CLOEXEC);
      if (client < 0) {
        if (errno == EINTR) {
          continue;
        }
        if (!running_) {
          break;
        }
        continue;
      }
      HandleClient(client);
      ::close(client);
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

void QueryServer::HandleClient(int client_fd) noexcept {
#if defined(__linux__)
  try {
    const auto initial_timeout_ms = std::min<std::uint32_t>(config_.handshake_timeout_ms, 60000U);
    timeval initial_timeout {};
    initial_timeout.tv_sec = static_cast<time_t>(initial_timeout_ms / 1000U);
    initial_timeout.tv_usec = static_cast<suseconds_t>((initial_timeout_ms % 1000U) * 1000U);
    ::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &initial_timeout, sizeof(initial_timeout));
    ::setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &initial_timeout, sizeof(initial_timeout));
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
      timeval timeout {};
      timeout.tv_sec = static_cast<time_t>(bounded / 1000U);
      timeout.tv_usec = static_cast<suseconds_t>((bounded % 1000U) * 1000U);
      ::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
      ::setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
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
        const auto map_id = lingtu::maps::JsonObjectStringAtPath(request->json, {"map_id"});
        const auto capability =
            lingtu::maps::JsonObjectStringAtPath(request->json, {"capability"});
        if (!map_id.has_value() || !capability.has_value()) {
          static_cast<void>(SendResponse(
              client_fd,
              {Status::kError, 0U,
               ErrorJson("bad_request", "open_artifact requires string map_id and capability"), -1},
              config_.max_json_bytes));
          return;
        }
        const auto opened = query_.OpenArtifact(*map_id, *capability);
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
