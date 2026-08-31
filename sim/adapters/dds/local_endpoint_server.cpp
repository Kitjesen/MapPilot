#include "local_endpoint_server.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
// clang-format off: Winsock must precede windows.h; BCrypt needs Windows types.
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <bcrypt.h>
#include <wincrypt.h>
// clang-format on
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif

namespace lingtu::sim::local_endpoint {
namespace {

using Clock = std::chrono::steady_clock;
using Deadline = Clock::time_point;

constexpr std::size_t kSecretBytes = 32;
constexpr std::size_t kMaximumHandshakeBytes = 4096;
constexpr std::size_t kMaximumBulkReadBytes = 1024 * 1024;
constexpr std::size_t kMaximumProtectedSecretBytes = 4096;
constexpr char kConnectSchema[] = "lingtu.sim.local_endpoint.connect.v1";
constexpr char kAckSchema[] = "lingtu.sim.local_endpoint.ack.v1";
constexpr char kReadinessSchema[] = "lingtu.sim.local_endpoint.v1";

#ifdef _WIN32
using NativeSocket = SOCKET;
constexpr NativeSocket kInvalidSocket = INVALID_SOCKET;
#else
using NativeSocket = int;
constexpr NativeSocket kInvalidSocket = -1;
#endif

struct FileIdentity final {
  std::uint64_t first{0};
  std::uint64_t second{0};
  bool valid{false};

  bool operator==(const FileIdentity &other) const noexcept {
    return valid && other.valid && first == other.first && second == other.second;
  }
};

struct PublishedFile final {
  std::filesystem::path path;
  FileIdentity identity;
  std::vector<std::uint8_t> bytes;
};

[[noreturn]] void throwSocketError(const char *const operation) {
  throw EndpointError(std::string(operation) + " failed");
}

bool isInterruptedSocketError() noexcept {
#ifdef _WIN32
  return WSAGetLastError() == WSAEINTR;
#else
  return errno == EINTR;
#endif
}

bool isWouldBlockSocketError() noexcept {
#ifdef _WIN32
  const auto error = WSAGetLastError();
  return error == WSAEWOULDBLOCK;
#else
  return errno == EAGAIN || errno == EWOULDBLOCK;
#endif
}

NativeSocket nativeSocket(const std::intptr_t value) noexcept {
  return static_cast<NativeSocket>(value);
}

std::intptr_t socketHandle(const NativeSocket socket) noexcept {
  return static_cast<std::intptr_t>(socket);
}

void closeSocket(const NativeSocket socket) noexcept {
  if (socket == kInvalidSocket) {
    return;
  }
#ifdef _WIN32
  static_cast<void>(closesocket(socket));
#else
  static_cast<void>(::close(socket));
#endif
}

void setNonBlocking(const NativeSocket socket) {
#ifdef _WIN32
  u_long enabled = 1;
  if (ioctlsocket(socket, FIONBIO, &enabled) != 0) {
    throwSocketError("setting socket nonblocking mode");
  }
#else
  const auto flags = fcntl(socket, F_GETFL, 0);
  if (flags < 0 || fcntl(socket, F_SETFL, flags | O_NONBLOCK) < 0) {
    throwSocketError("setting socket nonblocking mode");
  }
#endif
}

void setTcpNoDelay(const NativeSocket socket) {
  const int enabled = 1;
#ifdef _WIN32
  if (setsockopt(socket, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char *>(&enabled),
                 static_cast<int>(sizeof(enabled))) != 0) {
#else
  if (setsockopt(socket, IPPROTO_TCP, TCP_NODELAY, &enabled,
                 static_cast<socklen_t>(sizeof(enabled))) != 0) {
#endif
    throwSocketError("setting TCP_NODELAY");
  }
}

Deadline makeDeadline(const std::chrono::milliseconds timeout) {
  if (timeout.count() <= 0) {
    throw EndpointError("endpoint timeout must be positive");
  }
  return Clock::now() + timeout;
}

void waitForSocket(const NativeSocket socket, const bool writable, const Deadline deadline) {
  while (true) {
    const auto now = Clock::now();
    if (now >= deadline) {
      throw EndpointTimeout("local endpoint operation timed out");
    }
    auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);
    if (remaining.count() <= 0) {
      remaining = std::chrono::microseconds(1);
    }
    timeval timeout{};
    constexpr auto kMicrosecondsPerSecond = 1000000LL;
    timeout.tv_sec = static_cast<long>(remaining.count() / kMicrosecondsPerSecond);
    timeout.tv_usec = static_cast<long>(remaining.count() % kMicrosecondsPerSecond);
    fd_set readSet;
    fd_set writeSet;
    FD_ZERO(&readSet);
    FD_ZERO(&writeSet);
    if (writable) {
      FD_SET(socket, &writeSet);
    } else {
      FD_SET(socket, &readSet);
    }
#ifdef _WIN32
    const auto result =
        select(0, writable ? nullptr : &readSet, writable ? &writeSet : nullptr, nullptr, &timeout);
#else
    const auto result = select(socket + 1, writable ? nullptr : &readSet,
                               writable ? &writeSet : nullptr, nullptr, &timeout);
#endif
    if (result > 0) {
      return;
    }
    if (result == 0) {
      throw EndpointTimeout("local endpoint operation timed out");
    }
    if (!isInterruptedSocketError()) {
      throwSocketError("waiting for local endpoint socket");
    }
  }
}

std::vector<std::uint8_t> receiveExact(const NativeSocket socket, const std::size_t size,
                                       const Deadline deadline) {
  std::vector<std::uint8_t> result(size);
  std::size_t offset = 0;
  while (offset < size) {
    waitForSocket(socket, false, deadline);
    const auto remaining = size - offset;
#ifdef _WIN32
    const auto chunk = static_cast<int>(std::min<std::size_t>(
        remaining, static_cast<std::size_t>(std::numeric_limits<int>::max())));
    const auto received = recv(socket, reinterpret_cast<char *>(result.data() + offset), chunk, 0);
#else
    const auto received = recv(socket, result.data() + offset, remaining, 0);
#endif
    if (received > 0) {
      offset += static_cast<std::size_t>(received);
      continue;
    }
    if (received == 0) {
      throw EndpointError("local endpoint connection closed");
    }
    if (!isWouldBlockSocketError() && !isInterruptedSocketError()) {
      throwSocketError("receiving local endpoint bytes");
    }
  }
  return result;
}

std::optional<std::vector<std::uint8_t>>
receiveSome(const NativeSocket socket, const std::size_t maximum, const Deadline deadline) {
  std::vector<std::uint8_t> result(maximum);
  while (true) {
    waitForSocket(socket, false, deadline);
#ifdef _WIN32
    const auto chunk = static_cast<int>(
        std::min<std::size_t>(maximum, static_cast<std::size_t>(std::numeric_limits<int>::max())));
    const auto received = recv(socket, reinterpret_cast<char *>(result.data()), chunk, 0);
#else
    const auto received = recv(socket, result.data(), maximum, 0);
#endif
    if (received > 0) {
      result.resize(static_cast<std::size_t>(received));
      return result;
    }
    if (received == 0) {
      return std::nullopt;
    }
    if (!isWouldBlockSocketError() && !isInterruptedSocketError()) {
      throwSocketError("receiving local endpoint bytes");
    }
  }
}

void sendAll(const NativeSocket socket, const std::uint8_t *const payload, const std::size_t size,
             const Deadline deadline) {
  std::size_t offset = 0;
  while (offset < size) {
    waitForSocket(socket, true, deadline);
    const auto remaining = size - offset;
#ifdef _WIN32
    const auto chunk = static_cast<int>(std::min<std::size_t>(
        remaining, static_cast<std::size_t>(std::numeric_limits<int>::max())));
    const auto sent = send(socket, reinterpret_cast<const char *>(payload + offset), chunk, 0);
#else
    const auto sent = send(socket, payload + offset, remaining, MSG_NOSIGNAL);
#endif
    if (sent > 0) {
      offset += static_cast<std::size_t>(sent);
      continue;
    }
    if (sent == 0) {
      throw EndpointError("local endpoint connection closed");
    }
    if (!isWouldBlockSocketError() && !isInterruptedSocketError()) {
      throwSocketError("sending local endpoint bytes");
    }
  }
}

void sendFrame(const NativeSocket socket, const std::vector<std::uint8_t> &payload,
               const Deadline deadline) {
  if (payload.empty() || payload.size() > kMaximumHandshakeBytes) {
    throw EndpointError("local endpoint frame size is invalid");
  }
  const auto size = static_cast<std::uint32_t>(payload.size());
  const std::array<std::uint8_t, 4> header{static_cast<std::uint8_t>((size >> 24U) & 0xffU),
                                           static_cast<std::uint8_t>((size >> 16U) & 0xffU),
                                           static_cast<std::uint8_t>((size >> 8U) & 0xffU),
                                           static_cast<std::uint8_t>(size & 0xffU)};
  sendAll(socket, header.data(), header.size(), deadline);
  sendAll(socket, payload.data(), payload.size(), deadline);
}

std::vector<std::uint8_t> receiveFrame(const NativeSocket socket, const Deadline deadline) {
  const auto header = receiveExact(socket, 4, deadline);
  const auto size = (static_cast<std::uint32_t>(header[0]) << 24U) |
                    (static_cast<std::uint32_t>(header[1]) << 16U) |
                    (static_cast<std::uint32_t>(header[2]) << 8U) |
                    static_cast<std::uint32_t>(header[3]);
  if (size == 0 || size > kMaximumHandshakeBytes) {
    throw EndpointError("local endpoint frame size is invalid");
  }
  return receiveExact(socket, size, deadline);
}

bool isAsciiAlphanumeric(const char value) noexcept {
  const auto byte = static_cast<unsigned char>(value);
  return std::isalnum(byte) != 0;
}

bool isSafeToken(const std::string &value) noexcept {
  if (value.empty() || value.size() > 128 || !isAsciiAlphanumeric(value.front()) ||
      !isAsciiAlphanumeric(value.back())) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](const char character) {
    return isAsciiAlphanumeric(character) || character == '_' || character == '.' ||
           character == ':' || character == '-';
  });
}

bool isSafeAuthBasename(const std::string &value) noexcept {
  if (value.empty() || value.size() > 128 || !isAsciiAlphanumeric(value.front()) ||
      !isAsciiAlphanumeric(value.back())) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](const char character) {
    return isAsciiAlphanumeric(character) || character == '_' || character == '.' ||
           character == '-';
  });
}

bool constantTimeEqual(const std::vector<std::uint8_t> &left,
                       const std::vector<std::uint8_t> &right) noexcept {
  if (left.size() != right.size()) {
    return false;
  }
  std::uint8_t difference = 0;
  for (std::size_t index = 0; index < left.size(); ++index) {
    difference = static_cast<std::uint8_t>(difference | (left[index] ^ right[index]));
  }
  return difference == 0;
}

std::vector<std::uint8_t> secureRandom(const std::size_t size) {
  std::vector<std::uint8_t> result(size);
#ifdef _WIN32
  if (size > static_cast<std::size_t>(std::numeric_limits<ULONG>::max()) ||
      BCryptGenRandom(nullptr, result.data(), static_cast<ULONG>(size),
                      BCRYPT_USE_SYSTEM_PREFERRED_RNG) != 0) {
    throw EndpointError("secure random generation failed");
  }
#else
  const auto descriptor = open("/dev/urandom", O_RDONLY | O_CLOEXEC);
  if (descriptor < 0) {
    throw EndpointError("secure random generation failed");
  }
  std::size_t offset = 0;
  while (offset < size) {
    const auto received = read(descriptor, result.data() + offset, size - offset);
    if (received > 0) {
      offset += static_cast<std::size_t>(received);
      continue;
    }
    if (received < 0 && errno == EINTR) {
      continue;
    }
    static_cast<void>(close(descriptor));
    throw EndpointError("secure random generation failed");
  }
  static_cast<void>(close(descriptor));
#endif
  return result;
}

std::string toHex(const std::vector<std::uint8_t> &bytes) {
  constexpr char kHex[] = "0123456789abcdef";
  std::string result;
  result.reserve(bytes.size() * 2);
  for (const auto byte : bytes) {
    result.push_back(kHex[(byte >> 4U) & 0x0fU]);
    result.push_back(kHex[byte & 0x0fU]);
  }
  return result;
}

std::vector<std::uint8_t> protectSecret(const std::vector<std::uint8_t> &secret) {
#ifdef _WIN32
  if (secret.size() > static_cast<std::size_t>(std::numeric_limits<DWORD>::max())) {
    throw EndpointError("local endpoint secret is too large");
  }
  DATA_BLOB input{};
  input.cbData = static_cast<DWORD>(secret.size());
  input.pbData = const_cast<BYTE *>(secret.data());
  DATA_BLOB output{};
  if (CryptProtectData(&input, L"LingTu local runtime secret", nullptr, nullptr, nullptr,
                       CRYPTPROTECT_UI_FORBIDDEN, &output) == FALSE) {
    throw EndpointError("local endpoint secret protection failed");
  }
  std::vector<std::uint8_t> protectedSecret;
  try {
    if (output.cbData == 0 || output.cbData > kMaximumProtectedSecretBytes) {
      throw EndpointError("protected local endpoint secret size is invalid");
    }
    protectedSecret.assign(output.pbData, output.pbData + output.cbData);
  } catch (...) {
    static_cast<void>(LocalFree(output.pbData));
    throw;
  }
  static_cast<void>(LocalFree(output.pbData));
  return protectedSecret;
#else
  return secret;
#endif
}

std::filesystem::path temporaryPath(const std::filesystem::path &target) {
  const auto suffix = toHex(secureRandom(8));
  return target.parent_path() / ("." + target.filename().string() + ".tmp-" + suffix);
}

#ifdef _WIN32
FileIdentity identityFromHandle(const HANDLE handle) {
  BY_HANDLE_FILE_INFORMATION info{};
  if (GetFileInformationByHandle(handle, &info) == FALSE ||
      (info.dwFileAttributes & (FILE_ATTRIBUTE_DIRECTORY | FILE_ATTRIBUTE_REPARSE_POINT)) != 0) {
    throw EndpointError("published endpoint file identity is invalid");
  }
  return FileIdentity{static_cast<std::uint64_t>(info.dwVolumeSerialNumber),
                      (static_cast<std::uint64_t>(info.nFileIndexHigh) << 32U) |
                          static_cast<std::uint64_t>(info.nFileIndexLow),
                      true};
}
#else
FileIdentity identityFromStat(const struct stat &info) {
  if (!S_ISREG(info.st_mode)) {
    throw EndpointError("published endpoint file identity is invalid");
  }
  return FileIdentity{static_cast<std::uint64_t>(info.st_dev),
                      static_cast<std::uint64_t>(info.st_ino), true};
}
#endif

std::pair<FileIdentity, std::vector<std::uint8_t>>
readTrustedFile(const std::filesystem::path &path, const std::size_t maximum) {
#ifdef _WIN32
  const auto handle = CreateFileW(
      path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
      OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OPEN_REPARSE_POINT, nullptr);
  if (handle == INVALID_HANDLE_VALUE) {
    throw EndpointError("published endpoint file cannot be opened");
  }
  try {
    const auto identity = identityFromHandle(handle);
    LARGE_INTEGER size{};
    if (GetFileSizeEx(handle, &size) == FALSE || size.QuadPart < 0 ||
        static_cast<unsigned long long>(size.QuadPart) > maximum) {
      throw EndpointError("published endpoint file size is invalid");
    }
    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size.QuadPart));
    std::size_t offset = 0;
    while (offset < bytes.size()) {
      const auto request = static_cast<DWORD>(std::min<std::size_t>(
          bytes.size() - offset, static_cast<std::size_t>(std::numeric_limits<DWORD>::max())));
      DWORD readBytes = 0;
      if (ReadFile(handle, bytes.data() + offset, request, &readBytes, nullptr) == FALSE ||
          readBytes == 0) {
        throw EndpointError("published endpoint file cannot be read");
      }
      offset += readBytes;
    }
    static_cast<void>(CloseHandle(handle));
    return {identity, std::move(bytes)};
  } catch (...) {
    static_cast<void>(CloseHandle(handle));
    throw;
  }
#else
  const auto descriptor = open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (descriptor < 0) {
    throw EndpointError("published endpoint file cannot be opened");
  }
  try {
    struct stat info{};
    if (fstat(descriptor, &info) != 0 || info.st_size < 0 ||
        static_cast<std::uint64_t>(info.st_size) > maximum) {
      throw EndpointError("published endpoint file size is invalid");
    }
    const auto identity = identityFromStat(info);
    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(info.st_size));
    std::size_t offset = 0;
    while (offset < bytes.size()) {
      const auto readBytes = read(descriptor, bytes.data() + offset, bytes.size() - offset);
      if (readBytes > 0) {
        offset += static_cast<std::size_t>(readBytes);
        continue;
      }
      if (readBytes < 0 && errno == EINTR) {
        continue;
      }
      throw EndpointError("published endpoint file cannot be read");
    }
    static_cast<void>(close(descriptor));
    return {identity, std::move(bytes)};
  } catch (...) {
    static_cast<void>(close(descriptor));
    throw;
  }
#endif
}

void writeExclusiveFile(const std::filesystem::path &path, const std::vector<std::uint8_t> &bytes,
                        const bool privateFile) {
#ifdef _WIN32
  const auto handle = CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_NEW,
                                  FILE_ATTRIBUTE_TEMPORARY, nullptr);
  if (handle == INVALID_HANDLE_VALUE) {
    throw EndpointError("temporary endpoint file cannot be created");
  }
  try {
    std::size_t offset = 0;
    while (offset < bytes.size()) {
      const auto request = static_cast<DWORD>(std::min<std::size_t>(
          bytes.size() - offset, static_cast<std::size_t>(std::numeric_limits<DWORD>::max())));
      DWORD written = 0;
      if (WriteFile(handle, bytes.data() + offset, request, &written, nullptr) == FALSE ||
          written == 0) {
        throw EndpointError("temporary endpoint file cannot be written");
      }
      offset += written;
    }
    if (FlushFileBuffers(handle) == FALSE) {
      throw EndpointError("temporary endpoint file cannot be flushed");
    }
    static_cast<void>(CloseHandle(handle));
  } catch (...) {
    static_cast<void>(CloseHandle(handle));
    throw;
  }
  static_cast<void>(privateFile);
#else
  const auto mode = static_cast<mode_t>(privateFile ? 0600 : 0644);
  const auto descriptor =
      open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, mode);
  if (descriptor < 0) {
    throw EndpointError("temporary endpoint file cannot be created");
  }
  try {
    if (fchmod(descriptor, mode) != 0) {
      throw EndpointError("temporary endpoint file mode cannot be set");
    }
    std::size_t offset = 0;
    while (offset < bytes.size()) {
      const auto written = write(descriptor, bytes.data() + offset, bytes.size() - offset);
      if (written > 0) {
        offset += static_cast<std::size_t>(written);
        continue;
      }
      if (written < 0 && errno == EINTR) {
        continue;
      }
      throw EndpointError("temporary endpoint file cannot be written");
    }
    if (fsync(descriptor) != 0) {
      throw EndpointError("temporary endpoint file cannot be flushed");
    }
    static_cast<void>(close(descriptor));
  } catch (...) {
    static_cast<void>(close(descriptor));
    throw;
  }
#endif
}

PublishedFile publishNoReplace(const std::filesystem::path &target,
                               const std::vector<std::uint8_t> &bytes, const bool privateFile) {
  const auto temporary = temporaryPath(target);
  writeExclusiveFile(temporary, bytes, privateFile);
  try {
#ifdef _WIN32
    if (CreateHardLinkW(target.c_str(), temporary.c_str(), nullptr) == FALSE) {
      throw EndpointError("endpoint file already exists or cannot be published");
    }
#else
    if (link(temporary.c_str(), target.c_str()) != 0) {
      throw EndpointError("endpoint file already exists or cannot be published");
    }
#endif
    const auto [identity, publishedBytes] = readTrustedFile(
        target, privateFile ? kMaximumProtectedSecretBytes : kMaximumHandshakeBytes);
    if (publishedBytes != bytes) {
      throw EndpointError("published endpoint file changed unexpectedly");
    }
    std::error_code ignored;
    std::filesystem::remove(temporary, ignored);
    return PublishedFile{target, identity, bytes};
  } catch (...) {
    std::error_code ignored;
    std::filesystem::remove(temporary, ignored);
    throw;
  }
}

bool matchesOwnedFile(const PublishedFile &published) noexcept {
  if (!published.identity.valid || published.path.empty()) {
    return false;
  }
  try {
    const auto [identity, bytes] =
        readTrustedFile(published.path, std::max<std::size_t>(published.bytes.size(), 1));
    if (!(identity == published.identity) || bytes != published.bytes) {
      return false;
    }
    const auto [confirmedIdentity, confirmedBytes] =
        readTrustedFile(published.path, std::max<std::size_t>(published.bytes.size(), 1));
    return confirmedIdentity == published.identity && confirmedBytes == published.bytes;
  } catch (...) {
    return false;
  }
}

void removeIfOwned(const PublishedFile &published) noexcept {
  if (!matchesOwnedFile(published)) {
    return;
  }
  try {
    std::error_code ignored;
    static_cast<void>(std::filesystem::remove(published.path, ignored));
  } catch (...) {
    return;
  }
}

std::vector<std::uint8_t> bytesOf(const std::string &value) {
  return {value.begin(), value.end()};
}

std::string canonicalRequest(const ServerConfig &config, const std::string &nonce) {
  return "{\"nonce\":\"" + nonce + "\",\"product_session_id\":\"" +
         config.product_session_id +
         "\",\"protocol\":\"" + config.protocol + "\",\"role\":\"" + config.role +
         "\",\"schema\":\"" + kConnectSchema + "\"}";
}

std::string canonicalAck(const ServerConfig &config) {
  return "{\"ok\":true,\"product_session_id\":\"" + config.product_session_id +
         "\",\"protocol\":\"" + config.protocol + "\",\"role\":\"" + config.role +
         "\",\"schema\":\"" + kAckSchema + "\"}";
}

std::string canonicalReadiness(const ServerConfig &config, const std::uint16_t port) {
  return "{\"auth_file\":\"" + config.auth_file_name + "\",\"host\":\"127.0.0.1\",\"port\":" +
         std::to_string(port) + ",\"product_session_id\":\"" + config.product_session_id +
         "\",\"protocol\":\"" + config.protocol + "\",\"ready\":true,\"role\":\"" + config.role +
         "\",\"schema\":\"" + kReadinessSchema + "\"}";
}

void validateConfig(const ServerConfig &config) {
  if (!isSafeToken(config.role)) {
    throw EndpointError("local endpoint role is not a safe identifier");
  }
  if (!isSafeToken(config.protocol)) {
    throw EndpointError("local endpoint protocol is not a safe identifier");
  }
  if (!isSafeAuthBasename(config.product_session_id)) {
    throw EndpointError("local endpoint product session id is not a safe identifier");
  }
  if (!isSafeAuthBasename(config.auth_file_name)) {
    throw EndpointError("local endpoint auth file must be a safe basename");
  }
  if (!config.readiness_path.is_absolute() || config.readiness_path.filename().empty() ||
      config.readiness_path != config.readiness_path.lexically_normal()) {
    throw EndpointError("local endpoint readiness path must be normalized and absolute");
  }
  const auto parent = config.readiness_path.parent_path();
  std::error_code error;
  if (!std::filesystem::is_directory(parent, error) || error) {
    throw EndpointError("local endpoint readiness parent must exist");
  }
  if (config.readiness_path.filename().string() == config.auth_file_name) {
    throw EndpointError("local endpoint readiness and auth files must differ");
  }
}

}  // namespace

class LocalEndpointServer::Impl final {
 public:
  explicit Impl(ServerConfig serverConfig)
      : config(std::move(serverConfig)),
        readiness_path(config.readiness_path),
        auth_path(readiness_path.parent_path() / config.auth_file_name) {
    validateConfig(config);
  }

  ~Impl() { close(); }

  void start() {
    if (started) {
      throw EndpointError("local endpoint server is already started");
    }
    try {
#ifdef _WIN32
      WSADATA data{};
      if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
        throw EndpointError("local endpoint socket runtime failed");
      }
      socket_runtime_started = true;
#endif
      secret = secureRandom(kSecretBytes);
      const auto protectedSecret = protectSecret(secret);
      listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
      if (listener == kInvalidSocket) {
        throwSocketError("creating local endpoint listener");
      }
      const int reuse = 1;
#ifdef _WIN32
      if (setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&reuse),
                     static_cast<int>(sizeof(reuse))) != 0) {
#else
      if (setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &reuse,
                     static_cast<socklen_t>(sizeof(reuse))) != 0) {
#endif
        throwSocketError("configuring local endpoint listener");
      }
      sockaddr_in address{};
      address.sin_family = AF_INET;
      address.sin_port = htons(0);
      address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
      if (bind(listener, reinterpret_cast<const sockaddr *>(&address),
               static_cast<int>(sizeof(address))) != 0 ||
          listen(listener, 1) != 0) {
        throwSocketError("binding local endpoint listener");
      }
      setNonBlocking(listener);
      sockaddr_in bound{};
#ifdef _WIN32
      int boundSize = static_cast<int>(sizeof(bound));
#else
      socklen_t boundSize = static_cast<socklen_t>(sizeof(bound));
#endif
      if (getsockname(listener, reinterpret_cast<sockaddr *>(&bound), &boundSize) != 0 ||
          bound.sin_addr.s_addr != htonl(INADDR_LOOPBACK)) {
        throwSocketError("reading local endpoint listener address");
      }
      bound_port = ntohs(bound.sin_port);
      if (bound_port == 0) {
        throw EndpointError("local endpoint listener port is invalid");
      }
      const auto nonce = toHex(secret);
      expected_request = bytesOf(canonicalRequest(config, nonce));
      ack = bytesOf(canonicalAck(config));
      auth_file = publishNoReplace(auth_path, protectedSecret, true);
      readiness_file =
          publishNoReplace(readiness_path, bytesOf(canonicalReadiness(config, bound_port)), false);
      started = true;
    } catch (...) {
      close();
      throw;
    }
  }

  ClientStream acceptAuthenticated(const std::chrono::milliseconds accept_timeout,
                                   const std::chrono::milliseconds handshake_timeout) {
    if (!started) {
      throw EndpointError("local endpoint server is not started");
    }
    if (accepted) {
      throw EndpointError("local endpoint server accepts only one client");
    }
    const auto accept_deadline = makeDeadline(accept_timeout);
    static_cast<void>(makeDeadline(handshake_timeout));
    NativeSocket client = kInvalidSocket;
    try {
      while (true) {
        while (client == kInvalidSocket) {
          waitForSocket(listener, false, accept_deadline);
          sockaddr_in peer{};
#ifdef _WIN32
          int peerSize = static_cast<int>(sizeof(peer));
#else
          socklen_t peerSize = static_cast<socklen_t>(sizeof(peer));
#endif
          client = accept(listener, reinterpret_cast<sockaddr *>(&peer), &peerSize);
          if (client == kInvalidSocket) {
            if (isWouldBlockSocketError() || isInterruptedSocketError()) {
              continue;
            }
            throwSocketError("accepting local endpoint client");
          }
          if (peer.sin_family != AF_INET || peer.sin_addr.s_addr != htonl(INADDR_LOOPBACK)) {
            closeSocket(client);
            client = kInvalidSocket;
            throw EndpointError("local endpoint client must use loopback");
          }
        }

        setNonBlocking(client);
        setTcpNoDelay(client);
        const auto handshake_deadline = makeDeadline(handshake_timeout);
        std::vector<std::uint8_t> request;
        try {
          request = receiveFrame(client, handshake_deadline);
        } catch (const EndpointTimeout &) {
          closeSocket(client);
          client = kInvalidSocket;
          throw;
        } catch (const EndpointError &) {
          closeSocket(client);
          client = kInvalidSocket;
          continue;
        }
        if (!constantTimeEqual(request, expected_request)) {
          closeSocket(client);
          client = kInvalidSocket;
          continue;
        }
        sendFrame(client, ack, handshake_deadline);
        accepted = true;
        closeSocket(listener);
        listener = kInvalidSocket;
        return ClientStream(socketHandle(client));
      }
    } catch (const EndpointTimeout &) {
      closeSocket(client);
      throw;
    } catch (...) {
      closeSocket(client);
      throw;
    }
  }

  void close() noexcept {
    closeSocket(listener);
    listener = kInvalidSocket;
    if (readiness_file.identity.valid) {
      // The readiness file is the no-replace lease.  A compliant successor
      // cannot publish while this exact lease exists.  Keep it in place while
      // removing this instance's private auth file, then unlink the lease as
      // the final shared-path operation.  If lease ownership was lost, touch
      // neither path: they may already belong to the successor.
      if (matchesOwnedFile(readiness_file)) {
        removeIfOwned(auth_file);
        removeIfOwned(readiness_file);
      }
    } else {
      // start() may have published auth and then failed to publish readiness.
      // No endpoint lease was committed, so only that exact auth identity is
      // eligible for rollback.
      removeIfOwned(auth_file);
    }
    readiness_file = PublishedFile{};
    auth_file = PublishedFile{};
    std::fill(secret.begin(), secret.end(), std::uint8_t{0});
    secret.clear();
    expected_request.clear();
    ack.clear();
    bound_port = 0;
    started = false;
    accepted = false;
#ifdef _WIN32
    if (socket_runtime_started) {
      static_cast<void>(WSACleanup());
      socket_runtime_started = false;
    }
#endif
  }

  ServerConfig config;
  std::filesystem::path readiness_path;
  std::filesystem::path auth_path;
  NativeSocket listener{kInvalidSocket};
  std::uint16_t bound_port{0};
  bool started{false};
  bool accepted{false};
#ifdef _WIN32
  bool socket_runtime_started{false};
#endif
  std::vector<std::uint8_t> secret;
  std::vector<std::uint8_t> expected_request;
  std::vector<std::uint8_t> ack;
  PublishedFile auth_file;
  PublishedFile readiness_file;
};

ClientStream::ClientStream(const std::intptr_t socket) noexcept : socket_(socket) {}

ClientStream::ClientStream(ClientStream &&other) noexcept
    : socket_(std::exchange(other.socket_, -1)), line_buffer_(std::move(other.line_buffer_)) {}

ClientStream &ClientStream::operator=(ClientStream &&other) noexcept {
  if (this != &other) {
    close();
    socket_ = std::exchange(other.socket_, -1);
    line_buffer_ = std::move(other.line_buffer_);
  }
  return *this;
}

ClientStream::~ClientStream() {
  close();
}

std::vector<std::uint8_t> ClientStream::readExact(const std::size_t size,
                                                  const std::chrono::milliseconds timeout) {
  if (socket_ == -1) {
    throw EndpointError("local endpoint stream is closed");
  }
  if (size == 0) {
    return {};
  }
  return receiveExact(nativeSocket(socket_), size, makeDeadline(timeout));
}

std::optional<std::vector<std::uint8_t>>
ClientStream::readSome(const std::size_t max_bytes, const std::chrono::milliseconds timeout) {
  if (socket_ == -1) {
    throw EndpointError("local endpoint stream is closed");
  }
  if (max_bytes == 0 || max_bytes > kMaximumBulkReadBytes) {
    throw EndpointError("local endpoint bulk read limit is invalid");
  }
  return receiveSome(nativeSocket(socket_), max_bytes, makeDeadline(timeout));
}

std::string ClientStream::readLine(const std::size_t max_bytes,
                                   const std::chrono::milliseconds timeout) {
  if (socket_ == -1) {
    throw EndpointError("local endpoint stream is closed");
  }
  if (max_bytes == 0 || max_bytes > kMaximumHandshakeBytes) {
    throw EndpointError("local endpoint line limit is invalid");
  }
  const auto deadline = makeDeadline(timeout);
  while (true) {
    const auto newline = std::find(line_buffer_.begin(), line_buffer_.end(), '\n');
    if (newline != line_buffer_.end()) {
      const auto length = static_cast<std::size_t>(std::distance(line_buffer_.begin(), newline));
      if (length > max_bytes) {
        throw EndpointError("local endpoint line exceeds its limit");
      }
      if (!std::all_of(line_buffer_.begin(), newline,
                       [](const std::uint8_t value) { return value <= 0x7fU; })) {
        throw EndpointError("local endpoint line must be ASCII");
      }
      const std::string result(line_buffer_.begin(), newline);
      line_buffer_.erase(line_buffer_.begin(), std::next(newline));
      return result;
    }
    if (line_buffer_.size() > max_bytes) {
      throw EndpointError("local endpoint line exceeds its limit");
    }
    const auto capacity = max_bytes + 1 - line_buffer_.size();
    const auto chunk =
        receiveSome(nativeSocket(socket_), std::min<std::size_t>(capacity, 1024), deadline);
    if (!chunk.has_value()) {
      throw EndpointError("local endpoint connection closed");
    }
    line_buffer_.insert(line_buffer_.end(), chunk->begin(), chunk->end());
  }
}

void ClientStream::writeAll(const std::vector<std::uint8_t> &payload,
                            const std::chrono::milliseconds timeout) {
  if (socket_ == -1) {
    throw EndpointError("local endpoint stream is closed");
  }
  const auto deadline = makeDeadline(timeout);
  if (!payload.empty()) {
    sendAll(nativeSocket(socket_), payload.data(), payload.size(), deadline);
  }
}

void ClientStream::close() noexcept {
  if (socket_ != -1) {
    closeSocket(nativeSocket(socket_));
    socket_ = -1;
  }
  line_buffer_.clear();
}

LocalEndpointServer::LocalEndpointServer(ServerConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

LocalEndpointServer::~LocalEndpointServer() = default;

void LocalEndpointServer::start() {
  impl_->start();
}

ClientStream LocalEndpointServer::acceptAuthenticated(const std::chrono::milliseconds timeout) {
  return impl_->acceptAuthenticated(timeout, timeout);
}

ClientStream LocalEndpointServer::acceptAuthenticated(
    const std::chrono::milliseconds accept_timeout,
    const std::chrono::milliseconds handshake_timeout) {
  return impl_->acceptAuthenticated(accept_timeout, handshake_timeout);
}

std::uint16_t LocalEndpointServer::port() const {
  if (!impl_->started || impl_->bound_port == 0) {
    throw EndpointError("local endpoint server is not started");
  }
  return impl_->bound_port;
}

const std::filesystem::path &LocalEndpointServer::readinessPath() const noexcept {
  return impl_->readiness_path;
}

const std::filesystem::path &LocalEndpointServer::authPath() const noexcept {
  return impl_->auth_path;
}

void LocalEndpointServer::close() noexcept {
  impl_->close();
}

}  // namespace lingtu::sim::local_endpoint
