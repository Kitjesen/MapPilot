#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "local_endpoint_server.hpp"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
// clang-format off: Winsock must precede windows.h.
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <wincrypt.h>
// clang-format on
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace {

using namespace std::chrono_literals;
using lingtu::sim::local_endpoint::EndpointError;
using lingtu::sim::local_endpoint::EndpointTimeout;
using lingtu::sim::local_endpoint::LocalEndpointServer;
using lingtu::sim::local_endpoint::ServerConfig;

#ifdef _WIN32
using NativeSocket = SOCKET;
constexpr NativeSocket kInvalidSocket = INVALID_SOCKET;
#else
using NativeSocket = int;
constexpr NativeSocket kInvalidSocket = -1;
#endif

void require(const bool condition, const char *const message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

class SocketRuntime final {
 public:
  SocketRuntime() {
#ifdef _WIN32
    WSADATA data{};
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
      throw std::runtime_error("WSAStartup failed");
    }
#endif
  }

  ~SocketRuntime() {
#ifdef _WIN32
    WSACleanup();
#endif
  }

  SocketRuntime(const SocketRuntime &) = delete;
  SocketRuntime &operator=(const SocketRuntime &) = delete;
};

void closeSocket(const NativeSocket socket) {
  if (socket == kInvalidSocket) {
    return;
  }
#ifdef _WIN32
  closesocket(socket);
#else
  close(socket);
#endif
}

class ClientSocket final {
 public:
  explicit ClientSocket(const std::uint16_t port) {
    socket_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket_ == kInvalidSocket) {
      throw std::runtime_error("client socket creation failed");
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::connect(socket_, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0) {
      closeSocket(socket_);
      socket_ = kInvalidSocket;
      throw std::runtime_error("client loopback connect failed");
    }
  }

  ~ClientSocket() { closeSocket(socket_); }

  ClientSocket(const ClientSocket &) = delete;
  ClientSocket &operator=(const ClientSocket &) = delete;

  void sendAll(const std::vector<std::uint8_t> &payload) const {
    std::size_t offset = 0;
    while (offset < payload.size()) {
      const auto remaining = payload.size() - offset;
#ifdef _WIN32
      const auto sent = ::send(socket_, reinterpret_cast<const char *>(payload.data() + offset),
                               static_cast<int>(remaining), 0);
#else
      const auto sent = ::send(socket_, payload.data() + offset, remaining, MSG_NOSIGNAL);
#endif
      if (sent <= 0) {
        throw std::runtime_error("client send failed");
      }
      offset += static_cast<std::size_t>(sent);
    }
  }

  std::vector<std::uint8_t> receiveExact(const std::size_t size) const {
    std::vector<std::uint8_t> result(size);
    std::size_t offset = 0;
    while (offset < size) {
      const auto remaining = size - offset;
#ifdef _WIN32
      const auto received = ::recv(socket_, reinterpret_cast<char *>(result.data() + offset),
                                   static_cast<int>(remaining), 0);
#else
      const auto received = ::recv(socket_, result.data() + offset, remaining, 0);
#endif
      if (received <= 0) {
        throw std::runtime_error("client receive failed");
      }
      offset += static_cast<std::size_t>(received);
    }
    return result;
  }

  void sendFrame(const std::string &payload) const {
    require(!payload.empty() && payload.size() <= 4096, "test frame size is invalid");
    const auto size = static_cast<std::uint32_t>(payload.size());
    std::vector<std::uint8_t> frame{static_cast<std::uint8_t>((size >> 24U) & 0xffU),
                                    static_cast<std::uint8_t>((size >> 16U) & 0xffU),
                                    static_cast<std::uint8_t>((size >> 8U) & 0xffU),
                                    static_cast<std::uint8_t>(size & 0xffU)};
    frame.insert(frame.end(), payload.begin(), payload.end());
    sendAll(frame);
  }

  std::string receiveFrame() const {
    const auto header = receiveExact(4);
    const auto size = (static_cast<std::uint32_t>(header[0]) << 24U) |
                      (static_cast<std::uint32_t>(header[1]) << 16U) |
                      (static_cast<std::uint32_t>(header[2]) << 8U) |
                      static_cast<std::uint32_t>(header[3]);
    require(size > 0 && size <= 4096, "server frame size is invalid");
    const auto payload = receiveExact(size);
    return {payload.begin(), payload.end()};
  }

  void sendLine(const std::string &line) const {
    require(line.find('\n') == std::string::npos && line.find('\r') == std::string::npos,
            "test line contains a delimiter");
    std::vector<std::uint8_t> payload(line.begin(), line.end());
    payload.push_back('\n');
    sendAll(payload);
  }

  std::string receiveLine() const {
    std::string line;
    while (line.size() <= 4096) {
      const auto byte = receiveExact(1).front();
      if (byte == '\n') {
        return line;
      }
      require(byte <= 0x7fU, "server line is not ASCII");
      line.push_back(static_cast<char>(byte));
    }
    throw std::runtime_error("server line exceeds test limit");
  }

  void shutdownWrite() const {
#ifdef _WIN32
    if (shutdown(socket_, SD_SEND) != 0) {
#else
    if (shutdown(socket_, SHUT_WR) != 0) {
#endif
      throw std::runtime_error("client write shutdown failed");
    }
  }

 private:
  NativeSocket socket_{kInvalidSocket};
};

std::vector<std::uint8_t> readFile(const std::filesystem::path &path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot read test file");
  }
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

void writeFile(const std::filesystem::path &path, const std::vector<std::uint8_t> &bytes) {
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    throw std::runtime_error("cannot create test file");
  }
  stream.write(reinterpret_cast<const char *>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  if (!stream) {
    throw std::runtime_error("cannot write test file");
  }
}

std::array<std::uint8_t, 32> readSecret(const std::filesystem::path &path) {
  const auto protectedSecret = readFile(path);
#ifdef _WIN32
  DATA_BLOB input{};
  input.cbData = static_cast<DWORD>(protectedSecret.size());
  input.pbData = const_cast<BYTE *>(protectedSecret.data());
  DATA_BLOB output{};
  if (CryptUnprotectData(&input, nullptr, nullptr, nullptr, nullptr, CRYPTPROTECT_UI_FORBIDDEN,
                         &output) == FALSE) {
    throw std::runtime_error("test DPAPI decrypt failed");
  }
  std::array<std::uint8_t, 32> result{};
  try {
    require(output.cbData == result.size(), "test secret size is invalid");
    std::copy(output.pbData, output.pbData + output.cbData, result.begin());
  } catch (...) {
    LocalFree(output.pbData);
    throw;
  }
  LocalFree(output.pbData);
  return result;
#else
  require(protectedSecret.size() == 32, "test secret size is invalid");
  std::array<std::uint8_t, 32> result{};
  std::copy(protectedSecret.begin(), protectedSecret.end(), result.begin());
  return result;
#endif
}

std::string toHex(const std::array<std::uint8_t, 32> &bytes) {
  constexpr char kHex[] = "0123456789abcdef";
  std::string result;
  result.reserve(bytes.size() * 2);
  for (const auto byte : bytes) {
    result.push_back(kHex[(byte >> 4U) & 0x0fU]);
    result.push_back(kHex[byte & 0x0fU]);
  }
  return result;
}

std::filesystem::path uniqueTestDirectory() {
  const auto unique = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  const auto path = std::filesystem::temp_directory_path() / ("lingtu-local-endpoint-" + unique);
  std::filesystem::create_directory(path);
  return path;
}

std::string canonicalRequest(const std::string &product_session_id, const std::string &nonce,
                             const std::string &protocol, const std::string &role) {
  return "{\"nonce\":\"" + nonce + "\",\"product_session_id\":\"" + product_session_id +
         "\",\"protocol\":\"" + protocol + "\",\"role\":\"" + role +
         "\",\"schema\":\"lingtu.sim.local_endpoint.connect.v1\"}";
}

std::string canonicalAck(const std::string &product_session_id, const std::string &protocol,
                         const std::string &role) {
  return "{\"ok\":true,\"product_session_id\":\"" + product_session_id +
         "\",\"protocol\":\"" + protocol + "\",\"role\":\"" + role +
         "\",\"schema\":\"lingtu.sim.local_endpoint.ack.v1\"}";
}

std::vector<std::uint8_t> framed(const std::string &payload) {
  require(!payload.empty() && payload.size() <= 4096, "test frame size is invalid");
  const auto size = static_cast<std::uint32_t>(payload.size());
  std::vector<std::uint8_t> result{static_cast<std::uint8_t>((size >> 24U) & 0xffU),
                                   static_cast<std::uint8_t>((size >> 16U) & 0xffU),
                                   static_cast<std::uint8_t>((size >> 8U) & 0xffU),
                                   static_cast<std::uint8_t>(size & 0xffU)};
  result.insert(result.end(), payload.begin(), payload.end());
  return result;
}

void testCanonicalHandshakeAndBinaryStream() {
  const SocketRuntime socketRuntime;
  static_cast<void>(socketRuntime);
  const auto directory = uniqueTestDirectory();
  const auto readinessPath = directory / "sensor.ready.json";
  const std::string product_session_id(64, 'a');
  LocalEndpointServer server(
      ServerConfig{"sensor_publisher", "ltu1", product_session_id, readinessPath, "sensor.auth"});
  server.start();

  const std::vector<std::uint8_t> binaryPayload{'L', 'T', 'U',  '1',  1,    0,
                                                0,   0,   0xde, 0xad, 0xbe, 0xef};
  std::exception_ptr clientFailure;
  std::thread client([&]() {
    try {
      const auto nonce = toHex(readSecret(directory / "sensor.auth"));
      ClientSocket connection(server.port());
      connection.sendFrame(canonicalRequest(product_session_id, nonce, "ltu1", "sensor_publisher"));
      require(connection.receiveFrame() == canonicalAck(product_session_id, "ltu1", "sensor_publisher"),
              "canonical ACK bytes changed");
      connection.sendAll(binaryPayload);
    } catch (...) {
      clientFailure = std::current_exception();
    }
  });

  auto stream = server.acceptAuthenticated(1500ms);
  const auto received = stream.readExact(binaryPayload.size(), 1000ms);
  require(received == binaryPayload, "binary payload changed");
  stream.close();
  client.join();
  if (clientFailure) {
    std::rethrow_exception(clientFailure);
  }
  server.close();
  std::filesystem::remove_all(directory);
}

void testBulkReadReportsOrderlyEof() {
  const SocketRuntime socketRuntime;
  static_cast<void>(socketRuntime);
  const auto directory = uniqueTestDirectory();
  const auto readinessPath = directory / "bulk.ready.json";
  const std::string product_session_id(64, '6');
  LocalEndpointServer server(
      ServerConfig{"sensor_publisher", "ltu1", product_session_id, readinessPath, "bulk.auth"});
  server.start();

  const std::vector<std::uint8_t> payload{'L', 'T', 'U', '1', 2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  std::exception_ptr clientFailure;
  std::thread client([&]() {
    try {
      const auto nonce = toHex(readSecret(directory / "bulk.auth"));
      ClientSocket connection(server.port());
      connection.sendFrame(canonicalRequest(product_session_id, nonce, "ltu1", "sensor_publisher"));
      require(connection.receiveFrame() == canonicalAck(product_session_id, "ltu1", "sensor_publisher"),
              "bulk canonical ACK bytes changed");
      connection.sendAll(payload);
      connection.shutdownWrite();
    } catch (...) {
      clientFailure = std::current_exception();
    }
  });

  auto stream = server.acceptAuthenticated(1500ms);
  std::vector<std::uint8_t> received;
  while (received.size() < payload.size()) {
    const auto chunk = stream.readSome(64, 1000ms);
    require(chunk.has_value(), "bulk stream reached EOF before the payload");
    received.insert(received.end(), chunk->begin(), chunk->end());
  }
  require(received == payload, "bulk read changed sensor bytes");
  require(!stream.readSome(64, 1000ms).has_value(), "orderly EOF was not reported as nullopt");
  stream.close();
  client.join();
  if (clientFailure) {
    std::rethrow_exception(clientFailure);
  }
  server.close();
  std::filesystem::remove_all(directory);
}

void testBulkReadTimeoutIsNotEof() {
  const SocketRuntime socketRuntime;
  static_cast<void>(socketRuntime);
  const auto directory = uniqueTestDirectory();
  const auto readinessPath = directory / "bulk-timeout.ready.json";
  const std::string product_session_id(64, '8');
  LocalEndpointServer server(
      ServerConfig{"sensor_publisher", "ltu1", product_session_id, readinessPath, "bulk-timeout.auth"});
  server.start();

  std::exception_ptr clientFailure;
  std::thread client([&]() {
    try {
      const auto nonce = toHex(readSecret(directory / "bulk-timeout.auth"));
      ClientSocket connection(server.port());
      connection.sendFrame(canonicalRequest(product_session_id, nonce, "ltu1", "sensor_publisher"));
      require(connection.receiveFrame() == canonicalAck(product_session_id, "ltu1", "sensor_publisher"),
              "bulk timeout canonical ACK bytes changed");
      std::this_thread::sleep_for(300ms);
    } catch (...) {
      clientFailure = std::current_exception();
    }
  });

  auto stream = server.acceptAuthenticated(1500ms);
  bool timedOut = false;
  try {
    static_cast<void>(stream.readSome(64, 100ms));
  } catch (const EndpointTimeout &) {
    timedOut = true;
  }
  require(timedOut, "bulk read timeout was confused with EOF");
  stream.close();
  client.join();
  if (clientFailure) {
    std::rethrow_exception(clientFailure);
  }
  server.close();
  std::filesystem::remove_all(directory);
}

void testPreAcceptTimeoutCanBePolledBeforeOneClientArrives() {
  const SocketRuntime socketRuntime;
  static_cast<void>(socketRuntime);
  const auto directory = uniqueTestDirectory();
  const auto readinessPath = directory / "poll.ready.json";
  const std::string product_session_id(64, '6');
  LocalEndpointServer server(
      ServerConfig{"sensor_publisher", "ltu1-v1", product_session_id, readinessPath, "poll.auth"});
  server.start();

  bool firstPollTimedOut = false;
  try {
    static_cast<void>(server.acceptAuthenticated(50ms));
  } catch (const EndpointTimeout &) {
    firstPollTimedOut = true;
  }
  require(firstPollTimedOut, "pre-accept poll did not time out");

  std::exception_ptr clientFailure;
  std::thread client([&]() {
    try {
      const auto nonce = toHex(readSecret(directory / "poll.auth"));
      ClientSocket connection(server.port());
      connection.sendFrame(
          canonicalRequest(product_session_id, nonce, "ltu1-v1", "sensor_publisher"));
      require(connection.receiveFrame() ==
                  canonicalAck(product_session_id, "ltu1-v1", "sensor_publisher"),
              "polled endpoint canonical ACK bytes changed");
      connection.sendAll({'O', 'K'});
    } catch (...) {
      clientFailure = std::current_exception();
    }
  });

  try {
    auto stream = server.acceptAuthenticated(1500ms);
    require(stream.readExact(2, 1000ms) == std::vector<std::uint8_t>({'O', 'K'}),
            "pre-accept polling changed client bytes");
    stream.close();
  } catch (...) {
    server.close();
    client.join();
    throw;
  }
  client.join();
  if (clientFailure) {
    std::rethrow_exception(clientFailure);
  }
  server.close();
  std::filesystem::remove_all(directory);
}

void testAcceptedSocketGetsIndependentHandshakeDeadline() {
  const SocketRuntime socketRuntime;
  static_cast<void>(socketRuntime);
  const auto directory = uniqueTestDirectory();
  const auto readinessPath = directory / "independent-handshake.ready.json";
  const std::string product_session_id(64, '8');
  LocalEndpointServer server(ServerConfig{"sensor_publisher", "ltu1-v1", product_session_id,
                                          readinessPath, "independent-handshake.auth"});
  server.start();

  ClientSocket silentClient(server.port());
  const auto started = std::chrono::steady_clock::now();
  bool timedOut = false;
  try {
    static_cast<void>(server.acceptAuthenticated(50ms, 150ms));
  } catch (const EndpointTimeout &) {
    timedOut = true;
  }
  const auto elapsed = std::chrono::steady_clock::now() - started;
  require(timedOut, "silent authenticated socket did not time out");
  require(elapsed >= 100ms, "handshake reused the nearly-expired accept deadline");
  require(elapsed < 500ms, "independent handshake deadline was not bounded");

  server.close();
  std::filesystem::remove_all(directory);
}

void testUnauthenticatedSocketTimeoutDoesNotConsumeClientSlot() {
  const SocketRuntime socketRuntime;
  static_cast<void>(socketRuntime);
  const auto directory = uniqueTestDirectory();
  const auto readinessPath = directory / "auth-timeout.ready.json";
  const std::string product_session_id(64, 'c');
  LocalEndpointServer server(ServerConfig{"driver_bridge", "line-v1", product_session_id,
                                          readinessPath, "auth-timeout.auth"});
  server.start();

  std::thread silentClient([&]() {
    ClientSocket connection(server.port());
    std::this_thread::sleep_for(250ms);
  });
  bool firstAttemptTimedOut = false;
  try {
    static_cast<void>(server.acceptAuthenticated(100ms));
  } catch (const EndpointTimeout &) {
    firstAttemptTimedOut = true;
  }
  silentClient.join();
  require(firstAttemptTimedOut, "unauthenticated socket did not time out");

  std::exception_ptr clientFailure;
  std::thread authenticatedClient([&]() {
    try {
      const auto nonce = toHex(readSecret(directory / "auth-timeout.auth"));
      ClientSocket connection(server.port());
      connection.sendFrame(canonicalRequest(product_session_id, nonce, "line-v1", "driver_bridge"));
      require(connection.receiveFrame() ==
                  canonicalAck(product_session_id, "line-v1", "driver_bridge"),
              "retry after unauthenticated timeout changed canonical ACK bytes");
    } catch (...) {
      clientFailure = std::current_exception();
    }
  });
  try {
    auto stream = server.acceptAuthenticated(1500ms);
    stream.close();
  } catch (...) {
    server.close();
    authenticatedClient.join();
    throw;
  }
  authenticatedClient.join();
  if (clientFailure) {
    std::rethrow_exception(clientFailure);
  }

  bool authenticatedSlotRemainsSingleUse = false;
  try {
    static_cast<void>(server.acceptAuthenticated(50ms));
  } catch (const EndpointError &) {
    authenticatedSlotRemainsSingleUse = true;
  }
  require(authenticatedSlotRemainsSingleUse,
          "server accepted a second client after successful authentication");
  server.close();
  std::filesystem::remove_all(directory);
}

void testCanonicalHandshakeAndAsciiLineStream() {
  const SocketRuntime socketRuntime;
  static_cast<void>(socketRuntime);
  const auto directory = uniqueTestDirectory();
  const auto readinessPath = directory / "driver.ready.json";
  const std::string product_session_id(64, 'b');
  LocalEndpointServer server(
      ServerConfig{"driver_bridge", "line-v1", product_session_id, readinessPath, "driver.auth"});
  server.start();

  std::exception_ptr clientFailure;
  std::thread client([&]() {
    try {
      const auto nonce = toHex(readSecret(directory / "driver.auth"));
      ClientSocket connection(server.port());
      connection.sendFrame(canonicalRequest(product_session_id, nonce, "line-v1", "driver_bridge"));
      require(connection.receiveFrame() == canonicalAck(product_session_id, "line-v1", "driver_bridge"),
              "driver canonical ACK bytes changed");
      connection.sendLine("CMD 0.25 -0.50 9");
      require(connection.receiveLine() == "ACK 9 1", "driver response line changed");
    } catch (...) {
      clientFailure = std::current_exception();
    }
  });

  auto stream = server.acceptAuthenticated(1500ms);
  require(stream.readLine(4096, 1000ms) == "CMD 0.25 -0.50 9", "driver request line changed");
  const std::string response = "ACK 9 1\n";
  stream.writeAll({response.begin(), response.end()}, 1000ms);
  stream.close();
  client.join();
  if (clientFailure) {
    std::rethrow_exception(clientFailure);
  }
  server.close();
  std::filesystem::remove_all(directory);
}

void testRejectedAuthenticationDoesNotConsumeClientSlot() {
  const SocketRuntime socketRuntime;
  static_cast<void>(socketRuntime);
  const auto directory = uniqueTestDirectory();
  const auto readinessPath = directory / "wrong-auth.ready.json";
  const std::string product_session_id(64, 'c');
  LocalEndpointServer server(
      ServerConfig{"sensor_publisher", "ltu1", product_session_id, readinessPath, "wrong.auth"});
  server.start();

  bool malformedReceivedAck = false;
  bool wrongReceivedAck = false;
  std::exception_ptr clientFailure;
  std::thread client([&]() {
    try {
      {
        ClientSocket connection(server.port());
        connection.sendAll({0, 0, 0, 0});
        try {
          static_cast<void>(connection.receiveFrame());
          malformedReceivedAck = true;
        } catch (const std::runtime_error &) {}
      }
      {
        ClientSocket connection(server.port());
        connection.sendFrame(
            canonicalRequest(product_session_id, std::string(64, '0'), "ltu1", "sensor_publisher"));
        try {
          static_cast<void>(connection.receiveFrame());
          wrongReceivedAck = true;
        } catch (const std::runtime_error &) {}
      }
      const auto nonce = toHex(readSecret(directory / "wrong.auth"));
      ClientSocket connection(server.port());
      connection.sendFrame(canonicalRequest(product_session_id, nonce, "ltu1", "sensor_publisher"));
      require(connection.receiveFrame() == canonicalAck(product_session_id, "ltu1", "sensor_publisher"),
              "valid client after rejected authentication received a changed ACK");
      connection.sendAll({'O', 'K'});
    } catch (...) {
      clientFailure = std::current_exception();
    }
  });

  try {
    auto stream = server.acceptAuthenticated(1500ms);
    require(stream.readExact(2, 1000ms) == std::vector<std::uint8_t>({'O', 'K'}),
            "valid client after rejected authentication was not accepted");
    stream.close();
  } catch (...) {
    server.close();
    client.join();
    throw;
  }
  client.join();
  if (clientFailure) {
    std::rethrow_exception(clientFailure);
  }
  require(!malformedReceivedAck, "malformed authentication received an ACK");
  require(!wrongReceivedAck, "wrong authentication received an ACK");
  bool secondClientRejected = false;
  try {
    static_cast<void>(server.acceptAuthenticated(100ms));
  } catch (const EndpointError &) {
    secondClientRejected = true;
  }
  require(secondClientRejected, "server accepted more than one client");
  server.close();
  std::filesystem::remove_all(directory);
}

void testHandshakeDripUsesOneAbsoluteDeadline() {
  const SocketRuntime socketRuntime;
  static_cast<void>(socketRuntime);
  const auto directory = uniqueTestDirectory();
  const auto readinessPath = directory / "drip.ready.json";
  const std::string product_session_id(64, 'd');
  LocalEndpointServer server(
      ServerConfig{"sensor_publisher", "ltu1", product_session_id, readinessPath, "drip.auth"});
  server.start();

  std::atomic<bool> stopDrip{false};
  std::thread client([&]() {
    try {
      const auto nonce = toHex(readSecret(directory / "drip.auth"));
      const auto request = framed(canonicalRequest(product_session_id, nonce, "ltu1", "sensor_publisher"));
      ClientSocket connection(server.port());
      for (const auto byte : request) {
        if (stopDrip.load()) {
          return;
        }
        connection.sendAll({byte});
        std::this_thread::sleep_for(40ms);
      }
    } catch (const std::runtime_error &) {
      return;
    }
  });

  const auto started = std::chrono::steady_clock::now();
  bool timedOut = false;
  try {
    static_cast<void>(server.acceptAuthenticated(150ms));
  } catch (const EndpointTimeout &) {
    timedOut = true;
  }
  const auto elapsed = std::chrono::steady_clock::now() - started;
  stopDrip.store(true);
  client.join();
  require(timedOut, "drip handshake did not time out");
  require(elapsed >= 100ms, "drip timeout fired implausibly early");
  require(elapsed < 350ms, "drip bytes extended the absolute deadline");
  bool secondClientRejected = false;
  try {
    static_cast<void>(server.acceptAuthenticated(100ms));
  } catch (const EndpointError &) {
    secondClientRejected = true;
  }
  require(secondClientRejected,
          "server retried after an accepted socket consumed the handshake");
  server.close();
  std::filesystem::remove_all(directory);
}

void testLineDripUsesOneAbsoluteDeadline() {
  const SocketRuntime socketRuntime;
  static_cast<void>(socketRuntime);
  const auto directory = uniqueTestDirectory();
  const auto readinessPath = directory / "line-drip.ready.json";
  const std::string product_session_id(64, '4');
  LocalEndpointServer server(
      ServerConfig{"driver_bridge", "line-v1", product_session_id, readinessPath, "line-drip.auth"});
  server.start();

  std::atomic<bool> stopDrip{false};
  std::thread client([&]() {
    try {
      const auto nonce = toHex(readSecret(directory / "line-drip.auth"));
      ClientSocket connection(server.port());
      connection.sendFrame(canonicalRequest(product_session_id, nonce, "line-v1", "driver_bridge"));
      require(connection.receiveFrame() == canonicalAck(product_session_id, "line-v1", "driver_bridge"),
              "line drip canonical ACK bytes changed");
      const std::string line = "COMMAND 123456\n";
      for (const auto byte : line) {
        if (stopDrip.load()) {
          return;
        }
        connection.sendAll({static_cast<std::uint8_t>(static_cast<unsigned char>(byte))});
        std::this_thread::sleep_for(40ms);
      }
    } catch (const std::runtime_error &) {
      return;
    }
  });

  auto stream = server.acceptAuthenticated(1500ms);
  const auto started = std::chrono::steady_clock::now();
  bool timedOut = false;
  try {
    static_cast<void>(stream.readLine(4096, 150ms));
  } catch (const EndpointTimeout &) {
    timedOut = true;
  }
  const auto elapsed = std::chrono::steady_clock::now() - started;
  stopDrip.store(true);
  stream.close();
  client.join();
  require(timedOut, "line drip did not time out");
  require(elapsed >= 100ms, "line drip timeout fired implausibly early");
  require(elapsed < 350ms, "line drip extended the absolute deadline");
  server.close();
  std::filesystem::remove_all(directory);
}

void testReadinessAndPrivateAuthPublication() {
  const SocketRuntime socketRuntime;
  static_cast<void>(socketRuntime);
  const auto directory = uniqueTestDirectory();
  const auto readinessPath = directory / "published.ready.json";
  const auto authPath = directory / "published.auth";
  const std::string product_session_id(64, 'e');
  LocalEndpointServer server(
      ServerConfig{"driver_bridge", "line-v1", product_session_id, readinessPath, "published.auth"});
  server.start();

  const auto readinessBytes = readFile(readinessPath);
  const std::string readiness(readinessBytes.begin(), readinessBytes.end());
  const auto expectedReadiness =
      "{\"auth_file\":\"published.auth\",\"host\":\"127.0.0.1\",\"port\":" +
      std::to_string(server.port()) + ",\"product_session_id\":\"" + product_session_id +
      "\",\"protocol\":\"line-v1\",\"ready\":true,"
      "\"role\":\"driver_bridge\","
      "\"schema\":\"lingtu.sim.local_endpoint.v1\"}";
  require(readiness == expectedReadiness, "readiness fields or canonical bytes changed");
  require(readiness.find("nonce") == std::string::npos, "readiness disclosed the auth nonce");
  require(server.readinessPath() == readinessPath, "server readiness path changed");
  require(server.authPath() == authPath, "server auth path changed");

  const auto protectedSecret = readFile(authPath);
  const auto secret = readSecret(authPath);
  const std::vector<std::uint8_t> rawSecret(secret.begin(), secret.end());
  require(protectedSecret.size() <= 4096, "protected auth blob exceeds the Python client limit");
#ifdef _WIN32
  require(protectedSecret != rawSecret, "Windows auth file contains the plaintext secret");
  require(std::search(protectedSecret.begin(), protectedSecret.end(), rawSecret.begin(),
                      rawSecret.end()) == protectedSecret.end(),
          "Windows auth blob embeds the plaintext secret");
#else
  require(protectedSecret == rawSecret, "POSIX auth file changed from the canonical raw secret");
  struct stat authStat{};
  require(stat(authPath.c_str(), &authStat) == 0, "POSIX auth file cannot be stat'ed");
  require((authStat.st_mode & 0777) == 0600, "POSIX auth file mode is not 0600");
#endif

  server.close();
  require(!std::filesystem::exists(readinessPath), "owned readiness file survived server close");
  require(!std::filesystem::exists(authPath), "owned auth file survived server close");
  std::filesystem::remove_all(directory);
}

void testClosePreservesSuccessorFiles() {
  const SocketRuntime socketRuntime;
  static_cast<void>(socketRuntime);
  const auto directory = uniqueTestDirectory();
  const auto readinessPath = directory / "successor.ready.json";
  const auto authPath = directory / "successor.auth";
  const std::string product_session_id(64, 'f');
  LocalEndpointServer server(
      ServerConfig{"sensor_publisher", "ltu1", product_session_id, readinessPath, "successor.auth"});
  server.start();

  require(std::filesystem::remove(readinessPath), "cannot replace owned readiness file in test");
  require(std::filesystem::remove(authPath), "cannot replace owned auth file in test");
  const std::vector<std::uint8_t> successorReadiness{'s', 'u', 'c', 'c', 'e', 's',
                                                     's', 'o', 'r', '-', 'r'};
  const std::vector<std::uint8_t> successorAuth{'s', 'u', 'c', 'c', 'e', 's',
                                                's', 'o', 'r', '-', 'a'};
  writeFile(readinessPath, successorReadiness);
  writeFile(authPath, successorAuth);

  server.close();
  require(readFile(readinessPath) == successorReadiness, "old server removed successor readiness");
  require(readFile(authPath) == successorAuth, "old server removed successor auth");
  std::filesystem::remove_all(directory);
}

void testLostReadinessLeasePreventsAnyAuthCleanup() {
  const SocketRuntime socketRuntime;
  static_cast<void>(socketRuntime);
  const auto directory = uniqueTestDirectory();
  const auto readinessPath = directory / "lease.ready.json";
  const auto authPath = directory / "lease.auth";
  const std::string product_session_id(64, '0');
  LocalEndpointServer server(
      ServerConfig{"sensor_publisher", "ltu1", product_session_id, readinessPath, "lease.auth"});
  server.start();
  const auto ownedAuth = readFile(authPath);

  require(std::filesystem::remove(readinessPath), "cannot replace readiness lease in test");
  const std::vector<std::uint8_t> successorReadiness{'s', 'u', 'c', 'c', 'e', 's', 's', 'o', 'r'};
  writeFile(readinessPath, successorReadiness);
  server.close();

  require(readFile(readinessPath) == successorReadiness, "old server removed the successor lease");
  require(readFile(authPath) == ownedAuth,
          "old server touched auth after losing its readiness lease");
  std::filesystem::remove_all(directory);
}

void testAtomicPublicationNeverOverwritesExistingFiles() {
  const SocketRuntime socketRuntime;
  static_cast<void>(socketRuntime);
  const auto directory = uniqueTestDirectory();
  const auto readinessPath = directory / "existing.ready.json";
  const auto authPath = directory / "existing.auth";
  const std::vector<std::uint8_t> readinessSentinel{'e', 'x', 'i', 's', 't',
                                                    'i', 'n', 'g', '-', 'r'};
  writeFile(readinessPath, readinessSentinel);
  LocalEndpointServer server(ServerConfig{"sensor_publisher", "ltu1", std::string(64, '1'),
                                          readinessPath, "existing.auth"});
  bool rejected = false;
  try {
    server.start();
  } catch (const EndpointError &) {
    rejected = true;
  }
  require(rejected, "existing readiness file was overwritten");
  require(readFile(readinessPath) == readinessSentinel, "existing readiness bytes changed");
  require(!std::filesystem::exists(authPath), "failed publication left an auth file");

  std::filesystem::remove(readinessPath);
  const std::vector<std::uint8_t> authSentinel{'e', 'x', 'i', 's', 't', 'i', 'n', 'g', '-', 'a'};
  writeFile(authPath, authSentinel);
  LocalEndpointServer second(ServerConfig{"sensor_publisher", "ltu1", std::string(64, '2'),
                                          readinessPath, "existing.auth"});
  rejected = false;
  try {
    second.start();
  } catch (const EndpointError &) {
    rejected = true;
  }
  require(rejected, "existing auth file was overwritten");
  require(readFile(authPath) == authSentinel, "existing auth bytes changed");
  require(!std::filesystem::exists(readinessPath), "failed auth publication left readiness");
  std::filesystem::remove_all(directory);
}

void testUnsafePublicIdentityIsRejected() {
  const auto directory = uniqueTestDirectory();
  const auto path = directory / "identity.ready.json";
  const auto expectRejected = [&](ServerConfig config) {
    bool rejected = false;
    try {
      LocalEndpointServer server(std::move(config));
    } catch (const EndpointError &) {
      rejected = true;
    }
    require(rejected, "unsafe endpoint identity was accepted");
  };
  expectRejected(ServerConfig{"bad role", "ltu1", std::string(64, '3'), path, "identity.auth"});
  expectRejected(ServerConfig{"sensor_publisher", "bad/protocol", std::string(64, '3'), path,
                              "identity.auth"});
  expectRejected(
      ServerConfig{"sensor_publisher", "ltu1", "unsafe/session", path, "identity.auth"});
  expectRejected(
      ServerConfig{"sensor_publisher", "ltu1", "", path, "identity.auth"});
  expectRejected(ServerConfig{"sensor_publisher", "ltu1", std::string(64, '3'), path, "../auth"});
  expectRejected(ServerConfig{"sensor_publisher", "ltu1", std::string(64, '3'),
                              std::filesystem::path("relative.ready.json"), "identity.auth"});
  std::filesystem::remove_all(directory);
}

void servePythonCompatibilityClient(const std::filesystem::path &directory) {
  const auto readinessPath = directory / "python.ready.json";
  const std::string product_session_id(63, '7');
  LocalEndpointServer server(
      ServerConfig{"driver_bridge", "line-v1", product_session_id, readinessPath, "python.auth"});
  server.start();
  std::cout << "READY\n" << std::flush;
  auto stream = server.acceptAuthenticated(5000ms);
  require(stream.readLine(4096, 2000ms) == "PYTHON", "Python client line changed");
  const std::string response = "CPP\n";
  stream.writeAll({response.begin(), response.end()}, 2000ms);
  stream.close();
  server.close();
}

}  // namespace

int main(const int argc, char **const argv) {
  try {
    if (argc == 3 && std::string(argv[1]) == "--serve-python-compat") {
      servePythonCompatibilityClient(std::filesystem::path(argv[2]).lexically_normal());
      return 0;
    }
    require(argc == 1, "test arguments are invalid");
    testCanonicalHandshakeAndBinaryStream();
    testBulkReadReportsOrderlyEof();
    testBulkReadTimeoutIsNotEof();
    testPreAcceptTimeoutCanBePolledBeforeOneClientArrives();
    testAcceptedSocketGetsIndependentHandshakeDeadline();
    testUnauthenticatedSocketTimeoutDoesNotConsumeClientSlot();
    testCanonicalHandshakeAndAsciiLineStream();
    testRejectedAuthenticationDoesNotConsumeClientSlot();
    testHandshakeDripUsesOneAbsoluteDeadline();
    testLineDripUsesOneAbsoluteDeadline();
    testReadinessAndPrivateAuthPublication();
    testClosePreservesSuccessorFiles();
    testLostReadinessLeasePreventsAnyAuthCleanup();
    testAtomicPublicationNeverOverwritesExistingFiles();
    testUnsafePublicIdentityIsRejected();
    std::cout << "test_local_endpoint_server: PASS\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "test_local_endpoint_server: FAIL: " << error.what() << '\n';
    return 1;
  }
}
