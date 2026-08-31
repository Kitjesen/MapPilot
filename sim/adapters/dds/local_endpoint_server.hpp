#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace lingtu::sim::local_endpoint {

class EndpointError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

class EndpointTimeout final : public EndpointError {
 public:
  using EndpointError::EndpointError;
};

struct ServerConfig final {
  std::string role;
  std::string protocol;
  std::string product_session_id;
  std::filesystem::path readiness_path;
  std::string auth_file_name;
};

class ClientStream final {
 public:
  ClientStream(ClientStream &&other) noexcept;
  ClientStream &operator=(ClientStream &&other) noexcept;
  ~ClientStream();

  ClientStream(const ClientStream &) = delete;
  ClientStream &operator=(const ClientStream &) = delete;

  std::vector<std::uint8_t> readExact(std::size_t size, std::chrono::milliseconds timeout);
  std::optional<std::vector<std::uint8_t>> readSome(std::size_t max_bytes,
                                                    std::chrono::milliseconds timeout);
  std::string readLine(std::size_t max_bytes, std::chrono::milliseconds timeout);
  void writeAll(const std::vector<std::uint8_t> &payload, std::chrono::milliseconds timeout);
  void close() noexcept;

 private:
  friend class LocalEndpointServer;
  explicit ClientStream(std::intptr_t socket) noexcept;

  std::intptr_t socket_{-1};
  std::vector<std::uint8_t> line_buffer_;
};

class LocalEndpointServer final {
 public:
  explicit LocalEndpointServer(ServerConfig config);
  ~LocalEndpointServer();

  LocalEndpointServer(const LocalEndpointServer &) = delete;
  LocalEndpointServer &operator=(const LocalEndpointServer &) = delete;
  LocalEndpointServer(LocalEndpointServer &&) = delete;
  LocalEndpointServer &operator=(LocalEndpointServer &&) = delete;

  void start();
  ClientStream acceptAuthenticated(std::chrono::milliseconds timeout);
  ClientStream acceptAuthenticated(std::chrono::milliseconds accept_timeout,
                                   std::chrono::milliseconds handshake_timeout);
  std::uint16_t port() const;
  const std::filesystem::path &readinessPath() const noexcept;
  const std::filesystem::path &authPath() const noexcept;
  void close() noexcept;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::sim::local_endpoint
