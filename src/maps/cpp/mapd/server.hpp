#pragma once

#include <atomic>
#include <cstddef>
#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>

#include "query.hpp"
#include "lingtu/maps/mapd/protocol.hpp"

namespace lingtu::maps::mapd::query {

#if defined(_WIN32)
using QuerySocket = std::uintptr_t;
constexpr QuerySocket kInvalidQuerySocket = static_cast<QuerySocket>(-1);
#else
using QuerySocket = int;
constexpr QuerySocket kInvalidQuerySocket = -1;
#endif

struct QueryServerConfig {
  std::filesystem::path socket_path{kDefaultSocketPath};
  std::size_t max_json_bytes{kDefaultMaxJsonBytes};
  std::uint32_t handshake_timeout_ms{2000U};
};

class QueryServer {
 public:
  QueryServer(MapQueryCore& query, QueryServerConfig config);
  ~QueryServer();

  QueryServer(const QueryServer&) = delete;
  QueryServer& operator=(const QueryServer&) = delete;

  void Start();
  void Stop();
  bool running() const noexcept { return running_; }

 private:
  void Run();
  void HandleClient(QuerySocket client_fd) noexcept;
  void CloseListenSocket() noexcept;

  MapQueryCore& query_;
  QueryServerConfig config_;
  std::atomic_bool running_{false};
  std::atomic<QuerySocket> listen_fd_{kInvalidQuerySocket};
#if defined(_WIN32)
  bool winsock_started_{false};
#endif
  std::mutex startup_mutex_;
  std::condition_variable startup_cv_;
  bool startup_done_{false};
  std::string startup_error_;
  std::thread worker_;
};

std::string DefaultQuerySocketPath();

}  // namespace lingtu::maps::mapd::query
