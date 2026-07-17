#pragma once

#include "core.hpp"
#include "config.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace lingtu::driver {

struct ControlState {
  bool connected{false};
  bool ready{false};
  bool motors_enabled{false};
  bool critical_fault{false};
  bool lease_valid{false};
  std::uint32_t lease_remaining_ms{0};
  std::uint64_t accepted_sequence{0};
  bool initial_zero_acknowledged{false};
  std::string fsm{"unknown"};
  std::string owner{"none"};
  std::string owner_id;
  std::string reason{"not_connected"};
};

struct RpcResult {
  bool ok{false};
  bool transport_ok{false};
  bool accepted{false};
  ControlState state;
  std::string error;
  unsigned rpc_calls{1};
};

class Brainstem {
 public:
  Brainstem(
      std::string host,
      unsigned port,
      std::chrono::milliseconds timeout,
      BrainstemTlsConfig tls = {});
  ~Brainstem();

  Brainstem(const Brainstem&) = delete;
  Brainstem& operator=(const Brainstem&) = delete;

  RpcResult refreshControl();
  RpcResult send(const Walk& walk);
  void release() noexcept;
  const std::string& target() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::driver
