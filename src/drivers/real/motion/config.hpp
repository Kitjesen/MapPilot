#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "core.hpp"

namespace lingtu::driver {

inline constexpr std::chrono::milliseconds kLeaseRefreshPeriod{100};
inline constexpr double kMinimumPollHz = 1000.0 / static_cast<double>(kLeaseRefreshPeriod.count());

struct BrainstemTlsConfig {
  std::string ca_file;
  std::string certificate_file;
  std::string private_key_file;
  std::string server_name;

  bool enabled() const noexcept {
    return !ca_file.empty() && !certificate_file.empty() && !private_key_file.empty();
  }

  bool partiallyConfigured() const noexcept {
    const unsigned configured = static_cast<unsigned>(!ca_file.empty()) +
                                static_cast<unsigned>(!certificate_file.empty()) +
                                static_cast<unsigned>(!private_key_file.empty());
    return configured != 0 && configured != 3;
  }
};

struct Config {
  std::string robot;
  int domain_id{0};
  std::string network_interface;
  std::string host{"127.0.0.1"};
  std::uint16_t port{13145};
  BrainstemTlsConfig brainstem_tls;
  Limits limits;
  double poll_hz{100.0};
  std::chrono::milliseconds rpc_timeout{100};
  std::chrono::milliseconds reconnect_delay{1000};
  std::string status_file{"/dev/shm/lingtu/driver_status.json"};
  bool help{false};
};

Config loadConfig(int argc, char **argv);
const char *usage() noexcept;

}  // namespace lingtu::driver
