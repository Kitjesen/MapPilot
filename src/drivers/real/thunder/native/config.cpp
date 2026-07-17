#include "config.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>

namespace lingtu::driver {
namespace {

std::string envString(const char* name, std::string fallback) {
  const char* value = std::getenv(name);
  return value && *value ? std::string(value) : std::move(fallback);
}

double parseDouble(const std::string& value, const char* name) {
  std::size_t used = 0;
  const double parsed = std::stod(value, &used);
  if (used != value.size() || !std::isfinite(parsed)) {
    throw std::runtime_error(std::string(name) + " must be a finite number");
  }
  return parsed;
}

long parseLong(const std::string& value, const char* name) {
  std::size_t used = 0;
  const long parsed = std::stol(value, &used);
  if (used != value.size()) {
    throw std::runtime_error(std::string(name) + " must be an integer");
  }
  return parsed;
}

double envDouble(const char* name, double fallback) {
  const char* value = std::getenv(name);
  return value && *value ? parseDouble(value, name) : fallback;
}

long envLong(const char* name, long fallback) {
  const char* value = std::getenv(name);
  return value && *value ? parseLong(value, name) : fallback;
}

double compatibleEnvDouble(
    const char* canonical,
    const char* compatibility,
    double fallback) {
  const char* value = std::getenv(canonical);
  if (value && *value) {
    return parseDouble(value, canonical);
  }
  return envDouble(compatibility, fallback);
}

void validate(const Config& cfg) {
  if (cfg.domain_id < 0) {
    throw std::runtime_error("domain id must be nonnegative");
  }
  if (cfg.host.empty()) {
    throw std::runtime_error("Brainstem host must not be empty");
  }
  if (cfg.port == 0) {
    throw std::runtime_error("Brainstem port must be positive");
  }
  std::string normalized_host = cfg.host;
  std::transform(
      normalized_host.begin(),
      normalized_host.end(),
      normalized_host.begin(),
      [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  const bool loopback = normalized_host == "localhost" ||
      normalized_host == "localhost.localdomain" ||
      normalized_host == "127.0.0.1" || normalized_host == "::1" ||
      normalized_host == "[::1]";
  if (cfg.brainstem_tls.partiallyConfigured()) {
    throw std::runtime_error(
        "Brainstem mTLS requires CA, client certificate, and private key files");
  }
  if (!loopback && !cfg.brainstem_tls.enabled()) {
    throw std::runtime_error(
        "remote Brainstem requires mTLS credentials");
  }
  if (!std::isfinite(cfg.poll_hz) || cfg.poll_hz <= 0.0 || cfg.poll_hz > 1000.0) {
    throw std::runtime_error("poll rate must be finite and within (0, 1000]");
  }
  if (!std::isfinite(cfg.limits.max_linear_mps) || cfg.limits.max_linear_mps <= 0.0 ||
      !std::isfinite(cfg.limits.max_angular_rps) || cfg.limits.max_angular_rps <= 0.0 ||
      cfg.limits.command_timeout.count() <= 0 || cfg.rpc_timeout.count() <= 0 ||
      cfg.reconnect_delay.count() <= 0) {
    throw std::runtime_error("driver limits and timeouts must be finite and positive");
  }
}

std::uint16_t checkedPort(long value) {
  if (value <= 0 || value > std::numeric_limits<std::uint16_t>::max()) {
    throw std::runtime_error("Brainstem port must be within [1, 65535]");
  }
  return static_cast<std::uint16_t>(value);
}

std::chrono::milliseconds checkedMilliseconds(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0.0 || value > 3600000.0) {
    throw std::runtime_error(std::string(name) + " must be within (0, 3600000]");
  }
  return std::chrono::milliseconds(static_cast<long long>(std::llround(value)));
}

}  // namespace

Config loadConfig(int argc, char** argv) {
  Config cfg;
  cfg.domain_id = static_cast<int>(envLong("LINGTU_DDS_DOMAIN_ID", 0));
  cfg.host = envString("LINGTU_BRAINSTEM_HOST", "127.0.0.1");
  cfg.port = checkedPort(envLong("LINGTU_BRAINSTEM_PORT", 13145));
  cfg.brainstem_tls.ca_file =
      envString("LINGTU_BRAINSTEM_TLS_CA_FILE", "");
  cfg.brainstem_tls.certificate_file =
      envString("LINGTU_BRAINSTEM_TLS_CERT_FILE", "");
  cfg.brainstem_tls.private_key_file =
      envString("LINGTU_BRAINSTEM_TLS_KEY_FILE", "");
  cfg.brainstem_tls.server_name =
      envString("LINGTU_BRAINSTEM_TLS_SERVER_NAME", "");
  cfg.limits.max_linear_mps = compatibleEnvDouble(
      "LINGTU_DRIVER_MAX_LINEAR_MPS", "LINGTU_BRAINSTEM_MAX_LINEAR", 1.0);
  cfg.limits.max_angular_rps = compatibleEnvDouble(
      "LINGTU_DRIVER_MAX_ANGULAR_RPS", "LINGTU_BRAINSTEM_MAX_ANGULAR", 1.0);
  cfg.limits.command_timeout = checkedMilliseconds(
      compatibleEnvDouble(
          "LINGTU_DRIVER_CMD_TIMEOUT_MS", "LINGTU_BRAINSTEM_CMD_TIMEOUT_MS", 200.0),
      "command timeout");
  cfg.poll_hz = envDouble("LINGTU_DRIVER_POLL_HZ", 100.0);
  cfg.rpc_timeout = checkedMilliseconds(
      envDouble("LINGTU_DRIVER_RPC_TIMEOUT_MS", 100.0), "RPC timeout");
  cfg.reconnect_delay = checkedMilliseconds(
      envDouble("LINGTU_DRIVER_RECONNECT_MS", 1000.0), "reconnect delay");
  cfg.status_file = envString(
      "LINGTU_DRIVER_STATUS_FILE", "/dev/shm/lingtu/driver_status.json");

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + arg);
      }
      return argv[++i];
    };
    if (arg == "--domain-id") {
      cfg.domain_id = static_cast<int>(parseLong(next(), "domain id"));
    } else if (arg == "--host") {
      cfg.host = next();
    } else if (arg == "--port") {
      cfg.port = checkedPort(parseLong(next(), "port"));
    } else if (arg == "--max-linear") {
      cfg.limits.max_linear_mps = parseDouble(next(), "max linear speed");
    } else if (arg == "--max-angular") {
      cfg.limits.max_angular_rps = parseDouble(next(), "max angular speed");
    } else if (arg == "--cmd-timeout-ms") {
      cfg.limits.command_timeout =
          checkedMilliseconds(parseDouble(next(), "command timeout"), "command timeout");
    } else if (arg == "--poll-hz") {
      cfg.poll_hz = parseDouble(next(), "poll rate");
    } else if (arg == "--rpc-timeout-ms") {
      cfg.rpc_timeout =
          checkedMilliseconds(parseDouble(next(), "RPC timeout"), "RPC timeout");
    } else if (arg == "--reconnect-ms") {
      cfg.reconnect_delay =
          checkedMilliseconds(parseDouble(next(), "reconnect delay"), "reconnect delay");
    } else if (arg == "--status-file") {
      cfg.status_file = next();
    } else if (arg == "--help" || arg == "-h") {
      cfg.help = true;
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  if (!cfg.help) {
    validate(cfg);
  }
  return cfg;
}

const char* usage() noexcept {
  return
      "usage: lingtu_driver [options]\n"
      "  --domain-id N          CycloneDDS domain\n"
      "  --host HOST            Brainstem gRPC host\n"
      "  --port PORT            Brainstem gRPC port\n"
      "  remote hosts require LINGTU_BRAINSTEM_TLS_{CA,CERT,KEY}_FILE\n"
      "  --max-linear MPS       speed represented by normalized Walk=1\n"
      "  --max-angular RPS      yaw rate represented by normalized Walk=1\n"
      "  --cmd-timeout-ms MS    zero command watchdog\n"
      "  --poll-hz HZ           DDS polling rate\n"
      "  --rpc-timeout-ms MS    per-Walk RPC deadline\n"
      "  --reconnect-ms MS      zero-probe retry interval\n"
      "  --status-file PATH     atomic readiness JSON\n";
}

}  // namespace lingtu::driver
