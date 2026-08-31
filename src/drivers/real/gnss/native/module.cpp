#include "native/module.hpp"

#include "message/cpp/topics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace lingtu::drivers::gnss {
namespace {

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kPi = 3.14159265358979323846;

std::string env_string(const char* name, std::string fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  return value;
}

int env_int(const char* name, int fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  return std::stoi(value);
}

bool parse_bool(const std::string& value) {
  return value == "1" || value == "true" || value == "TRUE" || value == "yes" ||
         value == "on";
}

bool env_bool(const char* name, bool fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  return parse_bool(value);
}

std::optional<double> env_double(const char* name) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return std::nullopt;
  }
  return std::stod(value);
}

std::optional<Origin> env_origin() {
  const auto lat = env_double("LINGTU_GNSS_ORIGIN_LAT");
  const auto lon = env_double("LINGTU_GNSS_ORIGIN_LON");
  if (!lat || !lon) {
    return std::nullopt;
  }
  return Origin{*lat, *lon, env_double("LINGTU_GNSS_ORIGIN_ALT").value_or(0.0)};
}

}  // namespace

Module::Module(Config config) : config_(std::move(config)) {}

double now_seconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

Config config_from_args(int argc, char** argv) {
  Config cfg;
  cfg.fix_topic = std::string(lingtu::message::kGnssFix.dds_topic);
  cfg.status_topic = std::string(lingtu::message::kGnssStatus.dds_topic);
  cfg.odom_topic = std::string(lingtu::message::kGnssOdom.dds_topic);

  cfg.device = env_string("LINGTU_GNSS_DEVICE", cfg.device);
  cfg.baud = env_int("LINGTU_GNSS_BAUD", cfg.baud);
  cfg.timeout_ms = env_int("LINGTU_GNSS_TIMEOUT_MS", cfg.timeout_ms);
  cfg.domain_id = env_int("LINGTU_DDS_DOMAIN_ID", cfg.domain_id);
  cfg.frame_id = env_string("LINGTU_GNSS_FRAME_ID", cfg.frame_id);
  cfg.map_frame_id = env_string("LINGTU_GNSS_MAP_FRAME_ID", cfg.map_frame_id);
  cfg.status_file = env_string("LINGTU_GNSS_STATUS_FILE", cfg.status_file);
  cfg.fix_topic = env_string("LINGTU_GNSS_FIX_TOPIC", cfg.fix_topic);
  cfg.status_topic = env_string("LINGTU_GNSS_STATUS_TOPIC", cfg.status_topic);
  cfg.odom_topic = env_string("LINGTU_GNSS_ODOM_TOPIC", cfg.odom_topic);
  cfg.publish_odom = env_bool("LINGTU_GNSS_PUBLISH_ODOM", cfg.publish_odom);
  cfg.origin = env_origin();

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + arg);
      }
      return argv[++i];
    };
    if (arg == "--device") {
      cfg.device = next();
    } else if (arg == "--baud") {
      cfg.baud = std::stoi(next());
    } else if (arg == "--timeout-ms") {
      cfg.timeout_ms = std::stoi(next());
    } else if (arg == "--domain-id" || arg == "--domain") {
      cfg.domain_id = std::stoi(next());
    } else if (arg == "--frame-id") {
      cfg.frame_id = next();
    } else if (arg == "--map-frame-id") {
      cfg.map_frame_id = next();
    } else if (arg == "--status-file") {
      cfg.status_file = next();
    } else if (arg == "--fix-topic") {
      cfg.fix_topic = next();
    } else if (arg == "--status-topic") {
      cfg.status_topic = next();
    } else if (arg == "--odom-topic") {
      cfg.odom_topic = next();
    } else if (arg == "--origin-lat") {
      cfg.origin = cfg.origin.value_or(Origin{});
      cfg.origin->latitude_deg = std::stod(next());
    } else if (arg == "--origin-lon") {
      cfg.origin = cfg.origin.value_or(Origin{});
      cfg.origin->longitude_deg = std::stod(next());
    } else if (arg == "--origin-alt") {
      cfg.origin = cfg.origin.value_or(Origin{});
      cfg.origin->altitude_m = std::stod(next());
    } else if (arg == "--publish-odom") {
      cfg.publish_odom = parse_bool(next());
    } else if (arg == "--max-samples") {
      cfg.max_samples = static_cast<std::uint64_t>(std::stoull(next()));
    } else if (arg == "--help" || arg == "-h") {
      std::fprintf(stderr,
                   "usage: lingtu_gnss_dds [--device DEV] [--baud N]\n"
                   "                       [--domain-id N] [--frame-id FRAME]\n"
                   "                       [--fix-topic TOPIC] [--status-topic TOPIC]\n"
                   "                       [--publish-odom 0|1 --origin-lat LAT --origin-lon LON]\n");
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }

  if (cfg.publish_odom && !cfg.origin) {
    throw std::runtime_error("GNSS odometry requires --origin-lat and --origin-lon");
  }
  return cfg;
}

std::array<double, 9> covariance_for(const lingtu::gnss::FixSample& fix) {
  double horizontal_var = 9.0;
  if (fix.hdop) {
    horizontal_var = std::max(0.0004, (*fix.hdop) * (*fix.hdop));
  }
  if (fix.fix_type == 4) {
    horizontal_var = std::min(horizontal_var, 0.0004);
  } else if (fix.fix_type == 5) {
    horizontal_var = std::min(horizontal_var, 0.04);
  }
  const double vertical_var = std::max(horizontal_var * 2.0, 0.0004);
  return {
      horizontal_var, 0.0, 0.0,
      0.0, horizontal_var, 0.0,
      0.0, 0.0, vertical_var,
  };
}

Enu lla_to_enu(const lingtu::gnss::FixSample& fix, const Origin& origin) {
  const double lat = fix.latitude_deg.value_or(origin.latitude_deg) * kPi / 180.0;
  const double lon = fix.longitude_deg.value_or(origin.longitude_deg) * kPi / 180.0;
  const double lat0 = origin.latitude_deg * kPi / 180.0;
  const double lon0 = origin.longitude_deg * kPi / 180.0;
  return {
      (lon - lon0) * std::cos(lat0) * kEarthRadiusM,
      (lat - lat0) * kEarthRadiusM,
      fix.altitude_m.value_or(origin.altitude_m) - origin.altitude_m,
  };
}

void write_status_file(const std::string& path, const Status& status) {
  if (path.empty()) {
    return;
  }
  try {
    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }
    std::ofstream out(path, std::ios::trunc);
    out << "{"
        << "\"schema\":\"lingtu.gnss_dds_status.v1\","
        << "\"status\":\"" << (status.last_error.empty() ? "running" : "error") << "\","
        << "\"device\":\"" << status.device << "\","
        << "\"nmea_lines\":" << status.nmea_lines << ","
        << "\"fixes\":" << status.fixes << ","
        << "\"statuses\":" << status.statuses << ","
        << "\"odometry\":" << status.odometry << ","
        << "\"last_fix_type\":" << status.last_fix_type << ","
        << "\"last_ts\":" << status.last_ts << ","
        << "\"last_error\":\"" << status.last_error << "\""
        << "}\n";
  } catch (...) {
  }
}

}  // namespace lingtu::drivers::gnss
