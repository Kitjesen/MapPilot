#pragma once

#include "native/sdk.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace lingtu::drivers::gnss {

struct Origin {
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{0.0};
};

struct Config {
  std::string device{"/dev/wtrtk980"};
  int baud{115200};
  int timeout_ms{1000};
  int domain_id{0};
  std::string frame_id{"gnss_antenna"};
  std::string map_frame_id{"map"};
  std::string status_file{"/dev/shm/lingtu/gnss_status.json"};
  std::string fix_topic;
  std::string status_topic;
  std::string odom_topic;
  std::optional<Origin> origin;
  bool publish_odom{false};
  std::uint64_t max_samples{0};
};

struct Status {
  std::uint64_t nmea_lines{0};
  std::uint64_t fixes{0};
  std::uint64_t statuses{0};
  std::uint64_t odometry{0};
  double last_ts{0.0};
  int last_fix_type{0};
  std::string device;
  std::string last_error;
};

struct Enu {
  double east{0.0};
  double north{0.0};
  double up{0.0};
};

class Module {
 public:
  explicit Module(Config config);

  const Config& config() const { return config_; }
  Status& status() { return status_; }
  const Status& status() const { return status_; }

 private:
  Config config_;
  Status status_;
};

double now_seconds();
Config config_from_args(int argc, char** argv);
std::array<double, 9> covariance_for(const lingtu::gnss::FixSample& fix);
Enu lla_to_enu(const lingtu::gnss::FixSample& fix, const Origin& origin);
void write_status_file(const std::string& path, const Status& status);

}  // namespace lingtu::drivers::gnss
