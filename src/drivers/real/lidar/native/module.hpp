#pragma once

#include <cstdint>
#include <string>

namespace lingtu::drivers::lidar {

#pragma pack(push, 1)
struct Point {
  float x;
  float y;
  float z;
  float intensity;
  std::uint32_t offset_time_ns;
  std::uint8_t tag;
  std::uint8_t line;
  std::uint16_t flags;
};

struct ImuSample {
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float acc_x;
  float acc_y;
  float acc_z;
};

struct OdomPrior {
  double x;
  double y;
  double z;
  double qx;
  double qy;
  double qz;
  double qw;
  double vx;
  double vy;
  double vz;
  std::uint8_t has_velocity;
  std::uint8_t reserved[7];
};
#pragma pack(pop)

static_assert(sizeof(Point) == 24, "unexpected point record size");
static_assert(sizeof(ImuSample) == 24, "unexpected imu record size");
static_assert(sizeof(OdomPrior) == 88, "unexpected odom prior record size");

struct CliConfig {
  bool dds{false};
  bool stdin_records{false};
  bool validate_records{false};
  bool restamp_stdin_records{false};
  bool navigation_fixture{false};
  int domain_id{0};
  double replay_rate{1.0};
  double scan_window_s{0.1};
  double imu_publish_hz{0.0};
  std::string lidar_frame{"lidar_link"};
  std::string imu_frame{"imu_link"};
  std::string config_path;
};

}  // namespace lingtu::drivers::lidar
