#pragma once

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

#include "camera_record.hpp"
#include "native/module.hpp"

namespace lingtu::sim::dds_adapter {

enum class SensorRecordType : std::uint8_t {
  Cloud = 1,
  Imu = 2,
  OdomPrior = 3,
  RegisteredCloud = 4,
  Camera = 5,
};

struct SensorRecordHeader {
  SensorRecordType type{SensorRecordType::Cloud};
  std::uint64_t timestamp_ns{0};
  std::uint32_t sequence{0};
  std::uint32_t count{0};
  std::uint32_t payload_bytes{0};
};

struct SensorRecord {
  SensorRecordHeader header{};
  std::vector<std::uint8_t> payload;
};

struct CameraRecord {
  lingtu::drivers::camera::record::RecordHeader header{};
  std::vector<std::uint8_t> payload;
};

struct SensorRecordStats {
  std::uint64_t clouds{0};
  std::uint64_t imu{0};
  std::uint64_t odom_priors{0};
  std::uint64_t registered_clouds{0};
  std::uint64_t camera{0};
  std::uint64_t bytes{0};
};

enum class SensorRecordReadStatus {
  Ok,
  Eof,
  Error,
};

constexpr std::size_t kSensorRecordHeaderBytes = 28;
constexpr std::uint32_t kMaxSensorRecordPayloadBytes = 256U * 1024U * 1024U;

SensorRecordReadStatus read_sensor_record(std::istream &input, SensorRecord &record,
                                          std::string &error);

bool validate_sensor_record_header(const SensorRecordHeader &header, std::string &error);

std::vector<lingtu::drivers::lidar::Point> decode_point_payload(const SensorRecord &record);

lingtu::drivers::lidar::ImuSample decode_imu_payload(const SensorRecord &record);

lingtu::drivers::lidar::OdomPrior decode_odom_prior_payload(const SensorRecord &record);

CameraRecord decode_camera_payload(const SensorRecord &record);

void accumulate_sensor_record_stats(const SensorRecord &record, SensorRecordStats &stats);

const char *sensor_record_type_name(SensorRecordType type) noexcept;

}  // namespace lingtu::sim::dds_adapter
