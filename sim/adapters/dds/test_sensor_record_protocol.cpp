#include <array>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "dds_domain.hpp"
#include "sensor_record_protocol.hpp"

namespace {

using lingtu::drivers::lidar::ImuSample;
using lingtu::drivers::lidar::OdomPrior;
using lingtu::drivers::lidar::Point;
using lingtu::sim::dds_adapter::SensorRecord;
using lingtu::sim::dds_adapter::SensorRecordReadStatus;
using lingtu::sim::dds_adapter::SensorRecordType;
namespace camera_record = lingtu::drivers::camera::record;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void append_u32(std::string &out, std::uint32_t value) {
  for (int i = 0; i < 4; ++i) {
    out.push_back(static_cast<char>((value >> (8U * i)) & 0xFFU));
  }
}

void overwrite_u32(std::string &out, std::size_t offset, std::uint32_t value) {
  for (int i = 0; i < 4; ++i) {
    out[offset + static_cast<std::size_t>(i)] = static_cast<char>((value >> (8U * i)) & 0xFFU);
  }
}

void append_u64(std::string &out, std::uint64_t value) {
  for (int i = 0; i < 8; ++i) {
    out.push_back(static_cast<char>((value >> (8U * i)) & 0xFFU));
  }
}

template <typename T>
void append_pod(std::string &out, const T &value) {
  const auto *bytes = reinterpret_cast<const char *>(&value);
  out.append(bytes, bytes + sizeof(T));
}

std::string make_record(std::uint8_t type, std::uint64_t timestamp_ns, std::uint32_t sequence,
                        std::uint32_t count, const std::string &payload) {
  std::string out;
  out.reserve(28 + payload.size());
  out.append("LTU1", 4);
  out.push_back(static_cast<char>(type));
  out.append(3, '\0');
  append_u64(out, timestamp_ns);
  append_u32(out, sequence);
  append_u32(out, count);
  append_u32(out, static_cast<std::uint32_t>(payload.size()));
  out += payload;
  return out;
}

void valid_point_record_decodes() {
  Point point{};
  point.x = 1.0F;
  point.y = 2.0F;
  point.z = 3.0F;
  point.intensity = 4.0F;
  point.offset_time_ns = 500;
  point.tag = 6;
  point.line = 7;
  std::string payload;
  append_pod(payload, point);
  std::istringstream input(make_record(1, 123, 9, 1, payload));

  SensorRecord record;
  std::string error;
  require(lingtu::sim::dds_adapter::read_sensor_record(input, record, error) ==
              SensorRecordReadStatus::Ok,
          "valid point record must parse");
  require(record.header.type == SensorRecordType::Cloud, "point type mismatch");
  require(record.header.timestamp_ns == 123, "point timestamp mismatch");
  require(record.header.sequence == 9, "point sequence mismatch");
  const auto points = lingtu::sim::dds_adapter::decode_point_payload(record);
  require(points.size() == 1, "point count mismatch");
  require(points[0].x == 1.0F, "point x mismatch");
  require(points[0].offset_time_ns == 500, "point time offset mismatch");
  require(points[0].tag == 6, "point tag mismatch");
  require(points[0].line == 7, "point line mismatch");
}

void valid_imu_and_odom_decode() {
  ImuSample imu{};
  imu.gyro_x = 0.1F;
  imu.acc_z = 0.9F;
  std::string imu_payload;
  append_pod(imu_payload, imu);
  std::istringstream imu_input(make_record(2, 200, 1, 1, imu_payload));
  SensorRecord imu_record;
  std::string error;
  require(lingtu::sim::dds_adapter::read_sensor_record(imu_input, imu_record, error) ==
              SensorRecordReadStatus::Ok,
          "valid imu record must parse");
  const auto decoded_imu = lingtu::sim::dds_adapter::decode_imu_payload(imu_record);
  require(decoded_imu.gyro_x == 0.1F, "imu gyro mismatch");
  require(decoded_imu.acc_z == 0.9F, "imu acceleration mismatch");

  OdomPrior prior{};
  prior.x = 1.0;
  prior.qw = 1.0;
  prior.has_velocity = 1;
  std::string odom_payload;
  append_pod(odom_payload, prior);
  std::istringstream odom_input(make_record(3, 300, 2, 1, odom_payload));
  SensorRecord odom_record;
  require(lingtu::sim::dds_adapter::read_sensor_record(odom_input, odom_record, error) ==
              SensorRecordReadStatus::Ok,
          "valid odom record must parse");
  const auto decoded_prior = lingtu::sim::dds_adapter::decode_odom_prior_payload(odom_record);
  require(decoded_prior.x == 1.0, "odom x mismatch");
  require(decoded_prior.qw == 1.0, "odom orientation mismatch");
  require(decoded_prior.has_velocity == 1, "odom velocity flag mismatch");
}

void valid_camera_record_decodes() {
  auto camera_header = camera_record::makeRecordHeader(camera_record::kKindIntrinsics);
  camera_header.width = 640;
  camera_header.height = 480;
  camera_header.timestamp_s = 2.0;
  camera_header.fx = 500.0;
  camera_header.fy = 501.0;
  camera_header.cx = 320.0;
  camera_header.cy = 240.0;
  std::string payload;
  append_pod(payload, camera_header);
  std::istringstream input(make_record(5, 2000000000ULL, 3, 1, payload));

  SensorRecord record;
  std::string error;
  require(lingtu::sim::dds_adapter::read_sensor_record(input, record, error) ==
              SensorRecordReadStatus::Ok,
          "valid camera record must parse");
  require(record.header.type == SensorRecordType::Camera, "camera type mismatch");
  const auto decoded = lingtu::sim::dds_adapter::decode_camera_payload(record);
  require(decoded.header.kind == camera_record::kKindIntrinsics, "camera kind mismatch");
  require(decoded.header.fx == 500.0, "camera intrinsics mismatch");
  require(decoded.payload.empty(), "camera intrinsics payload must be empty");
}

void rejects_malformed_camera_records() {
  auto camera_header = camera_record::makeRecordHeader(camera_record::kKindColor);
  camera_header.width = 1;
  camera_header.height = 1;
  camera_header.channels = 3;
  camera_header.format = camera_record::kFormatRgb8;
  camera_header.timestamp_s = 1.0;
  camera_header.payload_size = 3;
  std::string payload;
  append_pod(payload, camera_header);
  payload.append("\x01\x02\x03", 3);

  SensorRecord record;
  std::string error;
  std::istringstream bad_count(make_record(5, 1000000000ULL, 1, 2, payload));
  require(lingtu::sim::dds_adapter::read_sensor_record(bad_count, record, error) ==
              SensorRecordReadStatus::Error,
          "camera LTU1 count other than one must fail");

  payload.pop_back();
  std::istringstream bad_inner_size(make_record(5, 1000000000ULL, 1, 1, payload));
  require(lingtu::sim::dds_adapter::read_sensor_record(bad_inner_size, record, error) ==
              SensorRecordReadStatus::Error,
          "camera LTOB payload size mismatch must fail");

  std::string oversized = make_record(5, 1000000000ULL, 1, 1, payload);
  overwrite_u32(oversized, 24,
                static_cast<std::uint32_t>(sizeof(camera_record::RecordHeader)) +
                    camera_record::kMaxPayloadBytes + 1U);
  std::istringstream oversized_input(oversized);
  require(lingtu::sim::dds_adapter::read_sensor_record(oversized_input, record, error) ==
              SensorRecordReadStatus::Error,
          "camera payload above the LTOB safety limit must fail before allocation");
}

void clean_eof_is_not_error() {
  std::istringstream input("");
  SensorRecord record;
  std::string error;
  require(lingtu::sim::dds_adapter::read_sensor_record(input, record, error) ==
              SensorRecordReadStatus::Eof,
          "clean EOF must not be an error");
  require(error.empty(), "clean EOF must not set an error message");
}

void rejects_malformed_records() {
  SensorRecord record;
  std::string error;

  std::string bad_magic_bytes = make_record(1, 1, 1, 0, "");
  bad_magic_bytes[0] = 'B';
  std::istringstream bad_magic(bad_magic_bytes);
  require(lingtu::sim::dds_adapter::read_sensor_record(bad_magic, record, error) ==
              SensorRecordReadStatus::Error,
          "bad magic must fail");
  require(error.find("bad LTU1 magic") != std::string::npos, "bad magic error must be actionable");

  std::string payload(sizeof(Point) - 1, '\0');
  std::istringstream bad_size(make_record(1, 1, 1, 1, payload));
  require(lingtu::sim::dds_adapter::read_sensor_record(bad_size, record, error) ==
              SensorRecordReadStatus::Error,
          "bad point payload size must fail");
  require(error.find("payload size mismatch") != std::string::npos,
          "bad point payload error must be actionable");

  std::istringstream unknown(make_record(99, 1, 1, 0, ""));
  require(lingtu::sim::dds_adapter::read_sensor_record(unknown, record, error) ==
              SensorRecordReadStatus::Error,
          "unknown record type must fail");
  require(error.find("unknown") != std::string::npos, "unknown record error must be actionable");

  std::string truncated = make_record(2, 1, 1, 1, std::string(sizeof(ImuSample), '\0'));
  truncated.pop_back();
  std::istringstream truncated_input(truncated);
  require(lingtu::sim::dds_adapter::read_sensor_record(truncated_input, record, error) ==
              SensorRecordReadStatus::Error,
          "truncated payload must fail");
  require(error.find("truncated") != std::string::npos,
          "truncated payload error must be actionable");
}

void validates_supported_dds_domain_range() {
  using lingtu::sim::dds_adapter::parse_supported_dds_domain_id;
  require(parse_supported_dds_domain_id("0") == 0, "domain zero must parse");
  require(parse_supported_dds_domain_id("232") == 232, "domain upper bound must parse");
  for (const std::string value : {"-1", "233", "42junk", ""}) {
    bool rejected = false;
    try {
      (void)parse_supported_dds_domain_id(value);
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    require(rejected, "invalid DDS domain must be rejected");
  }
}

}  // namespace

int main() {
  valid_point_record_decodes();
  valid_imu_and_odom_decode();
  valid_camera_record_decodes();
  rejects_malformed_camera_records();
  clean_eof_is_not_error();
  rejects_malformed_records();
  validates_supported_dds_domain_range();
  return 0;
}
