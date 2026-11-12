#include "sensor_record_protocol.hpp"

#include <array>
#include <cstring>
#include <cmath>
#include <istream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace lingtu::sim::dds_adapter {
namespace {

using lingtu::drivers::lidar::ImuSample;
using lingtu::drivers::lidar::OdomPrior;
using lingtu::drivers::lidar::Point;
namespace camera_record = lingtu::drivers::camera::record;

constexpr std::array<char, 4> kMagic{{'L', 'T', 'U', '1'}};

static_assert(sizeof(Point) == 24, "unexpected point payload size");
static_assert(sizeof(ImuSample) == 24, "unexpected imu payload size");
static_assert(sizeof(OdomPrior) == 112, "unexpected odom prior payload size");

bool host_is_little_endian() noexcept {
  const std::uint16_t value = 1;
  return *reinterpret_cast<const std::uint8_t *>(&value) == 1;
}

std::uint32_t read_u32_le(const std::array<std::uint8_t, kSensorRecordHeaderBytes> &bytes,
                          std::size_t offset) noexcept {
  return static_cast<std::uint32_t>(bytes[offset]) |
         (static_cast<std::uint32_t>(bytes[offset + 1]) << 8U) |
         (static_cast<std::uint32_t>(bytes[offset + 2]) << 16U) |
         (static_cast<std::uint32_t>(bytes[offset + 3]) << 24U);
}

std::uint64_t read_u64_le(const std::array<std::uint8_t, kSensorRecordHeaderBytes> &bytes,
                          std::size_t offset) noexcept {
  std::uint64_t value = 0;
  for (std::size_t i = 0; i < 8; ++i) {
    value |= static_cast<std::uint64_t>(bytes[offset + i]) << (8U * i);
  }
  return value;
}

bool is_point_record(SensorRecordType type) noexcept {
  return type == SensorRecordType::Cloud || type == SensorRecordType::RegisteredCloud ||
         type == SensorRecordType::RegisteredCloudWithOrigin;
}

void require_record_type(const SensorRecord &record, SensorRecordType expected) {
  if (record.header.type != expected) {
    throw std::runtime_error(std::string("record type mismatch: expected ") +
                             sensor_record_type_name(expected) + ", got " +
                             sensor_record_type_name(record.header.type));
  }
}

}  // namespace

SensorRecordReadStatus read_sensor_record(std::istream &input, SensorRecord &record,
                                          std::string &error) {
  record = SensorRecord{};
  error.clear();

  std::array<std::uint8_t, kSensorRecordHeaderBytes> header_bytes{};
  input.read(reinterpret_cast<char *>(header_bytes.data()), header_bytes.size());
  const std::streamsize header_read = input.gcount();
  if (header_read == 0 && input.eof()) {
    return SensorRecordReadStatus::Eof;
  }
  if (header_read != static_cast<std::streamsize>(header_bytes.size())) {
    error = "truncated LTU1 header: expected 28 bytes, got " +
            std::to_string(static_cast<long long>(header_read));
    return SensorRecordReadStatus::Error;
  }

  if (std::memcmp(header_bytes.data(), kMagic.data(), kMagic.size()) != 0) {
    error = "bad LTU1 magic";
    return SensorRecordReadStatus::Error;
  }
  if (header_bytes[5] != 0 || header_bytes[6] != 0 || header_bytes[7] != 0) {
    error = "LTU1 reserved header bytes must be zero";
    return SensorRecordReadStatus::Error;
  }

  const auto raw_type = header_bytes[4];
  switch (raw_type) {
    case 1:
      record.header.type = SensorRecordType::Cloud;
      break;
    case 2:
      record.header.type = SensorRecordType::Imu;
      break;
    case 3:
      record.header.type = SensorRecordType::OdomPrior;
      break;
    case 4:
      record.header.type = SensorRecordType::RegisteredCloud;
      break;
    case 5:
      record.header.type = SensorRecordType::Camera;
      break;
    case 6:
      record.header.type = SensorRecordType::SimulationClock;
      break;
    case 7:
      record.header.type = SensorRecordType::RegisteredCloudWithOrigin;
      break;
    case 8:
      record.header.type = SensorRecordType::Gnss;
      break;
    case 9:
      record.header.type = SensorRecordType::Measurement;
      break;
    default:
      error = "unknown LTU1 record type: " + std::to_string(raw_type);
      return SensorRecordReadStatus::Error;
  }

  record.header.timestamp_ns = read_u64_le(header_bytes, 8);
  record.header.sequence = read_u32_le(header_bytes, 16);
  record.header.count = read_u32_le(header_bytes, 20);
  record.header.payload_bytes = read_u32_le(header_bytes, 24);

  if (!validate_sensor_record_header(record.header, error)) {
    return SensorRecordReadStatus::Error;
  }

  record.payload.resize(record.header.payload_bytes);
  if (!record.payload.empty()) {
    input.read(reinterpret_cast<char *>(record.payload.data()),
               static_cast<std::streamsize>(record.payload.size()));
    const std::streamsize payload_read = input.gcount();
    if (payload_read != static_cast<std::streamsize>(record.payload.size())) {
      error = std::string("truncated LTU1 ") + sensor_record_type_name(record.header.type) +
              " payload: expected " + std::to_string(record.payload.size()) + " bytes, got " +
              std::to_string(static_cast<long long>(payload_read));
      return SensorRecordReadStatus::Error;
    }
  }

  if (record.header.type == SensorRecordType::Camera ||
      record.header.type == SensorRecordType::Gnss ||
      record.header.type == SensorRecordType::Measurement) {
    try {
      if (record.header.type == SensorRecordType::Camera) {
        (void)decode_camera_payload(record);
      } else if (record.header.type == SensorRecordType::Gnss) {
        (void)decode_gnss_payload(record);
      } else {
        (void)decode_measurement_payload(record);
      }
    } catch (const std::runtime_error &exc) {
      error = exc.what();
      return SensorRecordReadStatus::Error;
    }
  }

  return SensorRecordReadStatus::Ok;
}

bool validate_sensor_record_header(const SensorRecordHeader &header, std::string &error) {
  error.clear();
  if (!host_is_little_endian()) {
    error = "LTU1 payload decoding requires a little-endian host";
    return false;
  }
  if (header.payload_bytes > kMaxSensorRecordPayloadBytes) {
    error = "LTU1 payload exceeds 256 MiB safety limit";
    return false;
  }
  if (is_point_record(header.type)) {
    constexpr std::uint32_t point_size = sizeof(Point);
    if (header.count > std::numeric_limits<std::uint32_t>::max() / point_size) {
      error = "LTU1 point count overflows payload byte calculation";
      return false;
    }
    const std::uint64_t origin_bytes =
        header.type == SensorRecordType::RegisteredCloudWithOrigin ? 3 * sizeof(double) : 0;
    if (header.payload_bytes != static_cast<std::uint64_t>(header.count) * point_size + origin_bytes) {
      error =
          std::string("LTU1 ") + sensor_record_type_name(header.type) + " payload size mismatch";
      return false;
    }
    return true;
  }
  if (header.type == SensorRecordType::Imu) {
    if (header.count != 1 || header.payload_bytes != sizeof(ImuSample)) {
      error = "LTU1 imu payload must contain exactly one ImuSample";
      return false;
    }
    return true;
  }
  if (header.type == SensorRecordType::Gnss) {
    if (header.count != 1 || header.payload_bytes != kGnssHeaderBytes + kGnssPayloadBytes) {
      error = "LTU1 gnss payload must contain exactly one 300-byte LTGN record";
      return false;
    }
    return true;
  }
  if (header.type == SensorRecordType::Measurement) {
    if (header.count == 0 || header.payload_bytes < kMeasurementHeaderBytes ||
        header.payload_bytes > kMaxMeasurementPayloadBytes) {
      error = "LTU1 measurement requires positive count and 76-byte..32 MiB payload";
      return false;
    }
    return true;
  }
  if (header.type == SensorRecordType::OdomPrior) {
    if (header.count != 1 || (header.payload_bytes != sizeof(OdomPrior) &&
        header.payload_bytes != lingtu::drivers::lidar::kLegacyOdomPriorBytes)) {
      error = "LTU1 odom prior payload must contain exactly one OdomPrior";
      return false;
    }
    return true;
  }
  if (header.type == SensorRecordType::Camera) {
    constexpr std::uint32_t camera_header_bytes = sizeof(camera_record::RecordHeader);
    if (header.count != 1) {
      error = "LTU1 camera payload must contain exactly one LTOB record";
      return false;
    }
    if (header.payload_bytes < camera_header_bytes ||
        header.payload_bytes > camera_header_bytes + camera_record::kMaxPayloadBytes) {
      error = "LTU1 camera payload size is outside the LTOB safety limit";
      return false;
    }
    return true;
  }
  if (header.type == SensorRecordType::SimulationClock) {
    if (header.count != 1 || header.payload_bytes != 0) {
      error = "LTU1 simulation clock payload must contain exactly one empty record";
      return false;
    }
    constexpr std::uint64_t nanoseconds_per_second = 1000000000ULL;
    if (header.timestamp_ns / nanoseconds_per_second >
        static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max())) {
      error = "LTU1 simulation clock timestamp exceeds int32 seconds range";
      return false;
    }
    return true;
  }
  error = "unknown LTU1 record type";
  return false;
}

std::vector<Point> decode_point_payload(const SensorRecord &record) {
  if (!is_point_record(record.header.type)) {
    throw std::runtime_error("record does not contain points");
  }
  std::vector<Point> points(record.header.count);
  if (!points.empty()) {
    const std::size_t offset = record.header.type == SensorRecordType::RegisteredCloudWithOrigin
                                   ? 3 * sizeof(double) : 0;
    std::memcpy(points.data(), record.payload.data() + offset, points.size() * sizeof(Point));
  }
  return points;
}

std::optional<std::array<double, 3>> decode_sensor_origin(const SensorRecord &record) {
  if (record.header.type != SensorRecordType::RegisteredCloudWithOrigin) return std::nullopt;
  std::array<double, 3> origin{};
  std::memcpy(origin.data(), record.payload.data(), sizeof(origin));
  for (const double coordinate : origin)
    if (!std::isfinite(coordinate)) throw std::runtime_error("registered cloud origin is not finite");
  return origin;
}

ImuSample decode_imu_payload(const SensorRecord &record) {
  require_record_type(record, SensorRecordType::Imu);
  ImuSample imu{};
  std::memcpy(&imu, record.payload.data(), sizeof(imu));
  return imu;
}

OdomPrior decode_odom_prior_payload(const SensorRecord &record) {
  require_record_type(record, SensorRecordType::OdomPrior);
  OdomPrior prior{};
  std::memcpy(&prior, record.payload.data(), record.payload.size());
  if (record.payload.size() == lingtu::drivers::lidar::kLegacyOdomPriorBytes)
    prior.has_angular_velocity = 0;
  if (prior.has_angular_velocity > 1 ||
      (prior.has_angular_velocity && (!std::isfinite(prior.wx) ||
       !std::isfinite(prior.wy) || !std::isfinite(prior.wz))))
    throw std::runtime_error("LTU1 odom prior angular velocity is invalid");
  return prior;
}

CameraRecord decode_camera_payload(const SensorRecord &record) {
  require_record_type(record, SensorRecordType::Camera);
  if (record.payload.size() < sizeof(camera_record::RecordHeader)) {
    throw std::runtime_error("LTU1 camera payload is missing the LTOB header");
  }
  CameraRecord camera;
  std::memcpy(&camera.header, record.payload.data(), sizeof(camera.header));
  const auto validation = camera_record::validateRecordHeader(camera.header);
  if (validation != camera_record::RecordValidation::kValid) {
    throw std::runtime_error(camera_record::recordValidationReason(validation));
  }
  const std::size_t payload_bytes = record.payload.size() - sizeof(camera.header);
  if (payload_bytes != camera.header.payload_size) {
    throw std::runtime_error("LTU1 camera payload does not match the LTOB payload size");
  }
  camera.payload.assign(record.payload.begin() + sizeof(camera.header), record.payload.end());
  return camera;
}

GnssRecord decode_gnss_payload(const SensorRecord &record) {
  require_record_type(record, SensorRecordType::Gnss);
  if (record.header.count != 1 ||
      record.header.payload_bytes != kGnssHeaderBytes + kGnssPayloadBytes ||
      record.payload.size() != record.header.payload_bytes) {
    throw std::runtime_error("LTU1 gnss requires one complete 300-byte LTGN record");
  }
  std::istringstream input(std::string(record.payload.begin(), record.payload.end()));
  GnssRecord gnss;
  std::string error;
  if (read_gnss_record(input, gnss, error) != GnssReadStatus::Ok) {
    throw std::runtime_error("LTU1 gnss: " + error);
  }
  if (gnss.timestamp_ns != record.header.timestamp_ns || gnss.sequence != record.header.sequence) {
    throw std::runtime_error("LTU1 gnss outer/inner timestamp or sequence mismatch");
  }
  return gnss;
}

MeasurementRecord decode_measurement_payload(const SensorRecord &record) {
  require_record_type(record, SensorRecordType::Measurement);
  if (record.header.payload_bytes != record.payload.size())
    throw std::runtime_error("LTU1 measurement payload byte count mismatch");
  auto measurement = decode_measurement_payload(record.payload);
  if (measurement.sim_time_ns != record.header.timestamp_ns ||
      measurement.sequence != record.header.sequence ||
      static_cast<std::uint64_t>(measurement.rows) * measurement.columns != record.header.count)
    throw std::runtime_error("LTU1 measurement outer/inner time, sequence or count mismatch");
  return measurement;
}

void accumulate_sensor_record_stats(const SensorRecord &record, SensorRecordStats &stats) {
  stats.bytes += kSensorRecordHeaderBytes + record.payload.size();
  switch (record.header.type) {
    case SensorRecordType::Cloud:
      ++stats.clouds;
      break;
    case SensorRecordType::Imu:
      ++stats.imu;
      break;
    case SensorRecordType::OdomPrior:
      ++stats.odom_priors;
      break;
    case SensorRecordType::RegisteredCloud:
    case SensorRecordType::RegisteredCloudWithOrigin:
      ++stats.registered_clouds;
      break;
    case SensorRecordType::Camera:
      ++stats.camera;
      break;
    case SensorRecordType::SimulationClock:
      ++stats.simulation_clocks;
      break;
    case SensorRecordType::Gnss:
      ++stats.gnss;
      break;
    case SensorRecordType::Measurement:
      ++stats.measurements;
      break;
  }
}

const char *sensor_record_type_name(SensorRecordType type) noexcept {
  switch (type) {
    case SensorRecordType::Cloud:
      return "cloud";
    case SensorRecordType::Imu:
      return "imu";
    case SensorRecordType::OdomPrior:
      return "odom_prior";
    case SensorRecordType::RegisteredCloud:
      return "registered_cloud";
    case SensorRecordType::RegisteredCloudWithOrigin:
      return "registered_cloud_with_origin";
    case SensorRecordType::Camera:
      return "camera";
    case SensorRecordType::SimulationClock:
      return "simulation_clock";
    case SensorRecordType::Gnss:
      return "gnss";
    case SensorRecordType::Measurement:
      return "measurement";
  }
  return "unknown";
}

}  // namespace lingtu::sim::dds_adapter
