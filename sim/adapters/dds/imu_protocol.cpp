#include "imu_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <istream>
#include <limits>
#include <utility>

namespace lingtu::sim::dds_adapter {
namespace {

constexpr std::array<char, 4> kMagic{{'L', 'T', 'I', 'M'}};
constexpr std::uint64_t kMaximumIdlTimeNs =
    static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()) *
        1000000000ULL + 999999999ULL;

std::uint16_t read_u16(const std::uint8_t* p) noexcept {
  return static_cast<std::uint16_t>(p[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(p[1]) << 8U);
}
std::uint32_t read_u32(const std::uint8_t* p) noexcept {
  std::uint32_t value = 0;
  for (std::size_t i = 0; i < 4; ++i) value |= static_cast<std::uint32_t>(p[i]) << (8U * i);
  return value;
}
std::uint64_t read_u64(const std::uint8_t* p) noexcept {
  std::uint64_t value = 0;
  for (std::size_t i = 0; i < 8; ++i) value |= static_cast<std::uint64_t>(p[i]) << (8U * i);
  return value;
}
double read_double(const std::uint8_t* p) noexcept {
  const std::uint64_t bits = read_u64(p);
  double value = 0.0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
bool valid_session_id(const std::string& value) noexcept {
  if (value.empty() || value.size() > 63) return false;
  for (std::size_t index = 0; index < value.size(); ++index) {
    const char c = value[index];
    const bool alphanumeric =
        (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
        (c >= '0' && c <= '9');
    if (!alphanumeric && (index == 0 || (c != '_' && c != '.' && c != '-'))) return false;
  }
  return true;
}
bool frame(const std::string& value) noexcept {
  return !value.empty() && std::all_of(value.begin(), value.end(), [](char c) {
    return c != '\0' && c != '\r' && c != '\n' && c != '\t' && c != ' ';
  });
}
template <std::size_t N>
bool finite(const std::array<double, N>& values) noexcept {
  return std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value);
  });
}
template <std::size_t N>
void copy_values(const std::array<double, 10>& values, std::size_t offset,
                std::array<double, N>& output) {
  std::copy_n(values.begin() + static_cast<std::ptrdiff_t>(offset), N, output.begin());
}
template <std::size_t N>
bool zero(const std::array<double, N>& values) noexcept {
  return std::all_of(values.begin(), values.end(), [](double value) { return value == 0.0; });
}
template <std::size_t N>
bool read_exact(std::istream& input, std::array<std::uint8_t, N>& bytes) {
  input.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(N));
  return input.gcount() == static_cast<std::streamsize>(N);
}

}  // namespace

ImuReadStatus read_imu_record(std::istream& input, ImuRecord& record, std::string& error) {
  record = ImuRecord{};
  error.clear();
  std::array<std::uint8_t, kImuHeaderBytes> header{};
  input.read(reinterpret_cast<char*>(header.data()), static_cast<std::streamsize>(header.size()));
  const auto read = input.gcount();
  if (read == 0 && input.eof()) return ImuReadStatus::Eof;
  if (read != static_cast<std::streamsize>(header.size())) {
    error = "truncated LTIM header";
    return ImuReadStatus::Error;
  }
  if (std::memcmp(header.data(), kMagic.data(), kMagic.size()) != 0) {
    error = "bad LTIM magic";
    return ImuReadStatus::Error;
  }
  if (read_u16(header.data() + 4) != kImuProtocolVersion) {
    error = "unsupported LTIM protocol version";
    return ImuReadStatus::Error;
  }
  if (read_u16(header.data() + 6) != kImuAllFlags) {
    error = "LTIM flags reject missing or non-SI units";
    return ImuReadStatus::Error;
  }
  if (read_u32(header.data() + 8) != kImuHeaderBytes ||
      read_u32(header.data() + 12) != kImuPayloadBytes) {
    error = "LTIM record size mismatch";
    return ImuReadStatus::Error;
  }
  const auto* session_bytes = header.data() + 16;
  std::size_t session_size = 0;
  while (session_size < 64 && session_bytes[session_size] != 0) ++session_size;
  record.session_id.assign(reinterpret_cast<const char*>(session_bytes), session_size);
  if (!valid_session_id(record.session_id)) {
    error = "LTIM session_id is invalid";
    return ImuReadStatus::Error;
  }
  for (std::size_t i = session_size; i < 64; ++i) {
    if (session_bytes[i] != 0) {
      error = "LTIM session_id is not zero padded";
      return ImuReadStatus::Error;
    }
  }
  record.model_generation = read_u64(header.data() + 80);
  record.reset_generation = read_u64(header.data() + 88);
  record.sequence = read_u64(header.data() + 96);
  record.timestamp_ns = read_u64(header.data() + 104);
  if (record.timestamp_ns > kMaximumIdlTimeNs) {
    error = "LTIM timestamp is outside the canonical IDL Time range";
    return ImuReadStatus::Error;
  }
  const auto* frame_bytes = header.data() + 112;
  std::size_t frame_size = 0;
  while (frame_size < kImuFrameBytes && frame_bytes[frame_size] != 0) ++frame_size;
  record.frame_id.assign(reinterpret_cast<const char*>(frame_bytes), frame_size);
  for (std::size_t i = frame_size; i < kImuFrameBytes; ++i) {
    if (frame_bytes[i] != 0) {
      error = "LTIM frame field is not zero padded";
      return ImuReadStatus::Error;
    }
  }
  if (!frame(record.frame_id)) {
    error = "LTIM frame_id is invalid";
    return ImuReadStatus::Error;
  }

  std::array<std::uint8_t, kImuPayloadBytes> payload{};
  if (!read_exact(input, payload)) {
    error = "truncated LTIM payload";
    return ImuReadStatus::Error;
  }
  std::array<double, 10> values{};
  for (std::size_t i = 0; i < values.size(); ++i) values[i] = read_double(payload.data() + i * 8);
  copy_values(values, 0, record.orientation_wxyz);
  copy_values(values, 4, record.angular_velocity_rps);
  copy_values(values, 7, record.linear_acceleration_mps2);
  if (!finite(record.orientation_wxyz) || !finite(record.angular_velocity_rps) ||
      !finite(record.linear_acceleration_mps2)) {
    error = "LTIM values must be finite";
    return ImuReadStatus::Error;
  }
  if (zero(record.orientation_wxyz)) {
    error = "LTIM orientation quaternion must be non-zero";
    return ImuReadStatus::Error;
  }
  return ImuReadStatus::Ok;
}

ImuStreamValidator::ImuStreamValidator(std::string expected_session_id)
    : expected_session_id_(std::move(expected_session_id)) {}

bool ImuStreamValidator::accept(const ImuRecord& record, std::string& error) {
  error.clear();
  if (record.session_id != expected_session_id_) {
    error = "IMU session id mismatch";
    return false;
  }
  if (!initialized_ && record.sequence != 0) {
    error = "IMU generation must start at sequence zero";
    return false;
  }
  if (initialized_) {
    if (record.model_generation < model_generation_ ||
        (record.model_generation == model_generation_ &&
         record.reset_generation < reset_generation_)) {
      error = "IMU generation moved backward";
      return false;
    }
    const bool changed = record.model_generation != model_generation_ ||
                         record.reset_generation != reset_generation_;
    if (changed) {
      if (record.sequence != 0) {
        error = "IMU generation must restart at sequence zero";
        return false;
      }
    } else if (record.sequence <= sequence_ || record.timestamp_ns <= timestamp_ns_) {
      error = "IMU sequence and timestamp must increase within one generation";
      return false;
    }
  }
  initialized_ = true;
  model_generation_ = record.model_generation;
  reset_generation_ = record.reset_generation;
  sequence_ = record.sequence;
  timestamp_ns_ = record.timestamp_ns;
  return true;
}

}  // namespace lingtu::sim::dds_adapter
