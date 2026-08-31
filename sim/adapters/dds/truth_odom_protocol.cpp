#include "truth_odom_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <istream>
#include <limits>
#include <utility>

namespace lingtu::sim::dds_adapter {
namespace {

constexpr std::array<char, 4> kMagic{{'L', 'T', 'O', 'D'}};
constexpr std::uint64_t kMaximumIdlTimeNs =
    static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()) * 1000000000ULL +
    999999999ULL;

static_assert(sizeof(double) == 8, "truth odometry protocol requires 64-bit doubles");
static_assert(std::numeric_limits<double>::is_iec559,
              "truth odometry protocol requires IEEE-754 doubles");

std::uint16_t read_u16_le(const std::uint8_t *bytes) noexcept {
  return static_cast<std::uint16_t>(bytes[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[1]) << 8U);
}

std::uint32_t read_u32_le(const std::uint8_t *bytes) noexcept {
  std::uint32_t value = 0;
  for (std::size_t index = 0; index < 4; ++index) {
    value |= static_cast<std::uint32_t>(bytes[index]) << (8U * index);
  }
  return value;
}

std::uint64_t read_u64_le(const std::uint8_t *bytes) noexcept {
  std::uint64_t value = 0;
  for (std::size_t index = 0; index < 8; ++index) {
    value |= static_cast<std::uint64_t>(bytes[index]) << (8U * index);
  }
  return value;
}

double read_double_le(const std::uint8_t *bytes) noexcept {
  const std::uint64_t bits = read_u64_le(bytes);
  double value = 0.0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

template <std::size_t Size>
bool read_exact(std::istream &input, std::array<std::uint8_t, Size> &bytes) {
  input.read(reinterpret_cast<char *>(bytes.data()), static_cast<std::streamsize>(Size));
  return input.gcount() == static_cast<std::streamsize>(Size);
}

template <std::size_t Size>
void copy_values(const std::array<double, kTruthOdometryPayloadDoubleCount> &values,
                 std::size_t offset, std::array<double, Size> &destination) {
  std::copy_n(values.begin() + static_cast<std::ptrdiff_t>(offset), Size, destination.begin());
}

bool valid_id(const std::string &value) noexcept {
  if (value.empty() || value.size() > 63) return false;
  for (std::size_t index = 0; index < value.size(); ++index) {
    const char character = value[index];
    const bool alphanumeric =
        (character >= 'A' && character <= 'Z') ||
        (character >= 'a' && character <= 'z') ||
        (character >= '0' && character <= '9');
    if (!alphanumeric &&
        (index == 0 || (character != '_' && character != '.' && character != '-'))) return false;
  }
  return true;
}

template <std::size_t Size>
bool all_finite(const std::array<double, Size> &values) noexcept {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

template <std::size_t Size>
bool all_zero(const std::array<double, Size> &values) noexcept {
  return std::all_of(values.begin(), values.end(), [](double value) { return value == 0.0; });
}

}  // namespace

TruthOdometryReadStatus read_truth_odometry_record(std::istream &input, TruthOdometryRecord &record,
                                                   std::string &error) {
  record = TruthOdometryRecord{};
  error.clear();

  std::array<std::uint8_t, kTruthOdometryHeaderBytes> header{};
  input.read(reinterpret_cast<char *>(header.data()), static_cast<std::streamsize>(header.size()));
  const std::streamsize header_read = input.gcount();
  if (header_read == 0 && input.eof()) {
    return TruthOdometryReadStatus::Eof;
  }
  if (header_read != static_cast<std::streamsize>(header.size())) {
    error = "truncated LTOD header";
    return TruthOdometryReadStatus::Error;
  }
  if (std::memcmp(header.data(), kMagic.data(), kMagic.size()) != 0) {
    error = "bad LTOD magic";
    return TruthOdometryReadStatus::Error;
  }
  if (read_u16_le(header.data() + 4) != kTruthOdometryProtocolVersion) {
    error = "unsupported LTOD protocol version";
    return TruthOdometryReadStatus::Error;
  }
  const std::uint16_t flags = read_u16_le(header.data() + 6);
  if ((flags & static_cast<std::uint16_t>(~kTruthOdometryAllFlags)) != 0) {
    error = "LTOD flags contain unknown bits";
    return TruthOdometryReadStatus::Error;
  }
  if (read_u32_le(header.data() + 8) != kTruthOdometryHeaderBytes ||
      read_u32_le(header.data() + 12) != kTruthOdometryPayloadBytes) {
    error = "LTOD record size mismatch";
    return TruthOdometryReadStatus::Error;
  }

  const auto *session_bytes = header.data() + 16;
  std::size_t session_size = 0;
  while (session_size < 64 && session_bytes[session_size] != 0) ++session_size;
  record.session_id.assign(reinterpret_cast<const char *>(session_bytes), session_size);
  if (!valid_id(record.session_id)) {
    error = "LTOD session_id is invalid";
    return TruthOdometryReadStatus::Error;
  }
  for (std::size_t index = session_size; index < 64; ++index) {
    if (session_bytes[index] != 0) {
      error = "LTOD session_id is not zero padded";
      return TruthOdometryReadStatus::Error;
    }
  }
  record.model_generation = read_u64_le(header.data() + 80);
  record.reset_generation = read_u64_le(header.data() + 88);
  record.sequence = read_u64_le(header.data() + 96);
  record.timestamp_ns = read_u64_le(header.data() + 104);
  if (record.timestamp_ns > kMaximumIdlTimeNs) {
    error = "LTOD timestamp is outside the canonical IDL Time range";
    return TruthOdometryReadStatus::Error;
  }
  record.has_linear_velocity = (flags & kTruthOdometryHasLinearVelocity) != 0;
  record.has_angular_velocity = (flags & kTruthOdometryHasAngularVelocity) != 0;
  record.has_pose_covariance = (flags & kTruthOdometryHasPoseCovariance) != 0;
  record.has_twist_covariance = (flags & kTruthOdometryHasTwistCovariance) != 0;

  std::array<std::uint8_t, kTruthOdometryPayloadBytes> payload{};
  if (!read_exact(input, payload)) {
    error = "truncated LTOD payload";
    return TruthOdometryReadStatus::Error;
  }
  std::array<double, kTruthOdometryPayloadDoubleCount> values{};
  for (std::size_t index = 0; index < values.size(); ++index) {
    values[index] = read_double_le(payload.data() + index * sizeof(double));
  }
  copy_values(values, 0, record.position_m);
  copy_values(values, 3, record.orientation_wxyz);
  copy_values(values, 7, record.linear_velocity_mps);
  copy_values(values, 10, record.angular_velocity_rps);
  copy_values(values, 13, record.pose_covariance);
  copy_values(values, 49, record.twist_covariance);
  if (!all_finite(record.position_m) || !all_finite(record.orientation_wxyz) ||
      !all_finite(record.linear_velocity_mps) || !all_finite(record.angular_velocity_rps) ||
      !all_finite(record.pose_covariance) || !all_finite(record.twist_covariance)) {
    error = "LTOD payload values must be finite";
    return TruthOdometryReadStatus::Error;
  }
  if (all_zero(record.orientation_wxyz)) {
    error = "LTOD orientation quaternion must be non-zero";
    return TruthOdometryReadStatus::Error;
  }
  if (!record.has_linear_velocity && !all_zero(record.linear_velocity_mps)) {
    error = "LTOD absent linear velocity must be encoded as zeros";
    return TruthOdometryReadStatus::Error;
  }
  if (!record.has_angular_velocity && !all_zero(record.angular_velocity_rps)) {
    error = "LTOD absent angular velocity must be encoded as zeros";
    return TruthOdometryReadStatus::Error;
  }
  if (!record.has_pose_covariance && !all_zero(record.pose_covariance)) {
    error = "LTOD absent pose covariance must be encoded as zeros";
    return TruthOdometryReadStatus::Error;
  }
  if (!record.has_twist_covariance && !all_zero(record.twist_covariance)) {
    error = "LTOD absent twist covariance must be encoded as zeros";
    return TruthOdometryReadStatus::Error;
  }
  return TruthOdometryReadStatus::Ok;
}

TruthOdometryStreamValidator::TruthOdometryStreamValidator(std::string expected_session_id)
    : expected_session_id_(std::move(expected_session_id)) {}

bool TruthOdometryStreamValidator::accept(const TruthOdometryRecord &record, std::string &error) {
  error.clear();
  if (record.session_id != expected_session_id_) {
    error = "truth odometry session id mismatch";
    return false;
  }
  if (!initialized_ && record.sequence != 0) {
    error = "truth odometry generation must start at sequence zero";
    return false;
  }
  if (initialized_) {
    if (record.model_generation < model_generation_) {
      error = "truth odometry model generation must not move backward";
      return false;
    }
    const bool model_changed = record.model_generation > model_generation_;
    if (!model_changed && record.reset_generation < reset_generation_) {
      error = "truth odometry reset generation must not move backward";
      return false;
    }
    const bool generation_changed = model_changed || record.reset_generation != reset_generation_;
    if (generation_changed) {
      if (record.sequence != 0) {
        error = "truth odometry generation must start at sequence zero";
        return false;
      }
    } else {
      if (record.sequence <= sequence_) {
        error = "truth odometry sequence must increase within one generation";
        return false;
      }
      if (record.timestamp_ns <= timestamp_ns_) {
        error = "truth odometry timestamp must increase within one generation";
        return false;
      }
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
