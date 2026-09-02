#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "truth_odom_protocol.hpp"

namespace {

namespace adapter = lingtu::sim::dds_adapter;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void append_u16(std::string &output, std::uint16_t value) {
  for (int index = 0; index < 2; ++index) {
    output.push_back(static_cast<char>((value >> (8U * index)) & 0xffU));
  }
}

void append_u32(std::string &output, std::uint32_t value) {
  for (int index = 0; index < 4; ++index) {
    output.push_back(static_cast<char>((value >> (8U * index)) & 0xffU));
  }
}

void append_u64(std::string &output, std::uint64_t value) {
  for (int index = 0; index < 8; ++index) {
    output.push_back(static_cast<char>((value >> (8U * index)) & 0xffU));
  }
}

void append_double(std::string &output, double value) {
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "unexpected double size");
  std::memcpy(&bits, &value, sizeof(bits));
  append_u64(output, bits);
}

void set_u16(std::string &output, std::size_t offset, std::uint16_t value) {
  for (int index = 0; index < 2; ++index) {
    output[offset + static_cast<std::size_t>(index)] =
        static_cast<char>((value >> (8U * index)) & 0xffU);
  }
}

void set_u64(std::string &output, std::size_t offset, std::uint64_t value) {
  for (int index = 0; index < 8; ++index) {
    output[offset + static_cast<std::size_t>(index)] =
        static_cast<char>((value >> (8U * index)) & 0xffU);
  }
}

void set_double(std::string &output, std::size_t index, double value) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  set_u64(output, adapter::kTruthOdometryHeaderBytes + index * sizeof(double), bits);
}

std::string valid_record_bytes() {
  constexpr char kSessionId[] = "sim-session";
  std::array<double, adapter::kTruthOdometryPayloadDoubleCount> values{};
  values[0] = 1.0;
  values[1] = 2.0;
  values[2] = 3.0;
  values[3] = 1.0;  // quaternion w
  values[7] = 4.0;
  values[10] = 0.5;
  values[13] = 0.01;
  values[49] = 0.02;

  std::string output("LTOD", 4);
  append_u16(output, adapter::kTruthOdometryProtocolVersion);
  append_u16(output, adapter::kTruthOdometryAllFlags);
  append_u32(output, adapter::kTruthOdometryHeaderBytes);
  append_u32(output, adapter::kTruthOdometryPayloadBytes);
  output.append(kSessionId);
  output.append(64 - std::strlen(kSessionId), '\0');
  append_u64(output, 7);
  append_u64(output, 3);
  append_u64(output, 0);
  append_u64(output, 123456789);
  for (const double value : values) {
    append_double(output, value);
  }
  return output;
}

void valid_v1_record_decodes_and_starts_stream() {
  std::istringstream input(valid_record_bytes());
  adapter::TruthOdometryRecord record;
  std::string error;

  require(adapter::read_truth_odometry_record(input, record, error) ==
              adapter::TruthOdometryReadStatus::Ok,
          "valid truth odometry record must decode");
  require(error.empty(), "valid record must not set an error");
  require(record.model_generation == 7, "model generation mismatch");
  require(record.reset_generation == 3, "reset generation mismatch");
  require(record.sequence == 0, "sequence mismatch");
  require(record.timestamp_ns == 123456789, "timestamp mismatch");
  require(record.position_m == std::array<double, 3>{1.0, 2.0, 3.0}, "position mismatch");
  require(record.orientation_wxyz == std::array<double, 4>{1.0, 0.0, 0.0, 0.0},
          "orientation mismatch");
  require(record.linear_velocity_mps == std::array<double, 3>{4.0, 0.0, 0.0},
          "linear velocity mismatch");
  require(record.has_linear_velocity, "linear velocity flag mismatch");
  require(record.has_angular_velocity, "angular velocity flag mismatch");
  require(record.has_pose_covariance, "pose covariance flag mismatch");
  require(record.has_twist_covariance, "twist covariance flag mismatch");

  adapter::TruthOdometryStreamValidator validator(record.session_id);
  require(validator.accept(record, error), "first sequence-zero record must be accepted");
}

void require_parse_error(std::string bytes, const char *expected) {
  std::istringstream input(std::move(bytes));
  adapter::TruthOdometryRecord record;
  std::string error;
  require(adapter::read_truth_odometry_record(input, record, error) ==
              adapter::TruthOdometryReadStatus::Error,
          "malformed truth odometry record must fail closed");
  require(error.find(expected) != std::string::npos,
          "truth odometry parse error must be actionable");
}

void rejects_noncanonical_protocol_records() {
  auto unsupported_version = valid_record_bytes();
  set_u16(unsupported_version, 4, adapter::kTruthOdometryProtocolVersion + 1U);
  require_parse_error(std::move(unsupported_version), "version");

  auto unknown_flags = valid_record_bytes();
  set_u16(unknown_flags, 6, static_cast<std::uint16_t>(adapter::kTruthOdometryAllFlags | 0x10U));
  require_parse_error(std::move(unknown_flags), "flags");

  auto bad_session = valid_record_bytes();
  bad_session[16] = ' ';
  require_parse_error(std::move(bad_session), "session_id");

  auto overlong_session = valid_record_bytes();
  std::fill_n(overlong_session.begin() + 16, 64, static_cast<std::uint8_t>('a'));
  require_parse_error(std::move(overlong_session), "session_id");

  auto non_finite = valid_record_bytes();
  set_double(non_finite, 0, std::numeric_limits<double>::quiet_NaN());
  require_parse_error(std::move(non_finite), "finite");

  auto zero_orientation = valid_record_bytes();
  set_double(zero_orientation, 3, 0.0);
  require_parse_error(std::move(zero_orientation), "orientation");

  auto undeclared_velocity = valid_record_bytes();
  set_u16(undeclared_velocity, 6,
          static_cast<std::uint16_t>(adapter::kTruthOdometryAllFlags &
                                     ~adapter::kTruthOdometryHasLinearVelocity));
  require_parse_error(std::move(undeclared_velocity), "linear velocity");

  auto out_of_range_time = valid_record_bytes();
  constexpr std::uint64_t kFirstUnrepresentableTimeNs =
      (static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()) + 1U) * 1000000000ULL;
  set_u64(out_of_range_time, 104, kFirstUnrepresentableTimeNs);
  require_parse_error(std::move(out_of_range_time), "IDL Time");

  auto truncated = valid_record_bytes();
  truncated.pop_back();
  require_parse_error(std::move(truncated), "truncated");
}

adapter::TruthOdometryRecord valid_record() {
  std::istringstream input(valid_record_bytes());
  adapter::TruthOdometryRecord record;
  std::string error;
  require(adapter::read_truth_odometry_record(input, record, error) ==
              adapter::TruthOdometryReadStatus::Ok,
          "test fixture must decode");
  return record;
}

void require_rejected(adapter::TruthOdometryStreamValidator &validator,
                      const adapter::TruthOdometryRecord &record, const char *expected) {
  std::string error;
  require(!validator.accept(record, error), "invalid stream transition must fail closed");
  require(error.find(expected) != std::string::npos, "stream validation error must be actionable");
}

void validates_stream_identity_and_monotonicity() {
  const auto first = valid_record();

  adapter::TruthOdometryStreamValidator wrong_session(std::string(64, 'a'));
  require_rejected(wrong_session, first, "session");

  auto nonzero_first = first;
  nonzero_first.sequence = 1;
  adapter::TruthOdometryStreamValidator first_sequence(first.session_id);
  require_rejected(first_sequence, nonzero_first, "sequence zero");

  adapter::TruthOdometryStreamValidator sequence(first.session_id);
  std::string error;
  require(sequence.accept(first, error), "initial record must be accepted");
  auto second = first;
  second.sequence = 1;
  ++second.timestamp_ns;
  require(sequence.accept(second, error), "strictly increasing record must be accepted");
  auto duplicate = second;
  ++duplicate.timestamp_ns;
  require_rejected(sequence, duplicate, "sequence");
  auto third = second;
  third.sequence = 2;
  ++third.timestamp_ns;
  require(sequence.accept(third, error), "rejection must not advance validator state");

  adapter::TruthOdometryStreamValidator timestamp(first.session_id);
  require(timestamp.accept(first, error), "initial timestamp record must be accepted");
  auto repeated_time = first;
  repeated_time.sequence = 1;
  require_rejected(timestamp, repeated_time, "timestamp");

  adapter::TruthOdometryStreamValidator generations(first.session_id);
  require(generations.accept(first, error), "initial generation must be accepted");
  auto reset_backward = first;
  --reset_backward.reset_generation;
  require_rejected(generations, reset_backward, "reset generation");
  auto model_backward = first;
  --model_backward.model_generation;
  require_rejected(generations, model_backward, "model generation");

  auto next_reset = first;
  ++next_reset.reset_generation;
  next_reset.timestamp_ns = 0;
  require(generations.accept(next_reset, error),
          "new reset generation may restart sequence and simulation time");
  auto bad_generation_sequence = next_reset;
  ++bad_generation_sequence.model_generation;
  bad_generation_sequence.sequence = 1;
  require_rejected(generations, bad_generation_sequence, "sequence zero");
}

}  // namespace

int main() {
  try {
    valid_v1_record_decodes_and_starts_stream();
    rejects_noncanonical_protocol_records();
    validates_stream_identity_and_monotonicity();
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "test_truth_odom_protocol failed: %s\n", error.what());
    return 1;
  }
}
