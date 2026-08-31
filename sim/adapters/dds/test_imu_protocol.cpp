#include <array>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>

#include "imu_protocol.hpp"

namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void u16(std::string& out, std::uint16_t value) {
  out.push_back(static_cast<char>(value & 0xffU));
  out.push_back(static_cast<char>((value >> 8U) & 0xffU));
}
void u32(std::string& out, std::uint32_t value) {
  for (int i = 0; i < 4; ++i) out.push_back(static_cast<char>((value >> (i * 8)) & 0xffU));
}
void u64(std::string& out, std::uint64_t value) {
  for (int i = 0; i < 8; ++i) out.push_back(static_cast<char>((value >> (i * 8)) & 0xffU));
}
void dbl(std::string& out, double value) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  u64(out, bits);
}
std::string record(std::uint64_t model = 2, std::uint64_t reset = 3,
                   std::uint64_t sequence = 0, std::uint64_t time = 4000000,
                   const std::string& session_id = "sim-session") {
  std::string out("LTIM", 4);
  u16(out, lingtu::sim::dds_adapter::kImuProtocolVersion);
  u16(out, lingtu::sim::dds_adapter::kImuAllFlags);
  u32(out, lingtu::sim::dds_adapter::kImuHeaderBytes);
  u32(out, lingtu::sim::dds_adapter::kImuPayloadBytes);
  out.append(session_id);
  out.append(64 - session_id.size(), '\0');
  u64(out, model); u64(out, reset); u64(out, sequence); u64(out, time);
  out.append("thunder_01/imu");
  out.append(64 - std::strlen("thunder_01/imu"), '\0');
  for (double value : std::array<double, 10>{1.0, 0.1, 0.2, 0.3, 0.01, 0.02, 0.03,
                                             1.0, 2.0, 9.81}) dbl(out, value);
  return out;
}
}  // namespace

int main() {
  try {
    const auto bytes = record();
    std::istringstream input(bytes);
    lingtu::sim::dds_adapter::ImuRecord parsed;
    std::string error;
    require(lingtu::sim::dds_adapter::read_imu_record(input, parsed, error) ==
                lingtu::sim::dds_adapter::ImuReadStatus::Ok,
            error.c_str());
    require(parsed.frame_id == "thunder_01/imu", "frame roundtrip failed");
    require(parsed.orientation_wxyz[0] == 1.0 && parsed.angular_velocity_rps[2] == 0.03 &&
                parsed.linear_acceleration_mps2[2] == 9.81,
            "IMU values roundtrip failed");

    auto bad_units = bytes;
    bad_units[6] = 0;
    std::istringstream bad_input(bad_units);
    require(lingtu::sim::dds_adapter::read_imu_record(bad_input, parsed, error) ==
                lingtu::sim::dds_adapter::ImuReadStatus::Error,
            "bad units were accepted");

    std::istringstream overlong_input(record(2, 3, 0, 4000000, std::string(64, 'a')));
    require(lingtu::sim::dds_adapter::read_imu_record(overlong_input, parsed, error) ==
                lingtu::sim::dds_adapter::ImuReadStatus::Error,
            "64-character session id was accepted");

    lingtu::sim::dds_adapter::ImuStreamValidator validator("sim-session");
    std::istringstream first_input(bytes);
    require(lingtu::sim::dds_adapter::read_imu_record(first_input, parsed, error) ==
                lingtu::sim::dds_adapter::ImuReadStatus::Ok && validator.accept(parsed, error),
            "first generation sample rejected");
    std::istringstream generation_input(record(2, 4, 0, 8000000));
    require(lingtu::sim::dds_adapter::read_imu_record(generation_input, parsed, error) ==
                lingtu::sim::dds_adapter::ImuReadStatus::Ok && validator.accept(parsed, error),
            "reset generation sample rejected");
    std::istringstream bad_generation(record(2, 5, 1, 10000000));
    require(lingtu::sim::dds_adapter::read_imu_record(bad_generation, parsed, error) ==
                lingtu::sim::dds_adapter::ImuReadStatus::Ok && !validator.accept(parsed, error),
            "non-zero reset sequence accepted");
    return 0;
  } catch (const std::exception& error) {
    return std::fprintf(stderr, "test_imu_protocol failed: %s\n", error.what()), 1;
  }
}
