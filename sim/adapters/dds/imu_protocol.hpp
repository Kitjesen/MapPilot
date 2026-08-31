#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <string>

namespace lingtu::sim::dds_adapter {

inline constexpr std::uint16_t kImuProtocolVersion = 1;
inline constexpr std::uint16_t kImuHasOrientation = 1U << 0U;
inline constexpr std::uint16_t kImuHasGyro = 1U << 1U;
inline constexpr std::uint16_t kImuHasAcceleration = 1U << 2U;
inline constexpr std::uint16_t kImuUnitsSi = 1U << 3U;
inline constexpr std::uint16_t kImuOrientationWxyz = 1U << 4U;
inline constexpr std::uint16_t kImuAllFlags =
    kImuHasOrientation | kImuHasGyro | kImuHasAcceleration | kImuUnitsSi |
    kImuOrientationWxyz;
inline constexpr std::uint32_t kImuHeaderBytes = 176;
inline constexpr std::uint32_t kImuPayloadBytes = 10U * sizeof(double);
inline constexpr std::size_t kImuFrameBytes = 64;

struct ImuRecord {
  std::string session_id;
  std::uint64_t model_generation{0};
  std::uint64_t reset_generation{0};
  std::uint64_t sequence{0};
  std::uint64_t timestamp_ns{0};
  std::string frame_id;
  std::array<double, 4> orientation_wxyz{};
  std::array<double, 3> angular_velocity_rps{};
  std::array<double, 3> linear_acceleration_mps2{};
};

enum class ImuReadStatus { Ok, Eof, Error };

ImuReadStatus read_imu_record(std::istream& input, ImuRecord& record,
                              std::string& error);

class ImuStreamValidator final {
 public:
  explicit ImuStreamValidator(std::string expected_session_id);
  bool accept(const ImuRecord& record, std::string& error);

 private:
  std::string expected_session_id_;
  bool initialized_{false};
  std::uint64_t model_generation_{0};
  std::uint64_t reset_generation_{0};
  std::uint64_t sequence_{0};
  std::uint64_t timestamp_ns_{0};
};

}  // namespace lingtu::sim::dds_adapter
