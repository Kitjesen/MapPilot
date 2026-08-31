#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <string>

namespace lingtu::sim::dds_adapter {

inline constexpr std::uint16_t kTruthOdometryProtocolVersion = 1;
inline constexpr std::uint16_t kTruthOdometryHasLinearVelocity = 1U << 0U;
inline constexpr std::uint16_t kTruthOdometryHasAngularVelocity = 1U << 1U;
inline constexpr std::uint16_t kTruthOdometryHasPoseCovariance = 1U << 2U;
inline constexpr std::uint16_t kTruthOdometryHasTwistCovariance = 1U << 3U;
inline constexpr std::uint16_t kTruthOdometryAllFlags =
    kTruthOdometryHasLinearVelocity | kTruthOdometryHasAngularVelocity |
    kTruthOdometryHasPoseCovariance | kTruthOdometryHasTwistCovariance;
inline constexpr std::uint32_t kTruthOdometryHeaderBytes = 112;
inline constexpr std::size_t kTruthOdometryPayloadDoubleCount = 85;
inline constexpr std::uint32_t kTruthOdometryPayloadBytes =
    static_cast<std::uint32_t>(kTruthOdometryPayloadDoubleCount * sizeof(double));

struct TruthOdometryRecord {
  std::string session_id;
  std::uint64_t model_generation{0};
  std::uint64_t reset_generation{0};
  std::uint64_t sequence{0};
  std::uint64_t timestamp_ns{0};
  std::array<double, 3> position_m{};
  std::array<double, 4> orientation_wxyz{};
  std::array<double, 3> linear_velocity_mps{};
  std::array<double, 3> angular_velocity_rps{};
  std::array<double, 36> pose_covariance{};
  std::array<double, 36> twist_covariance{};
  bool has_linear_velocity{false};
  bool has_angular_velocity{false};
  bool has_pose_covariance{false};
  bool has_twist_covariance{false};
};

enum class TruthOdometryReadStatus {
  Ok,
  Eof,
  Error,
};

TruthOdometryReadStatus read_truth_odometry_record(std::istream &input, TruthOdometryRecord &record,
                                                   std::string &error);

class TruthOdometryStreamValidator final {
 public:
  explicit TruthOdometryStreamValidator(std::string expected_session_id);

  bool accept(const TruthOdometryRecord &record, std::string &error);

 private:
  std::string expected_session_id_;
  bool initialized_{false};
  std::uint64_t model_generation_{0};
  std::uint64_t reset_generation_{0};
  std::uint64_t sequence_{0};
  std::uint64_t timestamp_ns_{0};
};

}  // namespace lingtu::sim::dds_adapter
