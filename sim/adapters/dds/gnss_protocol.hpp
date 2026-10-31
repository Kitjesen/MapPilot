#pragma once

#include <array>
#include <cstdint>
#include <iosfwd>
#include <string>

namespace lingtu::sim::dds_adapter {

inline constexpr std::uint16_t kGnssProtocolVersion = 1;
inline constexpr std::uint16_t kGnssAllFlags = 7; // WGS84 degrees/metres, ENU covariance, simulation clock.
inline constexpr std::uint32_t kGnssHeaderBytes = 176;
inline constexpr std::uint32_t kGnssPayloadBytes = 124;

struct GnssRecord {
  std::string session_id;
  std::uint64_t model_generation{0};
  std::uint64_t reset_generation{0};
  std::uint64_t sequence{0};
  std::uint64_t timestamp_ns{0};
  std::string frame_id;
  std::int32_t fix_type{0};
  std::uint32_t num_sat{0};
  std::uint32_t num_sat_used{0};
  double latitude{0};
  double longitude{0};
  double altitude{0};
  std::array<double, 9> position_covariance{};
  double hdop{0};
  double rtcm_age_s{0};
};

bool valid_gnss_session_id(const std::string& value) noexcept;
enum class GnssReadStatus { Ok, Eof, Error };
GnssReadStatus read_gnss_record(std::istream& input, GnssRecord& record, std::string& error);

class GnssStreamValidator final {
 public:
  explicit GnssStreamValidator(std::string expected_session_id);
  bool accept(const GnssRecord& record, std::string& error);
 private:
  std::string expected_session_id_;
  bool initialized_{false};
  GnssRecord previous_;
};

} // namespace lingtu::sim::dds_adapter
