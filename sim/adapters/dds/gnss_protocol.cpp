#include "gnss_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <istream>
#include <limits>
#include <utility>

namespace lingtu::sim::dds_adapter {
namespace {
std::uint64_t integer(const std::uint8_t* p, std::size_t size) noexcept {
  std::uint64_t value = 0;
  for (std::size_t i = 0; i < size; ++i) value |= static_cast<std::uint64_t>(p[i]) << (8 * i);
  return value;
}
double real(const std::uint8_t* p) noexcept {
  const auto bits = integer(p, 8);
  double value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
bool text(const std::uint8_t* p, std::string& value) {
  std::size_t size = 0;
  while (size < 64 && p[size] != 0) ++size;
  if (size == 0 || size == 64) return false;
  for (std::size_t i = 0; i < size; ++i) if (p[i] < 33 || p[i] > 126) return false;
  for (std::size_t i = size; i < 64; ++i) if (p[i] != 0) return false;
  value.assign(reinterpret_cast<const char*>(p), size);
  return true;
}
bool covariance(const std::array<double, 9>& c) {
  double scale = 0;
  for (double v : c) {
    if (!std::isfinite(v)) return false;
    scale = std::max(scale, std::abs(v));
  }
  if (c[0] < 0 || c[4] < 0 || c[8] < 0) return false;
  if (scale == 0) return true;
  std::array<double, 9> a{};
  for (std::size_t i = 0; i < 9; ++i) a[i] = c[i] / scale;
  constexpr double tolerance = 1e-10;
  if (std::abs(a[1] - a[3]) > tolerance || std::abs(a[2] - a[6]) > tolerance ||
      std::abs(a[5] - a[7]) > tolerance) return false;
  // A covariance must be positive semidefinite (all principal minors >= 0).
  if (a[0]*a[4] - a[1]*a[3] < -tolerance ||
      a[0]*a[8] - a[2]*a[6] < -tolerance ||
      a[4]*a[8] - a[5]*a[7] < -tolerance) return false;
  const double determinant = a[0]*(a[4]*a[8]-a[5]*a[7]) -
      a[1]*(a[3]*a[8]-a[5]*a[6]) + a[2]*(a[3]*a[7]-a[4]*a[6]);
  return determinant >= -tolerance;
}
} // namespace

bool valid_gnss_session_id(const std::string& value) noexcept {
  if (value.empty() || value.size() > 63) return false;
  for (std::size_t i = 0; i < value.size(); ++i) {
    const char c = value[i];
    const bool alnum = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9');
    if (!alnum && (i == 0 || (c != '_' && c != '.' && c != '-'))) return false;
  }
  return true;
}

GnssReadStatus read_gnss_record(std::istream& input, GnssRecord& record, std::string& error) {
  record = GnssRecord{};
  error.clear();
  const auto reject = [&error](const char* message) { error = message; return GnssReadStatus::Error; };
  std::array<std::uint8_t, kGnssHeaderBytes> header{};
  input.read(reinterpret_cast<char*>(header.data()), static_cast<std::streamsize>(header.size()));
  if (input.gcount() == 0 && input.eof()) return GnssReadStatus::Eof;
  if (input.gcount() != static_cast<std::streamsize>(header.size())) return reject("truncated LTGN header");
  if (std::memcmp(header.data(), "LTGN", 4) != 0) return reject("bad LTGN magic");
  if (integer(header.data()+4, 2) != kGnssProtocolVersion) return reject("unsupported LTGN version");
  if (integer(header.data()+6, 2) != kGnssAllFlags) return reject("LTGN flags must declare WGS84/ENU/simulation clock");
  if (integer(header.data()+8, 4) != kGnssHeaderBytes || integer(header.data()+12, 4) != kGnssPayloadBytes)
    return reject("LTGN record size mismatch");
  if (!text(header.data()+16, record.session_id) || !valid_gnss_session_id(record.session_id))
    return reject("invalid LTGN session_id");
  record.model_generation = integer(header.data()+80, 8);
  record.reset_generation = integer(header.data()+88, 8);
  record.sequence = integer(header.data()+96, 8);
  record.timestamp_ns = integer(header.data()+104, 8);
  constexpr auto max_time = static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()) *
      1000000000ULL + 999999999ULL;
  if (record.timestamp_ns > max_time) return reject("LTGN time outside canonical IDL Time range");
  if (!text(header.data()+112, record.frame_id)) return reject("invalid LTGN frame_id");
  std::array<std::uint8_t, kGnssPayloadBytes> payload{};
  input.read(reinterpret_cast<char*>(payload.data()), static_cast<std::streamsize>(payload.size()));
  if (input.gcount() != static_cast<std::streamsize>(payload.size())) return reject("truncated LTGN payload");
  const auto fix_type = integer(payload.data(), 4);
  if (fix_type != 0 && fix_type != 8) return reject("LTGN permits only NO_FIX or SIMULATION");
  record.fix_type = static_cast<std::int32_t>(fix_type);
  record.num_sat = static_cast<std::uint32_t>(integer(payload.data()+4, 4));
  record.num_sat_used = static_cast<std::uint32_t>(integer(payload.data()+8, 4));
  record.latitude = real(payload.data()+12);
  record.longitude = real(payload.data()+20);
  record.altitude = real(payload.data()+28);
  for (std::size_t i = 0; i < 9; ++i) record.position_covariance[i] = real(payload.data()+36+i*8);
  record.hdop = real(payload.data()+108);
  record.rtcm_age_s = real(payload.data()+116);
  if (record.num_sat != 0 || record.num_sat_used != 0 || !std::isnan(record.hdop) || !std::isnan(record.rtcm_age_s))
    return reject("LTGN simulation must not invent satellites, HDOP or RTCM age");
  if (record.fix_type == 0) {
    if (!std::isnan(record.latitude) || !std::isnan(record.longitude) || !std::isnan(record.altitude) ||
        !std::all_of(record.position_covariance.begin(), record.position_covariance.end(),
                     [](double v) { return std::isnan(v); })) return reject("LTGN NO_FIX requires NaN coordinates/covariance");
  } else if (!std::isfinite(record.latitude) || record.latitude < -90 || record.latitude > 90 ||
             !std::isfinite(record.longitude) || record.longitude < -180 || record.longitude > 180 ||
             !std::isfinite(record.altitude) || !covariance(record.position_covariance)) {
    return reject("LTGN SIMULATION requires valid WGS84 coordinates and ENU covariance");
  }
  return GnssReadStatus::Ok;
}

GnssStreamValidator::GnssStreamValidator(std::string expected_session_id)
    : expected_session_id_(std::move(expected_session_id)) {}

bool GnssStreamValidator::accept(const GnssRecord& record, std::string& error) {
  error.clear();
  if (record.session_id != expected_session_id_) { error = "GNSS session id mismatch"; return false; }
  if (initialized_ && (record.model_generation < previous_.model_generation ||
      (record.model_generation == previous_.model_generation && record.reset_generation < previous_.reset_generation))) {
    error = "GNSS generation moved backward"; return false;
  }
  const bool changed = !initialized_ || record.model_generation != previous_.model_generation ||
      record.reset_generation != previous_.reset_generation;
  if (changed && record.sequence != 0) { error = "GNSS generation must start at sequence zero"; return false; }
  if (!changed && (record.sequence <= previous_.sequence || record.timestamp_ns <= previous_.timestamp_ns ||
                   record.frame_id != previous_.frame_id)) {
    error = "GNSS sequence/time must increase and frame must match within a generation"; return false;
  }
  previous_ = record;
  initialized_ = true;
  return true;
}
} // namespace lingtu::sim::dds_adapter
