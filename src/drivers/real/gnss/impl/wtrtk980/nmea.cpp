#include "wtrtk980/nmea.hpp"

#include <sstream>
#include <vector>

namespace lingtu::drivers::wtrtk980 {
namespace {

std::vector<std::string> split(const std::string& text, char delim) {
  std::vector<std::string> out;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, delim)) {
    out.push_back(item);
  }
  return out;
}

std::optional<double> to_double(const std::string& value) {
  if (value.empty()) {
    return std::nullopt;
  }
  try {
    std::size_t pos = 0;
    const double parsed = std::stod(value, &pos);
    if (pos != value.size()) {
      return std::nullopt;
    }
    return parsed;
  } catch (...) {
    return std::nullopt;
  }
}

int to_int_or_zero(const std::string& value) {
  if (value.empty()) {
    return 0;
  }
  try {
    return std::stoi(value);
  } catch (...) {
    return 0;
  }
}

void parse_gga(const std::vector<std::string>& parts, lingtu::gnss::FixSample& state) {
  if (parts.size() > 1 && !parts[1].empty()) {
    state.timestamp_utc = parts[1];
  }
  if (parts.size() > 6) {
    state.latitude_deg = nmea_coord_to_decimal(parts[2], parts[3]);
    state.longitude_deg = nmea_coord_to_decimal(parts[4], parts[5]);
    state.fix_type = to_int_or_zero(parts[6]);
  }
  if (parts.size() > 7) {
    state.num_sat = to_int_or_zero(parts[7]);
    state.num_sat_used = state.num_sat;
  }
  if (parts.size() > 8) {
    state.hdop = to_double(parts[8]);
  }
  if (parts.size() > 9) {
    state.altitude_m = to_double(parts[9]);
  }
  if (parts.size() > 13) {
    state.rtcm_age_s = to_double(parts[13]);
  }
}

void parse_rmc(const std::vector<std::string>& parts, lingtu::gnss::FixSample& state) {
  if (parts.size() > 1 && !parts[1].empty()) {
    state.timestamp_utc = parts[1];
  }
  if (parts.size() > 2 && parts[2] == "A" && state.fix_type == 0) {
    state.fix_type = 1;
  }
  if (parts.size() > 6) {
    state.latitude_deg = nmea_coord_to_decimal(parts[3], parts[4]);
    state.longitude_deg = nmea_coord_to_decimal(parts[5], parts[6]);
  }
  if (parts.size() > 7) {
    state.speed_knots = to_double(parts[7]);
  }
  if (parts.size() > 8) {
    state.course_deg = to_double(parts[8]);
  }
  if (parts.size() > 9 && !parts[9].empty()) {
    state.date_utc = parts[9];
  }
}

}  // namespace

std::optional<double> nmea_coord_to_decimal(
    const std::string& value,
    const std::string& hemisphere) {
  const auto raw = to_double(value);
  if (!raw || hemisphere.empty()) {
    return std::nullopt;
  }
  const int degrees = static_cast<int>(*raw / 100.0);
  const double minutes = *raw - static_cast<double>(degrees) * 100.0;
  double decimal = static_cast<double>(degrees) + minutes / 60.0;
  if (hemisphere == "S" || hemisphere == "W") {
    decimal = -decimal;
  }
  return decimal;
}

bool NmeaParser::parse_line(const std::string& raw) {
  if (raw.empty() || raw[0] != '$') {
    return false;
  }
  const std::string sentence = raw.substr(0, raw.find('*'));
  const auto parts = split(sentence, ',');
  if (parts.empty() || parts[0].size() < 4) {
    return false;
  }

  const std::string kind = parts[0].substr(parts[0].size() - 3);
  if (kind == "GGA") {
    parse_gga(parts, state_);
  } else if (kind == "RMC") {
    parse_rmc(parts, state_);
  } else {
    return false;
  }

  state_.has_fix =
      state_.fix_type > 0 && state_.latitude_deg.has_value() &&
      state_.longitude_deg.has_value();
  state_.last_sentence = raw;
  return true;
}

}  // namespace lingtu::drivers::wtrtk980
