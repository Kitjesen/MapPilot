#include "wtrtk980/nmea_parser.hpp"

#include <iomanip>
#include <sstream>
#include <vector>

namespace wtrtk980 {
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

std::optional<double> toDouble(const std::string& value) {
  if (value.empty()) return std::nullopt;
  try {
    size_t pos = 0;
    double parsed = std::stod(value, &pos);
    if (pos != value.size()) return std::nullopt;
    return parsed;
  } catch (...) {
    return std::nullopt;
  }
}

int toIntOrZero(const std::string& value) {
  if (value.empty()) return 0;
  try {
    return std::stoi(value);
  } catch (...) {
    return 0;
  }
}

void parseGga(const std::vector<std::string>& parts, GpsFix& state) {
  if (parts.size() > 1 && !parts[1].empty()) state.timestamp_utc = parts[1];
  if (parts.size() > 6) {
    state.lat = nmeaCoordToDecimal(parts[2], parts[3]);
    state.lon = nmeaCoordToDecimal(parts[4], parts[5]);
    state.quality = toIntOrZero(parts[6]);
  }
  if (parts.size() > 7) state.satellites = toIntOrZero(parts[7]);
  if (parts.size() > 8) state.hdop = toDouble(parts[8]);
  if (parts.size() > 9) state.altitude_m = toDouble(parts[9]);
}

void parseRmc(const std::vector<std::string>& parts, GpsFix& state) {
  if (parts.size() > 1 && !parts[1].empty()) state.timestamp_utc = parts[1];
  if (parts.size() > 2) state.rmc_valid = parts[2] == "A";
  if (parts.size() > 6) {
    state.lat = nmeaCoordToDecimal(parts[3], parts[4]);
    state.lon = nmeaCoordToDecimal(parts[5], parts[6]);
  }
  if (parts.size() > 7) state.speed_knots = toDouble(parts[7]);
  if (parts.size() > 8) state.course_deg = toDouble(parts[8]);
  if (parts.size() > 9 && !parts[9].empty()) state.date_utc = parts[9];
}

}  // namespace

std::optional<double> nmeaCoordToDecimal(const std::string& value, const std::string& hemisphere) {
  auto raw = toDouble(value);
  if (!raw || hemisphere.empty()) return std::nullopt;
  int degrees = static_cast<int>(*raw / 100.0);
  double minutes = *raw - degrees * 100.0;
  double decimal = degrees + minutes / 60.0;
  if (hemisphere == "S" || hemisphere == "W") decimal = -decimal;
  return decimal;
}

bool parseNmeaLine(const std::string& raw, GpsFix& state) {
  if (raw.empty() || raw[0] != '$') return false;
  std::string sentence = raw.substr(0, raw.find('*'));
  auto parts = split(sentence, ',');
  if (parts.empty() || parts[0].size() < 4) return false;
  std::string kind = parts[0].substr(parts[0].size() - 3);
  if (kind == "GGA") {
    parseGga(parts, state);
  } else if (kind == "RMC") {
    parseRmc(parts, state);
  } else {
    return false;
  }
  state.fix = state.rmc_valid || state.quality > 0;
  state.last_sentence = raw;
  return true;
}

std::string GpsFix::summary() const {
  auto opt = [](const auto& v) -> std::string {
    if (!v) return "None";
    std::ostringstream oss;
    oss << *v;
    return oss.str();
  };
  std::ostringstream oss;
  oss << std::boolalpha
      << "fix=" << fix
      << " quality=" << quality
      << " satellites=" << satellites
      << " lat=" << opt(lat)
      << " lon=" << opt(lon)
      << " alt_m=" << opt(altitude_m)
      << " hdop=" << opt(hdop)
      << " speed_knots=" << opt(speed_knots)
      << " course_deg=" << opt(course_deg);
  return oss.str();
}

}  // namespace wtrtk980
