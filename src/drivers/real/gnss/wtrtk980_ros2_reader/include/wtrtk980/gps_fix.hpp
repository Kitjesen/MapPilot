#pragma once

#include <optional>
#include <string>

namespace wtrtk980 {

struct GpsFix {
  bool fix = false;
  bool rmc_valid = false;
  int quality = 0;
  int satellites = 0;
  std::optional<double> lat;
  std::optional<double> lon;
  std::optional<double> altitude_m;
  std::optional<double> hdop;
  std::optional<double> speed_knots;
  std::optional<double> course_deg;
  std::optional<std::string> timestamp_utc;
  std::optional<std::string> date_utc;
  std::optional<std::string> last_sentence;

  std::string summary() const;
};

}  // namespace wtrtk980
