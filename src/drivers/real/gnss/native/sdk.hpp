#pragma once

#include <optional>
#include <string>

namespace lingtu::gnss {

enum class FixType {
  NoFix = 0,
  Single = 1,
  Dgps = 2,
  Pps = 3,
  RtkFixed = 4,
  RtkFloat = 5,
  Estimated = 6,
  Manual = 7,
  Simulation = 8,
};

struct FixSample {
  bool has_fix{false};
  int fix_type{0};
  int num_sat{0};
  int num_sat_used{0};
  std::optional<double> latitude_deg;
  std::optional<double> longitude_deg;
  std::optional<double> altitude_m;
  std::optional<double> hdop;
  std::optional<double> rtcm_age_s;
  std::optional<double> speed_knots;
  std::optional<double> course_deg;
  std::optional<std::string> timestamp_utc;
  std::optional<std::string> date_utc;
  std::string last_sentence;
};

struct StatusSample {
  std::string device;
  int fix_type{0};
  bool link_ok{false};
  bool rtk{false};
  int num_sat{0};
  int num_sat_used{0};
  double hdop{99.9};
  double rtcm_age_s{99.9};
  std::string error;
};

inline bool is_rtk_fix(int fix_type) {
  return fix_type == static_cast<int>(FixType::RtkFixed) ||
         fix_type == static_cast<int>(FixType::RtkFloat);
}

}  // namespace lingtu::gnss
