#pragma once

#include <optional>
#include <string>

#include "native/sdk.hpp"

namespace lingtu::drivers::wtrtk980 {

std::optional<double> nmea_coord_to_decimal(
    const std::string& value,
    const std::string& hemisphere);

class NmeaParser {
 public:
  bool parse_line(const std::string& raw);
  const lingtu::gnss::FixSample& state() const { return state_; }

 private:
  lingtu::gnss::FixSample state_;
};

}  // namespace lingtu::drivers::wtrtk980
