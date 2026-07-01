#pragma once

#include <optional>
#include <string>

#include "wtrtk980/gps_fix.hpp"

namespace wtrtk980 {

std::optional<double> nmeaCoordToDecimal(const std::string& value, const std::string& hemisphere);
bool parseNmeaLine(const std::string& raw, GpsFix& state);

}  // namespace wtrtk980
