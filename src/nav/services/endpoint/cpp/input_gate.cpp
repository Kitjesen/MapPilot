#include "input_gate.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>

namespace lingtu::nav::endpoint {
namespace {

std::string_view jsonValue(std::string_view json, std::string_view key) {
  const auto object_start = json.find_first_not_of(" \t\r\n");
  if (object_start == std::string_view::npos || json[object_start] != '{') {
    return {};
  }

  int depth = 0;
  for (std::size_t i = object_start; i < json.size(); ++i) {
    const char ch = json[i];
    if (ch == '{' || ch == '[') {
      ++depth;
      continue;
    }
    if (ch == '}' || ch == ']') {
      --depth;
      continue;
    }
    if (ch != '"') {
      continue;
    }

    const std::size_t text_start = i + 1;
    bool escaped = false;
    for (++i; i < json.size(); ++i) {
      if (escaped) {
        escaped = false;
      } else if (json[i] == '\\') {
        escaped = true;
      } else if (json[i] == '"') {
        break;
      }
    }
    if (i >= json.size() || depth != 1 ||
        json.substr(text_start, i - text_start) != key) {
      continue;
    }
    const auto colon_pos = json.find_first_not_of(" \t\r\n", i + 1);
    if (colon_pos == std::string_view::npos || json[colon_pos] != ':') {
      continue;
    }
    const auto value_pos = json.find_first_not_of(" \t\r\n", colon_pos + 1);
    if (value_pos != std::string_view::npos) {
      return json.substr(value_pos);
    }
  }
  return {};
}

bool parseString(std::string_view json, std::string_view key, std::string& out) {
  const std::string_view value = jsonValue(json, key);
  if (value.size() < 2 || value.front() != '"') {
    return false;
  }
  const auto end = value.find('"', 1);
  if (end == std::string_view::npos) {
    return false;
  }
  out.assign(value.substr(1, end - 1));
  return true;
}

bool parsePositiveFiniteDouble(
    std::string_view json,
    std::string_view key,
    double& out) {
  const std::string_view value = jsonValue(json, key);
  if (value.empty()) {
    return false;
  }
  const std::string owned(value);
  char* end = nullptr;
  errno = 0;
  const double parsed = std::strtod(owned.c_str(), &end);
  if (end == owned.c_str() || errno == ERANGE || !std::isfinite(parsed) || parsed <= 0.0) {
    return false;
  }
  out = parsed;
  return true;
}

}  // namespace

bool isHealthyLocalizationState(std::string_view state) {
  std::string normalized(state);
  std::transform(
      normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
      });
  return normalized == "TRACKING" || normalized == "LOCKED" ||
         normalized == "RECOVERED" || normalized == "OK";
}

bool isCatastrophicLocalizationReason(std::string_view reason) {
  std::string normalized(reason);
  std::transform(
      normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
  return normalized == "fastlio_state_nonfinite" ||
         normalized == "fastlio_velocity_out_of_bounds" ||
         normalized.find("catastrophic") != std::string::npos;
}

LocalizationHealthSample decodeLocalizationHealth(std::string_view json) {
  LocalizationHealthSample out;
  if (!parseString(json, "state", out.state)) {
    out.error = "localization_health_state_missing";
    return out;
  }
  if (!parsePositiveFiniteDouble(json, "ts", out.stamp_s)) {
    out.error = "localization_health_timestamp_missing";
    return out;
  }
  (void)parseString(json, "reason", out.reason);
  out.valid = true;
  out.healthy = isHealthyLocalizationState(out.state) &&
      !isCatastrophicLocalizationReason(out.reason);
  return out;
}

}  // namespace lingtu::nav::endpoint
