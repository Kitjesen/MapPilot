#pragma once

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace lingtu::maps {

// Parsed JSON document. Numbers are finite doubles; parsing rejects NaN,
// infinities, duplicate trailing data, and invalid escapes.
struct JsonValue {
  using Array = std::vector<JsonValue>;
  using Object = std::map<std::string, JsonValue>;
  using Storage = std::variant<std::nullptr_t, bool, double, std::string, Array, Object>;
  Storage value{nullptr};

  const Object& AsObject(std::string_view context) const {
    const auto* object = std::get_if<Object>(&value);
    if (object == nullptr) {
      throw std::invalid_argument(std::string(context) + " must be a JSON object");
    }
    return *object;
  }

  const Array& AsArray(std::string_view context) const {
    const auto* array = std::get_if<Array>(&value);
    if (array == nullptr) {
      throw std::invalid_argument(std::string(context) + " must be a JSON array");
    }
    return *array;
  }

  const std::string& AsString(std::string_view context) const {
    const auto* text = std::get_if<std::string>(&value);
    if (text == nullptr) {
      throw std::invalid_argument(std::string(context) + " must be a JSON string");
    }
    return *text;
  }

  std::uint64_t AsUnsigned(std::string_view context) const {
    const auto* number = std::get_if<double>(&value);
    if (number == nullptr || !std::isfinite(*number) || *number < 0.0 ||
        std::floor(*number) != *number ||
        *number > static_cast<double>(std::numeric_limits<std::uint64_t>::max())) {
      throw std::invalid_argument(std::string(context) + " must be a non-negative integer");
    }
    return static_cast<std::uint64_t>(*number);
  }
};

// Parse one complete JSON document; throws std::invalid_argument on error.
JsonValue ParseJson(std::string_view input);

// Escape a string for inclusion between JSON quotes, including every control
// character, so emitted documents always parse.
std::string JsonEscape(std::string_view value);
// JsonEscape wrapped in quotes.
std::string JsonString(std::string_view value);

bool IsValidJsonObject(std::string_view input) noexcept;
bool JsonObjectHasPath(
    std::string_view input,
    std::initializer_list<std::string_view> path) noexcept;
std::optional<bool> JsonObjectBoolAtPath(
    std::string_view input,
    std::initializer_list<std::string_view> path) noexcept;
std::optional<double> JsonObjectNumberAtPath(
    std::string_view input,
    std::initializer_list<std::string_view> path) noexcept;
std::optional<std::string> JsonObjectStringAtPath(
    std::string_view input,
    std::initializer_list<std::string_view> path) noexcept;
std::optional<std::vector<std::string>> JsonObjectPathList(
    std::string_view input,
    std::string_view key) noexcept;

}  // namespace lingtu::maps
