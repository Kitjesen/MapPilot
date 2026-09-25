#include "lingtu/maps/json.hpp"

#include <cctype>
#include <locale>
#include <sstream>

namespace lingtu::maps {
namespace {

class JsonParser final {
 public:
  explicit JsonParser(std::string_view input) : input_(input) {}

  JsonValue Parse() {
    SkipWhitespace();
    JsonValue result = ParseValue();
    SkipWhitespace();
    if (cursor_ != input_.size()) {
      Fail("unexpected trailing JSON data");
    }
    return result;
  }

 private:
  std::string_view input_;
  std::size_t cursor_{0U};

  [[noreturn]] void Fail(const std::string& message) const {
    throw std::invalid_argument(
        "semantic taxonomy JSON error at byte " + std::to_string(cursor_) + ": " + message);
  }

  void SkipWhitespace() {
    while (cursor_ < input_.size() &&
           std::isspace(static_cast<unsigned char>(input_[cursor_])) != 0) {
      ++cursor_;
    }
  }

  bool Consume(char token) {
    if (cursor_ < input_.size() && input_[cursor_] == token) {
      ++cursor_;
      return true;
    }
    return false;
  }

  void Require(char token) {
    if (!Consume(token)) {
      Fail(std::string("expected '") + token + "'");
    }
  }

  JsonValue ParseValue() {
    if (cursor_ >= input_.size()) {
      Fail("expected value");
    }
    switch (input_[cursor_]) {
      case '{':
        return JsonValue{ParseObject()};
      case '[':
        return JsonValue{ParseArray()};
      case '"':
        return JsonValue{ParseString()};
      case 't':
        ParseLiteral("true");
        return JsonValue{true};
      case 'f':
        ParseLiteral("false");
        return JsonValue{false};
      case 'n':
        ParseLiteral("null");
        return JsonValue{nullptr};
      default:
        if (input_[cursor_] == '-' ||
            std::isdigit(static_cast<unsigned char>(input_[cursor_])) != 0) {
          return JsonValue{ParseNumber()};
        }
        Fail("unsupported value");
    }
  }

  JsonValue::Object ParseObject() {
    Require('{');
    SkipWhitespace();
    JsonValue::Object object;
    if (Consume('}')) {
      return object;
    }
    while (true) {
      SkipWhitespace();
      if (cursor_ >= input_.size() || input_[cursor_] != '"') {
        Fail("object key must be a string");
      }
      std::string key = ParseString();
      SkipWhitespace();
      Require(':');
      SkipWhitespace();
      if (!object.emplace(std::move(key), ParseValue()).second) {
        Fail("duplicate object key");
      }
      SkipWhitespace();
      if (Consume('}')) {
        break;
      }
      Require(',');
    }
    return object;
  }

  JsonValue::Array ParseArray() {
    Require('[');
    SkipWhitespace();
    JsonValue::Array array;
    if (Consume(']')) {
      return array;
    }
    while (true) {
      SkipWhitespace();
      array.push_back(ParseValue());
      SkipWhitespace();
      if (Consume(']')) {
        break;
      }
      Require(',');
    }
    return array;
  }

  static void AppendUtf8(std::string* out, std::uint32_t codepoint) {
    if (codepoint <= 0x7FU) {
      out->push_back(static_cast<char>(codepoint));
    } else if (codepoint <= 0x7FFU) {
      out->push_back(static_cast<char>(0xC0U | (codepoint >> 6U)));
      out->push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
    } else {
      out->push_back(static_cast<char>(0xE0U | (codepoint >> 12U)));
      out->push_back(static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3FU)));
      out->push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
    }
  }

  std::uint32_t ParseHex4() {
    if (input_.size() - cursor_ < 4U) {
      Fail("truncated unicode escape");
    }
    std::uint32_t value = 0U;
    for (int i = 0; i < 4; ++i) {
      const char ch = input_[cursor_++];
      value <<= 4U;
      if (ch >= '0' && ch <= '9') {
        value += static_cast<std::uint32_t>(ch - '0');
      } else if (ch >= 'a' && ch <= 'f') {
        value += static_cast<std::uint32_t>(ch - 'a' + 10);
      } else if (ch >= 'A' && ch <= 'F') {
        value += static_cast<std::uint32_t>(ch - 'A' + 10);
      } else {
        Fail("invalid unicode escape");
      }
    }
    return value;
  }

  std::string ParseString() {
    Require('"');
    std::string out;
    while (cursor_ < input_.size()) {
      const char ch = input_[cursor_++];
      if (ch == '"') {
        return out;
      }
      if (static_cast<unsigned char>(ch) < 0x20U) {
        Fail("control character in string");
      }
      if (ch != '\\') {
        out.push_back(ch);
        continue;
      }
      if (cursor_ >= input_.size()) {
        Fail("truncated escape sequence");
      }
      const char escaped = input_[cursor_++];
      switch (escaped) {
        case '"': out.push_back('"'); break;
        case '\\': out.push_back('\\'); break;
        case '/': out.push_back('/'); break;
        case 'b': out.push_back('\b'); break;
        case 'f': out.push_back('\f'); break;
        case 'n': out.push_back('\n'); break;
        case 'r': out.push_back('\r'); break;
        case 't': out.push_back('\t'); break;
        case 'u': {
          const std::uint32_t codepoint = ParseHex4();
          if (codepoint >= 0xD800U && codepoint <= 0xDFFFU) {
            Fail("surrogate unicode escapes are not supported in taxonomy names");
          }
          AppendUtf8(&out, codepoint);
          break;
        }
        default:
          Fail("invalid escape sequence");
      }
    }
    Fail("unterminated string");
  }

  double ParseNumber() {
    const std::size_t begin = cursor_;
    if (Consume('-') && cursor_ >= input_.size()) {
      Fail("truncated number");
    }
    if (Consume('0')) {
      if (cursor_ < input_.size() &&
          std::isdigit(static_cast<unsigned char>(input_[cursor_])) != 0) {
        Fail("number has a leading zero");
      }
    } else {
      if (cursor_ >= input_.size() ||
          std::isdigit(static_cast<unsigned char>(input_[cursor_])) == 0) {
        Fail("invalid number");
      }
      while (cursor_ < input_.size() &&
             std::isdigit(static_cast<unsigned char>(input_[cursor_])) != 0) {
        ++cursor_;
      }
    }
    if (Consume('.')) {
      if (cursor_ >= input_.size() ||
          std::isdigit(static_cast<unsigned char>(input_[cursor_])) == 0) {
        Fail("invalid number fraction");
      }
      while (cursor_ < input_.size() &&
             std::isdigit(static_cast<unsigned char>(input_[cursor_])) != 0) {
        ++cursor_;
      }
    }
    if (cursor_ < input_.size() && (input_[cursor_] == 'e' || input_[cursor_] == 'E')) {
      ++cursor_;
      if (cursor_ < input_.size() && (input_[cursor_] == '+' || input_[cursor_] == '-')) {
        ++cursor_;
      }
      if (cursor_ >= input_.size() ||
          std::isdigit(static_cast<unsigned char>(input_[cursor_])) == 0) {
        Fail("invalid number exponent");
      }
      while (cursor_ < input_.size() &&
             std::isdigit(static_cast<unsigned char>(input_[cursor_])) != 0) {
        ++cursor_;
      }
    }
    const std::string text(input_.substr(begin, cursor_ - begin));
    std::istringstream stream(text);
    stream.imbue(std::locale::classic());
    double value = 0.0;
    stream >> value;
    if (!stream || stream.peek() != std::char_traits<char>::eof() ||
        !std::isfinite(value)) {
      Fail("invalid finite number");
    }
    return value;
  }

  void ParseLiteral(std::string_view literal) {
    if (input_.substr(cursor_, literal.size()) != literal) {
      Fail("invalid literal");
    }
    cursor_ += literal.size();
  }
};

const JsonValue* JsonValueAtPath(
    const JsonValue& root,
    std::initializer_list<std::string_view> path) {
  if (path.size() == 0U) {
    return nullptr;
  }
  const JsonValue* current = &root;
  for (const auto key : path) {
    const auto* object = std::get_if<JsonValue::Object>(&current->value);
    if (object == nullptr) {
      return nullptr;
    }
    const auto found = object->find(std::string(key));
    if (found == object->end()) {
      return nullptr;
    }
    current = &found->second;
  }
  return current;
}

}  // namespace

std::string JsonEscape(std::string_view value) {
  std::string escaped;
  escaped.reserve(value.size() + 8U);
  for (const unsigned char ch : value) {
    switch (ch) {
      case '"': escaped += "\\\""; break;
      case '\\': escaped += "\\\\"; break;
      case '\b': escaped += "\\b"; break;
      case '\f': escaped += "\\f"; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default:
        if (ch < 0x20U) {
          constexpr char kHex[] = "0123456789abcdef";
          escaped += "\\u00";
          escaped += kHex[(ch >> 4U) & 0x0FU];
          escaped += kHex[ch & 0x0FU];
        } else {
          escaped += static_cast<char>(ch);
        }
    }
  }
  return escaped;
}

std::string JsonString(std::string_view value) {
  return "\"" + JsonEscape(value) + "\"";
}

JsonValue ParseJson(std::string_view input) { return JsonParser(input).Parse(); }

bool IsValidJsonObject(std::string_view input) noexcept {
  try {
    static_cast<void>(JsonParser(input).Parse().AsObject("JSON value"));
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool JsonObjectHasPath(
    std::string_view input,
    std::initializer_list<std::string_view> path) noexcept {
  try {
    const JsonValue root = JsonParser(input).Parse();
    return std::holds_alternative<JsonValue::Object>(root.value) &&
        JsonValueAtPath(root, path) != nullptr;
  } catch (const std::exception&) {
    return false;
  }
}

std::optional<bool> JsonObjectBoolAtPath(
    std::string_view input,
    std::initializer_list<std::string_view> path) noexcept {
  try {
    const JsonValue root = JsonParser(input).Parse();
    if (!std::holds_alternative<JsonValue::Object>(root.value)) {
      return std::nullopt;
    }
    const JsonValue* value = JsonValueAtPath(root, path);
    if (value == nullptr) {
      return std::nullopt;
    }
    const auto* boolean = std::get_if<bool>(&value->value);
    return boolean == nullptr ? std::nullopt : std::optional<bool>(*boolean);
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

std::optional<double> JsonObjectNumberAtPath(
    std::string_view input,
    std::initializer_list<std::string_view> path) noexcept {
  try {
    const JsonValue root = JsonParser(input).Parse();
    if (!std::holds_alternative<JsonValue::Object>(root.value)) {
      return std::nullopt;
    }
    const JsonValue* value = JsonValueAtPath(root, path);
    if (value == nullptr) {
      return std::nullopt;
    }
    const auto* number = std::get_if<double>(&value->value);
    return number == nullptr ? std::nullopt : std::optional<double>(*number);
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

std::optional<std::string> JsonObjectStringAtPath(
    std::string_view input,
    std::initializer_list<std::string_view> path) noexcept {
  try {
    const JsonValue root = JsonParser(input).Parse();
    if (!std::holds_alternative<JsonValue::Object>(root.value)) {
      return std::nullopt;
    }
    const JsonValue* value = JsonValueAtPath(root, path);
    if (value == nullptr) {
      return std::nullopt;
    }
    const auto* text = std::get_if<std::string>(&value->value);
    return text == nullptr ? std::nullopt : std::optional<std::string>(*text);
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

std::optional<std::vector<std::string>> JsonObjectPathList(
    std::string_view input,
    std::string_view key) noexcept {
  try {
    const auto root = JsonParser(input).Parse().AsObject("JSON value");
    const auto found = root.find(std::string(key));
    if (found == root.end()) return std::nullopt;
    std::vector<std::string> paths;
    for (const auto& item : found->second.AsArray(key)) {
      const auto& object = item.AsObject(key);
      const auto path = object.find("path");
      if (path == object.end()) return std::nullopt;
      paths.push_back(path->second.AsString("path"));
    }
    return paths;
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

}  // namespace lingtu::maps
