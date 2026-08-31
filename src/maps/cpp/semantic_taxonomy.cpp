#include "lingtu/maps/semantic_taxonomy.hpp"

#include "lingtu/maps/json.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <variant>

namespace lingtu::maps {
namespace {

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

const JsonValue& Required(
    const JsonValue::Object& object,
    const std::string& key,
    std::string_view context) {
  const auto found = object.find(key);
  if (found == object.end()) {
    throw std::invalid_argument(std::string(context) + " is missing key '" + key + "'");
  }
  return found->second;
}

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

std::string SemanticTaxonomy::NormalizeLabel(std::string_view label) {
  std::string normalized;
  normalized.reserve(label.size());
  bool pending_space = false;
  for (const char ch : label) {
    const unsigned char value = static_cast<unsigned char>(ch);
    if (std::isspace(value) != 0 || ch == '_' || ch == '-') {
      pending_space = !normalized.empty();
      continue;
    }
    if (pending_space) {
      normalized.push_back(' ');
      pending_space = false;
    }
    normalized.push_back(static_cast<char>(std::tolower(value)));
  }
  return normalized;
}

SemanticTaxonomy SemanticTaxonomy::LoadJson(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("failed to open semantic taxonomy: " + path.string());
  }
  std::string input((std::istreambuf_iterator<char>(file)), {});
  if (input.size() >= 3U && static_cast<unsigned char>(input[0]) == 0xEFU &&
      static_cast<unsigned char>(input[1]) == 0xBBU &&
      static_cast<unsigned char>(input[2]) == 0xBFU) {
    input.erase(0U, 3U);
  }
  const auto root = JsonParser(input).Parse().AsObject("semantic taxonomy");

  SemanticTaxonomy taxonomy;
  taxonomy.name_ = Required(root, "name", "semantic taxonomy").AsString("taxonomy.name");
  const std::uint64_t version =
      Required(root, "version", "semantic taxonomy").AsUnsigned("taxonomy.version");
  if (taxonomy.name_.empty() || version == 0U ||
      version > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("semantic taxonomy name/version is invalid");
  }
  taxonomy.version_ = static_cast<std::uint32_t>(version);

  const auto& classes = Required(root, "classes", "semantic taxonomy")
                            .AsArray("taxonomy.classes");
  if (classes.empty()) {
    throw std::invalid_argument("semantic taxonomy classes must not be empty");
  }
  std::unordered_map<std::uint16_t, std::string> ids;
  std::unordered_map<std::string, std::uint16_t> aliases;
  taxonomy.classes_.reserve(classes.size());
  for (std::size_t index = 0U; index < classes.size(); ++index) {
    const std::string context = "taxonomy.classes[" + std::to_string(index) + "]";
    const auto& object = classes[index].AsObject(context);
    const std::uint64_t raw_id = Required(object, "id", context).AsUnsigned(context + ".id");
    if (raw_id > std::numeric_limits<std::uint16_t>::max()) {
      throw std::invalid_argument(context + ".id exceeds uint16");
    }
    SemanticClassDefinition definition;
    definition.id = static_cast<std::uint16_t>(raw_id);
    definition.name = Required(object, "name", context).AsString(context + ".name");
    const auto color = object.find("color");
    if (color != object.end()) {
      definition.color = color->second.AsString(context + ".color");
    }
    const auto alias_list = object.find("aliases");
    if (alias_list != object.end()) {
      for (const auto& value : alias_list->second.AsArray(context + ".aliases")) {
        definition.aliases.push_back(value.AsString(context + ".aliases[]"));
      }
    }
    const std::string normalized_name = NormalizeLabel(definition.name);
    if (normalized_name.empty() || !ids.emplace(definition.id, normalized_name).second) {
      throw std::invalid_argument(context + " has an empty name or duplicate id");
    }
    auto add_alias = [&](std::string_view raw) {
      const std::string normalized = NormalizeLabel(raw);
      const auto [found, inserted] = aliases.emplace(normalized, definition.id);
      if (normalized.empty() || (!inserted && found->second != definition.id)) {
        throw std::invalid_argument(context + " has an empty or ambiguous alias");
      }
    };
    add_alias(definition.name);
    for (const auto& alias : definition.aliases) {
      add_alias(alias);
    }
    taxonomy.classes_.push_back(std::move(definition));
  }
  const auto unknown = aliases.find("unknown");
  const auto background = aliases.find("background");
  if (unknown == aliases.end() || background == aliases.end() || unknown->second != 0U ||
      background->second != 0U) {
    throw std::invalid_argument(
        "semantic taxonomy id 0 must be addressable as unknown and background");
  }
  return taxonomy;
}

std::optional<std::uint16_t> SemanticTaxonomy::Resolve(std::string_view label) const {
  const std::string target = NormalizeLabel(label);
  for (const auto& definition : classes_) {
    if (NormalizeLabel(definition.name) == target) {
      return definition.id;
    }
    for (const auto& alias : definition.aliases) {
      if (NormalizeLabel(alias) == target) {
        return definition.id;
      }
    }
  }
  return std::nullopt;
}

const SemanticClassDefinition* SemanticTaxonomy::Find(std::uint16_t id) const noexcept {
  const auto found = std::find_if(
      classes_.begin(), classes_.end(), [id](const auto& item) { return item.id == id; });
  return found == classes_.end() ? nullptr : &*found;
}

}  // namespace lingtu::maps
