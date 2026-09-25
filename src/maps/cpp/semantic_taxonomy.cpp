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

}  // namespace

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
  const auto root = ParseJson(input).AsObject("semantic taxonomy");

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
