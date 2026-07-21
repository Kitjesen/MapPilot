#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace lingtu::maps {

struct SemanticClassDefinition {
  std::uint16_t id{0U};
  std::string name;
  std::string color;
  std::vector<std::string> aliases;
};

class SemanticTaxonomy final {
 public:
  static SemanticTaxonomy LoadJson(const std::filesystem::path& path);

  const std::string& name() const noexcept { return name_; }
  std::uint32_t version() const noexcept { return version_; }
  const std::vector<SemanticClassDefinition>& classes() const noexcept { return classes_; }

  std::optional<std::uint16_t> Resolve(std::string_view label) const;
  const SemanticClassDefinition* Find(std::uint16_t id) const noexcept;

  static std::string NormalizeLabel(std::string_view label);

 private:
  std::string name_;
  std::uint32_t version_{0U};
  std::vector<SemanticClassDefinition> classes_;
};

}  // namespace lingtu::maps
