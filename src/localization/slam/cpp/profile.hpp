#pragma once

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <stdexcept>

namespace lingtu::slam {
// One common algorithm file plus the selected installation's explicit overrides.
inline YAML::Node readProfile(const std::filesystem::path& path) {
  const auto profile = YAML::LoadFile(path.string());
  if (!profile["defaults"]) return profile;
  const auto common_path = path.parent_path() / profile["defaults"].as<std::string>();
  const auto common = YAML::LoadFile(common_path.string())["slam"];
  if (!common.IsMap()) throw std::runtime_error("SLAM defaults must contain a slam mapping");
  auto config = YAML::Clone(common);
  for (const auto& entry : profile) {
    const auto key = entry.first.as<std::string>();
    if (key == "defaults") continue;
    auto section = config[key];
    if (section.IsMap() && entry.second.IsMap()) {
      for (const auto& field : entry.second)
        section[field.first.as<std::string>()] = field.second;
    } else {
      config[key] = entry.second;
    }
  }
  return config;
}
}
