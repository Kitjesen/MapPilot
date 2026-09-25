#pragma once

#include "backend.hpp"
#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <type_traits>

namespace lingtu::localization::sam {
// yaml-cpp also reads the JSON config object in saved loop evidence. Keep field
// and replay configuration identical without making the solver depend on YAML.
inline Config readConfig(const YAML::Node& node) {
  Config config;
  if (!node) return config;
  if (!node.IsMap()) throw std::invalid_argument("sam configuration must be a map");
  auto read = [&](const char* name, auto& value) {
    if (node[name]) value = node[name].as<std::decay_t<decltype(value)>>();
  };
  read("radius_m", config.radius_m);
  read("min_time_s", config.min_time_s);
  read("submap_half_window", config.submap_half_window);
  read("voxel_m", config.voxel_m);
  read("max_fitness", config.max_fitness);
  read("correspondence_m", config.correspondence_m);
  read("inlier_distance_m", config.inlier_distance_m);
  read("min_overlap", config.min_overlap);
  read("translation_sigma_m", config.translation_sigma_m);
  read("rotation_sigma_rad", config.rotation_sigma_rad);
  read("huber_k", config.huber_k);
  if (const auto noise=node["odom_variance"]) {
    if (!noise.IsSequence() || noise.size()!=6)
      throw std::invalid_argument("sam odom_variance requires six values");
    for (std::size_t i=0;i<6;++i) config.odom_variance[i]=noise[i].as<double>();
  }
  return config;
}
}
