#pragma once

#include <string>

namespace lingtu::maps::build {

struct OptimizationResult {
  bool success{false};
  std::string message;
  std::string output_uri;
};

class MapOptimizer {
 public:
  virtual ~MapOptimizer() = default;
  virtual OptimizationResult Optimize(const std::string& map_uri) = 0;
};

}  // namespace lingtu::maps::build
