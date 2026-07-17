#pragma once

#include <string>

#include "lingtu/maps/model.hpp"

namespace lingtu::maps::build {

class ArtifactBuilder {
 public:
  virtual ~ArtifactBuilder() = default;
  virtual bool Supports(ArtifactType type) const = 0;
  virtual MapArtifact BuildArtifact(const std::string& map_id, ArtifactType type) = 0;
};

}  // namespace lingtu::maps::build
