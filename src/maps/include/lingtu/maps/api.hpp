#pragma once

#include <optional>
#include <string>
#include <vector>

#include "lingtu/maps/model.hpp"

namespace lingtu::maps {

enum class MapCapability {
  kGlobal2DPlanning,
  kTerrainReasoning,
  kTrajectoryOptimization,
  kCollision3D,
  kVisualization,
};

struct MapBundle {
  std::string map_id;
  MapCapability capability{MapCapability::kGlobal2DPlanning};
  std::vector<MapArtifact> artifacts;
  MapHealth health;
};

struct SetActiveMapRequest {
  std::string map_id;
  bool strict{true};
};

struct BuildArtifactRequest {
  std::string map_id;
  ArtifactType artifact_type{ArtifactType::kPointCloud};
};

class MapApi {
 public:
  virtual ~MapApi() = default;

  virtual std::vector<MapRecord> ListMaps() const = 0;
  virtual std::optional<MapRecord> GetMapRecord(const std::string& map_id) const = 0;
  virtual std::optional<MapRecord> GetActiveMap() const = 0;
  virtual std::optional<MapBundle> GetMapBundle(MapCapability capability) const = 0;

  virtual bool SetActiveMap(const SetActiveMapRequest& request) = 0;
  virtual bool BuildArtifact(const BuildArtifactRequest& request) = 0;
};

}  // namespace lingtu::maps
