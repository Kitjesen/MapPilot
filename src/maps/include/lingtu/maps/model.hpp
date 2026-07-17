#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace lingtu::maps {

enum class MapState {
  kDraft,
  kStale,
  kValidated,
  kActive,
  kRetired,
  kFailed,
};

enum class ArtifactType {
  kPointCloud,
  kOccupancy2D,
  kOctomap3D,
  kEsdf,
  kTraversability,
  kSemantic,
};

struct SpatialScope {
  std::string frame_id{"map"};
  double min_x_m{0.0};
  double min_y_m{0.0};
  double min_z_m{0.0};
  double max_x_m{0.0};
  double max_y_m{0.0};
  double max_z_m{0.0};
};

struct MapArtifact {
  ArtifactType type{ArtifactType::kPointCloud};
  std::string uri;
  std::string sha256;
  std::string source_map_id;
  std::string generator;
  std::unordered_map<std::string, std::string> build_config;
};

struct MapHealth {
  double localization_stability{0.0};
  double planning_success_rate{0.0};
  double collision_rate{0.0};
  double freshness{0.0};
  double overall_score{0.0};
};

struct MapRecord {
  std::string map_id;
  std::string lineage_id;
  std::int64_t version{0};
  MapState state{MapState::kDraft};
  SpatialScope scope;
  std::vector<MapArtifact> artifacts;
  MapHealth health;
  std::unordered_map<std::string, std::string> metadata;
};

struct Poi {
  std::string name;
  std::string frame_id{"map"};
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  std::optional<double> yaw_rad;
  std::unordered_map<std::string, std::string> tags;
};

}  // namespace lingtu::maps
