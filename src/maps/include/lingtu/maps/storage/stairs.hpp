#pragma once

#include "lingtu/maps/io/json.hpp"
#include "lingtu/maps/semantic/stairs.hpp"
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace lingtu::maps {

struct StairPoint {
  double x;
  double y;
  double z;
};

struct StairAnnotation {
  std::string id;
  std::string name;
  std::string frame_id;
  std::int64_t content_epoch;
  StairPoint minimum;
  StairPoint maximum;
  StairPoint lower;
  StairPoint upper;
  StairGeometry geometry;
};

StairAnnotation StairFromJson(const std::string& id, const JsonValue& value, bool read_geometry = true);
std::string StairJson(const StairAnnotation& stair);
// Read the same atomic record mapd writes, without linking its lifecycle service.
std::vector<StairAnnotation> ReadStairAnnotations(const std::filesystem::path& poi_path);

}  // namespace lingtu::maps
