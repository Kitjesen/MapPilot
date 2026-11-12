#include "lingtu/maps/no_go.hpp"
#include "lingtu/maps/io/json.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <set>
#include <sstream>
#include <stdexcept>

namespace lingtu::maps {

void ValidateNoGoRegion(const NoGoRegion& region) {
  if (region.label.size() > 512 || (region.kind != "no_go" && region.kind != "glass"))
    throw std::invalid_argument("invalid region label or kind");
  if (region.id.empty() || region.id.size() > 128 ||
      !std::all_of(region.id.begin(), region.id.end(), [](unsigned char ch) {
        return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
               (ch >= '0' && ch <= '9') || ch == '_' || ch == '-';
      })) throw std::invalid_argument("region id must use letters, digits, '_' or '-'");
  for (int axis = 0; axis < 3; ++axis)
    if (!std::isfinite(region.min[axis]) || !std::isfinite(region.max[axis]) ||
        region.min[axis] >= region.max[axis])
      throw std::invalid_argument("no-go bounds must be finite with min below max on every axis");
}

std::vector<NoGoRegion> ReadNoGoRegions(const std::filesystem::path& map_dir) {
  const auto path = map_dir / kNoGoFilename;
  if (!std::filesystem::exists(path)) return {};
  std::ifstream file(path, std::ios::binary);
  if (!file) throw std::runtime_error("cannot read no_go.json");
  const std::string text{std::istreambuf_iterator<char>(file), {}};
  const auto document = ParseJson(text);
  const auto& object = document.AsObject("no-go document");
  if (object.at("schema").AsString("schema") != "lingtu.no_go.v1" ||
      object.at("frame_id").AsString("frame_id") != "map")
    throw std::invalid_argument("no-go document requires lingtu.no_go.v1 in frame map");
  std::vector<NoGoRegion> regions;
  std::set<std::string> ids;
  for (const auto& item : object.at("regions").AsArray("regions")) {
    const auto& entry = item.AsObject("region");
    NoGoRegion region;
    region.id = entry.at("id").AsString("region id");
    if (entry.count("label")) region.label = entry.at("label").AsString("label");
    if (entry.count("kind")) region.kind = entry.at("kind").AsString("kind");
    for (const auto& field : {"min", "max"}) {
      const auto& values = entry.at(field).AsArray(field);
      if (values.size() != 3) throw std::invalid_argument("no-go coordinates need three values");
      auto& coordinates = std::string_view(field) == "min" ? region.min : region.max;
      for (int axis = 0; axis < 3; ++axis) {
        const auto* number = std::get_if<double>(&values[axis].value);
        if (!number) throw std::invalid_argument("no-go coordinate must be a number");
        coordinates[axis] = *number;
      }
    }
    ValidateNoGoRegion(region);
    if (!ids.insert(region.id).second) throw std::invalid_argument("duplicate no-go region id");
    regions.push_back(std::move(region));
  }
  return regions;
}

std::string NoGoRegionsJson(const std::vector<NoGoRegion>& regions) {
  std::ostringstream out;
  out << std::setprecision(17) << "{\"schema\":\"lingtu.no_go.v1\",\"frame_id\":\"map\",\"regions\":[";
  for (std::size_t i = 0; i < regions.size(); ++i) {
    const auto& region = regions[i];
    if (i) out << ',';
    out << "{\"id\":" << JsonString(region.id) << ",\"label\":" << JsonString(region.label)
        << ",\"kind\":" << JsonString(region.kind) << ",\"min\":[";
    for (int axis = 0; axis < 3; ++axis) out << (axis ? "," : "") << region.min[axis];
    out << "],\"max\":[";
    for (int axis = 0; axis < 3; ++axis) out << (axis ? "," : "") << region.max[axis];
    out << "]}";
  }
  return out.str() + "]}";
}

}  // namespace lingtu::maps
