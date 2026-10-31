#include "lingtu/maps/storage/stairs.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace lingtu::maps {
namespace {
const JsonValue& StairField(const JsonValue::Object& object, const std::string& key) {
  const auto found = object.find(key);
  if (found == object.end()) throw std::invalid_argument("stair field is required: " + key);
  return found->second;
}

double StairNumber(const JsonValue::Object& object, const std::string& key) {
  const auto* number = std::get_if<double>(&StairField(object, key).value);
  if (number == nullptr || !std::isfinite(*number)) {
    throw std::invalid_argument("stair coordinate must be finite: " + key);
  }
  return *number;
}

StairPoint StairPointFromJson(const JsonValue& value) {
  const auto& point = value.AsObject("stair point");
  return {StairNumber(point, "x"), StairNumber(point, "y"), StairNumber(point, "z")};
}


PointXyz GeometryPoint(const JsonValue& value) {
  const auto p = StairPointFromJson(value);
  return {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)};
}
StairGeometry GeometryFromJson(const JsonValue& value) {
  const auto& obj = value.AsObject("stair geometry");
  StairGeometry g;
  g.recognized = std::get<bool>(StairField(obj, "recognized").value);
  g.reason = StairField(obj, "reason").AsString("geometry reason");
  if (!g.recognized) return g;
  g.origin = GeometryPoint(StairField(obj, "origin"));
  g.lower = GeometryPoint(StairField(obj, "lower"));
  g.upper = GeometryPoint(StairField(obj, "upper"));
  g.axis_x = StairNumber(obj, "axis_x");
  g.axis_y = StairNumber(obj, "axis_y");
  g.width_m = StairNumber(obj, "width_m");
  g.mean_rise_m = StairNumber(obj, "mean_rise_m");
  g.mean_run_m = StairNumber(obj, "mean_run_m");
  for (const auto& row : StairField(obj, "surfaces").AsArray("stair surfaces")) {
    const auto& surface = row.AsObject("stair surface");
    g.surfaces.push_back({StairNumber(surface, "s_min"), StairNumber(surface, "s_max"),
        StairNumber(surface, "lateral_min"), StairNumber(surface, "lateral_max"),
        StairNumber(surface, "surface_z"),
        static_cast<std::size_t>(StairField(surface, "observed_points").AsUnsigned("observed points"))});
  }
  return g;
}
std::string GeometryJson(const StairGeometry& g) {
  std::ostringstream out;
  out << std::setprecision(17) << "{\"recognized\":" << (g.recognized ? "true" : "false")
      << ",\"reason\":" << JsonString(g.reason);
  if (g.recognized) {
    const auto point = [](const PointXyz& p) {
      std::ostringstream text;
      text << std::setprecision(17) << "{\"x\":" << p.x << ",\"y\":" << p.y << ",\"z\":" << p.z << '}';
      return text.str();
    };
    out << ",\"origin\":" << point(g.origin) << ",\"lower\":" << point(g.lower)
        << ",\"upper\":" << point(g.upper) << ",\"axis_x\":" << g.axis_x << ",\"axis_y\":" << g.axis_y
        << ",\"width_m\":" << g.width_m << ",\"mean_rise_m\":" << g.mean_rise_m
        << ",\"mean_run_m\":" << g.mean_run_m << ",\"surfaces\":[";
    for (std::size_t i=0; i<g.surfaces.size(); ++i) {
      if (i) out << ',';
      const auto& t = g.surfaces[i];
      out << "{\"s_min\":" << t.s_min << ",\"s_max\":" << t.s_max
          << ",\"lateral_min\":" << t.lateral_min << ",\"lateral_max\":" << t.lateral_max
          << ",\"surface_z\":" << t.surface_z << ",\"observed_points\":" << t.observed_points << '}';
    }
    out << ']';
  }
  out << '}';
  return out.str();
}

}  // namespace

StairAnnotation StairFromJson(const std::string& id, const JsonValue& value, bool read_geometry) {
  const auto& object = value.AsObject("stair annotation");
  const auto& bounds = StairField(object, "bounds").AsObject("stair bounds");
  StairAnnotation stair;
  stair.id = id;
  stair.name = StairField(object, "name").AsString("stair name");
  stair.frame_id = StairField(object, "frame_id").AsString("stair frame_id");
  const double epoch = StairNumber(object, "content_epoch");
  if (epoch <= 0.0 || std::floor(epoch) != epoch || epoch >= 9223372036854775808.0) {
    throw std::invalid_argument("stair content_epoch must be a positive integer");
  }
  stair.content_epoch = static_cast<std::int64_t>(epoch);
  if (stair.name.empty() || stair.frame_id.empty() || stair.frame_id.find_first_of("\t\r\n") != std::string::npos) {
    throw std::invalid_argument("stair name and frame_id are required");
  }
  stair.minimum = StairPointFromJson(StairField(bounds, "min"));
  stair.maximum = StairPointFromJson(StairField(bounds, "max"));
  stair.lower = StairPointFromJson(StairField(object, "lower"));
  stair.upper = StairPointFromJson(StairField(object, "upper"));
  if (!(stair.minimum.x < stair.maximum.x && stair.minimum.y < stair.maximum.y &&
        stair.minimum.z < stair.maximum.z)) {
    throw std::invalid_argument("stair bounds min must be below max on every axis");
  }
  if (!(stair.lower.z < stair.upper.z)) {
    throw std::invalid_argument("stair upper entry must be higher than lower entry");
  }
  const auto geometry = object.find("geometry");
  if (read_geometry && geometry != object.end()) stair.geometry = GeometryFromJson(geometry->second);
  return stair;
}

std::string StairPointJson(const StairPoint& point) {
  std::ostringstream out;
  out << std::setprecision(17) << "{\"x\":" << point.x << ",\"y\":" << point.y
      << ",\"z\":" << point.z << "}";
  return out.str();
}

std::string StairJson(const StairAnnotation& stair) {
  return "{\"id\":" + JsonString(stair.id) + ",\"name\":" + JsonString(stair.name) +
      ",\"frame_id\":" + JsonString(stair.frame_id) +
      ",\"content_epoch\":" + std::to_string(stair.content_epoch) +
      ",\"bounds\":{\"min\":" + StairPointJson(stair.minimum) +
      ",\"max\":" + StairPointJson(stair.maximum) + "},\"lower\":" +
      StairPointJson(stair.lower) + ",\"upper\":" + StairPointJson(stair.upper) +
      ",\"geometry\":" + GeometryJson(stair.geometry) + "}";
}


std::vector<StairAnnotation> ReadStairAnnotations(const std::filesystem::path& path) {
  std::ifstream file(path);
  if (!file && std::filesystem::exists(path)) throw std::runtime_error("failed to read stair store");
  std::vector<StairAnnotation> stairs;
  std::string line;
  while (std::getline(file, line)) {
    std::vector<std::string> fields;
    std::istringstream row(line);
    std::string field;
    while (std::getline(row, field, '\t')) fields.push_back(field);
    if (fields.size() != 8U) throw std::runtime_error("POI store is corrupt");
    if (JsonObjectStringAtPath(fields[7], {"schema_version"}) != "lingtu.stair.v1") continue;
    const auto tags = ParseJson(fields[7]);
    const auto& value = StairField(tags.AsObject("stair record"), "stair");
    const auto id = StairField(value.AsObject("stair annotation"), "id").AsString("stair id");
    auto stair = StairFromJson(id, value);
    if (fields[0] != "stair:" + id || fields[1] != stair.frame_id)
      throw std::runtime_error("stair storage identity is inconsistent");
    stairs.push_back(std::move(stair));
  }
  if (file.bad()) throw std::runtime_error("failed to read stair store");
  return stairs;
}

}  // namespace lingtu::maps
