#include "lingtu/maps/build/octomap_io.hpp"
#include "lingtu/maps/build/pipeline.hpp"
#include "lingtu/maps/build/ray_octomap.hpp"
#include "lingtu/maps/json.hpp"

#include <octomap/OcTree.h>

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace {
using lingtu::maps::JsonValue;

struct Box {
  std::string name;
  std::string role;
  std::array<double, 3> minimum{};
  std::array<double, 3> maximum{};
};

struct Label {
  std::string name;
  std::string kind;
  std::string expected;
  std::array<double, 3> point{};
};

struct BoxCounts {
  std::size_t free{0};
  std::size_t occupied{0};
  std::size_t unknown{0};
};

const JsonValue& Required(const JsonValue::Object& object, const std::string& key) {
  const auto it = object.find(key);
  if (it == object.end()) throw std::invalid_argument("missing JSON field: " + key);
  return it->second;
}

double Number(const JsonValue& value, const std::string& context) {
  const auto* number = std::get_if<double>(&value.value);
  if (number == nullptr) throw std::invalid_argument(context + " must be a number");
  return *number;
}

std::array<double, 3> Point(const JsonValue& value, const std::string& context) {
  const auto& array = value.AsArray(context);
  if (array.size() != 3U) throw std::invalid_argument(context + " must contain three numbers");
  return {Number(array[0], context), Number(array[1], context), Number(array[2], context)};
}

std::string ReadText(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot read: " + path.string());
  std::ostringstream output;
  output << input.rdbuf();
  return output.str();
}

std::vector<Box> ParseBoxes(const JsonValue::Object& root) {
  std::vector<Box> result;
  for (const auto& value : Required(root, "rois").AsArray("rois")) {
    const auto& object = value.AsObject("roi");
    Box box;
    box.name = Required(object, "name").AsString("roi.name");
    box.role = Required(object, "role").AsString("roi.role");
    box.minimum = Point(Required(object, "min"), "roi.min");
    box.maximum = Point(Required(object, "max"), "roi.max");
    for (std::size_t axis = 0; axis < 3U; ++axis)
      if (box.maximum[axis] < box.minimum[axis])
        throw std::invalid_argument("roi max must be greater than or equal to min");
    result.push_back(std::move(box));
  }
  return result;
}

std::vector<Label> ParseLabels(const JsonValue::Object& root) {
  std::vector<Label> result;
  const auto it = root.find("labels");
  if (it == root.end()) return result;
  for (const auto& value : it->second.AsArray("labels")) {
    const auto& object = value.AsObject("label");
    result.push_back({Required(object, "name").AsString("label.name"),
                      Required(object, "kind").AsString("label.kind"),
                      Required(object, "expected").AsString("label.expected"),
                      Point(Required(object, "point"), "label.point")});
  }
  return result;
}

std::string Classify(const octomap::OcTree& tree, const std::array<double, 3>& point) {
  const auto* node = tree.search(point[0], point[1], point[2]);
  if (node == nullptr) return "unknown";
  return tree.isNodeOccupied(node) ? "occupied" : "free";
}

BoxCounts CountBox(const octomap::OcTree& tree, const Box& box) {
  BoxCounts counts;
  const double resolution = tree.getResolution();
  const auto first_center = [resolution](double minimum) {
    double center = std::floor(minimum / resolution) * resolution + resolution / 2.0;
    if (center + 1e-9 < minimum) center += resolution;
    return center;
  };
  for (double x = first_center(box.minimum[0]); x <= box.maximum[0] + 1e-9; x += resolution)
    for (double y = first_center(box.minimum[1]); y <= box.maximum[1] + 1e-9; y += resolution)
      for (double z = first_center(box.minimum[2]); z <= box.maximum[2] + 1e-9; z += resolution) {
        const auto state = Classify(tree, {x, y, z});
        ++(state == "free" ? counts.free : state == "occupied" ? counts.occupied : counts.unknown);
      }
  return counts;
}

void EmitInspection(const octomap::OcTree& tree, const JsonValue::Object& config) {
  const auto boxes = ParseBoxes(config);
  const auto labels = ParseLabels(config);
  const double resolution = tree.getResolution();
  std::cout << "{\"resolution_m\":" << resolution << ",\"rois\":[";
  for (std::size_t i = 0; i < boxes.size(); ++i) {
    if (i != 0U) std::cout << ',';
    const auto counts = CountBox(tree, boxes[i]);
    const auto samples = counts.free + counts.occupied + counts.unknown;
    std::cout << "{\"name\":" << lingtu::maps::JsonString(boxes[i].name)
              << ",\"role\":" << lingtu::maps::JsonString(boxes[i].role)
              << ",\"sample_count\":" << samples << ",\"free\":" << counts.free
              << ",\"occupied\":" << counts.occupied << ",\"unknown\":" << counts.unknown << '}';
  }
  std::cout << "],\"labels\":[";
  for (std::size_t i = 0; i < labels.size(); ++i) {
    if (i != 0U) std::cout << ',';
    const auto actual = Classify(tree, labels[i].point);
    const bool matches = labels[i].expected == "not_occupied"
                             ? actual != "occupied"
                             : actual == labels[i].expected;
    std::cout << "{\"name\":" << lingtu::maps::JsonString(labels[i].name)
              << ",\"kind\":" << lingtu::maps::JsonString(labels[i].kind)
              << ",\"expected\":" << lingtu::maps::JsonString(labels[i].expected)
              << ",\"actual\":" << lingtu::maps::JsonString(actual)
              << ",\"matches\":" << (matches ? "true" : "false") << '}';
  }
  std::cout << "]}" << std::endl;
}

int Replay(const std::filesystem::path& directory, const std::filesystem::path& output,
           double resolution) {
  octomap::OcTree tree(resolution);
  tree.setProbHit(lingtu::maps::kSavedMapSensorModel.prob_hit);
  tree.setProbMiss(lingtu::maps::kSavedMapSensorModel.prob_miss);
  tree.setOccupancyThres(lingtu::maps::kSavedMapSensorModel.occupancy_threshold);
  tree.setClampingThresMin(lingtu::maps::kSavedMapSensorModel.clamping_min);
  tree.setClampingThresMax(lingtu::maps::kSavedMapSensorModel.clamping_max);
  const auto start = std::chrono::steady_clock::now();
  const auto stats = lingtu::maps::PopulateSavedRayOctomap(tree, directory);
  if (!lingtu::maps::SaveOctomapTree(tree, output))
    throw std::runtime_error("failed to write: " + output.string());
  const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  std::cout << "{\"resolution_m\":" << resolution
            << ",\"elapsed_ms\":" << elapsed
            << ",\"valid_endpoints\":" << stats.valid_endpoints
            << ",\"retained_endpoints\":" << stats.retained_endpoints
            << ",\"dropped_endpoints\":" << stats.dropped_endpoints
            << ",\"free_updates\":" << stats.free_updates
            << ",\"hit_updates\":" << stats.hit_updates
            << ",\"guarded_miss_suppressions\":" << stats.guarded_miss_suppressions
            << ",\"inserted_points\":" << stats.inserted_points() << "}" << std::endl;
  return 0;
}

int SelfTest() {
  octomap::OcTree tree(0.1);
  tree.updateNode(octomap::point3d(0.05F, 0.05F, 0.05F), true);
  tree.updateNode(octomap::point3d(0.15F, 0.05F, 0.05F), false);
  tree.updateInnerOccupancy();
  const auto config = lingtu::maps::ParseJson(
      R"({"rois":[{"name":"body","role":"body_clearance","min":[0,0,0],"max":[0.3,0.1,0.1]}],"labels":[{"name":"wall","kind":"wall","expected":"occupied","point":[0.05,0.05,0.05]}]})")
      .AsObject("config");
  EmitInspection(tree, config);
  const auto counts = CountBox(tree, ParseBoxes(config).front());
  return counts.occupied == 1U && counts.free == 1U && counts.unknown == 1U ? 0 : 1;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "self-test") return SelfTest();
    if (argc >= 4 && std::string(argv[1]) == "replay")
      return Replay(argv[2], argv[3], argc == 5 ? std::stod(argv[4]) : 0.05);
    if (argc == 4 && std::string(argv[1]) == "inspect") {
      const auto tree = lingtu::maps::LoadOctomapTree(argv[2]);
      if (!tree) throw std::runtime_error("cannot load OctoMap: " + std::string(argv[2]));
      const auto config = lingtu::maps::ParseJson(ReadText(argv[3])).AsObject("config");
      EmitInspection(*tree, config);
      return 0;
    }
    std::cerr << "usage: evidence_tool replay MAP_DIR OUTPUT.ot [RESOLUTION] | "
                 "inspect MAP.ot CONFIG.json | self-test\n";
    return 2;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
