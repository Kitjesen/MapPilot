#include "lingtu/maps/sources/unity_scene.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

std::string JsonEscape(const std::string& value) {
  std::string out;
  out.reserve(value.size() + 8U);
  for (const char ch : value) {
    switch (ch) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20U) {
          out += "?";
        } else {
          out.push_back(ch);
        }
    }
  }
  return out;
}

float ParseFloat(const std::string& text, const std::string& option) {
  std::size_t consumed = 0U;
  const float value = std::stof(text, &consumed);
  if (consumed != text.size()) {
    throw std::invalid_argument(option + " requires a numeric value");
  }
  return value;
}

std::uint64_t ParseUnsigned(const std::string& text, const std::string& option) {
  std::size_t consumed = 0U;
  const auto value = std::stoull(text, &consumed);
  if (consumed != text.size()) {
    throw std::invalid_argument(option + " requires a non-negative integer");
  }
  return value;
}

void PrintUsage() {
  std::cout
      << "Usage: lingtu-maps-import-unity --scene DIR --output FILE --taxonomy FILE "
         "[--voxel M] [--generation N] [--frame ID] [--include-unknown] "
         "[--include-dynamic]\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::filesystem::path scene;
    std::filesystem::path output;
    lingtu::maps::sources::UnitySemanticImportConfig config;
    for (int index = 1; index < argc; ++index) {
      const std::string option = argv[index];
      auto next = [&]() -> std::string {
        if (++index >= argc) {
          throw std::invalid_argument(option + " requires a value");
        }
        return argv[index];
      };
      if (option == "--scene") {
        scene = next();
      } else if (option == "--output") {
        output = next();
      } else if (option == "--taxonomy") {
        config.taxonomy_path = next();
      } else if (option == "--voxel") {
        config.voxel_size_m = ParseFloat(next(), option);
      } else if (option == "--generation") {
        config.generation = ParseUnsigned(next(), option);
      } else if (option == "--frame") {
        config.frame_id = next();
      } else if (option == "--max-objects") {
        config.max_objects = ParseUnsigned(next(), option);
      } else if (option == "--max-voxels") {
        config.max_voxels = ParseUnsigned(next(), option);
      } else if (option == "--max-voxel-checks") {
        config.max_voxel_checks = ParseUnsigned(next(), option);
      } else if (option == "--include-unknown") {
        config.include_unknown_geometry = true;
      } else if (option == "--include-dynamic") {
        config.exclude_dynamic_classes = false;
      } else if (option == "--help" || option == "-h") {
        PrintUsage();
        return EXIT_SUCCESS;
      } else {
        throw std::invalid_argument("unknown option: " + option);
      }
    }
    if (scene.empty() || output.empty() || config.taxonomy_path.empty()) {
      PrintUsage();
      return 2;
    }
    const auto stats = lingtu::maps::sources::ImportUnitySemanticMap(
        scene, output, config);
    std::cout << "{\"success\":true,\"output\":\"" << JsonEscape(output.string())
              << "\",\"objects\":" << stats.object_rows
              << ",\"accepted_objects\":" << stats.accepted_objects
              << ",\"skipped_unmapped_objects\":" << stats.skipped_unmapped_objects
              << ",\"skipped_dynamic_objects\":" << stats.skipped_dynamic_objects
              << ",\"voxel_checks\":" << stats.candidate_voxel_checks
              << ",\"semantic_conflicts\":" << stats.semantic_conflicts
              << ",\"output_voxels\":" << stats.output_voxels
              << ",\"unmapped_labels\":[";
    for (std::size_t index = 0U; index < stats.unmapped_labels.size(); ++index) {
      if (index != 0U) std::cout << ',';
      std::cout << '"' << JsonEscape(stats.unmapped_labels[index]) << '"';
    }
    std::cout << "]}\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::cerr << "{\"success\":false,\"error\":\"" << JsonEscape(error.what())
              << "\"}\n";
    return EXIT_FAILURE;
  }
}
