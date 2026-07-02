#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "pcd2octomap_converter.h"

namespace {

struct CliConfig {
  std::string input;
  std::string output;
  double resolution = 0.2;
  int free_layers_above = 3;
  int free_dilation_cells = 1;
};

CliConfig parseArgs(int argc, char ** argv)
{
  CliConfig cfg;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + arg);
      }
      return argv[++i];
    };
    if (arg == "--input" || arg == "-i") {
      cfg.input = next();
    } else if (arg == "--output" || arg == "-o") {
      cfg.output = next();
    } else if (arg == "--resolution") {
      cfg.resolution = std::stod(next());
    } else if (arg == "--free-layers-above") {
      cfg.free_layers_above = std::stoi(next());
    } else if (arg == "--free-dilation-cells") {
      cfg.free_dilation_cells = std::stoi(next());
    } else if (arg == "--frame") {
      (void)next();
    } else if (arg == "--help" || arg == "-h") {
      throw std::runtime_error(
        "usage: octoplanner3d_pcd_to_octomap --input map.pcd --output octomap.ot "
        "[--resolution 0.2] [--free-layers-above 3] [--free-dilation-cells 1] "
        "[--frame map]");
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  if (cfg.input.empty()) {
    throw std::runtime_error("missing --input");
  }
  if (cfg.output.empty()) {
    throw std::runtime_error("missing --output");
  }
  if (!(cfg.resolution > 0.0)) {
    throw std::runtime_error("--resolution must be positive");
  }
  if (cfg.free_layers_above < 0) {
    throw std::runtime_error("--free-layers-above must be non-negative");
  }
  if (cfg.free_dilation_cells < 0) {
    throw std::runtime_error("--free-dilation-cells must be non-negative");
  }
  return cfg;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const CliConfig cfg = parseArgs(argc, argv);
    pcd2octomap::Pcd2OctomapConverter converter;
    converter.setInputPcdFile(cfg.input);
    converter.setOutputBtFile(cfg.output);
    converter.setResolution(cfg.resolution);
    converter.setFreeEnvelopeLayers(cfg.free_layers_above);
    converter.setFreeEnvelopeDilationCells(cfg.free_dilation_cells);
    if (!converter.convert()) {
      std::cerr << "failed to convert PCD to OctoMap" << std::endl;
      return 3;
    }
    std::cout << "{\"success\":true,\"input\":\"" << cfg.input
              << "\",\"output\":\"" << cfg.output
              << "\",\"resolution\":" << cfg.resolution
              << ",\"free_layers_above\":" << cfg.free_layers_above
              << ",\"free_dilation_cells\":" << cfg.free_dilation_cells
              << "}" << std::endl;
    return 0;
  } catch (const std::exception & exc) {
    std::cerr << exc.what() << std::endl;
    return 2;
  }
}
