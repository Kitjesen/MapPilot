#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <set>
#include <string>

namespace {

struct Args {
  std::string input;
  std::string output;
  std::string state;
  std::string shape = "sphere";
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double radius = 0.2;
};

struct KeyLess {
  bool operator()(const octomap::OcTreeKey & a, const octomap::OcTreeKey & b) const
  {
    if (a[0] != b[0]) return a[0] < b[0];
    if (a[1] != b[1]) return a[1] < b[1];
    return a[2] < b[2];
  }
};

void usage()
{
  std::cerr
    << "usage: octoplanner3d_edit_octomap --map octomap.ot --output edited.ot "
    << "--state occupied|free|preblocked|traversable|clear "
    << "--x X --y Y --z Z [--radius R] [--shape sphere|box]\n";
}

bool readDouble(const char * text, double & out)
{
  char * end = nullptr;
  out = std::strtod(text, &end);
  return end != text && *end == '\0' && std::isfinite(out);
}

bool parse(int argc, char ** argv, Args & args)
{
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    auto need = [&](std::string & out) -> bool {
      if (i + 1 >= argc) return false;
      out = argv[++i];
      return true;
    };
    auto need_double = [&](double & out) -> bool {
      if (i + 1 >= argc) return false;
      return readDouble(argv[++i], out);
    };

    if (key == "--map" || key == "--input") {
      if (!need(args.input)) return false;
    } else if (key == "--output") {
      if (!need(args.output)) return false;
    } else if (key == "--state") {
      if (!need(args.state)) return false;
    } else if (key == "--shape") {
      if (!need(args.shape)) return false;
    } else if (key == "--x") {
      if (!need_double(args.x)) return false;
    } else if (key == "--y") {
      if (!need_double(args.y)) return false;
    } else if (key == "--z") {
      if (!need_double(args.z)) return false;
    } else if (key == "--radius") {
      if (!need_double(args.radius)) return false;
    } else {
      return false;
    }
  }
  return !args.input.empty() && !args.output.empty() && !args.state.empty() &&
    args.radius >= 0.0 && args.radius <= 10.0 &&
    (args.shape == "sphere" || args.shape == "box");
}

bool stateIsOccupied(const std::string & state, bool & occupied)
{
  if (state == "occupied" || state == "preblocked") {
    occupied = true;
    return true;
  }
  if (state == "free" || state == "traversable" || state == "clear") {
    occupied = false;
    return true;
  }
  return false;
}

bool endsWith(const std::string & value, const std::string & suffix)
{
  return value.size() >= suffix.size() &&
    value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::unique_ptr<octomap::OcTree> loadTree(const std::string & path)
{
  if (endsWith(path, ".bt") || endsWith(path, ".BT")) {
    auto tree = std::make_unique<octomap::OcTree>(0.2);
    if (!tree->readBinary(path)) {
      return nullptr;
    }
    return tree;
  }

  std::unique_ptr<octomap::AbstractOcTree> raw(octomap::AbstractOcTree::read(path));
  if (!raw) {
    auto tree = std::make_unique<octomap::OcTree>(0.2);
    if (tree->readBinary(path)) {
      return tree;
    }
    return nullptr;
  }
  auto * tree = dynamic_cast<octomap::OcTree *>(raw.get());
  if (tree == nullptr) {
    return nullptr;
  }
  raw.release();
  return std::unique_ptr<octomap::OcTree>(tree);
}

}  // namespace

int main(int argc, char ** argv)
{
  Args args;
  if (!parse(argc, argv, args)) {
    usage();
    return 2;
  }

  bool occupied = false;
  if (!stateIsOccupied(args.state, occupied)) {
    std::cerr << "unsupported state: " << args.state << "\n";
    usage();
    return 2;
  }

  std::unique_ptr<octomap::OcTree> tree = loadTree(args.input);
  if (tree == nullptr) {
    std::cerr << "input is not an OcTree: " << args.input << "\n";
    return 3;
  }

  const double res = tree->getResolution();
  const double radius = std::max(args.radius, res * 0.5);
  const int cells = static_cast<int>(std::ceil(radius / res));
  std::set<octomap::OcTreeKey, KeyLess> keys;

  for (int ix = -cells; ix <= cells; ++ix) {
    for (int iy = -cells; iy <= cells; ++iy) {
      for (int iz = -cells; iz <= cells; ++iz) {
        const double dx = ix * res;
        const double dy = iy * res;
        const double dz = iz * res;
        if (args.shape == "sphere" && (dx * dx + dy * dy + dz * dz) > radius * radius) {
          continue;
        }
        octomap::OcTreeKey key;
        if (tree->coordToKeyChecked(args.x + dx, args.y + dy, args.z + dz, key)) {
          keys.insert(key);
        }
      }
    }
  }

  const float log_odds = occupied ? tree->getClampingThresMaxLog() : tree->getClampingThresMinLog();
  int edited = 0;
  for (const auto & key : keys) {
    if (tree->setNodeValue(key, log_odds, true) != nullptr) {
      ++edited;
    }
  }
  tree->updateInnerOccupancy();

  const bool wrote = (endsWith(args.output, ".bt") || endsWith(args.output, ".BT")) ?
    tree->writeBinary(args.output) :
    tree->write(args.output);
  if (!wrote) {
    std::cerr << "failed to write output: " << args.output << "\n";
    return 4;
  }

  std::cout
    << "{\"ok\":true,"
    << "\"state\":\"" << args.state << "\","
    << "\"effective_state\":\"" << (occupied ? "occupied" : "free") << "\","
    << "\"shape\":\"" << args.shape << "\","
    << "\"resolution\":" << res << ","
    << "\"edited_voxels\":" << edited << "}\n";
  return 0;
}
