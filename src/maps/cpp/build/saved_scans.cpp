#include "lingtu/maps/build/saved_scans.hpp"
#include "lingtu/maps/build/pcd.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace lingtu::maps {

void VisitSavedScans(const std::filesystem::path& directory,
                     const std::function<void(const SavedScan&)>& visit) {
  std::array<double, 3> origin{};
  std::string tag;
  std::ifstream origins(directory / "scan_origin.txt");
  if (!(origins >> tag >> origin[0] >> origin[1] >> origin[2]) ||
      tag != "lidar_origin_in_patch")
    throw std::runtime_error("saved scans require a measured scan_origin.txt");
  for (double v : origin)
    if (!std::isfinite(v)) throw std::runtime_error("saved scan origin is not finite");
  std::ifstream poses(directory / "poses.txt");
  if (!poses) throw std::runtime_error("saved scans require poses.txt");
  std::unordered_set<std::string> names;
  std::string line;
  while (std::getline(poses, line)) {
    if (line.empty()) continue;
    std::istringstream row(line);
    std::string name, extra;
    std::array<double, 7> p{};
    row >> name;
    for (double& value : p) {
      if (!(row >> value) || !std::isfinite(value))
        throw std::runtime_error("invalid saved scan pose");
    }
    if ((row >> extra) || name.empty() || std::filesystem::path(name).filename() != name ||
        !names.insert(name).second)
      throw std::runtime_error("invalid or duplicate saved scan patch name");
    const double norm = std::sqrt(p[3]*p[3]+p[4]*p[4]+p[5]*p[5]+p[6]*p[6]);
    if (norm < 1e-12) throw std::runtime_error("invalid saved scan rotation");
    const double w=p[3]/norm, x=p[4]/norm, y=p[5]/norm, z=p[6]/norm;
    const auto transform = [&](double a, double b, double c) {
      return std::array<double,3>{
        p[0]+(1-2*(y*y+z*z))*a+2*(x*y-z*w)*b+2*(x*z+y*w)*c,
        p[1]+2*(x*y+z*w)*a+(1-2*(x*x+z*z))*b+2*(y*z-x*w)*c,
        p[2]+2*(x*z-y*w)*a+2*(y*z+x*w)*b+(1-2*(x*x+y*y))*c};
    };
    auto points = LoadPcdXyz(directory / "patches" / name);
    if (!points.ok || points.points.empty())
      throw std::runtime_error("unreadable saved scan: " + name);
    SavedScan scan;
    scan.origin = transform(origin[0],origin[1],origin[2]);
    scan.xyz.reserve(points.points.size()*3);
    for (const auto& point : points.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
      const auto xyz = transform(point.x,point.y,point.z);
      scan.xyz.insert(scan.xyz.end(),xyz.begin(),xyz.end());
    }
    visit(scan);
  }
  if (names.empty()) throw std::runtime_error("saved scans contain no poses");
}

}  // namespace lingtu::maps
