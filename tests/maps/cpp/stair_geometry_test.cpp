#include "lingtu/maps/semantic/stairs.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace lingtu::maps;

namespace {
void Require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

void Surface(std::vector<PointXyz>& cloud, double begin, double end, double z,
             double slope = 0, double lateral_min = -0.5, double lateral_max = 0.5) {
  for (int ix = 0; begin+ix*0.025 <= end+1e-6; ++ix) {
    const double x = begin+ix*0.025;
    for (int iy = 0; lateral_min+iy*0.025 <= lateral_max+1e-6; ++iy) {
      cloud.push_back({static_cast<float>(x),static_cast<float>(lateral_min+iy*0.025),
                       static_cast<float>(z+slope*(x-begin))});
    }
  }
}

std::vector<PointXyz> Flight(bool missing_tread = false) {
  std::vector<PointXyz> cloud;
  Surface(cloud,-0.6,-0.025,0);
  for (int step = 0; step < 5; ++step) {
    if (missing_tread && step == 2) continue;
    Surface(cloud,step*0.3,step*0.3+0.275,(step+1)*0.15);
    // Vertical riser observations must not be treated as wide horizontal treads.
    for (int iz = 0; iz <= 6; ++iz)
      for (int iy = -20; iy <= 20; ++iy)
        cloud.push_back({step*0.3F,iy*0.025F,step*0.15F+iz*0.025F});
  }
  Surface(cloud,1.5,2.1,0.9);
  return cloud;
}

const PointXyz kLower{-0.3F,0,0};
const PointXyz kUpper{1.8F,0,0.9F};

void RecognizesObservedTreadsAndLandings() {
  const auto cloud = Flight();
  const auto geometry = RecognizeStairGeometry(cloud,{},kLower,kUpper);
  Require(geometry.recognized,"ordinary observed staircase should be recognized");
  Require(geometry.surfaces.size() == 7,"six risers and seven observed surfaces");
  Require(std::abs(geometry.mean_rise_m-0.15) < 0.01,"measured rise");
  Require(geometry.mean_run_m > 0.20 && geometry.mean_run_m < 0.31,"measured tread run");
  Require(geometry.width_m > 0.85,"measured stair width");
  Require(geometry.surfaces.front().s_max < geometry.surfaces.back().s_min,"ordered flight limits");
  Require(cloud.size() == Flight().size(),"recognition must not rewrite points");
  PcdFilterOptions filter;
  filter.voxel_size = 0.05;
  Require(RecognizeStairGeometry(FilterPcdPoints(cloud,filter),{},kLower,kUpper).recognized,
          "saved map downsampled at 5 cm must preserve observed stairs");
}

void RejectsRampWallAndSingleObstacle() {
  std::vector<PointXyz> ramp;
  Surface(ramp,-0.6,-0.025,0);
  Surface(ramp,0,1.5,0,0.6);
  Surface(ramp,1.525,2.1,0.9);
  Require(!RecognizeStairGeometry(ramp,{},kLower,kUpper).recognized,"continuous ramp is not stairs");
  std::vector<PointXyz> wall;
  for (int z = 0; z < 50; ++z)
    for (int y = -20; y < 20; ++y) wall.push_back({0,y*0.025F,z*0.025F});
  Require(!RecognizeStairGeometry(wall,{},kLower,kUpper).recognized,"vertical wall is not stairs");
  std::vector<PointXyz> ledge;
  Surface(ledge,-0.6,0,0); Surface(ledge,0.025,1,0.15);
  Require(!RecognizeStairGeometry(ledge,{},kLower,{0.7F,0,0.15F}).recognized,"one obstacle is not a flight");
}

void PreservesOverlappingFloors() {
  auto cloud = Flight();
  Surface(cloud,-0.6,2.1,-2.5);
  Surface(cloud,-0.6,2.1,3.0);
  const auto geometry = RecognizeStairGeometry(cloud,{},kLower,kUpper);
  Require(geometry.recognized && geometry.surfaces.size() == 7,"floors at same XY must stay separate");
  for (const auto& patch : geometry.surfaces)
    Require(patch.surface_z > -0.02 && patch.surface_z < 0.92,"no other floor in selected chain");
  Require(!RecognizeStairGeometry(cloud,{},kLower,{1.8F,0,3.0F}).recognized,"no fabricated flight to ceiling");
}

void RequiresObservedConnection() {
  Require(!RecognizeStairGeometry(Flight(true),{},kLower,kUpper).recognized,"missing tread cannot be synthesized");
  auto cloud = Flight();
  cloud.erase(std::remove_if(cloud.begin(),cloud.end(),[](const auto& p){ return p.x >= 1.5F; }),cloud.end());
  Require(!RecognizeStairGeometry(cloud,{},kLower,kUpper).recognized,"missing exit landing is not inferred");
  PcdBounds bounds{true,-0.7,-0.6,-0.02,1.0,0.6,1.0};
  Require(!RecognizeStairGeometry(Flight(),bounds,kLower,kUpper).recognized,"ROI excludes required upper observations");
}

void WorksInRotatedElevatedNoisyMap() {
  auto cloud = Flight();
  const double yaw = 0.7, c = std::cos(yaw), s = std::sin(yaw);
  const auto transform = [=](PointXyz p) {
    return PointXyz{static_cast<float>(12+c*p.x-s*p.y),static_cast<float>(-7+s*p.x+c*p.y),p.z+4.0F};
  };
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    cloud[i].z += static_cast<float>(0.004*std::sin(static_cast<double>(i)));
    cloud[i] = transform(cloud[i]);
  }
  std::reverse(cloud.begin(),cloud.end());
  const auto geometry = RecognizeStairGeometry(cloud,{},transform(kLower),transform(kUpper));
  Require(geometry.recognized,"map yaw, altitude and measured noise should retain flight");
  Require(std::abs(geometry.axis_x-c) < 1e-5 && std::abs(geometry.axis_y-s) < 1e-5,"map direction preserved");
  Require(std::abs(geometry.lower.z-4) < 0.01 && std::abs(geometry.upper.z-4.9) < 0.01,"world height preserved");
  PcdFilterOptions filter;
  filter.voxel_size = 0.05;
  Require(RecognizeStairGeometry(FilterPcdPoints(cloud,filter),{},transform(kLower),transform(kUpper)).recognized,
          "rotated map voxelization must preserve separate tread surfaces");
}

void RejectsDisconnectedSideBySidePatches() {
  std::vector<PointXyz> cloud;
  Surface(cloud,-0.6,0,0,0,-0.5,0.5);
  Surface(cloud,0.025,0.3,0.15,0,1.0,2.0);
  Surface(cloud,0.325,0.9,0.3,0,-0.5,0.5);
  Require(!RecognizeStairGeometry(cloud,{},kLower,{0.6F,0,0.3F}).recognized,"sideways shelf is not a stair connection");
}

void PreservesLowTreadsAndLandingLip() {
  std::vector<PointXyz> cloud;
  Surface(cloud,-0.6,-0.025,0.03);
  for (int step = 0; step < 6; ++step) {
    Surface(cloud,step*0.36,step*0.36+0.35,(step+1)*0.06);
    for (int iz = 0; iz <= 6; ++iz)
      for (int iy = -20; iy <= 20; ++iy)
        cloud.push_back({step*0.36F,iy*0.025F,step*0.06F+iz*0.01F});
  }
  Surface(cloud,2.16,2.75,0.36);
  const auto geometry = RecognizeStairGeometry(cloud,{}, {-0.3F,0,0.03F}, {2.45F,0,0.36F});
  Require(geometry.recognized,"six-centimeter stairs at the configured bound must be recognized");
  Require(geometry.surfaces.size() == 7,"small treads must not flood into a single rising surface");
  Require(std::abs(geometry.surfaces[1].surface_z-geometry.surfaces[0].surface_z-0.03) < 0.008,
          "three-centimeter entrance lip must retain its observed rise");
  for (std::size_t i = 2; i < geometry.surfaces.size(); ++i)
    Require(std::abs(geometry.surfaces[i].surface_z-geometry.surfaces[i-1].surface_z-0.06) < 0.008,
            "each observed six-centimeter riser stays separate");

  std::vector<PointXyz> lips;
  Surface(lips,-0.6,-0.025,0);
  Surface(lips,0,0.35,0.03);
  Surface(lips,0.36,1.0,0.06);
  Require(!RecognizeStairGeometry(lips,{}, {-0.3F,0,0}, {0.7F,0,0.06F}).recognized,
          "landing lips alone do not establish a flight of stairs");
}
}  // namespace

int main() {
  try {
    RecognizesObservedTreadsAndLandings();
    RejectsRampWallAndSingleObstacle();
    PreservesOverlappingFloors();
    RequiresObservedConnection();
    WorksInRotatedElevatedNoisyMap();
    RejectsDisconnectedSideBySidePatches();
    PreservesLowTreadsAndLandingLip();
    std::cout << "stair geometry: 7 scenario groups passed\n";
    return 0;
  } catch (const std::exception& exc) {
    std::cerr << "stair geometry: " << exc.what() << '\n';
    return 1;
  }
}
