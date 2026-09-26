#pragma once

#include <string>
#include <vector>

#include "lingtu/maps/build/pcd.hpp"

namespace lingtu::maps {

// Whether a point cloud without saved scan rays (an imported PCD) can serve as
// a navigation map: its floor is level with z pointing up, its size is
// plausible in metres, and its floor is dense enough to support the robot at
// the map resolution.
struct PointCloudNavigationCheck {
  double floor_tilt_deg{0.0};
  double floor_share{0.0};  // share of points on the floor plane
  double floor_fill{0.0};   // share of floor cells with at least 6 of 8 neighbours
  double extent_xy_m{0.0};
  double extent_z_m{0.0};
  std::vector<std::string> blockers;
  bool ok() const { return blockers.empty(); }
};

PointCloudNavigationCheck CheckPointCloudForNavigation(const std::vector<PointXyz>& points,
                                                       double resolution);

std::string PointCloudNavigationCheckJson(const PointCloudNavigationCheck& check);

}  // namespace lingtu::maps
