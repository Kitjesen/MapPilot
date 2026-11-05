#pragma once

#include "core/types.hpp"

namespace lingtu::map_cleaning {

PointXYZI transformPoint(const PointXYZI &pt, const Pose &pose);

VoxelKey voxelKey(const PointXYZI &pt, float voxel_size_m);

}  // namespace lingtu::map_cleaning
