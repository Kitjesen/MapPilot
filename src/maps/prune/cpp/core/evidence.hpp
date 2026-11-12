#pragma once

#include "core/types.hpp"

namespace lingtu::map_cleaning {

PointXYZI transformPoint(const PointXYZI &pt, const Pose &pose);

}  // namespace lingtu::map_cleaning
