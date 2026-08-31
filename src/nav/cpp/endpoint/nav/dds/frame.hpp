#pragma once

#include <optional>

#include "input/frame.hpp"
#include "messages.h"

namespace lingtu::nav::endpoint {

// Generated-message conversion belongs to the DDS adapter. Navigation frame
// math itself lives in input/frame.* and uses only owning C++ values.
const char *odometryFramePairError(const lingtu_dds_Odometry &message) noexcept;
double ddsStampSeconds(const lingtu_dds_Time &stamp);
Quaternion quaternionFromDds(const lingtu_dds_Quaternion &q);
RigidTransform rigidTransformFromOdometry(const lingtu_dds_Odometry &message);
std::optional<RigidTransform> mapOdomTransformFromTf(const lingtu_dds_TFMessage &message);

}  // namespace lingtu::nav::endpoint
