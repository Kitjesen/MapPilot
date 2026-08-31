#include "dds/frame.hpp"

#include <cmath>
#include <cstdint>
#include <string_view>

namespace lingtu::nav::endpoint {

const char *odometryFramePairError(const lingtu_dds_Odometry &message) noexcept {
  const std::string_view parent =
      message.header.frame_id == nullptr ? std::string_view{} : message.header.frame_id;
  if (parent.empty()) {
    return "odom_frame_empty";
  }
  if (parent != "map" && parent != "odom") {
    return "odom_frame_unsupported";
  }

  const std::string_view child =
      message.child_frame_id == nullptr ? std::string_view{} : message.child_frame_id;
  if (child.empty()) {
    return "odom_child_frame_empty";
  }
  if (child != "body") {
    return "odom_child_frame_unsupported";
  }
  return nullptr;
}

double ddsStampSeconds(const lingtu_dds_Time &stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

Quaternion quaternionFromDds(const lingtu_dds_Quaternion &q) {
  return {q.x, q.y, q.z, q.w};
}

RigidTransform rigidTransformFromOdometry(const lingtu_dds_Odometry &message) {
  RigidTransform transform;
  transform.translation = {
      message.pose.pose.position.x,
      message.pose.pose.position.y,
      message.pose.pose.position.z,
  };
  const auto raw_rotation = quaternionFromDds(message.pose.pose.orientation);
  const bool rotation_valid = quaternionIsFiniteAndNonzero(raw_rotation);
  transform.rotation = normalizeQuaternion(raw_rotation);
  transform.yaw = quaternionYaw(transform.rotation);
  transform.stamp_s = ddsStampSeconds(message.header.stamp);
  transform.valid =
      std::isfinite(transform.translation.x) && std::isfinite(transform.translation.y) &&
      std::isfinite(transform.translation.z) && rotation_valid && std::isfinite(transform.yaw);
  return transform;
}

std::optional<RigidTransform> mapOdomTransformFromTf(const lingtu_dds_TFMessage &message) {
  if (message.transforms._length > 0 && message.transforms._buffer == nullptr) {
    return std::nullopt;
  }
  for (std::uint32_t index = 0; index < message.transforms._length; ++index) {
    const auto &stamped = message.transforms._buffer[index];
    const std::string_view parent =
        stamped.header.frame_id == nullptr ? std::string_view{} : stamped.header.frame_id;
    const std::string_view child =
        stamped.child_frame_id == nullptr ? std::string_view{} : stamped.child_frame_id;
    if (parent != "map" || child != "odom") {
      continue;
    }
    RigidTransform transform;
    transform.translation = {
        stamped.transform.translation.x,
        stamped.transform.translation.y,
        stamped.transform.translation.z,
    };
    const auto rotation = quaternionFromDds(stamped.transform.rotation);
    if (!std::isfinite(transform.translation.x) || !std::isfinite(transform.translation.y) ||
        !std::isfinite(transform.translation.z) || !quaternionIsFiniteAndNonzero(rotation)) {
      return std::nullopt;
    }
    transform.rotation = normalizeQuaternion(rotation);
    transform.yaw = quaternionYaw(transform.rotation);
    transform.stamp_s = ddsStampSeconds(stamped.header.stamp);
    transform.valid = true;
    return transform;
  }
  return std::nullopt;
}

}  // namespace lingtu::nav::endpoint
