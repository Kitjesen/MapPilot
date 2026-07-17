#include "sdk.hpp"

namespace lingtu::drivers::camera {
namespace {

std::string make_topic(const std::string &camera_namespace, const std::string &name) {
  std::string ns = camera_namespace.empty() ? "camera" : camera_namespace;
  if (ns.front() != '/') {
    ns.insert(ns.begin(), '/');
  }
  while (ns.size() > 1 && ns.back() == '/') {
    ns.pop_back();
  }
  return ns + "/" + name;
}

}  // namespace

Topics default_topics(const std::string &camera_namespace) {
  Topics topics;
  topics.camera_namespace = camera_namespace.empty() ? "/camera" : camera_namespace;
  if (topics.camera_namespace.front() != '/') {
    topics.camera_namespace.insert(topics.camera_namespace.begin(), '/');
  }
  while (topics.camera_namespace.size() > 1 && topics.camera_namespace.back() == '/') {
    topics.camera_namespace.pop_back();
  }

  topics.color_image = make_topic(topics.camera_namespace, "color/image_raw");
  topics.color_camera_info = make_topic(topics.camera_namespace, "color/camera_info");
  topics.color_metadata = make_topic(topics.camera_namespace, "color/metadata");
  topics.color_undistorted_image =
      make_topic(topics.camera_namespace, "color/image_undistorted");
  topics.left_color_image = make_topic(topics.camera_namespace, "left_color/image_raw");
  topics.left_color_camera_info =
      make_topic(topics.camera_namespace, "left_color/camera_info");
  topics.right_color_image = make_topic(topics.camera_namespace, "right_color/image_raw");
  topics.right_color_camera_info =
      make_topic(topics.camera_namespace, "right_color/camera_info");

  topics.depth_image = make_topic(topics.camera_namespace, "depth/image_raw");
  topics.depth_camera_info = make_topic(topics.camera_namespace, "depth/camera_info");
  topics.depth_metadata = make_topic(topics.camera_namespace, "depth/metadata");
  topics.depth_unaligned_image =
      make_topic(topics.camera_namespace, "depth/image_unaligned");
  topics.depth_points = make_topic(topics.camera_namespace, "depth/points");
  topics.depth_registered_points =
      make_topic(topics.camera_namespace, "depth_registered/points");
  topics.depth_to_color_image =
      make_topic(topics.camera_namespace, "depth_to_color/image_raw");

  topics.ir_image = make_topic(topics.camera_namespace, "ir/image_raw");
  topics.ir_camera_info = make_topic(topics.camera_namespace, "ir/camera_info");
  topics.left_ir_image = make_topic(topics.camera_namespace, "left_ir/image_raw");
  topics.left_ir_camera_info = make_topic(topics.camera_namespace, "left_ir/camera_info");
  topics.right_ir_image = make_topic(topics.camera_namespace, "right_ir/image_raw");
  topics.right_ir_camera_info =
      make_topic(topics.camera_namespace, "right_ir/camera_info");

  topics.gyro_sample = make_topic(topics.camera_namespace, "gyro/sample");
  topics.gyro_imu_info = make_topic(topics.camera_namespace, "gyro/imu_info");
  topics.accel_sample = make_topic(topics.camera_namespace, "accel/sample");
  topics.accel_imu_info = make_topic(topics.camera_namespace, "accel/imu_info");
  topics.gyro_accel_sample = make_topic(topics.camera_namespace, "gyro_accel/sample");

  topics.depth_to_ir_extrinsics = make_topic(topics.camera_namespace, "depth_to_ir");
  topics.depth_to_color_extrinsics =
      make_topic(topics.camera_namespace, "depth_to_color");
  topics.depth_to_left_ir_extrinsics =
      make_topic(topics.camera_namespace, "depth_to_left_ir");
  topics.depth_to_right_ir_extrinsics =
      make_topic(topics.camera_namespace, "depth_to_right_ir");
  topics.depth_to_accel_extrinsics =
      make_topic(topics.camera_namespace, "depth_to_accel");
  topics.depth_to_gyro_extrinsics = make_topic(topics.camera_namespace, "depth_to_gyro");
  topics.left_color_to_right_color_extrinsics =
      make_topic(topics.camera_namespace, "left_color_to_right_color");

  topics.device_status = make_topic(topics.camera_namespace, "device_status");
  topics.depth_filter_status = make_topic(topics.camera_namespace, "depth_filter_status");
  topics.depth_filters_status = make_topic(topics.camera_namespace, "depth_filters/status");
  return topics;
}

}  // namespace lingtu::drivers::camera
