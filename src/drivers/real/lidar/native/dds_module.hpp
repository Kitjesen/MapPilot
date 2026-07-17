#pragma once

#include "native/module.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace lingtu::drivers::lidar {

class DdsModule {
 public:
  DdsModule(
      int domain_id,
      std::string lidar_frame,
      std::string imu_frame,
      bool navigation_fixture = false);
  ~DdsModule();

  DdsModule(const DdsModule&) = delete;
  DdsModule& operator=(const DdsModule&) = delete;

  void publish_cloud(
      std::uint8_t lidar_id,
      std::uint64_t timestamp_ns,
      const std::vector<Point>& points);
  void publish_raw_packet(
      std::uint8_t lidar_id,
      std::uint64_t timestamp_ns,
      const std::vector<Point>& points);
  void publish_imu(std::uint64_t timestamp_ns, const ImuSample& imu);
  void publish_odom_prior(std::uint64_t timestamp_ns, const OdomPrior& prior);
  void publish_registered_cloud(
      std::uint64_t timestamp_ns,
      const std::vector<Point>& points);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::drivers::lidar
