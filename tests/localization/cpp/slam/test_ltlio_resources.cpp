#include "ltlio/frontend.hpp"
#include "ltlio/detail/Context.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace {
namespace lio = lingtu::ltlio;
namespace detail = lingtu::ltlio::detail;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

lio::Config baseConfig() {
  lio::Config config;
  config.elevator_enabled = false;
  config.automatic_door = false;
  config.imu_init_samples = 20;
  config.point_filter_num = 1;
  config.blind_m = 0.1;
  config.scan_resolution = 0.03;
  return config;
}

lio::TimedCloud room(double stamp, const Eigen::Vector3d& position = Eigen::Vector3d::Zero(),
                     const Eigen::Quaterniond& rotation = Eigen::Quaterniond::Identity(),
                     float marker = 0.0F, double world_shift_x = 0.0) {
  lio::TimedCloud scan;
  scan.stamp_s = stamp;
  scan.end_stamp_s = stamp + 0.020;
  auto append = [&](const Eigen::Vector3d& world, float intensity) {
    const Eigen::Vector3d sensor = rotation.conjugate() * (world - position);
    lio::TimedPoint point;
    point.x = static_cast<float>(sensor.x());
    point.y = static_cast<float>(sensor.y());
    point.z = static_cast<float>(sensor.z());
    point.intensity = intensity;
    scan.points.push_back(point);
  };
  for (int surface = 0; surface < 3; ++surface) {
    for (int a = -8; a <= 8; ++a) {
      for (int b = -8; b <= 8; ++b) {
        const double x = a * 0.073 + 0.007;
        const double y = b * 0.073 + 0.013;
        Eigen::Vector3d world;
        if (surface == 0) world = {3.031, x, y};
        else if (surface == 1) world = {x, 4.037, y};
        else world = {x, y, -2.041};
        world.x() += world_shift_x;
        append(world, 1.0F);
      }
    }
  }
  if (marker > 0.0F)
    append(Eigen::Vector3d(0.71 + (marker - 100.0) * 0.17, 0.83, 0.91), marker);
  for (std::size_t i = 0; i < scan.points.size(); ++i) {
    scan.points[i].offset_ns = static_cast<std::uint64_t>(
        std::llround(20'000'000.0 * i / (scan.points.size() - 1)));
    scan.points[i].line = static_cast<std::uint8_t>(i % 4);
  }
  return scan;
}

struct Session {
  explicit Session(const lio::Config& config) : frontend(config) {}
  lio::Frontend frontend;
  int imu_index = 0;
  int frame_index = 0;

  lio::Result process(lio::TimedCloud scan, const std::optional<lio::PosePrior>& prior = {}) {
    while (imu_index * 0.005 <= scan.end_stamp_s + 0.006 + 1e-12) {
      lio::ImuSample imu;
      imu.stamp_s = imu_index++ * 0.005;
      imu.acceleration = Eigen::Vector3d(0.0, 0.0, 9.80665);
      frontend.feedImu(imu);
    }
    frontend.feedCloud(scan);
    auto result = frontend.processNext([&](const lio::Result&) { return prior; });
    require(result.has_value(), "covered resource fixture did not return a scan");
    require(std::abs(result->scan_end_s - scan.end_stamp_s) < 1e-12,
            "resource fixture processed a different scan");
    ++frame_index;
    return *result;
  }

  lio::Result step(const Eigen::Vector3d& position = Eigen::Vector3d::Zero(),
                   const Eigen::Quaterniond& rotation = Eigen::Quaterniond::Identity(),
                   float marker = 0.0F, bool use_prior = false,
                   double world_shift_x = 0.0) {
    std::optional<lio::PosePrior> prior;
    if (use_prior) {
      prior.emplace();
      prior->world_imu_position = position;
      prior->world_imu_rotation = rotation;
      prior->world_imu_velocity = Eigen::Vector3d::Zero();
    }
    return process(room(0.20 + frame_index * 0.04, position, rotation, marker, world_shift_x), prior);
  }

  lio::Result initialize(bool use_prior = false) {
    lio::Result result;
    for (int frame = 0; frame < 12; ++frame)
      result = step(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), 0.0F, use_prior);
    require(result.accepted, "resource fixture did not establish actual LiDAR matching");
    return result;
  }
};

using MapPoints = std::vector<std::tuple<float, float, float, float>>;
MapPoints sortedMap(const lio::Cloud& cloud) {
  MapPoints points;
  for (const auto& p : cloud) points.emplace_back(p.x, p.y, p.z, p.intensity);
  std::sort(points.begin(), points.end());
  return points;
}

bool hasMarker(const lio::Cloud& cloud, float intensity) {
  return std::any_of(cloud.begin(), cloud.end(), [&](const auto& p) {
    return std::abs(p.intensity - intensity) < 0.01F;
  });
}

void fixedScanResolution() {
  auto fine_config = baseConfig();
  auto coarse_config = fine_config;
  coarse_config.scan_resolution = 0.30;
  Session fine(fine_config), coarse(coarse_config);
  const auto fine_result = fine.initialize();
  const auto coarse_result = coarse.initialize();
  require(fine_result.sampled_points > coarse_result.sampled_points * 2 &&
          coarse_result.sampled_points >= 10,
          "fixed scan resolution did not change the actual matching sample count");
  std::cout << "fixed scan points fine=" << fine_result.sampled_points
            << " coarse=" << coarse_result.sampled_points << '\n';
}

lio::Cloud::Ptr grid(float center_x, float intensity, int side = 16) {
  lio::Cloud::Ptr cloud(new lio::Cloud);
  for (int x = 0; x < side; ++x) {
    for (int y = 0; y < side; ++y) {
      pcl::PointXYZINormal point{};
      point.x = center_x + 0.031F + x * 0.083F;
      point.y = 0.037F + y * 0.083F;
      point.z = 0.043F;
      point.intensity = intensity;
      cloud->push_back(point);
    }
  }
  return cloud;
}

lio::Cloud snapshot(detail::Context& context) {
  detail::PointVector points;
  context.ikdtree.Snapshot(points);
  lio::Cloud cloud;
  cloud.points = std::move(points);
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  return cloud;
}

std::size_t populatedMapSize(double resolution) {
  auto config = baseConfig();
  config.map_resolution = resolution;
  const auto context = std::make_unique<detail::Context>(config);
  context->ikd_map->init(config.map_resolution);
  // The first Build has no incremental voxel selection. Seed it, then test
  // the real add path used while navigating through an existing local map.
  context->ikd_map->add(grid(-2.0F, 1.0F, 1));
  context->ikd_map->add(grid(1.0F, 2.0F));
  return snapshot(*context).size();
}

void mapResolutionAndBudget() {
  const auto fine = populatedMapSize(0.05);
  const auto coarse = populatedMapSize(0.40);
  require(fine > coarse * 2 && coarse > 1,
          "map resolution did not constrain incremental map density");

  auto config = baseConfig();
  config.map_resolution = 0.03;
  config.max_map_points = 64;
  config.cube_len = 100.0;
  config.det_range = 10.0;
  const auto context = std::make_unique<detail::Context>(config);
  context->ikd_map->removeFar(Eigen::Vector3d::Zero());
  context->ikd_map->add(grid(0.0F, 11.0F));
  const auto first = snapshot(*context);
  require(first.size() == 64 && hasMarker(first, 11.0F),
          "initial map insertion did not honor the active point budget");
  context->ikd_map->removeFar(Eigen::Vector3d(10.0, 0.0, 0.0));
  context->ikd_map->add(grid(10.0F, 22.0F));
  const auto second = snapshot(*context);
  require(!second.empty() && second.size() <= 64 && hasMarker(second, 22.0F),
          "map budget either overflowed or prevented a newly observed region from entering");
  require(!hasMarker(second, 11.0F),
          "map budget retained remote geometry instead of the current local neighborhood");
  std::cout << "map points fine=" << fine << " coarse=" << coarse
            << " capped=" << second.size() << '\n';
}

void stationaryInsertionRetainsMotion() {
  auto config = baseConfig();
  config.stationary_thresh = 0.10;
  config.det_range = 10.0;
  Session session(config);
  session.initialize(true);
  const auto before = sortedMap(*session.frontend.mapSnapshot());
  for (int frame = 1; frame <= 4; ++frame) {
    const float marker = 100.0F + frame;
    const auto result = session.step(Eigen::Vector3d(frame * 0.02, 0.0, 0.0),
                                    Eigen::Quaterniond::Identity(), marker, true);
    require(result.accepted, "small-motion insertion fixture lost matching");
    const auto map = session.frontend.mapSnapshot();
    require(!hasMarker(*map, marker) && sortedMap(*map) == before,
            "sub-threshold insertion did not remain throttled");
  }
  auto result = session.step(Eigen::Vector3d(0.12, 0.0, 0.0),
                            Eigen::Quaterniond::Identity(), 105.0F, true);
  require(result.accepted && hasMarker(*session.frontend.mapSnapshot(), 105.0F),
          "accumulated small translations never released map insertion");
  result = session.step(Eigen::Vector3d(0.12, 0.0, 0.0),
      Eigen::Quaterniond(Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ())), 106.0F, true);
  require(result.accepted && hasMarker(*session.frontend.mapSnapshot(), 106.0F),
          "rotation at a fixed origin discarded newly visible map geometry");
}

void rejectedCandidateDoesNotCommit() {
  auto guarded_config = baseConfig();
  guarded_config.max_update_translation_m = 0.001;
  Session guarded(guarded_config), unguarded(baseConfig()), prediction(baseConfig());
  guarded.initialize();
  unguarded.initialize();
  prediction.initialize();
  const auto original_map = sortedMap(*guarded.frontend.mapSnapshot());
  const auto rejected = guarded.step(Eigen::Vector3d::Zero(),
      Eigen::Quaterniond::Identity(), 0.0F, false, 0.04);
  const auto accepted = unguarded.step(Eigen::Vector3d::Zero(),
      Eigen::Quaterniond::Identity(), 0.0F, false, 0.04);
  auto sparse_scan = room(0.20 + prediction.frame_index * 0.04);
  sparse_scan.points.resize(3);
  sparse_scan.points.back().offset_ns = 20'000'000;
  const auto prior = prediction.process(std::move(sparse_scan));
  require(accepted.accepted && accepted.candidate_translation_m > 0.001,
          "candidate fixture did not exercise a real out-of-bound translation");
  require(rejected.attempted && !rejected.accepted &&
          rejected.rejection_reason == "update_translation_too_large",
          "translation bound did not reject the matched candidate");
  require(!prior.accepted &&
          (rejected.state.position - prior.state.position).norm() < 1e-9 &&
          rejected.state.orientation.angularDistance(prior.state.orientation) < 1e-9 &&
          (rejected.state.velocity - prior.state.velocity).norm() < 1e-9 &&
          (rejected.state.covariance - prior.state.covariance).norm() < 1e-9,
          "rejected candidate changed the IMU-predicted state or covariance");
  require(sortedMap(*guarded.frontend.mapSnapshot()) == original_map,
          "rejected candidate inserted geometry into the matching map");
  const auto resumed = guarded.step();
  const auto control = prediction.step();
  require(resumed.accepted && control.accepted &&
          (resumed.world_position - control.world_position).norm() < 1e-8 &&
          resumed.state.orientation.angularDistance(control.state.orientation) < 1e-8,
          "rejected candidate poisoned the next valid scan or prevented recovery");
}
}  // namespace

int main() {
  try {
    fixedScanResolution();
    mapResolutionAndBudget();
    stationaryInsertionRetainsMotion();
    rejectedCandidateDoesNotCommit();
    std::cout << "LTLIO resource budgets, insertion motion and rejected-state commit passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
