#include "core/dufo.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include <ufo/map/integration/integration.hpp>
#include <ufo/map/integration/integration_parameters.hpp>
#include <ufo/map/point_cloud.hpp>
#include <ufo/map/ufomap.hpp>

#include "core/evidence.hpp"

namespace lingtu::map_cleaning {

struct DufoMap::Impl {
  using Map = ufo::Map<ufo::MapType::SEEN_FREE | ufo::MapType::REFLECTION | ufo::MapType::LABEL>;

  explicit Impl(double resolution_m) : map(resolution_m, 20) {
    params.down_sampling_method = ufo::DownSamplingMethod::CENTER;
    params.min_range = 0.2F;
    params.max_range = -1.0F;
    params.inflate_unknown = 1;
    params.inflate_unknown_compensation = true;
    params.inflate_hits_dist = 0.2F;
    params.ray_passthrough_hits = false;
    params.ray_casting_method = ufo::RayCastingMethod::PROPER;
    params.parallel = true;
    params.num_threads = 2;
  }

  Map map;
  ufo::IntegrationParams params;
};

DufoMap::DufoMap(double resolution_m) : impl_(std::make_unique<Impl>(resolution_m)) {}
DufoMap::~DufoMap() = default;

void DufoMap::observe(const std::vector<PointXYZI>& scan, const Pose& pose,
                      const std::array<float, 3>& sensor_origin) {
  const auto origin = transformPoint({sensor_origin[0], sensor_origin[1], sensor_origin[2]}, pose);
  if (!std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(origin.z))
    throw std::runtime_error("transformed sensor origin is non-finite");
  const ufo::Point sensor{origin.x, origin.y, origin.z};
  ufo::PointCloud cloud;
  cloud.reserve(scan.size());
  for (const auto& local : scan) {
    const auto point = transformPoint(local, pose);
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      throw std::runtime_error("transformed scan point is non-finite");
    const ufo::Point world{point.x, point.y, point.z};
    if (sensor.squaredDistance(world) >= impl_->params.min_range * impl_->params.min_range)
      cloud.emplace_back(world);
  }
  // This overload expects both points and the calibrated sensor origin in map coordinates.
  ufo::insertPointCloud(impl_->map, std::move(cloud), sensor, impl_->params, false);
}

std::vector<std::uint8_t> DufoMap::removeMask(const std::vector<PointXYZI>& source) {
  impl_->map.propagateModified();
  std::vector<std::uint8_t> removed;
  removed.reserve(source.size());
  for (const auto& point : source)
    removed.push_back(impl_->map.seenFree(ufo::Point{point.x, point.y, point.z}) ? 1 : 0);
  return removed;
}

}  // namespace lingtu::map_cleaning
