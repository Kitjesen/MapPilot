#include "planning/local/planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "planning/local/cmu/backend.hpp"
#include "planning/local/scan/backend.hpp"
#include "planning/local/task.hpp"
#include "trajectory/spline.hpp"

namespace nav_kernel {

namespace {

bool finitePoint(const Vec3 &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

Vec3 gridPoint(const LocalCollisionMapView &grid, const Vec3 &planning_point) {
  const double c = std::cos(grid.gridFromPlanningYaw);
  const double s = std::sin(grid.gridFromPlanningYaw);
  return {
      grid.gridFromPlanningTranslation.x + c * planning_point.x - s * planning_point.y,
      grid.gridFromPlanningTranslation.y + s * planning_point.x + c * planning_point.y,
      grid.gridFromPlanningTranslation.z + planning_point.z,
  };
}

}  // namespace

bool LocalCollisionMapView::present() const noexcept {
  return inflatedBits != nullptr || inflatedBytes != 0U || sizeX != 0 || sizeY != 0 ||
      sizeZ != 0 || resolution > 0.0 || resetEpoch != 0U || observationSequence != 0U ||
      generation != 0U;
}

std::size_t LocalCollisionMapView::cellCount() const noexcept {
  if (sizeX <= 0 || sizeY <= 0 || sizeZ <= 0) {
    return 0U;
  }
  const std::uint64_t x = static_cast<std::uint64_t>(sizeX);
  const std::uint64_t y = static_cast<std::uint64_t>(sizeY);
  const std::uint64_t z = static_cast<std::uint64_t>(sizeZ);
  if (x > std::numeric_limits<std::uint64_t>::max() / y ||
      x * y > std::numeric_limits<std::uint64_t>::max() / z) {
    return 0U;
  }
  const std::uint64_t cells = x * y * z;
  return cells <= std::numeric_limits<std::size_t>::max()
      ? static_cast<std::size_t>(cells)
      : 0U;
}

bool LocalCollisionMapView::valid() const noexcept {
  const std::size_t cells = cellCount();
  const std::size_t expected_bytes = (cells + 7U) / 8U;
  if (cells == 0U || inflatedBits == nullptr || inflatedBytes != expected_bytes ||
      !std::isfinite(resolution) || resolution <= 0.0 || !finitePoint(aabbMin) ||
      !finitePoint(aabbMax) || !finitePoint(gridFromPlanningTranslation) ||
      !std::isfinite(gridFromPlanningYaw)) {
    return false;
  }
  const double tolerance = std::max(1e-6, 1e-5 * resolution);
  return std::abs((aabbMax.x - aabbMin.x) - static_cast<double>(sizeX) * resolution) <= tolerance &&
      std::abs((aabbMax.y - aabbMin.y) - static_cast<double>(sizeY) * resolution) <= tolerance &&
      std::abs((aabbMax.z - aabbMin.z) - static_cast<double>(sizeZ) * resolution) <= tolerance;
}

bool LocalCollisionMapView::covers(const Vec3 &planning_point, double tolerance) const noexcept {
  if (!valid() || !finitePoint(planning_point) || !std::isfinite(tolerance)) {
    return false;
  }
  const Vec3 point = gridPoint(*this, planning_point);
  const double allowance = std::max(0.0, tolerance);
  return point.x >= aabbMin.x - allowance && point.x <= aabbMax.x + allowance &&
      point.y >= aabbMin.y - allowance && point.y <= aabbMax.y + allowance &&
      point.z >= aabbMin.z - allowance && point.z <= aabbMax.z + allowance;
}

bool LocalCollisionMapView::coversCylinder(const Vec3 &planning_center, double radius,
                                           double below, double above) const noexcept {
  if (!valid() || !finitePoint(planning_center) || !std::isfinite(radius) ||
      !std::isfinite(below) || !std::isfinite(above) || radius < 0.0 || below < 0.0 ||
      above < 0.0) {
    return false;
  }
  const Vec3 center = gridPoint(*this, planning_center);
  return center.x - radius >= aabbMin.x - 1e-9 &&
      center.x + radius <= aabbMax.x + 1e-9 &&
      center.y - radius >= aabbMin.y - 1e-9 &&
      center.y + radius <= aabbMax.y + 1e-9 &&
      center.z - below >= aabbMin.z - 1e-9 &&
      center.z + above <= aabbMax.z + 1e-9;
}

bool LocalCollisionMapView::occupiedLinear(std::size_t linear) const noexcept {
  if (!valid() || linear >= cellCount()) {
    return true;
  }
  return (inflatedBits[linear / 8U] &
          static_cast<std::uint8_t>(1U << (linear % 8U))) != 0U;
}

bool LocalCollisionMapView::occupied(const Vec3 &planning_point) const noexcept {
  if (!valid() || !finitePoint(planning_point)) {
    return true;
  }
  const Vec3 point = gridPoint(*this, planning_point);
  const int x = static_cast<int>(std::floor((point.x - aabbMin.x) / resolution));
  const int y = static_cast<int>(std::floor((point.y - aabbMin.y) / resolution));
  const int z = static_cast<int>(std::floor((point.z - aabbMin.z) / resolution));
  if (x < 0 || x >= sizeX || y < 0 || y >= sizeY || z < 0 || z >= sizeZ) {
    return true;
  }
  const std::size_t linear =
      (static_cast<std::size_t>(z) * static_cast<std::size_t>(sizeY) +
       static_cast<std::size_t>(y)) *
          static_cast<std::size_t>(sizeX) +
      static_cast<std::size_t>(x);
  return (inflatedBits[linear / 8U] &
          static_cast<std::uint8_t>(1U << (linear % 8U))) != 0U;
}

std::size_t LocalCollisionMapView::occupiedCount() const noexcept {
  if (!valid()) {
    return 0U;
  }
  if (occupiedCellsKnown) {
    return occupiedCells;
  }
  std::size_t count = 0U;
  for (std::size_t index = 0U; index < inflatedBytes; ++index) {
    std::uint8_t value = inflatedBits[index];
    while (value != 0U) {
      value = static_cast<std::uint8_t>(value & static_cast<std::uint8_t>(value - 1U));
      ++count;
    }
  }
  return count;
}

Vec3 LocalCollisionMapView::planningCellCenter(std::size_t linear) const noexcept {
  if (!valid() || linear >= cellCount()) {
    return {};
  }
  const std::size_t plane = static_cast<std::size_t>(sizeX) * static_cast<std::size_t>(sizeY);
  const int z = static_cast<int>(linear / plane);
  const std::size_t remainder = linear % plane;
  const int y = static_cast<int>(remainder / static_cast<std::size_t>(sizeX));
  const int x = static_cast<int>(remainder % static_cast<std::size_t>(sizeX));
  const Vec3 grid_center{
      aabbMin.x + (static_cast<double>(x) + 0.5) * resolution,
      aabbMin.y + (static_cast<double>(y) + 0.5) * resolution,
      aabbMin.z + (static_cast<double>(z) + 0.5) * resolution,
  };
  const double dx = grid_center.x - gridFromPlanningTranslation.x;
  const double dy = grid_center.y - gridFromPlanningTranslation.y;
  const double c = std::cos(gridFromPlanningYaw);
  const double s = std::sin(gridFromPlanningYaw);
  return {
      c * dx + s * dy,
      -s * dx + c * dy,
      grid_center.z - gridFromPlanningTranslation.z,
  };
}

const LocalRouteView *LocalPlanRequest::route() const noexcept {
  if (const auto *target = std::get_if<RouteTarget>(&objective))
    return &target->route;
  if (const auto *intent = std::get_if<MotionIntentTarget>(&objective))
    return &intent->guide;
  return nullptr;
}

const LocalMotionIntent *LocalPlanRequest::intent() const noexcept {
  if (const auto *target = std::get_if<MotionIntentTarget>(&objective))
    return &target->intent;
  return nullptr;
}

struct LocalPlan::Payload {
  explicit Payload(PathTarget path) : target(std::move(path)) {}

  explicit Payload(SplineTarget spline) : target(std::move(spline)) {
    const auto &spline_target = std::get<SplineTarget>(target);
    const SplineView view(spline_target);
    if (!view.valid())
      return;

    const double duration = view.duration();
    const double step = std::max(0.01, std::min(0.05, view.interval() * 0.25));
    const int segments = std::max(1, static_cast<int>(std::ceil(duration / step)));
    preview.reserve(static_cast<std::size_t>(segments) + 1U);
    for (int index = 0; index <= segments; ++index) {
      preview.push_back(view.position(duration * static_cast<double>(index) /
                                      static_cast<double>(segments)));
    }
  }

  FollowTarget target;
  std::vector<Vec3> preview;
};

LocalPlan::LocalPlan() {
  static const auto empty = std::make_shared<const Payload>(PathTarget{});
  payload_ = empty;
}

LocalPlan LocalPlan::stopped(LocalPlanStatus status, ControlHints hints) {
  LocalPlan plan;
  plan.status_ = status;
  plan.hints_ = hints;
  return plan;
}

LocalPlan LocalPlan::path(std::vector<Vec3> points, ControlHints hints) {
  LocalPlan plan;
  plan.status_ = points.size() >= 2U ? LocalPlanStatus::Ready : LocalPlanStatus::NoPath;
  plan.payload_ = std::make_shared<const Payload>(PathTarget{std::move(points)});
  plan.hints_ = hints;
  return plan;
}

LocalPlan LocalPlan::path(std::vector<Vec3> points, LocalPlanStatus status,
                          ControlHints hints) {
  LocalPlan plan = path(std::move(points), hints);
  if (status != LocalPlanStatus::Ready)
    plan.status_ = status;
  return plan;
}

LocalPlan LocalPlan::spline(SplineTarget target, ControlHints hints) {
  LocalPlan plan;
  plan.status_ = SplineView(target).valid() ? LocalPlanStatus::Ready : LocalPlanStatus::NoPath;
  plan.payload_ = std::make_shared<const Payload>(std::move(target));
  plan.hints_ = hints;
  return plan;
}

LocalPlanStatus LocalPlan::status() const noexcept {
  return status_;
}

bool LocalPlan::ready() const noexcept {
  return status_ == LocalPlanStatus::Ready;
}

const FollowTarget &LocalPlan::target() const noexcept {
  return payload_->target;
}

const std::vector<Vec3> &LocalPlan::previewPath() const noexcept {
  if (const auto *path = std::get_if<PathTarget>(&payload_->target))
    return path->points;
  return payload_->preview;
}

const ControlHints &LocalPlan::hints() const noexcept {
  return hints_;
}

class local::Planner::Impl {
 public:
  explicit Impl(LocalPlannerParams params) : params_(std::move(params)) {
    if (params_.backend == LocalPlannerBackend::Scan) {
      task_ = std::make_unique<local::LocalPlanTask>(params_);
    } else {
      cmu_ = std::make_unique<local::cmu::Backend>(params_);
    }
  }

  bool configure(const std::string &path_library_dir) {
    if (params_.backend == LocalPlannerBackend::Scan) {
      const bool loaded = task_ != nullptr && task_->configure(path_library_dir);
      configured_ = configured_ || loaded;
      return loaded;
    }
    const bool loaded = cmu_ != nullptr && !path_library_dir.empty() &&
                        cmu_->loadPaths(path_library_dir);
    configured_ = configured_ || loaded;
    return loaded;
  }

  bool configured() const { return configured_; }

  LocalPlannerDebugSnapshot debugSnapshot() const {
    LocalPlannerDebugSnapshot snapshot =
        params_.backend == LocalPlannerBackend::Scan
            ? last_debug_
            : (cmu_ != nullptr ? cmu_->debugSnapshot() : LocalPlannerDebugSnapshot{});
    snapshot.backend = params_.backend;
    return snapshot;
  }

  LocalPlan plan(const LocalPlanRequest &request, const LocalPlanCancel &cancel = {}) {
    if (!configured_)
      return LocalPlan::stopped(LocalPlanStatus::NotConfigured);
    const LocalRouteView *route = request.route();
    if (route == nullptr || !route->valid())
      return LocalPlan::stopped(LocalPlanStatus::InvalidInput);
    if (cancel && cancel())
      return LocalPlan::stopped(LocalPlanStatus::Cancelled);
    if (params_.backend == LocalPlannerBackend::Scan) {
      const local::LocalPlanUpdate update = task_->update(request);
      last_debug_ = update.debug;
      return update.plan;
    }
    return cmu_->plan(request);
  }

  void reset() {
    if (params_.backend == LocalPlannerBackend::Scan)
      task_->reset();
    else
      cmu_->reset();
  }

  const LocalPlannerParams &params() const { return params_; }

 private:
  LocalPlannerParams params_;
  std::unique_ptr<local::cmu::Backend> cmu_;
  std::unique_ptr<local::LocalPlanTask> task_;
  LocalPlannerDebugSnapshot last_debug_{};
  bool configured_{false};
};

const char *localPlannerBackendName(LocalPlannerBackend backend) {
  switch (backend) {
    case LocalPlannerBackend::Cmu:
      return "cmu";
    case LocalPlannerBackend::Scan:
      return "scan";
  }
  return "cmu";
}

const char *localPlanStatusName(LocalPlanStatus status) {
  switch (status) {
    case LocalPlanStatus::Ready:
      return "ready";
    case LocalPlanStatus::Pending:
      return "pending";
    case LocalPlanStatus::InvalidInput:
      return "invalid_input";
    case LocalPlanStatus::NotConfigured:
      return "not_configured";
    case LocalPlanStatus::NoPath:
      return "no_path";
    case LocalPlanStatus::Blocked:
      return "blocked";
    case LocalPlanStatus::NearFieldStop:
      return "near_field_stop";
    case LocalPlanStatus::Cancelled:
      return "cancelled";
    case LocalPlanStatus::Expired:
      return "expired";
  }
  return "invalid_input";
}

local::Planner::Planner(const LocalPlannerParams &params) : impl_(std::make_unique<Impl>(params)) {}

local::Planner::~Planner() = default;
local::Planner::Planner(local::Planner &&) noexcept = default;
local::Planner &local::Planner::operator=(local::Planner &&) noexcept = default;

bool local::Planner::configure(const std::string &path_library_dir) {
  return impl_->configure(path_library_dir);
}

bool local::Planner::configured() const {
  return impl_->configured();
}

LocalPlannerDebugSnapshot local::Planner::debugSnapshot() const {
  return impl_->debugSnapshot();
}

LocalPlan local::Planner::plan(const LocalPlanRequest &request) {
  return impl_->plan(request);
}

LocalPlan local::Planner::plan(const LocalPlanRequest &request,
                               const LocalPlanCancel &cancel) {
  return impl_->plan(request, cancel);
}

void local::Planner::reset() {
  impl_->reset();
}

const LocalPlannerParams &local::Planner::params() const {
  return impl_->params();
}

}  // namespace nav_kernel
