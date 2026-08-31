#include "planning/local/planner.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "planning/local/cmu/backend.hpp"
#include "planning/local/scan/backend.hpp"
#include "planning/local/task.hpp"
#include "trajectory/spline.hpp"

namespace nav_kernel {

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

LocalPlan::LocalPlan() = default;

LocalPlan LocalPlan::stopped(LocalPlanStatus status, ControlHints hints) {
  LocalPlan plan;
  plan.status_ = status;
  plan.hints_ = hints;
  return plan;
}

LocalPlan LocalPlan::path(std::vector<Vec3> points, ControlHints hints) {
  LocalPlan plan;
  plan.status_ = points.size() >= 2U ? LocalPlanStatus::Ready : LocalPlanStatus::NoPath;
  plan.target_ = PathTarget{std::move(points)};
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
  plan.target_ = std::move(target);
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
  return target_;
}

std::vector<Vec3> LocalPlan::previewPath() const {
  if (const auto *path = std::get_if<PathTarget>(&target_))
    return path->points;

  const auto *target = std::get_if<SplineTarget>(&target_);
  if (target == nullptr)
    return {};
  const SplineView spline(*target);
  if (!spline.valid())
    return {};

  const double duration = spline.duration();
  const double step = std::max(0.01, std::min(0.05, target->intervalS * 0.25));
  const int segments = std::max(1, static_cast<int>(std::ceil(duration / step)));
  std::vector<Vec3> preview;
  preview.reserve(static_cast<std::size_t>(segments) + 1U);
  for (int index = 0; index <= segments; ++index) {
    preview.push_back(spline.position(duration * static_cast<double>(index) /
                                      static_cast<double>(segments)));
  }
  return preview;
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
