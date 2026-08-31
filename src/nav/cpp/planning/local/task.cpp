#include "planning/local/task.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <limits>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include "planning/local/scan/backend.hpp"
#include "trajectory/spline.hpp"

namespace nav_kernel::local {
namespace {

struct Completion {
  std::uint64_t sequence{0};
  Pose vehicle{};
  PlanIdentity identity{};
  std::uint64_t routeGeneration{0};
  PlanClock clock{};
  std::optional<LocalMotionIntent> intent;
  LocalPlan plan{};
  LocalPlannerDebugSnapshot debug{};
};

struct OwnedRequest {
  explicit OwnedRequest(const LocalPlanRequest &source)
      : robot(source.robot),
        identity(source.identity),
        clock(source.clock),
        environment(source.environment) {
    if (const LocalRouteView *source_route = source.route()) {
      routeView = *source_route;
      if (source_route->points != nullptr && source_route->count > 0)
        route.assign(source_route->points, source_route->points + source_route->count);
    }
    if (const LocalMotionIntent *source_intent = source.intent())
      intent = *source_intent;
    if (source.environment.obstacles.xyzh != nullptr &&
        source.environment.obstacles.count > 0) {
      obstacles.assign(
          source.environment.obstacles.xyzh,
          source.environment.obstacles.xyzh +
              static_cast<std::size_t>(source.environment.obstacles.count) * 4U);
    }
    if (source.environment.collision.occupiedXyz != nullptr &&
        source.environment.collision.occupiedCount > 0) {
      collision.assign(
          source.environment.collision.occupiedXyz,
          source.environment.collision.occupiedXyz +
              static_cast<std::size_t>(source.environment.collision.occupiedCount) * 3U);
    }
    if (source.environment.traversability.values != nullptr &&
        source.environment.traversability.rows > 0 &&
        source.environment.traversability.cols > 0) {
      const std::size_t cells =
          static_cast<std::size_t>(source.environment.traversability.rows) *
          static_cast<std::size_t>(source.environment.traversability.cols);
      traversability.assign(source.environment.traversability.values,
                            source.environment.traversability.values + cells);
    }
  }

  LocalPlanRequest view() const {
    LocalPlanRequest request;
    request.robot = robot;
    request.identity = identity;
    request.clock = clock;

    LocalRouteView route_view = routeView;
    route_view.points = route.empty() ? nullptr : route.data();
    request.objective = intent ? LocalObjective{MotionIntentTarget{*intent, route_view}}
                               : LocalObjective{RouteTarget{route_view}};

    request.environment = environment;
    request.environment.obstacles.xyzh = obstacles.empty() ? nullptr : obstacles.data();
    request.environment.collision.occupiedXyz =
        collision.empty() ? nullptr : collision.data();
    request.environment.traversability.values =
        traversability.empty() ? nullptr : traversability.data();
    return request;
  }

  RobotState robot{};
  PlanIdentity identity{};
  PlanClock clock{};
  LocalRouteView routeView{};
  EnvironmentView environment{};
  std::optional<LocalMotionIntent> intent;
  std::vector<Vec3> route;
  std::vector<float> obstacles;
  std::vector<float> collision;
  std::vector<float> traversability;
};

struct Job {
  std::uint64_t sequence{0};
  std::uint64_t epoch{0};
  OwnedRequest request;
  std::shared_ptr<std::atomic_bool> cancel;
  std::chrono::steady_clock::time_point deadline;

  Job(std::uint64_t sequence_value, std::uint64_t epoch_value, OwnedRequest owned_request,
      std::shared_ptr<std::atomic_bool> cancel_token,
      std::chrono::steady_clock::time_point planning_deadline)
      : sequence(sequence_value),
        epoch(epoch_value),
        request(std::move(owned_request)),
        cancel(std::move(cancel_token)),
        deadline(planning_deadline) {}

  [[nodiscard]] bool cancelled() const noexcept {
    return cancel && cancel->load(std::memory_order_relaxed);
  }

  [[nodiscard]] bool deadlineReached() const noexcept {
    return std::chrono::steady_clock::now() >= deadline;
  }
};

double executionTime(const PlanClock &clock) {
  return std::isfinite(clock.executionTimeS) && clock.executionTimeS >= 0.0
             ? clock.executionTimeS
             : clock.timestampS;
}

bool sameIdentity(const PlanIdentity &left, const PlanIdentity &right) {
  return left.frameEpoch == right.frameEpoch &&
         left.obstacleGeneration == right.obstacleGeneration &&
         left.traversabilityGeneration == right.traversabilityGeneration;
}

bool sameIntent(const std::optional<LocalMotionIntent> &left,
                const std::optional<LocalMotionIntent> &right) {
  if (left.has_value() != right.has_value())
    return false;
  if (!left)
    return true;
  // Guide generation owns direction and horizon identity; both move with the robot.
  return std::abs(left->speedNormalized - right->speedNormalized) <= 0.05 &&
         std::abs(left->maxDirectionDeviationDeg - right->maxDirectionDeviationDeg) <= 1e-6;
}

Vec3 bodyPointToPlanning(const Pose &body, const Vec3 &point) {
  const double c = std::cos(body.yaw);
  const double s = std::sin(body.yaw);
  return {
      body.position.x + c * point.x - s * point.y,
      body.position.y + s * point.x + c * point.y,
      body.position.z + point.z,
  };
}

Vec3 planningPointToBody(const Pose &body, const Vec3 &point) {
  const double dx = point.x - body.position.x;
  const double dy = point.y - body.position.y;
  const double c = std::cos(body.yaw);
  const double s = std::sin(body.yaw);
  return {
      c * dx + s * dy,
      -s * dx + c * dy,
      point.z - body.position.z,
  };
}

Vec3 relocatePoint(const Pose &source_body, const Pose &target_body, const Vec3 &point) {
  return planningPointToBody(target_body, bodyPointToPlanning(source_body, point));
}

LocalPlan relocatePlan(const Completion &completion, const LocalPlanRequest &request) {
  if (!completion.plan.ready())
    return completion.plan;

  if (const auto *path = std::get_if<PathTarget>(&completion.plan.target())) {
    std::vector<Vec3> relocated;
    relocated.reserve(path->points.size());
    for (const Vec3 &point : path->points)
      relocated.push_back(relocatePoint(completion.vehicle, request.robot.pose, point));
    const auto closest = std::min_element(
        relocated.begin(), relocated.end(), [](const Vec3 &left, const Vec3 &right) {
          return std::hypot(left.x, left.y) < std::hypot(right.x, right.y);
        });
    std::vector<Vec3> remaining{{}};
    if (closest != relocated.end()) {
      for (auto point = closest; point != relocated.end(); ++point) {
        if (distance3D(remaining.back(), *point) > 1e-4)
          remaining.push_back(*point);
      }
    }
    return remaining.size() >= 2U
               ? LocalPlan::path(std::move(remaining), completion.plan.hints())
               : LocalPlan::stopped(LocalPlanStatus::Expired, completion.plan.hints());
  }

  const auto *source = std::get_if<SplineTarget>(&completion.plan.target());
  if (source == nullptr)
    return LocalPlan::stopped(LocalPlanStatus::InvalidInput, completion.plan.hints());
  SplineTarget target = *source;
  for (Vec3 &control : target.controls)
    control = relocatePoint(completion.vehicle, request.robot.pose, control);
  const SplineView spline(target);
  if (!spline.valid())
    return LocalPlan::stopped(LocalPlanStatus::Expired, completion.plan.hints());
  const double started =
      std::isfinite(source->startTimeS) && source->startTimeS >= 0.0
          ? source->startTimeS
          : executionTime(completion.clock);
  const double elapsed = std::max(0.0, executionTime(request.clock) - started);
  target.timeS = std::clamp(std::max(0.0, source->timeS) + elapsed, 0.0, spline.duration());
  target.startTimeS = executionTime(request.clock);
  return LocalPlan::spline(std::move(target), completion.plan.hints());
}

LocalPlannerDebugSnapshot relocateDebug(const Completion &completion,
                                        const LocalPlanRequest &request) {
  LocalPlannerDebugSnapshot debug = completion.debug;
  for (LocalPlanCandidate &candidate : debug.candidates) {
    for (Vec3 &point : candidate.path)
      point = relocatePoint(completion.vehicle, request.robot.pose, point);
  }
  return debug;
}

std::optional<LocalPlan> revalidate(const Completion &completion,
                                    const LocalPlanRequest &request,
                                    const scan::Backend &validator,
                                    double max_path_length_m =
                                        std::numeric_limits<double>::infinity()) {
  LocalPlan retained = relocatePlan(completion, request);
  if (!retained.ready())
    return std::nullopt;
  const std::vector<Vec3> preview = retained.previewPath();
  std::vector<Vec3> planning_path;
  planning_path.reserve(preview.size() + 1U);
  planning_path.push_back(request.robot.pose.position);
  double accumulated_length_m = 0.0;
  for (const Vec3 &point : preview) {
    const Vec3 planning_point = bodyPointToPlanning(request.robot.pose, point);
    const double segment_length_m = distance3D(planning_path.back(), planning_point);
    if (segment_length_m <= 1e-4)
      continue;
    if (std::isfinite(max_path_length_m) &&
        accumulated_length_m + segment_length_m > max_path_length_m) {
      const double remaining_m = max_path_length_m - accumulated_length_m;
      if (remaining_m > 1e-4) {
        const double ratio = std::clamp(remaining_m / segment_length_m, 0.0, 1.0);
        const Vec3 &start = planning_path.back();
        planning_path.push_back({
            start.x + ratio * (planning_point.x - start.x),
            start.y + ratio * (planning_point.y - start.y),
            start.z + ratio * (planning_point.z - start.z),
        });
      }
      break;
    }
    planning_path.push_back(planning_point);
    accumulated_length_m += segment_length_m;
  }
  if (planning_path.size() < 2U || !validator.pathSafe(request, planning_path))
    return std::nullopt;
  return retained;
}

}  // namespace

class LocalPlanTask::Impl {
 public:
  explicit Impl(const LocalPlannerParams &params)
      : params_(params),
        planner_(params),
        validator_(params),
        planningBudget_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(
                std::clamp(params.scan.planningDeadlineS, 0.01, 1.0)))) {}

  ~Impl() { stop(); }

  bool configure(const std::string &path_library_dir) {
    (void)path_library_dir;
    if (configured_)
      return true;
    configured_ = true;
    if (configured_)
      worker_ = std::thread([this]() { run(); });
    return configured_;
  }

  bool configured() const { return configured_; }

  LocalPlanUpdate update(const LocalPlanRequest &request) {
    LocalPlanUpdate output;
    output.debug.backend = params_.backend;
    if (!configured_) {
      output.plan = LocalPlan::stopped(LocalPlanStatus::NotConfigured);
      return output;
    }
    const LocalRouteView *route = request.route();
    if (route == nullptr || !route->valid()) {
      output.plan = LocalPlan::stopped(LocalPlanStatus::InvalidInput);
      return output;
    }
    const std::optional<LocalMotionIntent> intent =
        request.intent() == nullptr ? std::nullopt
                                    : std::optional<LocalMotionIntent>{*request.intent()};

    if (auto completion = poll()) {
      const bool same_route = completion->routeGeneration == route->generation;
      const bool same_frame = completion->identity.frameEpoch == request.identity.frameEpoch;
      const bool same_motion = sameIntent(completion->intent, intent);
      bool accepted =
          same_route && same_frame && same_motion &&
          sameIdentity(completion->identity, request.identity);
      if (!accepted && same_route && same_frame && same_motion && completion->plan.ready()) {
        if (auto retained = revalidate(*completion, request, validator_)) {
          LocalPlannerDebugSnapshot relocated_debug = relocateDebug(*completion, request);
          completion->plan = std::move(*retained);
          completion->vehicle = request.robot.pose;
          completion->identity = request.identity;
          completion->clock = request.clock;
          completion->debug = std::move(relocated_debug);
          completion->debug.continuityReused = true;
          accepted = true;
        }
      }
      if (accepted)
        current_ = std::move(*completion);
    }

    const double now = request.clock.timestampS;
    const double refresh_interval =
        0.5 * std::max(0.05, params_.scan.continuityHorizon);
    const bool refresh_due =
        submittedTimeS_ < 0.0 || !std::isfinite(now) || now < submittedTimeS_ ||
        now - submittedTimeS_ >= refresh_interval;
    const bool submission_changed =
        submittedRouteGeneration_ != route->generation ||
        submittedIdentity_.frameEpoch != request.identity.frameEpoch ||
        !sameIntent(submittedIntent_, intent);
    bool submitted_now = false;
    if (submission_changed || refresh_due)
      submitted_now = submit(request);

    const bool current_exact =
        current_ && current_->routeGeneration == route->generation &&
        current_->identity.frameEpoch == request.identity.frameEpoch &&
        sameIntent(current_->intent, intent) &&
        sameIdentity(current_->identity, request.identity);
    if (current_exact) {
      output.plan = relocatePlan(*current_, request);
      output.debug = relocateDebug(*current_, request);
      return output;
    }

    const bool same_guide =
        current_ && current_->routeGeneration == route->generation &&
        current_->identity.frameEpoch == request.identity.frameEpoch &&
        sameIntent(current_->intent, intent);
    if (same_guide) {
      if (auto retained = revalidate(*current_, request, validator_)) {
        output.plan = std::move(*retained);
        output.debug = relocateDebug(*current_, request);
        output.debug.continuityReused = true;
        return output;
      }
      const double handoff_distance_m =
          std::max(0.25, params_.maxSpeed * refresh_interval);
      if (auto retained =
              revalidate(*current_, request, validator_, handoff_distance_m)) {
        if (!submitted_now)
          submit(request);
        output.plan = std::move(*retained);
        output.debug = relocateDebug(*current_, request);
        output.debug.continuityReused = true;
        return output;
      }
    }

    output.plan = LocalPlan::stopped(LocalPlanStatus::Pending);
    return output;
  }

  void reset() {
    current_.reset();
    submittedIdentity_ = {};
    submittedRouteGeneration_ = 0;
    submittedTimeS_ = -1.0;
    submittedIntent_.reset();
    if (!configured_)
      return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++epoch_;
      if (pending_ && pending_->cancel)
        pending_->cancel->store(true, std::memory_order_relaxed);
      if (activeCancel_)
        activeCancel_->store(true, std::memory_order_relaxed);
      pending_.reset();
      latest_.reset();
      resetRequested_ = true;
    }
    cv_.notify_one();
    validator_.reset();
  }

 private:
  bool submit(const LocalPlanRequest &request) {
    OwnedRequest owned(request);
    const LocalRouteView *route = request.route();
    const std::optional<LocalMotionIntent> intent =
        request.intent() == nullptr ? std::nullopt
                                    : std::optional<LocalMotionIntent>{*request.intent()};
    std::lock_guard<std::mutex> lock(mutex_);
    const std::uint64_t sequence = ++nextSequence_;
    if (pending_ && pending_->cancel)
      pending_->cancel->store(true, std::memory_order_relaxed);
    const bool active_superseded =
        activeCancel_ &&
        (activeRouteGeneration_ != (route == nullptr ? 0U : route->generation) ||
         activeFrameEpoch_ != request.identity.frameEpoch ||
         !sameIntent(activeIntent_, intent));
    if (active_superseded)
      activeCancel_->store(true, std::memory_order_relaxed);
    auto cancel = std::make_shared<std::atomic_bool>(false);
    pending_.emplace(sequence, epoch_, std::move(owned), cancel,
                     std::chrono::steady_clock::now() + planningBudget_);
    submittedIdentity_ = request.identity;
    submittedRouteGeneration_ = route == nullptr ? 0U : route->generation;
    submittedTimeS_ = request.clock.timestampS;
    submittedIntent_ = intent;
    cv_.notify_one();
    return true;
  }

  std::optional<Completion> poll() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto completion = std::move(latest_);
    latest_.reset();
    return completion;
  }

  void stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_)
        return;
      stopping_ = true;
      if (pending_ && pending_->cancel)
        pending_->cancel->store(true, std::memory_order_relaxed);
      if (activeCancel_)
        activeCancel_->store(true, std::memory_order_relaxed);
      pending_.reset();
    }
    cv_.notify_one();
    if (worker_.joinable())
      worker_.join();
  }

  void run() {
    for (;;) {
      std::optional<Job> job;
      bool reset = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return stopping_ || resetRequested_ || pending_.has_value(); });
        if (stopping_)
          return;
        if (resetRequested_) {
          resetRequested_ = false;
          reset = true;
        } else {
          job = std::move(pending_);
          pending_.reset();
          activeCancel_ = job->cancel;
          activeRouteGeneration_ = job->request.routeView.generation;
          activeFrameEpoch_ = job->request.identity.frameEpoch;
          activeIntent_ = job->request.intent;
        }
      }
      if (reset) {
        planner_.reset();
        continue;
      }
      if (!job)
        continue;

      const LocalPlanRequest request = job->request.view();
      const LocalRouteView *route = request.route();
      Completion completion;
      completion.sequence = job->sequence;
      completion.vehicle = request.robot.pose;
      completion.identity = request.identity;
      completion.routeGeneration = route == nullptr ? 0U : route->generation;
      completion.clock = request.clock;
      completion.intent = job->request.intent;
      const auto cancelled = [&]() { return job->cancelled() || job->deadlineReached(); };
      try {
        completion.plan = planner_.plan(request, cancelled);
        completion.debug = planner_.debugSnapshot();
      } catch (const std::exception &error) {
        completion.plan = LocalPlan::stopped(LocalPlanStatus::InvalidInput);
        completion.debug.searchReason = std::string("planner_exception:") + error.what();
      } catch (...) {
        completion.plan = LocalPlan::stopped(LocalPlanStatus::InvalidInput);
        completion.debug.searchReason = "planner_exception";
      }

      std::lock_guard<std::mutex> lock(mutex_);
      if (activeCancel_ == job->cancel) {
        activeCancel_.reset();
        activeRouteGeneration_ = 0;
        activeFrameEpoch_ = 0;
        activeIntent_.reset();
      }
      if (!job->cancelled() && job->deadlineReached()) {
        completion.plan = LocalPlan::stopped(LocalPlanStatus::Expired);
        completion.debug.searchReason = "planning_deadline_exceeded";
      }
      if (!job->cancelled() && !resetRequested_ && job->epoch == epoch_ && !stopping_)
        latest_ = std::move(completion);
    }
  }

  LocalPlannerParams params_;
  scan::Backend planner_;
  scan::Backend validator_;
  bool configured_{false};
  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<Job> pending_;
  std::shared_ptr<std::atomic_bool> activeCancel_;
  std::uint64_t activeRouteGeneration_{0};
  std::uint64_t activeFrameEpoch_{0};
  std::optional<LocalMotionIntent> activeIntent_;
  std::optional<Completion> latest_;
  std::uint64_t nextSequence_{0};
  std::uint64_t epoch_{1};
  bool resetRequested_{false};
  bool stopping_{false};
  std::thread worker_;
  std::chrono::steady_clock::duration planningBudget_{};

  std::optional<Completion> current_;
  PlanIdentity submittedIdentity_{};
  std::uint64_t submittedRouteGeneration_{0};
  double submittedTimeS_{-1.0};
  std::optional<LocalMotionIntent> submittedIntent_;
};

LocalPlanTask::LocalPlanTask(const LocalPlannerParams &params)
    : impl_(std::make_unique<Impl>(params)) {}

LocalPlanTask::~LocalPlanTask() = default;

bool LocalPlanTask::configure(const std::string &path_library_dir) {
  return impl_->configure(path_library_dir);
}

bool LocalPlanTask::configured() const {
  return impl_->configured();
}

LocalPlanUpdate LocalPlanTask::update(const LocalPlanRequest &request) {
  return impl_->update(request);
}

void LocalPlanTask::reset() {
  impl_->reset();
}

}  // namespace nav_kernel::local
