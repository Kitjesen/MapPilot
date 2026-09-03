#include "planning/local/task.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "planning/local/scan/backend.hpp"
#include "planning/local/scan/grid.hpp"
#include "trajectory/spline.hpp"

namespace nav_kernel::local {
namespace {

bool sameIntent(const std::optional<LocalMotionIntent> &left,
                const std::optional<LocalMotionIntent> &right) {
  if (left.has_value() != right.has_value())
    return false;
  if (!left)
    return true;
  return std::abs(left->speedNormalized - right->speedNormalized) <= 0.05 &&
         std::abs(left->maxDirectionDeviationDeg -
                   right->maxDirectionDeviationDeg) <= 1e-6;
}

struct OwnedRequest {
  OwnedRequest(const LocalPlanRequest &source,
               std::shared_ptr<const std::vector<std::uint8_t>> collisionBits)
      : robot(source.robot),
        identity(source.identity),
        clock(source.clock),
        environment(source.environment),
        collision(std::move(collisionBits)) {
    if (const LocalRouteView *sourceRoute = source.route()) {
      routeView = *sourceRoute;
      if (sourceRoute->points != nullptr && sourceRoute->count > 0) {
        route.assign(sourceRoute->points,
                     sourceRoute->points + sourceRoute->count);
      }
    }
    if (const LocalMotionIntent *sourceIntent = source.intent())
      intent = *sourceIntent;
  }

  LocalPlanRequest view() const {
    LocalPlanRequest request;
    request.robot = robot;
    request.identity = identity;
    request.clock = clock;

    LocalRouteView routeViewCopy = routeView;
    routeViewCopy.points = route.empty() ? nullptr : route.data();
    request.objective = intent
                            ? LocalObjective{MotionIntentTarget{*intent,
                                                               routeViewCopy}}
                            : LocalObjective{RouteTarget{routeViewCopy}};

    request.environment = environment;
    request.environment.obstacles = {};
    request.environment.traversability = {};
    request.environment.collision.inflatedBits =
        collision && !collision->empty() ? collision->data() : nullptr;
    request.environment.collision.inflatedBytes =
        collision ? collision->size() : 0U;
    return request;
  }

  RobotState robot{};
  PlanIdentity identity{};
  PlanClock clock{};
  LocalRouteView routeView{};
  EnvironmentView environment{};
  std::optional<LocalMotionIntent> intent;
  std::vector<Vec3> route;
  std::shared_ptr<const std::vector<std::uint8_t>> collision;
};

struct Job {
  std::uint64_t sequence{0};
  std::uint64_t epoch{0};
  OwnedRequest request;
  std::shared_ptr<std::atomic_bool> cancel;
  std::chrono::steady_clock::time_point queuedAt;

  Job(std::uint64_t sequenceValue, std::uint64_t epochValue,
      OwnedRequest ownedRequest, std::shared_ptr<std::atomic_bool> cancelToken)
      : sequence(sequenceValue),
        epoch(epochValue),
        request(std::move(ownedRequest)),
        cancel(std::move(cancelToken)),
        queuedAt(std::chrono::steady_clock::now()) {}

  [[nodiscard]] bool cancelled() const noexcept {
    return cancel && cancel->load(std::memory_order_relaxed);
  }
};

struct Completion {
  std::uint64_t sequence{0};
  std::uint64_t frameEpoch{0};
  std::uint64_t routeGeneration{0};
  std::optional<LocalMotionIntent> intent;
  LocalPlan plan{};
  LocalPlannerDebugSnapshot debug{};
  std::uint64_t collisionGeneration{0};
  std::uint64_t collisionResetEpoch{0};
};

bool sameGuide(const Completion &completion, const LocalPlanRequest &request,
               const std::optional<LocalMotionIntent> &intent) {
  const LocalRouteView *route = request.route();
  return route != nullptr &&
         completion.routeGeneration == route->generation &&
         completion.frameEpoch == request.identity.frameEpoch &&
         sameIntent(completion.intent, intent);
}

}  // namespace

class LocalPlanTask::Impl {
 public:
  explicit Impl(const LocalPlannerParams &params)
      : params_(params), planner_(params) {}

  ~Impl() { stop(); }

  bool configure(const std::string &pathLibraryDir) {
    (void)pathLibraryDir;
    if (configured_)
      return true;
    configured_ = true;
    worker_ = std::thread([this]() { run(); });
    return true;
  }

  bool configured() const { return configured_; }

  LocalPlanUpdate update(const LocalPlanRequest &request) {
    LocalPlanUpdate output;
    output.debug.backend = LocalPlannerBackend::Scan;
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
        request.intent() == nullptr
            ? std::nullopt
            : std::optional<LocalMotionIntent>{*request.intent()};

    if (auto completion = poll(); completion &&
        sameGuide(*completion, request, intent)) {
      latest_ = *completion;
      if (completion->plan.ready()) {
        current_ = std::move(*completion);
      } else if (completion->plan.status() != LocalPlanStatus::Pending) {
        current_.reset();
      }
    }

    submit(request);

    if (current_ && sameGuide(*current_, request, intent)) {
      if (safeOnCurrentMap(*current_, request)) {
        output.plan = current_->plan;
        output.debug = current_->debug;
        return output;
      }
      current_.reset();
    }
    if (latest_ && sameGuide(*latest_, request, intent) &&
        latest_->plan.status() != LocalPlanStatus::Pending) {
      if (!latest_->plan.ready() || safeOnCurrentMap(*latest_, request)) {
        output.plan = latest_->plan;
        output.debug = latest_->debug;
        return output;
      }
      latest_.reset();
    }
    output.plan = LocalPlan::stopped(LocalPlanStatus::Pending);
    return output;
  }

  void reset() {
    current_.reset();
    latest_.reset();
    collisionSnapshot_.reset();
    collisionGeneration_ = 0;
    collisionResetEpoch_ = 0;
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
      completed_.reset();
      resetRequested_ = true;
    }
    cv_.notify_one();
  }

 private:
  bool safeOnCurrentMap(Completion &completion,
                        const LocalPlanRequest &request) const {
    const auto &collision = request.environment.collision;
    if (completion.collisionGeneration == collision.generation &&
        completion.collisionResetEpoch == collision.resetEpoch) {
      return true;
    }
    const auto *target = std::get_if<SplineTarget>(&completion.plan.target());
    if (target == nullptr) {
      return false;
    }
    const scan::Grid grid(params_, request);
    const SplineView spline(*target);
    if (!grid.valid() || !spline.valid()) {
      return false;
    }

    constexpr double kStepS = 0.01;
    const double duration = spline.duration();
    const double current = std::clamp(
        request.clock.timestampS - target->startTimeS, 0.0, duration);
    // The async adapter may keep executing only the prefix that remains clear
    // during SCAN's official one-second emergency horizon. A farther obstacle
    // is handled by the planner worker without blanking a still-safe command.
    constexpr double kEmergencyHorizonS = 1.0;
    const double end = std::min(duration, current + kEmergencyHorizonS);
    for (double time = current; time < end; time += kStepS) {
      const Vec3 position = spline.position(time);
      const Vec3 next = spline.position(std::min(time + kStepS, duration));
      const double yaw = std::atan2(next.y - position.y,
                                    next.x - position.x);
      if (!grid.obstacleFree(position, yaw)) {
        return false;
      }
    }
    completion.collisionGeneration = collision.generation;
    completion.collisionResetEpoch = collision.resetEpoch;
    return true;
  }

  std::shared_ptr<const std::vector<std::uint8_t>> collisionSnapshot(
      const LocalPlanRequest &request) {
    const LocalCollisionMapView &collision = request.environment.collision;
    if (collision.inflatedBits == nullptr || collision.inflatedBytes == 0U)
      return {};
    if (collisionSnapshot_ && collisionGeneration_ == collision.generation &&
        collisionResetEpoch_ == collision.resetEpoch &&
        collisionSnapshot_->size() == collision.inflatedBytes) {
      return collisionSnapshot_;
    }
    auto snapshot = std::make_shared<std::vector<std::uint8_t>>(
        collision.inflatedBits,
        collision.inflatedBits + collision.inflatedBytes);
    collisionGeneration_ = collision.generation;
    collisionResetEpoch_ = collision.resetEpoch;
    collisionSnapshot_ = snapshot;
    return snapshot;
  }

  void submit(const LocalPlanRequest &request) {
    OwnedRequest owned(request, collisionSnapshot(request));
    const LocalRouteView *route = request.route();
    const std::optional<LocalMotionIntent> intent =
        request.intent() == nullptr
            ? std::nullopt
            : std::optional<LocalMotionIntent>{*request.intent()};
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_ && pending_->cancel)
      pending_->cancel->store(true, std::memory_order_relaxed);
    if (activeCancel_ &&
        (activeRouteGeneration_ != (route == nullptr ? 0U : route->generation) ||
         activeFrameEpoch_ != request.identity.frameEpoch ||
         !sameIntent(activeIntent_, intent))) {
      activeCancel_->store(true, std::memory_order_relaxed);
    }
    auto cancel = std::make_shared<std::atomic_bool>(false);
    pending_.emplace(++nextSequence_, epoch_, std::move(owned), cancel);
    cv_.notify_one();
  }

  std::optional<Completion> poll() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto completion = std::move(completed_);
    completed_.reset();
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
        cv_.wait(lock, [this]() {
          return stopping_ || resetRequested_ || pending_.has_value();
        });
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

      LocalPlanRequest request = job->request.view();
      request.clock.timestampS += std::chrono::duration<double>(
          std::chrono::steady_clock::now() - job->queuedAt).count();
      Completion completion;
      completion.sequence = job->sequence;
      completion.frameEpoch = request.identity.frameEpoch;
      completion.routeGeneration = job->request.routeView.generation;
      completion.intent = job->request.intent;
      completion.collisionGeneration =
          request.environment.collision.generation;
      completion.collisionResetEpoch =
          request.environment.collision.resetEpoch;
      try {
        completion.plan = planner_.plan(
            request, [&job]() { return job->cancelled(); });
        completion.debug = planner_.debugSnapshot();
      } catch (const std::exception &error) {
        completion.plan = LocalPlan::stopped(LocalPlanStatus::InvalidInput);
        completion.debug.searchReason =
            std::string("planner_exception:") + error.what();
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
      if (!job->cancelled() && !resetRequested_ && job->epoch == epoch_ &&
          !stopping_) {
        completed_ = std::move(completion);
      }
    }
  }

  LocalPlannerParams params_;
  scan::Backend planner_;
  bool configured_{false};
  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<Job> pending_;
  std::shared_ptr<std::atomic_bool> activeCancel_;
  std::uint64_t activeRouteGeneration_{0};
  std::uint64_t activeFrameEpoch_{0};
  std::optional<LocalMotionIntent> activeIntent_;
  std::optional<Completion> completed_;
  std::uint64_t nextSequence_{0};
  std::uint64_t epoch_{1};
  bool resetRequested_{false};
  bool stopping_{false};
  std::thread worker_;

  std::optional<Completion> current_;
  std::optional<Completion> latest_;
  std::shared_ptr<const std::vector<std::uint8_t>> collisionSnapshot_;
  std::uint64_t collisionGeneration_{0};
  std::uint64_t collisionResetEpoch_{0};
};

LocalPlanTask::LocalPlanTask(const LocalPlannerParams &params)
    : impl_(std::make_unique<Impl>(params)) {}

LocalPlanTask::~LocalPlanTask() = default;

bool LocalPlanTask::configure(const std::string &pathLibraryDir) {
  return impl_->configure(pathLibraryDir);
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
