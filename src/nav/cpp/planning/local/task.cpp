#include "planning/local/task.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "planning/local/scan/backend.hpp"

namespace nav_kernel::local {
namespace {

using Clock = std::chrono::steady_clock;

enum class ScanTimer { Fsm, Collision };

bool sameIntent(const std::optional<LocalMotionIntent> &left,
                const std::optional<LocalMotionIntent> &right) {
  if (left.has_value() != right.has_value())
    return false;
  if (!left)
    return true;
  return std::abs(left->speedNormalized - right->speedNormalized) <= 0.05 &&
         std::abs(left->maxDirectionDeviationDeg - right->maxDirectionDeviationDeg) <= 1e-6;
}

struct OwnedRequest {
  void assign(const LocalPlanRequest &source,
              std::shared_ptr<const std::vector<std::uint8_t>> collisionBits) {
    robot = source.robot;
    identity = source.identity;
    clock = source.clock;
    environment = source.environment;
    collision = std::move(collisionBits);
    if (const LocalRouteView *sourceRoute = source.route()) {
      const bool routeChanged = routeView.generation != sourceRoute->generation ||
                                routeView.count != sourceRoute->count ||
                                routeView.reachesGoal != sourceRoute->reachesGoal ||
                                sourceRoute->points == nullptr ||
                                !std::equal(route.begin(), route.end(), sourceRoute->points,
                                            sourceRoute->points + sourceRoute->count,
                                            [](const Vec3 &left, const Vec3 &right) {
                                              return left.x == right.x && left.y == right.y &&
                                                     left.z == right.z;
                                            });
      routeView = *sourceRoute;
      if (routeChanged && sourceRoute->points != nullptr && sourceRoute->count > 0) {
        route.assign(sourceRoute->points, sourceRoute->points + sourceRoute->count);
      }
    } else {
      routeView = {};
      route.clear();
    }
    if (const LocalMotionIntent *sourceIntent = source.intent()) {
      intent = *sourceIntent;
    } else {
      intent.reset();
    }
  }

  LocalPlanRequest view() const {
    LocalPlanRequest request;
    request.robot = robot;
    request.identity = identity;
    request.clock = clock;

    LocalRouteView routeViewCopy = routeView;
    routeViewCopy.points = route.empty() ? nullptr : route.data();
    routeViewCopy.count = static_cast<int>(route.size());
    request.objective = intent ? LocalObjective{MotionIntentTarget{*intent, routeViewCopy}}
                               : LocalObjective{RouteTarget{routeViewCopy}};

    request.environment = environment;
    request.environment.obstacles = {};
    request.environment.traversability = {};
    request.environment.collision.inflatedBits =
        collision && !collision->empty() ? collision->data() : nullptr;
    request.environment.collision.inflatedBytes = collision ? collision->size() : 0U;
    request.environment.collision.inflatedStorage = collision;
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

struct InputSnapshot {
  void assign(std::uint64_t epochValue, const LocalPlanRequest &source,
              std::shared_ptr<const std::vector<std::uint8_t>> collision) {
    epoch = epochValue;
    request.assign(source, std::move(collision));
    receivedAt = Clock::now();
  }

  void assign(const InputSnapshot &source) {
    epoch = source.epoch;
    request.assign(source.request.view(), source.request.collision);
    receivedAt = source.receivedAt;
  }

  LocalPlanRequest view(Clock::time_point now) const {
    LocalPlanRequest output = request.view();
    output.clock.timestampS += std::chrono::duration<double>(now - receivedAt).count();
    return output;
  }

  std::uint64_t epoch{0};
  OwnedRequest request;
  Clock::time_point receivedAt;
};

struct Completion {
  std::uint64_t epoch{0};
  std::uint64_t frameEpoch{0};
  std::uint64_t routeGeneration{0};
  std::optional<LocalMotionIntent> intent;
  LocalPlan plan{};
  LocalPlannerDebugSnapshot debug{};
};

struct PublishedPlan {
  std::uint64_t epoch{0};
  std::uint64_t frameEpoch{0};
  std::uint64_t routeGeneration{0};
  std::optional<LocalMotionIntent> intent;
  LocalPlanStatus status{LocalPlanStatus::Pending};
  std::int64_t trajectoryId{0};
  int slowdownLevel{0};
  bool retainRouteGuide{false};
};

struct WorkerDebug {
  std::uint64_t frameEpoch{0};
  std::uint64_t routeGeneration{0};
  std::optional<LocalMotionIntent> intent;
  LocalPlannerDebugSnapshot debug{};
};

struct PollResult {
  std::optional<Completion> completion;
  std::optional<WorkerDebug> debug;
};

bool sameGuide(std::uint64_t routeGeneration, std::uint64_t frameEpoch,
               const std::optional<LocalMotionIntent> &intent, const LocalPlanRequest &request) {
  const LocalRouteView *route = request.route();
  const std::optional<LocalMotionIntent> requestIntent =
      request.intent() == nullptr ? std::nullopt
                                  : std::optional<LocalMotionIntent>{*request.intent()};
  return route != nullptr && routeGeneration == route->generation &&
         frameEpoch == request.identity.frameEpoch && sameIntent(intent, requestIntent);
}

bool sameGuide(const Completion &completion, const LocalPlanRequest &request) {
  return sameGuide(completion.routeGeneration, completion.frameEpoch, completion.intent, request);
}

bool sameGuide(const WorkerDebug &debug, const LocalPlanRequest &request) {
  return sameGuide(debug.routeGeneration, debug.frameEpoch, debug.intent, request);
}

PublishedPlan publicationOf(const Completion &completion) {
  PublishedPlan result;
  result.epoch = completion.epoch;
  result.frameEpoch = completion.frameEpoch;
  result.routeGeneration = completion.routeGeneration;
  result.intent = completion.intent;
  result.status = completion.plan.status();
  result.slowdownLevel = completion.plan.hints().slowdownLevel;
  result.retainRouteGuide = completion.plan.hints().retainRouteGuide;
  if (const auto *spline = std::get_if<SplineTarget>(&completion.plan.target()))
    result.trajectoryId = spline->trajectoryId;
  return result;
}

bool samePublication(const PublishedPlan &left, const PublishedPlan &right) {
  return left.epoch == right.epoch && left.frameEpoch == right.frameEpoch &&
         left.routeGeneration == right.routeGeneration && sameIntent(left.intent, right.intent) &&
         left.status == right.status && left.trajectoryId == right.trajectoryId &&
         left.slowdownLevel == right.slowdownLevel &&
         left.retainRouteGuide == right.retainRouteGuide;
}

Clock::duration timerPeriod(double seconds) {
  return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
}

void advanceTimer(Clock::time_point &deadline, Clock::duration period, Clock::time_point now) {
  do {
    deadline += period;
  } while (deadline <= now);
}

}  // namespace

class LocalPlanTask::Impl {
 public:
  explicit Impl(const LocalPlannerParams &params) : planner_(params) {}

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

    const PollResult polled = poll();
    if (polled.completion && sameGuide(*polled.completion, request)) {
      latest_ = *polled.completion;
      if (polled.completion->plan.ready()) {
        current_ = *polled.completion;
      } else if (polled.completion->plan.status() != LocalPlanStatus::Pending) {
        current_.reset();
      }
    }
    const LocalPlannerDebugSnapshot *workerDebug =
        polled.debug && sameGuide(*polled.debug, request) ? &polled.debug->debug : nullptr;

    if (current_ && !sameGuide(*current_, request))
      current_.reset();
    if (latest_ && !sameGuide(*latest_, request))
      latest_.reset();

    publish(request);

    if (current_) {
      output.plan = current_->plan;
      output.debug = workerDebug != nullptr ? *workerDebug : current_->debug;
      return output;
    }
    if (latest_ && latest_->plan.status() != LocalPlanStatus::Pending) {
      output.plan = latest_->plan;
      output.debug = workerDebug != nullptr ? *workerDebug : latest_->debug;
      return output;
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
      cancelGeneration_.fetch_add(1U, std::memory_order_relaxed);
      hasInput_ = false;
      completed_.reset();
      workerDebug_.reset();
      publishedPlan_.reset();
      timersArmed_ = false;
      resetRequested_ = true;
    }
    cv_.notify_one();
  }

 private:
  std::shared_ptr<const std::vector<std::uint8_t>>
  collisionSnapshot(const LocalPlanRequest &request) {
    const LocalCollisionMapView &collision = request.environment.collision;
    if (collision.inflatedBits == nullptr || collision.inflatedBytes == 0U)
      return {};
    if (collision.inflatedStorage &&
        collision.inflatedStorage->data() == collision.inflatedBits &&
        collision.inflatedStorage->size() == collision.inflatedBytes) {
      collisionGeneration_ = collision.generation;
      collisionResetEpoch_ = collision.resetEpoch;
      collisionSnapshot_ = collision.inflatedStorage;
      return collisionSnapshot_;
    }
    if (collisionSnapshot_ && collisionGeneration_ == collision.generation &&
        collisionResetEpoch_ == collision.resetEpoch &&
        collisionSnapshot_->size() == collision.inflatedBytes) {
      return collisionSnapshot_;
    }
    auto snapshot = std::make_shared<std::vector<std::uint8_t>>(
        collision.inflatedBits, collision.inflatedBits + collision.inflatedBytes);
    collisionGeneration_ = collision.generation;
    collisionResetEpoch_ = collision.resetEpoch;
    collisionSnapshot_ = snapshot;
    return snapshot;
  }

  void publish(const LocalPlanRequest &request) {
    const auto collision = collisionSnapshot(request);
    bool wakeWorker = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool guideChanged =
          hasInput_ && !sameGuide(input_.request.routeView.generation,
                                  input_.request.identity.frameEpoch,
                                  input_.request.intent, request);
      if (guideChanged) {
        cancelGeneration_.fetch_add(1U, std::memory_order_relaxed);
        publishedPlan_.reset();
      }
      input_.assign(epoch_, request, collision);
      hasInput_ = true;
      if (!timersArmed_) {
        const Clock::time_point now = Clock::now();
        nextFsmTick_ = now;
        nextCollisionTick_ = now;
        timersArmed_ = true;
        wakeWorker = true;
      }
    }
    if (wakeWorker)
      cv_.notify_one();
  }

  PollResult poll() {
    std::lock_guard<std::mutex> lock(mutex_);
    PollResult result;
    result.completion = std::move(completed_);
    result.debug = workerDebug_;
    completed_.reset();
    return result;
  }

  void stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_)
        return;
      stopping_ = true;
      cancelGeneration_.fetch_add(1U, std::memory_order_relaxed);
      hasInput_ = false;
    }
    cv_.notify_one();
    if (worker_.joinable())
      worker_.join();
  }

  void run() {
    const Clock::duration fsmPeriod = timerPeriod(scan::Backend::fsmPeriodS());
    const Clock::duration collisionPeriod =
        timerPeriod(scan::Backend::collisionPeriodS());
    InputSnapshot snapshot;

    for (;;) {
      std::uint64_t cancelGeneration = 0U;
      ScanTimer timer = ScanTimer::Fsm;
      bool reset = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        for (;;) {
          if (stopping_)
            return;
          if (resetRequested_) {
            resetRequested_ = false;
            reset = true;
            break;
          }
          if (!hasInput_ || !timersArmed_) {
            cv_.wait(lock);
            continue;
          }

          const Clock::time_point now = Clock::now();
          const bool fsmDue = now >= nextFsmTick_;
          const bool collisionDue = now >= nextCollisionTick_;
          if (!fsmDue && !collisionDue) {
            cv_.wait_until(lock, std::min(nextFsmTick_, nextCollisionTick_));
            continue;
          }

          timer = collisionDue && (!fsmDue || nextCollisionTick_ < nextFsmTick_)
                      ? ScanTimer::Collision
                      : ScanTimer::Fsm;
          if (timer == ScanTimer::Fsm)
            advanceTimer(nextFsmTick_, fsmPeriod, now);
          else
            advanceTimer(nextCollisionTick_, collisionPeriod, now);

          snapshot.assign(input_);
          cancelGeneration = cancelGeneration_.load(std::memory_order_relaxed);
          break;
        }
      }

      if (reset) {
        planner_.reset();
        continue;
      }
      LocalPlanRequest request = snapshot.view(Clock::now());
      Completion completion;
      completion.epoch = snapshot.epoch;
      completion.frameEpoch = request.identity.frameEpoch;
      completion.routeGeneration = snapshot.request.routeView.generation;
      completion.intent = snapshot.request.intent;
      try {
        const LocalPlanCancel isCancelled = [this, cancelGeneration]() {
          return cancelGeneration_.load(std::memory_order_relaxed) != cancelGeneration;
        };
        completion.plan = timer == ScanTimer::Fsm ? planner_.tick(request, isCancelled)
                                                  : planner_.checkCollision(request, isCancelled);
        completion.debug = planner_.debugSnapshot();
      } catch (const std::exception &error) {
        completion.plan = LocalPlan::stopped(LocalPlanStatus::InvalidInput);
        completion.debug.searchReason = std::string("planner_exception:") + error.what();
      } catch (...) {
        completion.plan = LocalPlan::stopped(LocalPlanStatus::InvalidInput);
        completion.debug.searchReason = "planner_exception";
      }

      std::lock_guard<std::mutex> lock(mutex_);
      if (cancelGeneration_.load(std::memory_order_relaxed) == cancelGeneration &&
          !resetRequested_ && completion.epoch == epoch_ && !stopping_ && hasInput_ &&
          input_.epoch == completion.epoch &&
          sameGuide(completion.routeGeneration, completion.frameEpoch, completion.intent,
                    input_.request.view())) {
        workerDebug_ = WorkerDebug{completion.frameEpoch, completion.routeGeneration,
                                   completion.intent, completion.debug};
        const PublishedPlan publication = publicationOf(completion);
        if (!publishedPlan_ || !samePublication(*publishedPlan_, publication)) {
          publishedPlan_ = publication;
          completed_ = std::move(completion);
        }
      }
    }
  }

  scan::Backend planner_;
  bool configured_{false};
  std::mutex mutex_;
  std::condition_variable cv_;
  InputSnapshot input_;
  bool hasInput_{false};
  std::atomic<std::uint64_t> cancelGeneration_{1U};
  std::optional<Completion> completed_;
  std::optional<WorkerDebug> workerDebug_;
  std::optional<PublishedPlan> publishedPlan_;
  std::uint64_t epoch_{1};
  bool resetRequested_{false};
  bool stopping_{false};
  bool timersArmed_{false};
  Clock::time_point nextFsmTick_{};
  Clock::time_point nextCollisionTick_{};
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
