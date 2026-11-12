#include "planning/local/scan/task.hpp"

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
#include "planning/local/scan/upstream/plan_manage/plan_container.hpp"

namespace nav_kernel::local::scan {
namespace {

using Clock = std::chrono::steady_clock;

enum class Work { Input, Fsm, Collision };

bool sameIntent(const std::optional<LocalMotionIntent> &left,
                const std::optional<LocalMotionIntent> &right) {
  if (left.has_value() != right.has_value())
    return false;
  if (!left)
    return true;
  const bool leftMoving = left->speedNormalized > 1e-6;
  const bool rightMoving = right->speedNormalized > 1e-6;
  return leftMoving == rightMoving &&
         std::abs(left->directionBodyDeg - right->directionBodyDeg) <= 1e-6 &&
         std::abs(left->horizonM - right->horizonM) <= 1e-6 &&
         std::abs(left->maxDirectionDeviationDeg - right->maxDirectionDeviationDeg) <= 1e-6;
}

struct Input {
  void assign(const LocalPlanRequest &source,
              std::shared_ptr<const std::vector<std::uint8_t>> collision,
              bool replaceGuide) {
    LocalPlanRequest next = source;
    if (replaceGuide) {
      const auto &reference = *source.referenceRoute();
      route = std::make_shared<const std::vector<Vec3>>(
          reference.points, reference.points + reference.count);
      next.reference = {route->data(), reference.count, reference.generation};
      next.objective = source.intent()
          ? LocalObjective{MotionIntentTarget{*source.intent(), next.reference}}
          : LocalObjective{RouteTarget{next.reference}};
    } else {
      next.objective = request.objective;
      next.reference = request.reference;
    }
    next.environment.obstacles = {};
    next.environment.traversability = {};
    const auto &predicted = source.environment.predictions;
    predictions.clear();
    if (predicted.count > 0 && predicted.fresh(source.clock.timestampS))
      predictions.assign(predicted.obstacles, predicted.obstacles + predicted.count);
    next.environment.predictions.obstacles = nullptr;
    auto &map = next.environment.collision;
    map.inflatedStorage = std::move(collision);
    map.inflatedBits = map.inflatedStorage ? map.inflatedStorage->data() : nullptr;
    map.inflatedBytes = map.inflatedStorage ? map.inflatedStorage->size() : 0U;
    request = std::move(next);
    receivedAt = Clock::now();
  }

  double time(Clock::time_point now) const {
    return request.clock.afterElapsed(
        std::chrono::duration<double>(now - receivedAt).count());
  }

  LocalPlanRequest view(Clock::time_point now) const {
    LocalPlanRequest output = request;
    // Copying Input shares the route/map, but gives predictions new storage.
    output.environment.predictions.obstacles = predictions.data();
    output.environment.predictions.count = predictions.size();
    output.clock.timestampS = time(now);
    return output;
  }

  LocalPlanRequest request;
  std::shared_ptr<const std::vector<Vec3>> route;
  std::vector<PredictedObstacle> predictions;
  Clock::time_point receivedAt;
};

struct Guide {
  std::uint64_t frameEpoch{0};
  std::uint64_t routeGeneration{0};
  std::optional<LocalMotionIntent> intent;
  std::optional<double> bodyHeading;
  std::shared_ptr<const lingtu::nav::collision::Body> body;
};

Guide guideOf(const LocalPlanRequest &request) {
  const auto *intent = request.intent();
  return {request.identity.frameEpoch, request.referenceRoute()->generation,
          intent ? std::optional<LocalMotionIntent>{*intent} : std::nullopt,
          request.bodyHeading, request.body};
}

bool sameFrameAndBody(const Guide &left, const Guide &right) {
  return left.frameEpoch == right.frameEpoch &&
         left.bodyHeading == right.bodyHeading && left.body == right.body;
}

bool sameGuide(const Guide &left, const Guide &right) {
  return sameFrameAndBody(left, right) &&
         left.routeGeneration == right.routeGeneration &&
         sameIntent(left.intent, right.intent);
}

struct Completion {
  Guide guide;
  LocalPlan plan{};
  LocalPlannerDebugSnapshot debug{};
  std::shared_ptr<upstream::TrajectoryHandoff> handoff;
};

struct Publication {
  Guide guide;
  LocalPlanStatus status{LocalPlanStatus::Pending};
  std::int64_t trajectoryId{0};
  int slowdownLevel{0};
  bool retainRouteGuide{false};
  std::shared_ptr<upstream::TrajectoryHandoff> handoff;
};

struct WorkerDebug {
  Guide guide;
  LocalPlannerDebugSnapshot debug{};
};

struct PollResult {
  std::optional<Completion> completion;
  std::optional<WorkerDebug> debug;
};

Publication publicationOf(const Completion &completion) {
  Publication result;
  result.guide = completion.guide;
  result.status = completion.plan.status();
  result.slowdownLevel = completion.plan.hints().slowdownLevel;
  result.retainRouteGuide = completion.plan.hints().retainRouteGuide;
  result.handoff = completion.handoff;
  if (const auto *spline = std::get_if<SplineTarget>(&completion.plan.target()))
    result.trajectoryId = spline->trajectoryId;
  return result;
}

bool samePublication(const Publication &left, const Publication &right) {
  return sameFrameAndBody(left.guide, right.guide) &&
         left.handoff == right.handoff && left.status == right.status &&
         left.trajectoryId == right.trajectoryId &&
         left.slowdownLevel == right.slowdownLevel &&
         left.retainRouteGuide == right.retainRouteGuide &&
         (left.trajectoryId != 0 || sameGuide(left.guide, right.guide));
}

Clock::duration timerPeriod(double seconds) {
  return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
}

void advanceTimer(Clock::time_point &deadline, Clock::duration period, Clock::time_point now) {
  deadline += period;
  // Match roscpp TimerManager::updateNext(): retain one pending callback for a
  // short overrun, but reset the phase after a delay longer than two periods.
  if (deadline + period < now)
    deadline = now;
}

}  // namespace

class Task::Impl {
 public:
  explicit Impl(const LocalPlannerParams &params)
      : backend_(params), configuredSpeedLimit_(std::max(0.05, params.scan.maxVelocity)) {
    backend_.setExecutionFeedback([this] {
      std::lock_guard<std::mutex> lock(mutex_);
      return ExecutionFeedback{input_.time(Clock::now()), executingTrajectoryId_,
                                executionStartTimeS_, input_.request.clock.executionFrozen,
                                input_.request.execution};
    });
  }

  ~Impl() { stop(); }

  bool configure() {
    if (worker_.joinable())
      return true;
    const Clock::time_point now = Clock::now();
    nextFsmTick_ = now + timerPeriod(Backend::fsmPeriodS());
    nextCollisionTick_ = now + timerPeriod(Backend::collisionPeriodS());
    worker_ = std::thread([this]() { run(); });
    return true;
  }

  std::shared_ptr<const ScanFailureSnapshot> lastScanFailure() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lastScanFailure_;
  }

  Update update(const LocalPlanRequest &request) {
    const auto receivedAt = Clock::now();
    Update output;
    output.debug.backend = LocalPlannerBackend::Scan;
    if (!worker_.joinable()) {
      output.plan = LocalPlan::stopped(LocalPlanStatus::NotConfigured);
      return output;
    }
    const LocalRouteView *route = request.referenceRoute();
    if (route == nullptr || !route->valid() ||
        (request.bodyHeading && !std::isfinite(*request.bodyHeading))) {
      output.plan = LocalPlan::stopped(LocalPlanStatus::InvalidInput);
      return output;
    }

    // Initialization depends on metadata availability, not its numeric values.
    if (hasSupportHeight_ != request.supportHeightM.has_value() ||
        hasSupportTolerance_ != request.supportHeightToleranceM.has_value()) {
      reset();
      hasSupportHeight_ = request.supportHeightM.has_value();
      hasSupportTolerance_ = request.supportHeightToleranceM.has_value();
    }
    const Guide guide = guideOf(request);
    if (accepted_) {
      // Keep an executing curve while a new route is being planned, but not an old stop.
      const bool matches = accepted_->plan.ready()
          ? sameFrameAndBody(accepted_->guide, guide)
          : sameGuide(accepted_->guide, guide);
      if (!matches) accepted_.reset();
    }
    if (waiting_ && effectiveSpeedLimit(request.maxLinearSpeedMps) <
                        std::get<SplineTarget>(waiting_->plan.target()).maxLinearSpeedMps) {
      waiting_->handoff->withdraw();
      waiting_.reset();
    }
    // Publish before polling so a lower cap can discard a queued old offer
    // before that offer is acknowledged by this physical control tick.
    publish(request, guide);
    const PollResult polled = poll();
    const auto accept = [&](const Completion &completion) {
      accepted_ = completion;
      if (!completion.plan.ready()) return;
      const auto id = std::get<SplineTarget>(accepted_->plan.target()).trajectoryId;
      if (!completion.handoff &&
          (executingTrajectoryId_ == 0 ||
           (request.clock.executionFrozen && id != executingTrajectoryId_))) {
        auto spline = std::get<SplineTarget>(accepted_->plan.target());
        spline.startTimeS = request.clock.timestampS;
        accepted_->plan = LocalPlan::spline(std::move(spline), accepted_->plan.hints());
      }
      const auto &spline = std::get<SplineTarget>(accepted_->plan.target());
      executingTrajectoryId_ = spline.trajectoryId;
      executionStartTimeS_ = spline.startTimeS;
    };
    if (polled.completion &&
        sameGuide(polled.completion->guide, guide)) {
      const auto *spline = std::get_if<SplineTarget>(&polled.completion->plan.target());
      if (polled.completion->handoff) {
        // Receiving a future offer does not acknowledge execution. The next
        // physical control tick at its start time owns that acknowledgement.
        if (spline && spline->trajectoryId > executingTrajectoryId_)
          waiting_ = *polled.completion;
      } else if (!spline || spline->trajectoryId >= executingTrajectoryId_) {
        // Returning to the old publication explicitly withdraws a pending
        // offer; samePublication sees its different trajectory ID.
        waiting_.reset();
        std::lock_guard<std::mutex> lock(mutex_);
        accept(*polled.completion);
      }
    }
    if (waiting_) {
      std::lock_guard<std::mutex> lock(mutex_);
      const double nowS = request.clock.afterElapsed(
          std::chrono::duration<double>(Clock::now() - receivedAt).count());
      const auto &spline = std::get<SplineTarget>(waiting_->plan.target());
      if (!sameGuide(waiting_->guide, guide) ||
          request.clock.executionFrozen || nowS > waiting_->handoff->deadlineS) {
        waiting_->handoff->withdraw();
        waiting_.reset();
      } else if (nowS >= spline.startTimeS) {
        if (waiting_->handoff->receive()) accept(*waiting_);
        waiting_.reset();
      }
    }
    const LocalPlannerDebugSnapshot *workerDebug =
        polled.debug && sameGuide(polled.debug->guide, guide) ? &polled.debug->debug : nullptr;

    if (accepted_ && accepted_->plan.status() != LocalPlanStatus::Pending) {
      output.plan = accepted_->plan;
      output.debug = workerDebug != nullptr ? *workerDebug : accepted_->debug;
      return output;
    }
    output.plan = LocalPlan::stopped(LocalPlanStatus::Pending);
    if (workerDebug != nullptr)
      output.debug = *workerDebug;
    return output;
  }

  void pause() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hasInput_)
      input_.request.clock.executionFrozen = true;
  }

  void reset() {
    accepted_.reset();
    waiting_.reset();
    collisionSnapshot_.reset();
    collisionGeneration_ = 0;
    collisionResetEpoch_ = 0;
    if (!worker_.joinable())
      return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Repeated idle stops must preserve a pending reset without rebuilding
      // the core again. New input re-arms reset through publish().
      if (!hasInput_)
        return;
      cancelGeneration_.fetch_add(1U, std::memory_order_relaxed);
      hasInput_ = false;
      executingTrajectoryId_ = 0;
      completed_.reset();
      workerDebug_.reset();
      publication_.reset();
      inputEventPending_ = false;
      const Clock::time_point now = Clock::now();
      nextFsmTick_ = now + timerPeriod(Backend::fsmPeriodS());
      nextCollisionTick_ = now + timerPeriod(Backend::collisionPeriodS());
      resetRequested_ = true;
    }
    cv_.notify_one();
  }

 private:
  double effectiveSpeedLimit(double requested) const {
    return requested > 0.0 ? std::min(configuredSpeedLimit_, requested)
                           : configuredSpeedLimit_;
  }

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

  void publish(const LocalPlanRequest &request, const Guide &guide) {
    const auto collision = collisionSnapshot(request);
    bool wakeWorker = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool guideChanged = !hasInput_ || !sameGuide(guideOf(input_.request), guide);
      const bool speedReduced = hasInput_ &&
          effectiveSpeedLimit(request.maxLinearSpeedMps) + 1e-9 <
              effectiveSpeedLimit(input_.request.maxLinearSpeedMps);
      // Coalesce speed updates until the worker takes the latest snapshot.
      // Cancel an in-flight solve, without resetting its FSM or active curve.
      if (guideChanged || (speedReduced && !inputEventPending_))
        cancelGeneration_.fetch_add(1U, std::memory_order_relaxed);
      if (speedReduced) {
        if (completed_ && completed_->handoff) completed_->handoff->withdraw();
        completed_.reset();
        publication_.reset();
      }
      input_.assign(request, collision, guideChanged);
      if (accepted_ && accepted_->plan.ready()) {
        const auto &spline = std::get<SplineTarget>(accepted_->plan.target());
        executingTrajectoryId_ = spline.trajectoryId;
        executionStartTimeS_ = spline.startTimeS;
      } else {
        executingTrajectoryId_ = 0;
      }
      hasInput_ = true;
      // The official /initial_path subscriber has queue size one and runs in
      // the same callback queue as the timers. Preserve that latest-only,
      // serialized input event instead of waiting for the next FSM timer.
      inputEventPending_ = inputEventPending_ || guideChanged || speedReduced;
      wakeWorker = guideChanged || speedReduced;
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
      stopping_ = true;
      cancelGeneration_.fetch_add(1U, std::memory_order_relaxed);
      hasInput_ = false;
    }
    cv_.notify_one();
    if (worker_.joinable())
      worker_.join();
  }

  void run() {
    const Clock::duration fsmPeriod = timerPeriod(Backend::fsmPeriodS());
    const Clock::duration collisionPeriod =
        timerPeriod(Backend::collisionPeriodS());
    Input snapshot;

    for (;;) {
      std::uint64_t cancelGeneration = 0U;
      Work work = Work::Fsm;
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
          if (!hasInput_) {
            cv_.wait(lock);
            continue;
          }

          if (inputEventPending_) {
            inputEventPending_ = false;
            work = Work::Input;
            snapshot = input_;
            cancelGeneration =
                cancelGeneration_.load(std::memory_order_relaxed);
            break;
          }

          const Clock::time_point now = Clock::now();
          const bool fsmDue = now >= nextFsmTick_;
          const bool collisionDue = now >= nextCollisionTick_;
          if (!fsmDue && !collisionDue) {
            cv_.wait_until(lock, std::min(nextFsmTick_, nextCollisionTick_));
            continue;
          }

          work = collisionDue && (!fsmDue || nextCollisionTick_ < nextFsmTick_)
                     ? Work::Collision
                     : Work::Fsm;
          if (work == Work::Fsm)
            advanceTimer(nextFsmTick_, fsmPeriod, now);
          else
            advanceTimer(nextCollisionTick_, collisionPeriod, now);

          snapshot = input_;
          cancelGeneration = cancelGeneration_.load(std::memory_order_relaxed);
          break;
        }
      }

      if (reset) {
        backend_.reset();
        continue;
      }
      LocalPlanRequest request = snapshot.view(Clock::now());
      Completion completion;
      completion.guide = guideOf(request);
      try {
        const LocalPlanCancel isCancelled = [this, cancelGeneration]() {
          return cancelGeneration_.load(std::memory_order_relaxed) != cancelGeneration;
        };
        completion.plan = work == Work::Collision
                              ? backend_.checkCollision(request, isCancelled)
                              : backend_.tick(request, isCancelled);
        completion.debug = backend_.debugSnapshot();
        completion.handoff = backend_.handoff();
      } catch (const std::exception &error) {
        completion.plan = LocalPlan::stopped(LocalPlanStatus::InvalidInput);
        completion.debug.searchReason = std::string("planner_exception:") + error.what();
      } catch (...) {
        completion.plan = LocalPlan::stopped(LocalPlanStatus::InvalidInput);
        completion.debug.searchReason = "planner_exception";
      }

      std::lock_guard<std::mutex> lock(mutex_);
      if (cancelGeneration_.load(std::memory_order_relaxed) == cancelGeneration &&
          !resetRequested_ && !stopping_ && hasInput_) {
        if (completion.debug.lastScanFailure)
          lastScanFailure_ = completion.debug.lastScanFailure;
        workerDebug_ = WorkerDebug{completion.guide, completion.debug};
        const Publication publication = publicationOf(completion);
        if (!publication_ || !samePublication(*publication_, publication)) {
          publication_ = publication;
          completed_ = std::move(completion);
        }
      }
    }
  }

  Backend backend_;
  const double configuredSpeedLimit_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  Input input_;
  bool hasInput_{false};
  std::int64_t executingTrajectoryId_{0};
  double executionStartTimeS_{0.0};
  std::atomic<std::uint64_t> cancelGeneration_{1U};
  std::optional<Completion> completed_;
  std::optional<WorkerDebug> workerDebug_;
  std::shared_ptr<const ScanFailureSnapshot> lastScanFailure_;
  std::optional<Publication> publication_;
  bool resetRequested_{false};
  bool stopping_{false};
  bool inputEventPending_{false};
  Clock::time_point nextFsmTick_{};
  Clock::time_point nextCollisionTick_{};
  std::thread worker_;

  // Control-thread state; the worker reads the locked execution feedback above.
  std::optional<Completion> accepted_;
  std::optional<Completion> waiting_;
  std::shared_ptr<const std::vector<std::uint8_t>> collisionSnapshot_;
  std::uint64_t collisionGeneration_{0};
  std::uint64_t collisionResetEpoch_{0};
  bool hasSupportHeight_{false};
  bool hasSupportTolerance_{false};
};

Task::Task(const LocalPlannerParams &params)
    : impl_(std::make_unique<Impl>(params)) {}

Task::~Task() = default;

bool Task::configure() {
  return impl_->configure();
}

Update Task::update(const LocalPlanRequest &request) {
  return impl_->update(request);
}

std::shared_ptr<const ScanFailureSnapshot> Task::lastScanFailure() const {
  return impl_->lastScanFailure();
}

void Task::reset() {
  impl_->reset();
}

void Task::pause() {
  impl_->pause();
}

}  // namespace nav_kernel::local::scan
