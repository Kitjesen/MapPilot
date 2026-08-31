#include "runtime/goal/task.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <utility>

namespace lingtu::nav::endpoint {

std::string
globalPlanStaleReason(const GlobalPlanCompletion &completion, std::uint64_t current_goal_epoch,
                      std::uint64_t current_frame_epoch,
                      const std::optional<lingtu::nav::plan::MapIdentity> &current_map) {
  if (completion.context.goal_epoch != current_goal_epoch) {
    return "goal_superseded_during_planning";
  }
  if (completion.context.frame_epoch != current_frame_epoch) {
    return "planning_frame_changed_during_planning";
  }
  if (!completion.error.empty() || completion.result.cancelled) {
    return {};
  }
  const auto &overlay = completion.context.request.temporary_overlay;
  const bool result_has_overlay_identity =
      completion.result.overlay_revision != 0U || completion.result.overlay_frame_epoch != 0U ||
      completion.result.overlay_obstacle_generation != 0U ||
      completion.result.overlay_traversability_generation != 0U;
  if (overlay.empty()) {
    if (result_has_overlay_identity) {
      return "unexpected_temporary_overlay_identity";
    }
  } else if (completion.result.overlay_revision != overlay.revision ||
             completion.result.overlay_frame_epoch != overlay.frame_epoch ||
             completion.result.overlay_obstacle_generation != overlay.obstacle_generation ||
             completion.result.overlay_traversability_generation !=
                 overlay.traversability_generation) {
    return "temporary_overlay_identity_mismatch";
  }
  if (!completion.result.map_identity.valid()) {
    return "planner_map_identity_missing";
  }
  if (!current_map || !current_map->valid()) {
    return "active_map_unavailable_after_planning";
  }
  if (!lingtu::nav::plan::sameMapIdentity(completion.result.map_identity, *current_map)) {
    return "active_map_changed_during_planning";
  }
  return {};
}

std::vector<nav_kernel::Vec3> globalPlanPath(const GlobalPlanCompletion &completion) {
  std::vector<nav_kernel::Vec3> path;
  path.reserve(completion.result.path.size());
  for (const auto &point : completion.result.path) {
    path.push_back({point.x, point.y, point.z});
  }
  if (path.size() != 1U) {
    return path;
  }
  const auto &start = completion.context.start;
  const auto &point = path.front();
  const double dx = start.x - point.x;
  const double dy = start.y - point.y;
  const double dz = start.z - point.z;
  if (std::sqrt(dx * dx + dy * dy + dz * dz) > 0.02) {
    path.insert(path.begin(), start);
  } else {
    path.push_back(completion.context.goal);
  }
  return path;
}

GlobalPlanTask::GlobalPlanTask(Planner planner) : planner_(std::move(planner)) {
  if (!planner_) {
    throw std::invalid_argument("GlobalPlanTask requires a planner implementation");
  }
}

GlobalPlanTask::~GlobalPlanTask() {
  if (future_.valid()) {
    if (cancel_requested_) {
      cancel_requested_->store(true, std::memory_order_relaxed);
    }
    future_.wait();
  }
}

bool GlobalPlanTask::start(GlobalPlanContext context) {
  if (busy()) {
    return false;
  }
  context_ = std::move(context);
  discard_result_ = false;
  cancel_requested_ = std::make_shared<std::atomic_bool>(false);
  const auto request = context_->request;
  const auto cancel_requested = cancel_requested_;
  future_ = std::async(std::launch::async, [planner = planner_, request, cancel_requested]() {
    return planner(request, [cancel_requested]() {
      return cancel_requested->load(std::memory_order_relaxed);
    });
  });
  return true;
}

std::optional<GlobalPlanCompletion> GlobalPlanTask::poll() {
  if (!future_.valid() || !context_) {
    return std::nullopt;
  }
  if (future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    return std::nullopt;
  }

  GlobalPlanCompletion completion;
  completion.context = std::move(*context_);
  context_.reset();
  cancel_requested_.reset();
  try {
    completion.result = future_.get();
  } catch (const std::exception &exc) {
    completion.error = exc.what();
  } catch (...) {
    completion.error = "unknown planner exception";
  }
  if (discard_result_) {
    discard_result_ = false;
    return std::nullopt;
  }
  return completion;
}

void GlobalPlanTask::cancel() {
  if (future_.valid()) {
    discard_result_ = true;
    if (cancel_requested_) {
      cancel_requested_->store(true, std::memory_order_relaxed);
    }
  }
}

bool GlobalPlanTask::busy() const {
  return future_.valid();
}

}  // namespace lingtu::nav::endpoint
