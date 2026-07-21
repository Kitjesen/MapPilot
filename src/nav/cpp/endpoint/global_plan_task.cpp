#include "global_plan_task.hpp"

#include <chrono>
#include <exception>
#include <stdexcept>
#include <utility>

namespace lingtu::nav::endpoint {

std::string globalPlanStaleReason(
    const GlobalPlanCompletion& completion,
    std::uint64_t current_goal_epoch,
    std::uint64_t current_frame_epoch,
    const std::optional<lingtu::nav::plan::MapIdentity>& current_map) {
  if (completion.context.goal_epoch != current_goal_epoch) {
    return "goal_superseded_during_planning";
  }
  if (completion.context.frame_epoch != current_frame_epoch) {
    return "planning_frame_changed_during_planning";
  }
  if (!completion.error.empty() || completion.result.cancelled) {
    return {};
  }
  if (!completion.result.map_identity.valid()) {
    return "planner_map_identity_missing";
  }
  if (!current_map || !current_map->valid()) {
    return "active_map_unavailable_after_planning";
  }
  if (!lingtu::nav::plan::sameMapIdentity(
          completion.result.map_identity, *current_map)) {
    return "active_map_changed_during_planning";
  }
  return {};
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
  } catch (const std::exception& exc) {
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
