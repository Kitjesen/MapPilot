#include "global_plan_task.hpp"

#include <chrono>
#include <exception>
#include <stdexcept>
#include <utility>

namespace lingtu::nav::endpoint {

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
