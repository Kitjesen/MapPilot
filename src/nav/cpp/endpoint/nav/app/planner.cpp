#include "app/planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace lingtu::nav::endpoint {

std::string PlannerSelection::start(const CliConfig &config, const std::string &name) {
  if (busy()) return "planner_switch_pending";
  if (name != "scan" && name != "cmu") return "unknown_local_planner";
  const auto backend = name == "scan" ? nav_kernel::LocalPlannerBackend::Scan
                                      : nav_kernel::LocalPlannerBackend::Cmu;
  if (std::find(config.local_planners.begin(), config.local_planners.end(), backend) ==
      config.local_planners.end()) return "planner_not_available";
  if (config.control_mode != ControlMode::Autonomy) return "autonomy_mode_required";
  if (name == "scan" && std::abs(config.tick_hz - 100.0) > 1e-6)
    return "scan_requires_100_hz";
  if (name == "cmu" && config.path_library_dir.empty()) return "path_library_required";

  CliConfig next = config;
  next.local_planner_backend = backend;
  applyPlannerGeometry(next);
  loading_ = std::async(std::launch::async, [next = std::move(next)]() mutable {
    nav_kernel::LocalPlanner planner(buildLocalPlannerParams(next));
    if (!planner.configure(next.path_library_dir))
      throw std::runtime_error("local_planner_configuration_failed");
    auto execution = buildExecutorConfig(next);
    return PreparedPlanner{std::move(next), navigation::Executor(std::move(execution), std::move(planner))};
  });
  return {};
}

std::optional<PreparedPlanner> PlannerSelection::poll() {
  for (auto it = retired_.begin(); it != retired_.end();) {
    if (it->wait_for(std::chrono::seconds(0)) == std::future_status::ready)
      it = retired_.erase(it);
    else
      ++it;
  }
  if (!busy() || loading_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    return std::nullopt;
  return loading_.get();
}

void PlannerSelection::retire(navigation::Executor executor) {
  retired_.push_back(std::async(std::launch::async, [old = std::move(executor)]() mutable {
    // Destroy on this worker, not when the future's captured closure is released.
    auto owned = std::move(old);
  }));
}

}  // namespace lingtu::nav::endpoint
