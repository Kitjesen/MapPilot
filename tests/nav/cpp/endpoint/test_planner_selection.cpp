#include "app/planner.hpp"

#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <thread>

using namespace lingtu::nav::endpoint;

namespace {
void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

PreparedPlanner wait(PlannerSelection &selection) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  do {
    if (auto prepared = selection.poll()) return std::move(*prepared);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (std::chrono::steady_clock::now() < deadline);
  throw std::runtime_error("planner loading timed out");
}
}  // namespace

int main() {
  try {
    CliConfig config;
    config.tick_hz = 100.0;
    config.local_planner_backend = nav_kernel::LocalPlannerBackend::Scan;
    config.use_traversability_cost = true;
    PlannerSelection selection;
    require(selection.start(config, "unknown") == "unknown_local_planner", "unknown backend accepted");
    require(selection.start(config, "cmu") == "unknown_local_planner", "removed backend accepted");
    require(!selection.busy(), "invalid request started work");
    auto wrong_rate = config;
    wrong_rate.tick_hz = 50.0;
    require(selection.start(wrong_rate, "scan") == "scan_requires_100_hz", "SCAN clock changed");

    nav_kernel::LocalPlanner initial(buildLocalPlannerParams(config));
    require(initial.configure(), "initial SCAN configure failed");
    lingtu::nav::navigation::Executor executor(buildExecutorConfig(config), std::move(initial));
    lingtu::nav::navigation::Route old_route;
    old_route.points = {{0, 0, 0}, {2, 0, 0}};
    executor.setRoute(old_route);

    require(selection.start(config, "scan").empty(), "selection did not start");
    require(selection.start(config, "scan") == "planner_switch_pending", "overlapping load accepted");
    auto prepared = wait(selection);
    require(prepared.config.local_planner_backend == nav_kernel::LocalPlannerBackend::Scan,
            "selection left SCAN");
    const auto gate = inputGateConfig(prepared.config);
    require(gate.require_local_collision && !gate.require_cloud && !gate.require_traversability,
            "SCAN selection changed its collision input");
    std::swap(executor, prepared.executor);
    selection.retire(std::move(prepared.executor));
    require(!executor.hasRoute(), "new executor inherited old route");
    const auto idle = executor.tick({});
    require(idle.cmd_vel.vx == 0.0 && idle.cmd_vel.vy == 0.0 && idle.cmd_vel.wz == 0.0,
            "selection resumed old motion");
    std::puts("planner selection: passed");
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "planner selection: %s\n", error.what());
    return 1;
  }
}
