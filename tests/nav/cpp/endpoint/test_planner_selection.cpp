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
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
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
    config.path_library_dir = LINGTU_TEST_PATH_LIBRARY;
    config.use_traversability_cost = true;
    PlannerSelection selection;
    require(selection.start(config, "unknown") == "unknown_local_planner", "unknown backend accepted");
    require(!selection.busy(), "invalid request started work");
    auto restricted = config;
    restricted.local_planners = {nav_kernel::LocalPlannerBackend::Scan};
    require(selection.start(restricted, "cmu") == "planner_not_available", "Product restriction ignored");
    auto wrong_rate = config;
    wrong_rate.tick_hz = 50.0;
    require(selection.start(wrong_rate, "scan") == "scan_requires_100_hz", "SCAN clock changed");
    auto no_library = config;
    no_library.path_library_dir.clear();
    require(selection.start(no_library, "cmu") == "path_library_required", "CMU missing library accepted");

    nav_kernel::LocalPlanner initial(buildLocalPlannerParams(config));
    require(initial.configure(), "initial SCAN configure failed");
    lingtu::nav::navigation::Executor executor(buildExecutorConfig(config), std::move(initial));
    lingtu::nav::navigation::Route old_route;
    old_route.points = {{0, 0, 0}, {2, 0, 0}};
    executor.setRoute(old_route);

    auto invalid = config;
    invalid.path_library_dir += "/missing";
    require(selection.start(invalid, "cmu").empty(), "asynchronous load not started");
    bool load_failed = false;
    try { (void)wait(selection); } catch (const std::runtime_error &) { load_failed = true; }
    require(load_failed && !selection.busy(), "bad library did not fail");
    require(executor.hasRoute(), "failed preparation changed active executor");
    executor.clear();

    for (const char *name : {"cmu", "scan", "cmu"}) {
      require(selection.start(config, name).empty(), "selection did not start");
      require(selection.start(config, name) == "planner_switch_pending", "overlapping load accepted");
      auto prepared = wait(selection);
      const bool scan = prepared.config.local_planner_backend == nav_kernel::LocalPlannerBackend::Scan;
      const auto gate = inputGateConfig(prepared.config);
      require(gate.require_local_collision == scan && gate.require_cloud != scan,
              "new backend retained old input requirements");
      std::swap(executor, prepared.executor);
      selection.retire(std::move(prepared.executor));
      config = std::move(prepared.config);
      require(!executor.hasRoute(), "new executor inherited old route");
      const auto idle = executor.tick({});
      require(idle.cmd_vel.vx == 0.0 && idle.cmd_vel.vy == 0.0 && idle.cmd_vel.wz == 0.0,
              "selection resumed old motion");
    }
    std::puts("planner selection: passed");
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "planner selection: %s\n", error.what());
    return 1;
  }
}
