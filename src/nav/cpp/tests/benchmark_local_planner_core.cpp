#include "planning/local/planner.hpp"
#include "planner_fixture.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
constexpr double kNavTickBudgetMs = 50.0;

std::vector<float> makeObstacleCloud(int points) {
  std::vector<float> cloud;
  cloud.reserve(static_cast<std::size_t>(points) * 4);
  std::uint32_t state = 0x5EED1234u;
  auto next = [&]() {
    state = state * 1664525u + 1013904223u;
    return static_cast<float>((state >> 8) & 0x00FFFFFFu) /
           static_cast<float>(0x01000000u);
  };

  for (int i = 0; i < points; ++i) {
    const float x = -3.0f + 7.0f * next();
    const float y = -3.2f + 6.4f * next();
    const float z = -0.05f + 0.35f * next();
    const bool obstacle = (i % 5) == 0;
    const float height = obstacle ? (0.24f + 0.12f * next()) : (0.04f + 0.10f * next());
    cloud.push_back(x);
    cloud.push_back(y);
    cloud.push_back(z);
    cloud.push_back(height);
  }
  return cloud;
}

std::vector<float> makeTraversabilityGrid(int rows, int cols) {
  std::vector<float> grid(static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols), 0.0f);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const float cx = static_cast<float>(c - cols / 2);
      const float cy = static_cast<float>(r - rows / 2);
      const float d = std::sqrt(cx * cx + cy * cy);
      grid[static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) + static_cast<std::size_t>(c)] =
          d < 12.0f ? 55.0f : 5.0f;
    }
  }
  return grid;
}

std::vector<float> makeLateBlockedTraversabilityGrid(
    int rows, int cols, double resolution, double origin_x, double origin_y) {
  std::vector<float> grid(
      static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols), 0.0f);
  for (int row = 0; row < rows; ++row) {
    const double y = origin_y + (static_cast<double>(row) + 0.5) * resolution;
    for (int col = 0; col < cols; ++col) {
      const double x = origin_x + (static_cast<double>(col) + 0.5) * resolution;
      if (std::hypot(x, y) >= 1.6) {
        grid[static_cast<std::size_t>(row) * static_cast<std::size_t>(cols) +
             static_cast<std::size_t>(col)] = 100.0f;
      }
    }
  }
  return grid;
}

struct Stats {
  double mean_ms{0.0};
  double p50_ms{0.0};
  double p95_ms{0.0};
  double p99_ms{0.0};
  double max_ms{0.0};
  double over_budget_pct{0.0};
};

Stats summarize(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  const double sum = std::accumulate(values.begin(), values.end(), 0.0);
  auto percentile = [&](double p) {
    const std::size_t idx = std::min<std::size_t>(
        values.size() - 1,
        static_cast<std::size_t>(std::llround(p * static_cast<double>(values.size() - 1))));
    return values[idx];
  };
  const auto over_budget = std::count_if(values.begin(), values.end(),
                                         [](double value) { return value > kNavTickBudgetMs; });
  return {
      sum / static_cast<double>(values.size()),
      percentile(0.50),
      percentile(0.95),
      percentile(0.99),
      values.back(),
      100.0 * static_cast<double>(over_budget) / static_cast<double>(values.size()),
  };
}

template <typename Fn>
Stats runTimed(int iterations, int warmup, Fn&& fn) {
  for (int i = 0; i < warmup; ++i) {
    fn(static_cast<double>(i) * 0.05);
  }

  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(iterations));
  for (int i = 0; i < iterations; ++i) {
    const auto t0 = Clock::now();
    fn(static_cast<double>(warmup + i) * 0.05);
    const auto t1 = Clock::now();
    values.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
  }
  return summarize(std::move(values));
}

void printStats(const std::string& label, int points, const Stats& stats) {
  std::cout << std::left << std::setw(30) << label
            << " points=" << std::setw(6) << points
            << " mean_ms=" << std::fixed << std::setprecision(3) << stats.mean_ms
            << " p50_ms=" << stats.p50_ms
            << " p95_ms=" << stats.p95_ms
            << " p99_ms=" << stats.p99_ms
            << " max_ms=" << stats.max_ms
            << " over_50ms_pct=" << stats.over_budget_pct << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: benchmark_local_planner_core PATH_LIBRARY [iterations]\n";
    return 2;
  }

  const std::string path_library = argv[1];
  const int iterations = argc >= 3 ? std::max(1, std::atoi(argv[2])) : 300;
  const int warmup = std::max(20, iterations / 10);

  nav_kernel::LocalPlannerParams params;
  params.checkObstacle = true;
  params.useTerrainAnalysis = true;
  params.useTraversabilityCost = true;
  params.traversabilityNearFieldStop = true;
  params.pathScaleBySpeed = false;
  params.pathRangeBySpeed = false;
  params.autonomySpeed = 0.4;
  params.maxSpeed = 1.0;

  nav_kernel::PlannerFixture planner(params);
  if (!planner.configure(path_library)) {
    std::cerr << "failed to load path library: " << path_library << '\n';
    return 1;
  }

  constexpr int rows = 160;
  constexpr int cols = 160;
  constexpr double resolution = 0.10;
  constexpr double origin_x = -8.0;
  constexpr double origin_y = -8.0;
  const auto traversability = makeTraversabilityGrid(rows, cols);
  const auto late_blocked_traversability = makeLateBlockedTraversabilityGrid(
      rows, cols, resolution, origin_x, origin_y);

  std::cout << "path_library=" << path_library << " iterations=" << iterations
            << " warmup=" << warmup << " scoring_threads=" << params.scoringThreads
            << " tick_budget_ms=" << kNavTickBudgetMs << '\n';

  // 5800 is the field measured-obstacle cap plus the bounded prediction cap.
  for (const int points : {0, 500, 1500, 5000, 5800, 10000}) {
    const auto cloud = makeObstacleCloud(points);
    const float *cloud_data = cloud.empty() ? nullptr : cloud.data();

    const auto no_grid = runTimed(iterations, warmup, [&](double t) {
      planner.planFrame(
          0.0, 0.0, 0.0, 0.0,
          3.0, 0.4,
          nullptr, 0, 0, 0.0, 0.0, 0.0,
          cloud_data, points, t);
    });
    printStats("planFrame no grid", points, no_grid);

    const auto grid_each_tick = runTimed(iterations, warmup, [&](double t) {
      planner.planFrame(
          0.0, 0.0, 0.0, 0.0,
          3.0, 0.4,
          traversability.data(), rows, cols, resolution, origin_x, origin_y,
          cloud_data, points, t);
    });
    printStats("planFrame grid copy", points, grid_each_tick);

    planner.setTraversabilityGrid(traversability.data(), rows, cols, resolution, origin_x, origin_y);
    const auto cached_grid = runTimed(iterations, warmup, [&](double t) {
      planner.setVehicle(0.0, 0.0, 0.0, 0.0);
      planner.setGoal(3.0, 0.4);
      planner.plan(cloud_data, points, t);
    });
    printStats("plan cached grid", points, cached_grid);

    const auto teleop_intent = runTimed(iterations, warmup, [&](double t) {
      planner.setVehicle(0.0, 0.0, 0.0, 0.0);
      planner.planObjective(cloud_data, points, t, 0.0, 1.0, 2.0, 55.0);
    });
    printStats("teleopIntent cached grid", points, teleop_intent);

    if (points == 0 || points == 5000) {
      planner.setTraversabilityGrid(
          late_blocked_traversability.data(), rows, cols,
          resolution, origin_x, origin_y);
      const auto blocked_teleop_intent = runTimed(iterations, warmup, [&](double t) {
        planner.setVehicle(0.0, 0.0, 0.0, 0.0);
        planner.planObjective(cloud_data, points, t, 0.0, 1.0, 2.0, 55.0);
      });
      printStats("teleopIntent late blocked", points, blocked_teleop_intent);
    }
  }

  return 0;
}
