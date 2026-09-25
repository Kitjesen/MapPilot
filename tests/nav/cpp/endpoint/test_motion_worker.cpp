#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "input/motion_worker.hpp"
#include "runtime/time.hpp"

using namespace lingtu::nav::endpoint;

namespace {

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

MotionResult waitForResult(MotionWorker &worker) {
  const auto deadline = SteadyClock::now() + std::chrono::seconds(10);
  while (SteadyClock::now() < deadline) {
    if (auto result = worker.poll()) {
      require(result->error.empty(), "worker processing failed");
      return std::move(*result);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  throw std::runtime_error("worker completion timed out");
}

std::vector<float> cloud(std::size_t count) {
  std::vector<float> points;
  points.reserve(count * 4);
  for (std::size_t i = 0; i < count; ++i) {
    const auto x = static_cast<float>(i % 160) * 0.08f - 6.4f;
    const auto y = static_cast<float>((i / 160) % 160) * 0.08f - 6.4f;
    const auto z = static_cast<float>(i / 25600) * 0.12f + 0.2f;
    points.insert(points.end(), {x, y, z, z});
  }
  return points;
}

void testMatchesSynchronousGeometry() {
  MotionLayerConfig config;
  config.inflation_radius_m = 0.12;
  MotionLayer reference(config);
  MotionWorker worker(MotionLayer(config), 2000);
  const SensorOrigin origin{0.0, 0.0, 0.5, true};
  const auto points = cloud(4000);
  for (int frame = 0; frame < 5; ++frame) {
    const double stamp = 10.0 + frame * 0.1;
    worker.submit({points, origin, stamp, 100.0 + frame * 0.1});
    reference.updateFromScan(origin, points, stamp);
    const auto clusters = reference.dynamicClusters(32, stamp);
    const auto measured = reference.snapshot(2000, stamp);
    const auto predicted = reference.snapshotPredictedDynamic(kMaxDynamicPredictionPoints, stamp);
    const auto volumes = reference.predictedVolumes(stamp);
    const auto result = waitForResult(worker);
    require(result.obstacles == measured && result.predicted_points == predicted,
            "async worker must preserve measured and predicted obstacle geometry");
    require(result.clusters.size() == clusters.size() &&
                result.predictions.size() == volumes.size(), "track output must match");
    require(result.stats.cells == reference.stats().cells &&
                result.stamp_s == stamp && result.receive_s == 100.0 + frame * 0.1,
            "stats and source/receive clocks must belong to the same completed frame");
  }
}

void testLatestFrameAndClear() {
  MotionWorker worker(MotionLayer{}, 2000);
  const auto points = cloud(30000);
  for (int i = 1; i <= 40; ++i) {
    worker.submit({points, {0, 0, 0.5, true}, static_cast<double>(i), 100.0 + i});
  }
  auto result = waitForResult(worker);
  while (result.stamp_s != 40.0) result = waitForResult(worker);
  require(result.receive_s == 140.0, "overload must retain the latest waiting frame");

  for (int i = 41; i <= 50; ++i) {
    worker.submit({points, {0, 0, 0.5, true}, static_cast<double>(i), 100.0 + i});
  }
  worker.clear();
  worker.submit({{20.0f, 0.0f, 0.2f, 0.2f}, {}, 1.0, 200.0});
  result = waitForResult(worker);
  require(result.stamp_s == 1.0 && result.receive_s == 200.0,
          "clear must discard pending, completed and in-flight old results");
  require(!result.obstacles.empty(), "new epoch must still process observations");
  for (std::size_t i = 0; i < result.obstacles.size(); i += 4) {
    require(result.obstacles[i] > 19.0f, "old epoch cells must not survive worker reset");
  }
}

void testMovingTargetWithSkippedFrames() {
  MotionLayerConfig config;
  config.voxel_size_m = 0.1;
  // This fixture primes free space once; retain that evidence for the replay.
  config.decay_s = 10.0;
  config.inflation_radius_m = 0.0;
  config.ray_clearing_interval_s = 0.0;
  config.ray_clear_max_range_m = 4.0;
  config.dynamic_min_cells = 3;
  // Keep production confirmation counts and TTL. Missing frames must not
  // masquerade as extra observations or change velocity's source clock.
  MotionWorker worker(MotionLayer(config), 2000);
  MotionLayer reference(config);
  const SensorOrigin origin{0, 0, 0, true};
  std::vector<float> rays;
  for (int frame = 0; frame < 8; ++frame) {
    const float x = 1.05f + 0.2f * frame;
    for (int cell = 0; cell < 4; ++cell) {
      rays.insert(rays.end(), {3.5f, (0.05f + 0.1f * cell) * 3.5f / x, 0, 0.4f});
    }
  }
  for (int frame = 1; frame <= 3; ++frame) {
    const double stamp = frame * 0.1;
    worker.submit({rays, origin, stamp, 100.0 + stamp});
    (void)waitForResult(worker);
    reference.updateFromScan(origin, rays, stamp);
    (void)reference.dynamicClusters(32, stamp);
  }
  std::uint32_t id = 0;
  for (int frame = 0; frame < 8; ++frame) {
    // One observation per four 20 Hz input frames: three omitted scans.
    const double stamp = 1.0 + 0.2 * frame;
    const float x = 1.05f + 0.2f * frame;
    std::vector<float> points;
    for (int cell = 0; cell < 4; ++cell) {
      points.insert(points.end(), {x, 0.05f + 0.1f * cell, 0, 0.4f});
    }
    worker.submit({points, origin, stamp, 100.0 + stamp});
    const auto result = waitForResult(worker);
    reference.updateFromScan(origin, points, stamp);
    const auto tracks = reference.dynamicClusters(32, stamp);
    require(result.clusters.size() == tracks.size(), "dropped-frame tracks must match reference");
    if (frame < 3) {
      require(result.clusters.empty(), "skipped scans must not count toward confirmation");
      continue;
    }
    require(result.clusters.size() == 1 && !result.predicted_points.empty() &&
                !result.predictions.empty(), "asynchronous moving target must actually predict");
    const auto &track = result.clusters.front();
    if (id == 0) id = track.id;
    require(track.id == id && std::abs(track.x - x) < 0.11,
            "target identity and current position must survive gaps below TTL");
    require(track.vx > 0.5 && track.vx < 1.5 && std::abs(track.vy) < 0.1,
            "velocity must use source time, not worker completion time");
    require(track.id == tracks.front().id && track.vx == tracks.front().vx &&
                track.x == tracks.front().x, "track values, not just counts, must match");
    require(result.predicted_points == reference.snapshotPredictedDynamic(
                kMaxDynamicPredictionPoints, stamp), "future geometry must match after skips");
  }
  worker.submit({rays, origin, 4.0, 104.0});
  const auto expired = waitForResult(worker);
  require(expired.clusters.empty() && expired.predictions.empty() && expired.predicted_points.empty(),
          "a lost target must not leave a moving prediction beyond TTL");
}

double percentile(std::vector<double> values, double fraction) {
  std::sort(values.begin(), values.end());
  return values[static_cast<std::size_t>((values.size() - 1) * fraction)];
}

// An explicit benchmark, not a wall-time assertion in the regression suite.
void benchmark() {
  constexpr int frames = 100;
  const auto points = cloud(20000);
  const SensorOrigin origin{0, 0, 0.5, true};
  std::vector<double> sync, submit, completion, worker_work;
  MotionLayer layer;
  for (int i = 0; i < frames; ++i) {
    const auto start = SteadyClock::now();
    const double stamp = 10.0 + i * 0.1;
    layer.updateFromScan(origin, points, stamp);
    (void)layer.dynamicClusters(32, stamp);
    (void)layer.snapshot(20000, stamp);
    (void)layer.snapshotPredictedDynamic(kMaxDynamicPredictionPoints, stamp);
    (void)layer.predictedVolumes(stamp);
    sync.push_back(elapsedMs(start));
  }
  MotionWorker worker(MotionLayer{}, 20000);
  for (int i = 0; i < frames; ++i) {
    MotionObservation input{points, origin, 10.0 + i * 0.1, 100.0 + i * 0.1};
    const auto start = SteadyClock::now();
    worker.submit(std::move(input));
    submit.push_back(elapsedMs(start));
    const auto result = waitForResult(worker);
    completion.push_back(elapsedMs(start));
    worker_work.push_back(result.update_ms + result.snapshot_ms);
  }
  std::cout << "{\"frames\":" << frames << ",\"points_per_frame\":" << points.size()/4
            << ",\"synchronous_p95_ms\":" << percentile(sync, 0.95)
            << ",\"submit_p95_ms\":" << percentile(submit, 0.95)
            << ",\"worker_work_p95_ms\":" << percentile(worker_work, 0.95)
            << ",\"completion_p95_ms\":" << percentile(completion, 0.95) << "}\n";
}
}  // namespace

int main(int argc, char **argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--benchmark") {
      benchmark();
    } else {
      testMatchesSynchronousGeometry();
      testLatestFrameAndClear();
      testMovingTargetWithSkippedFrames();
      std::cout << "test_motion_worker passed\n";
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "test_motion_worker failed: " << error.what() << '\n';
    return 1;
  }
}
