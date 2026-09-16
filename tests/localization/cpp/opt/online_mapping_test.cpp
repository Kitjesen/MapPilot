#include "localization/opt/online_mapping.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace opt = lingtu::localization::opt;
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }

opt::MappingFrame frame(int index) {
  opt::MappingFrame result;
  result.keyframe.patch_name = std::to_string(index) + ".pcd";
  result.keyframe.pose.x = index * 0.3;
  result.stamp_s = 1.0 + index;
  for (int a = -12; a <= 12; ++a) for (int b = -12; b <= 12; ++b) {
    const float u = a * .2F, v = b * .2F, x = static_cast<float>(result.keyframe.pose.x);
    const float noise = index * .002F * std::sin(.7F*u + .3F*v);
    result.body_cloud.push_back({u-x, v, noise, 1});
    result.body_cloud.push_back({u-x, -2.2F+noise, v, 1});
    result.body_cloud.push_back({2.4F+noise-x, u, v, 1});
  }
  return result;
}

std::shared_ptr<const opt::OnlineMappingSnapshot> finish(opt::OnlineMapping& mapper) {
  std::shared_ptr<const opt::OnlineMappingSnapshot> latest;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(25);
  do {
    if (auto result = mapper.poll()) latest = result;
    if (!mapper.busy()) break;
    require(std::chrono::steady_clock::now() < deadline, "mapping worker timed out");
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  } while (true);
  return latest;
}

int main() {
  try {
    opt::OnlineMappingOptions options;
    options.max_keyframes = 3;
    options.max_pending = 2;
    opt::OnlineMapping mapper(options);
    mapper.reset(9);
    require(mapper.enqueue(frame(0)).ok, "first frame rejected");
    mapper.poll();
    require(mapper.enqueue(frame(1)).ok, "queued frame rejected");
    require(mapper.enqueue(frame(2)).ok, "third frame rejected");
    require(!mapper.enqueue(frame(3)).ok, "capacity was not bounded");
    const auto snapshot = finish(mapper);
    require(snapshot && snapshot->source_epoch == 9, "wrong mapping epoch");
    require(snapshot->registered_keyframes == 3 && snapshot->keyframes.size() == 3,
            "adjacent geometry did not enter graph");
    require(snapshot->loop_constraints == 0 && snapshot->optimizations == 0,
            "odometry fabricated a loop");
    require(!snapshot->cloud.empty() && snapshot->cloud.size() <= options.preview_points,
            "global preview is empty or unbounded");
    require(std::abs(snapshot->keyframes.back().pose.x - .6) < 1e-9,
            "uncorrected geometry moved");
    mapper.reset(10);
    require(mapper.enqueue(frame(0)).ok, "reset did not release capacity");
    mapper.poll();
    mapper.reset(11);
    require(mapper.enqueue(frame(1)).ok, "new generation rejected");
    const auto fresh = finish(mapper);
    require(fresh && fresh->source_epoch == 11 && fresh->keyframes.size() == 1 &&
            fresh->keyframes.front().patch_name == "1.pcd", "old mapping job leaked after reset");
    mapper.reset(12);
    auto bad = frame(0);
    bad.keyframe.pose.z = std::nan("");
    require(!mapper.enqueue(std::move(bad)).ok, "nonfinite frame accepted");

    opt::OnlineMappingOptions loop_options;
    loop_options.verification.min_index_separation = 6;
    loop_options.verification.min_path_separation_m = 2.0;
    loop_options.verification.submap_half_window = 0;
    opt::OnlineMapping loop_mapper(loop_options);
    loop_mapper.reset(20);
    std::shared_ptr<const opt::OnlineMappingSnapshot> closed;
    for (int i = 0; i <= 12; ++i) {
      const int place = i <= 5 ? i : i <= 10 ? 10-i : i-10;
      auto sample = frame(place);
      sample.keyframe.patch_name = std::to_string(i) + ".pcd";
      sample.stamp_s = i + 1.0;
      sample.keyframe.pose.x += i * .02;
      require(loop_mapper.enqueue(std::move(sample)).ok, "loop input rejected");
      closed = finish(loop_mapper);
      if (closed && closed->code == "optimizer_quality_failed")
        std::cout << "failed frame=" << i << " iterations=" << closed->last_optimization.iterations
                  << " accepted=" << closed->last_optimization.accepted_steps
                  << " rejected=" << closed->last_optimization.rejected_steps
                  << " cost=" << closed->last_optimization.initial_cost << " -> "
                  << closed->last_optimization.final_cost << '\n';
    }
    if (closed) std::cout << "loop frames=" << closed->registered_keyframes
        << " loops=" << closed->loop_constraints << " solves=" << closed->optimizations
        << " state=" << closed->code << " correction=" << closed->global_from_odom.x << '\n';
    require(closed && closed->loop_constraints > 0 && closed->optimizations > 0,
            "cloud-based online loop never reached optimization");
    require(std::abs(closed->global_from_odom.x + .24) < .02,
            "verified loop did not recover the injected odometry drift");
    require(std::abs(closed->keyframes.back().pose.x - .84) < .02,
            "loop correction jumped the current odometry anchor");
    loop_options.max_iterations = 1;
    opt::OnlineMapping limited_solver(loop_options);
    for (int i = 0; i <= 12; ++i) {
      const int place = i <= 5 ? i : i <= 10 ? 10-i : i-10;
      auto sample = frame(place);
      sample.keyframe.patch_name = std::to_string(i) + ".pcd";
      sample.stamp_s = i + 1.0;
      sample.keyframe.pose.x += i * .02;
      require(limited_solver.enqueue(std::move(sample)).ok, "limited solver input rejected");
      closed = finish(limited_solver);
    }
    require(closed && closed->optimization_failures > 0, "nonconverged solve fixture did not fail");
    require(closed->optimizations == 0 && closed->loop_constraints == 0,
            "failed solve committed new loop constraints");
    require(std::abs(closed->global_from_odom.x) < 1e-12,
            "failed solve published a map correction");
    std::cout << "online mapping passed\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
