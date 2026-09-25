#include "localization/opt/online_mapping.hpp"
#include "localization/opt/pose_math.hpp"
#include "localization/sam/config_yaml.hpp"
#include <sstream>
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
    namespace sam = lingtu::localization::sam;
    auto tuning=sam::readConfig(YAML::Load("rotation_sigma_rad: 0.08\nodom_variance: [1,2,3,4,5,6]"));
    std::ostringstream evidence;
    sam::writeLoopEvidence(evidence,tuning,{});
    const auto restored=sam::readConfig(YAML::Load(evidence.str())["config"]);
    require(restored.rotation_sigma_rad==.08 && restored.odom_variance==tuning.odom_variance,
            "saved configuration cannot reproduce field tuning");
    bool malformed=false;
    try { sam::readConfig(YAML::Load("odom_variance: [1,2]")); }
    catch(const std::invalid_argument&) { malformed=true; }
    require(malformed,"malformed odometry variances silently defaulted");
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

    opt::OnlineMappingOptions deferred_options;
    deferred_options.max_keyframes = 6;
    deferred_options.max_pending = 4;
    deferred_options.preview_points = 16;
    opt::OnlineMapping deferred(deferred_options);
    for (int i = 0; i < 4; ++i)
      require(deferred.enqueue(frame(i)).ok, "deferred fixture rejected initial frame");
    require(!deferred.canEnqueue(), "full pending queue accepted another frame");
    deferred.poll();
    require(deferred.canEnqueue(), "worker did not release pending slots");
    require(deferred.enqueue(frame(4)).ok && deferred.enqueue(frame(5)).ok,
            "deferred frames were lost after worker started");
    const auto deferred_snapshot = finish(deferred);
    require(deferred.dropped_frames() == 0 && deferred_snapshot &&
                deferred_snapshot->keyframes.size() == 6 &&
                deferred_snapshot->cloud.size() <= deferred_options.preview_points,
            "deferred frames or bounded preview were lost");

    opt::OnlineMappingOptions loop_options;
    loop_options.sam.voxel_m = .15;
    loop_options.sam.min_time_s = 30;
    loop_options.sam.submap_half_window = 2;
    opt::OnlineMapping loop_mapper(loop_options);
    std::shared_ptr<const opt::OnlineMappingSnapshot> closed;
    for (int i=0;i<16;++i) {
      auto sample=frame(i<=7?i:15-i);
      sample.keyframe.patch_name=std::to_string(i)+".pcd";
      sample.stamp_s=i*3.+1;
      sample.keyframe.pose.x+=i*.005;
      sample.keyframe.pose.z+=i*.003;
      if(i==3) sample.body_cloud={{0,0,0,1}};
      require(loop_mapper.enqueue(std::move(sample)).ok,"SAM input rejected");
      closed=finish(loop_mapper);
    }
    require(closed && closed->registered_keyframes==16 && closed->keyframes.size()==16,
            "sparse scan disconnected continuous LIO graph");
    require(closed->loop_constraints>0 && closed->optimizations>0,
            "upstream ICP loop did not update online map");
    require(std::abs(closed->keyframes.back().pose.x-.075)<1e-7 &&
            std::abs(closed->keyframes.back().pose.z-.045)<1e-7,
            "SAM correction jumped the continuous odometry anchor");
    require(closed->cloud.size()<=loop_options.preview_points,
            "corrected preview exceeded memory budget");
    require(closed->loop_records.size()==16 && closed->cloud_bytes>0 &&
            closed->worker_ms>0 && closed->preview_ms>0,
            "loop evidence or performance counters missing");
    for (const auto& record:closed->loop_records)
      for (auto target:record.target_frames)
        require(target<record.current && (record.current-target)*3.>30,
                "recent keyframe leaked into loop target");
    std::cout << "online mapping passed\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
