#include "localization/opt/online_mapping.hpp"
#include "localization/sam/config_yaml.hpp"
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <algorithm>
#include <iomanip>
#ifdef __linux__
#include <sys/resource.h>
#endif

namespace opt = lingtu::localization::opt;
int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: lt_mapping_replay MAP_DIRECTORY REPORT_JSON\n";
    return 2;
  }
  try {
    const auto root = std::filesystem::path(argv[1]);
    if (!std::filesystem::is_regular_file(root / "poses.raw.txt"))
      throw std::runtime_error("poses.raw.txt is required; optimized poses are not odometry");
    const auto poses = opt::read_poses(root / "poses.raw.txt");
    if (poses.empty()) throw std::runtime_error("raw LIO keyframes are required; optimized poses are not odometry");
    std::ifstream timestamps(root / "keyframes.timestamps.txt");
    if (!timestamps) throw std::runtime_error("recorded keyframe timestamps are required");
    std::vector<double> stamps;
    double previous = -1;
    for (const auto& pose : poses) {
      std::string name; double stamp;
      if (!(timestamps >> name >> stamp) || name != pose.patch_name ||
          !std::isfinite(stamp) || stamp <= previous)
        throw std::runtime_error("keyframe timestamps do not match raw poses");
      stamps.push_back(stamp); previous = stamp;
    }
    std::string extra;
    if (timestamps >> extra) throw std::runtime_error("extra keyframe timestamps");
    for (const auto* suffix : {"", ".loops.json", ".poses.txt"})
      if (std::filesystem::exists(std::string(argv[2])+suffix))
        throw std::runtime_error("replay output already exists");
    opt::OnlineMappingOptions options;
    const auto evidence_path = root / "sam_loops.json";
    if (std::filesystem::is_regular_file(evidence_path)) {
      const auto evidence=YAML::LoadFile(evidence_path.string());
      if (evidence["schema"].as<std::string>()!="lingtu.sam_loops.v1" || !evidence["config"])
        throw std::runtime_error("unsupported SAM evidence configuration");
      options.sam=lingtu::localization::sam::readConfig(evidence["config"]);
    }
    opt::OnlineMapping mapper(options);
    mapper.reset(1);
    std::shared_ptr<const opt::OnlineMappingSnapshot> snapshot;
    const auto started = std::chrono::steady_clock::now();
    double worst_ms = 0;
    double worst_worker_ms = 0, worst_preview_ms = 0;
    std::size_t peak_cloud_bytes = 0;
    std::vector<double> latency;
    for (std::size_t i = 0; i < poses.size(); ++i) {
      opt::MappingFrame frame;
      frame.keyframe = poses[i]; frame.stamp_s = stamps[i];
      frame.body_cloud = opt::read_point_cloud(root / "patches" / poses[i].patch_name);
      const auto tick = std::chrono::steady_clock::now();
      if (!mapper.enqueue(std::move(frame)).ok) throw std::runtime_error("frame rejected");
      do {
        if (auto result = mapper.poll()) snapshot = result;
        if (std::chrono::steady_clock::now() - tick > std::chrono::seconds(60))
          throw std::runtime_error("worker exceeded replay budget");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      } while (mapper.busy());
      if (!snapshot || snapshot->optimization_failures || snapshot->keyframes.size()!=i+1)
        throw std::runtime_error("replay worker failed or lost a frame");
      const double elapsed_ms=std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - tick).count();
      latency.push_back(elapsed_ms);
      worst_ms = std::max(worst_ms,elapsed_ms);
      worst_worker_ms = std::max(worst_worker_ms,snapshot->worker_ms);
      worst_preview_ms = std::max(worst_preview_ms,snapshot->preview_ms);
      peak_cloud_bytes = std::max(peak_cloud_bytes,snapshot->cloud_bytes);
      if ((i+1) % 20 == 0 || i+1 == poses.size())
        std::cout << "frames=" << i+1 << " registered=" << snapshot->registered_keyframes
                  << " rejected=" << snapshot->registration_rejections << " loops="
                  << snapshot->loop_constraints << " code=" << snapshot->code << std::endl;
    }
    if (!snapshot) return 3;
    const std::filesystem::path report_path(argv[2]);
    std::ofstream loops(report_path.string()+".loops.json");
    lingtu::localization::sam::writeLoopEvidence(loops,snapshot->sam_config,snapshot->loop_records);
    loops.close();
    if (!loops) throw std::runtime_error("cannot finish loop evidence");
    std::ofstream corrected(report_path.string()+".poses.txt");
    corrected<<std::setprecision(17);
    for (const auto& frame:snapshot->keyframes) {
      const auto& p=frame.pose;
      corrected<<frame.patch_name<<' '<<p.x<<' '<<p.y<<' '<<p.z<<' '<<p.qw<<' '<<p.qx<<' '<<p.qy<<' '<<p.qz<<'\n';
    }
    corrected.close();
    if (!corrected) throw std::runtime_error("cannot finish replay poses");
    std::sort(latency.begin(),latency.end());
    long peak_rss_kib=0;
#ifdef __linux__
    rusage usage{};
    if (getrusage(RUSAGE_SELF,&usage)==0) peak_rss_kib=usage.ru_maxrss;
#endif
    std::ofstream report(argv[2]);
    if (!report) throw std::runtime_error("cannot open replay report output");
    report << "{\"frames\":" << snapshot->keyframes.size()
           << ",\"registered\":" << snapshot->registered_keyframes
           << ",\"rejected\":" << snapshot->registration_rejections
           << ",\"loops\":" << snapshot->loop_constraints
           << ",\"optimizations\":" << snapshot->optimizations
           << ",\"optimization_failures\":" << snapshot->optimization_failures
           << ",\"points\":" << snapshot->cloud.size()
           << ",\"worst_frame_ms\":" << worst_ms
           << ",\"p95_frame_ms\":" << latency[static_cast<std::size_t>(.95*(latency.size()-1))]
           << ",\"worst_worker_ms\":" << worst_worker_ms
           << ",\"worst_preview_ms\":" << worst_preview_ms
           << ",\"peak_cloud_bytes\":" << peak_cloud_bytes
           << ",\"peak_rss_kib\":" << peak_rss_kib
           << ",\"elapsed_s\":" << std::chrono::duration<double>(
                std::chrono::steady_clock::now() - started).count() << "}\n";
    report.close();
    if (!report) throw std::runtime_error("cannot finish replay report output");
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
