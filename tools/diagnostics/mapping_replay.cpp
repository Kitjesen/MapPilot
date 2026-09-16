#include "localization/opt/online_mapping.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

namespace opt = lingtu::localization::opt;
int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: lt_mapping_replay MAP_DIRECTORY REPORT_JSON\n";
    return 2;
  }
  try {
    const auto root = std::filesystem::path(argv[1]);
    const auto poses = opt::read_poses(root / "poses.txt");
    opt::OnlineMapping mapper;
    mapper.reset(1);
    std::shared_ptr<const opt::OnlineMappingSnapshot> snapshot;
    const auto started = std::chrono::steady_clock::now();
    double worst_ms = 0;
    for (std::size_t i = 0; i < poses.size(); ++i) {
      opt::MappingFrame frame;
      frame.keyframe = poses[i]; frame.stamp_s = static_cast<double>(i + 1);
      frame.body_cloud = opt::read_point_cloud(root / "patches" / poses[i].patch_name);
      const auto tick = std::chrono::steady_clock::now();
      if (!mapper.enqueue(std::move(frame)).ok) throw std::runtime_error("frame rejected");
      do {
        if (auto result = mapper.poll()) snapshot = result;
        if (std::chrono::steady_clock::now() - tick > std::chrono::seconds(60))
          throw std::runtime_error("worker exceeded replay budget");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      } while (mapper.busy());
      worst_ms = std::max(worst_ms, std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - tick).count());
      if ((i+1) % 20 == 0 || i+1 == poses.size())
        std::cout << "frames=" << i+1 << " registered=" << snapshot->registered_keyframes
                  << " rejected=" << snapshot->registration_rejections << " loops="
                  << snapshot->loop_constraints << " code=" << snapshot->code << std::endl;
    }
    if (!snapshot) return 3;
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
           << ",\"elapsed_s\":" << std::chrono::duration<double>(
                std::chrono::steady_clock::now() - started).count() << "}\n";
    report.close();
    if (!report) throw std::runtime_error("cannot finish replay report output");
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
