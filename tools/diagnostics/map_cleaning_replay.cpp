#include "cleaner.hpp"
#include "core/io.hpp"
#include "core/evidence.hpp"
#include "cleaning/incremental.hpp"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
namespace fs=std::filesystem;
using namespace lingtu::map_cleaning;
int main(int argc,char** argv) {
  if(argc!=3){std::cerr<<"Usage: maps_cleaning_replay MAP_DIRECTORY NEW_OUTPUT_DIRECTORY\n";return 2;}
  try {
    const fs::path source=argv[1],output=argv[2];
    if(fs::exists(output))throw std::runtime_error("output directory already exists");
    const auto origin=readSensorOrigin(source/"scan_origin.txt");
    if(!origin)throw std::runtime_error("recorded calibrated scan_origin.txt required");
    std::vector<std::string> order;
    const auto poses=readLingtuPoses(source/"poses.txt",&order);
    if(order.empty())throw std::runtime_error("no recorded scans");
    fs::create_directories(output);
    StaticCleanerOptions options;
    options.map_dir=source; options.output_clean_pcd=output/"saved.clean.pcd";
    options.output_removed_pcd=output/"saved.removed.pcd";
    const auto started=std::chrono::steady_clock::now();
    const auto saved=cleanStaticMap(options);
    std::ofstream(output/"saved.report.json")<<toJson(saved);
    if(!saved.success)throw std::runtime_error(saved.message);
    const double saved_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    lingtu::maps::cleaning::Config config;
    config.max_ray_length_m=options.max_ray_length_m;
    lingtu::maps::cleaning::IncrementalCleaner live(config);
    std::vector<double> latency;
    std::size_t total_points=0;
    std::ofstream frames(output/"frames.csv");
    frames<<"frame,input_points,retained_cells,removed_events,capacity_rejections,observe_ms\n";
    for(std::size_t i=0;i<order.size();++i) {
      const auto scan=readPcd(source/"patches"/order[i]);
      const auto& pose=poses.at(order[i]);
      const auto map_origin=transformPoint({(*origin)[0],(*origin)[1],(*origin)[2]},pose);
      std::vector<lingtu::maps::cleaning::Point> points;
      points.reserve(scan.size());total_points+=scan.size();
      for(auto point:scan){const auto p=transformPoint(point,pose);points.push_back({p.x,p.y,p.z,p.intensity});}
      const auto tick=std::chrono::steady_clock::now();
      live.Observe(points,{map_origin.x,map_origin.y,map_origin.z});
      const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tick).count();
      latency.push_back(ms);
      frames<<i<<','<<scan.size()<<','<<live.Count()<<','<<live.RemovedCount()<<','<<live.CapacityRejections()<<','<<ms<<'\n';
      if((i+1)%20==0 || i+1==order.size())std::cout<<"frames="<<i+1<<" cells="<<live.Count()<<" removed="<<live.RemovedCount()<<" ms="<<ms<<std::endl;
    }
    std::vector<PointXYZI> points;
    for(const auto p:live.Snapshot())points.push_back({p.x,p.y,p.z,p.intensity});
    writePcd(output/"live.clean.pcd",points);
    std::sort(latency.begin(),latency.end());
    std::ofstream report(output/"replay.report.json");
    report<<"{\"frames\":"<<order.size()<<",\"input_points\":"<<total_points
          <<",\"saved_source_points\":"<<saved.source_points<<",\"saved_removed_points\":"<<saved.removed_points
          <<",\"saved_ground_protected\":"<<saved.kept_ground_points<<",\"saved_surface_protected\":"<<saved.kept_supported_surface_points
          <<",\"saved_ms\":"<<saved_ms<<",\"live_cells\":"<<points.size()<<",\"live_removal_events\":"<<live.RemovedCount()
          <<",\"capacity_rejections\":"<<live.CapacityRejections()<<",\"p50_observe_ms\":"<<latency[latency.size()/2]
          <<",\"p95_observe_ms\":"<<latency[static_cast<std::size_t>(.95*(latency.size()-1))]
          <<",\"max_observe_ms\":"<<latency.back()<<",\"semantic_ground_truth\":false}\n";
    if(!report || !frames)throw std::runtime_error("failed writing replay evidence");
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
