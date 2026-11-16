#include "prepared.hpp"
#include "mls_backend.hpp"
#include "surface_graph.hpp"
#include "lingtu/maps/io/binary.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>

#if defined(_WIN32)
#define NOMINMAX
#include <Windows.h>
#endif

namespace octoplanner3d::runtime {
namespace {
using lingtu::maps::BinaryReader;
using lingtu::maps::BinaryWriter;
using lingtu::nav::plan::MapIdentity;
using global_planner::PlannerConfig;
// Bump when prepared geometry semantics or the binary layout changes.
constexpr const char* kFormat="lingtu.navigation.prepared.8";
constexpr std::uint32_t kEndian=0x01020304U;
struct Header { MapIdentity map; std::uint64_t size; std::int64_t modified; std::array<double,24> preparation; };

Header readHeader(BinaryReader& in) {
  if(in.text()!=kFormat || in.value<std::uint32_t>()!=kEndian)
    throw std::runtime_error("unsupported prepared map format");
  Header header;
  header.map.map_id=in.text();header.map.content_epoch=in.value<std::uint64_t>();
  header.map.frame_id=in.text();
  header.size=in.value<std::uint64_t>();header.modified=in.value<std::int64_t>();
  for(auto& value:header.preparation)value=in.value<double>();
  return header;
}

std::filesystem::path profilePath(const std::filesystem::path& directory,
    const MapIdentity& identity,const PlannerConfig& config,bool exact) {
  if(!std::filesystem::is_directory(directory))return {};
  for(const auto& entry:std::filesystem::directory_iterator(directory)) {
    if(!entry.is_regular_file() || entry.path().extension()!=".bin" ||
        entry.path().filename().string().rfind("profile-",0)!=0)continue;
    try {
      std::ifstream stream(entry.path(),std::ios::binary);BinaryReader in(stream);
      const auto header=readHeader(in);
      if(header.map.map_id==identity.map_id && header.map.frame_id==identity.frame_id &&
          (!exact || header.map.content_epoch==identity.content_epoch) &&
          header.preparation==config.preparation())return entry.path();
    } catch(const std::exception&) { /* An incomplete optional cache is rebuilt. */ }
  }
  return {};
}
}

bool restorePrepared(const std::filesystem::path& directory,const std::filesystem::path& source,const MapIdentity& identity,
    const PlannerConfig& config,const std::shared_ptr<octomap::OcTree>& map,
    global_planner::OctoPlanner3D& planner) {
  try {
    const auto path=profilePath(directory,identity,config,true);
    if(path.empty())return false;
    std::ifstream stream(path,std::ios::binary);BinaryReader in(stream);
    const auto header=readHeader(in);
    if(!lingtu::nav::plan::sameMapIdentity(header.map,identity) ||
        header.preparation!=config.preparation() || header.size!=std::filesystem::file_size(source) ||
        header.modified!=std::filesystem::last_write_time(source).time_since_epoch().count())return false;
    const auto has_rust_graph=in.value<std::uint8_t>();
    if (has_rust_graph > 1) throw std::runtime_error("prepared map has invalid Rust graph flag");
    std::optional<global_planner::mls::Artifact> rust_artifact;
    if (has_rust_graph) {
      rust_artifact = global_planner::mls::read(in, false);
      if (rust_artifact->resolution_m != map->getResolution())
        throw std::runtime_error("prepared Rust graph resolution mismatch");
    } else {
      throw std::runtime_error("prepared map has no Rust MLS artifact");
    }
    const auto rust_abi=in.value<std::uint32_t>();
    if(rust_abi!=lt_mls_abi_version())
      throw std::runtime_error("prepared map Rust MLS ABI changed");
    if(!in.complete())throw std::runtime_error("unexpected prepared map tail");
    if (rust_artifact) {
      const double resolution = map->getResolution();
      const int stride = std::max(1, static_cast<int>(std::ceil(
          config.search_resolution_m / resolution)));
      const int height_stride = config.max_slope > 0.0
          ? std::max(stride, static_cast<int>(std::ceil(1.0 / config.max_slope)))
          : stride;
      const int step_cells = std::max(0, static_cast<int>(std::ceil(
          config.max_step_height / resolution)));
      const LtMlsGraphConfig rust_config{
          stride,
          std::max(height_stride, step_cells),
          0,
          std::max(1, config.obstacle_clearance_radius_cells),
          static_cast<float>(std::max(0.0, config.obstacle_clearance_weight)),
          static_cast<float>(std::max(0.0, config.floor_change_penalty)),
          static_cast<float>(std::max(0.0, config.max_slope)),
          step_cells,
          height_stride};
      global_planner::mls::RustGraph rust_graph;
      rust_graph.nodes = std::move(rust_artifact->nodes);
      rust_graph.edges = std::move(rust_artifact->edges);
      auto restored = std::make_shared<global_planner::mls::PreparedArtifactHandle>(
          global_planner::mls::PreparedArtifactHandle::createFromGraph(
              rust_artifact->observed, rust_artifact->inferred, rust_graph,
              rust_config, rust_artifact->revision));
      planner.setPreparedRustGraph(map, std::move(restored));
    }
    if(!planner.artifact() || !planner.artifact()->rust_graph)
      throw std::runtime_error("prepared map Rust MLS artifact was not rebuilt");
    std::fprintf(stderr,"OctoPlanner3D restored prepared map: %s\n",path.string().c_str());
    return true;
  } catch(const std::exception& error) {
    std::fprintf(stderr,"OctoPlanner3D prepared map unavailable: %s\n",error.what());
    return false;
  }
}

void savePrepared(const std::filesystem::path& directory,const std::filesystem::path& source,const MapIdentity& identity,
    const PlannerConfig& config,const global_planner::OctoPlanner3D& planner) {
  std::filesystem::path temporary;
  try {
    std::filesystem::create_directories(directory);
    auto path=profilePath(directory,identity,config,false);
    if(path.empty()) {
      std::size_t index=0;
      do {path=directory/("profile-"+std::to_string(index++)+".bin");}
      while(std::filesystem::exists(path));
    }
    temporary=path;temporary+=".tmp-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    {
      std::ofstream stream(temporary,std::ios::binary|std::ios::trunc);BinaryWriter out(stream);
      out.text(kFormat);out.value(kEndian);out.text(identity.map_id);
      out.value<std::uint64_t>(identity.content_epoch);out.text(identity.frame_id);
      out.value<std::uint64_t>(std::filesystem::file_size(source));
      out.value<std::int64_t>(std::filesystem::last_write_time(source).time_since_epoch().count());
      for(double value:config.preparation())out.value(value);
      planner.writePrepared(out);
      out.value<std::uint32_t>(lt_mls_abi_version());
      stream.flush();
      if(!stream)throw std::runtime_error("cannot flush prepared map");
    }
#if defined(_WIN32)
    if(!MoveFileExW(temporary.wstring().c_str(),path.wstring().c_str(),MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH))
      throw std::runtime_error("cannot publish prepared map");
#else
    std::filesystem::rename(temporary,path);
#endif
    std::fprintf(stderr,"OctoPlanner3D saved prepared map: %s\n",path.string().c_str());
  } catch(const std::exception& error) {
    std::error_code ignored;if(!temporary.empty())std::filesystem::remove(temporary,ignored);
    std::fprintf(stderr,"OctoPlanner3D prepared map not saved: %s\n",error.what());
  }
}

} // namespace octoplanner3d::runtime
